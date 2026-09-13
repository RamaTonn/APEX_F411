#include "estimate.h"

#include <math.h>
#include <stddef.h>

#include "loop.h"
#include "gate_driver.h"
#include "main.h"
#include "motor.h"
#include "sensors.h"

/* The motor being measured, for the duration of one run. */
static motor_t *motor;

/* ------------------------------------------------------------------
 * Resistance measurement settings
 * ------------------------------------------------------------------ */

/* The two currents the slope is taken between, in milliamps.
 *
 * Targeting currents rather than duties keeps the measurement
 * independent of both the winding and the supply: the duty is raised
 * until the current arrives, whatever it takes to get there. */
#define RESISTANCE_LOW_TARGET_MA  1000
#define RESISTANCE_HIGH_TARGET_MA 3000

/* How far the duty is raised each step while hunting, in parts per
 * thousand. Small enough that the current cannot go from safe to
 * dangerous in one step on any winding this board would be used with. */
#define RESISTANCE_DUTY_STEP 1U

/* Highest duty the hunt will reach before giving up.
 *
 * Two phases in series is around 70 milliohms on a motor this board
 * would drive, so the high target arrives somewhere near duty 40 on a
 * 12 volt bus. The ceiling is a backstop for a disconnected phase, where
 * no current flows however high the duty goes; the sustained-current
 * guard stops a working run long before it. */
#define RESISTANCE_MAX_DUTY 150U

/* How long to let the current settle after a duty change, and how long
 * to average it for once settled, both in milliseconds.
 *
 * The winding reaches its steady current in a few electrical time
 * constants -- a millisecond at the outside on a motor this size -- so
 * the settle is generous. The average removes what noise the control
 * loop's synchronous sampling has not already. */
#define RESISTANCE_SETTLE_MS  20U
#define RESISTANCE_AVERAGE_MS 20U

/* ------------------------------------------------------------------
 * Small helpers
 * ------------------------------------------------------------------ */

static int32_t absolute(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* Largest magnitude among the three phase currents.
 *
 * Phase C has no sensor; the three currents sum to zero, so C is
 * whatever makes the sum zero. That is Kirchhoff's current law at the
 * star point and holds whether or not the winding is balanced -- it is
 * the only place in this file a derived current is used, and it is used
 * for a safety limit rather than for an answer, so carrying both
 * sensors' errors costs nothing.
 *
 * The limit is on what the windings and the bridge physically carry, so
 * all three count, whichever phase the measurement happens to read. */
static int32_t largest_phase_current_ma(int32_t current_a_ma,
                                        int32_t current_b_ma)
{
    int32_t current_c_ma = -(current_a_ma + current_b_ma);

    int32_t largest = absolute(current_a_ma);
    if (absolute(current_b_ma) > largest) {
        largest = absolute(current_b_ma);
    }
    if (absolute(current_c_ma) > largest) {
        largest = absolute(current_c_ma);
    }
    return largest;
}

/* One phase's current, by index. */
static int32_t phase_current_ma(uint8_t phase,
                                int32_t current_a_ma,
                                int32_t current_b_ma)
{
    if (phase == GATE_DRIVER_PHASE_A) {
        return current_a_ma;
    }
    if (phase == GATE_DRIVER_PHASE_B) {
        return current_b_ma;
    }
    return -(current_a_ma + current_b_ma);
}

/* Where a run gave up, if it did. Module-level for the same reason
 * `motor` is: one run happens at a time. */
static uint8_t  fault_pair;
static uint16_t fault_duty;
static int32_t  fault_current_ma;

/* Bring every phase to ground, stopping the current without disabling
 * the bridge. */
static void rest_bridge(void)
{
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_set_duty(phase, 0u);
    }
}

/* Enable every phase, starting from ground. */
static void enable_bridge(void)
{
    rest_bridge();
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_enable_phase(phase);
    }
}

/* How long the winding is left shorted through the low-side FETs for
 * its current to decay before the bridge is reconfigured, in
 * milliseconds. The L/R time constant is a few hundred microseconds at
 * the outside, so one millisecond is several of them. */
#define DECAY_MS 1U

/* Enable exactly two phases and leave the third floating.
 *
 * Every phase is dropped to ground first and given a moment there, so
 * the current decays through the low-side FETs -- which are on at zero
 * duty -- before anything is disabled. Disabling a phase that is still
 * carrying current pushes it through the body diodes instead, which
 * works but is not what they are there for; the previous pair's driven
 * phase is live when this is called. */
static void enable_pair(uint8_t high_phase, uint8_t low_phase)
{
    rest_bridge();
    HAL_Delay(DECAY_MS);

    gate_driver_disable_all();

    gate_driver_enable_phase(low_phase);
    gate_driver_enable_phase(high_phase);
}

/* Wait, watching the current, and give up if it stays over the limit.
 *
 * Nothing needs re-applying while waiting: a duty written to the timer
 * stays written. That is the whole advantage of driving the bridge
 * directly -- the earlier vector-based version had to keep re-applying
 * its command so the dead-time correction could track the current, and
 * that feedback path was one of the things that made it unmeasurable.
 *
 * A SUSTAINED excursion is what stops a run, not a single reading: the
 * measurement carries a few counts of noise and a lone switching
 * transient damages nothing, which is the same argument openloop.c
 * makes at OPENLOOP_ABORT_CONSECUTIVE_SAMPLES.
 *
 * @param milliseconds  how long to wait
 * @param duty          the duty being applied, recorded if it faults
 * @return 1 on success, 0 on sustained overcurrent -- the bridge is
 *         already back at ground when this returns 0 */
static uint8_t wait_watching_current(uint32_t milliseconds, uint16_t duty)
{
    uint32_t start      = HAL_GetTick();
    uint32_t over_since = 0u;
    uint8_t  over       = 0u;
    int32_t  current_a;
    int32_t  current_b;

    while ((HAL_GetTick() - start) < milliseconds) {
        sensors_get_currents(&current_a, &current_b);

        int32_t largest = largest_phase_current_ma(current_a, current_b);

        if (largest > ESTIMATE_CURRENT_LIMIT_MA) {
            if (over == 0u) {
                over       = 1u;
                over_since = HAL_GetTick();
            } else if ((HAL_GetTick() - over_since)
                           >= ESTIMATE_OVERCURRENT_SUSTAIN_MS) {
                rest_bridge();
                fault_duty       = duty;
                fault_current_ma = largest;
                return 0u;
            }
        } else {
            over = 0u;
        }
    }
    return 1u;
}

/* Average one phase's current over a window, watching the limit
 * throughout.
 *
 * The bus is averaged over the SAME window, because the two are used
 * together. The supply on this bench sags nearly two volts between the
 * measurement's two operating points, so the bus is doing as much work
 * in the answer as the current is -- and a single instantaneous reading
 * of a supply that is being pulled about is not worth what an average
 * of the same window is.
 *
 * @param phase          which phase's sensor to read
 * @param sign           +1 if current enters that phase's terminal in
 *                       this topology, -1 if it leaves
 * @param milliseconds   how long to average over
 * @param duty           the duty being applied, recorded if it faults
 * @param average_ma_out where the average is written, sign applied
 * @param bus_mv_out     averaged bus over the same window, or NULL
 * @return 1 on success, 0 on sustained overcurrent */
static uint8_t average_phase_current(uint8_t   phase,
                                     int32_t   sign,
                                     uint32_t  milliseconds,
                                     uint16_t  duty,
                                     int32_t  *average_ma_out,
                                     uint32_t *bus_mv_out)
{
    int64_t  bus_total  = 0;
    int64_t  total      = 0;
    uint32_t count      = 0u;
    uint32_t start      = HAL_GetTick();
    uint32_t over_since = 0u;
    uint8_t  over       = 0u;
    int32_t  current_a;
    int32_t  current_b;

    while ((HAL_GetTick() - start) < milliseconds) {
        sensors_get_currents(&current_a, &current_b);

        int32_t largest = largest_phase_current_ma(current_a, current_b);

        if (largest > ESTIMATE_CURRENT_LIMIT_MA) {
            if (over == 0u) {
                over       = 1u;
                over_since = HAL_GetTick();
            } else if ((HAL_GetTick() - over_since)
                           >= ESTIMATE_OVERCURRENT_SUSTAIN_MS) {
                rest_bridge();
                fault_duty       = duty;
                fault_current_ma = largest;
                return 0u;
            }
        } else {
            over = 0u;
        }

        total     += phase_current_ma(phase, current_a, current_b) * sign;
        bus_total += (int64_t)sensors_get_bus_mv();
        count++;
    }

    *average_ma_out = (count > 0u) ? (int32_t)(total / (int32_t)count) : 0;

    if (bus_mv_out != NULL) {
        *bus_mv_out = (count > 0u)
                          ? (uint32_t)(bus_total / (int64_t)count) : 0u;
    }
    return 1u;
}

/* ------------------------------------------------------------------
 * Resistance
 * ------------------------------------------------------------------ */

/* One line-to-line measurement: which phase is driven, which is held at
 * ground, and which sensor reads the current that results. The third
 * phase is simply the one not named, and floats -- enable_pair() enables
 * these two and nothing else.
 *
 * The sense phase is always A or B, because those are the two with a
 * sensor. For the C-A pair the current leaves phase A's terminal rather
 * than entering it, so that sensor reads it negative and the sign puts
 * it back the right way up. The alternative would be a phase C current
 * derived from the other two, which carries both their errors -- fine
 * for the safety limit, where it is already used, but not for a number
 * the answer depends on. */
typedef struct {
    uint8_t high_phase;
    uint8_t low_phase;
    uint8_t sense_phase;
    int32_t sense_sign;
} resistance_pair_t;

static const resistance_pair_t resistance_pairs[3] = {
    /* AB, C floating */ { GATE_DRIVER_PHASE_A, GATE_DRIVER_PHASE_B,
                           GATE_DRIVER_PHASE_A,  1 },
    /* BC, A floating */ { GATE_DRIVER_PHASE_B, GATE_DRIVER_PHASE_C,
                           GATE_DRIVER_PHASE_B,  1 },
    /* CA, B floating */ { GATE_DRIVER_PHASE_C, GATE_DRIVER_PHASE_A,
                           GATE_DRIVER_PHASE_A, -1 },
};

/* Measure one line-to-line pair.
 *
 * Ramps the driven phase's duty until the current reaches the low
 * target, notes that point, carries on to the high target, and takes the
 * slope between the two. Both points were taken at the same current
 * polarity, so the dead time removed the same slice of voltage from
 * each and it cancels in the difference.
 *
 * THE BUS IS READ WITH NO CURRENT FLOWING, and that is deliberate.
 *
 * The reported bus falls as the winding current rises -- by nearly two
 * volts between this measurement's two operating points on the bench
 * this was developed against. That looked like supply sag, and an
 * earlier version subtracted it as such. It cannot be.
 *
 * The bridge chops phase A at about four percent to pass three amps, so
 * the supply only delivers about a tenth of an amp on average, and the
 * current it delivers changes by less than a tenth of an amp between the
 * two points. Two volts across that is twenty ohms of source impedance,
 * which no supply and no wiring has. Referred back through the divider
 * it is eighty millivolts at the ADC pin, which at three amps is
 * twenty-seven milliohms of shared sense-and-power ground -- entirely
 * ordinary on a small board.
 *
 * So it is the READING that moves with current, not the rail. Treating
 * it as real subtracted a measurement error from the applied voltage and
 * pushed the answer low by nearly half. The quiet reading is the honest
 * reference, and how far the loaded reading departs from it is reported
 * instead of acted on.
 *
 * @param pair          which phases to drive, ground, float and read
 * @param mohm_out      the two windings in series, in milliohms
 * @param bus_mv_out    the bus with no current flowing, millivolts
 * @param sag_mv_out    how far the reading had fallen by the upper point
 * @return an ESTIMATE_ result code */
static uint8_t measure_pair(const resistance_pair_t *pair,
                            uint32_t *mohm_out,
                            uint32_t *bus_mv_out,
                            int32_t  *sag_mv_out)
{
    uint16_t low_duty     = 0u;
    uint16_t high_duty    = 0u;
    int32_t  low_current  = 0;
    int32_t  high_current = 0;
    uint32_t high_bus_mv  = 0u;
    uint8_t  reached      = 0u;

    enable_pair(pair->high_phase, pair->low_phase);

    /* The reference, taken before any current flows. */
    uint32_t quiet_bus_mv = sensors_get_bus_mv();

    for (uint16_t duty = RESISTANCE_DUTY_STEP;
         duty <= RESISTANCE_MAX_DUTY;
         duty = (uint16_t)(duty + RESISTANCE_DUTY_STEP)) {

        gate_driver_set_duty(pair->low_phase, 0u);
        gate_driver_set_duty(pair->high_phase, duty);

        if (wait_watching_current(RESISTANCE_SETTLE_MS, duty) == 0u) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        int32_t  measured;
        uint32_t measured_bus_mv;

        if (average_phase_current(pair->sense_phase, pair->sense_sign,
                                  RESISTANCE_AVERAGE_MS, duty,
                                  &measured, &measured_bus_mv) == 0u) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        /* Recorded every step, so a run that reaches the ceiling can
         * report how far the current actually got rather than leaving
         * it to be guessed at. */
        fault_duty       = duty;
        fault_current_ma = measured;

        if ((low_duty == 0u) && (measured >= RESISTANCE_LOW_TARGET_MA)) {
            low_duty    = duty;
            low_current = measured;

            /* Nothing to record here: the voltage reference is the
             * quiet reading taken before the ramp began. */
        }

        if ((low_duty != 0u) && (measured >= RESISTANCE_HIGH_TARGET_MA)) {
            high_duty    = duty;
            high_current = measured;
            high_bus_mv  = measured_bus_mv;
            reached      = 1u;
            break;
        }
    }

    rest_bridge();

    if (reached == 0u) {
        return ESTIMATE_ERR_TOO_SMALL;
    }

    int32_t current_change = high_current - low_current;

    if (current_change < ESTIMATE_MINIMUM_CURRENT_CHANGE_MA) {
        return ESTIMATE_ERR_TOO_SMALL;
    }

    /* One bus, the quiet one, against the duty difference -- see the
     * note above on why the loaded reading is not used.
     *
     * Millivolts over milliamps is ohms directly, so the thousand turns
     * it into milliohms. */
    int32_t voltage_change_mv =
        (int32_t)((quiet_bus_mv * (uint32_t)(high_duty - low_duty))
                  / GATE_DRIVER_DUTY_SCALE);

    *mohm_out   = (uint32_t)((voltage_change_mv * 1000) / current_change);
    *bus_mv_out = quiet_bus_mv;
    *sag_mv_out = (int32_t)quiet_bus_mv - (int32_t)high_bus_mv;

    return ESTIMATE_OK;
}

/* Solve one phase from the three line-to-line sums.
 *
 * R_ab = R_a + R_b and R_ca = R_c + R_a, so their sum is
 * R_a + R_b + R_c + R_a; subtracting R_bc = R_b + R_c leaves 2 R_a.
 *
 * A phase reads as zero rather than going negative, which it can do on
 * a badly unbalanced winding where one phase is a small difference
 * between two larger measurements. */
static uint32_t solve_phase(uint32_t with_previous,
                            uint32_t with_next,
                            uint32_t opposite)
{
    uint32_t sum = with_previous + with_next;

    if (sum <= opposite) {
        return 0u;
    }
    return (sum - opposite) / 2u;
}

uint8_t estimate_resistance(motor_t *m,
                            uint8_t reverse_order,
                            estimate_resistance_result_t *result_out)
{
    motor = m;

    if (result_out == NULL) {
        return ESTIMATE_ERR_ARGUMENT;
    }
    if (loop_is_running() == 0u) {
        return ESTIMATE_ERR_NOT_READY;
    }

    fault_pair       = ESTIMATE_PAIR_NONE;
    fault_duty       = 0u;
    fault_current_ma = 0;

    result_out->ab_mohm           = 0u;
    result_out->bc_mohm           = 0u;
    result_out->ca_mohm           = 0u;
    result_out->a_mohm            = 0u;
    result_out->b_mohm            = 0u;
    result_out->c_mohm            = 0u;
    result_out->phase_mohm        = 0u;
    result_out->imbalance_percent = 0u;
    result_out->bus_mv            = 0u;
    result_out->sag_mv            = 0;
    result_out->fault_pair        = ESTIMATE_PAIR_NONE;
    result_out->fault_duty        = 0u;
    result_out->fault_current_ma  = 0;

    uint32_t line_mohm[3] = { 0u, 0u, 0u };
    uint32_t bus_total    = 0u;
    int32_t  worst_sag    = 0;
    uint8_t  outcome      = ESTIMATE_OK;

    for (uint8_t position = 0u; position < 3u; position++) {

        /* Which pair this position measures. Reversing it is a test, not
         * a tuning knob: a winding's resistance does not depend on the
         * order it was measured in, so if the three answers follow the
         * ORDER rather than the pairs, what is being measured is drift
         * -- the motor warming under the test, or a supply sagging as it
         * runs -- and not the winding at all. */
        uint8_t i = (reverse_order != 0u) ? (uint8_t)(2u - position)
                                          : position;

        uint32_t bus_mv = 0u;
        int32_t  sag_mv = 0;

        fault_pair = i;

        outcome = measure_pair(&resistance_pairs[i], &line_mohm[i],
                               &bus_mv, &sag_mv);
        if (outcome != ESTIMATE_OK) {
            break;
        }

        bus_total += bus_mv;

        if (sag_mv > worst_sag) {
            worst_sag = sag_mv;
        }
    }

    rest_bridge();
    HAL_Delay(DECAY_MS);
    gate_driver_disable_all();

    if (outcome != ESTIMATE_OK) {
        result_out->fault_pair       = fault_pair;
        result_out->fault_duty       = fault_duty;
        result_out->fault_current_ma = fault_current_ma;
        return outcome;
    }

    result_out->ab_mohm = line_mohm[ESTIMATE_PAIR_AB];
    result_out->bc_mohm = line_mohm[ESTIMATE_PAIR_BC];
    result_out->ca_mohm = line_mohm[ESTIMATE_PAIR_CA];
    result_out->bus_mv  = bus_total / 3u;
    result_out->sag_mv  = worst_sag;

    result_out->a_mohm = solve_phase(result_out->ab_mohm,
                                     result_out->ca_mohm,
                                     result_out->bc_mohm);
    result_out->b_mohm = solve_phase(result_out->bc_mohm,
                                     result_out->ab_mohm,
                                     result_out->ca_mohm);
    result_out->c_mohm = solve_phase(result_out->ca_mohm,
                                     result_out->bc_mohm,
                                     result_out->ab_mohm);

    uint32_t mean = (result_out->a_mohm + result_out->b_mohm
                     + result_out->c_mohm) / 3u;

    if (mean == 0u) {
        return ESTIMATE_ERR_TOO_SMALL;
    }

    uint32_t highest = result_out->a_mohm;
    uint32_t lowest  = result_out->a_mohm;

    if (result_out->b_mohm > highest) { highest = result_out->b_mohm; }
    if (result_out->c_mohm > highest) { highest = result_out->c_mohm; }
    if (result_out->b_mohm < lowest)  { lowest  = result_out->b_mohm; }
    if (result_out->c_mohm < lowest)  { lowest  = result_out->c_mohm; }

    result_out->imbalance_percent = ((highest - lowest) * 100u) / mean;
    result_out->phase_mohm        = mean;

    /* The control loop wants one resistance, so it gets the mean. That
     * is the right single number for a nearly balanced winding and the
     * best available one for an unbalanced winding; the per-phase values
     * are reported so a winding bad enough to need more than one number
     * is visible rather than averaged away. */
    motor_set_resistance(motor, (float)mean * 0.001f);

    return ESTIMATE_OK;
}
/* ------------------------------------------------------------------
 * Inductance
 *
 * Unlike the resistance measurement above, this drives all three phases
 * as a vector. It has to: the two axes must be excited while the rotor
 * is HELD, and only a three-phase drive can hold it on one axis while
 * exciting another.
 *
 * An earlier version drove two phases against each other for Lq, with
 * the field ninety electrical degrees from where the rotor was sitting
 * and nothing holding it there. The rotor simply turned to the field --
 * eighty degrees of it inside four milliseconds, on a motor this board
 * drives -- so by the end the axis being measured was no longer the q
 * axis, and the back EMF on the way there swamped an excitation worth
 * tens of millivolts. That is not a tuning problem; a torque-producing
 * field applied to a free rotor for milliseconds always ends that way.
 *
 * Holding a d-axis current and superimposing an ALTERNATING q-axis
 * excitation fixes it at the root. The d current is a restoring spring
 * -- the rotor sits where it points -- and the alternating q excitation
 * averages to no torque at all, at a frequency far above anything the
 * rotor can follow. The rotor stays put, and the q axis stays the q
 * axis.
 * ------------------------------------------------------------------ */

/* The bridge's resting point, in parts per thousand.
 *
 * With all three phases at half duty the terminals sit at the same
 * potential and no current flows, so this is the zero of every
 * perturbation below. It is also what lets the excitation be applied in
 * both directions: a bridge cannot drive a phase below the negative
 * rail, so a signed demand has to sit on a bias. */
#define RESTING_DUTY (GATE_DRIVER_DUTY_SCALE / 2u)

/* The d-axis current the rotor is held with, in milliamps, and how long
 * to let it settle there.
 *
 * A CURRENT rather than a duty. A fixed duty large enough to conduct on
 * a 12 volt bus draws twice as much on a 24 volt one, and a duty small
 * enough to be safe there is lost in the dead time here.
 *
 * Large enough to do two jobs. It has to hold the rotor against the
 * excitation's torque ripple and against cogging. And it has to keep
 * every phase current away from zero: the dead time reverses sign with
 * the current it is flowing through, so a phase that crosses zero puts
 * a step into the one term the two halves are subtracted to cancel.
 * At this hold the two return phases sit at an amp and a half apiece,
 * and a phase only reaches zero once the q current passes the hold over
 * root three -- around 1.7 amps, which is more than twice what the
 * measurement uses.
 *
 * It costs almost nothing from the supply. Every phase sits near half
 * duty, so the bridge is circulating this current between the windings
 * rather than drawing it from the rail: about 180 milliamps comes in
 * from the bus at a 3 amp hold. That matters here, because the supply
 * on this bench sags nearly two volts at the currents the RESISTANCE
 * measurement draws, and the bus reading is what sets the scale of the
 * answer. */
#define HOLD_TARGET_MA 3000
#define HOLD_SETTLE_MS 400U

/* Highest d-axis perturbation the holding ramp will reach before giving
 * up, and how long each of its steps settles and averages for.
 *
 * The ramp doubles as the alignment: the field points along phase A's
 * axis throughout and grows steadily, so the rotor is drawn into line as
 * the current comes up rather than being jerked there. */
#define HOLD_MAX_UNIT    150
#define HOLD_SETTLE_MS_STEP 4U
#define HOLD_AVERAGE_MS     4U

/* How long one half of the excitation lasts, in control periods, and
 * where its three samples sit.
 *
 * THREE SAMPLES, NOT TWO
 *
 *   Two would give a slope, and the difference between the rising and
 *   falling slopes would give an inductance -- but only an approximate
 *   one, because the resistive drop does not quite cancel between the
 *   halves: the current sits above its mean in the raised half and below
 *   it in the lowered half, so R times the current leaves a residue that
 *   reads as extra inductance.
 *
 *   A third sample removes it. The ratio between the two inner rises is
 *   exp(-gap / tau), which is the winding's own time constant measured
 *   from the shape of its response rather than assumed -- and with tau
 *   in hand the relation between excitation and slope is exact rather
 *   than first-order. It costs one extra read, and hands back the time
 *   constant as a result in its own right.
 *
 * WHY THE HALF LENGTH IS CHOSEN AT RUN TIME
 *
 *   Everything depends on the ratio between the half and the winding's
 *   own time constant, and a fixed half cannot suit both a fast winding
 *   and a slow one.
 *
 *   Too long and the current settles before the half ends. The second
 *   rise is then a small difference between large numbers, tau is fitted
 *   almost entirely to noise -- and in that regime the inductance is
 *   directly proportional to tau, so all of that noise lands on the
 *   answer. A fixed twelve periods did exactly this on the motor this
 *   was developed against: the raw response repeated to three percent
 *   while the inductance scattered by sixteen, and the inductance
 *   tracked the fitted tau with a correlation of 0.98.
 *
 *   Too short and the current barely curves at all. Tau is then poorly
 *   determined too -- but harmlessly, because a straight line is very
 *   nearly the right answer in that regime and the correction tau makes
 *   is small.
 *
 *   Around two time constants is where both are comfortable, and it is
 *   a broad optimum. So the measurement runs twice: once to find out
 *   roughly what the winding's time constant is, then again at a half
 *   length picked to suit it. */
#define SLOPE_HALF_TARGET_TAUS 2U

/* Bounds on that choice. Four periods is the shortest half that leaves
 * three distinct samples; twenty-four is long enough for the slowest
 * winding this board would drive. */
#define SLOPE_HALF_MINIMUM 4U
#define SLOPE_HALF_MAXIMUM 24U

/* Where the first pass starts, before anything is known about the
 * winding. Suits a time constant near a hundred microseconds and gives
 * a usable answer from about forty to four hundred, which is the whole
 * range worth probing. */
#define SLOPE_HALF_PROBE 8U

#define SLOPE_MICROSECONDS_PER_PERIOD (1000000.0f / (float)LOOP_RATE_HZ)

/* One excitation's timing: how long a half lasts, and which periods
 * within it the three samples are taken at.
 *
 * No sample is taken across a duty change. The current sensor is read
 * once per switching period at a fixed point in the cycle, and where
 * that point falls relative to the switching ripple moves when the duty
 * moves, so readings either side of a change differ by an amount that
 * has nothing to do with the winding. Holding the duty constant across
 * all three samples makes that offset a constant, which subtracts out.
 * That is what costs the first and last period of every half. */
typedef struct {
    uint32_t half_periods;
    uint32_t first_period;
    uint32_t middle_period;
    uint32_t last_period;
} excitation_shape_t;

/* Space three samples evenly inside a half, clear of both its edges. */
static excitation_shape_t shape_for_half(uint32_t half_periods)
{
    excitation_shape_t shape;

    if (half_periods < SLOPE_HALF_MINIMUM) {
        half_periods = SLOPE_HALF_MINIMUM;
    }
    if (half_periods > SLOPE_HALF_MAXIMUM) {
        half_periods = SLOPE_HALF_MAXIMUM;
    }

    shape.half_periods  = half_periods;
    shape.first_period  = 1u;
    shape.middle_period = 1u + ((half_periods - 2u) / 2u);
    shape.last_period   = half_periods - 1u;

    return shape;
}

/* The half length that puts SLOPE_HALF_TARGET_TAUS time constants inside
 * one half, rounded to an even number of periods so the three samples
 * land evenly.
 *
 * A tau of zero means the first pass saw no curvature at all, which
 * says the winding is slower than anything that half could resolve --
 * so the longest half available is the right next guess. */
static uint32_t half_for_tau(uint32_t tau_us)
{
    if (tau_us == 0u) {
        return SLOPE_HALF_MAXIMUM;
    }

    uint32_t periods = (tau_us * SLOPE_HALF_TARGET_TAUS * LOOP_RATE_HZ)
                       / 1000000u;

    periods = ((periods + 1u) / 2u) * 2u;

    if (periods < SLOPE_HALF_MINIMUM) {
        periods = SLOPE_HALF_MINIMUM;
    }
    if (periods > SLOPE_HALF_MAXIMUM) {
        periods = SLOPE_HALF_MAXIMUM;
    }
    return periods;
}

/* Cycles run before anything is recorded, and cycles recorded.
 *
 * The warm-up matters: the cancellation is exact only once the current
 * is oscillating steadily about its mean, which takes a couple of cycles
 * to establish from the standing hold. */
#define SLOPE_WARMUP_CYCLES 2U
#define SLOPE_CYCLES        32U

/* Cycles for the first pass, which only has to find the winding's time
 * constant well enough to choose a half length for the second. Being
 * roughly right is all that is asked of it, and the optimum it is aiming
 * for is broad, so it runs a quarter as long. */
#define SLOPE_PROBE_CYCLES  8U

/* How far apart the rising and falling responses must be for an
 * excitation size to count, in milliamps.
 *
 * This is really the precision setting. Each response is a difference
 * between single sensor readings, so it carries around 57 milliamps of
 * quantisation; averaging eight cycles brings that under half a percent
 * of this figure. The excitation is raised until it is met, so asking
 * for more here buys accuracy and pays in current.
 *
 * Twenty-two hundred is where that trade settles. Below it the q axis
 * gets noticeably noisier; above it the excitation jumps to the next
 * size up and starts eating into the margin the holding current leaves
 * before a phase crosses zero. */
#define SLOPE_MINIMUM_DIFFERENCE_MA 2200

/* Above this, the two inner rises are too alike for the ratio between
 * them to say anything about the time constant.
 *
 * That happens when the winding is far slower than the half it is being
 * excited over, which is the case where the correction the ratio exists
 * to make is negligible anyway -- so the straight-line relation is used
 * instead of a time constant fitted to noise. */
#define SLOPE_FLAT_RATIO 0.98f

/* Excitation sizes, smallest first.
 *
 * Raised only until the response is large enough to measure, so a low
 * inductance winding is never driven harder than it needs to be, and a
 * high voltage bus is not driven as hard as a low voltage one -- the
 * response a given size produces scales with the bus, so the entry that
 * works on a 48 volt supply is a quarter of the one needed on a 12 volt
 * supply. */
static const int32_t excitation_steps[] = { 1, 2, 4, 8, 16, 32, 64 };
#define EXCITATION_STEP_COUNT \
    (sizeof excitation_steps / sizeof excitation_steps[0])

/* One axis of the rotor frame, expressed as what it costs the bridge.
 *
 * The rotor is held in line with phase A, so the rotor frame and the
 * stator frame coincide and the transforms collapse to constants -- no
 * angle, no sine, no Park. A d-axis demand is just phase A against the
 * other two, and a q-axis demand is just phase B against phase C.
 *
 * `voltage_scale` is how much axis voltage one unit of the perturbation
 * produces, as a fraction of the bus in parts per thousand. It is the
 * amplitude about the mean -- what ONE half of the excitation applies,
 * which is what the inductance relation is written in terms of -- not
 * the step between the two halves, which is twice it.
 *
 * It falls out of the forward transform applied to the duties
 * themselves, which is what makes it exact: the duties are integers, so
 * working back from what was actually written rather than from the
 * voltage that was asked for leaves no rounding anywhere. */
typedef struct {
    int32_t duty_a;
    int32_t duty_b;
    int32_t duty_c;
    float   voltage_scale;
} axis_drive_t;

/* d: phase A against B and C together. Its axis voltage is
 * (2 va - vb - vc) / 3, so one unit of (2, -1, -1) is worth 2/1000 of
 * the bus. */
static const axis_drive_t axis_d = { 2, -1, -1, 2.0f };

/* q: phase B against phase C, which is ninety electrical degrees from
 * phase A's axis. Its axis voltage is (vb - vc) / sqrt(3), so one unit
 * of (0, 1, -1) is worth 2/(1000 sqrt(3)) of the bus. */
static const axis_drive_t axis_q = { 0, 1, -1, 1.1547005f };

/* Clamp a signed perturbation about the resting point into a duty the
 * bridge will accept. */
static uint16_t duty_from_perturbation(int32_t perturbation)
{
    int32_t duty = (int32_t)RESTING_DUTY + perturbation;

    if (duty < 0) {
        duty = 0;
    }
    if (duty > (int32_t)GATE_DRIVER_DUTY_MAXIMUM) {
        duty = (int32_t)GATE_DRIVER_DUTY_MAXIMUM;
    }
    return (uint16_t)duty;
}

/* Drive a holding vector along phase A's axis with an excitation
 * superimposed on one axis.
 *
 * @param hold        d-axis holding size, in perturbation units
 * @param axis        which axis the excitation acts on
 * @param excitation  signed excitation size, in perturbation units */
static void apply_vector(int32_t hold, const axis_drive_t *axis,
                         int32_t excitation)
{
    gate_driver_set_duty(GATE_DRIVER_PHASE_A,
        duty_from_perturbation((2 * hold) + (axis->duty_a * excitation)));
    gate_driver_set_duty(GATE_DRIVER_PHASE_B,
        duty_from_perturbation((-hold) + (axis->duty_b * excitation)));
    gate_driver_set_duty(GATE_DRIVER_PHASE_C,
        duty_from_perturbation((-hold) + (axis->duty_c * excitation)));
}

/* The current along one axis, in milliamps.
 *
 * With the rotor held in line with phase A the d-axis current is simply
 * phase A's, and the q-axis current is the other two differenced --
 * (ia + 2 ib) / sqrt(3), which is the forward transform with the angle
 * set to zero. Phase C has no sensor and is not needed for either. */
static int32_t axis_current_ma(const axis_drive_t *axis,
                               int32_t current_a_ma,
                               int32_t current_b_ma)
{
    if (axis == &axis_q) {
        return (int32_t)(((float)current_a_ma + (2.0f * (float)current_b_ma))
                         * 0.57735027f);
    }
    return current_a_ma;
}

/* Spin until the control loop has run the given number of periods.
 *
 * Against the loop's iteration counter rather than the millisecond tick,
 * because everything here is timed in tens of microseconds and the tick
 * has no resolution at that scale. */
static void wait_periods(uint32_t periods)
{
    uint32_t start = loop_get_iteration_count();

    while ((loop_get_iteration_count() - start) < periods) {
        /* The loop's ISR is the only thing that advances this. */
    }
}

/* Raise the d-axis holding current until it arrives, aligning the rotor
 * as it climbs.
 *
 * @param hold_out  the perturbation size that produced the target
 * @return an ESTIMATE_ result code */
static uint8_t ramp_to_hold_current(int32_t *hold_out)
{
    for (int32_t hold = 1; hold <= HOLD_MAX_UNIT; hold++) {

        apply_vector(hold, &axis_d, 0);

        if (wait_watching_current(HOLD_SETTLE_MS_STEP,
                                  (uint16_t)hold) == 0u) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        int32_t measured;
        if (average_phase_current(GATE_DRIVER_PHASE_A, 1,
                                  HOLD_AVERAGE_MS, (uint16_t)hold,
                                  &measured, NULL) == 0u) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        fault_duty       = (uint16_t)hold;
        fault_current_ma = measured;

        if (measured >= HOLD_TARGET_MA) {
            *hold_out = hold;
            return ESTIMATE_OK;
        }
    }

    /* The ceiling was reached without the current arriving, which means
     * a phase is not connected -- there is no duty at which a broken
     * winding passes current. */
    return ESTIMATE_ERR_TOO_SMALL;
}

/* Drive one half of the excitation and read the two inner rises.
 *
 * @param hold        d-axis holding size
 * @param axis        which axis the excitation acts on
 * @param shape       how long the half lasts and where its samples sit
 * @param excitation  signed excitation size for this half
 * @param first_out   rise from the first sample to the second
 * @param second_out  rise from the second sample to the third
 * @return 1 on success, 0 if the current exceeded the limit */
static uint8_t half_cycle(int32_t                   hold,
                          const axis_drive_t       *axis,
                          const excitation_shape_t *shape,
                          int32_t                   excitation,
                          int32_t                  *first_out,
                          int32_t                  *second_out)
{
    int32_t current_a;
    int32_t current_b;

    apply_vector(hold, axis, excitation);

    wait_periods(shape->first_period);
    sensors_get_currents(&current_a, &current_b);
    int32_t first = axis_current_ma(axis, current_a, current_b);

    wait_periods(shape->middle_period - shape->first_period);
    sensors_get_currents(&current_a, &current_b);
    int32_t middle = axis_current_ma(axis, current_a, current_b);

    wait_periods(shape->last_period - shape->middle_period);
    sensors_get_currents(&current_a, &current_b);
    int32_t last    = axis_current_ma(axis, current_a, current_b);
    int32_t largest = largest_phase_current_ma(current_a, current_b);

    wait_periods(shape->half_periods - shape->last_period);

    *first_out  = middle - first;
    *second_out = last - middle;

    if (largest > ESTIMATE_CURRENT_LIMIT_MA) {
        fault_current_ma = largest;
        return 0u;
    }
    return 1u;
}

/* Turn a pair of averaged rises into an inductance.
 *
 * WHAT THE TWO NUMBERS ARE
 *
 *   `first` and `second` are the current's rise over the two halves of
 *   the sampling window, each already differenced between the raised and
 *   lowered excitation. Differencing kills everything that is common to
 *   the two: the resistive drop at the mean current, the dead time, any
 *   back EMF, and any fixed offset in the sensor.
 *
 * THE TIME CONSTANT
 *
 *   What survives differencing is the winding's own response, which is
 *   exponential. So the ratio of the second rise to the first is
 *   exp(-gap / tau), and
 *
 *       tau = -gap / ln(second / first)
 *
 *   needs no voltage, no resistance and no clock beyond the control
 *   loop's own.
 *
 * THE INDUCTANCE
 *
 *   With tau known, the steady-state response of an R-L winding to a
 *   square wave of amplitude V about its mean gives, between samples at
 *   a and b inside a half of length h,
 *
 *       difference = (V tau / L) (4 / (1 + exp(-h/tau)))
 *                    (exp(-a/tau) - exp(-b/tau))
 *
 *   which rearranges for L. The awkward-looking factor is what a step
 *   response does NOT have: the current here starts each half from where
 *   the previous one left it, not from rest, and (1 + exp(-h/tau)) is
 *   the price of that. As the winding gets slow relative to the half,
 *   the whole expression collapses to L = V (b - a) / difference, which
 *   is the straight-line answer.
 *
 * @param shape            how long the half lasts and where its samples
 *                         sit, which is what the relation is written in
 *                         terms of
 * @param step_voltage_mv  the axis voltage one half applies, about the
 *                         mean
 * @param first_ma         differenced rise over the first gap
 * @param second_ma        differenced rise over the second gap
 * @param tau_us_out       the winding's time constant, 0 if not fitted
 * @return the inductance in nanohenries, or 0 if it could not be found */
static uint32_t inductance_from_rises(const excitation_shape_t *shape,
                                      float     step_voltage_mv,
                                      int32_t   first_ma,
                                      int32_t   second_ma,
                                      uint32_t *tau_us_out)
{
    const float period_us = SLOPE_MICROSECONDS_PER_PERIOD;
    const float gap_us    = (float)(shape->middle_period
                                    - shape->first_period) * period_us;
    const float a_us      = (float)shape->first_period * period_us;
    const float b_us      = (float)shape->last_period * period_us;
    const float half_us   = (float)shape->half_periods * period_us;

    float total = (float)first_ma + (float)second_ma;

    if ((total <= 0.0f) || (first_ma <= 0)) {
        *tau_us_out = 0u;
        return 0u;
    }

    float ratio = (float)second_ma / (float)first_ma;
    float inductance_uh;

    if ((ratio >= SLOPE_FLAT_RATIO) || (ratio <= 0.0f)) {
        /* No usable curvature: the winding is slow enough that the
         * straight-line relation is the right answer. */
        *tau_us_out   = 0u;
        inductance_uh = (2.0f * step_voltage_mv * (b_us - a_us)) / total;
    } else {
        float tau_us = -gap_us / logf(ratio);

        *tau_us_out = (uint32_t)tau_us;

        inductance_uh = (step_voltage_mv * tau_us * 4.0f
                         * (expf(-a_us / tau_us) - expf(-b_us / tau_us)))
                        / (total * (1.0f + expf(-half_us / tau_us)));
    }

    if (inductance_uh <= 0.0f) {
        return 0u;
    }
    return (uint32_t)((inductance_uh * 1000.0f) + 0.5f);
}

/* Measure the inductance along one axis, raising the excitation until
 * the response is large enough to trust.
 *
 * @param hold            d-axis holding size, kept throughout
 * @param axis            which axis to excite
 * @param shape           how long each half lasts and where its samples
 *                        sit
 * @param cycles          how many cycles to average
 * @param bus_mv          the bus the excitation is a fraction of
 * @param inductance_out  result in nanohenries
 * @param difference_out  the differenced response that produced it
 * @param step_out        the excitation size that was needed
 * @param tau_us_out      the winding's time constant on this axis
 * @return an ESTIMATE_ result code */
static uint8_t measure_axis(int32_t                   hold,
                            const axis_drive_t       *axis,
                            const excitation_shape_t *shape,
                            uint32_t                  cycles,
                            uint32_t                  bus_mv,
                            uint32_t                 *inductance_out,
                            int32_t                  *difference_out,
                            uint16_t                 *step_out,
                            uint32_t                 *tau_us_out)
{
    for (uint32_t i = 0u; i < EXCITATION_STEP_COUNT; i++) {

        int32_t step = excitation_steps[i];
        int32_t raised_first;
        int32_t raised_second;
        int32_t lowered_first;
        int32_t lowered_second;

        for (uint32_t warm = 0u; warm < SLOPE_WARMUP_CYCLES; warm++) {
            if ((half_cycle(hold, axis, shape,  step,
                            &raised_first, &raised_second) == 0u)
                || (half_cycle(hold, axis, shape, -step,
                               &lowered_first, &lowered_second) == 0u)) {
                return ESTIMATE_ERR_OVERCURRENT;
            }
        }

        int64_t first_total  = 0;
        int64_t second_total = 0;

        for (uint32_t cycle = 0u; cycle < cycles; cycle++) {
            if ((half_cycle(hold, axis, shape,  step,
                            &raised_first, &raised_second) == 0u)
                || (half_cycle(hold, axis, shape, -step,
                               &lowered_first, &lowered_second) == 0u)) {
                return ESTIMATE_ERR_OVERCURRENT;
            }
            first_total  += (int64_t)raised_first  - (int64_t)lowered_first;
            second_total += (int64_t)raised_second - (int64_t)lowered_second;
        }

        /* Back to the plain hold before anything else happens. */
        apply_vector(hold, axis, 0);

        int32_t first  = (int32_t)(first_total  / (int64_t)cycles);
        int32_t second = (int32_t)(second_total / (int64_t)cycles);

        *difference_out = first + second;
        *step_out       = (uint16_t)step;

        if ((first + second) < SLOPE_MINIMUM_DIFFERENCE_MA) {
            continue;                     /* too small to trust; drive harder */
        }

        /* The axis voltage one half of the excitation applies, worked
         * back from the duties actually written rather than from what
         * was asked for, so the integer rounding in them is not an
         * error here. */
        float step_voltage_mv =
            ((float)bus_mv * (float)step * axis->voltage_scale) / 1000.0f;

        uint32_t nanohenries = inductance_from_rises(shape, step_voltage_mv,
                                                     first, second,
                                                     tau_us_out);
        if (nanohenries == 0u) {
            continue;
        }

        *inductance_out = nanohenries;
        return ESTIMATE_OK;
    }

    return ESTIMATE_ERR_TOO_SMALL;
}

uint8_t estimate_inductance(motor_t *m,
                            estimate_inductance_result_t *result_out)
{
    motor = m;

    if (result_out == NULL) {
        return ESTIMATE_ERR_ARGUMENT;
    }
    if (loop_is_running() == 0u) {
        return ESTIMATE_ERR_NOT_READY;
    }

    fault_pair       = ESTIMATE_PAIR_NONE;
    fault_duty       = 0u;
    fault_current_ma = 0;

    result_out->inductance_d_nh  = 0u;
    result_out->inductance_q_nh  = 0u;
    result_out->saliency_percent = 0u;
    result_out->d_difference_ma  = 0;
    result_out->q_difference_ma  = 0;
    result_out->d_duty_used      = 0u;
    result_out->q_duty_used      = 0u;
    result_out->d_tau_us         = 0u;
    result_out->q_tau_us         = 0u;
    result_out->d_mohm           = 0u;
    result_out->q_mohm           = 0u;
    result_out->half_periods     = 0u;
    result_out->hold_duty        = 0u;
    result_out->line_to_line_d_nh = 0u;
    result_out->line_to_line_q_nh = 0u;
    result_out->bus_quiet_mv      = 0u;
    result_out->bus_loaded_mv     = 0u;

    enable_bridge();

    /* The bus with nothing flowing. This is the scale of the whole
     * answer -- the inductance is proportional to it -- and the reading
     * moves with winding current on this board, for the reasons
     * measure_pair() sets out. The holding current is three amps, so a
     * reading taken during the hold carries the full error. */
    uint32_t bus_mv = sensors_get_bus_mv();

    /* Draw the rotor into line with phase A's axis and hold it there.
     * The field points where the rotor is asked to go, so once it
     * arrives there is no torque left and it stays. */
    int32_t hold = 0;

    uint8_t held = ramp_to_hold_current(&hold);
    if (held != ESTIMATE_OK) {
        rest_bridge();
        HAL_Delay(DECAY_MS);
        gate_driver_disable_all();
        return held;
    }

    result_out->hold_duty = (uint16_t)hold;

    if (wait_watching_current(HOLD_SETTLE_MS, (uint16_t)hold) == 0u) {
        HAL_Delay(DECAY_MS);
        gate_driver_disable_all();
        return ESTIMATE_ERR_OVERCURRENT;
    }

    /* What the bus reads once the holding current is established. Not
     * used for anything: reported so the departure from the quiet
     * reading is visible, since that departure scales the answer
     * directly if it is ever believed. */
    result_out->bus_quiet_mv  = bus_mv;
    result_out->bus_loaded_mv = sensors_get_bus_mv();

    /* First pass: a short run at a middling half length, whose only job
     * is to find out roughly how fast this winding is. The d axis is
     * used for it because it produces no torque at all -- the excitation
     * points the way the rotor is already aligned.
     *
     * Its inductance is thrown away. The half length it suggests is the
     * whole point of it. */
    excitation_shape_t probe_shape = shape_for_half(SLOPE_HALF_PROBE);

    uint32_t probe_inductance = 0u;
    int32_t  probe_difference = 0;
    uint16_t probe_step       = 0u;
    uint32_t probe_tau_us     = 0u;

    uint8_t outcome = measure_axis(hold, &axis_d, &probe_shape,
                                   SLOPE_PROBE_CYCLES, bus_mv,
                                   &probe_inductance, &probe_difference,
                                   &probe_step, &probe_tau_us);

    if (outcome != ESTIMATE_OK) {
        rest_bridge();
        HAL_Delay(DECAY_MS);
        gate_driver_disable_all();
        return outcome;
    }

    /* Second pass, at a half length suited to what the first pass
     * found. Both axes are measured with the same shape so that the
     * saliency is a comparison of like with like: the two differ in
     * inductance, and nothing else about how they were measured should
     * differ with them. */
    excitation_shape_t shape = shape_for_half(half_for_tau(probe_tau_us));

    result_out->half_periods = (uint16_t)shape.half_periods;

    outcome = measure_axis(hold, &axis_d, &shape, SLOPE_CYCLES, bus_mv,
                           &result_out->inductance_d_nh,
                           &result_out->d_difference_ma,
                           &result_out->d_duty_used,
                           &result_out->d_tau_us);

    if (outcome == ESTIMATE_OK) {
        /* Across the magnet axis, with the SAME holding current still
         * pinning the rotor in place. The excitation alternates, so the
         * torque it produces averages to nothing, and it alternates at
         * well over a kilohertz -- hundreds of times faster than the
         * rotor's own resonance in the holding field, which is where the
         * rotor's inability to follow it comes from.
         *
         * Measured independently of the d axis rather than assumed equal
         * to it: the difference between the two is the saliency, which
         * is the whole reason for taking two measurements. */
        outcome = measure_axis(hold, &axis_q, &shape, SLOPE_CYCLES, bus_mv,
                               &result_out->inductance_q_nh,
                               &result_out->q_difference_ma,
                               &result_out->q_duty_used,
                               &result_out->q_tau_us);
    }

    rest_bridge();
    HAL_Delay(DECAY_MS);
    gate_driver_disable_all();

    if (outcome != ESTIMATE_OK) {
        return outcome;
    }

    if (result_out->inductance_d_nh > 0u) {
        result_out->saliency_percent =
            (result_out->inductance_q_nh * 100u) / result_out->inductance_d_nh;
    }

    /* The resistance each axis implies, from R = L / tau.
     *
     * Not used for anything here -- it is reported because it is an
     * independent check on estimate_resistance(), arrived at with
     * nothing in common with it. Both axes should agree with each other,
     * since the winding has one resistance whatever its inductance, and
     * both should agree with what the resistance measurement found by
     * an entirely different route. Where they do not, the one drawing
     * amps through a sagging supply is the one to doubt. */
    if (result_out->d_tau_us > 0u) {
        result_out->d_mohm = (result_out->inductance_d_nh
                              + (result_out->d_tau_us / 2u))
                             / result_out->d_tau_us;
    }
    if (result_out->q_tau_us > 0u) {
        result_out->q_mohm = (result_out->inductance_q_nh
                              + (result_out->q_tau_us / 2u))
                             / result_out->q_tau_us;
    }

    /* The same two inductances as a meter across two motor leads would
     * see them. Current entering one terminal and leaving another passes
     * through two windings in series, and the flux that links is twice
     * what one axis carries, so the line-to-line figure is exactly
     * double the per-phase one.
     *
     * Reported because that is the form anyone checking this against an
     * LCR meter will have in front of them, and the factor of two
     * between the two conventions is otherwise an easy way to conclude
     * the measurement is out by half when it is not. */
    result_out->line_to_line_d_nh = result_out->inductance_d_nh * 2u;
    result_out->line_to_line_q_nh = result_out->inductance_q_nh * 2u;

    /* Stored in henries, converted from the nanohenries the protocol
     * reports. */
    motor_set_inductance(motor,
                         (float)result_out->inductance_d_nh * 1e-9f,
                         (float)result_out->inductance_q_nh * 1e-9f);

    return ESTIMATE_OK;
}

const char *estimate_pair_text(uint8_t pair)
{
    switch (pair) {
        case ESTIMATE_PAIR_AB: return "ab";
        case ESTIMATE_PAIR_BC: return "bc";
        case ESTIMATE_PAIR_CA: return "ca";
        default:               return "none";
    }
}

const char *estimate_result_text(uint8_t result)
{
    switch (result) {
        case ESTIMATE_OK:              return "ok";
        case ESTIMATE_ERR_NOT_READY:   return "control_loop_not_running";
        case ESTIMATE_ERR_TOO_SMALL:   return "current_change_too_small";
        case ESTIMATE_ERR_OVERCURRENT: return "overcurrent";
        case ESTIMATE_ERR_ARGUMENT:    return "bad_argument";
        default:                       return "unknown";
    }
}
