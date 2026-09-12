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
 * Inductance measurement settings
 * ------------------------------------------------------------------ */

/* The holding current the rotor is aligned with and the pulse steps
 * from, in milliamps, and how long to let the rotor settle at it.
 *
 * A CURRENT rather than a duty, and that is the important part. The
 * pulse measures a DIFFERENCE between two duties, and that difference
 * is free of the dead-time loss only if current was already flowing at
 * the lower of them -- the loss is then identical at both points and
 * cancels. A baseline below the dead time passes no current at all, so
 * the step crosses from not conducting to conducting and the entire
 * dead-time loss lands in the answer instead.
 *
 * An earlier version held a fixed duty of 8, which is a third of the
 * dead time on this board: the winding saw nothing, the baseline was
 * zero, and the pulse was measured against a starting point that was
 * not where it was assumed to be.
 *
 * Targeting a current fixes that for any board rather than for this
 * one. A fixed duty large enough to conduct on a 12 volt bus draws
 * twice as much on a 24 volt one and four times on a 48 volt one, and
 * a duty small enough to be safe there is back below the dead time
 * here. An amp and a half is enough to hold a rotor this board drives
 * against friction and cogging, and is a small fraction of what the
 * winding carries in use. */
#define ALIGN_TARGET_MA 1500
#define ALIGN_MS        400U

/* Highest duty the alignment ramp will reach before giving up, and how
 * long each of its steps settles and averages for, in milliseconds.
 *
 * The ramp doubles as the alignment: the rotor is dragged into line
 * gradually as the current comes up, rather than being slammed there by
 * a duty applied all at once. A few milliseconds a step is many
 * electrical time constants and a hundred and fifty steps is under a
 * second and a half in the worst case. */
#define ALIGN_MAX_DUTY    150U
#define ALIGN_SETTLE_MS   4U
#define ALIGN_AVERAGE_MS  4U

/* Duties added on top for the measuring pulse, smallest first.
 *
 * Started small and raised only if the current change was too small to
 * measure, so a low inductance winding is never hit harder than it
 * needs to be -- and, just as importantly, so a high voltage bus is not
 * hit as hard as a low voltage one. The change a given step produces
 * scales with the bus, so the step that satisfies
 * ESTIMATE_MINIMUM_CURRENT_CHANGE_MA on a 48 volt supply is a quarter
 * of the one needed on a 12 volt supply, and the current that flows is
 * about the same on both. That is why the list starts at 1 rather than
 * at something comfortable for this board: the first entry that works
 * is the one used. */
static const uint16_t pulse_steps[] = { 1u, 2u, 4u, 8u, 16u, 32u, 64u, 128u };
#define PULSE_STEP_COUNT (sizeof pulse_steps / sizeof pulse_steps[0])

/* How long the across-axis pre-hold lasts before its pulse, in
 * milliseconds.
 *
 * The across-axis field is ninety degrees from where the rotor is
 * sitting, so it produces maximum torque -- unlike the along-axis case,
 * this cannot be held for long. Two milliseconds is several L/R time
 * constants, so the current has settled and the pulse has a steady
 * baseline to step from, while being far less than the rotor needs to
 * accelerate anywhere meaningful. */
#define ACROSS_PREHOLD_MS 2U

/* How many phases' worth of winding each topology puts in the current's
 * path, doubled so the arithmetic stays in halves.
 *
 * Phase A driven against B and C in parallel is 1.5, and the flux that
 * geometry links at the terminals is 1.5 Ld. Phase B driven against C
 * with A floating is 2, and links 2 Lq. Both factors come out of the
 * winding geometry rather than being fitted, and both are used twice --
 * once to turn a terminal resistance into a series resistance for the
 * inversion, once to turn the series inductance back into a per-axis
 * one. */
#define ALONG_AXIS_PHASES_DOUBLED  3U
#define ACROSS_AXIS_PHASES_DOUBLED 4U

/* How far into its exponential a pulse may be before the inversion is
 * abandoned, as a fraction of the final current.
 *
 * L = -t R / ln(1 - dI R / V) loses its grip as that fraction
 * approaches one: the logarithm's argument goes to zero, so a couple of
 * counts of sensor noise move the answer by an unbounded amount. Past
 * this point the pulse has simply outlasted the winding and the only
 * real fix is a shorter one, which is what ESTIMATE_ERR_PULSE_TOO_LONG
 * says. */
#define PULSE_SETTLED_FRACTION_LIMIT 0.98f

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

/* Phase A at `duty`, B and C grounded. Field along phase A's own axis;
 * the winding sees 1.5 R and 1.5 L. */
static void drive_along_axis(uint16_t duty)
{
    gate_driver_set_duty(GATE_DRIVER_PHASE_B, 0u);
    gate_driver_set_duty(GATE_DRIVER_PHASE_C, 0u);
    gate_driver_set_duty(GATE_DRIVER_PHASE_A, duty);
}

/* Phase B at `duty`, C grounded, A left however the caller set it.
 * With A floating this is two phases in series: 2 R and 2 L, with the
 * field ninety electrical degrees from phase A's axis. */
static void drive_across_axis(uint16_t duty)
{
    gate_driver_set_duty(GATE_DRIVER_PHASE_C, 0u);
    gate_driver_set_duty(GATE_DRIVER_PHASE_B, duty);
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
 * @param phase          which phase's sensor to read
 * @param sign           +1 if current enters that phase's terminal in
 *                       this topology, -1 if it leaves
 * @param milliseconds   how long to average over
 * @param duty           the duty being applied, recorded if it faults
 * @param average_ma_out where the average is written, sign applied
 * @return 1 on success, 0 on sustained overcurrent */
static uint8_t average_phase_current(uint8_t   phase,
                                     int32_t   sign,
                                     uint32_t  milliseconds,
                                     uint16_t  duty,
                                     int32_t  *average_ma_out)
{
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

        total += phase_current_ma(phase, current_a, current_b) * sign;
        count++;
    }

    *average_ma_out = (count > 0u) ? (int32_t)(total / (int32_t)count) : 0;
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
 * @param pair          which phases to drive, ground, float and read
 * @param mohm_out      the two windings in series, in milliohms
 * @param bus_mv_out    the bus while current was flowing, millivolts
 * @return an ESTIMATE_ result code */
static uint8_t measure_pair(const resistance_pair_t *pair,
                            uint32_t *mohm_out,
                            uint32_t *bus_mv_out)
{
    uint16_t low_duty     = 0u;
    uint16_t high_duty    = 0u;
    int32_t  low_current  = 0;
    int32_t  high_current = 0;
    uint32_t bus_mv       = 0u;
    uint8_t  reached      = 0u;

    enable_pair(pair->high_phase, pair->low_phase);

    for (uint16_t duty = RESISTANCE_DUTY_STEP;
         duty <= RESISTANCE_MAX_DUTY;
         duty = (uint16_t)(duty + RESISTANCE_DUTY_STEP)) {

        gate_driver_set_duty(pair->low_phase, 0u);
        gate_driver_set_duty(pair->high_phase, duty);

        if (wait_watching_current(RESISTANCE_SETTLE_MS, duty) == 0u) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        int32_t measured;
        if (average_phase_current(pair->sense_phase, pair->sense_sign,
                                  RESISTANCE_AVERAGE_MS, duty,
                                  &measured) == 0u) {
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

            /* Read under load rather than before it: the supply sags
             * once the measurement starts drawing amps, and the voltage
             * that matters is the one actually available. */
            bus_mv = sensors_get_bus_mv();
        }

        if ((low_duty != 0u) && (measured >= RESISTANCE_HIGH_TARGET_MA)) {
            high_duty    = duty;
            high_current = measured;
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

    /* Millivolts over milliamps is ohms directly, so the thousand turns
     * it into milliohms. */
    int32_t voltage_change_mv =
        (int32_t)((bus_mv * (uint32_t)(high_duty - low_duty))
                  / GATE_DRIVER_DUTY_SCALE);

    *mohm_out   = (uint32_t)((voltage_change_mv * 1000) / current_change);
    *bus_mv_out = bus_mv;

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
    result_out->fault_pair        = ESTIMATE_PAIR_NONE;
    result_out->fault_duty        = 0u;
    result_out->fault_current_ma  = 0;

    uint32_t line_mohm[3] = { 0u, 0u, 0u };
    uint32_t bus_total    = 0u;
    uint8_t  outcome      = ESTIMATE_OK;

    for (uint8_t i = 0u; i < 3u; i++) {
        uint32_t bus_mv = 0u;

        fault_pair = i;

        outcome = measure_pair(&resistance_pairs[i], &line_mohm[i], &bus_mv);
        if (outcome != ESTIMATE_OK) {
            break;
        }

        bus_total += bus_mv;
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
 * ------------------------------------------------------------------ */

/* How long the measuring pulse lasts, in microseconds. A whole number of
 * control periods, each 1000000 / LOOP_RATE_HZ microseconds. */
#define PULSE_MICROSECONDS \
    ((ESTIMATE_PULSE_PERIODS * 1000000u) / LOOP_RATE_HZ)

/* Raise the along-axis duty until the holding current arrives.
 *
 * Also what aligns the rotor: the field points along phase A's axis
 * throughout and grows steadily, so the rotor is pulled into line as the
 * ramp climbs rather than being jerked there.
 *
 * @param duty_out  the duty that produced the target current
 * @return an ESTIMATE_ result code */
static uint8_t ramp_to_align_current(uint16_t *duty_out)
{
    for (uint16_t duty = 1u; duty <= ALIGN_MAX_DUTY; duty++) {

        drive_along_axis(duty);

        if (wait_watching_current(ALIGN_SETTLE_MS, duty) == 0u) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        int32_t measured;
        if (average_phase_current(GATE_DRIVER_PHASE_A, 1,
                                  ALIGN_AVERAGE_MS, duty,
                                  &measured) == 0u) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        fault_duty       = duty;
        fault_current_ma = measured;

        if (measured >= ALIGN_TARGET_MA) {
            *duty_out = duty;
            return ESTIMATE_OK;
        }
    }

    /* The ceiling was reached without the current arriving, which means
     * a phase is not connected -- there is no duty at which a broken
     * winding passes current. */
    return ESTIMATE_ERR_TOO_SMALL;
}

/* Step the voltage up and measure how fast the current climbs.
 *
 * The step is taken from a duty that is ALREADY passing a settled
 * current, not up from zero. Two things follow from that. The dead time
 * removes the same slice of voltage before and after, so it cancels in
 * the difference -- the same reasoning the resistance measurement's two
 * points rest on. And the current starts from its own steady state, so
 * the response to the step is a clean single exponential from a known
 * starting point, which is what estimate_inductance()'s inversion
 * assumes.
 *
 * Timed against the control loop's iteration counter rather than the
 * millisecond tick, because the pulse is a quarter of a millisecond long
 * and the tick has no resolution at that scale.
 *
 * @param phase       which phase's sensor carries the current
 * @param hold_duty   duty already applied, and the baseline to step from
 * @param step        how much to add for the pulse
 * @param along_axis  1 to pulse along phase A's axis, 0 to pulse across
 * @param change_out  current change in milliamps, signed
 * @return 1 on success, 0 if the current exceeded the limit */
static uint8_t pulse_and_measure(uint8_t   phase,
                                 uint16_t  hold_duty,
                                 uint16_t  step,
                                 uint8_t   along_axis,
                                 int32_t  *change_out)
{
    int32_t  current_a;
    int32_t  current_b;
    uint32_t start_iteration;
    uint32_t elapsed;

    sensors_get_currents(&current_a, &current_b);
    int32_t before = phase_current_ma(phase, current_a, current_b);

    start_iteration = loop_get_iteration_count();

    if (along_axis != 0u) {
        drive_along_axis((uint16_t)(hold_duty + step));
    } else {
        drive_across_axis((uint16_t)(hold_duty + step));
    }

    do {
        elapsed = loop_get_iteration_count() - start_iteration;
    } while (elapsed < ESTIMATE_PULSE_PERIODS);

    sensors_get_currents(&current_a, &current_b);
    int32_t after = phase_current_ma(phase, current_a, current_b);

    /* Back to the holding duty immediately, so the current stops
     * climbing the moment the measurement is over. */
    if (along_axis != 0u) {
        drive_along_axis(hold_duty);
    } else {
        drive_across_axis(hold_duty);
    }

    *change_out = after - before;

    int32_t largest = largest_phase_current_ma(current_a, current_b);

    if (largest > ESTIMATE_CURRENT_LIMIT_MA) {
        fault_duty       = (uint16_t)(hold_duty + step);
        fault_current_ma = largest;
        return 0u;
    }
    return 1u;
}

/* Turn one pulse into an inductance.
 *
 * The current in an R-L winding stepped by V from its own steady state
 * goes as dI(t) = (V/R)(1 - exp(-t R / L)), so
 *
 *     L = -t R / ln(1 - dI R / V)
 *
 * which is what this computes. The naive L = V t / dI is the first term
 * of that expansion and is only honest while t is far below L/R, which
 * on this board it is not -- see the note at the top of estimate.h.
 *
 * @param series_mohm    the resistance the pulse actually saw, milliohms
 * @param step_mv        the voltage step, millivolts
 * @param change_ma      the current change it produced, milliamps
 * @param series_nh_out  the series inductance, nanohenries. Left as a
 *                       float so the geometry factor divides into it
 *                       before anything is rounded.
 * @return an ESTIMATE_ result code */
static uint8_t invert_pulse(uint32_t  series_mohm,
                            uint32_t  step_mv,
                            int32_t   change_ma,
                            float    *series_nh_out)
{
    if ((step_mv == 0u) || (series_mohm == 0u)) {
        return ESTIMATE_ERR_ARGUMENT;
    }

    float series_ohm = (float)series_mohm * 0.001f;
    float step_volts = (float)step_mv * 0.001f;
    float change_amp = (float)absolute(change_ma) * 0.001f;

    /* How far along its exponential the current got, as a fraction of
     * where it would have ended up. */
    float settled = (change_amp * series_ohm) / step_volts;

    if (settled >= PULSE_SETTLED_FRACTION_LIMIT) {
        return ESTIMATE_ERR_PULSE_TOO_LONG;
    }
    if (settled <= 0.0f) {
        return ESTIMATE_ERR_TOO_SMALL;
    }

    /* Microseconds times ohms gives microhenries, so the thousand is
     * all that stands between that and nanohenries. */
    float tau_us = -(float)PULSE_MICROSECONDS / logf(1.0f - settled);

    *series_nh_out = tau_us * series_ohm * 1000.0f;

    return ESTIMATE_OK;
}

/* Measure the inductance along one axis, raising the pulse until the
 * current change is large enough to trust.
 *
 * @param phase           which phase's sensor carries the current
 * @param hold_duty       duty already applied, stepped up from
 * @param along_axis      1 for phase A's own axis, 0 for across it
 * @param phases_doubled  how many phases the current passes through in
 *                        this topology, times two -- 3 for A against B
 *                        and C in parallel, 4 for B against C. Kept
 *                        doubled so the division stays in integers.
 * @param inductance_out  per-axis result in nanohenries
 * @param change_out      the current change that produced it
 * @param step_out        the pulse step that was needed
 * @param series_mohm_out the series resistance it was solved against
 * @return an ESTIMATE_ result code */
static uint8_t measure_axis(uint8_t   phase,
                            uint16_t  hold_duty,
                            uint8_t   along_axis,
                            uint32_t  phases_doubled,
                            uint32_t *inductance_out,
                            int32_t  *change_out,
                            uint16_t *step_out,
                            uint32_t *series_mohm_out)
{
    uint32_t bus_mv = sensors_get_bus_mv();

    /* The pulse passes through more than one phase's worth of winding,
     * and through the same amount of each. The per-phase resistance the
     * motor carries scales up by the same factor the inductance will
     * scale back down by. */
    uint32_t phase_mohm =
        (uint32_t)((motor->resistance_ohm * 1000.0f) + 0.5f);
    uint32_t series_mohm = (phase_mohm * phases_doubled) / 2u;

    *series_mohm_out = series_mohm;

    uint8_t last = ESTIMATE_ERR_TOO_SMALL;

    for (uint32_t i = 0u; i < PULSE_STEP_COUNT; i++) {

        /* Several pulses at this step, averaged. Each one's change is a
         * difference between two single sensor readings, so repeating
         * is the cheapest precision available -- the pulse itself lasts
         * a quarter of a millisecond. */
        int32_t total = 0;

        for (uint32_t repeat = 0u; repeat < ESTIMATE_PULSE_REPEATS; repeat++) {

            int32_t one_change;

            if (pulse_and_measure(phase, hold_duty, pulse_steps[i],
                                  along_axis, &one_change) == 0u) {
                return ESTIMATE_ERR_OVERCURRENT;
            }

            total += one_change;

            /* Let the current fall back to its holding value before the
             * next pulse, so every one of them steps from the same
             * place. */
            if (wait_watching_current(ESTIMATE_PULSE_RECOVERY_MS,
                                      hold_duty) == 0u) {
                return ESTIMATE_ERR_OVERCURRENT;
            }
        }

        int32_t change = total / (int32_t)ESTIMATE_PULSE_REPEATS;

        *change_out = change;
        *step_out   = pulse_steps[i];

        if (absolute(change) < ESTIMATE_MINIMUM_CURRENT_CHANGE_MA) {
            last = ESTIMATE_ERR_TOO_SMALL;
            continue;                     /* too small to trust; try harder */
        }

        /* Only the STEP counts as the applied voltage: the holding duty
         * was already there before the pulse and is still there after,
         * so it drives no change. */
        uint32_t step_mv =
            (bus_mv * pulse_steps[i]) / GATE_DRIVER_DUTY_SCALE;

        float series_nh;

        last = invert_pulse(series_mohm, step_mv, change, &series_nh);
        if (last != ESTIMATE_OK) {
            /* A pulse that has outrun the winding will do so at every
             * step size -- the fraction settled does not depend on how
             * hard the step is -- so there is nothing to gain by trying
             * a bigger one. */
            return last;
        }

        /* Divide out the topology to get back to one axis, rounding
         * once, here, rather than at each step of the arithmetic. */
        *inductance_out =
            (uint32_t)(((series_nh * 2.0f) / (float)phases_doubled) + 0.5f);

        return ESTIMATE_OK;
    }

    return last;
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

    /* The inversion is in terms of R, so there is no answer at all
     * without one. Refusing is the honest response; the alternative is
     * the straight-line approximation, which reads high and collapses
     * the two axes onto each other. */
    if (motor->resistance_ohm <= 0.0f) {
        return ESTIMATE_ERR_NEED_RESISTANCE;
    }

    fault_pair       = ESTIMATE_PAIR_NONE;
    fault_duty       = 0u;
    fault_current_ma = 0;

    result_out->inductance_d_nh     = 0u;
    result_out->inductance_q_nh     = 0u;
    result_out->saliency_percent    = 0u;
    result_out->d_current_change_ma = 0;
    result_out->q_current_change_ma = 0;
    result_out->d_duty_used         = 0u;
    result_out->q_duty_used         = 0u;
    result_out->d_series_mohm       = 0u;
    result_out->q_series_mohm       = 0u;

    enable_bridge();

    /* Pull the rotor into line with phase A's axis by ramping the
     * current up to the holding target, then let it settle there. The
     * field points where the rotor is asked to go, so once it arrives
     * there is no torque left and it stays put. */
    uint16_t align_duty = 0u;

    uint8_t aligned = ramp_to_align_current(&align_duty);
    if (aligned != ESTIMATE_OK) {
        rest_bridge();
        HAL_Delay(DECAY_MS);
        gate_driver_disable_all();
        return aligned;
    }

    if (wait_watching_current(ALIGN_MS, align_duty) == 0u) {
        HAL_Delay(DECAY_MS);
        gate_driver_disable_all();
        return ESTIMATE_ERR_OVERCURRENT;
    }

    /* Along the magnet axis. The pulse points the same way the rotor is
     * already aligned, so it produces no torque and the rotor does not
     * move. Current passes through A and then B and C in parallel. */
    uint8_t outcome = measure_axis(GATE_DRIVER_PHASE_A, align_duty, 1u,
                                   ALONG_AXIS_PHASES_DOUBLED,
                                   &result_out->inductance_d_nh,
                                   &result_out->d_current_change_ma,
                                   &result_out->d_duty_used,
                                   &result_out->d_series_mohm);
    if (outcome != ESTIMATE_OK) {
        rest_bridge();
        HAL_Delay(DECAY_MS);
        gate_driver_disable_all();
        return outcome;
    }

    /* Across the magnet axis: phase A floated so the current has only
     * one path, B to C, ninety electrical degrees from where the rotor
     * is sitting.
     *
     * That direction produces maximum torque, so the pre-hold is kept to
     * a couple of milliseconds -- several time constants, so the current
     * is settled and the pulse steps from a known baseline, but far too
     * short for the rotor to accelerate anywhere that matters.
     *
     * The same duty is reused rather than re-ramped: this topology puts
     * two phases in series where the other put one and a half, so the
     * holding current comes out three quarters of what it was. The
     * baseline only has to be steady and conducting, not any particular
     * value, and re-ramping here would mean ramping under full torque.
     *
     * Measured independently of the d-axis rather than assumed equal to
     * it: the difference between the two is the saliency, which is the
     * whole reason for taking two measurements instead of one. */
    rest_bridge();
    HAL_Delay(DECAY_MS);
    gate_driver_disable_phase(GATE_DRIVER_PHASE_A);

    drive_across_axis(align_duty);
    if (wait_watching_current(ACROSS_PREHOLD_MS, align_duty) == 0u) {
        HAL_Delay(DECAY_MS);
        gate_driver_disable_all();
        return ESTIMATE_ERR_OVERCURRENT;
    }

    outcome = measure_axis(GATE_DRIVER_PHASE_B, align_duty, 0u,
                           ACROSS_AXIS_PHASES_DOUBLED,
                           &result_out->inductance_q_nh,
                           &result_out->q_current_change_ma,
                           &result_out->q_duty_used,
                           &result_out->q_series_mohm);

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
        case ESTIMATE_OK:                  return "ok";
        case ESTIMATE_ERR_NOT_READY:       return "control_loop_not_running";
        case ESTIMATE_ERR_TOO_SMALL:       return "current_change_too_small";
        case ESTIMATE_ERR_OVERCURRENT:     return "overcurrent";
        case ESTIMATE_ERR_ARGUMENT:        return "bad_argument";
        case ESTIMATE_ERR_NEED_RESISTANCE: return "measure_resistance_first";
        case ESTIMATE_ERR_PULSE_TOO_LONG:  return "pulse_outlasted_winding";
        default:                           return "unknown";
    }
}
