#include "estimate.h"

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
 * Driving one phase against the other two, the winding sees the full
 * duty as a fraction of the bus -- 150 parts per thousand is 1.8 volts
 * on a 12 volt bus, which is already 30 amps through a 60 milliohm
 * path. The sustained-current guard stops the run long before this in
 * practice; the ceiling is a backstop for a disconnected phase, where
 * no current flows however high the duty goes. */
#define RESISTANCE_MAX_DUTY 150U

/* How long to let the current settle after a duty change, and how long
 * to average it for once settled, both in milliseconds.
 *
 * The winding reaches its steady current in a few electrical time
 * constants -- microseconds on a motor this size -- so the settle is
 * generous. The average removes what noise the control loop's
 * synchronous sampling has not already. */
#define RESISTANCE_SETTLE_MS  20U
#define RESISTANCE_AVERAGE_MS 20U

/* ------------------------------------------------------------------
 * Inductance measurement settings
 * ------------------------------------------------------------------ */

/* Duty used to pull the rotor into line with phase A, and how long to
 * hold it there.
 *
 * This produces a real current, so it is kept modest; it only has to
 * overcome friction and cogging, not accelerate anything. */
#define ALIGN_DUTY 8U
#define ALIGN_MS   400U

/* Duties added on top for the measuring pulse, smallest first.
 *
 * Started small and raised only if the current change was too small to
 * measure, so a low inductance winding is never hit harder than it
 * needs to be. */
static const uint16_t pulse_steps[] = { 8u, 16u, 32u, 64u, 128u };
#define PULSE_STEP_COUNT (sizeof pulse_steps / sizeof pulse_steps[0])

/* How long the across-axis pre-hold lasts before its pulse, in
 * milliseconds.
 *
 * The across-axis field is ninety degrees from where the rotor is
 * sitting, so it produces maximum torque -- unlike the along-axis case,
 * this cannot be held for long. A couple of milliseconds is far less
 * than the rotor needs to accelerate anywhere meaningful, and it gives
 * the current somewhere to start from so the dead time cancels in the
 * step the same way it does along the other axis. */
#define ACROSS_PREHOLD_MS 2U

/* ------------------------------------------------------------------
 * Small helpers
 * ------------------------------------------------------------------ */

static int32_t absolute(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* Largest magnitude among the three phase currents.
 *
 * Phase C has no sensor; the three currents of a star-connected winding
 * sum to zero, so C is whatever makes the sum zero. The limit is on what
 * the windings and the bridge physically carry, so all three count --
 * whichever phase the measurement happens to be reading. */
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

/* Enable every phase, starting from ground. */
static void enable_bridge(void)
{
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_set_duty(phase, 0u);
    }
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_enable_phase(phase);
    }
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
 * @param milliseconds   how long to average over
 * @param duty           the duty being applied, recorded if it faults
 * @param average_ma_out where the average is written
 * @return 1 on success, 0 on sustained overcurrent */
static uint8_t average_phase_current(uint8_t   phase,
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

        total += phase_current_ma(phase, current_a, current_b);
        count++;
    }

    *average_ma_out = (count > 0u) ? (int32_t)(total / (int32_t)count) : 0;
    return 1u;
}

/* ------------------------------------------------------------------
 * Resistance
 * ------------------------------------------------------------------ */

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

    fault_duty       = 0u;
    fault_current_ma = 0;

    result_out->resistance_mohm  = 0u;
    result_out->low_duty         = 0u;
    result_out->high_duty        = 0u;
    result_out->low_current_ma   = 0;
    result_out->high_current_ma  = 0;
    result_out->bus_mv           = 0u;
    result_out->fault_duty       = 0u;
    result_out->fault_current_ma = 0;

    enable_bridge();

    uint16_t low_duty     = 0u;
    uint16_t high_duty    = 0u;
    int32_t  low_current  = 0;
    int32_t  high_current = 0;
    uint32_t bus_mv       = 0u;
    uint8_t  outcome      = ESTIMATE_ERR_TOO_SMALL;

    for (uint16_t duty = RESISTANCE_DUTY_STEP;
         duty <= RESISTANCE_MAX_DUTY;
         duty = (uint16_t)(duty + RESISTANCE_DUTY_STEP)) {

        drive_along_axis(duty);

        if (wait_watching_current(RESISTANCE_SETTLE_MS, duty) == 0u) {
            outcome = ESTIMATE_ERR_OVERCURRENT;
            break;
        }

        int32_t measured;
        if (average_phase_current(GATE_DRIVER_PHASE_A, RESISTANCE_AVERAGE_MS,
                                  duty, &measured) == 0u) {
            outcome = ESTIMATE_ERR_OVERCURRENT;
            break;
        }

        /* Recorded every step, so a run that reaches the ceiling can
         * report how far the current actually got rather than leaving
         * it to be guessed at. */
        fault_duty       = duty;
        fault_current_ma = measured;

        if ((low_duty == 0u) && (absolute(measured) >= RESISTANCE_LOW_TARGET_MA)) {
            low_duty    = duty;
            low_current = measured;

            /* Read under load rather than before it: the supply sags
             * once the measurement starts drawing amps, and the voltage
             * that matters is the one actually available. */
            bus_mv = sensors_get_bus_mv();
        }

        if ((low_duty != 0u) && (absolute(measured) >= RESISTANCE_HIGH_TARGET_MA)) {
            high_duty    = duty;
            high_current = measured;
            outcome      = ESTIMATE_OK;
            break;
        }
    }

    rest_bridge();
    gate_driver_disable_all();

    result_out->low_duty         = low_duty;
    result_out->low_current_ma   = low_current;
    result_out->high_duty        = high_duty;
    result_out->high_current_ma  = high_current;
    result_out->bus_mv           = bus_mv;
    result_out->fault_duty       = fault_duty;
    result_out->fault_current_ma = fault_current_ma;

    if (outcome != ESTIMATE_OK) {
        return outcome;
    }

    int32_t current_change = high_current - low_current;

    if (absolute(current_change) < ESTIMATE_MINIMUM_CURRENT_CHANGE_MA) {
        return ESTIMATE_ERR_TOO_SMALL;
    }

    /* The slope between the two points. Both were measured with the same
     * dead time loss, so the difference between them contains none of
     * it -- only the extra voltage that produced the extra current.
     *
     * Millivolts over milliamps is ohms directly, so the thousand turns
     * it into milliohms. */
    int32_t voltage_change_mv =
        (int32_t)((bus_mv * (uint32_t)(high_duty - low_duty))
                  / GATE_DRIVER_DUTY_SCALE);

    int32_t terminal_mohm = (voltage_change_mv * 1000) / current_change;

    /* What the slope measured is the whole path: phase A out, phases B
     * and C back in parallel, which is R + R/2. Two thirds of it is one
     * phase. */
    result_out->resistance_mohm = (uint32_t)absolute((terminal_mohm * 2) / 3);

    /* Stored in ohms: the motor holds SI units, the protocol keeps
     * milliohms. */
    motor_set_resistance(motor, (float)result_out->resistance_mohm * 0.001f);

    return ESTIMATE_OK;
}

/* ------------------------------------------------------------------
 * Inductance
 * ------------------------------------------------------------------ */

/* How long the measuring pulse lasts, in microseconds. A whole number of
 * control periods, each 1000000 / LOOP_RATE_HZ microseconds. */
#define PULSE_MICROSECONDS \
    ((ESTIMATE_PULSE_PERIODS * 1000000u) / LOOP_RATE_HZ)

/* Step the voltage up and measure how fast the current climbs.
 *
 * The step is taken from a duty that is ALREADY passing current, not up
 * from zero, so the dead time removes the same slice of voltage before
 * and after and cancels in the difference -- the same reasoning the
 * resistance measurement's two points rest on.
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

    return (largest_phase_current_ma(current_a, current_b)
                > ESTIMATE_CURRENT_LIMIT_MA) ? 0u : 1u;
}

/* Measure the inductance along one axis, raising the pulse until the
 * current change is large enough to trust.
 *
 * @param phase           which phase's sensor carries the current
 * @param hold_duty       duty already applied, stepped up from
 * @param along_axis      1 for phase A's own axis, 0 for across it
 * @param series_phases_doubled
 *                        how many phases the current passes through in
 *                        this topology, times two: 3 for A against B and
 *                        C in parallel (1.5 phases), 4 for B against C
 *                        (2 phases). Kept doubled so the division stays
 *                        in integers.
 * @param inductance_out  result in microhenries
 * @param change_out      the current change that produced it
 * @param step_out        the pulse step that was needed
 * @return an ESTIMATE_ result code */
static uint8_t measure_axis(uint8_t   phase,
                            uint16_t  hold_duty,
                            uint8_t   along_axis,
                            uint32_t  series_phases_doubled,
                            uint32_t *inductance_out,
                            int32_t  *change_out,
                            uint16_t *step_out)
{
    uint32_t bus_mv = sensors_get_bus_mv();

    for (uint32_t i = 0u; i < PULSE_STEP_COUNT; i++) {

        int32_t change;

        if (pulse_and_measure(phase, hold_duty, pulse_steps[i],
                              along_axis, &change) == 0u) {
            fault_duty       = (uint16_t)(hold_duty + pulse_steps[i]);
            fault_current_ma = change;
            return ESTIMATE_ERR_OVERCURRENT;
        }

        if (absolute(change) < ESTIMATE_MINIMUM_CURRENT_CHANGE_MA) {
            continue;                     /* too small to trust; try harder */
        }

        /* Inductance is the applied voltage times the time, divided by
         * the current change. Millivolts times microseconds over
         * milliamps gives microhenries directly, with no further
         * scaling.
         *
         * Only the STEP counts as the applied voltage: the holding duty
         * was already there before the pulse and is still there after,
         * so it drives no change. */
        uint32_t step_mv =
            (bus_mv * pulse_steps[i]) / GATE_DRIVER_DUTY_SCALE;

        uint32_t series_inductance_uh =
            (step_mv * PULSE_MICROSECONDS) / (uint32_t)absolute(change);

        /* Divide out the topology: the pulse passed through more than
         * one phase's worth of winding. */
        *inductance_out = (series_inductance_uh * 2u) / series_phases_doubled;
        *change_out     = change;
        *step_out       = pulse_steps[i];

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

    fault_duty       = 0u;
    fault_current_ma = 0;

    result_out->inductance_d_uh     = 0u;
    result_out->inductance_q_uh     = 0u;
    result_out->saliency_percent    = 0u;
    result_out->d_current_change_ma = 0;
    result_out->q_current_change_ma = 0;
    result_out->duty_used           = 0u;

    enable_bridge();

    /* Pull the rotor into line with phase A's axis and let it settle.
     * The field points where the rotor is asked to go, so once it
     * arrives there is no torque left and it stays put. */
    drive_along_axis(ALIGN_DUTY);
    if (wait_watching_current(ALIGN_MS, ALIGN_DUTY) == 0u) {
        gate_driver_disable_all();
        return ESTIMATE_ERR_OVERCURRENT;
    }

    /* Along the magnet axis. The pulse points the same way the rotor is
     * already aligned, so it produces no torque and the rotor does not
     * move. Current passes through A and then B and C in parallel:
     * 1.5 phases, so 3 halves. */
    uint8_t outcome = measure_axis(GATE_DRIVER_PHASE_A, ALIGN_DUTY, 1u, 3u,
                                   &result_out->inductance_d_uh,
                                   &result_out->d_current_change_ma,
                                   &result_out->duty_used);
    if (outcome != ESTIMATE_OK) {
        rest_bridge();
        gate_driver_disable_all();
        return outcome;
    }

    /* Across the magnet axis: phase A floated so the current has only
     * one path, B to C, ninety electrical degrees from where the rotor
     * is sitting.
     *
     * That direction produces maximum torque, so the pre-hold is kept to
     * a couple of milliseconds -- long enough to give the pulse a
     * current to step from, far too short for the rotor to accelerate
     * anywhere that matters. */
    rest_bridge();
    gate_driver_disable_phase(GATE_DRIVER_PHASE_A);

    drive_across_axis(ALIGN_DUTY);
    if (wait_watching_current(ACROSS_PREHOLD_MS, ALIGN_DUTY) == 0u) {
        gate_driver_disable_all();
        return ESTIMATE_ERR_OVERCURRENT;
    }

    /* Current passes through B and then C: 2 phases, so 4 halves. */
    uint16_t across_step = 0u;
    outcome = measure_axis(GATE_DRIVER_PHASE_B, ALIGN_DUTY, 0u, 4u,
                           &result_out->inductance_q_uh,
                           &result_out->q_current_change_ma,
                           &across_step);

    rest_bridge();
    gate_driver_disable_all();

    if (outcome != ESTIMATE_OK) {
        return outcome;
    }

    if (result_out->inductance_d_uh > 0u) {
        result_out->saliency_percent =
            (result_out->inductance_q_uh * 100u) / result_out->inductance_d_uh;
    }

    /* Stored in henries, converted from the microhenries the protocol
     * reports. */
    motor_set_inductance(motor,
                         (float)result_out->inductance_d_uh * 1e-6f,
                         (float)result_out->inductance_q_uh * 1e-6f);

    return ESTIMATE_OK;
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
