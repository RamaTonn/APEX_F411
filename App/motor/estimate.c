#include "estimate.h"

#include "loop.h"
#include "gate_driver.h"
#include "main.h"
#include "motor.h"
#include "sensors.h"
#include "units.h"

/* The motor being measured, for the duration of one run. */
static motor_t *motor;

/* The two currents the resistance measurement works between, in
 * milliamps.
 *
 * WHY CURRENTS RATHER THAN DUTIES OR VOLTAGES
 *
 *   A duty is a fraction of the bus, so the same duty gives twice the
 *   current at twice the supply voltage -- the measurement would depend
 *   on what the bench supply happened to be set to.
 *
 *   A fixed voltage is better but still needs the winding's resistance
 *   to be known in advance in order to choose one. Too small and no
 *   current flows; too large and it is dangerous. On this motor the
 *   whole useful range is under 300 millivolts, and 2 volts would draw
 *   nearly 40 amps.
 *
 *   Targeting a current sidesteps both. The voltage is raised from zero
 *   until the current arrives, which works on any winding and any supply
 *   without being told anything about either, and approaches the
 *   operating point from below rather than discovering it from above.
 *
 * WHY TWO OF THEM
 *
 *   The dead time removes a fixed amount of voltage -- about 290
 *   millivolts on this board -- before any of it reaches the winding.
 *   Measured at one point, that loss is indistinguishable from
 *   resistance, and gets attributed to it: an earlier single-point
 *   version of this measurement reported 186 milliohms for a 36 milliohm
 *   winding, and the difference was almost entirely dead time.
 *
 *   Between two points the loss is the same, so subtracting one from the
 *   other removes it completely and leaves only the winding. */
#define RESISTANCE_LOW_TARGET_MA  1500
#define RESISTANCE_HIGH_TARGET_MA 4000

/* How far the duty is raised each step while hunting for a target
 * current, in parts per thousand.
 *
 * Small enough that the current cannot go from safe to dangerous in one
 * step even on a winding with no resistance to speak of. */
#define RESISTANCE_DUTY_STEP 1U

/* Highest duty the hunt will reach before giving up. Well above what any
 * sensible winding needs, and still far below anything that would damage
 * the bridge.
 *
 * On this motor's winding (tens of milliohms) the targets below arrive
 * within about ten to twenty parts per thousand once the current is
 * actually being measured correctly (see driven_axis_current_ma()) --
 * this ceiling is a backstop against a disconnected phase or a wildly
 * different motor, not something the hunt should ordinarily approach. */
#define RESISTANCE_MAX_DUTY 120U

/* How long to hold each resistance point before reading, in
 * milliseconds. The current must reach its steady value, which takes
 * several electrical time constants -- a few milliseconds is thousands
 * of them on a motor this size. */
#define RESISTANCE_SETTLE_MS 50U

/* How long to average a steady current over, in milliseconds. */
#define RESISTANCE_AVERAGE_MS 20U

/* Duties tried for the inductance pulse, smallest first.
 *
 * Started small and raised only if the current change was too small to
 * measure, so a low inductance winding is never hit harder than it needs
 * to be.
 *
 * These are small because the winding is: at 36 milliohms on a 12 volt
 * bus, a duty of 4 parts per thousand is already most of an amp in
 * steady state, and the pulse adds to whatever current is holding the
 * rotor in place. An earlier version began at 20 and overcurrented on
 * its first attempt, with nothing smaller to fall back to. */
static const uint16_t pulse_duties[] = { 4u, 8u, 16u, 32u, 64u };
#define PULSE_DUTY_COUNT (sizeof pulse_duties / sizeof pulse_duties[0])

/* A quarter turn on the 16-bit electrical angle scale: ninety degrees,
 * which is the axis across the rotor's magnets. */
#define QUARTER_TURN 16384u

static int32_t absolute(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* Apply a vector on the q axis at one angle.
 *
 * Amplitude arrives as parts per thousand of the bus, as every command
 * on this board does, and is turned into volts against the measured bus;
 * motor_apply_dq turns it back into a duty.
 *
 * @param angle      electrical angle, 0 to 65535
 * @param amplitude  parts per thousand of bus voltage
 * @param current_a  phase A current, milliamps, for dead time correction
 * @param current_b  phase B current, milliamps, for dead time correction */
static void apply_vector(uint16_t angle,
                         uint16_t amplitude,
                         int32_t  current_a,
                         int32_t  current_b)
{
    float bus_volts = (float)sensors_get_bus_mv() * 0.001f;
    float v_q = ((float)amplitude / (float)GATE_DRIVER_DUTY_SCALE)
                * bus_volts;

    motor_apply_dq(motor, 0.0f, v_q,
                   (float)angle * MOTOR_ANGLE_TO_RAD,
                   current_a, current_b,
                   sensors_get_bus_mv());
}

/* Largest magnitude among the three phase currents, for the overcurrent
 * safety check.
 *
 * This has to look at real phase current rather than the driven-axis
 * current below: the safety limit is on what the windings and the
 * bridge actually carry, and at some angles a phase can carry current
 * while the driven-axis reading of a DIFFERENT phase sits near zero --
 * see driven_axis_current_ma() for why. Checking only one phase, as an
 * earlier version of this file did, could let real current climb
 * unnoticed on the other two. */
static int32_t largest_phase_current_ma(int32_t current_a_ma,
                                        int32_t current_b_ma)
{
    /* Phase C has no sensor. The three phase currents of a star-connected
     * winding must sum to zero, so C is whatever makes the sum zero. */
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

/* The current actually flowing along the axis being driven, in
 * milliamps.
 *
 * WHY THIS, AND NOT A RAW PHASE CURRENT
 *
 *   apply_vector() always drives a pure q-axis vector (Vd = 0) at
 *   whatever angle it is given. motor_apply_dq turns that into a
 *   BALANCED three-phase voltage set -- the three phase voltages always
 *   sum to zero, by construction of the inverse Clarke transform.
 *
 *   A balanced drive only puts its full magnitude onto one particular
 *   phase's own axis at one particular angle (ninety degrees from
 *   wherever that phase's own axis sits in this frame); at any other
 *   angle -- angle zero included, which is what this file used to read
 *   phase A at -- a phase's own current can sit near zero while real
 *   current flows through the other two. That is not a fault in the
 *   winding or the measurement setup: it is what a balanced vector
 *   looks like projected onto an axis it happens to be orthogonal to.
 *   No amount of raising the drive voltage changes that projection.
 *
 *   Reading the current back through the same Park/Clarke transform
 *   motor_get_dq_currents() already provides -- at the SAME angle the
 *   vector was driven at -- recovers the actual driven-axis magnitude
 *   correctly regardless of which angle was chosen, because the
 *   transform undoes exactly the rotation that made a raw phase reading
 *   angle-dependent in the first place. */
static int32_t driven_axis_current_ma(uint16_t angle,
                                      int32_t  current_a_ma,
                                      int32_t  current_b_ma)
{
    float id_a;
    float iq_a;

    motor_get_dq_currents(motor, current_a_ma, current_b_ma,
                          (float)angle * MOTOR_ANGLE_TO_RAD, &id_a, &iq_a);

    /* Vd is always zero going out, so Iq is always the component that
     * matters coming back -- see the note above. */
    return (int32_t)(iq_a * 1000.0f);
}

/* Average the driven-axis current over a period of time, and report the
 * largest raw phase current seen during that window for the overcurrent
 * check.
 *
 * The control loop samples at a fixed point in every switching period,
 * so its values are already free of switching ripple. Averaging removes
 * what remains.
 *
 * @param angle             electrical angle the vector is held at
 * @param amplitude         parts per thousand of bus voltage
 * @param milliseconds      how long to average over
 * @param peak_phase_ma_out largest raw phase current magnitude seen,
 *                          in milliamps. May be NULL.
 * @return average driven-axis current in milliamps */
static int32_t average_current_ma(uint16_t angle,
                                  uint16_t amplitude,
                                  uint32_t milliseconds,
                                  int32_t *peak_phase_ma_out)
{
    int64_t  total      = 0;
    uint32_t count      = 0u;
    uint32_t start       = HAL_GetTick();
    int32_t  peak_phase = 0;
    int32_t  current_a;
    int32_t  current_b;

    while ((HAL_GetTick() - start) < milliseconds) {
        sensors_get_currents(&current_a, &current_b);

        /* The vector is re-applied while averaging, for the same reason
         * hold_vector_for exists: the dead time correction is computed
         * from the present current, so it has to be recomputed as that
         * current moves. Leaving the duties as they were would freeze
         * the correction at whatever it happened to be. */
        apply_vector(angle, amplitude, current_a, current_b);

        int32_t phase_peak = largest_phase_current_ma(current_a, current_b);
        if (phase_peak > peak_phase) {
            peak_phase = phase_peak;
        }

        total += driven_axis_current_ma(angle, current_a, current_b);
        count++;
    }

    if (peak_phase_ma_out != NULL) {
        *peak_phase_ma_out = peak_phase;
    }

    return (count > 0u) ? (int32_t)(total / (int32_t)count) : 0;
}

/* Hold a voltage vector steady for a period, re-applying it as the
 * current changes.
 *
 * Applying it once is not enough. The dead time correction inside
 * motor_apply_dq depends on the measured current, and at the moment the
 * vector is first applied that current is zero -- so no correction is
 * made, the commanded voltage is swallowed by the dead time, and no
 * current ever starts to flow. The measurement then reports that
 * nothing happened, which is true but for the wrong reason.
 *
 * Re-applying lets the correction find its own level: whatever small
 * current does begin to flow produces a correction, which admits more
 * voltage, which produces more current, until it settles.
 *
 * @param angle         electrical angle to hold
 * @param amplitude     parts per thousand of bus voltage
 * @param milliseconds  how long to hold it */
static void hold_vector_for(uint16_t angle,
                            uint16_t amplitude,
                            uint32_t milliseconds)
{
    uint32_t start = HAL_GetTick();
    int32_t  current_a;
    int32_t  current_b;

    while ((HAL_GetTick() - start) < milliseconds) {
        sensors_get_currents(&current_a, &current_b);
        apply_vector(angle, amplitude, current_a, current_b);
    }
}

/* Enable every phase with nothing applied. */
static void enable_bridge(void)
{
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_set_duty(phase, 500u);
    }
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_enable_phase(phase);
    }
}

/* ------------------------------------------------------------------
 * Resistance
 * ------------------------------------------------------------------ */

/* Raise the applied voltage until the current reaches a target.
 *
 * Starts from wherever the caller left the duty, so a second call
 * continues upward from the first rather than beginning again -- which
 * both saves time and avoids letting the current fall back to zero
 * between the two measurement points.
 *
 * The angle held throughout is zero. Which angle is used no longer
 * matters for correctness -- see driven_axis_current_ma() -- so zero is
 * as good as any other and keeps this aligned with hold_vector_for's
 * own default.
 *
 * @param target_ma      current to reach, in milliamps
 * @param duty_in_out    duty to start from, updated to the duty reached
 * @param current_out    the driven-axis current actually measured there
 * @return an ESTIMATE_ result code */
static uint8_t hunt_for_current(int32_t   target_ma,
                                uint16_t *duty_in_out,
                                int32_t  *current_out)
{
    for (uint16_t duty = *duty_in_out;
         duty <= RESISTANCE_MAX_DUTY;
         duty = (uint16_t)(duty + RESISTANCE_DUTY_STEP)) {

        /* Held rather than applied once, so the dead time correction can
         * settle: it is computed from the measured current, which is
         * zero at the instant a new duty is first written. */
        hold_vector_for(0u, duty, RESISTANCE_SETTLE_MS);

        int32_t peak_phase_ma;
        int32_t measured = average_current_ma(0u, duty, RESISTANCE_AVERAGE_MS,
                                              &peak_phase_ma);

        if (peak_phase_ma > ESTIMATE_CURRENT_LIMIT_MA) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        if (absolute(measured) >= target_ma) {
            *duty_in_out = duty;
            *current_out = measured;
            return ESTIMATE_OK;
        }
    }

    /* The ceiling was reached without the current arriving. Either the
     * winding is far more resistive than anything expected here, or a
     * phase is not connected. */
    return ESTIMATE_ERR_TOO_SMALL;
}

uint8_t estimate_resistance(motor_t *m,
                            estimate_resistance_result_t *result_out)
{
    /* Held for the run so the static helpers can reach the motor. */
    motor = m;

    uint16_t duty = RESISTANCE_DUTY_STEP;
    int32_t  low_current;
    int32_t  high_current;
    uint16_t low_duty;

    if (result_out == NULL) {
        return ESTIMATE_ERR_ARGUMENT;
    }
    if (loop_is_running() == 0u) {
        return ESTIMATE_ERR_NOT_READY;
    }

    enable_bridge();

    uint8_t outcome = hunt_for_current(RESISTANCE_LOW_TARGET_MA,
                                       &duty, &low_current);
    if (outcome != ESTIMATE_OK) {
        gate_driver_disable_all();
        return outcome;
    }
    low_duty = duty;

    /* The bus is read between the two points rather than before them,
     * because it sags under the load the measurement itself creates --
     * particularly with a resistor in series with the supply. Reading it
     * while current is flowing gives the voltage that is actually
     * available. */
    uint32_t bus_millivolts = sensors_get_bus_mv();

    outcome = hunt_for_current(RESISTANCE_HIGH_TARGET_MA,
                               &duty, &high_current);

    gate_driver_disable_all();

    if (outcome != ESTIMATE_OK) {
        return outcome;
    }

    int32_t current_change = high_current - low_current;

    if (absolute(current_change) < ESTIMATE_MINIMUM_CURRENT_CHANGE_MA) {
        result_out->resistance_mohm = 0u;
        return ESTIMATE_ERR_TOO_SMALL;
    }

    /* The slope between the two points.
     *
     * Both were measured with the same dead time loss, so the difference
     * between them contains none of it -- only the extra voltage that
     * produced the extra current.
     *
     * NO FACTOR OF TWO-THIRDS HERE
     *
     *   An earlier version of this measurement scaled the result by 2/3,
     *   on the reasoning that current driven into one phase returns
     *   through the other two in parallel -- true for injecting current
     *   through a single terminal, with the other two grounded.
     *
     *   That is not what happens here. motor_apply_dq always produces a
     *   BALANCED three-phase voltage set: the three phase voltages sum
     *   to zero by construction of the inverse Clarke transform, the
     *   same way the three phase currents of a star winding always do.
     *   For a balanced drive into a floating-neutral star winding, the
     *   star point sits at zero by symmetry, and each phase's current is
     *   simply that phase's own voltage divided by its resistance --
     *   which means, since the Park/Clarke transform is linear, that the
     *   q-axis current is just the q-axis voltage divided by R, with no
     *   correction factor at all. Applying one anyway would have under-
     *   reported every resistance measured this way by a third. */
    int32_t voltage_change_mv =
        (int32_t)((bus_millivolts * (uint32_t)(duty - low_duty))
                  / GATE_DRIVER_DUTY_SCALE);

    int32_t total_milliohms = (voltage_change_mv * 1000) / current_change;

    result_out->resistance_mohm = (uint32_t)absolute(total_milliohms);
    result_out->low_duty        = low_duty;
    result_out->high_duty       = duty;
    result_out->low_current_ma  = low_current;
    result_out->high_current_ma = high_current;

    /* Stored in ohms: the motor holds SI units, the protocol keeps
     * milliohms. */
    motor_set_resistance(motor, (float)result_out->resistance_mohm * 0.001f);

    return ESTIMATE_OK;
}

/* ------------------------------------------------------------------
 * Inductance
 * ------------------------------------------------------------------ */

/* Apply a voltage pulse along one axis and measure how far the
 * driven-axis current moved.
 *
 * Timed against the control loop's own iteration counter rather than the
 * millisecond tick, because the pulse is a quarter of a millisecond long
 * and the tick has no resolution at that scale. Counting control periods
 * gives exactly 31.25 microseconds each.
 *
 * @param angle            electrical angle to pulse along
 * @param duty             pulse strength, parts per thousand
 * @param change_out       driven-axis current change in milliamps, signed
 * @return 1 on success, 0 if the current exceeded the limit */
static uint8_t apply_pulse(uint16_t angle, uint16_t duty, int32_t *change_out)
{
    int32_t  current_a;
    int32_t  current_b;
    uint32_t start_iteration;
    uint32_t elapsed;

    /* The current before the pulse. Not assumed to be zero: the rotor is
     * being held in place by a steady current, and that is the baseline
     * the change is measured from. Read along the pulse's own axis --
     * if that axis is not the one the holding current was driven on,
     * this comes back near zero, which is correct: the holding vector
     * has no component on an axis orthogonal to it, the same reasoning
     * driven_axis_current_ma() documents. */
    sensors_get_currents(&current_a, &current_b);
    int32_t before = driven_axis_current_ma(angle, current_a, current_b);

    start_iteration = loop_get_iteration_count();

    apply_vector(angle, duty, 0, 0);

    /* Spin until the loop has run the required number of periods. The
     * subtraction is correct across the counter's wrap. */
    do {
        elapsed = loop_get_iteration_count() - start_iteration;
    } while (elapsed < ESTIMATE_PULSE_PERIODS);

    sensors_get_currents(&current_a, &current_b);
    int32_t after       = driven_axis_current_ma(angle, current_a, current_b);
    int32_t peak_phase_ma = largest_phase_current_ma(current_a, current_b);

    /* Remove the pulse immediately. Every phase back to half duty means
     * all three terminals sit at the same potential and the current
     * decays rather than continuing to climb. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_set_duty(phase, 500u);
    }

    *change_out = after - before;

    return (peak_phase_ma > ESTIMATE_CURRENT_LIMIT_MA) ? 0u : 1u;
}

/* Measure the inductance along one axis, raising the pulse voltage until
 * the current change is large enough to be meaningful.
 *
 * @param angle             electrical angle to pulse along
 * @param hold_duty         current holding the rotor in place, applied
 *                          at electrical angle zero (see
 *                          estimate_inductance())
 * @param inductance_out    result in microhenries
 * @param change_out        the current change that produced it
 * @param duty_out          the pulse duty that was needed
 * @return an ESTIMATE_ result code */
static uint8_t measure_axis(uint16_t  angle,
                            uint16_t  hold_duty,
                            uint32_t *inductance_out,
                            int32_t  *change_out,
                            uint16_t *duty_out)
{
    uint32_t bus_millivolts = sensors_get_bus_mv();

    for (uint32_t i = 0u; i < PULSE_DUTY_COUNT; i++) {

        /* Re-establish the holding current and let the rotor settle back
         * before each attempt, so every pulse starts from the same
         * place. */
        hold_vector_for(0u, hold_duty, 50u);

        int32_t change;

        if (apply_pulse(angle, pulse_duties[i], &change) == 0u) {
            return ESTIMATE_ERR_OVERCURRENT;
        }

        if (absolute(change) < ESTIMATE_MINIMUM_CURRENT_CHANGE_MA) {
            continue;                     /* too small to trust; try harder */
        }

        /* Inductance is the applied voltage times the time, divided by
         * the current change.
         *
         * THE VOLTAGE ACTUALLY APPLIED IS A STEP, NOT AN ABSOLUTE VALUE
         *
         *   apply_pulse() commands pulse_duties[i] outright -- it does
         *   not add to whatever was there before. The holding vector is
         *   driven at electrical angle zero, so it contributes
         *   hold_duty's worth of voltage to a pulse that shares that
         *   same angle, and nothing to one ninety degrees away (a
         *   vector has no component on an axis orthogonal to it, same
         *   as the current it produces). Subtracting that prior
         *   contribution is what turns pulse_duties[i] into the actual
         *   step the winding saw. Skipping this would overstate the
         *   voltage for the angle-zero measurement specifically, by
         *   double-counting the four parts per thousand already being
         *   applied to hold the rotor there. */
        uint16_t prior_duty_on_axis = (angle == 0u) ? hold_duty : 0u;

        uint32_t applied_mv =
            (bus_millivolts * (uint32_t)(pulse_duties[i] - prior_duty_on_axis))
            / GATE_DRIVER_DUTY_SCALE;

        uint32_t pulse_microseconds =
            (ESTIMATE_PULSE_PERIODS * 1000000u) / LOOP_RATE_HZ;

        *inductance_out = (applied_mv * pulse_microseconds)
                          / (uint32_t)absolute(change);
        *change_out = change;
        *duty_out   = pulse_duties[i];

        return ESTIMATE_OK;
    }

    return ESTIMATE_ERR_TOO_SMALL;
}

uint8_t estimate_inductance(motor_t *m,
                            estimate_inductance_result_t *result_out)
{
    motor = m;

    /* Current used to hold the rotor aligned while the pulses happen.
     *
     * Kept low because it counts against the same current limit the
     * pulses do: a hold that already draws two amps leaves very little
     * headroom before a pulse trips the limit. Four parts per thousand
     * is around one amp here, enough to keep a small rotor from turning
     * during a quarter-millisecond pulse. Matches pulse_duties[0], so
     * the first pulse attempt at electrical angle zero is a genuine
     * zero-volt step rather than a wasted attempt -- see measure_axis(). */
    const uint16_t hold_duty = 4u;

    if (result_out == NULL) {
        return ESTIMATE_ERR_ARGUMENT;
    }
    if (loop_is_running() == 0u) {
        return ESTIMATE_ERR_NOT_READY;
    }

    result_out->inductance_d_uh     = 0u;
    result_out->inductance_q_uh     = 0u;
    result_out->saliency_percent    = 0u;
    result_out->d_current_change_ma = 0;
    result_out->q_current_change_ma = 0;
    result_out->duty_used           = 0u;

    enable_bridge();

    /* Pull the rotor into line with electrical angle zero and let it
     * settle. Every pulse below is measured relative to this position. */
    hold_vector_for(0u, hold_duty, ESTIMATE_HOLD_MS);

    /* Along the magnet axis. The pulse points the same way the rotor is
     * already aligned, so it produces no torque and the rotor does not
     * move. */
    uint8_t outcome = measure_axis(0u, hold_duty,
                                   &result_out->inductance_d_uh,
                                   &result_out->d_current_change_ma,
                                   &result_out->duty_used);
    if (outcome != ESTIMATE_OK) {
        gate_driver_disable_all();
        return outcome;
    }

    /* Across the magnet axis. This direction does produce torque, but
     * the pulse lasts a quarter of a millisecond and the rotor's inertia
     * means it barely begins to move before the pulse is over. */
    outcome = measure_axis(QUARTER_TURN, hold_duty,
                           &result_out->inductance_q_uh,
                           &result_out->q_current_change_ma,
                           &result_out->duty_used);

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
