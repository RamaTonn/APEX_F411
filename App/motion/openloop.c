#include "openloop.h"

#include "loop.h"
#include "gate_driver.h"
#include "main.h"
#include "motor.h"
#include "sensors.h"
#include "telemetry.h"

/* ------------------------------------------------------------------
 * Phase accumulator
 *
 * The applied electrical angle lives in a 32-bit counter that is allowed
 * to overflow. One complete overflow is one electrical revolution, so
 * the wrap-around at the top of the range IS the wrap from 360 degrees
 * back to zero. No comparison or subtraction is needed to keep the angle
 * in range, which is both faster and impossible to get wrong.
 *
 * The upper bits form the electrical angle handed to motor_apply_dq; the lower bits are fractional precision that accumulates
 * between steps -- and that is what makes a frequency of 1 Hz work at
 * all. One 32000th of a revolution per step is a tiny fraction of a
 * degree, so without the fractional bits it would round to zero and the
 * vector would never move.
 * ------------------------------------------------------------------ */

/* How far the accumulator advances per step, per hertz of commanded
 * frequency.
 *
 * A full revolution is 2^32 accumulator units and there are
 * LOOP_RATE_HZ steps per second, so one hertz advances by 2^32
 * divided by the loop rate each step. 4294967296 over 32000 is 134217.7,
 * so this rounds down; the resulting frequency error is under one part
 * in 100000. */
#define ACCUMULATOR_STEP_PER_HZ 134217u

/* One third of a revolution in accumulator units, for the 120 degree
 * spacing between phases.
 *
 * Applied in the accumulator domain rather than as a table index offset.
 * 256 divided by 3 is not a whole number, so offsetting the index would
 * put the phases half a degree out; offsetting the accumulator is exact
 * to within one part in four billion. */
#define ACCUMULATOR_THIRD_TURN 1431655765u

/* ------------------------------------------------------------------
 * Module state
 *
 * Everything the control interrupt touches is volatile, because the main
 * loop reads and writes these too and the compiler cannot see that an
 * interrupt exists.
 * ------------------------------------------------------------------ */

/* Where the rotating vector currently points, in accumulator units. */
/* The motor this drive commands. */
static motor_t *motor;

static volatile uint32_t angle_accumulator;

/* How far the accumulator advances each control period. Derived from the
 * commanded frequency; this is what the loop actually uses. */
static volatile uint32_t accumulator_increment;

/* Peak phase voltage, as parts per thousand of the bus voltage. */
static volatile uint16_t commanded_amplitude;

/* Commanded rotation rate in hertz, kept only so it can be reported
 * back to the console. */
static volatile uint16_t commanded_frequency_hz;

/* Non-zero while the vector is rotating. */
static volatile uint8_t running;

/* Set when the control interrupt stopped the loop because current
 * exceeded the abort threshold. Cleared by the next successful start, so
 * a caller can tell a deliberate stop from an abort. */
static volatile uint8_t aborted_on_overcurrent;

/* The largest phase current seen at the moment of an abort, in
 * milliamps. Kept so that an abort can be told apart from a marginal
 * trip: a value just over the threshold means the limit is too tight,
 * while a very large one means something genuinely went wrong. */
static volatile int32_t abort_current_ma;

/* Largest magnitude seen on any phase since the last start, in
 * milliamps. This is a single-sample maximum, so it captures transients
 * as readily as steady current -- which is precisely why the filtered
 * average below exists alongside it. */
static volatile int32_t peak_current_ma;

/* Filtered average of the phase current magnitude, in milliamps.
 *
 * Updated each period by a fraction of the difference between the new
 * sample and the running value, giving a time constant of about 8
 * milliseconds. Read against the peak, this answers the question a peak
 * alone cannot: was that high reading a moment or a state? */
static volatile int32_t average_current_ma;

/* Consecutive samples seen above the limit so far. Reset by any sample
 * below it, so only an unbroken run counts toward an abort. */
static volatile uint16_t consecutive_over_limit;

/* Current at which the loop aborts, in milliamps. Zero disables the
 * check entirely. */
static volatile int32_t abort_limit_ma = OPENLOOP_DEFAULT_CURRENT_LIMIT_MA;

/* ------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------ */

/* Magnitude of a signed value.
 *
 * @param value  any signed number
 * @return its magnitude, always zero or positive */
static inline int32_t absolute(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* ------------------------------------------------------------------
 * The control function
 *
 * Installed into the control module and called once per PWM period in
 * interrupt context, with both measured currents already converted to
 * milliamps.
 *
 * @param current_a_ma  phase A current in milliamps, signed
 * @param current_b_ma  phase B current in milliamps, signed
 * ------------------------------------------------------------------ */
static void openloop_control_step(int32_t current_a_ma, int32_t current_b_ma)
{
    if (running == 0u) {
        return;
    }

    /* Phase C has no sensor. The three phase currents of a star-connected
     * winding must sum to zero, because the star point is the only place
     * they meet and nothing leaves through it -- so C is whatever makes
     * the sum zero. */
    int32_t current_c_ma = -(current_a_ma + current_b_ma);

    /* Abort before applying anything further. Checked here rather than
     * only in the main loop because open-loop drive has nothing
     * regulating current, so it can rise sharply between two main loop
     * passes. */
    /* Largest magnitude across the three phases, which is what both the
     * abort test and the peak record care about. */
    int32_t largest = absolute(current_a_ma);
    if (absolute(current_b_ma) > largest) {
        largest = absolute(current_b_ma);
    }
    if (absolute(current_c_ma) > largest) {
        largest = absolute(current_c_ma);
    }

    if (largest > peak_current_ma) {
        peak_current_ma = largest;
    }

    /* Exponential moving average: move the running value a fixed
     * fraction of the way toward each new sample. One multiply-free
     * shift, no history buffer, and it converges on the true mean.
     *
     * The shift is applied to the difference rather than to the sample,
     * so small differences still move the average instead of being
     * rounded away to nothing. */
    average_current_ma += (largest - average_current_ma)
                          >> OPENLOOP_AVERAGE_FILTER_SHIFT;

    /* A limit of zero means the caller has switched the check off. */
    if (abort_limit_ma > 0) {

        if (largest > abort_limit_ma) {
            consecutive_over_limit++;

            /* Only an unbroken run trips. A lone sample over the limit
             * is far more likely to be noise or a switching transient
             * than a real fault, and neither damages anything. */
            if (consecutive_over_limit
                    >= OPENLOOP_ABORT_CONSECUTIVE_SAMPLES) {
                running                = 0u;
                aborted_on_overcurrent = 1u;
                abort_current_ma       = largest;
                gate_driver_disable_all();
                return;
            }
        } else {
            consecutive_over_limit = 0u;
        }
    }

    /* Advance the vector. Unsigned overflow is defined behaviour in C
     * and wraps cleanly, which is the whole point of holding an angle in
     * a 32-bit accumulator. */
    angle_accumulator += accumulator_increment;

    /* Read the volatile amplitude once, so all three phases are computed
     * from the same value even if the main loop changes it mid-call. */
    uint16_t amplitude = commanded_amplitude;

    /* Drive the whole command on the q axis, which is the axis that
     * produces torque, and nothing on d. Open loop means the angle is
     * the accumulator's, not the rotor's -- the rotor is expected to
     * follow the vector rather than the vector following the rotor.
     *
     * The amplitude is commanded as parts per thousand of the bus, so it
     * is turned into volts against the measured bus here; motor_apply_dq
     * turns it back into a duty. Going through volts means every drive
     * on this board expresses itself the same way, and the applied
     * voltage stays correct if the supply sags. */
    float bus_volts = (float)sensors_get_bus_mv() * 0.001f;
    float v_q = ((float)amplitude / (float)GATE_DRIVER_DUTY_SCALE)
                * bus_volts;

    motor_apply_dq(motor, 0.0f, v_q,
                   (float)(uint16_t)(angle_accumulator >> 16)
                       * MOTOR_ANGLE_TO_RAD,
                   current_a_ma, current_b_ma,
                   sensors_get_bus_mv());

    /* Publish the applied angle so telemetry can attach it to this
     * period's sample. The capture itself happens in the control loop,
     * which runs whether or not this module is driving.
     *
     * The encoder is published as zero because reading it over SPI takes
     * about six microseconds, which would be added to every control
     * period for a value nothing currently uses. Closed-loop control
     * will read the angle in the interrupt anyway, and can pass it
     * through here once it does. */
    telemetry_set_angles((uint16_t)(angle_accumulator >> 16), 0u);
}

/* ------------------------------------------------------------------
 * Public interface
 * ------------------------------------------------------------------ */

void openloop_init(motor_t *m)
{
    /* Remember the motor this drive commands. */
    motor = m;

    angle_accumulator      = 0u;
    accumulator_increment  = 0u;
    commanded_amplitude    = 0u;
    commanded_frequency_hz = 0u;
    running                = 0u;
    aborted_on_overcurrent = 0u;
    abort_current_ma       = 0;
    peak_current_ma        = 0;
    average_current_ma     = 0;
    consecutive_over_limit = 0u;
    abort_limit_ma         = OPENLOOP_DEFAULT_CURRENT_LIMIT_MA;

    loop_set_function(openloop_control_step);
}

/* Turn a frequency in hertz into an accumulator increment, clamping on
 * the way.
 *
 * @param frequency_hz  requested rate
 * @return the clamped rate, so the caller records what was applied
 *         rather than what was asked for */
static uint16_t apply_frequency(uint16_t frequency_hz)
{
    if (frequency_hz > OPENLOOP_FREQUENCY_CEILING) {
        frequency_hz = OPENLOOP_FREQUENCY_CEILING;
    }
    accumulator_increment = (uint32_t)frequency_hz * ACCUMULATOR_STEP_PER_HZ;
    return frequency_hz;
}

/* Clamp and store an amplitude.
 *
 * @param amplitude  requested peak, parts per thousand of bus voltage
 * @return the clamped value actually applied */
static uint16_t apply_amplitude(uint16_t amplitude)
{
    if (amplitude > OPENLOOP_AMPLITUDE_CEILING) {
        amplitude = OPENLOOP_AMPLITUDE_CEILING;
    }
    commanded_amplitude = amplitude;
    return amplitude;
}

uint8_t openloop_start(uint16_t frequency_hz, uint16_t amplitude)
{
    /* Without the control loop running, the function above would never
     * be called and the bridge would sit at whatever duty it was left
     * at -- enabled but static, which on a low resistance winding means
     * a steady DC current through one phase. */
    if (loop_is_running() == 0u) {
        return 0u;
    }

    aborted_on_overcurrent = 0u;
    abort_current_ma       = 0;
    peak_current_ma        = 0;
    average_current_ma     = 0;
    consecutive_over_limit = 0u;

    commanded_frequency_hz = apply_frequency(frequency_hz);
    (void)apply_amplitude(amplitude);

    /* Every phase starts at the resting point, so enabling cannot apply
     * a leftover duty from a previous run. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_set_duty(phase, 500u);
    }

    /* Enabling charges each bootstrap capacitor in turn, blocking for a
     * few milliseconds per phase. Done before the vector starts rotating
     * so that the first commanded voltage is actually deliverable. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_enable_phase(phase);
    }

    running = 1u;
    return 1u;
}

void openloop_stop(void)
{
    /* Clearing the flag first means the control function returns
     * immediately on its next call, so it cannot write a duty after the
     * bridge has already been disabled. */
    running = 0u;
    gate_driver_disable_all();

    /* Stop reporting the last applied angle. Left as it was, a stopped
     * stream would keep showing it as though the vector were still
     * turning. */
    telemetry_set_angles(0u, 0u);
}

void openloop_set_frequency(uint16_t frequency_hz)
{
    commanded_frequency_hz = apply_frequency(frequency_hz);
}

void openloop_set_amplitude(uint16_t amplitude)
{
    (void)apply_amplitude(amplitude);
}

uint8_t openloop_is_running(void)
{
    return running;
}

void openloop_get_state(uint16_t *frequency_out,
                        uint16_t *amplitude_out,
                        uint16_t *electrical_angle_out,
                        uint8_t  *aborted_out)
{
    if (frequency_out != NULL) {
        *frequency_out = commanded_frequency_hz;
    }
    if (amplitude_out != NULL) {
        *amplitude_out = commanded_amplitude;
    }
    if (electrical_angle_out != NULL) {
        /* Reported as 16 bits rather than 32, because the low bits are
         * fractional precision that means nothing to a reader. */
        *electrical_angle_out = (uint16_t)(angle_accumulator >> 16);
    }
    if (aborted_out != NULL) {
        *aborted_out = aborted_on_overcurrent;
    }
}

void openloop_get_currents(int32_t *peak_out, int32_t *abort_out)
{
    if (peak_out != NULL) {
        *peak_out = peak_current_ma;
    }
    if (abort_out != NULL) {
        *abort_out = abort_current_ma;
    }
}

int32_t openloop_get_average_current(void)
{
    return average_current_ma;
}

void openloop_set_current_limit(int32_t limit_ma)
{
    if (limit_ma < 0) {
        limit_ma = 0;
    }
    abort_limit_ma = limit_ma;

    /* Any run of over-limit samples counted under the old value is no
     * longer meaningful, so it starts again from zero. */
    consecutive_over_limit = 0u;
}

int32_t openloop_get_current_limit(void)
{
    return abort_limit_ma;
}
