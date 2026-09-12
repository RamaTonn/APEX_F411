#include "calibration.h"

#include "motor.h"
#include "loop.h"
#include "encoder.h"
#include "gate_driver.h"
#include "main.h"
#include "sensors.h"
#include "units.h"

/* Current above which the sweep gives up, in milliamps. Generous enough
 * that a normal calibration never approaches it, tight enough that a
 * stalled rotor is caught before anything heats. */
#define CALIBRATION_CURRENT_LIMIT_MA 4000

/* Encoder movement below which the rotor is judged not to have turned at
 * all, in counts. A tenth of a revolution: less than that and either the
 * drive is too weak or something is holding the shaft. */
#define MINIMUM_TOTAL_MOVEMENT (UNITS_COUNTS_PER_TURN / 10)

/* How far the two sweeps may disagree before the measurement is
 * rejected, as a percentage of the larger.
 *
 * They should be equal and opposite. Friction makes the rotor lag
 * whichever way it is going, which shortens both by a similar amount, so
 * a modest difference is expected. A large one means the rotor slipped
 * or stuck partway. */
#define MAXIMUM_SWEEP_DISAGREEMENT_PERCENT 20

/* Largest rounding error in the pole pair count that is still trusted,
 * in hundredths.
 *
 * The error is not symmetric, and there are two reasons for that.
 *
 * Friction makes the rotor lag the field, so it travels slightly less
 * than the field does. That makes the computed count come out HIGH.
 *
 * More significantly, the encoder's own angle is not perfectly linear
 * when the magnet is not well centred over the sensing element. That
 * error averages away over a whole mechanical revolution but not over a
 * fraction of one -- and this sweep covers about half a turn on a seven
 * pole pair motor. A magnet reading near the top of the sensor's gain
 * range, which means it is far away or off centre, makes this worse.
 *
 * Both push the same direction, so the threshold is generous rather than
 * symmetric. Forty hundredths still rejects a measurement that landed
 * near the midpoint between two counts, which is the case that actually
 * matters. The measured fractional value is reported either way, so a
 * result of 7.3 can be seen for what it is. */
#define MAXIMUM_POLE_PAIR_ERROR_PERCENT 40u

/* Total steps in one sweep. */
#define SWEEP_STEPS \
    (CALIBRATION_ELECTRICAL_REVOLUTIONS * CALIBRATION_STEPS_PER_REVOLUTION)

/* How far the applied angle advances per step, on the 16-bit scale. */
#define ANGLE_STEP (UNITS_ANGLE_SCALE / CALIBRATION_STEPS_PER_REVOLUTION)

/* ------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------ */

static int32_t absolute(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* How far the shaft moved between two encoder readings, allowing for the
 * reading wrapping at a full turn.
 *
 * A plain subtraction fails at the wrap: moving forward from 16300 to
 * 100 looks like a jump backwards of 16200 rather than a small step
 * forward of 184. Since the rotor cannot move half a turn between two
 * closely spaced samples, any apparent jump that large must be a wrap
 * the other way.
 *
 * @param previous  earlier reading, 0 to 16383
 * @param current   later reading
 * @return signed movement in counts */
static int32_t movement_between(uint16_t previous, uint16_t current)
{
    int32_t difference = (int32_t)current - (int32_t)previous;

    if (difference > (UNITS_COUNTS_PER_TURN / 2)) {
        difference -= UNITS_COUNTS_PER_TURN;
    } else if (difference < -(UNITS_COUNTS_PER_TURN / 2)) {
        difference += UNITS_COUNTS_PER_TURN;
    }
    return difference;
}

/* The motor being calibrated, for the duration of one run. */
static motor_t *motor;

/* Apply a vector at one angle, using measured current for dead time
 * correction.
 *
 * @param angle      electrical angle, 0 to 65535
 * @param amplitude  parts per thousand of bus voltage */
static void hold_vector(uint16_t angle, uint16_t amplitude)
{
    int32_t current_a;
    int32_t current_b;

    sensors_get_currents(&current_a, &current_b);

    /* The whole vector goes on the q axis. Which physical axis that is
     * depends on where electrical zero sits -- which is precisely what
     * this routine is measuring -- so the choice cancels: the offset is
     * measured with the same convention it will later be driven with.
     *
     * Amplitude arrives as parts per thousand of the bus and is turned
     * into volts against the measured bus, which motor_apply_dq turns
     * back into a duty. */
    float bus_volts = (float)sensors_get_bus_mv() * 0.001f;
    float v_q = ((float)amplitude / (float)GATE_DRIVER_DUTY_SCALE)
                * bus_volts;

    motor_apply_dq(motor, 0.0f, v_q,
                   (float)angle * MOTOR_ANGLE_TO_RAD,
                   current_a, current_b,
                   sensors_get_bus_mv());
}

/* Check the currents and note the largest seen.
 *
 * @param peak_out  updated if this reading exceeds it
 * @return 1 if the limit was exceeded */
static uint8_t current_too_high(int32_t *peak_out)
{
    int32_t current_a;
    int32_t current_b;

    sensors_get_currents(&current_a, &current_b);

    int32_t largest = absolute(current_a);
    if (absolute(current_b) > largest) {
        largest = absolute(current_b);
    }
    if (largest > *peak_out) {
        *peak_out = largest;
    }

    return (largest > CALIBRATION_CURRENT_LIMIT_MA) ? 1u : 0u;
}

/* Rotate the vector through the full sweep, tracking the shaft.
 *
 * @param start_angle    where the sweep begins, 0 to 65535
 * @param step           how far to advance each step; negative for a
 *                       reverse sweep
 * @param amplitude      drive strength
 * @param movement_out   total shaft movement in counts, signed
 * @param peak_out       largest current seen, updated as it goes
 * @return a CALIBRATION_ result code */
static uint8_t sweep(uint16_t  start_angle,
                     int32_t   step,
                     uint16_t  amplitude,
                     int32_t  *movement_out,
                     int32_t  *peak_out)
{
    uint16_t previous_reading;
    uint16_t reading;
    int32_t  total_movement = 0;
    int32_t  angle          = (int32_t)start_angle;

    if (encoder_read_angle(&previous_reading) == 0u) {
        return CALIBRATION_ERR_ENCODER;
    }

    for (uint32_t i = 0u; i < SWEEP_STEPS; i++) {

        angle += step;

        /* The cast wraps at 65536, which is exactly one electrical
         * revolution, so the angle needs no explicit reduction. */
        hold_vector((uint16_t)angle, amplitude);

        HAL_Delay(CALIBRATION_STEP_MS);

        if (current_too_high(peak_out) != 0u) {
            *movement_out = total_movement;
            return CALIBRATION_ERR_OVERCURRENT;
        }

        if (encoder_read_angle(&reading) == 0u) {
            *movement_out = total_movement;
            return CALIBRATION_ERR_ENCODER;
        }

        total_movement += movement_between(previous_reading, reading);
        previous_reading = reading;
    }

    *movement_out = total_movement;
    return CALIBRATION_OK;
}

/* Park at electrical zero and read where the shaft settled.
 *
 * @param amplitude    drive strength
 * @param reading_out  where to store the encoder reading
 * @param peak_out     largest current seen, updated
 * @return a CALIBRATION_ result code */
static uint8_t park_and_read(uint16_t  amplitude,
                             uint16_t *reading_out,
                             int32_t  *peak_out)
{
    hold_vector(0u, amplitude);
    HAL_Delay(CALIBRATION_SETTLE_MS);

    if (current_too_high(peak_out) != 0u) {
        return CALIBRATION_ERR_OVERCURRENT;
    }
    if (encoder_read_angle(reading_out) == 0u) {
        return CALIBRATION_ERR_ENCODER;
    }
    return CALIBRATION_OK;
}

/* ------------------------------------------------------------------
 * The routine
 * ------------------------------------------------------------------ */

uint8_t calibration_run(motor_t              *m,
                        uint16_t              amplitude,
                        calibration_result_t *result_out)
{
    /* Held for the duration of the run so the static helpers below can
     * reach the motor without threading it through each of them. */
    motor = m;

    uint16_t first_reading;
    uint16_t second_reading;
    int32_t  forward_movement = 0;
    int32_t  reverse_movement = 0;
    int32_t  peak_current     = 0;
    uint8_t  outcome;

    if (result_out == NULL) {
        return CALIBRATION_ERR_RANGE;
    }

    /* Start from a blank result, so a failure partway leaves zeroes
     * rather than values left over from a previous run. */
    result_out->pole_pairs              = 0u;
    result_out->offset_counts           = 0u;
    result_out->direction_forward       = 1u;
    result_out->pole_pair_error_percent = 0u;
    result_out->forward_counts          = 0;
    result_out->reverse_counts          = 0;
    result_out->offset_spread           = 0u;
    result_out->peak_current_ma         = 0;

    /* The currents this routine watches come from the control loop. With
     * it stopped they are stale, and a stall would go unnoticed. */
    if (loop_is_running() == 0u) {
        return CALIBRATION_ERR_NOT_READY;
    }

    /* Every phase starts at rest, so enabling cannot apply a duty left
     * over from something else. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_set_duty(phase, 500u);
    }
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_enable_phase(phase);
    }

    outcome = park_and_read(amplitude, &first_reading, &peak_current);
    if (outcome != CALIBRATION_OK) {
        goto finished;
    }

    outcome = sweep(0u, (int32_t)ANGLE_STEP, amplitude,
                    &forward_movement, &peak_current);
    if (outcome != CALIBRATION_OK) {
        goto finished;
    }

    outcome = sweep((uint16_t)(SWEEP_STEPS * ANGLE_STEP),
                    -(int32_t)ANGLE_STEP, amplitude,
                    &reverse_movement, &peak_current);
    if (outcome != CALIBRATION_OK) {
        goto finished;
    }

    outcome = park_and_read(amplitude, &second_reading, &peak_current);
    if (outcome != CALIBRATION_OK) {
        goto finished;
    }

    result_out->forward_counts = forward_movement;
    result_out->reverse_counts = reverse_movement;

    /* The rotor must actually have moved for anything to be measured. */
    if ((absolute(forward_movement) < MINIMUM_TOTAL_MOVEMENT)
            || (absolute(reverse_movement) < MINIMUM_TOTAL_MOVEMENT)) {
        outcome = CALIBRATION_ERR_NO_MOVEMENT;
        goto finished;
    }

    /* The two sweeps should be equal and opposite. Both being the same
     * sign means the rotor did not follow the vector at all -- it stuck,
     * or slipped a pole and settled somewhere unrelated. */
    if (((forward_movement > 0) && (reverse_movement > 0))
            || ((forward_movement < 0) && (reverse_movement < 0))) {
        outcome = CALIBRATION_ERR_INCONSISTENT;
        goto finished;
    }

    int32_t forward_size = absolute(forward_movement);
    int32_t reverse_size = absolute(reverse_movement);
    int32_t larger  = (forward_size > reverse_size) ? forward_size : reverse_size;
    int32_t smaller = (forward_size > reverse_size) ? reverse_size : forward_size;

    if (((larger - smaller) * 100) > (larger * MAXIMUM_SWEEP_DISAGREEMENT_PERCENT)) {
        outcome = CALIBRATION_ERR_INCONSISTENT;
        goto finished;
    }

    /* Direction: the forward sweep advanced the applied angle, so if the
     * encoder also counted up, increasing counts mean forward. */
    result_out->direction_forward = (forward_movement > 0) ? 1u : 0u;

    /* Pole pairs.
     *
     * The sweep covered a known number of electrical revolutions. The
     * shaft moved (forward_size + reverse_size) / 2 counts on average
     * for that. One electrical revolution therefore takes that many
     * counts divided by the revolution count, and the pole pair number
     * is how many of those fit in a mechanical turn.
     *
     * Worked in hundredths so the fractional part survives, then rounded
     * and the rounding reported. */
    int32_t average_counts = (forward_size + reverse_size) / 2;
    int32_t counts_per_electrical_revolution =
        average_counts / (int32_t)CALIBRATION_ELECTRICAL_REVOLUTIONS;

    if (counts_per_electrical_revolution <= 0) {
        outcome = CALIBRATION_ERR_NO_MOVEMENT;
        goto finished;
    }

    int32_t pole_pairs_hundredths =
        (UNITS_COUNTS_PER_TURN * 100) / counts_per_electrical_revolution;

    int32_t rounded = (pole_pairs_hundredths + 50) / 100;

    if ((rounded < (int32_t)MOTOR_MIN_POLE_PAIRS)
            || (rounded > (int32_t)MOTOR_MAX_POLE_PAIRS)) {
        result_out->pole_pairs = (uint8_t)((rounded < 0) ? 0 : rounded);
        outcome = CALIBRATION_ERR_RANGE;
        goto finished;
    }

    /* How far the measurement had to be rounded, in hundredths. Zero
     * means it landed exactly on a whole number. */
    int32_t rounding_error = pole_pairs_hundredths - (rounded * 100);
    result_out->pole_pair_error_percent = (uint16_t)absolute(rounding_error);
    result_out->pole_pairs              = (uint8_t)rounded;
    result_out->pole_pairs_hundredths   = (uint16_t)pole_pairs_hundredths;

    if (result_out->pole_pair_error_percent > MAXIMUM_POLE_PAIR_ERROR_PERCENT) {
        outcome = CALIBRATION_ERR_INCONSISTENT;
        goto finished;
    }

    /* Offset: the rotor was parked at electrical zero twice, once before
     * the sweeps and once after. Friction leaves it slightly short of
     * true alignment on each occasion, and from opposite sides, so the
     * midpoint is closer to the truth than either. */
    int32_t between = movement_between(first_reading, second_reading);
    int32_t midpoint = (int32_t)first_reading + (between / 2);

    if (midpoint < 0) {
        midpoint += UNITS_COUNTS_PER_TURN;
    }
    midpoint %= UNITS_COUNTS_PER_TURN;

    result_out->offset_counts = (uint16_t)midpoint;
    result_out->offset_spread = (uint16_t)absolute(between);

    /* Apply the results, so closed-loop control can start straight away
     * with nothing further to configure. Only done on full success --
     * a failed run leaves whatever was configured before, rather than
     * replacing it with a measurement known to be bad. */
    (void)motor_set_pole_pairs(motor, result_out->pole_pairs);
    encoder_set_offset(motor->encoder, result_out->offset_counts);
    encoder_set_direction(motor->encoder, result_out->direction_forward);

    outcome = CALIBRATION_OK;

finished:
    gate_driver_disable_all();
    result_out->peak_current_ma = peak_current;
    return outcome;
}

const char *calibration_result_text(uint8_t result)
{
    switch (result) {
        case CALIBRATION_OK:               return "ok";
        case CALIBRATION_ERR_NO_MOVEMENT:  return "rotor_did_not_turn";
        case CALIBRATION_ERR_INCONSISTENT: return "sweeps_disagreed";
        case CALIBRATION_ERR_ENCODER:      return "encoder_read_failed";
        case CALIBRATION_ERR_OVERCURRENT:  return "overcurrent";
        case CALIBRATION_ERR_NOT_READY:    return "control_loop_not_running";
        case CALIBRATION_ERR_RANGE:        return "result_out_of_range";
        default:                           return "unknown";
    }
}
