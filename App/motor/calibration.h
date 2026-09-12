/*
 * calibration.h
 *
 * Works out the three motor constants automatically, in one command,
 * with no interpretation required from whoever runs it.
 *
 * WHAT IT MEASURES
 *
 *   DIRECTION    whether the encoder counts up when the windings turn
 *                the rotor forward. Depends on how the magnet was
 *                oriented and which way round the motor leads went on.
 *
 *   POLE PAIRS   how many electrical cycles the windings go through for
 *                one turn of the shaft. A property of the rotor magnets,
 *                countable but not derivable.
 *
 *   OFFSET       where the encoder's zero sits relative to the rotor's
 *                magnetic axis. Depends on how the magnet was glued on,
 *                so it is specific to this motor and this sensor.
 *
 * HOW
 *
 *   1. A current vector is applied at electrical zero and the rotor is
 *      allowed to settle there. The encoder is read: that is the offset,
 *      provisionally.
 *
 *   2. The vector is rotated slowly forward through a whole number of
 *      electrical revolutions, dragging the rotor with it, while the
 *      encoder is tracked and unwrapped.
 *
 *      How far the shaft moved for a known number of electrical
 *      revolutions gives the pole pair count. Which way it moved gives
 *      the direction.
 *
 *   3. The same sweep is run backwards.
 *
 *      Friction makes the rotor lag whichever way it is going, so a
 *      single sweep is biased. Averaging the two cancels most of it.
 *
 *   4. The vector returns to electrical zero and the encoder is read
 *      again. Averaging with the first reading cancels the friction bias
 *      in the offset the same way.
 *
 * REQUIREMENTS
 *
 *   The rotor must be free to turn through at least one full revolution.
 *   A load, a gearbox, or a stop will make the sweep slip, and slipping
 *   produces a pole pair count that is simply wrong rather than one that
 *   fails obviously. The confidence figure reported afterwards is what
 *   catches this.
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <stdint.h>

#include "motor.h"

/* Electrical revolutions swept in each direction.
 *
 * More revolutions average out cogging and improve the pole pair
 * estimate, but take longer and need more free rotation: with 14 pole
 * pairs, four electrical revolutions is under a third of a mechanical
 * turn, while with one pole pair it is four full turns. Four is a
 * compromise that suits both ends. */
#define CALIBRATION_ELECTRICAL_REVOLUTIONS 4U

/* Steps per electrical revolution during the sweep. Finer steps make the
 * rotor follow more smoothly; coarser ones risk it snapping between
 * positions and overshooting. */
#define CALIBRATION_STEPS_PER_REVOLUTION 64U

/* Milliseconds per step. The rotor must have time to reach each position
 * before the vector moves on, or it lags progressively and the sweep
 * measures the lag rather than the rotation. */
#define CALIBRATION_STEP_MS 5U

/* How long to let the rotor settle when parked at electrical zero. Long
 * enough for the ringing after it snaps into place to die away. */
#define CALIBRATION_SETTLE_MS 600U

/* Results. */
#define CALIBRATION_OK               0U
#define CALIBRATION_ERR_NO_MOVEMENT  1U  /* rotor never turned          */
#define CALIBRATION_ERR_INCONSISTENT 2U  /* the two sweeps disagreed    */
#define CALIBRATION_ERR_ENCODER      3U  /* sensor would not answer     */
#define CALIBRATION_ERR_OVERCURRENT  4U  /* current exceeded the limit  */
#define CALIBRATION_ERR_NOT_READY    5U  /* control loop is not running */
#define CALIBRATION_ERR_RANGE        6U  /* result outside sane bounds  */

/* Everything the routine worked out. */
typedef struct {
    uint8_t  pole_pairs;
    uint16_t offset_counts;
    uint8_t  direction_forward;

    /* How well the pole pair count fits.
     *
     * The measurement gives a fractional number that is then rounded.
     * This reports how far it had to be rounded, in hundredths: 0 means
     * it landed exactly on a whole number, 50 means it fell halfway
     * between two and one of them was chosen arbitrarily.
     *
     * Anything above about 15 means the rotor was not following the
     * vector faithfully, and the count should not be trusted. */
    uint16_t pole_pair_error_percent;

    /* The pole pair count before rounding, in hundredths.
     *
     * Reported so the raw measurement can be judged rather than only its
     * verdict: 730 says the motor is almost certainly seven pole pairs
     * measured with a three percent bias, while 750 would say the
     * measurement genuinely cannot tell seven from eight. */
    uint16_t pole_pairs_hundredths;

    /* Encoder counts the shaft moved during each sweep, signed. Equal
     * and opposite is what a clean measurement looks like; a large
     * difference means friction or slipping. */
    int32_t  forward_counts;
    int32_t  reverse_counts;

    /* Difference between the offset measured before and after the
     * sweeps, in encoder counts. Small means the rotor returned to where
     * it started and the offset can be trusted. */
    uint16_t offset_spread;

    /* Largest current seen at any point, in milliamps. */
    int32_t  peak_current_ma;
} calibration_result_t;

/**
 * Measure all three constants and apply them.
 *
 * Takes several seconds. Blocks throughout, because it must step the
 * vector and wait for the rotor at each step -- there is nothing useful
 * to do in between.
 *
 * On success the results are applied to the motor, so
 * closed-loop control can be started immediately afterwards with nothing
 * further to configure.
 *
 * On failure nothing is changed, leaving whatever was configured before
 * intact rather than replacing it with a bad measurement.
 *
 * The bridge is disabled before returning, whatever the outcome.
 *
 * @param amplitude   drive strength, in parts per thousand of the bus
 *                    voltage. On a low resistance winding this is single
 *                    digits: each unit is worth roughly 330 milliamps on
 *                    a 36 milliohm motor at 12 volts. Enough to turn the
 *                    rotor against its own cogging, no more.
 * @param result_out  where the measurements are written. Filled in as
 *                    far as the routine got, even on failure, so a
 *                    partial result can still be inspected.
 * @return one of the CALIBRATION_ constants
 */
uint8_t calibration_run(motor_t             *m,
                        uint16_t             amplitude,
                        calibration_result_t *result_out);

/**
 * Turn a result code into a short word for a reply.
 *
 * @param result  one of the CALIBRATION_ constants
 * @return a static string, never NULL
 */
const char *calibration_result_text(uint8_t result);

#endif /* CALIBRATION_H_ */
