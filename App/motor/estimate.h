/*
 * estimate.h
 *
 * Measures the motor's electrical properties: phase resistance, and the
 * two inductances.
 *
 * THE WINDING IS DRIVEN DIRECTLY, NOT THROUGH THE dq TRANSFORM
 *
 *   Every measurement here drives the bridge directly -- one phase at a
 *   chosen duty, the others grounded or floating -- and reads the
 *   current sensor on whichever phase is carrying it. No Park or Clarke
 *   transform, no electrical angle, no dead-time compensation loop.
 *
 *   That is deliberate, and it is the whole design. Resistance and
 *   inductance are properties of copper and iron; none of the
 *   rotating-frame machinery is needed to measure them, and all of it
 *   gets in the way. An earlier version of this file drove a q-axis
 *   vector through motor_apply_dq() at a fixed electrical angle and read
 *   one phase's current back. Three separate things then had to be right
 *   before the number meant anything: the balanced three-phase output
 *   put no voltage at all on the phase being measured at that angle, the
 *   rotor pulled itself around under the torque the vector produced, and
 *   the dead-time correction fed the measured current back into the
 *   applied voltage. Driving one phase directly removes all three at
 *   once, and what is left is Ohm's law.
 *
 * THE TWO TOPOLOGIES
 *
 *   Phase A driven, B and C grounded
 *       Current leaves A and returns through B and C in parallel, so the
 *       measurement sees R + R/2 = 1.5 R, and likewise 1.5 L. The field
 *       points along phase A's own axis.
 *
 *   Phase A floating, B driven, C grounded
 *       Current flows B to C and nowhere else, so the measurement sees
 *       two phases in series: 2 R and 2 L. The field points ninety
 *       electrical degrees from phase A's axis.
 *
 *   Those two field directions are what separate Ld from Lq once the
 *   rotor has been pulled into line with phase A. Each has its own
 *   geometry factor, applied where the result is computed.
 *
 * WHY RESISTANCE TAKES TWO POINTS
 *
 *   The dead time removes a fixed slice of voltage before any of it
 *   reaches the winding -- about 24 parts per thousand of the bus on
 *   this board, which is more than the whole command at the duties this
 *   measurement uses. At a single operating point that loss is
 *   indistinguishable from resistance and gets charged to it; an early
 *   single-point version reported 186 milliohms for a 36 milliohm
 *   winding, almost all of it dead time.
 *
 *   Between two points at the same current polarity the loss is
 *   identical, so it cancels in the difference and leaves only the
 *   winding. The same trick covers the d-axis inductance pulse, which
 *   steps between two duties rather than up from zero.
 */

#ifndef ESTIMATE_H_
#define ESTIMATE_H_

#include <stdint.h>

#include "motor.h"

/* Control periods the measuring pulse lasts.
 *
 * Long enough that the current change is well above the sensor's
 * resolution, short enough that resistance has not yet taken over from
 * inductance. Eight periods is a quarter of a millisecond. */
#define ESTIMATE_PULSE_PERIODS 8U

/* How far the current must move for a pulse to count, in milliamps.
 *
 * The sensor resolves about 40 milliamps per count and carries a few
 * counts of noise. Below this the measurement would be mostly noise, so
 * the pulse is repeated at a higher voltage instead. */
#define ESTIMATE_MINIMUM_CURRENT_CHANGE_MA 400

/* Current at which a measurement is abandoned, in milliamps. */
#define ESTIMATE_CURRENT_LIMIT_MA 6000

/* How long the current must stay continuously above that limit before a
 * measurement gives up, in milliseconds.
 *
 * A single reading over the limit is a poor basis for abandoning a run,
 * for the reasons openloop.c spells out at
 * OPENLOOP_ABORT_CONSECUTIVE_SAMPLES: the measurement carries a few
 * counts of noise, and a one-off switching transient is not what damages
 * anything -- sustained current is. Two milliseconds is far too short
 * for a winding or a transistor to heat appreciably, so nothing is given
 * up by waiting that long to be sure. */
#define ESTIMATE_OVERCURRENT_SUSTAIN_MS 2U

/* Results. */
#define ESTIMATE_OK               0U
#define ESTIMATE_ERR_NOT_READY    1U  /* control loop is not running    */
#define ESTIMATE_ERR_TOO_SMALL    2U  /* current never moved enough     */
#define ESTIMATE_ERR_OVERCURRENT  3U
#define ESTIMATE_ERR_ARGUMENT     4U

/* What a resistance measurement produced. */
typedef struct {
    uint32_t resistance_mohm;

    /* The two operating points the slope was taken between. Reported so
     * the answer can be recomputed by hand: the resistance is the
     * voltage difference between them over the current difference,
     * times two thirds for the return path through B and C. */
    uint16_t low_duty;
    uint16_t high_duty;
    int32_t  low_current_ma;
    int32_t  high_current_ma;

    /* Bus voltage while current was flowing, in millivolts. Read under
     * load rather than before it, since the supply sags once the
     * measurement starts drawing amps. */
    uint32_t bus_mv;

    /* Where a run gave up, when one did: the duty being applied and the
     * current there. Both zero on a successful run.
     *
     * Reported on the error reply as well, because a reason on its own
     * does not say whether the drive never got going or ran away, and
     * those want opposite fixes. */
    uint16_t fault_duty;
    int32_t  fault_current_ma;
} estimate_resistance_result_t;

/* What an inductance measurement produced. */
typedef struct {
    uint32_t inductance_d_uh;
    uint32_t inductance_q_uh;

    /* The ratio between them, as a percentage. Under about 110 means
     * high-frequency injection will not work reliably on this motor,
     * since there is too little difference between the axes to detect. */
    uint32_t saliency_percent;

    /* Current change seen on each pulse, in milliamps. Reported because
     * a small change means a noisy measurement even when it passed the
     * threshold. */
    int32_t  d_current_change_ma;
    int32_t  q_current_change_ma;

    /* The pulse duty that was needed, in parts per thousand. */
    uint16_t duty_used;
} estimate_inductance_result_t;

/**
 * Measure the per-phase resistance.
 *
 * Ramps the duty on phase A, with B and C grounded, until the current
 * reaches a first target, then continues to a second, and takes the
 * slope between them -- which cancels the fixed voltage the dead time
 * removes. See the notes at the top of this file.
 *
 * The rotor is pulled into line with phase A by the current and must be
 * free enough to get there; it stays put once it has, since the field
 * does not move during the measurement.
 *
 * @param m           the motor, whose resistance is updated on success
 * @param result_out  where the measurement is written, including where
 *                    it stopped if it failed
 * @return one of the ESTIMATE_ constants
 */
uint8_t estimate_resistance(motor_t *m,
                            estimate_resistance_result_t *result_out);

/**
 * Measure both inductances.
 *
 * Holds the rotor in line with phase A, then steps the voltage along
 * that axis for Ld, and along the axis ninety electrical degrees away
 * for Lq, measuring how fast the current climbs in each.
 *
 * @param m           the motor, whose inductances are updated on success
 * @param result_out  where the measurements are written
 * @return one of the ESTIMATE_ constants
 */
uint8_t estimate_inductance(motor_t *m,
                            estimate_inductance_result_t *result_out);

/**
 * Turn a result code into a short word for a reply.
 *
 * @param result  one of the ESTIMATE_ constants
 * @return a static string, never NULL
 */
const char *estimate_result_text(uint8_t result);

#endif /* ESTIMATE_H_ */
