/*
 * estimate.h
 *
 * Measures the motor's electrical properties: resistance, and the two
 * inductances.
 *
 * HOW INDUCTANCE IS MEASURED
 *
 *   Inductance resists a change in current. Apply a step of voltage and
 *   the current climbs at a rate the inductance sets:
 *
 *       inductance = voltage * time / current change
 *
 *   So a known voltage is applied for a known number of control periods,
 *   the current is read before and after, and the arithmetic follows.
 *
 *   The measurement must be brief. Over a long pulse the resistance
 *   starts to dominate and the current levels off, at which point the
 *   rise no longer says anything about inductance. A few control periods
 *   is short enough that the resistive drop is a small correction rather
 *   than the whole answer.
 *
 * WHY TWO INDUCTANCES
 *
 *   A rotor's magnets present a different magnetic path along their axis
 *   than across it, so the winding's inductance depends on where the
 *   rotor is. The value along the magnet axis is called Ld, the value
 *   across it Lq.
 *
 *   The difference is what makes position estimation by high-frequency
 *   injection possible: the current response to an injected signal
 *   varies with rotor position only because the inductance does. With no
 *   difference there is nothing to detect.
 *
 *   So both are measured, and the ratio between them decides whether
 *   injection is viable on this motor at all.
 *
 * HOW THE TWO ARE SEPARATED
 *
 *   The rotor is first pulled to a known position by a steady current,
 *   which puts its magnet axis along phase A.
 *
 *   The d-axis measurement then pulses along that same direction. The
 *   pulse pushes the rotor no further, because it is already aligned
 *   with it.
 *
 *   The q-axis measurement pulses ninety electrical degrees away. That
 *   direction does produce torque, but the pulse lasts under a fifth of
 *   a millisecond and the rotor's inertia means it barely moves in that
 *   time.
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

/* Current at which a pulse is abandoned, in milliamps. */
#define ESTIMATE_CURRENT_LIMIT_MA 6000

/* How long the current must stay continuously above that limit before a
 * measurement gives up, in milliseconds.
 *
 * A single reading over the limit is a poor basis for abandoning a run,
 * for the same reasons openloop.c spells out at
 * OPENLOOP_ABORT_CONSECUTIVE_SAMPLES: the measurement carries a few
 * counts of noise, and a one-off switching transient is not what damages
 * anything -- sustained current is. Two milliseconds is far too short
 * for a winding or a transistor to heat appreciably, so nothing is given
 * up by waiting that long to be sure, and every spurious trip goes away.
 *
 * The tick has one millisecond of resolution, so the real threshold lies
 * somewhere between one and two milliseconds. Both are safe. */
#define ESTIMATE_OVERCURRENT_SUSTAIN_MS 2U

/* How long the rotor is held before a measurement, in milliseconds. */
#define ESTIMATE_HOLD_MS 500U

/* Results. */
#define ESTIMATE_OK               0U
#define ESTIMATE_ERR_NOT_READY    1U  /* control loop is not running    */
#define ESTIMATE_ERR_TOO_SMALL    2U  /* current never moved enough     */
#define ESTIMATE_ERR_OVERCURRENT  3U
#define ESTIMATE_ERR_ARGUMENT     4U

/* What an inductance measurement produced. */
typedef struct {
    uint32_t inductance_d_uh;
    uint32_t inductance_q_uh;

    /* The ratio between them, as a percentage. See
     * the estimate_inductance_result_t field above for how to read it. */
    uint32_t saliency_percent;

    /* Current change seen on each pulse, in milliamps. Reported because
     * a small change means a noisy measurement even when it passed the
     * threshold. */
    int32_t  d_current_change_ma;
    int32_t  q_current_change_ma;

    /* The duty that was needed, in parts per thousand. */
    uint16_t duty_used;
} estimate_inductance_result_t;

/* What a resistance measurement produced. */
typedef struct {
    uint32_t resistance_mohm;

    /* The two operating points the measurement was taken from. Two are
     * used rather than one because a single point cannot separate the
     * winding's resistance from the fixed voltage the dead time loses --
     * the slope between two points cancels anything constant. */
    uint16_t low_duty;
    uint16_t high_duty;
    int32_t  low_current_ma;
    int32_t  high_current_ma;

    /* Where a run gave up, when one did: the duty being applied and the
     * largest phase current seen there, in milliamps. Both zero on a
     * successful run.
     *
     * Reported on the error reply as well as the success one, because
     * "overcurrent" or "current_change_too_small" on its own says
     * nothing about whether the drive never got going or ran away, and
     * those want opposite fixes. */
    uint16_t fault_duty;
    int32_t  fault_current_ma;
} estimate_resistance_result_t;

/**
 * Measure the per-phase resistance.
 *
 * Applies two steady currents and takes the slope between them, which
 * cancels the fixed voltage error the dead time introduces. A single
 * point cannot: it attributes that lost voltage to the winding and
 * reports a resistance several times too high.
 *
 * The rotor is held still by the applied current, and must be free
 * enough to reach that position.
 *
 * @param result_out  where the measurement is written
 * @return one of the ESTIMATE_ constants
 */
uint8_t estimate_resistance(motor_t *m,
                            estimate_resistance_result_t *result_out);

/**
 * Measure both inductances.
 *
 * Holds the rotor aligned, then pulses along and across its magnet axis,
 * raising the pulse voltage until the current change is large enough to
 * measure.
 *
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
