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
 * THE TOPOLOGIES
 *
 *   Two phases in series, the third floating
 *       Current flows from the driven phase to the grounded one and
 *       nowhere else, so the path is exactly two windings: R_x + R_y and
 *       L_x + L_y. This is what the resistance measurement uses, three
 *       times over, and what the q-axis inductance pulse uses.
 *
 *   One phase driven, the other two grounded
 *       Current leaves the driven phase and returns through the other
 *       two in parallel, so the path is R + R/2 = 1.5 R. The field points
 *       along the driven phase's own axis, which is what the d-axis
 *       inductance pulse needs.
 *
 *   Those two field directions are what separate Ld from Lq once the
 *   rotor has been pulled into line with phase A. Each carries its own
 *   geometry factor, applied where the result is computed: a field along
 *   phase A's axis links 1.5 Ld at the terminals, and one driven B to C
 *   with A floating links 2 Lq.
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
 *   winding. The same trick covers the inductance pulses, which step
 *   between two duties rather than up from zero.
 *
 * WHY INDUCTANCE NEEDS THE RESISTANCE
 *
 *   The textbook L = V t / dI holds only while the pulse is far shorter
 *   than the winding's own L/R time constant. On this board it is not:
 *   the pulse is a quarter of a millisecond and the time constant is
 *   about the same, so the current is already well into its exponential
 *   bend by the time it is read.
 *
 *   Two things go wrong if that is ignored. The inductance comes out
 *   high -- by about sixty percent, on the motor this was developed
 *   against -- because the current rose less than a straight line
 *   through the origin would predict. Worse, the answer stops depending
 *   on the inductance at all as the pulse lengthens: the current settles
 *   towards V/R, which is the same on both axes, so Ld and Lq converge
 *   and the saliency reads 100 percent whatever the rotor is actually
 *   built like. That is exactly the failure this file was rewritten to
 *   remove.
 *
 *   The honest inversion is the exponential one,
 *
 *       L = -t R / ln(1 - dI R / V)
 *
 *   which is exact at any pulse length and reduces to V t / dI when the
 *   pulse is short. It needs R, so estimate_inductance() refuses to run
 *   until estimate_resistance() has supplied one.
 */

#ifndef ESTIMATE_H_
#define ESTIMATE_H_

#include <stdint.h>

#include "motor.h"

/* Control periods the measuring pulse lasts. Eight is a quarter of a
 * millisecond.
 *
 * Long enough that the current change is well above the sensor's
 * resolution. There is no upper bound from the arithmetic, since the
 * inversion is exact at any length -- but there is one from the
 * conditioning: once the current has all but settled, the answer stops
 * depending on the inductance and a couple of counts of noise move it
 * a long way. PULSE_SETTLED_FRACTION_LIMIT in estimate.c is where that
 * is caught. A quarter of a millisecond leaves a motor this board
 * drives around sixty percent settled, which is comfortably short of
 * it. */
#define ESTIMATE_PULSE_PERIODS 8U

/* How far the current must move for a pulse to count, in milliamps.
 *
 * The sensor resolves about 40 milliamps per count, and the change is a
 * difference between two single readings, so it carries the quantisation
 * of both. At this threshold that is around five percent, and the pulse
 * step is raised until it is met -- so the number is really a floor on
 * the precision of the answer rather than a validity check.
 *
 * It sets the operating point across a range of boards, not just this
 * one: the current change scales with the bus and with the step
 * together, so demanding a fixed change picks a small step on a high
 * voltage bus and a large one on a low voltage bus, and arrives at the
 * same signal-to-noise either way.
 *
 * The resistance measurement checks its own two-point change against
 * this as well, where it is a much weaker condition: those two points
 * are averages over twenty milliseconds each and are aimed two amps
 * apart, so failing it means something went wrong rather than that the
 * answer is merely coarse. */
#define ESTIMATE_MINIMUM_CURRENT_CHANGE_MA 800

/* How many times each measuring pulse is repeated and averaged.
 *
 * Four readings of the change instead of one, which halves what the
 * sensor's quantisation contributes. Kept small because the across-axis
 * pulse runs under full torque: four pulses and their recovery dwells
 * come to about five milliseconds, which is far too short for the rotor
 * to move anywhere that would matter. */
#define ESTIMATE_PULSE_REPEATS 4U

/* How long the current is given to fall back to its holding value
 * between repeated pulses, in milliseconds. Several L/R time constants
 * on any winding this board would drive. */
#define ESTIMATE_PULSE_RECOVERY_MS 1U

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
#define ESTIMATE_OK                  0U
#define ESTIMATE_ERR_NOT_READY       1U  /* control loop is not running   */
#define ESTIMATE_ERR_TOO_SMALL       2U  /* current never moved enough    */
#define ESTIMATE_ERR_OVERCURRENT     3U
#define ESTIMATE_ERR_ARGUMENT        4U
#define ESTIMATE_ERR_NEED_RESISTANCE 5U  /* run the resistance first     */
#define ESTIMATE_ERR_PULSE_TOO_LONG  6U  /* current reached its ceiling   */

/* Which line-to-line pair a resistance run was on when it stopped. */
#define ESTIMATE_PAIR_AB   0U
#define ESTIMATE_PAIR_BC   1U
#define ESTIMATE_PAIR_CA   2U
#define ESTIMATE_PAIR_NONE 255U

/* What a resistance measurement produced.
 *
 * Three line-to-line measurements are taken -- A against B, B against C,
 * C against A, each with the third phase floating -- and the per-phase
 * values solved from them. See estimate_resistance() for why that
 * topology rather than one phase against the other two. */
typedef struct {
    /* Line to line, each measured directly. */
    uint32_t ab_mohm;
    uint32_t bc_mohm;
    uint32_t ca_mohm;

    /* Solved per phase: R_a = (R_ab + R_ca - R_bc) / 2, and so round. */
    uint32_t a_mohm;
    uint32_t b_mohm;
    uint32_t c_mohm;

    /* The mean of the three, which is what the motor is configured with,
     * since the control loop wants one number. */
    uint32_t phase_mohm;

    /* Spread between the highest and lowest phase, as a percentage of
     * the mean. A few percent is ordinary; a large number means either
     * a genuinely unbalanced winding or a bad connection on one phase. */
    uint32_t imbalance_percent;

    /* Bus voltage while current was flowing, in millivolts. Read under
     * load rather than before it, since the supply sags once the
     * measurement starts drawing amps. */
    uint32_t bus_mv;

    /* Where a run gave up, when one did: which pair was being measured,
     * the duty being applied, and the current there. The pair is
     * ESTIMATE_PAIR_NONE and the rest zero on a successful run.
     *
     * Which pair matters: a single phase with a bad connection fails two
     * of the three runs and passes the one that does not touch it, so
     * the pair that stopped names the suspect directly. */
    uint8_t  fault_pair;
    uint16_t fault_duty;
    int32_t  fault_current_ma;
} estimate_resistance_result_t;

/* What an inductance measurement produced. */
typedef struct {
    /* Nanohenries rather than microhenries, because a motor this board
     * drives has single-figure microhenries per phase and an integer
     * microhenry would be a ten percent quantisation on the answer --
     * which lands directly on the current loop's proportional gain,
     * since that is L times the bandwidth. */
    uint32_t inductance_d_nh;
    uint32_t inductance_q_nh;

    /* The ratio between them, as a percentage. Under about 110 means
     * high-frequency injection will not work reliably on this motor,
     * since there is too little difference between the axes to detect.
     *
     * A surface-magnet rotor genuinely has very little saliency, so a
     * number near 100 can be the honest answer rather than a failed
     * measurement. */
    uint32_t saliency_percent;

    /* Current change seen on each pulse, in milliamps. Reported because
     * a small change means a noisy measurement even when it passed the
     * threshold. */
    int32_t  d_current_change_ma;
    int32_t  q_current_change_ma;

    /* The pulse step each axis needed, in parts per thousand. Reported
     * separately because the two are chosen independently -- the axis
     * with more inductance takes longer to reach the same current
     * change and can need a larger step to get there. */
    uint16_t d_duty_used;
    uint16_t q_duty_used;

    /* The series resistance each axis was solved against, in milliohms.
     * Reported because the answer depends on it -- see
     * estimate_inductance() on why resistance has to be known first. */
    uint32_t d_series_mohm;
    uint32_t q_series_mohm;
} estimate_inductance_result_t;

/**
 * Measure the resistance of each of the three phases.
 *
 * Runs three line-to-line measurements -- A driven against B, B against
 * C, C against A, with the remaining phase floating each time -- and
 * solves the per-phase values from them.
 *
 * Line to line rather than one phase against the other two, because that
 * is the topology whose answer can actually be solved. One phase against
 * the other two measures R_a + (R_b || R_c), and three of those give
 * three nonlinear equations that do not separate cleanly. Two phases in
 * series measures R_a + R_b, so the three runs give
 *
 *     R_ab = R_a + R_b,  R_bc = R_b + R_c,  R_ca = R_c + R_a
 *
 * which inverts in one line: R_a = (R_ab + R_ca - R_bc) / 2, and so
 * round. It also means every pair is read by a real current sensor --
 * phase C has none, and a C current derived from the other two carries
 * both their errors.
 *
 * Each pair ramps its duty until the current reaches a first target,
 * continues to a second, and takes the slope between them, which cancels
 * the fixed voltage the dead time removes. See the notes at the top of
 * this file.
 *
 * The rotor is pulled about by the successive field directions and must
 * be free enough to follow; it stays put within each measurement, since
 * the field does not move while a pair is being ramped.
 *
 * @param m           the motor, whose resistance is set to the mean of
 *                    the three phases on success
 * @param result_out  where the measurement is written, including where
 *                    it stopped if it failed
 * @return one of the ESTIMATE_ constants
 */
uint8_t estimate_resistance(motor_t *m,
                            estimate_resistance_result_t *result_out);

/**
 * Measure both inductances, independently of one another.
 *
 * Holds the rotor in line with phase A, then steps the voltage along
 * that axis and reads how fast the current climbs, which gives Ld; then
 * steps it along the axis ninety electrical degrees away, which gives
 * Lq. Neither is assumed from the other, and the difference between them
 * is what the reported saliency is.
 *
 * Requires estimate_resistance() to have run first -- see the note at
 * the top of this file on why the inversion needs R -- and returns
 * ESTIMATE_ERR_NEED_RESISTANCE if it has not.
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

/**
 * Turn a line-to-line pair index into a short word for a reply.
 *
 * @param pair  ESTIMATE_PAIR_AB, _BC, _CA or _NONE
 * @return a static string, never NULL
 */
const char *estimate_pair_text(uint8_t pair);

#endif /* ESTIMATE_H_ */
