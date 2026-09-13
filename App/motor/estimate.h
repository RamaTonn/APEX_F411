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
 * HOW INDUCTANCE IS MEASURED
 *
 *   Not by stepping the voltage once and dividing. L = V t / dI holds
 *   only while the step is far shorter than the winding's own L/R time
 *   constant, and on this board it is not -- the shortest step the
 *   control loop can time is a quarter of a millisecond and the time
 *   constant is about the same. A version of this file did that, needed
 *   the resistance to correct for it, and still reported the two axes
 *   as identical, because a step that has half settled tends towards
 *   V/R, which is the same on both axes.
 *
 *   Instead the duty is SQUARED about a steady holding current -- up by
 *   a step, down by a step, over and over -- and the current's slope is
 *   measured in each half. The winding obeys
 *
 *       L di/dt = v - R i - e
 *
 *   with e whatever the rotor's motion induces. Subtracting the falling
 *   slope from the rising one leaves
 *
 *       L (di/dt|up - di/dt|down) = 2 V_step
 *
 *   and nothing else. The resistive drop, the dead time, the back EMF
 *   and any fixed offset in the sensor are all identical in the two
 *   halves and subtract away. So the inductance needs no resistance
 *   measurement, no dead-time figure, and no assumption that the rotor
 *   held still -- only the bus voltage, the step, and a clock.
 *
 *   Two conditions make that exact rather than approximate, and both are
 *   arranged for. The oscillation must be steady, so that the current
 *   has the same average in both halves; a few warm-up cycles see to
 *   that. And the current must not cross zero, since the dead time
 *   reverses with it; the excitation swings about a hold several times
 *   larger than the swing.
 */

#ifndef ESTIMATE_H_
#define ESTIMATE_H_

#include <stdint.h>

#include "motor.h"

/* How far the current must move for a resistance point to count, in
 * milliamps.
 *
 * The two points the slope is taken between are twenty-millisecond
 * averages aimed two amps apart, so falling short of this means
 * something went wrong rather than that the answer is merely coarse.
 *
 * The inductance measurement has its own threshold, on the difference
 * between its two slopes, in estimate.c. */
#define ESTIMATE_MINIMUM_CURRENT_CHANGE_MA 800

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

    /* Bus voltage at the lower of the two points, in millivolts. Read
     * under load rather than before it, since the supply sags once the
     * measurement starts drawing amps. */
    uint32_t bus_mv;

    /* How much further the bus had sagged by the upper point, worst of
     * the three pairs. Subtracted from the answer rather than charged to
     * the winding, so this is reported rather than corrected for -- but
     * a large number says the supply, not the motor, is what limits how
     * repeatable the measurement can be. */
    int32_t  sag_mv;

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

    /* How far apart the rising and falling slopes came out on each axis,
     * in milliamps across the sampling span. This is the measurement --
     * the inductance is just this turned the right way up -- so it is
     * reported for the same reason a raw count is: a number close to the
     * threshold means a noisy answer even though it passed.
     *
     * It is also the direct check on saliency being real. The two axes
     * are driven at the same step, so a genuine difference in inductance
     * shows here as a difference in slope; if these two are equal the
     * saliency is 100 percent because the motor says so. */
    int32_t  d_difference_ma;
    int32_t  q_difference_ma;

    /* The excitation step each axis needed, in parts per thousand.
     * Chosen independently -- the axis with more inductance gives a
     * shallower slope at the same step and can need a larger one. */
    uint16_t d_duty_used;
    uint16_t q_duty_used;

    /* Each axis's L/R time constant in microseconds, fitted from the
     * shape of its own response rather than assumed.
     *
     * Reported because it is an independent check on the resistance:
     * R = L / tau, worked out from this measurement alone, should agree
     * with what estimate_resistance() found by an entirely different
     * route. A zero means the winding was too slow for the curvature to
     * be fitted, and the straight-line relation was used -- which is the
     * right answer in that case, not a failure.
     *
     * The two axes have different time constants only because they have
     * different inductances; the resistance is the same for both. */
    uint32_t d_tau_us;
    uint32_t q_tau_us;

    /* The per-phase resistance each axis implies, R = L / tau, in
     * milliohms.
     *
     * This measurement shares nothing with estimate_resistance(): it
     * draws almost no current from the supply, since every phase sits
     * near half duty and the holding current circulates between the
     * windings rather than coming in from the rail. So where the two
     * disagree, this is the one to believe.
     *
     * The two axes should also agree with EACH OTHER. A winding has one
     * resistance whatever its inductance, so if these differ by much,
     * something axis-dependent is in the measurement that should not be
     * -- the likeliest being a rotor that did not stay where it was
     * put. */
    uint32_t d_mohm;
    uint32_t q_mohm;

    /* Half length the second pass settled on, in control periods.
     * Chosen from the winding's own time constant rather than fixed --
     * see estimate.c on why a fixed one cannot suit every motor. */
    uint16_t half_periods;

    /* The d-axis holding size the ramp settled on, in perturbation
     * units. Reported because everything else sits on it: it is what
     * pins the rotor, and what keeps the phase currents away from the
     * zero crossing the dead time turns on. */
    uint16_t hold_duty;
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
 * @param m             the motor, whose resistance is set to the mean of
 *                      the three phases on success
 * @param reverse_order measure C-A, B-C, A-B instead of A-B, B-C, C-A.
 *                      A winding's resistance cannot depend on the order
 *                      it was measured in, so running both ways is what
 *                      separates a genuinely unbalanced motor -- the
 *                      three answers stay with their pairs -- from
 *                      something drifting during the test, where they
 *                      stay with their position in the sequence
 * @param result_out  where the measurement is written, including where
 *                    it stopped if it failed
 * @return one of the ESTIMATE_ constants
 */
uint8_t estimate_resistance(motor_t *m,
                            uint8_t reverse_order,
                            estimate_resistance_result_t *result_out);

/**
 * Measure both inductances, independently of one another.
 *
 * Holds the rotor in line with phase A, then squares the duty about that
 * holding current -- first along the magnet axis for Ld, then along the
 * axis ninety electrical degrees away for Lq -- and takes the difference
 * between the rising and falling slopes in each. See the note at the top
 * of this file for why that difference is the whole measurement.
 *
 * Needs no resistance measurement and does not care whether one has been
 * taken: resistance cancels between the two slopes.
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
