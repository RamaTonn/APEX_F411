#include "motor.h"

#include "encoder.h"
#include "gate_driver.h"

#include "arm_math.h"

/* ------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------ */

/* The encoder reports 14 bits per revolution and the control path holds
 * angles in 16, so a reading shifts up by two to change scale. */
#define ENCODER_TO_ANGLE_SHIFT 2u

/* Dead time to correct for, in nanoseconds: the gate driver's own delay
 * plus the transistors' turn-on time through the 10 ohm gate resistors.
 * Measured on this board rather than taken from a datasheet. */
#define DEAD_TIME_NS 750u

/* The switching period in nanoseconds. The timer counts to 1499 and back
 * at 96 MHz, so 2998 counts, which is 31229 nanoseconds. */
#define PERIOD_NS 31229u

/* Dead time as parts per thousand of the switching period. Derived
 * rather than written as a constant, so changing either figure keeps it
 * correct. With the values above this comes to 24. */
#define DEAD_TIME_PER_MILLE ((DEAD_TIME_NS * 1000u) / PERIOD_NS)

/* Current at which the dead time correction reaches full value, in
 * milliamps. Below this it is scaled down in proportion.
 *
 * A plain sign test fails here. The current sensor resolves about 40
 * milliamps per count and carries a few counts of noise, so a reading
 * near zero changes sign at random, and the correction would then flip
 * between plus and minus a full dead time -- a swing of 48 parts per
 * thousand, which is larger than the whole command. Scaling through the
 * zero crossing turns a reversal into a proportionally small wobble. */
#define DEAD_TIME_FULL_MA 800

/* Milliamps to amps, and millivolts to volts. */
#define MILLI_TO_UNIT 0.001f

/* Radians to degrees.
 *
 * arm_sin_cos_f32() takes its angle in DEGREES, which is easy to miss
 * given every other quantity in this module is SI. The public interface
 * stays in radians, as the rest of the control mathematics does, and the
 * conversion happens here at the one point that needs it. */
#define RAD_TO_DEG 57.2957795131f

/* ------------------------------------------------------------------
 * Turning an encoder reading into a rotor electrical angle
 * ------------------------------------------------------------------ */

/* Convert the encoder's mechanical angle into the rotor's electrical
 * angle.
 *
 * The encoder has already done the zero-referencing and direction
 * correction (see encoder_capture()), so the only thing left here is
 * multiplying by the pole pair count, because the windings see that many
 * electrical cycles per mechanical turn.
 *
 * @param m  the motor, for its encoder and pole pairs
 * @return the electrical angle, 0 to 65535 */
static uint16_t compute_electrical_angle(const motor_t *m)
{
    /* Multiply into electrical cycles, then keep only the part within
     * one cycle. The mask is the modulo, since the encoder scale is a
     * power of two. The product reaches at most 16383 times 30, well
     * inside 32 bits. */
    uint32_t within_cycle = ((uint32_t)m->e->mechanical_angle
                             * (uint32_t)m->pole_pairs)
                            & (MOTOR_ENCODER_COUNTS - 1u);

    /* Scale the encoder's 14 bits up to the 16-bit angle used everywhere
     * else, so one representation serves the whole control path. */
    return (uint16_t)(within_cycle << ENCODER_TO_ANGLE_SHIFT);
}

/* ------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------ */

void motor_init(motor_t *m, encoder_t *e)
{
    /* One pole pair is deliberately wrong for any real motor. A default
     * that looked plausible would let an unconfigured setup produce
     * nearly-right behaviour, which is far harder to notice than obvious
     * nonsense. The offset and direction that would similarly be "wrong
     * on purpose" live on the encoder now -- see encoder_init(). */
    m->pole_pairs        = 1u;

    m->electrical_angle  = 0u;
    m->reading_enabled   = 0u;

    m->e = e;

    m->resistance_ohm    = 0.0f;
    m->inductance_d_h    = 0.0f;
    m->inductance_q_h    = 0.0f;
    m->motor_id 		 = 0.0f;
	m->motor_iq			 = 0.0f;
    m->commanded_vd 	 = 0.0f;
	m->commanded_vq 	 = 0.0f;

}

/* ------------------------------------------------------------------
 * Geometry
 * ------------------------------------------------------------------ */

uint8_t motor_set_pole_pairs(motor_t *m, uint8_t pole_pairs)
{
    if ((pole_pairs < MOTOR_MIN_POLE_PAIRS)
            || (pole_pairs > MOTOR_MAX_POLE_PAIRS)) {
        return 0u;
    }

    m->pole_pairs = pole_pairs;
    return 1u;
}

uint8_t motor_get_pole_pairs(const motor_t *m)
{
    return m->pole_pairs;
}

void motor_set_offset(motor_t *m, uint16_t offset_counts)
{
    encoder_set_offset(m->e, offset_counts);
}

uint16_t motor_get_offset(const motor_t *m)
{
    return encoder_get_offset(m->e);
}

void motor_set_direction(motor_t *m, uint8_t forward)
{
    encoder_set_direction(m->e, (forward != 0u) ? 1u : 0u);
}

uint8_t motor_get_direction(const motor_t *m)
{
    return encoder_get_direction(m->e);
}

/* ------------------------------------------------------------------
 * Rotor angle
 * ------------------------------------------------------------------ */

uint8_t motor_update(motor_t *m)
{
    if (m->reading_enabled == 0u) {
        return 0u;
    }

    if (encoder_capture(m->e) == 0u) {
        /* The previous electrical angle is kept. Substituting zero would
         * look like the rotor jumping to the origin, and a loop acting
         * on that would apply a large correction to something that
         * never happened. */
        return 0u;
    }

    m->electrical_angle = compute_electrical_angle(m);

    return 1u;
}

uint16_t motor_get_electrical_angle(const motor_t *m)
{
    return m->electrical_angle;
}

uint16_t motor_get_raw_angle(const motor_t *m)
{
    return encoder_get_raw_count(m->e);
}

void motor_prime(motor_t *m)
{
    (void)m;
    encoder_prime_pipeline();
}

void motor_set_reading_enabled(motor_t *m, uint8_t enabled)
{
    if (enabled != 0u) {
        /* Flush the pipeline before enabling, so the first angle the
         * control loop sees is a real measurement rather than whatever
         * the sensor had queued from before. */
        encoder_prime_pipeline();
        m->reading_enabled = 1u;
    } else {
        m->reading_enabled = 0u;
    }
}

uint8_t motor_reading_enabled(const motor_t *m)
{
    return m->reading_enabled;
}

/* ------------------------------------------------------------------
 * Transforms
 * ------------------------------------------------------------------ */

void motor_get_dq_currents(const motor_t *m,
                           int32_t current_a_ma,
                           int32_t current_b_ma,
                           float   angle_rad,
                           float  *i_d_out,
                           float  *i_q_out)
{
    float sin_val;
    float cos_val;
    float i_alpha;
    float i_beta;

    (void)m;

    arm_sin_cos_f32(angle_rad * RAD_TO_DEG, &sin_val, &cos_val);

    /* Clarke: three phase currents to two stationary axes. Only two are
     * passed, because the three sum to zero and the third carries no
     * information the first two do not already hold. */
    arm_clarke_f32((float)current_a_ma * MILLI_TO_UNIT,
                   (float)current_b_ma * MILLI_TO_UNIT,
                   &i_alpha, &i_beta);

    /* Park: stationary axes to rotor axes, using the angle given rather
     * than the rotor's own. See the note at the top of motor.h. */
    arm_park_f32(i_alpha, i_beta, i_d_out, i_q_out, sin_val, cos_val);
}

/* Work out one phase's duty from the voltage demanded of it.
 *
 * @param phase_voltage     what this phase should produce, in volts,
 *                          either side of the resting point
 * @param bus_mv            measured bus voltage, millivolts
 * @param phase_current_ma  this phase's current in milliamps, signed;
 *                          positive means flowing into the terminal
 * @return duty in parts per thousand, 0 to 1000 */
static uint16_t phase_duty(float    phase_voltage,
                           uint16_t bus_mv,
                           int32_t  phase_current_ma)
{
    /* A bridge cannot drive a phase below the negative rail, so the
     * demand is applied as a deviation either side of half duty. At half
     * on all three phases the terminals sit at the same potential and no
     * current flows, which is why half is the resting point. */
    int32_t duty = (int32_t)(GATE_DRIVER_DUTY_SCALE / 2u);

    if (bus_mv == 0u) {
        /* Nothing sensible can be computed without a bus measurement,
         * and dividing by it would fault. Resting duty applies no
         * voltage, which is the safe answer. */
        return (uint16_t)duty;
    }

    /* Express the demand as a fraction of the bus, in the same parts per
     * thousand as the duty. Deriving it from the measured bus rather
     * than a nominal figure means the voltage actually applied stays
     * correct as the supply sags. */
    int32_t commanded = (int32_t)((phase_voltage
                                   * (float)GATE_DRIVER_DUTY_SCALE)
                                  / ((float)bus_mv * MILLI_TO_UNIT));
    duty += commanded;

    /* Dead time correction.
     *
     * Current flowing OUT of the terminal -- negative here -- holds the
     * phase toward the positive rail during the gap, so it spent longer
     * high than commanded and the correction is downward. Current
     * flowing in does the opposite.
     *
     * Scaled in proportion below the threshold rather than switched on a
     * sign test, so that noise near a zero crossing produces a small
     * wobble instead of a full reversal. */
    int32_t scaled = phase_current_ma;

    if (scaled > DEAD_TIME_FULL_MA) {
        scaled = DEAD_TIME_FULL_MA;
    } else if (scaled < -DEAD_TIME_FULL_MA) {
        scaled = -DEAD_TIME_FULL_MA;
    }

    int32_t correction = ((int32_t)DEAD_TIME_PER_MILLE * scaled)
                         / DEAD_TIME_FULL_MA;

    /* The correction may never exceed what was actually commanded.
     *
     * WHY THIS BOUND IS NOT OPTIONAL
     *
     *   The correction takes its sign from the measured current, and the
     *   voltage it admits sustains that current. That is a loop, and
     *   whether it is stable depends on how large the correction is next
     *   to the command.
     *
     *   On this board the full correction is 24 parts per thousand of
     *   the bus -- roughly 290 millivolts, which across a 54 milliohm
     *   path is over five amps. So on a low resistance winding the
     *   correction alone can drive far more current than any sensible
     *   command, and once current flows in either direction the
     *   correction holds it there. The motor draws heavily, produces no
     *   useful torque, and which way it latched is arbitrary.
     *
     *   Bounding the correction by the commanded value breaks the loop.
     *   It can still restore voltage the dead time removed, which is its
     *   entire purpose, but it can no longer create voltage that was
     *   never asked for -- so with nothing commanded, nothing happens.
     */
    int32_t bound = (commanded < 0) ? -commanded : commanded;

    if (correction > bound) {
        correction = bound;
    } else if (correction < -bound) {
        correction = -bound;
    }

    duty += correction;

    /* Clamped here as well as in the gate driver. A phase clipped
     * downstream would distort the vector relative to the other two
     * without anything noticing. */
    if (duty < 0) {
        duty = 0;
    } else if (duty > (int32_t)GATE_DRIVER_DUTY_SCALE) {
        duty = (int32_t)GATE_DRIVER_DUTY_SCALE;
    }

    return (uint16_t)duty;
}

void motor_apply_dq(motor_t *m,
                    float    v_d,
                    float    v_q,
                    float    angle_rad,
                    int32_t  current_a_ma,
                    int32_t  current_b_ma,
                    uint16_t bus_mv)
{
    float sin_val;
    float cos_val;
    float v_alpha;
    float v_beta;
    float v_a;
    float v_b;

    (void)m;

    arm_sin_cos_f32(angle_rad * RAD_TO_DEG, &sin_val, &cos_val);

    /* Inverse Park: rotor axes back to the stationary frame, against the
     * angle given rather than the rotor's own. */
    arm_inv_park_f32(v_d, v_q, &v_alpha, &v_beta, sin_val, cos_val);

    /* Inverse Clarke: stationary frame to two phase voltages. */
    arm_inv_clarke_f32(v_alpha, v_beta, &v_a, &v_b);

    /* The third phase follows from the other two, since the three
     * voltages of a balanced set sum to zero -- the same reasoning that
     * lets two current sensors serve three windings. */
    float v_c = -(v_a + v_b);

    /* Phase C has no current sensor either. The three currents of a
     * star-connected winding sum to zero, because the star point is the
     * only place they meet and none leaves through it. */
    int32_t current_c_ma = -(current_a_ma + current_b_ma);

    gate_driver_set_duty(GATE_DRIVER_PHASE_A,
                         phase_duty(v_a, bus_mv, current_a_ma));
    gate_driver_set_duty(GATE_DRIVER_PHASE_B,
                         phase_duty(v_b, bus_mv, current_b_ma));
    gate_driver_set_duty(GATE_DRIVER_PHASE_C,
                         phase_duty(v_c, bus_mv, current_c_ma));
}

/* ------------------------------------------------------------------
 * Measured properties
 * ------------------------------------------------------------------ */

void motor_set_resistance(motor_t *m, float ohm)
{
    m->resistance_ohm = ohm;
}

void motor_set_inductance(motor_t *m, float d_h, float q_h)
{
    m->inductance_d_h = d_h;
    m->inductance_q_h = q_h;
}
