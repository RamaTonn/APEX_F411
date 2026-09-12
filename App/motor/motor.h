/*
 * motor.h
 *
 * The motor in one place: its geometry, its rotor angle, its measured
 * electrical properties, and the transforms between the three phase
 * windings and the two rotor axes.
 *
 * Replaces commutation.h, params.h, and the maths half of modulation.h.
 *
 * ANGLE IS ALWAYS A PARAMETER, NEVER READ INSIDE A TRANSFORM
 *
 *   motor_apply_dq() and motor_get_dq_currents() both take the electrical
 *   angle as an argument rather than reaching for the rotor's own.
 *
 *   High-frequency injection applies a voltage at an ESTIMATED angle and
 *   works out, from how the current responds, how wrong that estimate
 *   was. The angle it drives against is deliberately not the measured
 *   one. If a transform reached inside for the rotor's angle, injection
 *   would need a second, parallel path that did not, and the two would
 *   drift out of step. Passed as a parameter, one call serves ordinary
 *   control (hand it the measured angle) and injection (hand it the
 *   estimate).
 *
 * UNITS
 *
 *   The transforms work in floating point: volts, amps, radians. The
 *   angle keeps its 16-bit form where it is stored, because the encoder,
 *   telemetry and the protocol all speak it. Phase currents arrive from
 *   the sensor as milliamps.
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "encoder.h"

/* One electrical revolution in the 16-bit angle scale. The type's own
 * wrap-around performs the modulo for free. */
#define MOTOR_ANGLE_SCALE 65536u

/* The AS5147 reports 14 bits per mechanical revolution. */
#define MOTOR_ENCODER_COUNTS 16384u

/* Plausible bounds on the pole pair count. One is a real motor; beyond
 * about fifty is almost certainly a mistyped number. */
#define MOTOR_MIN_POLE_PAIRS 1u
#define MOTOR_MAX_POLE_PAIRS 50u

/* Radians per step of the 16-bit angle scale. A caller holding a stored
 * angle multiplies by this to get the radians the transforms take. */
#define MOTOR_ANGLE_TO_RAD (6.28318530718f / 65536.0f)
#define MOTOR_16BIT_TO_DEG (360.0f / 65536.0f)

/*
 * One motor. A single instance is owned by main.c and passed by pointer
 * to everything that touches the motor.
 */
typedef struct {
    /* --- Geometry: how an encoder reading becomes a rotor angle --- */
    uint8_t  pole_pairs;

    /* --- Measured electrical properties, in SI units --- */
    float resistance_ohm;   /* per phase, measured at the terminals       */
    float inductance_d_h;   /* along the rotor's magnetic axis            */
    float inductance_q_h;   /* across it, the torque-producing axis       */

    /* --- Live angle, refreshed once per control period ---
     *
     * volatile because the control interrupt writes these and the main
     * loop reads them; without it the compiler may cache a stale copy in
     * a register. */
    volatile uint16_t electrical_angle;  /* 0..65535 = one electrical rev */

    /* Non-zero while motor_update() should read the encoder each period.
     * OFF BY DEFAULT -- see motor_set_reading_enabled() for why. */
    volatile uint8_t reading_enabled;

    encoder_t* encoder;

    /* --- Live dq currents and voltages, refreshed once per control period ---
         *
         * volatile because the control interrupt writes these and the main
         * loop reads them; without it the compiler may cache a stale copy in
         * a register. */
    volatile float motor_id, motor_iq;
    volatile float commanded_vd, commanded_vq;

} motor_t;

/* ==================================================================
 * Lifecycle
 * ================================================================== */

/**
 * Reset a motor to its starting state: one pole pair, zeroed properties,
 * per-period encoder read off. Does not reset the encoder itself --
 * call encoder_init() separately, since the encoder may outlive or be
 * shared differently than any one motor.
 *
 * @param m  the motor to initialise
 * @param e  the encoder this motor reads its angle from
 */
void motor_init(motor_t *m, encoder_t *e);

/* ==================================================================
 * Geometry -- written by calibration, read by the control path
 * ================================================================== */

/**
 * Set the number of pole pairs.
 *
 * @param m           the motor
 * @param pole_pairs  between MOTOR_MIN_POLE_PAIRS and MOTOR_MAX_POLE_PAIRS
 * @return 1 on success, 0 if out of bounds (count left unchanged)
 */
uint8_t motor_set_pole_pairs(motor_t *m, uint8_t pole_pairs);

/**
 * @param m  the motor
 * @return the pole pair count in use
 */
uint8_t motor_get_pole_pairs(const motor_t *m);

/* ==================================================================
 * Rotor angle
 *
 * The offset and direction that used to live here now belong to the
 * encoder -- see motor_t::encoder and encoder_set_offset() /
 * encoder_set_direction(). A caller that already has a motor_t* reaches
 * them as encoder_set_offset(m->encoder, ...): motor_t holds the pointer,
 * it doesn't re-expose the encoder's own API.
 * ================================================================== */

/**
 * Read the encoder and refresh the electrical and raw angles.
 *
 * CALLED FROM THE CONTROL INTERRUPT ONLY, once per period.
 *
 * Uses the pipelined single-frame read, so the angle is one period old.
 * A failed read leaves the previous angles in place rather than zeroing
 * them: a momentary zero would look like the rotor jumping to the origin,
 * and a loop acting on it would apply a large correction to something
 * that never happened.
 *
 * Does nothing and returns 0 while the per-period read is disabled.
 *
 * @param m  the motor
 * @return 1 if the encoder answered and the angles were updated
 */
uint8_t motor_update(motor_t *m);

/**
 * @param m  the motor
 * @return the rotor's electrical angle, 0..65535
 */
uint16_t motor_get_electrical_angle(const motor_t *m);

/**
 * Turn the per-period encoder read on or off.
 *
 * OFF BY DEFAULT, and the default matters. motor_update() runs 32000
 * times a second and each call is an SPI transaction. The control
 * interrupt sits above USB in priority, so enough time spent there
 * starves the USB stack and enumeration fails. The read happens only
 * while something actually uses the angle. Enabling also primes the
 * pipeline, so the first angle is real.
 *
 * @param m        the motor
 * @param enabled  non-zero to read the encoder every control period
 */
void motor_set_reading_enabled(motor_t *m, uint8_t enabled);

/**
 * @param m  the motor
 * @return 1 if the per-period read is running
 */
uint8_t motor_reading_enabled(const motor_t *m);

/* ==================================================================
 * Transforms
 *
 * The angle is a parameter on both, for the injection reason given at
 * the top of this file. Neither reads the motor's live angle, so the
 * same call serves a measured angle and an estimated one.
 * ================================================================== */

/**
 * Phase currents to rotor-frame currents: Clarke then Park.
 *
 * Only two phase currents are needed; the third follows, since the three
 * sum to zero.
 *
 * @param m             the motor
 * @param current_a_ma  phase A current, milliamps, signed
 * @param current_b_ma  phase B current, milliamps, signed
 * @param angle_rad     the electrical angle to transform against
 * @param i_d_out       d-axis current written here, in amps
 * @param i_q_out       q-axis current written here, in amps
 */
void motor_get_dq_currents(const motor_t *m,
                           int32_t current_a_ma,
                           int32_t current_b_ma,
                           float   angle_rad,
                           float  *i_d_out,
                           float  *i_q_out);

/**
 * Apply a rotor-frame voltage demand to the bridge: inverse Park, then
 * inverse Clarke, then hand each phase voltage to the gate driver.
 *
 * The one call that turns a controller's output into switching. A current
 * loop hands it two PI outputs; open-loop drive hands it zero on d and
 * the desired amplitude on q.
 *
 * The phase currents are passed straight through to
 * gate_driver_apply_voltage(), which is what actually turns a voltage
 * into a duty and applies dead-time compensation -- see gate_driver.h
 * for why that correction needs them.
 *
 * @param m             the motor
 * @param v_d           d-axis voltage demand, volts
 * @param v_q           q-axis voltage demand, volts
 * @param angle_rad     the electrical angle to transform against
 * @param current_a_ma  phase A current, for dead-time compensation
 * @param current_b_ma  phase B current, for dead-time compensation
 * @param bus_mv        measured bus voltage, millivolts
 */
void motor_apply_dq(motor_t *m,
                    float    v_d,
                    float    v_q,
                    float    angle_rad,
                    int32_t  current_a_ma,
                    int32_t  current_b_ma,
                    uint16_t bus_mv);

/* ==================================================================
 * Measured properties -- written by estimation, read by the loops
 * ================================================================== */

/**
 * Record a measured phase resistance.
 *
 * @param m    the motor
 * @param ohm  per-phase resistance, ohms
 */
void motor_set_resistance(motor_t *m, float ohm);

/**
 * Record measured inductances.
 *
 * @param m    the motor
 * @param d_h  inductance along the magnetic axis, henries
 * @param q_h  inductance across it, henries
 */
void motor_set_inductance(motor_t *m, float d_h, float q_h);

#endif /* MOTOR_H_ */
