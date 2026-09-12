/*
 * currentloop.h
 *
 * Closed-loop dq current control: the innermost stage of the cascaded
 * FOC loop this board is being built toward (position -> velocity, or
 * impedance -> current -> bridge). Regulates Id and Iq independently
 * against measured feedback, using the rotor's actual angle -- unlike
 * openloop.c, this expects the rotor to be wherever the encoder says it
 * is, not wherever the applied vector points.
 *
 * WHAT THIS IS USEFUL FOR ON ITS OWN
 *
 *   Velocity and position control are not built yet. In the meantime,
 *   handing this an Iq target directly produces torque, the same way
 *   openloop_start() produces motion -- but with actual current
 *   regulation instead of an open commitment to whatever voltage was
 *   asked for. That also makes it the right place to first validate the
 *   PI gains before anything is stacked on top of it.
 *
 * WHY Id DEFAULTS TO ZERO
 *
 *   Driving zero d-axis current is the standard simplification for a
 *   surface-mount PM motor: with no reluctance torque to exploit, all
 *   the torque comes from Iq, and every other drive routine already on
 *   this board (openloop, calibration, estimation) uses the same
 *   convention. A caller wanting field weakening or MTPA can still set
 *   a non-zero Id target -- this module does not assume zero, it is
 *   just what a caller with no reason to do otherwise should pass.
 *
 * GAIN COMPUTATION
 *
 *   The plant seen by each axis is a first-order R-L circuit,
 *   I(s)/V(s) = 1/(R + sL). Choosing Kp = L*wc and Ki = R*wc places a PI
 *   zero exactly on the plant's pole, leaving a first-order closed loop
 *   with bandwidth wc -- the standard "internal model control" current
 *   loop design. wc is CURRENTLOOP_BANDWIDTH_HZ, converted to radians
 *   per second.
 *
 *   This needs the motor's measured resistance and inductance, so
 *   estimate_resistance() and estimate_inductance() must have produced
 *   real values before currentloop_start() is called -- it refuses
 *   otherwise, since computing gains from zero would silently produce
 *   zero gains rather than an error.
 *
 *   CURRENTLOOP_BANDWIDTH_HZ is a starting point taken from a common
 *   rule of thumb (bandwidth well below the sample rate, to leave
 *   margin for the one-period measurement delay), not a value measured
 *   on this motor. Expect to retune once this is running against real
 *   hardware.
 *
 * OUTPUT LIMITS TRACK THE MEASURED BUS
 *
 *   Anti-windup (see pid.h) only does its job if the PID's own output
 *   clamp matches what the hardware can actually deliver -- a clamp set
 *   too generously would let the integrator keep winding up well past
 *   the point the bridge has already saturated. So each period's output
 *   limit is derived from the measured bus voltage rather than fixed at
 *   init time, the same reasoning gate_driver_apply_voltage() uses for
 *   why it takes a measured bus_mv rather than a nominal one.
 */

#ifndef CURRENTLOOP_H_
#define CURRENTLOOP_H_

#include <stdint.h>

#include "motor.h"
#include "loop.h"

/* Current-loop bandwidth, in hertz -- see GAIN COMPUTATION above.
 *
 * A tenth to a twentieth of the sample rate is a common starting point
 * for a current loop carrying a one-period measurement delay; this uses
 * a twentieth for extra margin since it has never been tuned against
 * real hardware. */
#define CURRENTLOOP_BANDWIDTH_HZ (LOOP_RATE_HZ / 20u)

/* Fraction of the bus voltage each axis's PID output is clamped to, as
 * parts per thousand of the bus.
 *
 * Matches GATE_DRIVER_DUTY_MAXIMUM's headroom (duty never exceeds 90%,
 * i.e. 400 parts per thousand either side of the resting point), so
 * anti-windup engages at roughly the point the hardware would actually
 * saturate. Approximate: Vd and Vq are clamped independently here
 * rather than as a combined vector, which is conservative but not
 * exact -- refine once this is being tuned for real. */
#define CURRENTLOOP_OUTPUT_LIMIT_PER_MILLE 400u

/**
 * Prepare the module. Does not move anything.
 *
 * Must be called after loop_init(), because it installs itself as the
 * loop's function.
 */
void currentloop_init(motor_t *m);

/**
 * Begin regulating current.
 *
 * Computes PI gains from the motor's measured resistance and
 * inductance (see GAIN COMPUTATION above), enables the per-period
 * encoder read (motor_set_reading_enabled()) since closed-loop control
 * needs the real rotor angle every period, resets both integrators, and
 * enables all three phases -- which blocks briefly per phase to charge
 * each bootstrap capacitor, the same as openloop_start().
 *
 * @param id_target_ma  d-axis current target, milliamps, signed
 * @param iq_target_ma  q-axis current target, milliamps, signed
 * @return 1 on success, 0 if the control loop is not running, or the
 *         motor's resistance/inductance have not been measured yet
 */
uint8_t currentloop_start(int32_t id_target_ma, int32_t iq_target_ma);

/**
 * Change the current targets without restarting -- the bootstrap
 * capacitors stay charged and the integrators keep their state, unlike
 * stopping and starting again.
 *
 * @param id_target_ma  d-axis current target, milliamps, signed
 * @param iq_target_ma  q-axis current target, milliamps, signed
 */
void currentloop_set_targets(int32_t id_target_ma, int32_t iq_target_ma);

/**
 * Stop regulating and disable the bridge. The motor coasts.
 */
void currentloop_stop(void);

/**
 * @return 1 if the loop is currently regulating, 0 otherwise
 */
uint8_t currentloop_is_running(void);

/**
 * Report what the module is doing, for the console.
 *
 * @param id_target_ma_out    commanded d-axis target, milliamps. May be
 *                            NULL.
 * @param iq_target_ma_out    commanded q-axis target, milliamps. May be
 *                            NULL.
 * @param id_measured_ma_out  most recent measured d-axis current,
 *                            milliamps. May be NULL.
 * @param iq_measured_ma_out  most recent measured q-axis current,
 *                            milliamps. May be NULL.
 */
void currentloop_get_state(int32_t *id_target_ma_out,
                           int32_t *iq_target_ma_out,
                           int32_t *id_measured_ma_out,
                           int32_t *iq_measured_ma_out);

#endif /* CURRENTLOOP_H_ */
