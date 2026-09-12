/*
 * pid.h
 *
 * A PI/PID regulator: proportional, integral and derivative terms,
 * output clamped to a configurable range, with anti-windup on the
 * integrator. The building block every stage of the cascaded FOC loop
 * (current, velocity, position) is made from.
 *
 * WHY THIS ISN'T A WRAPPER OVER CMSIS-DSP's arm_pid_instance_f32
 *
 *   CMSIS's PID keeps its internal state as the coefficients of a
 *   recursive difference equation (effectively y[n-1], y[n-2] and the
 *   running sum, folded together through derived A0/A1/A2 terms), not
 *   as a separately addressable integrator. That is fine for a filter,
 *   but it means there is no single value to clamp or back-calculate
 *   when the output saturates -- clean anti-windup is not something
 *   that representation supports without reverse-engineering state
 *   values to reproduce a desired clamped output, which is fragile and
 *   not something the library documents as safe to do.
 *
 *   A cascaded FOC loop saturates routinely and by design -- a current
 *   loop pinned at the phase current limit, a velocity loop pinned at
 *   the current limit downstream of it -- so anti-windup is not
 *   optional here, it is the difference between a controller that
 *   recovers cleanly and one that overshoots badly every time a limit
 *   is hit. This keeps the integrator as its own field and applies
 *   conditional integration (freeze the integrator while the output is
 *   saturated and integrating would push further into saturation).
 *   CMSIS-DSP is still used elsewhere -- see filter.h -- just not here,
 *   where its state representation works against what this needs.
 *
 * DERIVATIVE ON MEASUREMENT, NOT ON ERROR
 *
 *   The derivative term is computed from the change in the measurement,
 *   not the change in the error. A setpoint step would otherwise appear
 *   as an instantaneous rate of change and produce a derivative spike
 *   ("derivative kick") that has nothing to do with how the plant is
 *   actually responding.
 */

#ifndef PID_H_
#define PID_H_

/*
 * One PI/PID instance. A cascaded loop owns one of these per stage
 * (e.g. one for Id, one for Iq, one for velocity, one for position) --
 * there is nothing shared between stages, so each just gets its own.
 */
typedef struct {
    float kp;
    float ki;
    float kd;

    float sample_time_s;   /* how often pid_update() is called, seconds */

    float output_min;
    float output_max;

    float integrator;
    float previous_measurement;
} pid_t;

/**
 * Set up a PID and zero its internal state.
 *
 * @param p              the controller to initialise
 * @param kp             proportional gain
 * @param ki             integral gain, output units per (error unit *
 *                       second)
 * @param kd             derivative gain, output units per (error unit /
 *                       second)
 * @param sample_time_s  how often pid_update() will be called, seconds.
 *                       Must match the loop's actual rate, since it
 *                       scales both the integral and derivative terms.
 * @param output_min     lower clamp on the returned output
 * @param output_max     upper clamp on the returned output
 */
void pid_init(pid_t *p,
             float kp, float ki, float kd,
             float sample_time_s,
             float output_min, float output_max);

/**
 * Zero the integrator and the derivative's measurement history, without
 * touching the gains or limits.
 *
 * Call this whenever a stage is being enabled after being idle, or a
 * mode switch hands it a fresh setpoint -- otherwise the first update
 * reacts to a derivative computed against a stale measurement, and any
 * integrator value accumulated while the stage was idle carries into a
 * command that is no longer relevant.
 *
 * @param p  the controller to reset
 */
void pid_reset(pid_t *p);

/**
 * Change the gains without disturbing the integrator or limits.
 *
 * @param p   the controller
 * @param kp  proportional gain
 * @param ki  integral gain
 * @param kd  derivative gain
 */
void pid_set_gains(pid_t *p, float kp, float ki, float kd);

/**
 * Change the output clamp without disturbing the gains or integrator.
 *
 * @param p            the controller
 * @param output_min   lower clamp on the returned output
 * @param output_max   upper clamp on the returned output
 */
void pid_set_output_limits(pid_t *p, float output_min, float output_max);

/**
 * Run one control period: compute the error, advance the integrator
 * (subject to anti-windup), compute the derivative against the previous
 * measurement, and return the clamped sum.
 *
 * CALLED ONCE PER SAMPLE PERIOD, at the rate given to pid_init() as
 * sample_time_s. Calling it at a different rate without updating that
 * field desyncs the integral and derivative terms from real time.
 *
 * @param p            the controller
 * @param setpoint     the desired value, in the same units as measurement
 * @param measurement  the measured value
 * @return the controller output, clamped to [output_min, output_max]
 */
float pid_update(pid_t *p, float setpoint, float measurement);

#endif /* PID_H_ */
