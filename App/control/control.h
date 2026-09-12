/*
 * control.h
 *
 * The fixed-rate control loop: a function that runs once per PWM period,
 * triggered by hardware, with phase currents already sampled at exactly
 * the right instant.
 *
 * Everything above this -- commutation, current regulation, high
 * frequency injection -- runs inside that function. This module provides
 * the timing and the measurements; it does not decide what to do with
 * them.
 *
 * WHY THE SAMPLING IS SYNCHRONISED
 *
 *   Phase current is not constant within a PWM period. It ramps up while
 *   the high side conducts and decays while the winding freewheels, so
 *   the instantaneous value depends entirely on when you look.
 *
 *   With centre-aligned PWM all three phase pulses are symmetric about
 *   the middle of the period, which means the ripple on all three phases
 *   crosses its own average at the same two instants: the counter peak
 *   and the counter valley. Sampling there gives the average current
 *   directly, with no filtering and no lag.
 *
 *   That property is why centre-aligned mode is not optional here. With
 *   edge-aligned PWM every phase has its ripple midpoint somewhere
 *   different, and no single sample point is correct for all three.
 *
 * WHY THIS MATTERS MORE FOR HIGH FREQUENCY INJECTION
 *
 *   Injection works by applying a small high-frequency voltage and
 *   measuring how much current it produces. Because the rotor's
 *   inductance differs along and across its magnetic axis, that current
 *   response varies with rotor position -- which is what makes position
 *   estimation possible with the motor stationary.
 *
 *   The response being measured is small, and it is extracted by
 *   differencing consecutive samples. Any jitter in when the samples are
 *   taken appears directly as noise on the position estimate. Hardware
 *   triggering from the timer gives sample instants with no jitter at
 *   all, which software triggering cannot.
 *
 * WHERE THE SAMPLE POINT IS, AND HOW TO MOVE IT
 *
 *   Timer channel 4 is not connected to any pin. Its only job is to
 *   reach its compare value at the moment the ADC should sample, and
 *   trigger the conversion. Moving control_set_sample_point() moves the
 *   sample instant without touching the PWM itself.
 *
 *   In centre-aligned mode 1 a channel's compare event is only flagged
 *   while the counter is counting down, so this fires once per period
 *   rather than twice.
 *
 * EXECUTION CONTEXT
 *
 *   The registered control function runs in interrupt context at the PWM
 *   rate. At 32 kHz that is one call every 31.25 microseconds, or about
 *   3000 CPU cycles. It must not block, must not call anything that
 *   waits, and must not send anything over USB. control_get_duration_us()
 *   reports how long it actually takes, which is the number to watch
 *   before adding anything to it.
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdint.h>

#include "motor.h"

/* Timer counts in one PWM period, one direction. The timer counts up to
 * this and back down, so a full period is twice this many counts:
 * 96 MHz divided by 3000 gives 32.0 kHz exactly. */
#define CONTROL_TIMER_PERIOD_COUNTS 1499U

/* Loop rate in hertz, for anything that needs to turn a rate of change
 * into a per-step increment. */
#define CONTROL_LOOP_RATE_HZ 32000U

/* Default sample point, as a compare value for channel 4.
 *
 * The counter is descending from its peak when compare events are
 * flagged, so a value just below the peak fires shortly after the
 * counter turns around -- placing the conversion close to the middle of
 * the interval where all three low-side FETs conduct, which is where the
 * ripple sits at its average. */
#define CONTROL_DEFAULT_SAMPLE_POINT (CONTROL_TIMER_PERIOD_COUNTS - 20U)

/* Signature of the function that runs every period.
 *
 * @param current_a_ma  phase A current in milliamps, signed, positive
 *                      meaning current flowing into the motor terminal
 * @param current_b_ma  phase B current, same convention
 *
 * Phase C is not measured. It can be derived when needed, since the
 * three phase currents of a star-connected motor sum to zero:
 * current C is the negative of A plus B.
 */
typedef void (*control_function_t)(int32_t current_a_ma,
                                   int32_t current_b_ma);

/**
 * Prepare the loop. Measures the no-current sensor readings, positions
 * the sample point, and starts the injected conversion trigger.
 *
 * Does NOT start calling the control function -- control_start() does
 * that. Separating the two means the loop can be set up while the bridge
 * is safely disabled.
 *
 * @return 1 on success, 0 if the ADC did not respond during calibration
 */
uint8_t control_init(motor_t *m);

/**
 * Install the function to run every period.
 *
 * Passing NULL removes it, which leaves the loop running and measuring
 * but doing nothing -- useful for checking sample quality before any
 * control algorithm exists.
 *
 * @param control_function  the function to call, or NULL for none
 */
void control_set_function(control_function_t control_function);

/**
 * Begin calling the installed function once per PWM period.
 *
 * @return 1 on success, 0 if control_init() has not run successfully
 */
uint8_t control_start(void);

/**
 * Stop calling the control function. Sampling continues, so currents can
 * still be read, but nothing acts on them. Does not disable the bridge.
 */
void control_stop(void);

/**
 * @return 1 if the control function is currently being called
 */
uint8_t control_is_running(void);

/**
 * Whether control_init() completed successfully.
 *
 * Distinguishes "set up but not started" from "never set up". A loop
 * that is not initialised will refuse to start, and the usual cause is
 * that the timer never triggered the ADC -- so this is the first thing
 * to check when currents read exactly zero.
 *
 * @return 1 if initialisation succeeded, 0 otherwise
 */
uint8_t control_is_initialised(void);

/**
 * Most recent phase currents, in milliamps.
 *
 * Safe to call from the main loop. The values are a snapshot from
 * whichever period completed most recently.
 *
 * @param current_a_out  where to store phase A current. May be NULL.
 * @param current_b_out  where to store phase B current. May be NULL.
 */
void control_get_currents(int32_t *current_a_out, int32_t *current_b_out);

/**
 * How long the last iteration took, in microseconds.
 *
 * Measured with the cycle counter, so it includes the sampling, the
 * conversion to milliamps, and the control function itself.
 *
 * The period is 31.25 microseconds. Anything approaching that means the
 * loop is close to not finishing before the next one starts, which
 * produces missed periods rather than a clean failure. Keep well below.
 *
 * @return duration in microseconds
 */
uint32_t control_get_duration_us(void);

/**
 * How many periods have elapsed since control_start().
 *
 * Divided by the elapsed time, this confirms the loop is actually
 * running at the rate it claims -- which is worth checking directly
 * rather than assuming, since a misconfigured trigger produces a loop
 * that runs at a plausible but wrong rate.
 *
 * @return period count
 */
uint32_t control_get_iteration_count(void);

/**
 * How many periods were missed because the previous iteration had not
 * finished.
 *
 * Any number above zero means the control function is too slow.
 *
 * @return overrun count
 */
uint32_t control_get_overrun_count(void);

/**
 * Move the instant at which currents are sampled.
 *
 * @param compare_value  0 to CONTROL_TIMER_PERIOD_COUNTS. Larger values
 *                       sample earlier after the counter peak. Values
 *                       outside the range are clamped.
 */
void control_set_sample_point(uint16_t compare_value);

/**
 * @return the current sample point compare value
 */
uint16_t control_get_sample_point(void);


void control_get_zero_counts(uint16_t *zero_a_out, uint16_t *zero_b_out);

#endif /* CONTROL_H_ */
