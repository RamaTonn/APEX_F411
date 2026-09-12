/*
 * loop.h
 *
 * The fixed-rate control loop: a function that runs once per PWM period,
 * triggered by hardware, with the rotor angle and phase currents already
 * refreshed for this period.
 *
 * Everything above this -- commutation, current regulation, high
 * frequency injection -- runs inside that function. This module provides
 * the timing; it does not decide what to do with the measurements, and
 * it does not own them either. Phase currents live in sensors_t (see
 * sensors.h), the rotor angle in motor_t (see motor.h) -- loop_t's only
 * job is making sure both are current before the installed function
 * runs, once per period, at a rate you can trust.
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
 *   trigger the conversion. Moving loop_set_sample_point() moves the
 *   sample instant without touching the PWM itself.
 *
 *   In centre-aligned mode 1 a channel's compare event is only flagged
 *   while the counter is counting down, so this fires once per period
 *   rather than twice.
 *
 * EXECUTION CONTEXT
 *
 *   The registered loop function runs in interrupt context at the PWM
 *   rate. At 32 kHz that is one call every 31.25 microseconds, or about
 *   3000 CPU cycles. It must not block, must not call anything that
 *   waits, and must not send anything over USB. loop_get_duration_us()
 *   reports how long it actually takes, which is the number to watch
 *   before adding anything to it.
 */

#ifndef LOOP_H_
#define LOOP_H_

#include <stdint.h>

#include "motor.h"

/* Timer counts in one PWM period, one direction. The timer counts up to
 * this and back down, so a full period is twice this many counts:
 * 96 MHz divided by 3000 gives 32.0 kHz exactly. */
#define LOOP_TIMER_PERIOD_COUNTS 1499U

/* Loop rate in hertz, for anything that needs to turn a rate of change
 * into a per-step increment. */
#define LOOP_RATE_HZ 32000U

/* Default sample point, as a compare value for channel 4.
 *
 * The counter is descending from its peak when compare events are
 * flagged, so a value just below the peak fires shortly after the
 * counter turns around -- placing the conversion close to the middle of
 * the interval where all three low-side FETs conduct, which is where the
 * ripple sits at its average. */
#define LOOP_DEFAULT_SAMPLE_POINT (LOOP_TIMER_PERIOD_COUNTS - 20U)

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
typedef void (*loop_function_t)(int32_t current_a_ma,
                                int32_t current_b_ma);

/*
 * The loop. Owned by main.c and initialised once; every other function
 * below acts on that one instance, which this module keeps a pointer to
 * internally -- there is exactly one PWM-rate loop on this board, so
 * nothing outside this file needs the struct itself.
 */
typedef struct {
    volatile uint32_t iteration_count;
    volatile uint32_t overrun_count;
    volatile uint32_t last_duration_cycles;
    volatile uint8_t  iteration_in_progress;
    volatile loop_function_t installed_function;
    volatile uint8_t  running;
    uint8_t  initialised;
    uint16_t sample_point;
} loop_t;

/**
 * Prepare the loop. Measures the no-current sensor references, positions
 * the sample point, and starts the injected conversion trigger.
 *
 * Does NOT start calling the installed function -- loop_start() does
 * that. Separating the two means the loop can be set up while the bridge
 * is safely disabled.
 *
 * @param l  the loop instance
 * @param m  the motor this loop refreshes the angle of, once per period
 * @return 1 on success, 0 if the ADC did not respond during calibration
 */
uint8_t loop_init(loop_t *l, motor_t *m);

/**
 * Install the function to run every period.
 *
 * Passing NULL removes it, which leaves the loop running and measuring
 * but doing nothing -- useful for checking sample quality before any
 * control algorithm exists.
 *
 * @param loop_function  the function to call, or NULL for none
 */
void loop_set_function(loop_function_t loop_function);

/**
 * Begin calling the installed function once per PWM period.
 *
 * @return 1 on success, 0 if loop_init() has not run successfully
 */
uint8_t loop_start(void);

/**
 * Stop calling the installed function. Sampling continues, so currents
 * and the rotor angle can still be read, but nothing acts on them. Does
 * not disable the bridge.
 */
void loop_stop(void);

/**
 * @return 1 if the installed function is currently being called
 */
uint8_t loop_is_running(void);

/**
 * Whether loop_init() completed successfully.
 *
 * Distinguishes "set up but not started" from "never set up". A loop
 * that is not initialised will refuse to start, and the usual cause is
 * that the timer never triggered the ADC -- so this is the first thing
 * to check when currents read exactly zero.
 *
 * @return 1 if initialisation succeeded, 0 otherwise
 */
uint8_t loop_is_initialised(void);

/**
 * How long the last iteration took, in microseconds.
 *
 * Measured with the cycle counter, so it includes the sampling, the
 * angle refresh, and the installed function itself.
 *
 * The period is 31.25 microseconds. Anything approaching that means the
 * loop is close to not finishing before the next one starts, which
 * produces missed periods rather than a clean failure. Keep well below.
 *
 * @return duration in microseconds
 */
uint32_t loop_get_duration_us(void);

/**
 * How many periods have elapsed since loop_start().
 *
 * Divided by the elapsed time, this confirms the loop is actually
 * running at the rate it claims -- which is worth checking directly
 * rather than assuming, since a misconfigured trigger produces a loop
 * that runs at a plausible but wrong rate.
 *
 * @return period count
 */
uint32_t loop_get_iteration_count(void);

/**
 * How many periods were missed because the previous iteration had not
 * finished.
 *
 * Any number above zero means the installed function is too slow.
 *
 * @return overrun count
 */
uint32_t loop_get_overrun_count(void);

/**
 * Move the instant at which currents are sampled.
 *
 * @param compare_value  0 to LOOP_TIMER_PERIOD_COUNTS. Larger values
 *                       sample earlier after the counter peak. Values
 *                       outside the range are clamped.
 */
void loop_set_sample_point(uint16_t compare_value);

/**
 * @return the current sample point compare value
 */
uint16_t loop_get_sample_point(void);

#endif /* LOOP_H_ */
