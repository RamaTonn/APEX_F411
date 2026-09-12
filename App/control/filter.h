/*
 * filter.h
 *
 * A single second-order (biquad) IIR filter, for smoothing a
 * measurement before it feeds a pid_t -- e.g. a noisy velocity estimate,
 * or a current reading with switching ripple still on it.
 *
 * Wraps CMSIS-DSP's transposed direct form II biquad
 * (arm_biquad_cascade_df2T_f32), chosen over direct form I for its
 * better numerical behaviour at single precision.
 *
 * COEFFICIENT CONVENTION -- READ BEFORE DESIGNING COEFFICIENTS
 *
 *   CMSIS expects five coefficients, in the order {b0, b1, b2, a1, a2},
 *   implementing
 *
 *       y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + a1*y[n-1] + a2*y[n-2]
 *
 *   Note the SIGN of a1/a2. Most filter design tools (MATLAB, scipy,
 *   an analogue textbook) hand you a transfer function
 *   H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1' z^-1 + a2' z^-2) --
 *   CMSIS's a1 and a2 are the NEGATIVE of that a1'/a2'. Forgetting to
 *   negate them is the single most common way to turn a low-pass filter
 *   into an oscillator.
 *
 *   This header does not compute coefficients for you -- it is the
 *   container and the per-sample executor, not a filter design tool.
 *   Design the coefficients for your sample rate and cutoff externally
 *   (a Butterworth/biquad calculator, MATLAB, scipy.signal) and pass
 *   the result to filter_init().
 */

#ifndef FILTER_H_
#define FILTER_H_

#include "arm_math.h"

/*
 * One biquad section. Owns its own copy of the coefficients CMSIS's
 * instance points at, so the array passed to filter_init() does not
 * need to outlive the call.
 */
typedef struct {
    arm_biquad_cascade_df2T_instance_f32 instance;
    float32_t state[2];
    float32_t coeffs[5];
} filter_t;

/**
 * Set up a filter from its coefficients and zero its state.
 *
 * @param f             the filter to initialise
 * @param coefficients  {b0, b1, b2, a1, a2} -- see the note at the top
 *                      of this file about the sign of a1/a2
 */
void filter_init(filter_t *f, const float32_t coefficients[5]);

/**
 * Zero the filter's internal state, without changing its coefficients.
 *
 * Call this whenever the input is about to jump discontinuously for a
 * reason the filter shouldn't react to -- e.g. re-enabling a stage after
 * it was idle -- so the old state doesn't leak into the next output as
 * a transient.
 *
 * @param f  the filter to reset
 */
void filter_reset(filter_t *f);

/**
 * Filter one sample.
 *
 * CALLED ONCE PER SAMPLE PERIOD. The coefficients were designed for a
 * specific sample rate; calling this at a different rate silently moves
 * the filter's actual cutoff away from the one it was designed for.
 *
 * @param f      the filter
 * @param input  the new sample
 * @return the filtered output
 */
float filter_process(filter_t *f, float input);

#endif /* FILTER_H_ */
