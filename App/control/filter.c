#include "filter.h"

void filter_init(filter_t *f, const float32_t coefficients[5])
{
    for (uint8_t i = 0u; i < 5u; i++) {
        f->coeffs[i] = coefficients[i];
    }

    /* One stage: this is a single biquad, not a cascade. */
    arm_biquad_cascade_df2T_init_f32(&f->instance, 1u, f->coeffs, f->state);

    filter_reset(f);
}

void filter_reset(filter_t *f)
{
    f->state[0] = 0.0f;
    f->state[1] = 0.0f;
}

float filter_process(filter_t *f, float input)
{
    float output;

    /* Block size 1: one new measurement arrives per call. */
    arm_biquad_cascade_df2T_f32(&f->instance, &input, &output, 1u);

    return output;
}
