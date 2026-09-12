#include "loop.h"

#include "encoder.h"
#include "motor.h"
#include "sensors.h"
#include "main.h"
#include "telemetry.h"

/* Handles CubeMX creates in main.c. */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

/* How long to wait for zero-calibration samples to arrive before giving
 * up.
 *
 * sensors.c averages 64 samples at the 32 kHz loop rate, which take 2
 * milliseconds, so 200 is fifty times the margin needed. The point is
 * not to be generous but to fail quickly and definitely if the trigger
 * is not working at all, rather than hanging. */
#define ZERO_CALIBRATION_TIMEOUT_MS 200U

/* CPU cycles per microsecond, for turning cycle counter readings into
 * something readable. The core runs at 96 MHz. */
#define CPU_CYCLES_PER_MICROSECOND 96U

/* The loop instance handed to loop_init(). Cached here because the
 * interrupt needs it and has no way to be handed a context of its own. */
static loop_t *self;

/* The motor this loop refreshes each period. Set by loop_init(). */
static motor_t *motor;

/* ------------------------------------------------------------------
 * Cycle counter
 *
 * The Cortex-M4 debug unit contains a free-running counter that
 * increments once per CPU clock. It is the only timing source fine
 * enough to measure something lasting a few microseconds, and reading it
 * costs a single instruction.
 * ------------------------------------------------------------------ */

static void cycle_counter_enable(void)
{
    /* The trace enable bit must be set before the cycle counter will
     * run: the counter lives in the data watchpoint unit, which is part
     * of the trace subsystem. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ------------------------------------------------------------------
 * The interrupt
 *
 * Called by HAL when the injected conversion sequence completes. Both
 * phase currents are already sitting in the injected data registers,
 * sampled at the instant timer channel 4 reached its compare value.
 *
 * @param hadc  which ADC completed; checked because HAL routes every
 *              ADC's completion through this one callback
 * ------------------------------------------------------------------ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) {
        return;
    }

    uint32_t start_cycles = DWT->CYCCNT;

    /* A trigger arriving while the previous iteration is still running
     * means the installed function takes longer than a PWM period.
     * Counting it and returning is better than running two iterations on
     * top of each other, which would corrupt whatever state that
     * function keeps. */
    if (self->iteration_in_progress != 0u) {
        self->overrun_count++;
        return;
    }
    self->iteration_in_progress = 1u;

    /* Reads the injected group and, outside calibration, publishes
     * current_a_ma/current_b_ma. During calibration it accumulates raw
     * samples instead -- see sensors.h. */
    sensors_capture_currents();

    /* The rotor angle is updated before the installed function runs, so
     * that anything closing a loop acts on this period's position rather
     * than the previous one.
     *
     * The pipelined read costs about three and a half microseconds. That
     * is paid every period whether or not anything uses the angle, which
     * is the price of having it available the moment a function is
     * installed. */
    (void)motor_update(motor);

    int32_t current_a;
    int32_t current_b;
    sensors_get_currents(&current_a, &current_b);

    if ((self->running != 0u) && (self->installed_function != NULL)) {
        self->installed_function(current_a, current_b);
    }

    /* Captured every period regardless of whether anything is driving
     * the motor.
     *
     * Streaming the currents with the bridge idle is how the sensors get
     * checked before anything moves, so this must not depend on a
     * loop function being installed -- and it runs after that function
     * so that any angle it published belongs to this period rather than
     * the previous one. */
    telemetry_capture(current_a, current_b);

    self->iteration_count++;

    /* Unsigned subtraction gives the correct elapsed count even when the
     * cycle counter wraps between the two readings. */
    self->last_duration_cycles = DWT->CYCCNT - start_cycles;

    self->iteration_in_progress = 0u;
}

/* ------------------------------------------------------------------
 * Setup
 * ------------------------------------------------------------------ */

/* Collect the no-current readings through the running interrupt.
 *
 * Assumes the timer is already generating trigger events and the
 * injected interrupt is already armed -- so this must be called after
 * both have been started, not before.
 *
 * The bridge must be disabled while this runs, or the "no current"
 * references will have real current baked into them.
 *
 * @return 1 on success, 0 if the samples never arrived, which means the
 *         timer is not triggering the ADC */
static uint8_t collect_zero_references(void)
{
    sensors_begin_current_calibration();

    uint32_t start_tick = HAL_GetTick();

    while (sensors_current_calibration_done() == 0u) {
        /* Subtracting the ticks rather than comparing against a sum
         * keeps this correct across the 49-day tick wrap. */
        if ((HAL_GetTick() - start_tick) > ZERO_CALIBRATION_TIMEOUT_MS) {
            sensors_cancel_current_calibration();
            return 0u;
        }
    }

    sensors_end_current_calibration();
    return 1u;
}

uint8_t loop_init(loop_t *l, motor_t *m)
{
    self  = l;
    motor = m;

    self->initialised           = 0u;
    self->running               = 0u;
    self->installed_function    = NULL;
    self->iteration_count       = 0u;
    self->overrun_count         = 0u;
    self->iteration_in_progress = 0u;
    self->sample_point          = LOOP_DEFAULT_SAMPLE_POINT;

    cycle_counter_enable();

    /* Position the sample instant before starting anything, so the very
     * first conversion lands where intended. */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, self->sample_point);

    /* Channel 4 is configured as PWM generation with no output pin. It
     * drives nothing; its only job is to produce a compare event once
     * per period, which is what the ADC's injected group is triggered
     * from. Starting it also starts the timer counter.
     *
     * This must happen BEFORE the calibration below, because a stopped
     * timer produces no compare events and therefore no conversions. */
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) {
        return 0u;
    }

    /* Arm the injected group with its completion interrupt. From here
     * the timer triggers a conversion once per PWM period and the
     * callback above runs each time, whether or not a loop function is
     * installed. */
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) {
        return 0u;
    }

    /* Now that conversions are actually happening, measure zero. */
    if (collect_zero_references() == 0u) {
        return 0u;
    }

    /* Flush the encoder pipeline so the first angle the loop sees is a
     * real measurement rather than whatever the sensor had queued. */
    encoder_prime_pipeline();

    self->initialised = 1u;
    return 1u;
}

/* ------------------------------------------------------------------
 * Running
 * ------------------------------------------------------------------ */

void loop_set_function(loop_function_t loop_function)
{
    /* Stop calling the old function before swapping the pointer, so an
     * interrupt cannot land between the two and call a half-written
     * value. */
    uint8_t was_running = self->running;

    self->running            = 0u;
    self->installed_function = loop_function;
    self->running            = was_running;
}

uint8_t loop_start(void)
{
    if (self->initialised == 0u) {
        return 0u;
    }
    self->iteration_count = 0u;
    self->overrun_count   = 0u;
    self->running         = 1u;
    return 1u;
}

void loop_stop(void)
{
    self->running = 0u;
}

uint8_t loop_is_running(void)
{
    return self->running;
}

/* ------------------------------------------------------------------
 * Reporting
 * ------------------------------------------------------------------ */

uint8_t loop_is_initialised(void)
{
    return self->initialised;
}

uint32_t loop_get_duration_us(void)
{
    return self->last_duration_cycles / CPU_CYCLES_PER_MICROSECOND;
}

uint32_t loop_get_iteration_count(void)
{
    return self->iteration_count;
}

uint32_t loop_get_overrun_count(void)
{
    return self->overrun_count;
}

void loop_set_sample_point(uint16_t compare_value)
{
    if (compare_value > LOOP_TIMER_PERIOD_COUNTS) {
        compare_value = LOOP_TIMER_PERIOD_COUNTS;
    }
    self->sample_point = compare_value;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, self->sample_point);
}

uint16_t loop_get_sample_point(void)
{
    return self->sample_point;
}
