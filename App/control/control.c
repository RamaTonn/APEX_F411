#include "control.h"

#include "encoder.h"
#include "motor.h"
#include "main.h"
#include "telemetry.h"

/* Handles CubeMX creates in main.c. */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

/* One ADC count is 40.28 milliamps: a 1 milliohm shunt into an INA240A1
 * with a gain of 20 gives 20 millivolts per amp, and the ADC resolves
 * 0.806 millivolts per count. Held as a fraction so the decimal part
 * survives integer arithmetic. */
#define COUNTS_TO_MILLIAMPS_NUMERATOR   4028
#define COUNTS_TO_MILLIAMPS_DENOMINATOR 100

/* Samples averaged when establishing the no-current readings.
 *
 * At the 32 kHz loop rate these arrive in about two milliseconds, so
 * this costs nothing at startup. 64 samples reduces a couple of counts
 * of noise to a fraction of a count. */
#define ZERO_CALIBRATION_SAMPLES 64U

/* How long to wait for calibration samples to arrive before giving up.
 *
 * 64 samples at 32 kHz take 2 milliseconds, so 200 is fifty times the
 * margin needed. The point is not to be generous but to fail quickly and
 * definitely if the trigger is not working at all, rather than hanging. */
#define ZERO_CALIBRATION_TIMEOUT_MS 200U

/* CPU cycles per microsecond, for turning cycle counter readings into
 * something readable. The core runs at 96 MHz. */
#define CPU_CYCLES_PER_MICROSECOND 96U

/* ------------------------------------------------------------------
 * State
 *
 * Everything written by the interrupt and read by the main loop is
 * volatile. Without it the compiler is entitled to cache these in
 * registers, and the main loop would spin forever on a stale value.
 * ------------------------------------------------------------------ */

/* Most recent phase currents in milliamps, published by the interrupt
 * for anything in the main loop that wants them. */
static volatile int32_t latest_current_a_ma;
static volatile int32_t latest_current_b_ma;

/* The motor this loop drives. Set by control_init() and used by the
 * interrupt, which has no way to be handed a context of its own. */
static motor_t *motor;

/* How many times the interrupt has run since control_start(). */
static volatile uint32_t iteration_count;

/* How many times a new trigger arrived while the previous iteration was
 * still executing. Any non-zero value means the control function is too
 * slow for the loop rate. */
static volatile uint32_t overrun_count;

/* Duration of the last iteration, in CPU cycles. */
static volatile uint32_t last_duration_cycles;

/* Set while the interrupt body is executing, so that a second trigger
 * arriving mid-iteration can be detected rather than allowed to run on
 * top of the first. */
static volatile uint8_t iteration_in_progress;

/* The function to call each period, or NULL for none. */
static volatile control_function_t installed_function;

/* Non-zero when the installed function should actually be called.
 * Sampling continues regardless, so currents stay live even when no
 * control algorithm is running. */
static volatile uint8_t loop_running;

/* ------------------------------------------------------------------
 * Zero-current calibration state
 *
 * Calibration happens through the interrupt rather than by polling,
 * because the injected conversions are triggered by the timer and
 * nothing else can make them happen.
 *
 * An earlier version of this file tried to start an injected conversion
 * in software and poll for it. That cannot work with a hardware trigger
 * configured: the software start arms the group but never fires it, so
 * the poll times out and initialisation fails -- silently, since the
 * only symptom is that both currents read exactly zero forever.
 * ------------------------------------------------------------------ */

/* Non-zero while the interrupt should be collecting raw samples for
 * calibration instead of running the control function. */
static volatile uint8_t calibrating;

/* Running totals of raw conversion results during calibration. 64
 * samples of a 12-bit value reach at most 262080, well inside 32 bits. */
static volatile uint32_t calibration_total_a;
static volatile uint32_t calibration_total_b;

/* How many calibration samples have been collected so far. */
static volatile uint16_t calibration_samples_taken;

/* What the current sensors read with no current flowing. Every current
 * measurement is a difference from these.
 *
 * Initialised to mid-scale so that a failed calibration produces
 * plausible-looking rather than absurd values -- though control_init()
 * returning zero is the signal that should actually be acted on. */
static uint16_t zero_counts_a = 2048u;
static uint16_t zero_counts_b = 2048u;

/* Set once initialisation has fully succeeded. control_start() refuses
 * until then, because a loop whose zero references were never measured
 * would report currents offset by up to two amps. */
static uint8_t initialised;

/* Compare value for timer channel 4, which is what triggers the injected
 * conversions and therefore decides when in the PWM period the currents
 * are sampled. */
static uint16_t sample_point = CONTROL_DEFAULT_SAMPLE_POINT;

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
 * Current conversion
 * ------------------------------------------------------------------ */

/* Turn a raw injected conversion result into milliamps.
 *
 * @param raw_counts      the value from the injected data register,
 *                        0 to 4095
 * @param zero_reference  that channel's no-current reading
 * @return signed current in milliamps; positive means current flowing
 *         into the motor terminal */
static inline int32_t counts_to_milliamps(uint32_t raw_counts,
                                          uint16_t zero_reference)
{
    /* Both operands are cast to signed before subtracting. Left
     * unsigned, a reading below the reference would wrap to an enormous
     * positive number instead of going negative. */
    int32_t counts_from_zero = (int32_t)raw_counts - (int32_t)zero_reference;

    return (counts_from_zero * COUNTS_TO_MILLIAMPS_NUMERATOR)
           / COUNTS_TO_MILLIAMPS_DENOMINATOR;
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
     * means the control function takes longer than a PWM period.
     * Counting it and returning is better than running two iterations on
     * top of each other, which would corrupt whatever state the control
     * function keeps. */
    if (iteration_in_progress != 0u) {
        overrun_count++;
        return;
    }
    iteration_in_progress = 1u;

    /* Injected rank 1 is channel 0 (phase A) and rank 2 is channel 1
     * (phase B), in the order the injected sequence was configured in
     * CubeMX. */
    uint32_t raw_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_b = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    /* During calibration the raw values are accumulated and nothing else
     * happens. They cannot be converted to milliamps yet, because the
     * references that conversion needs are exactly what is being
     * measured. */
    if (calibrating != 0u) {
        if (calibration_samples_taken < ZERO_CALIBRATION_SAMPLES) {
            calibration_total_a += raw_a;
            calibration_total_b += raw_b;
            calibration_samples_taken++;
        }
        iteration_in_progress = 0u;
        return;
    }

    int32_t current_a = counts_to_milliamps(raw_a, zero_counts_a);
    int32_t current_b = counts_to_milliamps(raw_b, zero_counts_b);

    latest_current_a_ma = current_a;
    latest_current_b_ma = current_b;

    /* The rotor angle is updated before the control function runs, so
     * that anything closing a loop acts on this period's position rather
     * than the previous one.
     *
     * The pipelined read costs about three and a half microseconds. That
     * is paid every period whether or not anything uses the angle, which
     * is the price of having it available the moment a control function
     * is installed. */
    (void)motor_update(motor);

    if ((loop_running != 0u) && (installed_function != NULL)) {
        installed_function(current_a, current_b);
    }

    /* Captured every period regardless of whether anything is driving
     * the motor.
     *
     * Streaming the currents with the bridge idle is how the sensors get
     * checked before anything moves, so this must not depend on a
     * control function being installed -- and it runs after that
     * function so that any angle it published belongs to this period
     * rather than the previous one. */
    telemetry_capture(current_a, current_b);

    iteration_count++;

    /* Unsigned subtraction gives the correct elapsed count even when the
     * cycle counter wraps between the two readings. */
    last_duration_cycles = DWT->CYCCNT - start_cycles;

    iteration_in_progress = 0u;
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
    calibration_total_a       = 0u;
    calibration_total_b       = 0u;
    calibration_samples_taken = 0u;
    calibrating               = 1u;

    uint32_t start_tick = HAL_GetTick();

    while (calibration_samples_taken < ZERO_CALIBRATION_SAMPLES) {
        /* Subtracting the ticks rather than comparing against a sum
         * keeps this correct across the 49-day tick wrap. */
        if ((HAL_GetTick() - start_tick) > ZERO_CALIBRATION_TIMEOUT_MS) {
            calibrating = 0u;
            return 0u;
        }
    }

    calibrating = 0u;

    zero_counts_a = (uint16_t)(calibration_total_a / ZERO_CALIBRATION_SAMPLES);
    zero_counts_b = (uint16_t)(calibration_total_b / ZERO_CALIBRATION_SAMPLES);

    return 1u;
}

uint8_t control_init(motor_t *m)
{
    /* Remember the motor the interrupt updates each period. */
    motor = m;

    initialised           = 0u;
    loop_running          = 0u;
    installed_function    = NULL;
    iteration_count       = 0u;
    overrun_count         = 0u;
    iteration_in_progress = 0u;

    cycle_counter_enable();

    /* Position the sample instant before starting anything, so the very
     * first conversion lands where intended. */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, sample_point);

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
     * callback above runs each time, whether or not a control function
     * is installed. */
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

    initialised = 1u;
    return 1u;
}

/* ------------------------------------------------------------------
 * Running
 * ------------------------------------------------------------------ */

void control_set_function(control_function_t control_function)
{
    /* Stop calling the old function before swapping the pointer, so an
     * interrupt cannot land between the two and call a half-written
     * value. */
    uint8_t was_running = loop_running;

    loop_running       = 0u;
    installed_function = control_function;
    loop_running       = was_running;
}

uint8_t control_start(void)
{
    if (initialised == 0u) {
        return 0u;
    }
    iteration_count = 0u;
    overrun_count   = 0u;
    loop_running    = 1u;
    return 1u;
}

void control_stop(void)
{
    loop_running = 0u;
}

uint8_t control_is_running(void)
{
    return loop_running;
}

/* ------------------------------------------------------------------
 * Reporting
 * ------------------------------------------------------------------ */

void control_get_currents(int32_t *current_a_out, int32_t *current_b_out)
{
    if (current_a_out != NULL) {
        *current_a_out = latest_current_a_ma;
    }
    if (current_b_out != NULL) {
        *current_b_out = latest_current_b_ma;
    }
}

void control_get_zero_counts(uint16_t *zero_a_out, uint16_t *zero_b_out)
{
    if (zero_a_out != NULL) {
        *zero_a_out = zero_counts_a;
    }
    if (zero_b_out != NULL) {
        *zero_b_out = zero_counts_b;
    }
}

uint8_t control_is_initialised(void)
{
    return initialised;
}

uint32_t control_get_duration_us(void)
{
    return last_duration_cycles / CPU_CYCLES_PER_MICROSECOND;
}

uint32_t control_get_iteration_count(void)
{
    return iteration_count;
}

uint32_t control_get_overrun_count(void)
{
    return overrun_count;
}

void control_set_sample_point(uint16_t compare_value)
{
    if (compare_value > CONTROL_TIMER_PERIOD_COUNTS) {
        compare_value = CONTROL_TIMER_PERIOD_COUNTS;
    }
    sample_point = compare_value;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, sample_point);
}

uint16_t control_get_sample_point(void)
{
    return sample_point;
}
