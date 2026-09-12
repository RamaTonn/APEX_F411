#include "gate_driver.h"
#include "main.h"

/* The timer handle CubeMX creates in main.c. */
extern TIM_HandleTypeDef htim3;

/* ------------------------------------------------------------------
 * Phase table
 *
 * Everything that differs between the three phases, in one place. Adding
 * the shutdown pin and its port here means no function below contains a
 * switch statement on the phase number.
 *
 * timer_channel   which TIM3 compare channel drives this phase's PWM
 * shutdown_port   GPIO port of the IR2104 SD pin
 * shutdown_pin    GPIO pin of the same
 * ------------------------------------------------------------------ */

static const struct {
    uint32_t      timer_channel;
    GPIO_TypeDef *shutdown_port;
    uint16_t      shutdown_pin;
} phase_table[GATE_DRIVER_PHASE_COUNT] = {
    [GATE_DRIVER_PHASE_A] = { TIM_CHANNEL_1, ENA_GPIO_Port, ENA_Pin },
    [GATE_DRIVER_PHASE_B] = { TIM_CHANNEL_2, ENB_GPIO_Port, ENB_Pin },
    [GATE_DRIVER_PHASE_C] = { TIM_CHANNEL_3, ENC_GPIO_Port, ENC_Pin },
};

/* The bridge instance handed to gate_driver_init(). Cached here because
 * every other function in this file needs it and none of them can be
 * handed a context of their own -- same reasoning as control.c's static
 * motor pointer. */
static gate_driver_t *self;

/* ------------------------------------------------------------------
 * Duty conversion
 * ------------------------------------------------------------------ */

/* Convert tenths of a percent into a timer compare value.
 *
 * The timer counts from 0 to ARR and the output is high while the count
 * is below the compare register, so compare value divided by (ARR + 1)
 * is the fraction of each period spent with the high side on.
 *
 * ARR is read from the peripheral rather than hard-coded, so changing
 * the PWM frequency in CubeMX does not silently break the duty scale.
 *
 * @param duty_per_mille  0..1000
 * @return the value to write to a compare register */
static uint32_t duty_to_compare_value(uint16_t duty_per_mille)
{
    uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1u;

    /* Multiply before dividing, or integer division throws away the
     * fraction and every duty below 100% becomes zero. The product is at
     * most 1000 * 3125 = 3,125,000, well inside 32 bits. */
    return ((uint32_t)duty_per_mille * period_counts)
           / GATE_DRIVER_DUTY_SCALE;
}

/* Write a duty straight to the hardware, with no clamping and no
 * bookkeeping. Used both by the public setter and by the enable sequence
 * when it forces duty to zero temporarily.
 *
 * @param phase           already validated
 * @param duty_per_mille  already clamped */
static void write_duty_to_timer(uint8_t phase, uint16_t duty_per_mille)
{
    __HAL_TIM_SET_COMPARE(&htim3,
                          phase_table[phase].timer_channel,
                          duty_to_compare_value(duty_per_mille));
}

/* Drive one shutdown pin.
 *
 * The IR2104's SD input is active low, so releasing the shutdown means
 * driving the pin HIGH. This is the only place in the module that knows
 * that, which is why nothing else has to remember it.
 *
 * @param phase    already validated
 * @param running  1 to let the phase switch, 0 to shut it down */
static void write_shutdown_pin(uint8_t phase, uint8_t running)
{
    HAL_GPIO_WritePin(phase_table[phase].shutdown_port,
                      phase_table[phase].shutdown_pin,
                      (running != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ------------------------------------------------------------------
 * Public interface
 * ------------------------------------------------------------------ */

void gate_driver_init(gate_driver_t *g)
{
    self = g;

    /* Shut everything down BEFORE starting the timer. If the timer began
     * generating edges while a shutdown pin happened to be released, the
     * bridge would start switching before any duty had been set. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        write_shutdown_pin(phase, 0u);
        self->phase_enabled[phase]  = 0u;
        self->requested_duty[phase] = 0u;
    }

    /* Starting a PWM channel makes the timer drive its pin. With every
     * shutdown pin held low the drivers ignore these edges entirely, so
     * this is safe with a motor and bus voltage connected. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        write_duty_to_timer(phase, 0u);
        HAL_TIM_PWM_Start(&htim3, phase_table[phase].timer_channel);
    }
}

uint8_t gate_driver_set_duty(uint8_t phase, uint16_t duty_per_mille)
{
    if (phase >= GATE_DRIVER_PHASE_COUNT) {
        return 0u;
    }

    /* Clamp rather than reject. A control loop that saturates should keep
     * running at the limit, not fail and leave the bridge in whatever
     * state it was in. */
    if (duty_per_mille > GATE_DRIVER_DUTY_MAXIMUM) {
        duty_per_mille = GATE_DRIVER_DUTY_MAXIMUM;
    }

    self->requested_duty[phase] = duty_per_mille;
    write_duty_to_timer(phase, duty_per_mille);

    return 1u;
}

uint8_t gate_driver_enable_phase(uint8_t phase)
{
    uint16_t duty_to_restore;

    if (phase >= GATE_DRIVER_PHASE_COUNT) {
        return 0u;
    }

    duty_to_restore = self->requested_duty[phase];

    /* Zero duty means the low-side FET conducts continuously, which pulls
     * the phase output to ground -- and that is the only condition under
     * which the bootstrap capacitor charges. */
    write_duty_to_timer(phase, 0u);

    write_shutdown_pin(phase, 1u);
    self->phase_enabled[phase] = 1u;

    HAL_Delay(GATE_DRIVER_BOOTSTRAP_CHARGE_MS);

    /* The capacitor now has enough charge to turn the high-side FET fully
     * on, so switching can begin. */
    write_duty_to_timer(phase, duty_to_restore);

    return 1u;
}

uint8_t gate_driver_disable_phase(uint8_t phase)
{
    if (phase >= GATE_DRIVER_PHASE_COUNT) {
        return 0u;
    }

    write_shutdown_pin(phase, 0u);
    self->phase_enabled[phase] = 0u;

    return 1u;
}

void gate_driver_disable_all(void)
{
    /* This is the fault handler's escape hatch (see the header), so it
     * has to survive being called before gate_driver_init() ever ran --
     * e.g. a CubeMX peripheral init failing during startup. The shutdown
     * pins are what actually stop the FETs and don't need `self` at all;
     * only the bookkeeping does, so that part is skipped rather than
     * dereferencing a pointer that was never set. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        /* Shutdown pin first, duty second. The pin is what actually stops
         * the FETs; zeroing duty afterwards only makes sure that a later
         * enable starts from rest rather than resuming at speed. */
        write_shutdown_pin(phase, 0u);
        write_duty_to_timer(phase, 0u);

        if (self != NULL) {
            self->phase_enabled[phase]  = 0u;
            self->requested_duty[phase] = 0u;
        }
    }
}

uint16_t gate_driver_get_duty(uint8_t phase)
{
    if (phase >= GATE_DRIVER_PHASE_COUNT) {
        return 0u;
    }
    return self->requested_duty[phase];
}

uint8_t gate_driver_is_enabled(uint8_t phase)
{
    if (phase >= GATE_DRIVER_PHASE_COUNT) {
        return 0u;
    }
    return self->phase_enabled[phase];
}
