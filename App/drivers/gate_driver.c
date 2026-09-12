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

/* Dead time as parts per thousand of the switching period, from
 * gate_driver.h -- public there because measurements that work backwards
 * from a duty to an applied voltage need it too. */
#define DEAD_TIME_PER_MILLE GATE_DRIVER_DEAD_TIME_PER_MILLE

/* Current at which the dead time correction reaches full value, in
 * milliamps. Below this it is scaled down in proportion.
 *
 * A plain sign test fails here. The current sensor resolves about 40
 * milliamps per count and carries a few counts of noise, so a reading
 * near zero changes sign at random, and the correction would then flip
 * between plus and minus a full dead time -- a swing of 48 parts per
 * thousand, which is larger than the whole command. Scaling through the
 * zero crossing turns a reversal into a proportionally small wobble. */
#define DEAD_TIME_FULL_MA 800

/* Milliamps to amps, and millivolts to volts. */
#define MILLI_TO_UNIT 0.001f

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

uint8_t gate_driver_apply_voltage(uint8_t  phase,
                                  float    phase_voltage,
                                  uint16_t bus_mv,
                                  int32_t  phase_current_ma)
{
    if (phase >= GATE_DRIVER_PHASE_COUNT) {
        return 0u;
    }

    /* A bridge cannot drive a phase below the negative rail, so the
     * demand is applied as a deviation either side of half duty. At half
     * on all three phases the terminals sit at the same potential and no
     * current flows, which is why half is the resting point. */
    int32_t duty = (int32_t)(GATE_DRIVER_DUTY_SCALE / 2u);

    if (bus_mv == 0u) {
        /* Nothing sensible can be computed without a bus measurement,
         * and dividing by it would fault. Resting duty applies no
         * voltage, which is the safe answer. */
        return gate_driver_set_duty(phase, (uint16_t)duty);
    }

    /* Express the demand as a fraction of the bus, in the same parts per
     * thousand as the duty. Deriving it from the measured bus rather
     * than a nominal figure means the voltage actually applied stays
     * correct as the supply sags. */
    int32_t commanded = (int32_t)((phase_voltage
                                   * (float)GATE_DRIVER_DUTY_SCALE)
                                  / ((float)bus_mv * MILLI_TO_UNIT));
    duty += commanded;

    /* Dead time correction.
     *
     * Current flowing OUT of the terminal -- negative here -- holds the
     * phase toward the positive rail during the gap, so it spent longer
     * high than commanded and the correction is downward. Current
     * flowing in does the opposite.
     *
     * Scaled in proportion below the threshold rather than switched on a
     * sign test, so that noise near a zero crossing produces a small
     * wobble instead of a full reversal. */
    int32_t scaled = phase_current_ma;

    if (scaled > DEAD_TIME_FULL_MA) {
        scaled = DEAD_TIME_FULL_MA;
    } else if (scaled < -DEAD_TIME_FULL_MA) {
        scaled = -DEAD_TIME_FULL_MA;
    }

    int32_t correction = ((int32_t)DEAD_TIME_PER_MILLE * scaled)
                         / DEAD_TIME_FULL_MA;

    /* The correction may never exceed what was actually commanded.
     *
     * WHY THIS BOUND IS NOT OPTIONAL
     *
     *   The correction takes its sign from the measured current, and the
     *   voltage it admits sustains that current. That is a loop, and
     *   whether it is stable depends on how large the correction is next
     *   to the command.
     *
     *   On this board the full correction is 24 parts per thousand of
     *   the bus -- roughly 290 millivolts, which across a 54 milliohm
     *   path is over five amps. So on a low resistance winding the
     *   correction alone can drive far more current than any sensible
     *   command, and once current flows in either direction the
     *   correction holds it there. The motor draws heavily, produces no
     *   useful torque, and which way it latched is arbitrary.
     *
     *   Bounding the correction by the commanded value breaks the loop.
     *   It can still restore voltage the dead time removed, which is its
     *   entire purpose, but it can no longer create voltage that was
     *   never asked for -- so with nothing commanded, nothing happens.
     */
    int32_t bound = (commanded < 0) ? -commanded : commanded;

    if (correction > bound) {
        correction = bound;
    } else if (correction < -bound) {
        correction = -bound;
    }

    duty += correction;

    /* Clamped here too, before gate_driver_set_duty's own clamp to
     * GATE_DRIVER_DUTY_MAXIMUM. That clamp alone would still produce a
     * safe final duty, but duty is signed here and set_duty's parameter
     * is not -- an unclamped negative value would wrap to a huge
     * positive one on the cast below instead of saturating to zero. */
    if (duty < 0) {
        duty = 0;
    } else if (duty > (int32_t)GATE_DRIVER_DUTY_SCALE) {
        duty = (int32_t)GATE_DRIVER_DUTY_SCALE;
    }

    return gate_driver_set_duty(phase, (uint16_t)duty);
}
