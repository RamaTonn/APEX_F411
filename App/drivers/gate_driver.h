/*
 * gate_driver.h
 *
 * Drives the three IR2104 half-bridge drivers (U3 phase A, U4 phase B,
 * U1 phase C) and through them the six FDMS86300DC MOSFETs.
 *
 * WIRING
 *
 *   PWM inputs     PA6 = TIM3_CH1 = phase A
 *                  PA7 = TIM3_CH2 = phase B
 *                  PB0 = TIM3_CH3 = phase C
 *
 *   Shutdown       PA3 = EN_A, PA4 = EN_B, PA5 = EN_C
 *                  These go to the driver's SD pin, which is ACTIVE LOW.
 *                  The pin must be driven HIGH for the phase to run.
 *                  GPIO outputs reset low, so the bridge is shut down
 *                  out of reset without firmware doing anything.
 *
 * HOW ONE PHASE BEHAVES
 *
 *   enable  duty      result
 *   0       anything  both FETs off, output floating
 *   1       0%        low side on continuously, output at ground
 *   1       50%       output square wave, half bus voltage on average
 *   1       90%       output near bus voltage on average
 *
 *   The driver produces the complementary pair itself with about 520 ns
 *   of dead time inserted. No combination of pin states can turn both
 *   FETs on at once, so shoot-through cannot be caused from firmware.
 *
 * THE BOOTSTRAP CONSTRAINT
 *
 *   The high-side gate needs a voltage above the bus rail to turn on.
 *   That comes from a 0.1 uF capacitor charged from +12 V through 10 ohms
 *   and a diode, and it only charges while the phase output is pulled to
 *   ground -- that is, while the LOW side is conducting.
 *
 *   Two rules follow, and this module enforces both:
 *
 *   1. A phase must spend a few milliseconds at zero duty after being
 *      enabled, before any high-side switching is asked for. Turning the
 *      high side on with an empty bootstrap capacitor can leave the FET
 *      part-way on, dissipating heavily, which destroys it.
 *      gate_driver_enable_phase() handles this.
 *
 *   2. Duty can never reach 100%, or the capacitor never recharges. The
 *      ceiling here is 90%, which leaves about 3 microseconds of
 *      low-side conduction per cycle against a roughly 1 microsecond
 *      charging time constant.
 */

#ifndef GATE_DRIVER_H_
#define GATE_DRIVER_H_

#include <stdint.h>

#define GATE_DRIVER_PHASE_A 0U
#define GATE_DRIVER_PHASE_B 1U
#define GATE_DRIVER_PHASE_C 2U
#define GATE_DRIVER_PHASE_COUNT 3U

/* Duty is expressed in tenths of a percent, so 0..1000 would be 0..100%.
 * Integers throughout -- no floating point anywhere in the control path. */
#define GATE_DRIVER_DUTY_SCALE 1000U

/* Highest duty that still leaves the bootstrap capacitor time to
 * recharge each cycle. Requests above this are clamped, not rejected. */
#define GATE_DRIVER_DUTY_MAXIMUM 900U

/* How long to hold a newly enabled phase at zero duty so the low-side
 * FET can charge the bootstrap capacitor. The RC is about 1 us, so this
 * is thousands of time constants -- generous on purpose, since it only
 * happens once per enable. */
#define GATE_DRIVER_BOOTSTRAP_CHARGE_MS 5U

/* Dead time to correct for, in nanoseconds: the driver's own delay plus
 * the transistors' turn-on time through the 10 ohm gate resistors.
 * Measured on this board rather than taken from a datasheet. */
#define GATE_DRIVER_DEAD_TIME_NS 750U

/* The switching period in nanoseconds. The timer counts to 1499 and back
 * at 96 MHz, so 2998 counts, which is 31229 nanoseconds. */
#define GATE_DRIVER_PERIOD_NS 31229U

/* The voltage the dead time removes before any of it reaches the
 * winding, as parts per thousand of the bus -- the same units duty is
 * expressed in. Derived rather than written as a constant, so changing
 * either figure above keeps it correct. Comes to 24.
 *
 * Public because anything working BACKWARDS from a duty to the voltage
 * the winding actually saw has to account for it. A measurement that
 * takes two points at the same current polarity gets it for free, since
 * the loss is identical at both and cancels in the difference; one that
 * steps from zero current has to subtract it explicitly. */
#define GATE_DRIVER_DEAD_TIME_PER_MILLE \
    ((GATE_DRIVER_DEAD_TIME_NS * 1000U) / GATE_DRIVER_PERIOD_NS)

/*
 * One bridge. Owned by main.c and initialised once; every other function
 * below acts on that one instance, which this module keeps a pointer to
 * internally -- there is exactly one bridge on this board, so nothing
 * outside this file ever needs to touch the struct's fields directly.
 *
 * Which timer and which shutdown pins drive which phase is board wiring,
 * not runtime state, so it stays a compile-time table in gate_driver.c
 * rather than living here: porting to a board with the bridge on a
 * different timer means editing that table, not this struct.
 */
typedef struct {
    uint16_t requested_duty[GATE_DRIVER_PHASE_COUNT];
    uint8_t  phase_enabled[GATE_DRIVER_PHASE_COUNT];
} gate_driver_t;

/**
 * Start the PWM timer with every phase at zero duty and every phase
 * disabled. Safe to call with a motor and bus voltage connected.
 *
 * @param g  the bridge instance this board drives
 */
void gate_driver_init(gate_driver_t *g);

/**
 * Set the duty for one phase.
 *
 * Takes effect on the timer's next update, whether or not the phase is
 * currently enabled -- so a duty set while disabled is what the phase
 * will start with when enabled.
 *
 * @param phase          GATE_DRIVER_PHASE_A, _B or _C
 * @param duty_per_mille 0..1000, tenths of a percent of high-side on
 *                       time. Values above GATE_DRIVER_DUTY_MAXIMUM are
 *                       silently clamped down to it.
 * @return 1 on success, 0 if the phase index was invalid
 */
uint8_t gate_driver_set_duty(uint8_t phase, uint16_t duty_per_mille);

/**
 * Enable one phase, charging its bootstrap capacitor first.
 *
 * Forces duty to zero, raises the shutdown pin so the low-side FET
 * conducts, BLOCKS for GATE_DRIVER_BOOTSTRAP_CHARGE_MS while the
 * capacitor charges, then restores the duty that was previously set.
 *
 * The block is deliberate. Making this asynchronous would mean the
 * caller could raise duty before charging finished, which is exactly the
 * failure this function exists to prevent.
 *
 * @param phase  GATE_DRIVER_PHASE_A, _B or _C
 * @return 1 on success, 0 if the phase index was invalid
 */
uint8_t gate_driver_enable_phase(uint8_t phase);

/**
 * Disable one phase. Both its FETs turn off and the output floats. The
 * duty setting is remembered but has no effect while disabled.
 *
 * @param phase  GATE_DRIVER_PHASE_A, _B or _C
 * @return 1 on success, 0 if the phase index was invalid
 */
uint8_t gate_driver_disable_phase(uint8_t phase);

/**
 * Shut every phase down and zero every duty. The bridge stops driving
 * entirely and the motor coasts.
 *
 * Cannot fail, takes no arguments, and is safe to call from anywhere --
 * so it is what any fault handler should call.
 */
void gate_driver_disable_all(void);

/**
 * @param phase  GATE_DRIVER_PHASE_A, _B or _C
 * @return the duty most recently set for that phase, or 0 for an invalid
 *         phase index
 */
uint16_t gate_driver_get_duty(uint8_t phase);

/**
 * @param phase  GATE_DRIVER_PHASE_A, _B or _C
 * @return 1 if the phase is currently enabled, 0 if disabled or invalid
 */
uint8_t gate_driver_is_enabled(uint8_t phase);

/**
 * Convert a voltage demand for one phase into a duty and drive it,
 * including dead-time compensation.
 *
 * A bridge cannot drive a phase below the negative rail, so the demand
 * is applied as a deviation either side of half duty -- at half on all
 * three phases the terminals sit at the same potential and no current
 * flows, which is why half is the resting point. The demand is expressed
 * as a fraction of the measured bus rather than a nominal voltage, so
 * the voltage actually applied stays correct as the supply sags.
 *
 * Dead-time compensation restores the fraction of each switching period
 * that the driver's own turn-on delay and dead time otherwise steal from
 * the commanded voltage. It is scaled down below a threshold current
 * rather than switched on a sign test, so noise near a zero crossing
 * produces a small wobble instead of the correction flipping a full dead
 * time from one sign to the other. The correction is also never allowed
 * to exceed what was actually commanded -- see gate_driver.c for why
 * that bound is load-bearing rather than a nicety.
 *
 * @param phase             GATE_DRIVER_PHASE_A, _B or _C
 * @param phase_voltage     what this phase should produce, in volts,
 *                          either side of the resting point
 * @param bus_mv            measured bus voltage, millivolts
 * @param phase_current_ma  this phase's current in milliamps, signed;
 *                          positive means flowing into the terminal, used
 *                          only for dead-time compensation
 * @return 1 on success, 0 if the phase index was invalid
 */
uint8_t gate_driver_apply_voltage(uint8_t  phase,
                                  float    phase_voltage,
                                  uint16_t bus_mv,
                                  int32_t  phase_current_ma);

#endif /* GATE_DRIVER_H_ */
