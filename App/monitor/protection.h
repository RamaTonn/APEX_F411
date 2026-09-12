/*
 * protection.h
 *
 * Watches the sensors and shuts the bridge down when something goes
 * wrong. Everything that spins the motor continuously must have this
 * running.
 *
 * WHAT IT WATCHES, AND WHY EACH ONE MATTERS
 *
 *   Instantaneous current   A short circuit, a wrongly commutated phase,
 *                           or a stalled rotor all show up as current
 *                           far above normal within one PWM cycle. This
 *                           limit is set high, above anything the motor
 *                           legitimately draws, and exists to catch
 *                           faults rather than to regulate anything.
 *
 *   Sustained current       A current that is not dangerous for a
 *                           moment is dangerous for a minute, because
 *                           the winding and the FETs heat up. This limit
 *                           is set lower and only trips if the current
 *                           stays above it for a while.
 *
 *   Bus overvoltage         A decelerating motor pushes energy back into
 *                           the supply. With no load to absorb it the
 *                           bus rises, and it can rise past what the
 *                           capacitors and FETs tolerate.
 *
 *   Bus undervoltage        The gate drivers get their supply from the
 *                           bus through the buck converter. If the bus
 *                           collapses, the drivers can end up with just
 *                           enough voltage to turn a FET part-way on,
 *                           which dissipates heavily.
 *
 * SAMPLING: ONE SAMPLE, NOT AN AVERAGE
 *
 *   Phase current is not flat within a PWM cycle -- it ramps up while
 *   the high side conducts and decays while it freewheels. A single ADC
 *   sample therefore lands at an arbitrary point on that ripple.
 *
 *   For measurement that is a problem, and the characterisation code
 *   averages to remove it. For protection it is an advantage: a sample
 *   that happens to catch the peak trips slightly early, which is the
 *   safe direction to be wrong in. Averaging would smooth away the very
 *   transient this is meant to catch, and would cost most of a
 *   millisecond that a safety check cannot spare.
 *
 * FAULTS LATCH
 *
 *   Once tripped, the bridge stays disabled until something explicitly
 *   clears the fault. It does not recover by itself.
 *
 *   Automatic recovery would be worse than useless here: the usual
 *   result is a board that re-enables into the same fault, over and
 *   over, several times a second. That both hides the problem and does
 *   real damage. A latched fault stops everything and stays stopped
 *   until a person looks at it.
 */

#ifndef PROTECTION_H_
#define PROTECTION_H_

#include <stdint.h>

/* Fault codes, one bit each, so several can be reported together. */
#define PROTECTION_FAULT_NONE               0x00U
#define PROTECTION_FAULT_OVERCURRENT_A      0x01U
#define PROTECTION_FAULT_OVERCURRENT_B      0x02U
#define PROTECTION_FAULT_SUSTAINED_CURRENT  0x04U
#define PROTECTION_FAULT_BUS_OVERVOLTAGE    0x08U
#define PROTECTION_FAULT_BUS_UNDERVOLTAGE   0x10U
#define PROTECTION_FAULT_SENSOR_FAILED      0x20U

/* Defaults, chosen conservatively for an unknown motor on a 12 volt
 * supply. Every one of these should be revisited once the motor's
 * ratings are known. */
#define PROTECTION_DEFAULT_INSTANT_LIMIT_MA   8000
#define PROTECTION_DEFAULT_SUSTAINED_LIMIT_MA 3000
#define PROTECTION_DEFAULT_SUSTAINED_WINDOW_MS 500U
#define PROTECTION_DEFAULT_BUS_MAXIMUM_MV     18000U
#define PROTECTION_DEFAULT_BUS_MINIMUM_MV      8000U

/* Bus voltage conversion takes about 21 microseconds because the divider
 * is high impedance, against roughly 3 microseconds for a current
 * channel. Bus voltage also cannot change quickly, so it is checked once
 * every this many calls rather than every call. */
#define PROTECTION_BUS_CHECK_INTERVAL 64U

/**
 * Set up the supervisor and measure the no-current sensor readings.
 *
 * Disables the bridge, then averages many samples of both current
 * channels to establish what zero looks like. Every current check
 * afterwards is a difference from those readings, so this must run
 * before protection_poll() is useful -- and it must run with nothing
 * driving the motor.
 *
 * @return 1 on success, 0 if the ADC did not respond, in which case the
 *         supervisor will report a sensor fault on its first poll rather
 *         than silently using wrong references
 */
uint8_t protection_init(void);

/**
 * Check the sensors and trip if anything is out of bounds.
 *
 * Call from the main loop, as often as is convenient. Every call reads
 * both current channels; bus voltage is read on one call in
 * PROTECTION_BUS_CHECK_INTERVAL.
 *
 * Does nothing if a fault is already latched, so calling it in a tight
 * loop after a trip costs almost nothing.
 *
 * @return the fault bitmask, PROTECTION_FAULT_NONE when all is well
 */
uint8_t protection_poll(void);

/**
 * Adjust the current limits.
 *
 * @param instantaneous_ma  trips on a single sample above this. Set well
 *                          above the motor's peak working current, since
 *                          the sampled value includes PWM ripple.
 * @param sustained_ma      trips if current stays above this for longer
 *                          than the window. Set near the motor's
 *                          continuous rating.
 * @param window_ms         how long current may exceed sustained_ma
 *                          before tripping
 * @return 1 on success, 0 if instantaneous_ma is not greater than
 *         sustained_ma, which would make the sustained check unreachable
 */
uint8_t protection_set_current_limits(int32_t  instantaneous_ma,
                                      int32_t  sustained_ma,
                                      uint32_t window_ms);

/**
 * Adjust the bus voltage limits.
 *
 * @param minimum_mv  trips below this
 * @param maximum_mv  trips above this
 * @return 1 on success, 0 if minimum is not below maximum
 */
uint8_t protection_set_bus_limits(uint32_t minimum_mv, uint32_t maximum_mv);

/**
 * Enable or disable the supervisor entirely.
 *
 * When disabled, protection_poll() does nothing at all: no limits are
 * checked and nothing will trip. Any fault already latched stays
 * latched, so disabling does not silently re-enable a bridge that was
 * shut down.
 *
 * This exists because during bring-up the limits sometimes fire on
 * conditions you created deliberately, and fighting the supervisor is
 * worse than switching it off knowingly. With it off, nothing in
 * firmware is watching the bus or the currents -- whatever current
 * limiting is in the supply, and your own attention, become the only
 * protection.
 *
 * @param enabled  non-zero to check limits, zero to stop checking
 */
void protection_set_enabled(uint8_t enabled);

/**
 * @return 1 if the supervisor is checking limits, 0 if disabled
 */
uint8_t protection_is_enabled(void);

/**
 * @return the currently latched fault bitmask
 */
uint8_t protection_get_faults(void);

/**
 * Clear a latched fault so the bridge can be enabled again.
 *
 * Re-reads the sensors first and refuses if the condition that caused
 * the trip is still present. Otherwise a clear command would simply
 * re-arm a board that is about to trip again immediately.
 *
 * @return 1 if the fault was cleared, 0 if the condition persists
 */
uint8_t protection_clear_faults(void);

/**
 * Turn a single fault bit into a short word for a protocol reply.
 *
 * @param fault_bit  exactly one of the PROTECTION_FAULT_ constants
 * @return a static string, never NULL
 */
const char *protection_fault_text(uint8_t fault_bit);

#endif /* PROTECTION_H_ */
