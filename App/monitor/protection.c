#include "protection.h"

#include "apex_board.h"
#include "loop.h"
#include "gate_driver.h"
#include "main.h"
#include "sensors.h"

/* No scaling constants here on purpose. Currents arrive from
 * sensors_get_currents() already in milliamps with the zero-current
 * reference subtracted, and bus voltage arrives from sensors_get_bus_mv()
 * already in millivolts. A supervisor that re-derived either from raw
 * counts would be a second place to get the shunt value or the divider
 * ratio wrong. */

/* ------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------ */

static int32_t  instantaneous_limit_ma = PROTECTION_DEFAULT_INSTANT_LIMIT_MA;
static int32_t  sustained_limit_ma     = PROTECTION_DEFAULT_SUSTAINED_LIMIT_MA;
static uint32_t sustained_window_ms    = PROTECTION_DEFAULT_SUSTAINED_WINDOW_MS;
static uint32_t bus_maximum_mv         = PROTECTION_DEFAULT_BUS_MAXIMUM_MV;
static uint32_t bus_minimum_mv         = PROTECTION_DEFAULT_BUS_MINIMUM_MV;

/* Faults currently latched. Zero means running normally. */
static uint8_t latched_faults = PROTECTION_FAULT_NONE;

/* When current first went above the sustained limit. Zero means it is
 * not currently above it. Used to measure how long the excursion has
 * lasted without needing a timer. */
static uint32_t sustained_excursion_start_tick;

/* Counts down to the next bus voltage check. */
static uint32_t polls_until_bus_check;

/* Whether limits are checked at all. Disabling does not clear anything
 * already latched. */
static uint8_t supervisor_enabled = 1u;

/* ------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------ */

/* Magnitude of a signed value.
 *
 * Used because current flows both ways through a winding depending on
 * where the rotor is in its commutation cycle, and either direction is
 * equally capable of overheating something. Only the size matters when
 * comparing against a limit.
 *
 * @param value  any signed number
 * @return its magnitude, always zero or positive */
static int32_t absolute(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* Fetch the most recent phase currents.
 *
 * The control loop sampled these at an exact instant in the PWM period
 * and already converted them to milliamps, so this is a snapshot read
 * rather than a conversion -- it costs two memory accesses and cannot
 * fail or block.
 *
 * @param current_a  where to store phase A current in milliamps, signed
 * @param current_b  where to store phase B current in milliamps, signed
 * @return 1 on success, 0 if the control loop is not running. In that
 *         case the published values are frozen at whatever they were
 *         when it stopped, and comparing stale values against a limit
 *         would give false confidence rather than protection. */
static uint8_t read_currents(int32_t *current_a, int32_t *current_b)
{
    /* Already sampled at the exact PWM instant by the control loop, so
     * this is a snapshot read rather than a conversion. If the loop is
     * not running these values are stale, which is why that is checked. */
    if (loop_is_running() == 0u) {
        return 0u;
    }
    sensors_get_currents(current_a, current_b);
    return 1u;
}

/* Shut the bridge down and record why.
 *
 * The order is deliberate: the bridge is disabled before anything is
 * recorded or indicated, because stopping the current is the only part
 * that is urgent.
 *
 * @param fault_bits  one or more PROTECTION_FAULT_ constants */
static void trip(uint8_t fault_bits)
{
    gate_driver_disable_all();

    latched_faults |= fault_bits;
    led2(1);                       /* comError LED */
}

/* ------------------------------------------------------------------
 * Setup
 * ------------------------------------------------------------------ */

uint8_t protection_init(void)
{
    latched_faults                 = PROTECTION_FAULT_NONE;
    sustained_excursion_start_tick = 0u;
    polls_until_bus_check          = 0u;
    supervisor_enabled             = 1u;

    /* Start from a known-safe state: bridge off, error LED off. The
     * zero-current references this used to measure are now established
     * by loop_init(), which must therefore run first. */
    gate_driver_disable_all();
    led2(0);
    return 1u;
}

uint8_t protection_set_current_limits(int32_t  instantaneous_ma,
                                      int32_t  sustained_ma,
                                      uint32_t window_ms)
{
    /* A sustained limit at or above the instantaneous one would never be
     * reached, because the instantaneous check trips first every time.
     * Refusing is better than accepting a configuration that silently
     * disables half the protection. */
    if (instantaneous_ma <= sustained_ma) {
        return 0u;
    }
    if ((instantaneous_ma <= 0) || (sustained_ma <= 0)) {
        return 0u;
    }

    instantaneous_limit_ma = instantaneous_ma;
    sustained_limit_ma     = sustained_ma;
    sustained_window_ms    = window_ms;
    return 1u;
}

uint8_t protection_set_bus_limits(uint32_t minimum_mv, uint32_t maximum_mv)
{
    if (minimum_mv >= maximum_mv) {
        return 0u;
    }
    bus_minimum_mv = minimum_mv;
    bus_maximum_mv = maximum_mv;
    return 1u;
}

/* ------------------------------------------------------------------
 * Polling
 * ------------------------------------------------------------------ */

/* Track how long current has been above the sustained limit, and trip if
 * it has been too long.
 *
 * The excursion start time is recorded on the first sample above the
 * limit and cleared on the first sample below it, so brief peaks -- which
 * are normal during acceleration -- never accumulate towards a trip.
 *
 * @param highest_current_ma  the larger magnitude of the two channels */
static void check_sustained_current(int32_t highest_current_ma)
{
    if (highest_current_ma > sustained_limit_ma) {

        if (sustained_excursion_start_tick == 0u) {
            /* First sample above the limit. Start the clock.
             *
             * HAL_GetTick() can legitimately return 0 in the first
             * millisecond after reset, which would look like "not
             * currently above the limit". Substituting 1 costs a
             * millisecond of accuracy once, and avoids the excursion
             * never being timed at all. */
            uint32_t now = HAL_GetTick();
            sustained_excursion_start_tick = (now == 0u) ? 1u : now;

        } else if ((HAL_GetTick() - sustained_excursion_start_tick)
                       > sustained_window_ms) {
            trip(PROTECTION_FAULT_SUSTAINED_CURRENT);
        }

    } else {
        sustained_excursion_start_tick = 0u;
    }
}

uint8_t protection_poll(void)
{
    int32_t current_a;
    int32_t current_b;

    /* Disabled by request. Any latched fault is reported unchanged --
     * turning the supervisor off must not look like clearing a fault. */
    if (supervisor_enabled == 0u) {
        return latched_faults;
    }

    /* Already tripped: the bridge is off, so there is nothing left to
     * protect and nothing to gain from measuring again. */
    if (latched_faults != PROTECTION_FAULT_NONE) {
        return latched_faults;
    }

    /* A measurement that cannot be obtained is treated as a fault rather
     * than ignored. Unmeasured current is not the same as no current. */
    if (read_currents(&current_a, &current_b) == 0u) {
        trip(PROTECTION_FAULT_SENSOR_FAILED);
        return latched_faults;
    }

    /* Magnitude, not sign: current flows both ways through a winding
     * depending on where in the commutation cycle the rotor is, and
     * either direction is equally capable of overheating something. */
    int32_t magnitude_a = absolute(current_a);
    int32_t magnitude_b = absolute(current_b);

    if (magnitude_a > instantaneous_limit_ma) {
        trip(PROTECTION_FAULT_OVERCURRENT_A);
        return latched_faults;
    }
    if (magnitude_b > instantaneous_limit_ma) {
        trip(PROTECTION_FAULT_OVERCURRENT_B);
        return latched_faults;
    }

    check_sustained_current((magnitude_a > magnitude_b)
                              ? magnitude_a : magnitude_b);

    if (latched_faults != PROTECTION_FAULT_NONE) {
        return latched_faults;
    }

    /* Bus voltage, checked occasionally rather than every call. */
    if (polls_until_bus_check == 0u) {
        uint32_t bus_mv;

        polls_until_bus_check = PROTECTION_BUS_CHECK_INTERVAL;

        bus_mv = sensors_get_bus_mv();

        if (bus_mv > bus_maximum_mv) {
            trip(PROTECTION_FAULT_BUS_OVERVOLTAGE);
        } else if (bus_mv < bus_minimum_mv) {
            trip(PROTECTION_FAULT_BUS_UNDERVOLTAGE);
        }
    } else {
        polls_until_bus_check--;
    }

    return latched_faults;
}

/* ------------------------------------------------------------------
 * Fault reporting and clearing
 * ------------------------------------------------------------------ */

void protection_set_enabled(uint8_t enabled)
{
    supervisor_enabled = (enabled != 0u) ? 1u : 0u;
}

uint8_t protection_is_enabled(void)
{
    return supervisor_enabled;
}

uint8_t protection_get_faults(void)
{
    return latched_faults;
}

uint8_t protection_clear_faults(void)
{
    int32_t  current_a;
    int32_t  current_b;
    uint32_t bus_mv;

    if (latched_faults == PROTECTION_FAULT_NONE) {
        return 1u;
    }

    /* The bridge is already off, so current should now be near zero and
     * the bus should be back in range. If either is still out of bounds,
     * something is wrong that clearing the fault will not fix. */
    if (read_currents(&current_a, &current_b) == 0u) {
        return 0u;
    }
    bus_mv = sensors_get_bus_mv();

    if (absolute(current_a) > sustained_limit_ma) {
        return 0u;
    }
    if (absolute(current_b) > sustained_limit_ma) {
        return 0u;
    }
    if ((bus_mv > bus_maximum_mv) || (bus_mv < bus_minimum_mv)) {
        return 0u;
    }

    latched_faults                 = PROTECTION_FAULT_NONE;
    sustained_excursion_start_tick = 0u;
    led2(0);

    return 1u;
}

const char *protection_fault_text(uint8_t fault_bit)
{
    switch (fault_bit) {
        case PROTECTION_FAULT_NONE:              return "none";
        case PROTECTION_FAULT_OVERCURRENT_A:     return "overcurrent_a";
        case PROTECTION_FAULT_OVERCURRENT_B:     return "overcurrent_b";
        case PROTECTION_FAULT_SUSTAINED_CURRENT: return "sustained_current";
        case PROTECTION_FAULT_BUS_OVERVOLTAGE:   return "bus_overvoltage";
        case PROTECTION_FAULT_BUS_UNDERVOLTAGE:  return "bus_undervoltage";
        case PROTECTION_FAULT_SENSOR_FAILED:     return "sensor_failed";
        default:                                 return "unknown";
    }
}
