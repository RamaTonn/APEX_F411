#include "sensors.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

/* Bus counts to millivolts: 3300 millivolts of ADC range multiplied by
 * the measured divider ratio of 23, then divided by 4096 counts. The
 * division is a shift because 4096 is a power of two.
 *
 * Worst case is 4095 times 75900, about 311 million, which fits in a
 * 32-bit unsigned value with room to spare. */
#define BUS_COUNTS_TO_MILLIVOLTS 75900u
#define BUS_COUNTS_SHIFT         12u

/* The sensor instance handed to sensors_start(). Cached here because
 * every other function in this file needs it and none of them can be
 * handed a context of their own -- same reasoning as control.c's static
 * motor pointer. */
static sensors_t *self;

uint8_t sensors_start(sensors_t *s)
{
    self = s;

    /* The cast discards volatile because HAL's prototype does not
     * declare the parameter volatile. The DMA still writes the buffer
     * exactly as before; only the compiler's view of it through this one
     * pointer changes, and HAL does not read the buffer at all. */
    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)(void *)self->regular_results,
                          SENSORS_BUFFER_LENGTH) != HAL_OK) {
        return 0u;
    }
    return 1u;
}

uint16_t sensors_get_bus_counts(void)
{
    return self->regular_results[SENSORS_BUFFER_INDEX_BUS];
}

uint16_t sensors_get_temperature_counts(void)
{
    return self->regular_results[SENSORS_BUFFER_INDEX_TEMPERATURE];
}

uint32_t sensors_get_bus_mv(void)
{
    return ((uint32_t)sensors_get_bus_counts() * BUS_COUNTS_TO_MILLIVOLTS)
           >> BUS_COUNTS_SHIFT;
}
