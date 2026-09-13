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
#define CURRENT_ZERO_CALIBRATION_SAMPLES 64U

/* Default zero-current reading, mid-scale on a 12-bit ADC. Used until a
 * real calibration completes. */
#define CURRENT_ZERO_COUNTS_DEFAULT 2048u

/* The sensor instance handed to sensors_start(). Cached here because
 * every other function in this file needs it and none of them can be
 * handed a context of their own -- same reasoning as control.c's static
 * motor pointer. */
static sensors_t *self;

uint8_t sensors_start(sensors_t *s)
{
    self = s;

    self->current_a_ma            = 0;
    self->current_b_ma            = 0;
    self->zero_counts_a           = CURRENT_ZERO_COUNTS_DEFAULT;
    self->zero_counts_b           = CURRENT_ZERO_COUNTS_DEFAULT;

    /* Assumed until a known current has been driven and
     * estimate_current_direction() has had a look. */
    self->direction_a             = 1;
    self->direction_b             = 1;
    self->calibrating_currents    = 0u;
    self->calibration_total_a     = 0u;
    self->calibration_total_b     = 0u;
    self->calibration_samples_taken = 0u;

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

void sensors_capture_currents(void)
{
    /* Injected rank 1 is channel 0 (phase A) and rank 2 is channel 1
     * (phase B), in the order the injected sequence was configured in
     * CubeMX. */
    uint32_t raw_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_b = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    /* During calibration the raw values are accumulated and nothing else
     * happens. They cannot be converted to milliamps yet, because the
     * references that conversion needs are exactly what is being
     * measured. */
    if (self->calibrating_currents != 0u) {
        if (self->calibration_samples_taken < CURRENT_ZERO_CALIBRATION_SAMPLES) {
            self->calibration_total_a += raw_a;
            self->calibration_total_b += raw_b;
            self->calibration_samples_taken++;
        }
        return;
    }

    self->current_a_ma = counts_to_milliamps(raw_a, self->zero_counts_a)
                         * (int32_t)self->direction_a;
    self->current_b_ma = counts_to_milliamps(raw_b, self->zero_counts_b)
                         * (int32_t)self->direction_b;
}

void sensors_get_currents(int32_t *current_a_out, int32_t *current_b_out)
{
    if (current_a_out != NULL) {
        *current_a_out = self->current_a_ma;
    }
    if (current_b_out != NULL) {
        *current_b_out = self->current_b_ma;
    }
}

void sensors_set_direction(int8_t direction_a, int8_t direction_b)
{
    if ((direction_a == 1) || (direction_a == -1)) {
        self->direction_a = direction_a;
    }
    if ((direction_b == 1) || (direction_b == -1)) {
        self->direction_b = direction_b;
    }
}

void sensors_get_direction(int8_t *direction_a_out, int8_t *direction_b_out)
{
    if (direction_a_out != NULL) {
        *direction_a_out = self->direction_a;
    }
    if (direction_b_out != NULL) {
        *direction_b_out = self->direction_b;
    }
}

void sensors_begin_current_calibration(void)
{
    self->calibration_total_a       = 0u;
    self->calibration_total_b       = 0u;
    self->calibration_samples_taken = 0u;
    self->calibrating_currents      = 1u;
}

uint8_t sensors_current_calibration_done(void)
{
    return (self->calibration_samples_taken >= CURRENT_ZERO_CALIBRATION_SAMPLES)
               ? 1u : 0u;
}

void sensors_end_current_calibration(void)
{
    self->zero_counts_a = (uint16_t)(self->calibration_total_a
                                     / CURRENT_ZERO_CALIBRATION_SAMPLES);
    self->zero_counts_b = (uint16_t)(self->calibration_total_b
                                     / CURRENT_ZERO_CALIBRATION_SAMPLES);
    self->calibrating_currents = 0u;
}

void sensors_cancel_current_calibration(void)
{
    self->calibrating_currents = 0u;
}
