/*
 * sensors.h
 *
 * The two slow analogue channels: motor bus voltage and temperature.
 *
 * WHAT CHANGED, AND WHY
 *
 *   The ADC now serves two jobs with very different needs.
 *
 *   Phase currents must be sampled at an exact instant in the PWM cycle
 *   and read out within microseconds. They live on the ADC's injected
 *   group, triggered by the timer, and are handled entirely by the
 *   control module. Nothing here touches them -- use
 *   control_get_currents().
 *
 *   Bus voltage and temperature change slowly, come from high impedance
 *   sources that need a long sampling window, and nothing needs them at
 *   any particular instant. They live on the regular group, converting
 *   continuously into a DMA buffer, so reading one is a memory access
 *   with no waiting at all.
 *
 *   The injected group automatically preempts the regular group, so a
 *   current sample is never delayed by a slow housekeeping conversion.
 *   That is the whole reason for splitting them.
 *
 * TIMING
 *
 *   Each regular conversion takes about 20.5 microseconds at 480 sample
 *   cycles, so both channels refresh roughly every 45 microseconds once
 *   the injected interruptions are allowed for. Values read here are
 *   therefore at most about that old.
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>

/* Positions in the DMA buffer, matching the regular conversion ranks
 * configured in CubeMX: rank 1 is channel 2, rank 2 is channel 9. */
#define SENSORS_BUFFER_INDEX_BUS         0U
#define SENSORS_BUFFER_INDEX_TEMPERATURE 1U
#define SENSORS_BUFFER_LENGTH            2U

/* Reading with the thermistor connector empty. R31 pulls the pin to the
 * supply rail with nothing pulling it down, so the reading sits just
 * below full scale. Useful as a "nothing plugged in" test. */
#define SENSORS_TEMPERATURE_DISCONNECTED 4000U

/*
 * The slow channels. Owned by main.c and started once; every reader below
 * acts on that one instance, which this module keeps a pointer to
 * internally -- there is exactly one bus/temperature pair on this board,
 * so nothing outside this file needs the struct itself.
 */
typedef struct {
    /* The DMA writes conversion results here continuously and forever.
     *
     * volatile because the DMA controller changes it without the
     * compiler having any way to know. Without it, a reader could be
     * given a value cached in a register from an earlier read. */
    volatile uint16_t regular_results[SENSORS_BUFFER_LENGTH];
} sensors_t;

/**
 * Start continuous conversion into the DMA buffer.
 *
 * Must be called before any reader below, and before control_init(),
 * because the injected group expects the ADC to already be running.
 *
 * @param s  the sensor instance this board's ADC feeds
 * @return 1 on success, 0 if the ADC or DMA would not start
 */
uint8_t sensors_start(sensors_t *s);

/**
 * Motor supply voltage in millivolts.
 *
 * Uses the divider ratio of 23.0, which was measured against a
 * multimeter rather than calculated from the schematic -- the bottom leg
 * has two 10k resistors in parallel, which is easy to miss.
 *
 * @return bus voltage in millivolts
 */
uint32_t sensors_get_bus_mv(void);

/**
 * Raw temperature channel reading.
 *
 * Returned as counts rather than degrees because the thermistor is
 * off-board on connector J5 and its curve is unknown. A reading above
 * SENSORS_TEMPERATURE_DISCONNECTED means nothing is plugged in.
 *
 * @return 0..4095
 */
uint16_t sensors_get_temperature_counts(void);

/**
 * Raw bus channel reading, for checking scaling against a meter.
 *
 * @return 0..4095
 */
uint16_t sensors_get_bus_counts(void);

#endif /* SENSORS_H_ */
