/*
 * sensors.h
 *
 * Everything that comes off the ADC: motor bus voltage, temperature, and
 * the two measured phase currents. One peripheral, three physical
 * quantities, all owned here so nowhere else has to know a shunt value,
 * a divider ratio, or which ADC group a reading came from.
 *
 * WHY CURRENTS AND THE SLOW CHANNELS SHARE A FILE DESPITE DIFFERENT
 * TIMING
 *
 *   Phase currents must be sampled at an exact instant in the PWM cycle
 *   and read out within microseconds. They live on the ADC's injected
 *   group, triggered by the timer, and sensors_capture_currents() is
 *   called from the control loop's interrupt once per period.
 *
 *   Bus voltage and temperature change slowly, come from high impedance
 *   sources that need a long sampling window, and nothing needs them at
 *   any particular instant. They live on the regular group, converting
 *   continuously into a DMA buffer, so reading one is a memory access
 *   with no waiting at all.
 *
 *   The injected group automatically preempts the regular group, so a
 *   current sample is never delayed by a slow housekeeping conversion.
 *   That is the whole reason the two groups exist. But both groups are
 *   the same ADC measuring properties of the same board, and a caller
 *   asking "how much current, and how healthy is the bus" shouldn't have
 *   to know that one answer came from an interrupt and the other from a
 *   free-running buffer -- hence one module, one owned instance.
 *
 * TIMING
 *
 *   Each regular conversion takes about 20.5 microseconds at 480 sample
 *   cycles, so both slow channels refresh roughly every 45 microseconds
 *   once the injected interruptions are allowed for. Values read here are
 *   therefore at most about that old. Currents are refreshed once per
 *   31.25 microsecond control period.
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
 * Everything the ADC measures. Owned by main.c and started once; every
 * function below acts on that one instance, which this module keeps a
 * pointer to internally -- there is exactly one ADC on this board, so
 * nothing outside this file needs the struct itself.
 */
typedef struct {
    /* The DMA writes conversion results here continuously and forever.
     *
     * volatile because the DMA controller changes it without the
     * compiler having any way to know. Without it, a reader could be
     * given a value cached in a register from an earlier read. */
    volatile uint16_t regular_results[SENSORS_BUFFER_LENGTH];

    /* Most recent phase currents in milliamps, published by
     * sensors_capture_currents() for anything that wants them. */
    volatile int32_t current_a_ma;
    volatile int32_t current_b_ma;

    /* What the current sensors read with no current flowing. Every
     * current measurement is a difference from these.
     *
     * Defaulted to mid-scale so a failed calibration produces
     * plausible-looking rather than absurd values -- though a failed
     * sensors_end_current_calibration() sequence is the signal that
     * should actually be acted on. */
    uint16_t zero_counts_a;
    uint16_t zero_counts_b;

    /* Non-zero while sensors_capture_currents() should accumulate raw
     * samples for zero calibration instead of publishing currents. */
    volatile uint8_t  calibrating_currents;
    volatile uint32_t calibration_total_a;
    volatile uint32_t calibration_total_b;
    volatile uint16_t calibration_samples_taken;
} sensors_t;

/**
 * Start continuous conversion into the DMA buffer and zero a sensors_t
 * to a safe starting state (currents at zero, zero references at
 * mid-scale, no calibration in progress).
 *
 * Must be called before any reader below, and before the control loop
 * starts, because the injected group expects the ADC to already be
 * running.
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

/**
 * Refresh the two phase currents from the injected ADC group.
 *
 * CALLED FROM THE CONTROL INTERRUPT ONLY, once per period, right after
 * the injected conversion completes -- the values are latched in the
 * ADC's injected data registers until the next conversion overwrites
 * them, so this must run before that happens.
 *
 * While a current-zero calibration is in progress (see
 * sensors_begin_current_calibration()), the raw readings are
 * accumulated instead of published, since the zero references that
 * conversion needs are exactly what is being measured.
 */
void sensors_capture_currents(void);

/**
 * Most recent phase currents, in milliamps.
 *
 * Safe to call from the main loop. The values are a snapshot from
 * whichever period completed most recently.
 *
 * @param current_a_out  where to store phase A current. May be NULL.
 * @param current_b_out  where to store phase B current. May be NULL.
 */
void sensors_get_currents(int32_t *current_a_out, int32_t *current_b_out);

/**
 * Begin accumulating raw current samples for zero calibration.
 *
 * Must be called with the injected conversions already running (the
 * timer trigger and interrupt armed), since sensors_capture_currents()
 * is what actually collects the samples -- nothing here polls the ADC
 * directly.
 */
void sensors_begin_current_calibration(void);

/**
 * @return 1 once enough samples have been accumulated to compute zero
 *         references, 0 while still collecting
 */
uint8_t sensors_current_calibration_done(void);

/**
 * Finish a zero calibration begun with sensors_begin_current_calibration(),
 * computing zero_counts_a/b from the accumulated samples.
 *
 * Only call this once sensors_current_calibration_done() reports 1 --
 * calling it early would average too few samples into the reference.
 */
void sensors_end_current_calibration(void);

/**
 * Abandon a zero calibration begun with
 * sensors_begin_current_calibration(), e.g. because it timed out.
 *
 * Leaves zero_counts_a/b untouched, so a failed calibration keeps
 * whatever reference was in effect before rather than replacing it with
 * one averaged from too few samples.
 */
void sensors_cancel_current_calibration(void);

#endif /* SENSORS_H_ */
