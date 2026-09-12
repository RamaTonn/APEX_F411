/*
 * telemetry.h
 *
 * Streams samples from the control loop to the host continuously,
 * without the control loop ever waiting on USB.
 *
 * WHY A RING BUFFER SITS IN THE MIDDLE
 *
 *   The control interrupt runs every 31 microseconds and must finish
 *   well inside that. Sending over USB can wait up to 20 milliseconds for
 *   a previous transfer to clear -- six hundred times the entire budget.
 *   An interrupt that sent directly would stall the loop, miss periods,
 *   and take the bridge with it.
 *
 *   So the interrupt writes a sample into a ring buffer and returns, and
 *   the main loop drains that buffer whenever USB is free. This is the
 *   same arrangement as the USB receive queue, for the same reason: one
 *   producer, one consumer, running at different times.
 *
 * RATE
 *
 *   The loop runs at 32 kHz. Capturing every iteration would produce
 *   about 640 kilobytes a second in binary and far more as text, which
 *   is beyond what the link will carry and far more than is useful to
 *   look at.
 *
 *   A divider selects every Nth iteration instead. A divider of 32 gives
 *   1 kHz, which resolves anything a mechanical system does while
 *   costing about 20 kilobytes a second.
 *
 * DROPPED SAMPLES ARE COUNTED, NEVER HIDDEN
 *
 *   If the host stops reading, or the main loop falls behind, the ring
 *   fills. New samples are then discarded and counted.
 *
 *   The count is reported, because a gap in a plot that nobody knows
 *   about is worse than no plot: a step that is really a missing
 *   millisecond looks exactly like a step that really happened.
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>

#include "motor.h"

/* How many samples the ring holds.
 *
 * Must be a power of two, so the wrap-around is a bitwise AND.
 *
 * At 1 kHz this is a quarter of a second of buffering, which comfortably
 * covers a main loop briefly held up by a slow command. Each slot is 20
 * bytes, so the whole ring is 5 kilobytes. */
#define TELEMETRY_RING_SIZE 256U

/* Signals that can be streamed. Each is one bit in the channel mask, so
 * any combination can be selected. */
#define TELEMETRY_CHANNEL_TIME       0x01u  /* loop iteration counter   */
#define TELEMETRY_CHANNEL_CURRENT_A  0x02u  /* phase A, milliamps       */
#define TELEMETRY_CHANNEL_CURRENT_B  0x04u  /* phase B, milliamps       */
#define TELEMETRY_CHANNEL_CURRENT_C  0x08u  /* phase C, derived         */
#define TELEMETRY_CHANNEL_E_ANGLE    0x10u  /* applied electrical angle */
#define TELEMETRY_CHANNEL_ENCODER    0x20u  /* rotor angle, if sampled  */
#define TELEMETRY_CHANNEL_R_ANGLE    0x40u  /* rotor electrical angle   */

#define TELEMETRY_CHANNEL_ALL        0x7Fu

/* A useful default: both measured currents and a timestamp. */
#define TELEMETRY_CHANNEL_DEFAULT \
    (TELEMETRY_CHANNEL_TIME | TELEMETRY_CHANNEL_CURRENT_A \
     | TELEMETRY_CHANNEL_CURRENT_B)

/* One captured sample.
 *
 * Every field is stored whether or not its channel is selected. Storing
 * a fixed record costs a little memory but keeps the ring a plain array
 * -- variable-length records would need their own length handling, and
 * the interrupt would have to do that work. Selection is applied when
 * the sample is sent instead, which happens in the main loop where there
 * is time. */
typedef struct {
    uint32_t iteration;        /* which loop period this came from */
    int32_t  current_a_ma;
    int32_t  current_b_ma;
    uint16_t electrical_angle; /* applied, 0 to 65535 for one turn   */
    uint16_t encoder_angle;    /* 0 to 16383, or 0 when not sampled  */
    uint16_t rotor_angle;      /* measured electrical, 0 to 65535    */
} telemetry_sample_t;

/**
 * Reset the module. Streaming is off afterwards.
 */
void telemetry_init(motor_t *m);

/**
 * Choose what is streamed and how often.
 *
 * Safe to call while streaming; the change takes effect on the next
 * captured sample.
 *
 * @param channel_mask  which signals to send, any combination of the
 *                      TELEMETRY_CHANNEL_ constants
 * @param divider       capture one loop iteration in this many. 1 means
 *                      every iteration at 32 kHz, 32 gives 1 kHz. Zero
 *                      is treated as 1.
 * @return 1 on success, 0 if the mask selects nothing
 */
uint8_t telemetry_configure(uint32_t channel_mask, uint16_t divider);

/**
 * Begin capturing. Clears the ring and the drop count first, so a run
 * never carries stale samples or a stale drop count from a previous one.
 */
void telemetry_start(void);

/**
 * Stop capturing. Samples already in the ring are still sent, so the
 * final moments before a stop are not lost.
 */
void telemetry_stop(void);

/**
 * @return 1 if capturing
 */
uint8_t telemetry_is_running(void);

/**
 * Record one sample.
 *
 * CALLED FROM THE CONTROL INTERRUPT ONLY.
 *
 * Applies the divider, so it can be called every iteration and will
 * store only the ones wanted. Never waits, never sends anything, and
 * discards rather than blocking if the ring is full.
 *
 * The angles are not passed in, because the control interrupt does not
 * always know them -- with no control algorithm running there is no
 * electrical angle to report at all. Whatever generates one publishes it
 * separately with telemetry_set_angles(), and the most recent values are
 * attached to each sample as it is captured.
 *
 * @param current_a_ma  phase A current in milliamps
 * @param current_b_ma  phase B current in milliamps
 */
void telemetry_capture(int32_t current_a_ma, int32_t current_b_ma);

/**
 * Publish the angles to attach to subsequent samples.
 *
 * Called from the control interrupt by whichever module is generating
 * the angles, before telemetry_capture() runs for that period.
 *
 * A module that stops driving should publish zeros, so that a stopped
 * stream does not keep reporting its last angle as though it were still
 * live.
 *
 * @param electrical_angle  applied electrical angle, 0 to 65535
 * @param encoder_angle     measured rotor angle, or 0 if not sampled
 */
void telemetry_set_angles(uint16_t electrical_angle, uint16_t encoder_angle);

/**
 * Send whatever samples are waiting.
 *
 * CALLED FROM THE MAIN LOOP ONLY.
 *
 * Sends nothing and returns immediately if USB is still busy with a
 * previous transfer, so a host that has stopped reading slows the stream
 * rather than stalling the main loop.
 *
 * Sends at most a few samples per call, so that one flush cannot
 * monopolise the loop and delay the protection supervisor.
 */
void telemetry_flush(void);

/**
 * How many samples were discarded because the ring was full.
 *
 * Any non-zero value means the stream has gaps. Either the host is not
 * reading fast enough, or the divider is too small for the link.
 *
 * @return the count since telemetry_start()
 */
uint32_t telemetry_get_dropped(void);

/**
 * How many samples have been sent since telemetry_start().
 *
 * @return the count
 */
uint32_t telemetry_get_sent(void);

/**
 * @return the active channel mask
 */
uint32_t telemetry_get_channels(void);

/**
 * @return the active divider
 */
uint16_t telemetry_get_divider(void);

#endif /* TELEMETRY_H_ */
