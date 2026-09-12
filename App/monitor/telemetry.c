#include "telemetry.h"

#include "motor.h"
#include "encoder.h"
#include "protocol.h"
#include "USB_Comm.h"

/* Samples sent per call to telemetry_flush().
 *
 * A flush that emptied the whole ring could occupy the main loop for
 * hundreds of transfers, delaying the protection supervisor and the
 * command console. Sending a few per pass keeps the loop responsive; at
 * a 1 kHz capture rate and a main loop running far faster, this still
 * drains faster than samples arrive. */
#define SAMPLES_PER_FLUSH 8U

/* ------------------------------------------------------------------
 * The ring
 *
 * Written by the control interrupt, read by the main loop. Exactly one
 * writer for each index, so neither side needs to disable interrupts:
 * the interrupt only writes the write index, the main loop only writes
 * the read index, and each merely reads the other's.
 * ------------------------------------------------------------------ */

/* The motor whose electrical angle is recorded with each sample. */
static motor_t *motor;

static volatile telemetry_sample_t ring[TELEMETRY_RING_SIZE];

/* Where the interrupt puts the next sample. */
static volatile uint32_t write_index;

/* Where the main loop takes the next sample from. */
static volatile uint32_t read_index;

/* Samples discarded because the ring was full. */
static volatile uint32_t dropped_count;

/* Samples handed to USB. */
static uint32_t sent_count;

/* ------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------ */

static volatile uint32_t active_channels = TELEMETRY_CHANNEL_DEFAULT;
static volatile uint16_t active_divider  = 32u;   /* 32 kHz / 32 = 1 kHz */
static volatile uint8_t  capturing;

/* Counts down to the next sample to keep. Reloaded from the divider. */
static volatile uint16_t divider_countdown;

/* Angles published by whatever is generating them, attached to each
 * sample as it is captured.
 *
 * Held here rather than passed into telemetry_capture() because the
 * control interrupt does not always have them: with no control algorithm
 * installed there is no electrical angle in existence, yet the currents
 * are still worth streaming. */
static volatile uint16_t latest_electrical_angle;
static volatile uint16_t latest_encoder_angle;

/* Advance a ring index, wrapping at the end.
 *
 * The size is a power of two, so subtracting one gives a mask of all
 * ones in the low bits and the wrap costs a single instruction.
 *
 * @param index  the index to advance
 * @return the next index */
static inline uint32_t next_index(uint32_t index)
{
    return (index + 1u) & (TELEMETRY_RING_SIZE - 1u);
}

/* ------------------------------------------------------------------
 * Capture -- interrupt context
 * ------------------------------------------------------------------ */

void telemetry_set_angles(uint16_t electrical_angle, uint16_t encoder_angle)
{
    latest_electrical_angle = electrical_angle;
    latest_encoder_angle    = encoder_angle;
}

void telemetry_capture(int32_t current_a_ma, int32_t current_b_ma)
{
    if (capturing == 0u) {
        return;
    }

    /* The divider is applied here rather than by the caller, so the
     * control loop can call this unconditionally every period and stay
     * ignorant of the streaming rate. */
    if (divider_countdown > 1u) {
        divider_countdown--;
        return;
    }
    divider_countdown = active_divider;

    uint32_t proposed_write = next_index(write_index);

    /* Landing exactly on the read index means the ring is full. One slot
     * is always left empty so that a full ring and an empty one stay
     * distinguishable -- they would otherwise both show the two indices
     * equal.
     *
     * The new sample is discarded rather than overwriting the oldest.
     * Overwriting would leave the main loop reading a record while the
     * interrupt was part way through replacing it, producing a sample
     * with fields from two different moments. A missing sample is
     * honest; a spliced one is not. */
    if (proposed_write == read_index) {
        dropped_count++;
        return;
    }

    /* The encoder is read here, on a period that is definitely being
     * kept, rather than every control period.
     *
     * Two SPI frames take about seven microseconds -- roughly a fifth of
     * the 31 microsecond control period. Paying that on every period for
     * a value only the stream uses would be wasteful; paying it only on
     * captured periods costs a thirty-second of that at the usual
     * divider, and the angle is then measured at the same instant as the
     * currents rather than at some other moment in the main loop.
     *
     * A failed read leaves the previous value rather than substituting
     * zero, since a momentary zero in the middle of a rotation would
     * look like a real jump back to the origin. */
    if ((active_channels & TELEMETRY_CHANNEL_ENCODER) != 0u) {
        uint16_t measured_angle;
        if (encoder_read_angle(&measured_angle) != 0u) {
            latest_encoder_angle = measured_angle;
        }
    }

    ring[write_index].iteration        = 0u;   /* filled in below */
    ring[write_index].current_a_ma     = current_a_ma;
    ring[write_index].current_b_ma     = current_b_ma;
    ring[write_index].electrical_angle = latest_electrical_angle;
    ring[write_index].encoder_angle    = latest_encoder_angle;

    /* Taken straight from the motor rather than published
     * separately: it is derived from the encoder every control period
     * regardless of what is driving, so there is nothing to publish. */
    ring[write_index].rotor_angle = motor_get_electrical_angle(motor);

    /* The iteration number is taken from the count of samples captured
     * rather than from the control loop's own counter, so that a
     * timestamp still increases evenly when the divider changes. */
    static uint32_t capture_number;
    capture_number++;
    ring[write_index].iteration = capture_number;

    /* The record is fully written before the index moves. If the index
     * moved first, the main loop could read a half-filled slot. */
    write_index = proposed_write;
}

/* ------------------------------------------------------------------
 * Flush -- main loop context
 * ------------------------------------------------------------------ */

/* Send one sample as a reply line or packet.
 *
 * Uses the ordinary reply path, so a sample comes out as readable text
 * or as a binary packet depending on the active mode, with no separate
 * formatting code for each.
 *
 * The status word is "dat" rather than "ok", which is what marks it as
 * unsolicited: a host can tell a streamed sample from an answer to
 * something it asked for without tracking what it has outstanding.
 *
 * @param sample  the sample to send */
static void send_sample(const telemetry_sample_t *sample)
{
    uint32_t channels = active_channels;

    protocol_reply_begin(PROTOCOL_STATUS_DATA, "tel");

    if ((channels & TELEMETRY_CHANNEL_TIME) != 0u) {
        protocol_reply_uint("t", sample->iteration);
    }
    if ((channels & TELEMETRY_CHANNEL_CURRENT_A) != 0u) {
        protocol_reply_int("ia_ma", sample->current_a_ma);
    }
    if ((channels & TELEMETRY_CHANNEL_CURRENT_B) != 0u) {
        protocol_reply_int("ib_ma", sample->current_b_ma);
    }
    if ((channels & TELEMETRY_CHANNEL_CURRENT_C) != 0u) {
        /* Phase C has no sensor. The three currents of a star-connected
         * winding sum to zero, so C is whatever makes that true. */
        protocol_reply_int("ic_ma",
            -(sample->current_a_ma + sample->current_b_ma));
    }
    if ((channels & TELEMETRY_CHANNEL_E_ANGLE) != 0u) {
        protocol_reply_uint("eangle", sample->electrical_angle);
    }
    if ((channels & TELEMETRY_CHANNEL_ENCODER) != 0u) {
        protocol_reply_uint("enc", sample->encoder_angle);
    }
    if ((channels & TELEMETRY_CHANNEL_R_ANGLE) != 0u) {
        /* The rotor's electrical angle, on the same 0 to 65535 scale as
         * the applied one, so the two can be compared directly. */
        protocol_reply_uint("rangle", sample->rotor_angle);
    }

    protocol_reply_end();
}

void telemetry_flush(void)
{
    for (uint32_t i = 0u; i < SAMPLES_PER_FLUSH; i++) {

        if (read_index == write_index) {
            return;                      /* nothing waiting */
        }

        /* Do not start a transfer USB cannot take yet. Returning leaves
         * the samples in the ring for the next pass, whereas calling
         * usb_tx_bytes would wait up to 20 milliseconds and hold up the
         * protection supervisor behind it. */
        if (usb_tx_is_ready() == 0u) {
            return;
        }

        /* Copied out of the volatile ring before use, so the interrupt
         * cannot alter fields half way through formatting them. */
        telemetry_sample_t sample;
        sample.iteration        = ring[read_index].iteration;
        sample.current_a_ma     = ring[read_index].current_a_ma;
        sample.current_b_ma     = ring[read_index].current_b_ma;
        sample.electrical_angle = ring[read_index].electrical_angle;
        sample.encoder_angle    = ring[read_index].encoder_angle;
        sample.rotor_angle      = ring[read_index].rotor_angle;

        /* The slot is released only after its contents have been
         * copied, for the mirror of the reason the write index moves
         * last. */
        read_index = next_index(read_index);

        send_sample(&sample);
        sent_count++;
    }
}

/* ------------------------------------------------------------------
 * Configuration and lifecycle
 * ------------------------------------------------------------------ */

void telemetry_init(motor_t *m)
{
    /* Remember the motor whose angle is attached to each sample. */
    motor = m;

    write_index       = 0u;
    read_index        = 0u;
    dropped_count     = 0u;
    sent_count        = 0u;
    capturing         = 0u;
    active_channels   = TELEMETRY_CHANNEL_DEFAULT;
    active_divider    = 32u;
    divider_countdown = 32u;

    latest_electrical_angle = 0u;
    latest_encoder_angle    = 0u;
}

uint8_t telemetry_configure(uint32_t channel_mask, uint16_t divider)
{
    /* A mask selecting nothing would stream empty records forever, which
     * looks like a working stream carrying no information -- more
     * confusing than a refusal. */
    if ((channel_mask & TELEMETRY_CHANNEL_ALL) == 0u) {
        return 0u;
    }

    if (divider == 0u) {
        divider = 1u;
    }

    active_channels = channel_mask & TELEMETRY_CHANNEL_ALL;
    active_divider  = divider;

    return 1u;
}

void telemetry_start(void)
{
    /* Emptied before capture begins, so a run never opens with samples
     * left over from the previous one. */
    read_index        = write_index;
    dropped_count     = 0u;
    sent_count        = 0u;
    divider_countdown = active_divider;

    capturing = 1u;
}

void telemetry_stop(void)
{
    /* Only capture stops. Whatever is already in the ring is still sent,
     * so the last moments before a stop -- usually the interesting ones,
     * since a stop often follows something going wrong -- are not lost. */
    capturing = 0u;
}

uint8_t telemetry_is_running(void)
{
    return capturing;
}

uint32_t telemetry_get_dropped(void)
{
    return dropped_count;
}

uint32_t telemetry_get_sent(void)
{
    return sent_count;
}

uint32_t telemetry_get_channels(void)
{
    return active_channels;
}

uint16_t telemetry_get_divider(void)
{
    return active_divider;
}
