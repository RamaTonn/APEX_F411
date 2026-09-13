#include "protocol_binary.h"

#include "commands.h"
#include "cobs.h"
#include "USB_Comm.h"

#include <string.h>

/* ------------------------------------------------------------------
 * Receiving
 * ------------------------------------------------------------------ */

static packet_receiver_t receiver;
static packet_t          received;

/* ------------------------------------------------------------------
 * Field name to id
 *
 * A handler names its fields with strings, since that is what makes the
 * text form readable. Binary looks the name up here and sends a number.
 *
 * Both sides of the link must agree on this table, so it is the one
 * place that has to be kept in step with the PC software. Adding a row
 * is safe at any time; changing or reusing an existing number is not,
 * because an older reader would silently misinterpret the field.
 * ------------------------------------------------------------------ */

static const struct {
    const char *name;
    uint8_t     id;
} field_table[] = {
    /* identity and timing */
    { "fw",        0x01u },
    { "ms",        0x02u },
    { "iters",     0x03u },
    { "us",        0x04u },
    { "over",      0x05u },
    { "init",      0x06u },
    { "run",       0x07u },
    /* measurements */
    { "ia_ma",     0x10u },
    { "ib_ma",     0x11u },
    { "vbus_mv",   0x12u },
    { "temp_raw",  0x13u },
    { "peak_ma",   0x14u },
    { "avg_ma",    0x15u },
    { "lim_ma",    0x16u },
    { "abort_ma",  0x17u },
    /* encoder */
    { "raw",       0x20u },
    { "deg10",     0x21u },
    { "agc",       0x22u },
    { "mag",       0x23u },
    { "enc",       0x24u },
    /* bridge and motion */
    { "duty",      0x30u },
    { "on",        0x31u },
    { "hz",        0x32u },
    { "amp",       0x33u },
    { "eangle",    0x34u },
    { "rangle",    0x36u },
    { "abort",     0x35u },
    /* characterisation */
    { "mohm",      0x40u },
    { "a_mohm",    0x52u },
    { "b_mohm",    0x53u },
    { "c_mohm",    0x54u },
    { "ab_mohm",   0x55u },
    { "bc_mohm",   0x56u },
    { "ca_mohm",   0x57u },
    { "imbal",     0x58u },
    { "ld_nh",     0x59u },
    { "lq_nh",     0x5Au },
    { "d_step",    0x5Du },
    { "q_step",    0x61u },
    { "sag_mv",    0x65u },
    { "rev",       0x66u },
    { "half",      0x69u },
    { "ld_ll",     0x6Cu },
    { "lq_ll",     0x6Du },
    { "quiet_mv",  0x6Eu },
    { "load_mv",   0x6Fu },
    { "ms",        0x70u },
    { "eff",       0x71u },
    { "sum_ma",    0x72u },
    { "ref_mohm",  0x79u },
    { "want_ma",   0x7Au },
    { "gain_pct",  0x7Bu },
    { "ripple_ma", 0x77u },
    { "zmargin_ma",0x78u },
    { "dir_a",     0x73u },
    { "dir_b",     0x74u },
    { "ra_ma",     0x75u },
    { "rb_ma",     0x76u },
    { "d_mohm",    0x6Au },
    { "q_mohm",    0x6Bu },
    { "d_tau",     0x67u },
    { "q_tau",     0x68u },
    { "d_slope",   0x62u },
    { "q_slope",   0x63u },
    { "hold",      0x64u },
    { "at_pair",   0x5Eu },
    { "at_duty",   0x5Fu },
    { "at_ma",     0x60u },
    { "sal",       0x4Cu },
    /* Retired, and left here rather than deleted: the numbers must not
     * be handed to anything else, or an older reader would silently
     * misinterpret the field. */
    { "tau_us",    0x4Du },
    { "lo_ma",     0x4Eu },
    { "hi_ma",     0x4Fu },
    { "lo_duty",   0x37u },
    { "hi_duty",   0x38u },
    { "ld_uh",     0x4Au },
    { "lq_uh",     0x4Bu },
    { "dchg_ma",   0x18u },
    { "qchg_ma",   0x19u },
    { "offset",    0x41u },
    { "spread",    0x42u },
    { "poles",     0x43u },
    { "dir",       0x46u },
    { "fit",       0x47u },
    { "fwd",       0x48u },
    { "rev",       0x49u },
    { "off_mrad",     0x26u },
    { "pos_mrad",     0x27u },
    { "eangle_mrad",  0x28u },
    { "vel_mrads",    0x29u },
    { "zero_a",    0x44u },
    { "zero_b",    0x45u },
    /* faults */
    { "bits",      0x50u },
    { "reason",    0x51u },
};

#define FIELD_TABLE_LENGTH (sizeof field_table / sizeof field_table[0])

/* Find the number assigned to a field name.
 *
 * @param name  the name a handler used
 * @return the assigned id, or PROTOCOL_BINARY_FIELD_NAMED if the name is
 *         not in the table */
static uint8_t field_id_for(const char *name)
{
    for (uint32_t i = 0u; i < FIELD_TABLE_LENGTH; i++) {
        if (strcmp(name, field_table[i].name) == 0) {
            return field_table[i].id;
        }
    }
    return PROTOCOL_BINARY_FIELD_NAMED;
}

/* ------------------------------------------------------------------
 * Reply building
 * ------------------------------------------------------------------ */

/* Payload under construction, and how much of it is used. */
static uint8_t  reply_payload[PACKET_MAX_PAYLOAD];
static uint16_t reply_length;

/* Which command this reply answers, echoed back as the packet id so the
 * host can match a response to its request. */
static uint8_t reply_command_id;

/* Whether the reply has overflowed. A truncated payload would decode as
 * a valid but incomplete set of fields, so an overflowing reply is
 * turned into an error instead of being sent short. */
static uint8_t reply_overflowed;

/* Room needed for one numeric field: the id plus four bytes. */
#define FIELD_SIZE 5u

/* Reserve space for a field, or record that there is none.
 *
 * @param bytes_needed  how many bytes the field will occupy
 * @return 1 if the space is available, 0 if the reply has overflowed */
static uint8_t reserve(uint16_t bytes_needed)
{
    if (reply_overflowed != 0u) {
        return 0u;
    }
    if (((uint32_t)reply_length + bytes_needed) > PACKET_MAX_PAYLOAD) {
        reply_overflowed = 1u;
        return 0u;
    }
    return 1u;
}

/* Write a field whose name has no assigned number.
 *
 * Sent as the reserved id, then the name as a length-prefixed string,
 * then the value. Inefficient, but it means a field added to a handler
 * appears on the wire immediately instead of vanishing until someone
 * remembers to update the table.
 *
 * @param key    the field name
 * @param value  the value, as raw 32 bits */
static void write_named_field(const char *key, uint32_t value)
{
    uint16_t name_length = (uint16_t)strlen(key);

    if (name_length > 255u) {
        name_length = 255u;
    }
    if (reserve((uint16_t)(2u + name_length + 4u)) == 0u) {
        return;
    }

    reply_length = (uint16_t)packet_write_u8(reply_payload, reply_length,
                                             PROTOCOL_BINARY_FIELD_NAMED);
    reply_length = (uint16_t)packet_write_u8(reply_payload, reply_length,
                                             (uint8_t)name_length);
    memcpy(&reply_payload[reply_length], key, name_length);
    reply_length = (uint16_t)(reply_length + name_length);
    reply_length = (uint16_t)packet_write_u32(reply_payload, reply_length,
                                              value);
}

/* Write one numeric field.
 *
 * @param key    the field name, looked up in the table
 * @param value  the value, as raw 32 bits; signed values are passed
 *               through unchanged as two's complement */
static void write_field(const char *key, uint32_t value)
{
    uint8_t id = field_id_for(key);

    if (id == PROTOCOL_BINARY_FIELD_NAMED) {
        write_named_field(key, value);
        return;
    }
    if (reserve(FIELD_SIZE) == 0u) {
        return;
    }

    reply_length = (uint16_t)packet_write_u8(reply_payload, reply_length, id);
    reply_length = (uint16_t)packet_write_u32(reply_payload, reply_length,
                                              value);
}

void protocol_binary_reply_begin(const char *status, const char *command)
{
    reply_length     = 0u;
    reply_overflowed = 0u;

    /* The command name is turned back into the number the host used, so
     * a response can be matched to its request. A name with no number --
     * which happens for internal reports not tied to a command -- is
     * answered with zero. */
    reply_command_id = commands_id_for(command);

    uint8_t status_byte = PROTOCOL_BINARY_STATUS_OK;

    if (strcmp(status, PROTOCOL_STATUS_ERROR) == 0) {
        status_byte = PROTOCOL_BINARY_STATUS_ERROR;
    } else if (strcmp(status, PROTOCOL_STATUS_DATA) == 0) {
        status_byte = PROTOCOL_BINARY_STATUS_DATA;
    }

    reply_length = (uint16_t)packet_write_u8(reply_payload, 0u, status_byte);
}

void protocol_binary_reply_uint(const char *key, uint32_t value)
{
    write_field(key, value);
}

void protocol_binary_reply_int(const char *key, int32_t value)
{
    /* Cast to unsigned before storing. The wire format carries the
     * two's-complement bit pattern, and the reader turns it back into a
     * signed value; going through unsigned here avoids relying on
     * implementation-defined behaviour when shifting a negative number. */
    write_field(key, (uint32_t)value);
}

void protocol_binary_reply_text(const char *key, const char *value)
{
    /* Text values in an otherwise numeric format are almost always short
     * keywords -- a fault name, a phase letter. They are sent as a
     * length-prefixed string so the reader does not need to know which
     * fields are text in advance. */
    uint16_t value_length = (uint16_t)strlen(value);

    if (value_length > 255u) {
        value_length = 255u;
    }
    if (reserve((uint16_t)(2u + value_length)) == 0u) {
        return;
    }

    reply_length = (uint16_t)packet_write_u8(reply_payload, reply_length,
                                             field_id_for(key));
    reply_length = (uint16_t)packet_write_u8(reply_payload, reply_length,
                                             (uint8_t)value_length);
    memcpy(&reply_payload[reply_length], value, value_length);
    reply_length = (uint16_t)(reply_length + value_length);
}

void protocol_binary_reply_end(void)
{
    uint8_t framed[PACKET_FRAMED_MAX];

    if (reply_overflowed != 0u) {
        /* Replace the whole reply rather than send a partial one. A
         * truncated payload would parse as a complete but shorter set of
         * fields, which the reader would have no way to question. */
        reply_length = (uint16_t)packet_write_u8(reply_payload, 0u,
                                                 PROTOCOL_BINARY_STATUS_ERROR);
        reply_overflowed = 0u;
    }

    size_t framed_length = packet_build(PACKET_TYPE_RESPONSE,
                                        reply_command_id,
                                        reply_payload,
                                        reply_length,
                                        framed, sizeof framed);
    if (framed_length > 0u) {
        usb_tx_bytes(framed, (uint16_t)framed_length);
    }

    reply_length = 0u;
}

/* ------------------------------------------------------------------
 * Receiving and dispatch
 * ------------------------------------------------------------------ */

/* Turn a command packet's payload into an argument block.
 *
 * The payload is a sequence of type-tagged values. Index 0 of the
 * argument block is left unused so that a handler indexes arguments the
 * same way in both formats -- in text, index 0 is the command name.
 *
 * @param packet  the received command packet
 * @param args    the argument block to fill in */
static void unpack_arguments(const packet_t *packet, protocol_args_t *args)
{
    args->source_mode = PROTOCOL_MODE_BINARY;
    args->count       = 1u;          /* index 0 reserved, as in text */
    args->values[0]   = 0;

    uint16_t at = 0u;

    while ((at < packet->payload_length)
             && (args->count < PROTOCOL_MAX_ARGUMENTS)) {

        uint8_t argument_type = packet->payload[at];
        at++;

        if (argument_type != PROTOCOL_BINARY_ARG_I32) {
            /* An unrecognised type makes everything after it
             * unreadable, since its length is unknown. Stopping here
             * gives the handler the arguments understood so far, and its
             * own argument-count check will reject the call. */
            break;
        }
        if ((uint32_t)(at + 4u) > packet->payload_length) {
            break;
        }

        args->values[args->count] = packet_read_i32(packet->payload, at);
        args->count++;
        at = (uint16_t)(at + 4u);
    }
}

void protocol_binary_init(void)
{
    packet_receiver_init(&receiver);
    reply_length     = 0u;
    reply_overflowed = 0u;
    reply_command_id = 0u;
}

uint8_t protocol_binary_receive_byte(uint8_t received_byte)
{
    uint8_t result = packet_receive_byte(&receiver, received_byte, &received);

    if (result == PACKET_RESULT_NONE) {
        return 0u;
    }

    if (result != PACKET_RESULT_OK) {
        /* Reported through the shared reply path, so the message arrives
         * in whichever format is currently active -- readable in a
         * terminal while testing, machine-readable once an application
         * is driving. */
        protocol_reply_error("pkt", packet_result_text(result));
        return 1u;
    }

    if (received.type != PACKET_TYPE_COMMAND) {
        protocol_reply_error("pkt", "not_a_command");
        return 1u;
    }

    protocol_args_t args;
    unpack_arguments(&received, &args);

    const char *name = commands_name_for(received.id);

    if (name == NULL) {
        protocol_reply_error("pkt", "unknown_command_id");
        return 1u;
    }

    commands_dispatch(name, &args);
    return 1u;
}

void protocol_binary_discard_frame(void)
{
    receiver.frame_length = 0u;
    receiver.overflowed   = 0u;
}

void protocol_binary_get_counts(uint32_t *accepted_out,
                                uint32_t *rejected_out)
{
    if (accepted_out != NULL) {
        *accepted_out = receiver.packets_accepted;
    }
    if (rejected_out != NULL) {
        *rejected_out = receiver.packets_rejected;
    }
}
