#include "packet.h"

#include "cobs.h"
#include "crc16.h"

#include <string.h>

/* Delimiter that marks the end of a framed packet.
 *
 * This can be a fixed byte only because COBS guarantees the encoded data
 * contains no zeros. Without that guarantee the receiver would find this
 * value inside payloads and cut frames in the wrong place. */
#define PACKET_DELIMITER 0x00u

/* Offsets of the fixed fields within a decoded packet. */
#define OFFSET_VERSION 0u
#define OFFSET_TYPE    1u
#define OFFSET_ID      2u
#define OFFSET_LENGTH  3u   /* two bytes, little-endian */
#define OFFSET_PAYLOAD 5u

/* Smallest a decoded packet can be: header plus CRC, with no payload. */
#define MINIMUM_DECODED_SIZE (PACKET_HEADER_SIZE + PACKET_CRC_SIZE)

/* ------------------------------------------------------------------
 * Serialisation
 *
 * Little-endian throughout: least significant byte first. Written out a
 * byte at a time rather than by pointer casting, because a cast would
 * depend on the machine's own byte order and on the address being
 * suitably aligned. Doing it explicitly is correct on any machine and
 * needs no alignment at all.
 * ------------------------------------------------------------------ */

size_t packet_write_u8(uint8_t *buffer, size_t offset, uint8_t value)
{
    buffer[offset] = value;
    return offset + 1u;
}

size_t packet_write_u16(uint8_t *buffer, size_t offset, uint16_t value)
{
    buffer[offset]      = (uint8_t)(value & 0xFFu);
    buffer[offset + 1u] = (uint8_t)((value >> 8) & 0xFFu);
    return offset + 2u;
}

size_t packet_write_u32(uint8_t *buffer, size_t offset, uint32_t value)
{
    buffer[offset]      = (uint8_t)(value & 0xFFu);
    buffer[offset + 1u] = (uint8_t)((value >> 8) & 0xFFu);
    buffer[offset + 2u] = (uint8_t)((value >> 16) & 0xFFu);
    buffer[offset + 3u] = (uint8_t)((value >> 24) & 0xFFu);
    return offset + 4u;
}

/* Signed values are converted to unsigned before shifting.
 *
 * Shifting a negative signed value is not fully defined in C, and the
 * result would depend on the compiler. Converting first makes the
 * behaviour the two's-complement bit pattern in every case, which is
 * what the wire format specifies. */
size_t packet_write_i16(uint8_t *buffer, size_t offset, int16_t value)
{
    return packet_write_u16(buffer, offset, (uint16_t)value);
}

size_t packet_write_i32(uint8_t *buffer, size_t offset, int32_t value)
{
    return packet_write_u32(buffer, offset, (uint32_t)value);
}

uint8_t packet_read_u8(const uint8_t *buffer, size_t offset)
{
    return buffer[offset];
}

uint16_t packet_read_u16(const uint8_t *buffer, size_t offset)
{
    return (uint16_t)((uint16_t)buffer[offset]
                    | ((uint16_t)buffer[offset + 1u] << 8));
}

uint32_t packet_read_u32(const uint8_t *buffer, size_t offset)
{
    return (uint32_t)buffer[offset]
         | ((uint32_t)buffer[offset + 1u] << 8)
         | ((uint32_t)buffer[offset + 2u] << 16)
         | ((uint32_t)buffer[offset + 3u] << 24);
}

int16_t packet_read_i16(const uint8_t *buffer, size_t offset)
{
    return (int16_t)packet_read_u16(buffer, offset);
}

int32_t packet_read_i32(const uint8_t *buffer, size_t offset)
{
    return (int32_t)packet_read_u32(buffer, offset);
}

/* ------------------------------------------------------------------
 * Building
 * ------------------------------------------------------------------ */

size_t packet_build(uint8_t        type,
                    uint8_t        id,
                    const uint8_t *payload,
                    uint16_t       payload_length,
                    uint8_t       *output,
                    size_t         output_capacity)
{
    /* The packet is assembled here in plain form, then encoded into the
     * caller's buffer. Two buffers are needed because COBS cannot encode
     * in place -- the output is longer than the input, and the encoder
     * reads bytes it would already have overwritten. */
    uint8_t decoded[PACKET_MAX_DECODED];

    if (payload_length > PACKET_MAX_PAYLOAD) {
        return 0u;
    }
    if ((payload == NULL) && (payload_length > 0u)) {
        return 0u;
    }
    if (output == NULL) {
        return 0u;
    }

    size_t at = 0u;
    at = packet_write_u8(decoded, at, PACKET_VERSION);
    at = packet_write_u8(decoded, at, type);
    at = packet_write_u8(decoded, at, id);
    at = packet_write_u16(decoded, at, payload_length);

    if (payload_length > 0u) {
        memcpy(&decoded[at], payload, payload_length);
        at += payload_length;
    }

    /* The CRC covers everything written so far and nothing else: the
     * header and the payload, but not itself. */
    uint16_t crc = crc16_compute(decoded, at);
    at = packet_write_u16(decoded, at, crc);

    /* One zero byte is appended after encoding, so the buffer needs room
     * for the encoded form plus that delimiter. */
    size_t encoded_length = cobs_encode(decoded, at,
                                        output, output_capacity - 1u);
    if (encoded_length == 0u) {
        return 0u;
    }
    if ((encoded_length + 1u) > output_capacity) {
        return 0u;
    }

    output[encoded_length] = PACKET_DELIMITER;

    return encoded_length + 1u;
}

/* ------------------------------------------------------------------
 * Receiving
 * ------------------------------------------------------------------ */

void packet_receiver_init(packet_receiver_t *receiver)
{
    if (receiver == NULL) {
        return;
    }
    receiver->frame_length     = 0u;
    receiver->overflowed       = 0u;
    receiver->packets_accepted = 0u;
    receiver->packets_rejected = 0u;
}

/* Decode and check one gathered frame.
 *
 * @param receiver    receiver holding the frame
 * @param packet_out  filled in on success
 * @return a PACKET_RESULT_ constant */
static uint8_t process_frame(packet_receiver_t *receiver,
                             packet_t          *packet_out)
{
    uint8_t decoded[PACKET_MAX_DECODED];

    size_t decoded_length = cobs_decode(receiver->frame,
                                        receiver->frame_length,
                                        decoded, sizeof decoded);
    if (decoded_length == 0u) {
        return PACKET_RESULT_ERR_COBS;
    }
    if (decoded_length < MINIMUM_DECODED_SIZE) {
        return PACKET_RESULT_ERR_TOO_SHORT;
    }

    uint16_t payload_length = packet_read_u16(decoded, OFFSET_LENGTH);

    /* The length field must agree with how many bytes actually arrived.
     * A frame claiming more payload than it carries would otherwise be
     * read past its end, and one claiming less would leave stray bytes.
     *
     * This is checked before the CRC because it decides how much data
     * the CRC covers -- a length field that has itself been corrupted
     * must not be used to compute anything. */
    if (payload_length > PACKET_MAX_PAYLOAD) {
        return PACKET_RESULT_ERR_TOO_LONG;
    }
    if ((size_t)(PACKET_HEADER_SIZE + payload_length + PACKET_CRC_SIZE)
            != decoded_length) {
        return PACKET_RESULT_ERR_LENGTH;
    }

    /* The CRC sits in the last two bytes and covers everything before
     * them. */
    size_t   crc_offset   = decoded_length - PACKET_CRC_SIZE;
    uint16_t received_crc = packet_read_u16(decoded, crc_offset);
    uint16_t computed_crc = crc16_compute(decoded, crc_offset);

    if (received_crc != computed_crc) {
        return PACKET_RESULT_ERR_CRC;
    }

    /* Version last. A packet that fails its CRC tells us nothing
     * reliable, including what version it claims to be -- so there is no
     * point reporting a version mismatch until the contents are known to
     * be intact. */
    uint8_t version = packet_read_u8(decoded, OFFSET_VERSION);
    if (version != PACKET_VERSION) {
        return PACKET_RESULT_ERR_VERSION;
    }

    packet_out->version        = version;
    packet_out->type           = packet_read_u8(decoded, OFFSET_TYPE);
    packet_out->id             = packet_read_u8(decoded, OFFSET_ID);
    packet_out->payload_length = payload_length;

    if (payload_length > 0u) {
        memcpy(packet_out->payload, &decoded[OFFSET_PAYLOAD], payload_length);
    }

    return PACKET_RESULT_OK;
}

uint8_t packet_receive_byte(packet_receiver_t *receiver,
                            uint8_t            byte,
                            packet_t          *packet_out)
{
    if ((receiver == NULL) || (packet_out == NULL)) {
        return PACKET_RESULT_NONE;
    }

    if (byte != PACKET_DELIMITER) {

        if (receiver->frame_length < sizeof receiver->frame) {
            receiver->frame[receiver->frame_length] = byte;
            receiver->frame_length++;
        } else {
            /* Too long for the buffer. The rest is discarded rather than
             * wrapped: wrapping would corrupt whatever comes next, and a
             * truncated frame would fail its CRC anyway. */
            receiver->overflowed = 1u;
        }
        return PACKET_RESULT_NONE;
    }

    /* A delimiter arrived, so whatever was gathered is a complete
     * frame -- or a run of nothing, if two delimiters arrived together
     * or the link has just been idle. */
    uint8_t result;

    if (receiver->overflowed != 0u) {
        result = PACKET_RESULT_ERR_TOO_LONG;
    } else if (receiver->frame_length == 0u) {
        /* An empty frame is not an error. A sender may emit a leading
         * delimiter to flush the receiver into a known state, and a
         * receiver joining mid-stream sees one before its first whole
         * frame. Reporting these as errors would fill the log with noise
         * from entirely normal behaviour. */
        result = PACKET_RESULT_NONE;
    } else {
        result = process_frame(receiver, packet_out);
    }

    if (result == PACKET_RESULT_OK) {
        receiver->packets_accepted++;
    } else if (result != PACKET_RESULT_NONE) {
        receiver->packets_rejected++;
    }

    /* Start the next frame regardless of how this one turned out.
     *
     * No recovery step is needed after an error. A zero byte cannot
     * appear inside a valid frame, so this delimiter is a real boundary
     * whatever went wrong before it -- the damage is confined to one
     * frame and the stream is already back in step. That self-recovery
     * is exactly what COBS is for. */
    receiver->frame_length = 0u;
    receiver->overflowed   = 0u;

    return result;
}

const char *packet_result_text(uint8_t result)
{
    switch (result) {
        case PACKET_RESULT_NONE:          return "none";
        case PACKET_RESULT_OK:            return "ok";
        case PACKET_RESULT_ERR_COBS:      return "cobs_error";
        case PACKET_RESULT_ERR_TOO_SHORT: return "too_short";
        case PACKET_RESULT_ERR_TOO_LONG:  return "too_long";
        case PACKET_RESULT_ERR_CRC:       return "crc_error";
        case PACKET_RESULT_ERR_LENGTH:    return "length_mismatch";
        case PACKET_RESULT_ERR_VERSION:   return "version_mismatch";
        default:                          return "unknown";
    }
}
