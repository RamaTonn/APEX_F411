/*
 * packet.h
 *
 * The binary application protocol: builds outgoing packets, and
 * reassembles incoming ones from the byte stream that USB delivers.
 *
 * THE LAYERS, AND WHY THEY ARE SEPARATE
 *
 *     USB CDC      moves bytes, and makes no promise about where one
 *                  message ends and the next begins
 *     COBS         removes every zero byte, so a zero byte can only mean
 *                  end of frame
 *     CRC          says whether the bytes inside the frame are the ones
 *                  that were sent
 *     this module  gives those bytes a structure: version, type, id,
 *                  length, payload
 *
 *   Framing and integrity are genuinely different problems. COBS finds
 *   the right boundaries but says nothing about the contents; the CRC
 *   checks contents but cannot tell you the frame was cut in the right
 *   place. Each needs its own mechanism.
 *
 * ON THE WIRE
 *
 *     +---------+------+----+--------+---------+-------+
 *     | VERSION | TYPE | ID | LENGTH | PAYLOAD | CRC16 |
 *     |    1    |  1   | 1  |   2    |  0..N   |   2   |
 *     +---------+------+----+--------+---------+-------+
 *
 *   That whole structure is then COBS encoded and a single 0x00 is
 *   appended:
 *
 *     COBS( VERSION TYPE ID LENGTH PAYLOAD CRC16 ) 0x00
 *
 *   LENGTH counts the PAYLOAD only -- not the header, not the CRC.
 *   Counting only the variable part means the fixed part never has to be
 *   remembered or adjusted, which is one fewer thing for the two sides to
 *   disagree about.
 *
 *   The CRC covers VERSION, TYPE, ID, LENGTH and PAYLOAD. It does not
 *   cover itself, and it is computed before COBS encoding, so it protects
 *   the packet contents rather than the framing.
 *
 *   Multi-byte fields are LITTLE-ENDIAN: least significant byte first.
 *   0x1234 is transmitted as 34 12. This matches the STM32's native
 *   layout, so serialising costs a copy rather than a shuffle.
 *
 * THE VERSION BYTE
 *
 *   Present so the two sides can notice they disagree. Without it, a
 *   receiver meeting a packet format it does not know parses it anyway
 *   and produces plausible but wrong numbers -- a setpoint at the wrong
 *   scale, a telemetry channel plotting nonsense. With it, the mismatch
 *   is reported instead of acted on. One byte to turn a silent misparse
 *   into an explicit error.
 */

#ifndef PACKET_H_
#define PACKET_H_

#include <stdint.h>
#include <stddef.h>

/* Included rather than forward-declared because COBS_ENCODED_MAX is a
 * macro used below to size buffers. */
#include "cobs.h"

/* Protocol version. Increment when a change would make an older
 * implementation misread a packet rather than merely miss a feature. */
#define PACKET_VERSION 1u

/* Largest payload accepted or produced.
 *
 * The length field is 16 bits and could describe far more; this is a RAM
 * decision, not a format one. Several buffers are sized from it, so
 * raising it costs memory in more than one place. Raise it when a
 * telemetry frame genuinely needs more, and remember that the PC side
 * must agree -- a sender using a larger maximum than the receiver
 * allocates will have its frames rejected. */
#define PACKET_MAX_PAYLOAD 256u

/* Fixed fields before the payload: version, type, id, and the two length
 * bytes. */
#define PACKET_HEADER_SIZE 5u

/* Size of the CRC field. */
#define PACKET_CRC_SIZE 2u

/* Largest complete packet before COBS encoding. */
#define PACKET_MAX_DECODED \
    (PACKET_HEADER_SIZE + PACKET_MAX_PAYLOAD + PACKET_CRC_SIZE)

/* ------------------------------------------------------------------
 * Packet types
 *
 * Direction is a convention, not a rule the code enforces: meaning comes
 * from the type field rather than from which way the packet travelled.
 * ------------------------------------------------------------------ */

#define PACKET_TYPE_COMMAND   0x01u  /* PC to board: do something      */
#define PACKET_TYPE_TELEMETRY 0x02u  /* board to PC: periodic samples  */
#define PACKET_TYPE_RESPONSE  0x03u  /* board to PC: answer to command */
#define PACKET_TYPE_EVENT     0x04u  /* board to PC: unprompted news   */

/* ------------------------------------------------------------------
 * Results from the receiver
 * ------------------------------------------------------------------ */

#define PACKET_RESULT_NONE           0u  /* nothing complete yet        */
#define PACKET_RESULT_OK             1u  /* a valid packet is ready     */
#define PACKET_RESULT_ERR_COBS       2u  /* frame would not decode      */
#define PACKET_RESULT_ERR_TOO_SHORT  3u  /* smaller than a header + CRC */
#define PACKET_RESULT_ERR_TOO_LONG   4u  /* beyond PACKET_MAX_DECODED   */
#define PACKET_RESULT_ERR_CRC        5u  /* contents corrupted          */
#define PACKET_RESULT_ERR_LENGTH     6u  /* length disagrees with frame */
#define PACKET_RESULT_ERR_VERSION    7u  /* unknown protocol version    */

/* A received packet, after decoding and checking. */
typedef struct {
    uint8_t  version;                        /* always PACKET_VERSION   */
    uint8_t  type;                           /* one of PACKET_TYPE_*    */
    uint8_t  id;                             /* meaning depends on type */
    uint16_t payload_length;                 /* bytes actually present  */
    uint8_t  payload[PACKET_MAX_PAYLOAD];
} packet_t;

/* ------------------------------------------------------------------
 * Building packets to send
 * ------------------------------------------------------------------ */

/**
 * Build a complete framed packet, ready to hand to the transport.
 *
 * Assembles the header, appends the CRC, COBS encodes the result and
 * appends the delimiter. The output contains exactly one zero byte, at
 * the very end.
 *
 * @param type             one of the PACKET_TYPE_ constants
 * @param id               identifies which command, response, telemetry
 *                         frame or event this is
 * @param payload          the payload bytes; may be NULL if length is 0
 * @param payload_length   how many payload bytes, up to
 *                         PACKET_MAX_PAYLOAD
 * @param output           where to write the framed packet. Size it with
 *                         PACKET_FRAMED_MAX to guarantee it always fits.
 * @param output_capacity  size of that buffer
 * @return number of bytes written, or 0 if the payload was too long or
 *         the output buffer too small
 */
size_t packet_build(uint8_t        type,
                    uint8_t        id,
                    const uint8_t *payload,
                    uint16_t       payload_length,
                    uint8_t       *output,
                    size_t         output_capacity);

/* Largest framed packet: the encoded form of the largest decoded packet,
 * plus the one delimiter byte. Use this to size a transmit buffer so
 * that building can never fail for lack of room. */
#define PACKET_ENCODED_MAX COBS_ENCODED_MAX(PACKET_MAX_DECODED)
#define PACKET_FRAMED_MAX  (PACKET_ENCODED_MAX + 1u)

/* ------------------------------------------------------------------
 * Receiving packets
 *
 * Bytes arrive from USB in arbitrary chunks: a frame may be split across
 * several, and several frames may arrive together. The receiver
 * accumulates bytes until it meets a delimiter, so the caller can feed
 * it whatever it happens to have.
 * ------------------------------------------------------------------ */

/* Receiver state. The caller owns this; nothing here is global, so a
 * second link could be added without touching this module. */
typedef struct {
    /* Encoded bytes gathered since the last delimiter. */
    uint8_t  frame[PACKET_ENCODED_MAX];
    uint16_t frame_length;

    /* Set when the frame being gathered has already exceeded the buffer.
     * The rest of it is discarded rather than wrapped or truncated: a
     * truncated frame would fail its CRC anyway, and wrapping would
     * corrupt the next one. Cleared at the following delimiter. */
    uint8_t  overflowed;

    /* Counters, for diagnosing a link that is misbehaving rather than
     * merely idle. */
    uint32_t packets_accepted;
    uint32_t packets_rejected;
} packet_receiver_t;

/**
 * Prepare a receiver. Call once before feeding it anything.
 *
 * @param receiver  the receiver state to initialise
 */
void packet_receiver_init(packet_receiver_t *receiver);

/**
 * Feed one received byte.
 *
 * Returns PACKET_RESULT_NONE for every byte that does not complete a
 * frame, which is almost all of them. On a delimiter it decodes, checks
 * and reports the outcome.
 *
 * An error result is not a reason to reset anything. Because a zero byte
 * cannot occur inside a valid frame, the receiver is already correctly
 * positioned for the next one -- a corrupted frame costs exactly one
 * frame and never desynchronises the stream. That self-recovery is the
 * property COBS exists to provide.
 *
 * @param receiver     receiver state
 * @param byte         the byte just received
 * @param packet_out   filled in only when the result is
 *                     PACKET_RESULT_OK; untouched otherwise
 * @return one of the PACKET_RESULT_ constants
 */
uint8_t packet_receive_byte(packet_receiver_t *receiver,
                            uint8_t            byte,
                            packet_t          *packet_out);

/**
 * Turn a result code into a short word, for logging or a reply.
 *
 * @param result  one of the PACKET_RESULT_ constants
 * @return a static string, never NULL
 */
const char *packet_result_text(uint8_t result);

/* ------------------------------------------------------------------
 * Payload serialisation
 *
 * Explicit field-by-field, never by casting a struct to bytes. A struct
 * cast carries the compiler's padding and alignment choices onto the
 * wire, where the other side has no way to know about them.
 *
 * Each function returns the new offset, so calls chain naturally:
 *
 *     size_t at = 0;
 *     at = packet_write_u16(buffer, at, channel_id);
 *     at = packet_write_i32(buffer, at, current_ma);
 *
 * None of these bounds-check. The caller knows its buffer size and the
 * fields it is writing; adding a check to every field would cost more in
 * clutter than it saves.
 * ------------------------------------------------------------------ */

size_t packet_write_u8(uint8_t *buffer, size_t offset, uint8_t value);
size_t packet_write_u16(uint8_t *buffer, size_t offset, uint16_t value);
size_t packet_write_u32(uint8_t *buffer, size_t offset, uint32_t value);
size_t packet_write_i16(uint8_t *buffer, size_t offset, int16_t value);
size_t packet_write_i32(uint8_t *buffer, size_t offset, int32_t value);

uint8_t  packet_read_u8(const uint8_t *buffer, size_t offset);
uint16_t packet_read_u16(const uint8_t *buffer, size_t offset);
uint32_t packet_read_u32(const uint8_t *buffer, size_t offset);
int16_t  packet_read_i16(const uint8_t *buffer, size_t offset);
int32_t  packet_read_i32(const uint8_t *buffer, size_t offset);

#endif /* PACKET_H_ */
