/*
 * protocol_binary.h
 *
 * The machine-readable wire format: COBS-framed packets with a CRC,
 * carrying commands in and responses out.
 *
 * WHAT IT ADDS OVER THE TEXT FORMAT
 *
 *   Corruption is detected rather than acted on. A text reply with a
 *   flipped bit is still a well-formed line, and a reader has no way to
 *   know a number is wrong. A packet carries a checksum, so damage is
 *   caught instead of believed.
 *
 *   Frame boundaries are unambiguous. COBS guarantees no zero byte
 *   appears inside a frame, so a receiver that loses its place recovers
 *   at the very next delimiter with no guessing.
 *
 *   It is compact and cheap to parse: no string formatting on the way
 *   out, no tokenising on the way in.
 *
 * COMMAND PACKETS, HOST TO BOARD
 *
 *   type = PACKET_TYPE_COMMAND, id = which command.
 *
 *   The payload holds the arguments, one after another:
 *
 *       [ARG_TYPE:1][VALUE:4]  repeated
 *
 *   with ARG_TYPE saying how to read the four bytes. Only signed 32-bit
 *   integers are defined so far, which covers every argument any current
 *   command takes.
 *
 * RESPONSE PACKETS, BOARD TO HOST
 *
 *   type = PACKET_TYPE_RESPONSE, id = the command being answered.
 *
 *       [STATUS:1][FIELD_ID:1][VALUE:4] ...
 *
 *   STATUS distinguishes success from failure. Each field that followed
 *   in the text form as name=value becomes a one-byte id and four bytes
 *   of value.
 *
 * FIELD IDS
 *
 *   A handler names its fields with strings, because that is what reads
 *   well in text mode. Binary replaces each name with a number from a
 *   shared table.
 *
 *   A name absent from that table is not dropped: it is sent with id
 *   zero and the name included as text. Losing a field silently would be
 *   far worse than sending it inefficiently, and this way a field added
 *   to a handler works immediately and can be given an id later.
 */

#ifndef PROTOCOL_BINARY_H_
#define PROTOCOL_BINARY_H_

#include <stdint.h>
#include "protocol.h"
#include "packet.h"

/* Argument types inside a command payload. */
#define PROTOCOL_BINARY_ARG_I32 0x01u

/* Status byte at the head of a response payload. */
#define PROTOCOL_BINARY_STATUS_OK    0x00u
#define PROTOCOL_BINARY_STATUS_ERROR 0x01u
#define PROTOCOL_BINARY_STATUS_DATA  0x02u

/* Field id used when a name has no number assigned. The name follows as
 * a length-prefixed string, so nothing is lost. */
#define PROTOCOL_BINARY_FIELD_NAMED 0x00u

/**
 * Reset the packet receiver.
 */
void protocol_binary_init(void);

/**
 * Offer one received byte to the packet receiver.
 *
 * Every byte must be offered, not only the delimiters: a frame is only
 * recognisable once complete, so there is no way to know in advance
 * which bytes belong to one.
 *
 * @param received_byte  the byte just received
 * @return 1 if this byte completed a frame, whether or not the frame was
 *         valid. The caller uses this to discard any partly-typed text
 *         line, since a payload can contain printable bytes that the
 *         text assembler will have collected on their way past.
 */
uint8_t protocol_binary_receive_byte(uint8_t received_byte);

/**
 * Throw away the partly-gathered frame.
 *
 * Called when a text line completes. Every received byte is offered to
 * the packet receiver, because a frame is only recognisable once its
 * delimiter arrives and there is no way to know in advance which bytes
 * belong to one. So a typed command leaves its characters sitting in the
 * frame buffer.
 *
 * Left there, they would be treated as the opening bytes of the next
 * real frame, which would then fail to decode -- the symptom being that
 * binary works until someone types something, and then stops.
 *
 * Only the partial frame is cleared. The accepted and rejected counters
 * describe the link's history and are left alone, since a discarded
 * partial frame was never a frame at all.
 */
void protocol_binary_discard_frame(void);

/**
 * How many packets have been accepted and rejected since init.
 *
 * A rising rejection count on an otherwise working link points at
 * electrical noise or a length mismatch between the two sides.
 *
 * @param accepted_out  where to store the accepted count; may be NULL
 * @param rejected_out  where to store the rejected count; may be NULL
 */
void protocol_binary_get_counts(uint32_t *accepted_out,
                                uint32_t *rejected_out);

/* Reply building. Called by protocol.c when binary is the active reply
 * format; not called directly by command handlers. */
void protocol_binary_reply_begin(const char *status, const char *command);
void protocol_binary_reply_uint(const char *key, uint32_t value);
void protocol_binary_reply_int(const char *key, int32_t value);
void protocol_binary_reply_text(const char *key, const char *value);
void protocol_binary_reply_end(void);

#endif /* PROTOCOL_BINARY_H_ */
