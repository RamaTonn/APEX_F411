/*
 * protocol_text.h
 *
 * The human-readable wire format: one command per line, one reply per
 * line, everything printable.
 *
 *     host -> board   sense
 *                     duty a 500
 *
 *     board -> host   ok sense ia_ma=40 vbus_mv=11970
 *                     err duty reason=bad_phase
 *                     dat spin eangle=18401
 *
 *   Lines from the host end in CR, LF, or both. Replies end in CR LF.
 *
 *   The first word is always ok, err, or dat, so a reader can tell
 *   success from failure from unsolicited data without knowing anything
 *   about the particular command.
 *
 * WHY THIS FORMAT STILL EXISTS ALONGSIDE THE BINARY ONE
 *
 *   It can be driven from any terminal with no software written. Several
 *   real faults in this project were diagnosed by typing a command and
 *   reading the answer, at points when no application existed that could
 *   have talked to the board at all. Keeping it costs little and
 *   preserves that.
 *
 * NAMED FIELDS
 *
 *   Replies are key=value rather than bare positional values. A reader
 *   splits on spaces and then on the equals sign, and fields can be
 *   added, reordered or omitted without breaking anything already
 *   written. Positional output would silently misalign the moment a
 *   field was inserted.
 */

#ifndef PROTOCOL_TEXT_H_
#define PROTOCOL_TEXT_H_

#include <stdint.h>
#include "protocol.h"

/* Longest command line accepted. A longer line is discarded whole and
 * reported, rather than truncated: a truncated line can be a different
 * valid command, which would be executed without anyone intending it. */
#define PROTOCOL_TEXT_LINE_CAPACITY 96U

/**
 * Reset the line assembler. Echo is left off.
 */
void protocol_text_init(void);

/**
 * Offer one received byte to the text assembler.
 *
 * Printable characters are accumulated; CR or LF completes a line, which
 * is then tokenised and dispatched. Everything else is ignored, which is
 * what keeps binary frame bytes from polluting a typed command.
 *
 * @param received_byte  the byte just received
 * @return 1 if this byte completed a line, 0 otherwise. The caller uses
 *         this to discard any partly-gathered binary frame, since the
 *         bytes of that line will also have reached the packet receiver
 *         on their way past.
 */
uint8_t protocol_text_receive_byte(uint8_t received_byte);

/**
 * Discard the partly-typed line.
 *
 * Called when a binary frame completes, because a payload can contain
 * bytes that happen to be printable and those will have been appended on
 * their way past. Without this they would prefix the next typed command.
 */
void protocol_text_discard_line(void);

/**
 * Turn character echo on or off.
 *
 * @param enabled  non-zero to echo typed characters back
 */
void protocol_text_set_echo(uint8_t enabled);

/**
 * @return 1 if echo is on
 */
uint8_t protocol_text_get_echo(void);

/* Reply building. Called by protocol.c when text is the active reply
 * format; not called directly by command handlers. */
void protocol_text_reply_begin(const char *status, const char *command);
void protocol_text_reply_uint(const char *key, uint32_t value);
void protocol_text_reply_int(const char *key, int32_t value);
void protocol_text_reply_text(const char *key, const char *value);
void protocol_text_reply_end(void);

#endif /* PROTOCOL_TEXT_H_ */
