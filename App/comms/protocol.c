#include "protocol.h"

#include "protocol_text.h"
#include "protocol_binary.h"
#include "USB_Comm.h"

#include <stdlib.h>
#include <string.h>

/* Which format replies are written in. Both formats are always accepted
 * on the way in -- see protocol_set_mode() for why. */
static uint8_t active_mode;

/* ------------------------------------------------------------------
 * Argument accessors
 *
 * A handler asks for "argument 2 as a number" and these work out what
 * that means for whichever format delivered the command.
 * ------------------------------------------------------------------ */

uint32_t protocol_arg_count(const protocol_args_t *args)
{
    return (args != NULL) ? args->count : 0u;
}

int32_t protocol_arg_int(const protocol_args_t *args,
                         uint32_t               index,
                         int32_t                fallback_value)
{
    if ((args == NULL) || (index >= args->count)) {
        return fallback_value;
    }

    if (args->source_mode == PROTOCOL_MODE_BINARY) {
        return args->values[index];
    }

    /* Text mode. atoi returns 0 for anything it cannot parse, which is
     * indistinguishable from a genuine zero -- so the string is checked
     * for at least one digit first, and the caller's fallback is used
     * when there is none. That way a handler can tell "they typed 0"
     * from "they typed nonsense" if it needs to. */
    const char *word = args->words[index];
    if (word == NULL) {
        return fallback_value;
    }

    uint8_t has_digit = 0u;
    for (const char *c = word; *c != '\0'; c++) {
        if ((*c >= '0') && (*c <= '9')) {
            has_digit = 1u;
        }
    }
    if (has_digit == 0u) {
        return fallback_value;
    }

    return (int32_t)atoi(word);
}

const char *protocol_arg_text(const protocol_args_t *args, uint32_t index)
{
    if ((args == NULL) || (index >= args->count)) {
        return "";
    }
    if (args->source_mode != PROTOCOL_MODE_TEXT) {
        /* Binary carries numbers, not words. Returning an empty string
         * rather than NULL means a caller can pass the result straight
         * to strcmp without checking. */
        return "";
    }
    return (args->words[index] != NULL) ? args->words[index] : "";
}

uint8_t protocol_arg_matches(const protocol_args_t *args,
                             uint32_t               index,
                             const char            *keyword,
                             int32_t                binary_equivalent)
{
    if ((args == NULL) || (index >= args->count)) {
        return 0u;
    }

    if (args->source_mode == PROTOCOL_MODE_BINARY) {
        return (args->values[index] == binary_equivalent) ? 1u : 0u;
    }

    const char *word = args->words[index];
    if (word == NULL) {
        return 0u;
    }
    return (strcmp(word, keyword) == 0) ? 1u : 0u;
}

/* ------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------ */

void protocol_init(void)
{
    active_mode = PROTOCOL_MODE_TEXT;

    protocol_text_init();
    protocol_binary_init();
}

void protocol_poll(void)
{
    uint8_t received_byte;

    if (usb_rx_is_overflow() != 0u) {
        protocol_reply_error("?", "rx_overflow");
    }

    while (usb_rx_dequeue(&received_byte) != 0u) {

        /* Every byte goes to the packet receiver, because a frame is
         * only recognisable once its delimiter arrives -- there is no
         * way to know in advance which bytes belong to one. */
        uint8_t frame_completed = protocol_binary_receive_byte(received_byte);

        if (frame_completed != 0u) {
            /* A binary payload can contain bytes that happen to be
             * printable, and those will have reached the text assembler
             * on their way past. Clearing the partial line stops them
             * from prefixing the next typed command. */
            protocol_text_discard_line();
            continue;
        }

        /* Not part of a completed frame, so offer it to the text
         * assembler. Frame bytes that are not printable are dropped
         * there anyway, which is what lets the two formats share one
         * stream without interfering. */
        if (protocol_text_receive_byte(received_byte) != 0u) {
            /* A text line just completed, so the bytes the packet
             * receiver gathered were that line, not a frame. */
            protocol_binary_discard_frame();
        }
    }
}

uint8_t protocol_get_mode(void)
{
    return active_mode;
}

uint8_t protocol_set_mode(uint8_t mode)
{
    if ((mode != PROTOCOL_MODE_TEXT) && (mode != PROTOCOL_MODE_BINARY)) {
        return 0u;
    }
    active_mode = mode;
    return 1u;
}

void protocol_set_echo(uint8_t enabled)
{
    protocol_text_set_echo(enabled);
}

uint8_t protocol_get_echo(void)
{
    return protocol_text_get_echo();
}

/* ------------------------------------------------------------------
 * Reply routing
 *
 * Each of these forwards to whichever backend is active. Handlers call
 * only these, and so never learn which format they are producing.
 * ------------------------------------------------------------------ */

void protocol_reply_begin(const char *status, const char *command)
{
    if (active_mode == PROTOCOL_MODE_BINARY) {
        protocol_binary_reply_begin(status, command);
    } else {
        protocol_text_reply_begin(status, command);
    }
}

void protocol_reply_uint(const char *key, uint32_t value)
{
    if (active_mode == PROTOCOL_MODE_BINARY) {
        protocol_binary_reply_uint(key, value);
    } else {
        protocol_text_reply_uint(key, value);
    }
}

void protocol_reply_int(const char *key, int32_t value)
{
    if (active_mode == PROTOCOL_MODE_BINARY) {
        protocol_binary_reply_int(key, value);
    } else {
        protocol_text_reply_int(key, value);
    }
}

void protocol_reply_text(const char *key, const char *value)
{
    if (active_mode == PROTOCOL_MODE_BINARY) {
        protocol_binary_reply_text(key, value);
    } else {
        protocol_text_reply_text(key, value);
    }
}

void protocol_reply_end(void)
{
    if (active_mode == PROTOCOL_MODE_BINARY) {
        protocol_binary_reply_end();
    } else {
        protocol_text_reply_end();
    }
}

void protocol_reply_error(const char *command, const char *reason)
{
    protocol_reply_begin(PROTOCOL_STATUS_ERROR, command);
    protocol_reply_text("reason", reason);
    protocol_reply_end();
}
