#include "protocol_text.h"

#include "commands.h"
#include "USB_Comm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------
 * Line assembly
 * ------------------------------------------------------------------ */

/* Characters gathered for the command currently being typed. Not
 * zero-terminated until a line ending arrives. */
static char     line_buffer[PROTOCOL_TEXT_LINE_CAPACITY];
static uint32_t line_length;

/* Set once a line has been abandoned for being too long. Everything up
 * to the next line ending is then thrown away, so the tail of an
 * overlong line is not mistaken for a command of its own. */
static uint8_t discarding_line;

/* Whether typed characters are sent back. Off by default: the usual
 * caller is a program, for which echo is noise to be filtered. */
static uint8_t echo_enabled;

/* ------------------------------------------------------------------
 * Reply building
 * ------------------------------------------------------------------ */

static char     reply_buffer[PROTOCOL_REPLY_CAPACITY];
static uint32_t reply_length;

/* Append text, stopping at capacity rather than overrunning.
 *
 * Two bytes are always held back for the CR LF that ends every reply, so
 * a reply that runs long loses its tail rather than its terminator --
 * a reader would otherwise wait forever for a line that never ends.
 *
 * @param text  zero-terminated string to append */
static void reply_append(const char *text)
{
    uint32_t space_remaining = PROTOCOL_REPLY_CAPACITY - 2u - reply_length;
    uint32_t text_length     = (uint32_t)strlen(text);

    if (text_length > space_remaining) {
        text_length = space_remaining;
    }
    memcpy(&reply_buffer[reply_length], text, text_length);
    reply_length += text_length;
}

void protocol_text_reply_begin(const char *status, const char *command)
{
    reply_length = 0u;
    reply_append(status);
    reply_append(" ");
    reply_append(command);
}

void protocol_text_reply_uint(const char *key, uint32_t value)
{
    char number_text[12];

    snprintf(number_text, sizeof number_text, "%lu", (unsigned long)value);

    reply_append(" ");
    reply_append(key);
    reply_append("=");
    reply_append(number_text);
}

void protocol_text_reply_int(const char *key, int32_t value)
{
    char number_text[13];

    snprintf(number_text, sizeof number_text, "%ld", (long)value);

    reply_append(" ");
    reply_append(key);
    reply_append("=");
    reply_append(number_text);
}

void protocol_text_reply_text(const char *key, const char *value)
{
    reply_append(" ");
    reply_append(key);
    reply_append("=");
    reply_append(value);
}

void protocol_text_reply_end(void)
{
    reply_buffer[reply_length++] = '\r';
    reply_buffer[reply_length++] = '\n';

    usb_tx_bytes(reply_buffer, (uint16_t)reply_length);
    reply_length = 0u;
}

/* ------------------------------------------------------------------
 * Dispatch
 * ------------------------------------------------------------------ */

/* Split a completed line into words and run the matching handler.
 *
 * strtok replaces each delimiter with a zero terminator, so this
 * destroys line_buffer -- which is fine, the line is finished with. The
 * pointers it produces point into line_buffer and stay valid only until
 * the next line is assembled, which is why the handler runs immediately
 * rather than the words being stored.
 *
 * @param line  the completed, zero-terminated line */
static void dispatch_line(char *line)
{
    protocol_args_t args;

    args.source_mode = PROTOCOL_MODE_TEXT;
    args.count       = 0u;

    char *word = strtok(line, " \t");
    while ((word != NULL) && (args.count < PROTOCOL_MAX_ARGUMENTS)) {
        args.words[args.count] = word;
        args.count++;
        word = strtok(NULL, " \t");
    }

    if (args.count == 0u) {
        return;
    }

    commands_dispatch(args.words[0], &args);
}

/* Handle a completed line: report a discarded one, or run a real one. */
static void end_of_line(void)
{
    if (echo_enabled != 0u) {
        usb_tx_string("\r\n");
    }

    if (discarding_line != 0u) {
        protocol_reply_error("?", "line_too_long");
        discarding_line = 0u;
    } else if (line_length > 0u) {
        line_buffer[line_length] = '\0';
        dispatch_line(line_buffer);
    }

    line_length = 0u;
}

/* Remove the last typed character. */
static void backspace(void)
{
    if (line_length > 0u) {
        line_length--;
        if (echo_enabled != 0u) {
            /* Move left, overwrite with a space, move left again --
             * which is what actually erases a character on a terminal. */
            usb_tx_string("\b \b");
        }
    }
}

/* Append one printable character, or begin discarding if full. */
static void append_character(uint8_t received_byte)
{
    /* One slot is reserved for the zero terminator added at end of
     * line, hence the minus one. */
    if (line_length < (PROTOCOL_TEXT_LINE_CAPACITY - 1u)) {
        line_buffer[line_length] = (char)received_byte;
        line_length++;
        if (echo_enabled != 0u) {
            usb_tx_bytes(&received_byte, 1u);
        }
    } else {
        discarding_line = 1u;
        line_length     = 0u;
    }
}

/* ------------------------------------------------------------------
 * Entry points
 * ------------------------------------------------------------------ */

void protocol_text_init(void)
{
    line_length     = 0u;
    discarding_line = 0u;
    echo_enabled    = 0u;
    reply_length    = 0u;
}

uint8_t protocol_text_receive_byte(uint8_t received_byte)
{
    /* Carriage return or line feed ends the line. A host sending both
     * produces one straight after the other, so the second lands on an
     * empty line -- which is why an empty line must be ignored rather
     * than treated as a command. */
    if ((received_byte == '\r') || (received_byte == '\n')) {
        end_of_line();
        return 1u;
    }

    if (discarding_line != 0u) {
        return 0u;
    }

    /* Backspace or delete. */
    if ((received_byte == 0x08u) || (received_byte == 0x7Fu)) {
        backspace();
        return 0u;
    }

    /* Printable ASCII only. Control characters and anything above 0x7E
     * are dropped, which is also what keeps the bytes of a binary frame
     * out of the command buffer as they pass through. */
    if ((received_byte >= 0x20u) && (received_byte <= 0x7Eu)) {
        append_character(received_byte);
    }

    return 0u;
}

void protocol_text_discard_line(void)
{
    line_length     = 0u;
    discarding_line = 0u;
}

void protocol_text_set_echo(uint8_t enabled)
{
    echo_enabled = (enabled != 0u) ? 1u : 0u;
}

uint8_t protocol_text_get_echo(void)
{
    return echo_enabled;
}
