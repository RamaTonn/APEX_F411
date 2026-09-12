/*
 * protocol.h
 *
 * The console: turns bytes from the host into command calls, and command
 * results back into bytes.
 *
 * HOW THE PIECES FIT
 *
 *     protocol.c          chooses which wire format is active and routes
 *                         to it. Nothing else knows a choice exists.
 *
 *     protocol_text.c     human-readable lines ending in CR or LF.
 *                         Typeable in a terminal, readable in a log.
 *
 *     protocol_binary.c   COBS-framed packets with a CRC. Compact,
 *                         checked, and safe to stream at rate.
 *
 *     commands.c          the handlers. One set, shared by both formats,
 *                         with no knowledge of which is in use.
 *
 * WHY THE HANDLERS ARE SHARED
 *
 *   Two copies of a command would drift. One would gain a check the
 *   other lacked, or a limit would be updated in one place only, and the
 *   difference would show up as a bug that appears in one mode and not
 *   the other -- the hardest kind to reason about.
 *
 *   Sharing them needs two things to be abstracted: how arguments arrive,
 *   and how replies are written. Both are below.
 *
 * ARGUMENTS
 *
 *   In text, arguments are words split on spaces. In binary, they are
 *   typed values packed into a payload. A handler asks for "argument 2 as
 *   an integer" and the active backend works out what that means.
 *
 * REPLIES
 *
 *   A handler calls protocol_reply_begin(), then one call per field, then
 *   protocol_reply_end(). In text that produces
 *
 *       ok sense ia_ma=40 vbus_mv=11970
 *
 *   and in binary a RESPONSE packet whose payload is the same fields with
 *   their names replaced by numeric ids. The handler cannot tell.
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>
#include <stddef.h>

/* Wire formats. */
#define PROTOCOL_MODE_TEXT   0U
#define PROTOCOL_MODE_BINARY 1U

/* Status words. Text writes them literally; binary maps them to a byte. */
#define PROTOCOL_STATUS_OK    "ok"
#define PROTOCOL_STATUS_ERROR "err"
#define PROTOCOL_STATUS_DATA  "dat"   /* unsolicited; nobody asked */

/* Longest reply either format will build. */
#define PROTOCOL_REPLY_CAPACITY 240U

/* ------------------------------------------------------------------
 * Command arguments
 *
 * Filled in by whichever backend received the command, and passed to the
 * handler. Handlers use the accessor functions rather than reaching into
 * the fields, so the same handler works for both formats.
 * ------------------------------------------------------------------ */

/* Largest number of arguments either format will present. */
#define PROTOCOL_MAX_ARGUMENTS 6U

typedef struct {
    /* Which backend produced this, so the accessors know how to read it.
     * PROTOCOL_MODE_TEXT or PROTOCOL_MODE_BINARY. */
    uint8_t source_mode;

    /* How many arguments are present, INCLUDING the command name at
     * index 0 in text mode. Binary mode has no name to include, so its
     * arguments start at index 1 too and index 0 is left empty -- that
     * way a handler's indexing is identical either way. */
    uint32_t count;

    /* Text mode: pointers to the words, cut out of the received line. */
    char *words[PROTOCOL_MAX_ARGUMENTS];

    /* Binary mode: the values, already decoded from the payload. */
    int32_t values[PROTOCOL_MAX_ARGUMENTS];
} protocol_args_t;

/**
 * How many arguments were supplied, counting the command itself.
 *
 * A command taking one parameter therefore sees 2. This matches how the
 * text form reads on screen and keeps both backends consistent.
 *
 * @param args  the argument block passed to the handler
 * @return the count
 */
uint32_t protocol_arg_count(const protocol_args_t *args);

/**
 * Read one argument as a whole number.
 *
 * In text mode the word is parsed; anything unparseable yields the
 * fallback rather than an error, so a handler decides for itself whether
 * a missing or malformed argument matters.
 *
 * @param args            the argument block
 * @param index           which argument; 1 is the first parameter
 * @param fallback_value  returned when the argument is absent or cannot
 *                        be read as a number
 * @return the value, or fallback_value
 */
int32_t protocol_arg_int(const protocol_args_t *args,
                         uint32_t               index,
                         int32_t                fallback_value);

/**
 * Read one argument as text.
 *
 * Binary mode carries numbers rather than words, so this returns an
 * empty string there. A handler that needs to compare against a keyword
 * -- "on", "off", "text" -- should use protocol_arg_matches() instead,
 * which works in both modes.
 *
 * @param args   the argument block
 * @param index  which argument; 1 is the first parameter
 * @return the text, never NULL; empty if unavailable
 */
const char *protocol_arg_text(const protocol_args_t *args, uint32_t index);

/**
 * Test an argument against a keyword.
 *
 * In text mode this compares the word. In binary mode, where there are
 * no words, it compares against the keyword's agreed numeric equivalent
 * supplied by the caller -- so a handler can accept "off" from a
 * terminal and 0 from an application with one piece of code.
 *
 * @param args             the argument block
 * @param index            which argument; 1 is the first parameter
 * @param keyword          the word to match in text mode
 * @param binary_equivalent  the value that means the same thing in
 *                         binary mode
 * @return 1 if the argument matches, 0 otherwise
 */
uint8_t protocol_arg_matches(const protocol_args_t *args,
                             uint32_t               index,
                             const char            *keyword,
                             int32_t                binary_equivalent);

/* ------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------ */

/**
 * Prepare both backends. Text mode is active afterwards, with character
 * echo OFF.
 *
 * Echo defaults off because the usual caller is an application, not a
 * person: echoed characters are noise it has to filter, and a reply that
 * arrives with the request mixed into it is harder to parse. Anyone
 * typing by hand can turn it on with "echo 1".
 */
void protocol_init(void);

/**
 * Drain received bytes and run any complete commands.
 *
 * Call from the main loop. Both backends are fed regardless of which
 * mode is active, so a host can switch modes without a handshake: a
 * binary frame is recognised even while text mode is selected, and a
 * typed line still works after switching to binary.
 */
void protocol_poll(void);

/**
 * @return PROTOCOL_MODE_TEXT or PROTOCOL_MODE_BINARY
 */
uint8_t protocol_get_mode(void);

/**
 * Choose the format used for REPLIES.
 *
 * Only the outgoing direction is affected; both formats continue to be
 * accepted on the way in. That asymmetry is deliberate -- it means a mode
 * switch can never leave the board unable to hear a command to switch
 * back.
 *
 * @param mode  PROTOCOL_MODE_TEXT or PROTOCOL_MODE_BINARY
 * @return 1 on success, 0 if the mode is not recognised
 */
uint8_t protocol_set_mode(uint8_t mode);

/**
 * Turn character echo on or off.
 *
 * Text mode only; binary has nothing to echo.
 *
 * @param enabled  non-zero to echo typed characters back
 */
void protocol_set_echo(uint8_t enabled);

/**
 * @return 1 if character echo is on
 */
uint8_t protocol_get_echo(void);

/* ------------------------------------------------------------------
 * Building replies
 *
 * Always in this order:
 *
 *     protocol_reply_begin(PROTOCOL_STATUS_OK, "sense");
 *     protocol_reply_int("ia_ma", current_a);
 *     protocol_reply_uint("vbus_mv", bus_millivolts);
 *     protocol_reply_end();
 *
 * The reply is accumulated and sent as one transfer by
 * protocol_reply_end(). Sending field by field would mean one blocking
 * USB transfer per field.
 * ------------------------------------------------------------------ */

void protocol_reply_begin(const char *status, const char *command);
void protocol_reply_uint(const char *key, uint32_t value);
void protocol_reply_int(const char *key, int32_t value);
void protocol_reply_text(const char *key, const char *value);
void protocol_reply_end(void);

/**
 * Shorthand for a failure carrying a single reason.
 *
 * @param command  the command that failed
 * @param reason   a short word, no spaces, describing what went wrong
 */
void protocol_reply_error(const char *command, const char *reason);

#endif /* PROTOCOL_H_ */
