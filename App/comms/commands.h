/*
 * commands.h
 *
 * The command handlers, and the table that maps names and numbers to
 * them.
 *
 * ONE SET OF HANDLERS, TWO WIRE FORMATS
 *
 *   Nothing here knows whether it was reached from a typed line or a
 *   binary packet. Arguments arrive through protocol_arg_* and replies
 *   go out through protocol_reply_*, both of which adapt to whichever
 *   format is active.
 *
 *   Two copies would drift: a check added to one, a limit changed in the
 *   other, and a bug that appears in one mode and not the other. Sharing
 *   them makes that impossible rather than merely unlikely.
 *
 * NAMES AND NUMBERS
 *
 *   Every command has both. Text uses the name, binary uses the number,
 *   and one table holds the pairing so the two can never disagree.
 *
 *   Numbers are assigned explicitly rather than taken from a command's
 *   position in the table, so that inserting or reordering commands does
 *   not renumber the ones after it. A host built against an older
 *   firmware keeps working.
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <stdint.h>
#include "motor.h"
#include "protocol.h"

/**
 * Run the handler for a named command.
 *
 * Replies with an error if the name is not known, so every command that
 * arrives produces exactly one reply -- a host can always match a
 * response to its request rather than waiting on a timeout.
 *
 * @param name  the command name
 * @param args  the argument block, already filled in by the backend
 */
/**
 * Give the command layer the motor its handlers report on and configure.
 *
 * @param m  the motor owned by main.c
 */
void commands_init(motor_t *m);

void commands_dispatch(const char *name, const protocol_args_t *args);

/**
 * Look up the number assigned to a command name.
 *
 * @param name  the command name
 * @return the number, or 0 if the name is not a command
 */
uint8_t commands_id_for(const char *name);

/**
 * Look up the name of a numbered command.
 *
 * @param id  the command number
 * @return the name, or NULL if the number is not assigned
 */
const char *commands_name_for(uint8_t id);

/**
 * @return how many commands exist
 */
uint32_t commands_count(void);

/**
 * Name of the command at a position in the table, for building a list.
 *
 * @param index  0 to commands_count() - 1
 * @return the name, or NULL if the index is out of range
 */
const char *commands_name_at(uint32_t index);

#endif /* COMMANDS_H_ */
