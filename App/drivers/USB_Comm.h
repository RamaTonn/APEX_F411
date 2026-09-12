/*
 * USB_Comm.h
 *
 *  Created on: Aug 9, 2026
 *      Author: ramis
 */

#ifndef USB_COMM_H_
#define USB_COMM_H_

/**
 *
 * Sending bytes to the host over USB CDC.
 *
 * Why this is not just a wrapper around CDC_Transmit_FS:
 *
 *   CDC_Transmit_FS() does not copy your data. It stores the POINTER you
 *   give it, tells the USB peripheral to start reading from that address,
 *   and returns immediately -- long before the bytes have actually gone
 *   out on the wire.
 *
 *   That means the memory you pointed at must stay alive and unchanged
 *   until the transfer finishes. Passing it a local array is a bug: the
 *   array's stack space gets reused by the next function call while the
 *   USB peripheral is still reading from it, and the host receives
 *   whatever happened to land there.
 *
 *   This module owns one buffer with a permanent lifetime, and enforces
 *   the rule that nothing writes into it while a transfer is in flight.
 *
 * Called from the main loop only. Not interrupt-safe.
 */

#include <stdint.h>


/* Capacity of the queue in bytes.
 *
 * This must be a power of two. The wrap-around arithmetic in the .c file
 * uses a bitwise AND instead of a modulo or a comparison, which is only
 * correct for powers of two.
 *
 * How to choose the number: it must hold everything that can arrive
 * between two consecutive calls to the main loop's poll function. A USB
 * full-speed bulk endpoint delivers at most 64 bytes per packet and can
 * deliver several packets per millisecond, so 512 gives a wide margin for
 * a main loop that runs far more often than once per millisecond.
 *
 * Note that one slot is always kept empty (see the .c file), so the real
 * usable capacity is 511 bytes, not 512. */
#define RX_QUEUE_SIZE 512U
/* Largest single message that can be sent in one call. Anything longer is
 * truncated. One console reply line fits comfortably in this. */
#define TX_BUFFER_SIZE 256U
#define TX_TIMEOUT_MS 20u
/**
 * Add bytes to the back of the queue.
 *
 * CALLED FROM THE USB INTERRUPT ONLY.
 *
 * If the queue does not have room for all of them, the ones that do not
 * fit are discarded and an overflow flag is raised. Discarding the newest
 * bytes rather than the oldest is deliberate: the oldest bytes are the
 * start of a command that the main loop may already be part-way through
 * assembling, and throwing those away would corrupt it.
 *
 * @param bytes  pointer to the bytes to copy in
 * @param count  how many bytes to copy
 */
void usb_rx_enqueue(const uint8_t *bytes, uint32_t count);

/**
 * Remove one byte from the front of the queue.
 *
 * CALLED FROM THE MAIN LOOP ONLY.
 *
 * @param byte_out  where to store the byte, if there is one
 * @return true if a byte was removed, false if the queue was empty
 */
uint8_t usb_rx_dequeue(uint8_t *byte);

/**
 * Report whether bytes have been discarded since the last time this was
 * asked, and clear the record.
 *
 * CALLED FROM THE MAIN LOOP ONLY.
 *
 * An overflow means a command was almost certainly mangled. The caller
 * should tell the user rather than silently acting on a partial command.
 *
 * @return true if at least one byte was discarded since the last call
 */
uint8_t usb_rx_is_overflow(void);

/**
 * Send a block of bytes to the host.
 *
 * Waits for any previous transfer to finish, copies the caller's data
 * into the module's own buffer, and starts a new transfer.
 *
 * @param bytes  data to send; may be a local variable, it is copied
 * @param count  how many bytes to send
 * @return 1 if the transfer was started, 0 on timeout or failure
 */

uint8_t usb_tx_bytes(const void *bytes, uint16_t count);

/**
 * Send a zero-terminated string. The terminator itself is not sent.
 *
 * @return 1 if the transfer was started, 0 on timeout or failure
 */
uint8_t usb_tx_string(const char *text);

/**
 * Whether a transfer can be started without waiting.
 *
 * usb_tx_bytes() waits up to TX_TIMEOUT_MS for a previous transfer to
 * finish. That is acceptable for a command reply, which happens rarely,
 * but not for streaming: a host that has stopped reading would hold up
 * the main loop on every sample, and the protection supervisor sits
 * behind it.
 *
 * Callers that would rather skip than wait check this first.
 *
 * @return 1 if a transfer would start immediately, 0 if USB is still
 *         busy or the port is not open
 */
uint8_t usb_tx_is_ready(void);

/**
 * Format a string and send it, in the manner of printf.
 *
 * Floating point conversions are deliberately not usable here: newlib's
 * float formatting is not linked in by default and needs far more stack
 * than is currently configured. Send scaled integers instead -- for
 * example millivolts rather than volts -- and let the host divide.
 *
 * @return 1 if the transfer was started, 0 on timeout or failure
 */
/*uint8_t usb_tx_formatted(const char *format, ...);*/

#endif /* USB_COMM_H_ */
