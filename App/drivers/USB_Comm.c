/*
 * USB_Comm.c
 *
 *  Created on: Aug 9, 2026
 *      Author: ramis
 */


#include "USB_Comm.h"

#include "main.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

/* The bytes themselves. Nothing outside this file may touch this array. */
static volatile uint8_t usb_rx_queue[RX_QUEUE_SIZE];

/* The FRONT of the queue: the position of the oldest byte still waiting.
 *
 * The main loop reads from here and then advances it.
 * The interrupt only ever reads this value, never writes it. */
static volatile uint32_t rx_queue_start_index;

/* The BACK of the queue: the position where the next arriving byte goes.
 *
 * The interrupt writes here and then advances it.
 * The main loop only ever reads this value, never writes it. */
static volatile uint32_t rx_queue_end_index;

/* Set when a byte had to be discarded because the queue was full. */
static volatile uint8_t rx_queue_overflow_flag;

/* Why every one of the above is 'volatile':
 *
 * The compiler cannot see that an interrupt exists. If the main loop
 * reads usb_receive_queue_end_index inside a while() condition, an
 * optimising compiler will notice that nothing in the loop body changes
 * it, load it into a register once, and spin forever on the stale value.
 * 'volatile' forbids that -- it forces a fresh load from memory at every
 * single access.
 *
 * Why no interrupt disabling is needed:
 *
 * There is exactly one writer for each index. The interrupt is the only
 * thing that writes end_index; the main loop is the only thing that
 * writes start_index. Each side only READS the other's index. A 32-bit
 * aligned load or store on a Cortex-M4 cannot be split in half by an
 * interrupt, so each side always sees either the old value or the new
 * value of the other's index -- never a corrupted half-written one.
 *
 * Reading a slightly stale value is harmless. If the main loop reads an
 * end_index that the interrupt has since advanced, the main loop simply
 * thinks the queue is a byte shorter than it really is, and picks up the
 * remaining byte on its next pass. Nothing is lost.
 *
 * This property is why the "one caller each" rule in the header is not a
 * style preference. Add a second caller of either function and the
 * reasoning above collapses. */

static uint8_t usb_tx_buffer[TX_BUFFER_SIZE];

/* ------------------------------------------------------------------
 * Wrap-around
 * ------------------------------------------------------------------ */

/* Advance an index by one, wrapping back to 0 at the end of the array.
 *
 * Because the capacity is a power of two, subtracting 1 from it gives a
 * mask of all-ones in the low bits: 512 - 1 = 0x1FF. ANDing with that
 * mask keeps the index in the range 0..511 and costs one instruction,
 * whereas a modulo would call a division routine. */
static inline uint32_t advance_index(uint32_t index)
{
    return (index + 1u) & (RX_QUEUE_SIZE - 1u);
}

/* ------------------------------------------------------------------
 * Producer side -- interrupt context
 * ------------------------------------------------------------------ */

void usb_rx_enqueue(const uint8_t *bytes, uint32_t count)
{
    for (uint32_t i = 0u; i < count; i++) {

        uint32_t proposed_end_index =
            advance_index(rx_queue_end_index);

        /* If advancing the back of the queue would land it exactly on the
         * front, the queue is full.
         *
         * This is why one slot is always left empty. If we allowed the
         * back to catch up with the front, then start_index ==
         * end_index would mean "completely full" -- but it already means
         * "completely empty", and there would be no way to tell the two
         * apart. Sacrificing one slot keeps the two states distinct. */
        if (proposed_end_index == rx_queue_start_index) {
            rx_queue_overflow_flag = 1u;
            return;
        }

        usb_rx_queue[rx_queue_end_index] = bytes[i];

        /* The store above must be visible before the index moves. If the
         * index moved first, the main loop could read the slot before the
         * byte landed in it. Writing the data first and the index second
         * is the correct order, and on a Cortex-M4 with volatile accesses
         * the compiler will not reorder them. */
        rx_queue_end_index = proposed_end_index;
    }
}

uint8_t usb_rx_dequeue(uint8_t *byte)
{
	  /* Front and back in the same place means there is nothing waiting. */
	    if (rx_queue_start_index == rx_queue_end_index) {
	        return 0;
	    }

	    *byte = usb_rx_queue[rx_queue_start_index];

	    /* Free the slot only after the byte has been copied out, for the
	     * mirror-image reason given in the enqueue function. */
	    rx_queue_start_index =
	        advance_index(rx_queue_start_index);

	    return 1;
}

uint8_t usb_rx_is_overflow(void)
{
    uint8_t overflow_happened = rx_queue_overflow_flag;
    rx_queue_overflow_flag = 0u;
    return overflow_happened;
}

/* ------------------------------------------------------------------
 * Transmit
 * ------------------------------------------------------------------ */

static uint8_t usb_tx_is_idle(void)
{
    USBD_CDC_HandleTypeDef *cdc_state =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    if (cdc_state == NULL) {
        return 0u;
    }
    return (cdc_state->TxState == 0u) ? 1u : 0u;
}


uint8_t usb_tx_bytes(const void *bytes, uint16_t count)
{
    if (count == 0u) {
        return 1u;
    }
    if (count > TX_BUFFER_SIZE) {
        count = TX_BUFFER_SIZE;
    }

    /* Wait until the previous transfer has released the buffer.
     *
     * Note the subtraction: HAL_GetTick() wraps to zero after about 49
     * days. Writing (now - start) > limit rather than now > (start +
     * limit) keeps the comparison correct across that wrap, because the
     * unsigned subtraction still yields the true elapsed time. */
    uint32_t wait_start_tick = HAL_GetTick();

    while (usb_tx_is_idle() == 0u) {
        if ((HAL_GetTick() - wait_start_tick)
                > TX_TIMEOUT_MS) {
            return 0u;
        }
    }

    /* The buffer is ours again. Copy first, then hand it over. */
    memcpy(usb_tx_buffer, bytes, count);

    return (CDC_Transmit_FS(usb_tx_buffer, count) == USBD_OK)
             ? 1u : 0u;
}

uint8_t usb_tx_is_ready(void)
{
    return usb_tx_is_idle();
}

uint8_t usb_tx_string(const char *text)
{
    return usb_tx_bytes(text, (uint16_t)strlen(text));
}

/*
uint8_t usb_tx_formatted(const char *format, ...)
{
     This local array is safe, unlike passing one to CDC_Transmit_FS
     * directly, because usb_transmit_bytes() copies out of it before
     * returning. Nothing outside this function ever holds a pointer to
     * it.
    char formatted_text[TX_BUFFER_SIZE];
    va_list arguments;

    va_start(arguments, format);
    int formatted_length = vsnprintf(formatted_text,
                                     sizeof formatted_text,
                                     format, arguments);
    va_end(arguments);

    if (formatted_length <= 0) {
        return 0u;
    }

     vsnprintf returns the length it WOULD have produced, which can be
     * larger than the buffer. Clamp it so we never send past the end.
    if ((uint32_t)formatted_length >= sizeof formatted_text) {
        formatted_length = (int)sizeof formatted_text - 1;
    }

    return usb_transmit_bytes(formatted_text, (uint16_t)formatted_length);
}
*/
