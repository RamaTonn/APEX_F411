#include "crc16.h"

/* The generator polynomial, 0x1021, written without its top bit.
 *
 * In full the polynomial is x^16 + x^12 + x^5 + 1. The x^16 term is
 * implicit in a 16-bit register -- it is the bit that shifts out -- so
 * only the lower 16 bits are written. */
#define CRC16_POLYNOMIAL 0x1021u

uint16_t crc16_update(uint16_t       starting_value,
                      const uint8_t *data,
                      size_t         length)
{
    uint16_t crc = starting_value;

    if ((data == NULL) || (length == 0u)) {
        return crc;
    }

    for (size_t i = 0u; i < length; i++) {

        /* Bring the next byte in at the top of the register. This
         * variant does not reflect its input, so the most significant
         * bit of the byte is processed first and the byte aligns with
         * the high half of the register. */
        crc ^= (uint16_t)((uint16_t)data[i] << 8);

        for (uint8_t bit = 0u; bit < 8u; bit++) {

            /* Shift left, and if the bit that fell off the top was set,
             * subtract the polynomial. In this arithmetic subtraction is
             * exclusive-or, which is why the whole operation reduces to
             * a shift and a conditional xor. */
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ CRC16_POLYNOMIAL);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

uint16_t crc16_compute(const uint8_t *data, size_t length)
{
    /* No final exclusive-or and no output reflection in this variant, so
     * the running value is the result as it stands. */
    return crc16_update(CRC16_INITIAL_VALUE, data, length);
}
