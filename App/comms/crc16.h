/*
 * crc16.h
 *
 * CRC-16/CCITT-FALSE, used to detect corruption in received packets.
 *
 * THE EXACT VARIANT
 *
 *   "CRC16" on its own does not identify an algorithm -- there are dozens
 *   of incompatible variants under that name, and two implementations
 *   that disagree on any one parameter will reject every packet the other
 *   sends. The full specification of the one used here:
 *
 *     Width          16 bits
 *     Polynomial     0x1021
 *     Initial value  0xFFFF
 *     Input          not reflected
 *     Output         not reflected
 *     Final XOR      0x0000
 *     Check value    0x29B1 for the ASCII bytes "123456789"
 *
 *   That last line is the important one. It is the standard way to prove
 *   two implementations agree: any correct implementation of this variant
 *   returns 0x29B1 for that input. A PC-side implementation should be
 *   checked against it before anything else is debugged.
 *
 *   In common catalogues this variant is listed as CRC-16/IBM-3740, with
 *   CRC-16/CCITT-FALSE as a widely used alias.
 *
 * WHAT IT COVERS
 *
 *   Everything in the decoded packet except the CRC field itself:
 *   version, type, id, length and payload. It is computed before COBS
 *   encoding and checked after COBS decoding, so it protects the packet
 *   contents rather than the framing.
 *
 * WHY BOTH COBS AND A CRC
 *
 *   They catch different problems. COBS guarantees the receiver finds the
 *   right frame boundaries; it says nothing about whether the bytes
 *   inside are the ones that were sent. The CRC checks the contents but
 *   cannot help if the frame was cut in the wrong place. Framing and
 *   integrity are separate concerns and each needs its own mechanism.
 *
 * IMPLEMENTATION NOTE
 *
 *   Computed a bit at a time rather than with a 512-byte lookup table.
 *   Packets here are small and the CRC is computed outside the control
 *   interrupt, so the table's speed advantage buys nothing worth the
 *   flash it would occupy.
 */

#ifndef CRC16_H_
#define CRC16_H_

#include <stdint.h>
#include <stddef.h>

/* Value a correct implementation returns for the nine ASCII bytes
 * "123456789". Provided so that both sides of the link can verify they
 * agree before any real traffic is attempted. */
#define CRC16_CHECK_VALUE 0x29B1u

/* The value the running CRC starts from. Exposed because incremental
 * computation over several buffers needs somewhere to begin. */
#define CRC16_INITIAL_VALUE 0xFFFFu

/**
 * Compute the CRC of a block of data.
 *
 * @param data    bytes to compute over; may be NULL only if length is 0
 * @param length  how many bytes
 * @return the 16-bit CRC
 */
uint16_t crc16_compute(const uint8_t *data, size_t length);

/**
 * Continue a CRC across a further block of data.
 *
 * Lets a CRC be computed over several separate buffers without first
 * copying them together -- useful when a packet header and its payload
 * live in different places.
 *
 * @param starting_value  CRC16_INITIAL_VALUE for the first block, or the
 *                        value returned by the previous call
 * @param data            bytes to add; may be NULL only if length is 0
 * @param length          how many bytes
 * @return the updated CRC
 */
uint16_t crc16_update(uint16_t       starting_value,
                      const uint8_t *data,
                      size_t         length);

#endif /* CRC16_H_ */
