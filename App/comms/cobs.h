/*
 * cobs.h
 *
 * Consistent Overhead Byte Stuffing: a way to remove every zero byte
 * from a block of data, so that a zero byte can be used as an
 * unambiguous frame delimiter.
 *
 * THE PROBLEM IT SOLVES
 *
 *   A binary packet can contain any byte value, including whatever byte
 *   was chosen as the end-of-frame marker. A receiver scanning for that
 *   marker will find it inside the payload, cut the frame in the wrong
 *   place, and stay out of step until it happens to resynchronise by
 *   luck.
 *
 *   Choosing a longer marker only makes the collision rarer, not
 *   impossible -- which arguably makes it worse, because a fault that
 *   appears once a week is far harder to find than one that appears
 *   every minute.
 *
 *   COBS removes the possibility entirely. After encoding, the data
 *   provably contains no zero bytes, so a zero byte can only ever mean
 *   end of frame.
 *
 * HOW IT WORKS
 *
 *   Every zero byte in the input is replaced by a count of how far it is
 *   to the next zero. The counts form a chain: the first byte of the
 *   encoded output says how many bytes until the next link, that link
 *   says how far to the one after, and so on to the end.
 *
 *   Since a count of zero would be meaningless -- it would point at
 *   itself -- no count is ever zero, and no data byte is ever zero
 *   because zeros are exactly what got replaced.
 *
 *   A run of more than 254 non-zero bytes has no zero to point at, so
 *   the chain inserts a link at 255 and carries on. That is where the
 *   one byte of overhead per 254 comes from.
 *
 * COST
 *
 *   One byte always, plus one more per 254 bytes of data. For the packet
 *   sizes used here that is one or two bytes -- negligible against the
 *   guarantee it buys.
 *
 * SELF-SYNCHRONISATION
 *
 *   Because a zero byte cannot appear inside a frame, a receiver that
 *   loses its place recovers at the very next delimiter. It never needs
 *   to guess, and a corrupted frame costs exactly one frame.
 */

#ifndef COBS_H_
#define COBS_H_

#include <stdint.h>
#include <stddef.h>

/* Worst-case encoded size for a given amount of input.
 *
 * One overhead byte to start, plus one more for every 254 bytes that
 * contain no zero. Use this to size an output buffer so that encoding
 * can never run out of room.
 *
 * Note this does NOT include the trailing delimiter byte; add one more
 * if the buffer must hold that too. */
#define COBS_ENCODED_MAX(input_length) \
    ((input_length) + (((input_length) + 253u) / 254u) + 1u)

/**
 * Encode a block so that it contains no zero bytes.
 *
 * Does not append the trailing zero delimiter -- the caller does that,
 * because the caller owns the framing.
 *
 * @param input          data to encode; may contain any byte values
 * @param input_length   how many bytes to encode; zero is valid and
 *                       produces a single output byte
 * @param output         where to write the encoded data. Must have room
 *                       for at least COBS_ENCODED_MAX(input_length)
 *                       bytes. Must not overlap input.
 * @param output_capacity  size of the output buffer in bytes
 * @return number of bytes written, or 0 if the output buffer was too
 *         small. Zero is unambiguous as a failure, because a successful
 *         encode always writes at least one byte.
 */
size_t cobs_encode(const uint8_t *input,
                   size_t         input_length,
                   uint8_t       *output,
                   size_t         output_capacity);

/**
 * Reverse the encoding, restoring the original bytes.
 *
 * @param input           encoded data, WITHOUT the trailing delimiter
 * @param input_length    how many encoded bytes there are
 * @param output          where to write the decoded data. Must have room
 *                        for at least input_length bytes, which is always
 *                        enough since decoding only ever shrinks. Must
 *                        not overlap input.
 * @param output_capacity size of the output buffer in bytes
 * @return number of bytes written, or 0 if the input was malformed or
 *         the output buffer was too small.
 *
 *         Malformed means a length byte that points beyond the end of
 *         the input, or a zero byte inside the data -- neither can occur
 *         in a correctly encoded block, so both indicate corruption. The
 *         CRC would catch most such damage anyway, but rejecting it here
 *         means the CRC is never computed over nonsense.
 */
size_t cobs_decode(const uint8_t *input,
                   size_t         input_length,
                   uint8_t       *output,
                   size_t         output_capacity);

#endif /* COBS_H_ */
