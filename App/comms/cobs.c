#include "cobs.h"

/* Longest run of non-zero bytes that a single length byte can describe.
 *
 * A length byte counts the bytes up to and including the next link in
 * the chain, so a value of 255 covers 254 data bytes. When a run reaches
 * that without meeting a zero, a link is inserted and the chain
 * continues -- which is the one byte of overhead per 254 bytes. */
#define COBS_MAXIMUM_RUN 254u

size_t cobs_encode(const uint8_t *input,
                   size_t         input_length,
                   uint8_t       *output,
                   size_t         output_capacity)
{
    /* Where the length byte for the run currently being built will go.
     * Its value is not known until the run ends, so the position is
     * reserved now and filled in later. */
    size_t length_byte_position = 0u;

    /* Where the next data byte goes. Starts one past the reserved length
     * byte. */
    size_t output_position = 1u;

    /* How many bytes the current run covers, counting the length byte
     * itself. Starts at 1 because the length byte is part of its own
     * count -- a run with no data bytes is written as 1. */
    uint8_t run_length = 1u;

    /* Whether a length byte has been reserved and still needs its value
     * written after the loop.
     *
     * This is normally true, but there is one case where it is not: when
     * the data ends exactly as a maximum-length run completes. That run
     * gets its length byte inside the loop, and no further run follows,
     * so reserving a byte for one would append a stray 0x01 that the
     * decoder would read as an empty trailing group. */
    uint8_t final_length_byte_pending = 1u;

    if ((input == NULL) || (output == NULL)) {
        return 0u;
    }

    /* One byte is always written even for empty input, so the buffer
     * must have room for at least that. */
    if (output_capacity < 1u) {
        return 0u;
    }

    for (size_t i = 0u; i < input_length; i++) {

        if (input[i] == 0u) {
            /* A zero ends the current run. The length byte is written
             * with the run's length, and the zero itself is not stored
             * at all -- its position is what the length encodes. */
            output[length_byte_position] = run_length;

            if (output_position >= output_capacity) {
                return 0u;
            }
            length_byte_position = output_position;
            output_position++;
            run_length = 1u;

        } else {
            /* A non-zero byte is copied through unchanged. */
            if (output_position >= output_capacity) {
                return 0u;
            }
            output[output_position] = input[i];
            output_position++;
            run_length++;

            /* A run that reaches the maximum has no zero to point at, so
             * a link is inserted to keep the chain going. */
            if (run_length == (COBS_MAXIMUM_RUN + 1u)) {
                output[length_byte_position] = run_length;
                run_length = 1u;

                /* Only reserve a byte for the next run if there is more
                 * input to put in it. Without this check, data whose
                 * length is an exact multiple of the maximum run gains a
                 * trailing 0x01 describing a group that does not
                 * exist. */
                if ((i + 1u) < input_length) {
                    if (output_position >= output_capacity) {
                        return 0u;
                    }
                    length_byte_position = output_position;
                    output_position++;
                    final_length_byte_pending = 1u;
                } else {
                    final_length_byte_pending = 0u;
                }
            }
        }
    }

    /* Close the final run, unless the loop already closed it and
     * deliberately declined to open another. */
    if (final_length_byte_pending != 0u) {
        output[length_byte_position] = run_length;
    }

    return output_position;
}

size_t cobs_decode(const uint8_t *input,
                   size_t         input_length,
                   uint8_t       *output,
                   size_t         output_capacity)
{
    size_t input_position  = 0u;
    size_t output_position = 0u;

    if ((input == NULL) || (output == NULL)) {
        return 0u;
    }
    if (input_length == 0u) {
        return 0u;
    }

    while (input_position < input_length) {

        uint8_t run_length = input[input_position];

        /* A length byte of zero cannot occur in valid encoded data: it
         * would point at itself. Its presence means the data is
         * corrupt, or that a delimiter was mistakenly included in the
         * input. */
        if (run_length == 0u) {
            return 0u;
        }

        input_position++;

        /* Copy the run's data bytes. run_length counts the length byte
         * itself, so there are run_length - 1 data bytes. */
        for (uint8_t i = 1u; i < run_length; i++) {

            /* A run pointing past the end of the input is corruption. */
            if (input_position >= input_length) {
                return 0u;
            }
            /* A zero inside the data cannot occur in valid encoded
             * data either -- zeros are exactly what encoding removed. */
            if (input[input_position] == 0u) {
                return 0u;
            }
            if (output_position >= output_capacity) {
                return 0u;
            }

            output[output_position] = input[input_position];
            output_position++;
            input_position++;
        }

        /* A run shorter than the maximum ended because the original data
         * had a zero there, so that zero is restored.
         *
         * The exception is a run that ends exactly at the end of the
         * input: there is no following zero to restore, because encoding
         * closed the final run without one. */
        if ((run_length < (COBS_MAXIMUM_RUN + 1u))
                && (input_position < input_length)) {

            if (output_position >= output_capacity) {
                return 0u;
            }
            output[output_position] = 0u;
            output_position++;
        }
    }

    return output_position;
}
