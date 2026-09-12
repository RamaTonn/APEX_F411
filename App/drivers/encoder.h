/*
 * encoder.h
 *
 * Driver for the AS5147 magnetic rotary position sensor (U8).
 *
 * WIRING ON THIS BOARD
 *
 *   SPI1 on PB3 (SCK), PB4 (MISO), PB5 (MOSI), with chip select on PA8
 *   driven as a plain GPIO rather than by the SPI peripheral. The
 *   sensor's incremental outputs (A, B, I) and its PWM output are not
 *   connected -- SPI is the only way to talk to it.
 *
 *   It runs from 3V3 with VDD5V and VDD3V tied together, which is the
 *   part's 3.3 volt mode.
 *
 * THE FRAME FORMAT
 *
 *   Every exchange is exactly 16 bits, most significant bit first. The
 *   same bit layout is used in both directions:
 *
 *     bit 15     parity, chosen so that the whole frame contains an
 *                even number of set bits
 *     bit 14     going out: 1 means read, 0 means write
 *                coming back: 1 means the sensor flagged an error
 *     bits 13:0  going out: the register address
 *                coming back: the register contents
 *
 * THE ONE FRAME DELAY
 *
 *   The sensor does not answer immediately. A command sent in one frame
 *   is answered in the NEXT frame. So a single register read costs two
 *   chip-select cycles:
 *
 *     frame 1:  send "read ANGLECOM"     receive: stale, ignore it
 *     frame 2:  send NOP                 receive: the ANGLECOM value
 *
 *   The second frame does not have to be a NOP. Sending the next real
 *   command there gives one useful answer per frame instead of one per
 *   two. This driver uses the simple two-frame form because at command
 *   rate the cost is irrelevant, and correctness is easier to see.
 *
 * REGISTER MAP (only the ones this driver uses)
 *
 *   0x0000  NOP       nothing; a filler frame to clock out an answer
 *   0x0001  ERRFL     error flags, cleared by reading them
 *   0x3FFC  DIAAGC    diagnostics and automatic gain control
 *   0x3FFD  MAG       measured field magnitude
 *   0x3FFF  ANGLECOM  angle with dynamic error compensation
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

/* Register addresses. */
#define AS5147_REGISTER_NOP      0x0000U
#define AS5147_REGISTER_ERRFL    0x0001U
#define AS5147_REGISTER_DIAAGC   0x3FFCU
#define AS5147_REGISTER_MAG      0x3FFDU
#define AS5147_REGISTER_ANGLECOM 0x3FFFU

/* Counts in one full revolution. The angle is 14 bits, so a turn is
 * 0..16383 and one count is about 0.022 degrees. */
#define ENCODER_COUNTS_PER_REVOLUTION 16384U

/* Bit positions inside the DIAAGC register. */
#define AS5147_DIAAGC_MAGNET_TOO_WEAK   (1U << 11) /* MAGL */
#define AS5147_DIAAGC_MAGNET_TOO_STRONG (1U << 10) /* MAGH */
#define AS5147_DIAAGC_CORDIC_OVERFLOW   (1U << 9)  /* COF  */
#define AS5147_DIAAGC_OFFSET_READY      (1U << 8)  /* LF   */
#define AS5147_DIAAGC_AGC_MASK          0x00FFU

/* Everything the diagnostics register tells you, unpacked.
 *
 * automatic_gain  0..255. The sensor raises its gain when the field is
 *                 weak, so a HIGH number means the magnet is far away,
 *                 off centre, or weak, and a LOW number means it is
 *                 close or strong. Somewhere near the middle is healthy.
 *                 This is the number to watch while aligning a magnet.
 *
 * field_magnitude 0..16383, the raw strength the sensor measured.
 *
 * magnet_too_weak / magnet_too_strong
 *                 1 when the field is outside the usable range. Either
 *                 one means the angle should not be trusted.
 *
 * cordic_overflow 1 when the angle calculation overflowed. The angle is
 *                 meaningless when this is set.
 *
 * offset_ready    1 once internal offset compensation has finished.
 *                 Should be 1 shortly after power-up; a persistent 0
 *                 suggests the sensor never started properly.
 */
typedef struct {
    uint16_t automatic_gain;
    uint16_t field_magnitude;
    uint8_t  magnet_too_weak;
    uint8_t  magnet_too_strong;
    uint8_t  cordic_overflow;
    uint8_t  offset_ready;
} encoder_diagnostics_t;

/*
 * One encoder. Owned by main.c, wherever it is on the board, and handed
 * by pointer to whatever needs it -- currently just the motor it is
 * bolted to.
 *
 * mechanical_angle is the one field callers actually want: zero-referenced
 * against offset_counts and corrected for direction_forward, so it always
 * reads 0 at the rotor position electrical zero was calibrated against,
 * rising as the shaft turns forward. raw_count is what the sensor itself
 * reported, kept mainly for diagnostics and for commands.c to report back
 * during calibration.
 */
typedef struct {
    volatile uint16_t raw_count;        /* sensor's own reading, 0..16383 */
    uint16_t offset_counts;             /* raw_count at electrical zero   */
    uint8_t  direction_forward;         /* 1 if rising counts mean forward */
    volatile uint16_t mechanical_angle; /* zero-referenced, direction-corrected, 0..16383 */
    encoder_diagnostics_t encoder_diag;
} encoder_t;

/**
 * Set up the sensor and zero an encoder_t to a safe starting state:
 * no offset, forward direction, angles at zero.
 *
 * Clears whatever error flags the sensor latched during power-up, so the
 * first real read is not reported as a failure.
 *
 * @param e  the encoder instance this board's sensor feeds
 */
void encoder_init(encoder_t* e);

uint8_t encoder_get_direction(encoder_t* e);
uint16_t encoder_get_offset(encoder_t* e);
uint16_t encoder_get_mangle(encoder_t* e);
uint16_t encoder_get_raw_count(encoder_t* e);

void encoder_set_direction(encoder_t* e, uint8_t direction);
void encoder_set_offset(encoder_t* e, uint16_t offset);

/**
 * Refresh raw_count and mechanical_angle from the sensor.
 *
 * CALLED FROM THE CONTROL INTERRUPT ONLY, once per period -- it uses the
 * pipelined single-frame read, so the angle it produces is one period
 * old. See encoder_read_angle_pipelined() below for why that is the
 * right trade here.
 *
 * A failed read leaves both fields at their previous value rather than
 * zeroing them: a momentary zero would look like the rotor jumping to the
 * origin, and a loop acting on it would apply a large correction to
 * something that never happened.
 *
 * @param e  the encoder to refresh
 * @return 1 if the sensor answered and the angles were updated
 */
uint8_t encoder_capture(encoder_t* e);

/**
 * Read one register.
 *
 * Performs the two chip-select cycles described above: the first posts
 * the request, the second clocks out the answer.
 *
 * @param register_address  one of the AS5147_REGISTER_ constants. Only
 *                          the low 14 bits are used; the read bit and
 *                          the parity bit are added by this function.
 * @param value_out         where to store the 14-bit register contents.
 *                          The parity and error bits are stripped, so
 *                          this is always 0..16383.
 * @return 1 on success. 0 if the SPI transfer failed, if the sensor set
 *         its error flag, or if the reply's parity did not check out.
 */
uint8_t encoder_read_register(uint16_t register_address,
                              uint16_t *value_out);

/**
 * Read the angle one frame at a time, for use in the control loop.
 *
 * The ordinary read costs two chip-select cycles because the sensor
 * answers a command in the FOLLOWING frame. Sending the same read
 * command every time turns that delay into a pipeline: each frame
 * carries the request for the next angle and returns the answer to the
 * previous one. One frame per call instead of two, so roughly three and
 * a half microseconds rather than seven.
 *
 * The price is that the angle is one call old. Called once per control
 * period that is 31 microseconds, during which even a fast motor turns a
 * fraction of a degree -- far less than the lag already present in the
 * current measurement.
 *
 * The first call after a gap returns whatever the sensor had queued from
 * before, which is meaningless. Call encoder_prime_pipeline() to flush
 * that, or simply discard the first reading.
 *
 * @param angle_out  where to store the angle, 0..16383
 * @return 1 on success, 0 on a transfer, parity or sensor error
 */
uint8_t encoder_read_angle_pipelined(uint16_t *angle_out);

/**
 * Discard the pipeline's stale first answer.
 *
 * Sends one read command whose reply is thrown away, so that the next
 * call to encoder_read_angle_pipelined() returns a fresh angle rather
 * than whatever was left over.
 */
void encoder_prime_pipeline(void);

/**
 * Read the current shaft angle.
 *
 * @param angle_out  where to store the angle, 0..16383 for a full turn
 * @return 1 on success, 0 on any communication or parity failure
 */
uint8_t encoder_read_angle(uint16_t *angle_out);

/**
 * Read and unpack the diagnostics and field magnitude registers.
 *
 * @param diagnostics_out  where to store the unpacked results
 * @return 1 on success, 0 on any communication or parity failure
 */
uint8_t encoder_read_diagnostics(encoder_diagnostics_t *diagnostics_out);

/**
 * Read and clear the sensor's error flag register.
 *
 * Reading ERRFL clears it, so this both reports and resets. Worth doing
 * once at startup to clear anything left over from power-up.
 *
 * @param error_flags_out  where to store the raw ERRFL contents. Bit 0
 *                         is a parity error in a command the sensor
 *                         received, bit 1 an invalid command, bit 2 a
 *                         framing error.
 * @return 1 on success, 0 on any communication failure
 */
uint8_t encoder_read_and_clear_errors(uint16_t *error_flags_out);

#endif /* ENCODER_H_ */
