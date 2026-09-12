#include "encoder.h"
#include "main.h"

/* The SPI handle CubeMX creates in main.c. */
extern SPI_HandleTypeDef hspi1;

/* Bit 14 of an outgoing frame. Set means read, clear means write. */
#define AS5147_FRAME_READ_BIT 0x4000U

/* Bit 15, the parity bit, in either direction. */
#define AS5147_FRAME_PARITY_BIT 0x8000U

/* Bit 14 of an incoming frame. Set means the sensor rejected something. */
#define AS5147_FRAME_ERROR_BIT 0x4000U

/* Bits 13:0, the payload in either direction. */
#define AS5147_FRAME_DATA_MASK 0x3FFFU

/* A single 16-bit exchange takes under 3 microseconds at 6 MHz. This
 * bound exists only so a dead SPI clock cannot stall the caller forever.
 *
 * Kept to one millisecond because this is now also called from the
 * control interrupt when encoder telemetry is enabled. A timeout there
 * costs 32 missed control periods, which is bad but recoverable; the
 * two milliseconds it used to be would have cost 64. If a read ever does
 * time out the reading is unusable anyway, so a shorter wait loses
 * nothing. */
#define ENCODER_SPI_TIMEOUT_MS 1U

/* ------------------------------------------------------------------
 * Parity
 * ------------------------------------------------------------------ */

/* Set bit 15 so the complete 16-bit frame contains an even number of
 * ones.
 *
 * "Even parity" means the transmitter arranges for the total count of
 * set bits to be even. The receiver counts them, and an odd total proves
 * that at least one bit flipped in transit. It catches any single-bit
 * error, which is what a noisy SPI line usually produces.
 *
 * @param frame  the 15 payload bits, with bit 15 clear
 * @return the frame with bit 15 set or cleared as required */
static uint16_t apply_even_parity(uint16_t frame)
{
    /* Folded rather than counted.
     *
     * Each step exclusive-ors the value with itself shifted, combining
     * pairs of bits: after the first, every bit holds the parity of
     * itself and the one eight places up; after the second, of four
     * bits; and so on, until bit 0 holds the parity of all fifteen.
     *
     * The loop this replaces did the same job in fifteen iterations of
     * shift, mask, add and compare -- around two microseconds in an
     * unoptimised build, inside an interrupt with thirty-one to spend in
     * total.
     *
     * Only the low fifteen bits take part: bit 15 is the one being
     * decided, so it must not count towards its own value. */
    uint16_t folded = frame & 0x7FFFu;

    folded ^= (uint16_t)(folded >> 8);
    folded ^= (uint16_t)(folded >> 4);
    folded ^= (uint16_t)(folded >> 2);
    folded ^= (uint16_t)(folded >> 1);

    if ((folded & 1u) != 0u) {
        frame |= AS5147_FRAME_PARITY_BIT;
    }
    return frame;
}

/* Check that a received frame has even parity across all 16 bits.
 *
 * @param frame  the frame exactly as received
 * @return 1 if parity is correct, 0 if the frame is corrupt */
static uint8_t check_even_parity(uint16_t frame)
{
    /* The same fold as above, over all sixteen bits this time, since
     * here the parity bit is part of what is being checked. */
    frame ^= (uint16_t)(frame >> 8);
    frame ^= (uint16_t)(frame >> 4);
    frame ^= (uint16_t)(frame >> 2);
    frame ^= (uint16_t)(frame >> 1);

    return ((frame & 1u) == 0u) ? 1u : 0u;
}

/* ------------------------------------------------------------------
 * One SPI frame
 * ------------------------------------------------------------------ */

/* Drive chip select low, exchange 16 bits, drive it high again.
 *
 * SPI is inherently a swap: the same clock edges that shift a bit out
 * shift a bit in. There is no "send only" or "receive only" -- every
 * frame you send returns a frame, which is exactly why the sensor's
 * answer arrives during your next command.
 *
 * @param frame_to_send      the 16 bits to clock out
 * @param frame_received_out where to store the 16 bits clocked in
 * @return 1 on success, 0 if the transfer failed or timed out */
static uint8_t exchange_frame(uint16_t frame_to_send,
                              uint16_t *frame_received_out)
{
    uint16_t received = 0u;
    HAL_StatusTypeDef status;

    /* Chip select is active low: pulling it down tells the sensor that
     * the following clock edges are addressed to it, and releasing it
     * marks the end of the frame. */
    HAL_GPIO_WritePin(AS5147_CS_GPIO_Port, AS5147_CS_Pin, GPIO_PIN_RESET);

    /* The arguments to HAL_SPI_TransmitReceive:
     *
     *   1. the peripheral handle
     *   2. pointer to the data to send
     *   3. pointer to where received data goes
     *   4. SIZE -- and this is the one that catches people. Size counts
     *      DATA FRAMES, not bytes. SPI1 is configured for 16-bit frames,
     *      so 1 means one 16-bit word. Passing 2 here would try to send
     *      two words and read past the end of these variables.
     *   5. timeout in milliseconds
     *
     * The casts to uint8_t* are what HAL's prototype demands; the
     * peripheral still moves 16 bits at a time because that is how the
     * data size was configured. */
    status = HAL_SPI_TransmitReceive(&hspi1,
                                     (uint8_t *)&frame_to_send,
                                     (uint8_t *)&received,
                                     1u,
                                     ENCODER_SPI_TIMEOUT_MS);

    HAL_GPIO_WritePin(AS5147_CS_GPIO_Port, AS5147_CS_Pin, GPIO_PIN_SET);

    /* The sensor requires chip select to stay high for at least 350
     * nanoseconds between frames. At 96 MHz that is about 34 clock
     * cycles. Ordinary function call overhead usually covers it, but a
     * few explicit no-ops make the guarantee independent of how the
     * compiler decides to optimise the surrounding code. */
    for (uint8_t i = 0u; i < 40u; i++) {
        __NOP();
    }

    if (status != HAL_OK) {
        return 0u;
    }

    *frame_received_out = received;
    return 1u;
}

/* ------------------------------------------------------------------
 * Register access
 * ------------------------------------------------------------------ */

void encoder_init(encoder_t* e)
{
    uint16_t discarded_errors;

    /* Chip select idles high. MX_GPIO_Init leaves outputs low unless the
     * initial state was set in CubeMX, so make it explicit here rather
     * than depending on that setting being right. */
    HAL_GPIO_WritePin(AS5147_CS_GPIO_Port, AS5147_CS_Pin, GPIO_PIN_SET);

    /* Clear anything the sensor latched during power-up, so the first
     * real read is not reported as a failure. */
    (void)encoder_read_and_clear_errors(&discarded_errors);

    e->raw_count         = 0u;
    e->offset_counts     = 0u;
    e->direction_forward = 1u;
    e->mechanical_angle  = 0u;
    e->encoder_diag.automatic_gain    = 0u;
    e->encoder_diag.field_magnitude   = 0u;
    e->encoder_diag.magnet_too_weak   = 0u;
    e->encoder_diag.magnet_too_strong = 0u;
    e->encoder_diag.cordic_overflow   = 0u;
    e->encoder_diag.offset_ready      = 0u;
}

uint8_t encoder_get_direction(encoder_t* e){
	return e->direction_forward;
}
uint16_t encoder_get_offset(encoder_t* e){
	return e->offset_counts;
}
uint16_t encoder_get_mangle(encoder_t* e){
	return e->mechanical_angle;
}
uint16_t encoder_get_raw_count(encoder_t* e){
	return e->raw_count;
}

void encoder_set_direction(encoder_t* e, uint8_t direction){
	e->direction_forward = direction;
}
void encoder_set_offset(encoder_t* e, uint16_t offset){
	/* Any value describes some angle, so out-of-range wraps rather than
	 * being rejected. */
	e->offset_counts = offset & (ENCODER_COUNTS_PER_REVOLUTION - 1u);
}

uint8_t encoder_read_register(uint16_t register_address,
                              uint16_t *value_out)
{
    uint16_t command_frame;
    uint16_t nop_frame;
    uint16_t first_reply;
    uint16_t second_reply;

    /* Build the request: the address in bits 13:0, the read bit at 14,
     * then parity on top of both. */
    command_frame = (register_address & AS5147_FRAME_DATA_MASK)
                  | AS5147_FRAME_READ_BIT;
    command_frame = apply_even_parity(command_frame);

    nop_frame = AS5147_REGISTER_NOP | AS5147_FRAME_READ_BIT;
    nop_frame = apply_even_parity(nop_frame);

    /* First cycle posts the request. Whatever comes back belongs to some
     * earlier command, so it is discarded. */
    if (exchange_frame(command_frame, &first_reply) == 0u) {
        return 0u;
    }

    /* Second cycle carries a NOP purely to generate the clock edges that
     * shift out the answer to the first. */
    if (exchange_frame(nop_frame, &second_reply) == 0u) {
        return 0u;
    }

    /* A corrupt frame is worse than no frame, because the value would
     * still look like a plausible angle. */
    if (check_even_parity(second_reply) == 0u) {
        return 0u;
    }

    /* Bit 14 set means the sensor is complaining about something it
     * received. The payload is not trustworthy. */
    if ((second_reply & AS5147_FRAME_ERROR_BIT) != 0u) {
        return 0u;
    }

    *value_out = second_reply & AS5147_FRAME_DATA_MASK;
    return 1u;
}

/* How many times to spin waiting for an SPI flag before giving up.
 *
 * A 16-bit frame at 6 MHz takes about 2.7 microseconds, which is 256 CPU
 * cycles at 96 MHz. Each pass of the wait loop is several instructions,
 * so a few hundred passes already covers a healthy transfer many times
 * over. Two thousand is generous and still bounded.
 *
 * A count is used rather than a millisecond timeout because this runs
 * inside the control interrupt, and HAL_GetTick() cannot advance there:
 * SysTick sits below the control interrupt in priority, so the tick is
 * frozen for as long as the interrupt runs. A tick-based timeout would
 * therefore never expire, turning any stall into a permanent hang. */
#define SPI_SPIN_LIMIT 2000u

/* Chip select, driven by writing the port's set/reset register directly.
 *
 * That register exists precisely for this: writing a bit in its lower
 * half sets the pin, writing one in its upper half clears it, and either
 * is a single store with no read-modify-write and no chance of an
 * interrupt landing in the middle.
 *
 * The driver function does the same thing, but as a call with argument
 * checking around it. In an unoptimised build that is a few hundred
 * nanoseconds each way, which matters when the whole exchange is
 * supposed to take under four microseconds. */
#define ENCODER_CS_LOW() \
    do { AS5147_CS_GPIO_Port->BSRR = (uint32_t)AS5147_CS_Pin << 16; } while (0)

#define ENCODER_CS_HIGH() \
    do { AS5147_CS_GPIO_Port->BSRR = (uint32_t)AS5147_CS_Pin; } while (0)

/* Exchange one 16-bit frame by talking to the peripheral directly.
 *
 * The driver-layer equivalent does the same work, but with a lock, a
 * state machine, several function calls and a tick-based timeout around
 * it. In an unoptimised build that overhead can be ten times the
 * transfer itself -- enough to push the control interrupt past its own
 * period, at which point it never finishes before the next one starts
 * and nothing else on the processor runs again.
 *
 * At register level the whole exchange is a handful of instructions plus
 * the transfer time, and it depends on nothing that an interrupt might
 * be holding.
 *
 * @param frame_to_send       the 16 bits to clock out
 * @param frame_received_out  where the 16 bits clocked in are stored
 * @return 1 on success, 0 if a flag never appeared */
static uint8_t exchange_frame_direct(uint16_t  frame_to_send,
                                     uint16_t *frame_received_out)
{
    SPI_TypeDef *spi = hspi1.Instance;
    uint32_t     timeout_ticks;

    ENCODER_CS_LOW();

    /* Wait for the transmit register to be free, then load the frame.
     * SPI1 is configured for 16-bit data, so one write sends all 16.
     * SR_TXE = tx buffer empty */
    timeout_ticks = 0u;
    while ((spi->SR & SPI_SR_TXE) == 0u) {
        if (++timeout_ticks > SPI_SPIN_LIMIT) {
            ENCODER_CS_HIGH();
            return 0u;
        }
    }
    spi->DR = frame_to_send;

    /* Every bit shifted out shifts one in, so the reply is complete at
     * the same moment the transmission is.
     * SR_RXNE = rx buffer not empty*/
    timeout_ticks = 0u;
    while ((spi->SR & SPI_SR_RXNE) == 0u) {
        if (++timeout_ticks > SPI_SPIN_LIMIT) {
            ENCODER_CS_HIGH();
            return 0u;
        }
    }
    /* DR = data register, holds transmitted and received data*/
    *frame_received_out = (uint16_t)spi->DR;

    /* No busy-flag wait, and no delay after raising chip select.
     *
     * The receive flag only sets once the final bit has been shifted in,
     * so by the time it is read the frame is complete and chip select
     * can be released immediately.
     *
     * The sensor does require chip select to stay high for 350
     * nanoseconds before the next frame begins. That matters for the
     * two-frame read, where one frame follows another straight away, and
     * that path still pads. Here the next frame is a whole control
     * period away -- 31 microseconds, or ninety times the requirement --
     * so padding would only be waiting for something that has already
     * happened. */
    ENCODER_CS_HIGH();

    return 1u;
}

/* The read command sent every pipelined call, built once.
 *
 * Zero means it has not been built yet; a real command always has its
 * read bit set, so zero is unambiguous as "not ready". */
static uint16_t pipelined_command;

/* Build the ANGLECOM read command if it has not been built already. */
static void ensure_pipelined_command(void)
{
    if (pipelined_command == 0u) {
        uint16_t frame = (AS5147_REGISTER_ANGLECOM & AS5147_FRAME_DATA_MASK)
                       | AS5147_FRAME_READ_BIT;
        pipelined_command = apply_even_parity(frame);
    }
}

void encoder_prime_pipeline(void)
{
    uint16_t discarded;

    ensure_pipelined_command();
    (void)exchange_frame_direct(pipelined_command, &discarded);
}

uint8_t encoder_read_angle_pipelined(uint16_t *angle_out)
{
    uint16_t reply;

    ensure_pipelined_command();

    /* One exchange does both jobs: it carries the request for the next
     * angle out, and brings the answer to the previous request back.
     *
     * The direct register path is used rather than the driver one,
     * because this is called from the control interrupt where the
     * driver's overhead does not fit in the period available. */
    if (exchange_frame_direct(pipelined_command, &reply) == 0u) {
        return 0u;
    }
    if (check_even_parity(reply) == 0u) {
        return 0u;
    }
    if ((reply & AS5147_FRAME_ERROR_BIT) != 0u) {
        return 0u;
    }

    *angle_out = reply & AS5147_FRAME_DATA_MASK;
    return 1u;
}

/* Turn a raw sensor reading into a zero-referenced, direction-corrected
 * mechanical angle.
 *
 * Masking rather than a conditional keeps the subtraction correct when
 * it would otherwise go negative: the encoder scale is a power of two, so
 * the mask performs the modulo.
 *
 * @param e    the encoder, for its offset and direction
 * @param raw  the sensor's own reading, 0..16383
 * @return the zero-referenced angle, 0..16383 */
static uint16_t compute_mechanical_angle(const encoder_t *e, uint16_t raw)
{
    uint32_t from_zero = ((uint32_t)raw - (uint32_t)e->offset_counts)
                        & (ENCODER_COUNTS_PER_REVOLUTION - 1u);

    if (e->direction_forward == 0u) {
        /* Reflect about the origin. Subtracting from the full scale turns
         * increasing counts into decreasing angle, and the mask keeps the
         * result in range including at zero. */
        from_zero = ((uint32_t)ENCODER_COUNTS_PER_REVOLUTION - from_zero)
                    & (ENCODER_COUNTS_PER_REVOLUTION - 1u);
    }

    return (uint16_t)from_zero;
}

uint8_t encoder_capture(encoder_t *e)
{
    uint16_t raw;

    if (encoder_read_angle_pipelined(&raw) == 0u) {
        /* The previous angles are kept; see the header for why a failed
         * read must not be reported as a jump to zero. */
        return 0u;
    }

    e->raw_count        = raw;
    e->mechanical_angle = compute_mechanical_angle(e, raw);
    return 1u;
}

uint8_t encoder_read_angle(uint16_t *angle_out)
{
    return encoder_read_register(AS5147_REGISTER_ANGLECOM, angle_out);
}

uint8_t encoder_read_and_clear_errors(uint16_t *error_flags_out)
{
    return encoder_read_register(AS5147_REGISTER_ERRFL, error_flags_out);
}

uint8_t encoder_read_diagnostics(encoder_diagnostics_t *diagnostics_out)
{
    uint16_t diagnostics_register;
    uint16_t magnitude_register;

    if (encoder_read_register(AS5147_REGISTER_DIAAGC,
                              &diagnostics_register) == 0u) {
        return 0u;
    }
    if (encoder_read_register(AS5147_REGISTER_MAG,
                              &magnitude_register) == 0u) {
        return 0u;
    }

    diagnostics_out->automatic_gain =
        diagnostics_register & AS5147_DIAAGC_AGC_MASK;

    diagnostics_out->field_magnitude = magnitude_register;

    /* Each of these turns a bit buried in the register into a plain 0 or
     * 1, so callers never have to know the bit positions. */
    diagnostics_out->magnet_too_weak =
        ((diagnostics_register & AS5147_DIAAGC_MAGNET_TOO_WEAK) != 0u)
            ? 1u : 0u;

    diagnostics_out->magnet_too_strong =
        ((diagnostics_register & AS5147_DIAAGC_MAGNET_TOO_STRONG) != 0u)
            ? 1u : 0u;

    diagnostics_out->cordic_overflow =
        ((diagnostics_register & AS5147_DIAAGC_CORDIC_OVERFLOW) != 0u)
            ? 1u : 0u;

    diagnostics_out->offset_ready =
        ((diagnostics_register & AS5147_DIAAGC_OFFSET_READY) != 0u)
            ? 1u : 0u;

    return 1u;
}
