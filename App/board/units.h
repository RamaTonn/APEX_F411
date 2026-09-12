/*
 * units.h
 *
 * The units every value crossing the console uses, and the conversions
 * between them and the raw quantities inside the firmware.
 *
 * WHY THIS EXISTS
 *
 *   Raw counts are an implementation detail. An encoder reading of 8231
 *   means nothing without knowing the sensor is 14 bits; a duty of 500
 *   means nothing without knowing the scale is per-thousand. Anyone
 *   using this board -- or writing software against it -- should not
 *   have to learn either.
 *
 *   So the console speaks in physical units, and this header defines
 *   which ones and how they are represented.
 *
 * FIXED POINT, NOT FLOATING POINT
 *
 *   Every value is a whole number in a scaled unit: milliamps rather
 *   than amps, milliradians per second rather than radians per second.
 *
 *   Not because floating point is unavailable -- this part has a
 *   hardware unit for it -- but because a scaled integer has exactly one
 *   representation. There is no rounding surprise between the board and
 *   the host, no locale-dependent decimal separator, and no question
 *   about how many digits to print. The host divides by a thousand when
 *   it wants to show a person.
 *
 * THE UNITS
 *
 *   current           milliamps                     ia_ma
 *   voltage           millivolts                    vbus_mv
 *   resistance        milliohms                     mohm
 *   angle, mechanical milliradians, 0 to 6283       pos_mrad
 *   angle, electrical milliradians, 0 to 6283       eangle_mrad
 *   velocity          milliradians per second       vel_mrads
 *   temperature       millidegrees Celsius          temp_mdeg
 *   time              milliseconds                  ms
 *
 *   A full turn is 2*pi radians, so 6283 milliradians. Mechanical angle
 *   is the shaft's position; electrical angle repeats once per pole
 *   pair.
 *
 * WHY MILLIRADIANS RATHER THAN DEGREES
 *
 *   Radians are what the control mathematics uses. Every velocity,
 *   every rate, every transform works in them, so reporting anything
 *   else would mean converting twice and rounding twice. A thousandth of
 *   a radian is about a twentieth of a degree, finer than the encoder
 *   resolves.
 */

#ifndef UNITS_H_
#define UNITS_H_

#include <stdint.h>

/* Milliradians in one full turn: two pi, times a thousand, rounded. */
#define UNITS_MRAD_PER_TURN 6283

/* Encoder counts in one mechanical turn, from the AS5147's 14 bits. */
#define UNITS_COUNTS_PER_TURN 16384

/* The scale used for angles inside the control path: a 16-bit value
 * spanning one turn, so the type's own overflow performs the wrap. */
#define UNITS_ANGLE_SCALE 65536

/**
 * Encoder counts to milliradians.
 *
 * @param counts  0 to 16383
 * @return 0 to 6282
 */
static inline int32_t units_counts_to_mrad(uint16_t counts)
{
    /* Multiply before dividing so the fraction survives. The product
     * reaches at most 16383 times 6283, about 103 million, which fits
     * comfortably in a signed 32-bit value. */
    return ((int32_t)counts * UNITS_MRAD_PER_TURN) / UNITS_COUNTS_PER_TURN;
}

/**
 * Internal 16-bit angle to milliradians.
 *
 * @param angle  0 to 65535 for one turn
 * @return 0 to 6282
 */
static inline int32_t units_angle_to_mrad(uint16_t angle)
{
    return ((int32_t)angle * UNITS_MRAD_PER_TURN) / UNITS_ANGLE_SCALE;
}

/**
 * Milliradians to the internal 16-bit angle.
 *
 * @param mrad  0 to 6283
 * @return 0 to 65535
 */
static inline uint16_t units_mrad_to_angle(int32_t mrad)
{
    /* Reduced into one turn first, so that a caller passing several
     * turns' worth gets the equivalent angle rather than an overflow. */
    int32_t within_turn = mrad % UNITS_MRAD_PER_TURN;

    if (within_turn < 0) {
        within_turn += UNITS_MRAD_PER_TURN;
    }
    return (uint16_t)((within_turn * UNITS_ANGLE_SCALE)
                      / UNITS_MRAD_PER_TURN);
}

/**
 * Encoder counts per second to milliradians per second.
 *
 * @param counts_per_second  signed rate
 * @return velocity in milliradians per second
 */
static inline int32_t units_counts_rate_to_mrads(int32_t counts_per_second)
{
    /* Divided before multiplying would lose everything below 2.6 counts
     * per second, so the multiply comes first. At 6283, a rate of
     * 340,000 counts per second is the point of overflow -- about 20
     * revolutions per second faster than this motor will ever turn. */
    return (counts_per_second * UNITS_MRAD_PER_TURN) / UNITS_COUNTS_PER_TURN;
}

/**
 * Milliradians per second to revolutions per minute, for anyone who
 * thinks in those.
 *
 * @param mrads  velocity in milliradians per second
 * @return revolutions per minute
 */
static inline int32_t units_mrads_to_rpm(int32_t mrads)
{
    /* One revolution per minute is 6283/60 milliradians per second,
     * which is 104.7. Multiplying by 60 first keeps the precision. */
    return (mrads * 60) / UNITS_MRAD_PER_TURN;
}

#endif /* UNITS_H_ */
