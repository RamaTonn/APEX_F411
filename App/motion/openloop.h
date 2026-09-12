/*
 * openloop.h
 *
 * Spins the motor without any position feedback, the way a stepper is
 * driven: a rotating voltage vector is applied at a chosen frequency and
 * the rotor is expected to keep up.
 *
 * WHAT THIS IS FOR
 *
 *   It is the first thing that makes the motor turn continuously, and it
 *   needs neither the encoder offset nor the pole pair count -- so it can
 *   be tested before either has been measured. It is also how those two
 *   get measured in practice: with the motor turning steadily, the
 *   encoder reading can be compared against the applied electrical angle.
 *
 *   It is NOT a way to drive a load. There is no feedback, so if the
 *   rotor falls behind the applied vector it simply stalls, and a stalled
 *   motor draws whatever current the applied voltage and the winding
 *   resistance allow.
 *
 * WHY THE AMPLITUDE NUMBERS ARE SO SMALL
 *
 *   The winding measured about 36 milliohms per phase. To push 2 amps
 *   through that takes 72 millivolts. Against a 12 volt bus that is 6
 *   parts per thousand of the bus.
 *
 *   So the useful range of the amplitude setting is single digits, not
 *   hundreds. An amplitude that looks trivially small will produce a
 *   large current. Start at 2 and work up while watching the current.
 *
 * DEAD TIME, AND WHY IT IS COMPENSATED HERE
 *
 *   The gate driver deliberately waits about 520 nanoseconds between
 *   turning one transistor off and the other on, so that both are never
 *   on together. Adding the transistors' own turn-on delay, roughly 750
 *   nanoseconds of every period belongs to neither transistor.
 *
 *   During that gap the phase output is not driven at all. Its voltage is
 *   instead pulled by the current already flowing in the winding: current
 *   flowing out of the terminal pulls it down, current flowing in pulls
 *   it up. The result is a voltage error whose size is fixed and whose
 *   sign follows the current.
 *
 *   At 31.25 microseconds per period, 750 nanoseconds is 24 parts per
 *   thousand -- four times larger than the 6 parts per thousand that
 *   would drive 2 amps. Uncompensated, the distortion would completely
 *   overwhelm the command, and the current would be set by the dead time
 *   rather than by anything asked for.
 *
 *   Compensation is simple once the cause is clear: measure which way the
 *   current is flowing in each phase and add back the voltage the dead
 *   time removed.
 */

#ifndef OPENLOOP_H_
#define OPENLOOP_H_

#include <stdint.h>

#include "motor.h"

/* Largest amplitude this module will apply, in parts per thousand of the
 * bus voltage, measured as the peak of the sine rather than its
 * peak-to-peak swing.
 *
 * 60 against a 12 volt bus is 720 millivolts, which on a 36 milliohm
 * winding is about 20 amps -- far beyond anything intended. The ceiling
 * is not a safe operating point; it is a backstop against a typing
 * mistake. */
#define OPENLOOP_AMPLITUDE_CEILING 60U

/* Highest electrical frequency that may be commanded, in hertz.
 *
 * Without feedback the rotor can only follow if the field rotates slowly
 * enough for it to keep up. Beyond that it slips and stalls, which draws
 * heavy current while producing no motion. */
#define OPENLOOP_FREQUENCY_CEILING 200U

/* Default current above which the loop shuts itself down, in milliamps.
 *
 * Changeable at runtime with openloop_set_current_limit(), including
 * switching it off entirely.
 *
 * As a guide to what to expect: commanded phase voltage is the amplitude
 * as a fraction of the bus, and current is that divided by the winding
 * resistance. On this motor, 36 milliohms against a 12 volt bus, each
 * unit of amplitude is worth roughly 330 milliamps. So an amplitude of
 * 15 sits right on this limit. */
#define OPENLOOP_DEFAULT_CURRENT_LIMIT_MA 5000

/* How many consecutive samples must exceed the limit before the loop
 * aborts.
 *
 * A single sample is a poor basis for shutting down. The measurement
 * carries two or three counts of noise, roughly 100 milliamps, and a
 * genuine one-off spike from a switching transient is not what damages
 * anything -- sustained current is. Requiring a run of consecutive
 * samples ignores both.
 *
 * 32 samples at the 32 kHz loop rate is one millisecond. That is far too
 * short for a winding or a transistor to heat appreciably, so nothing is
 * given up by waiting, and it removes every spurious trip. */
#define OPENLOOP_ABORT_CONSECUTIVE_SAMPLES 32U

/* Filter strength for the average current magnitude, as a right shift.
 *
 * The average is updated each period by adding a fraction of the
 * difference between the new sample and the running value. A shift of 8
 * means one 256th of the difference, giving a time constant of 256 loop
 * periods, which is 8 milliseconds.
 *
 * Comparing this average against the peak is what distinguishes a brief
 * spike from a steady current: if the peak is far above the average, it
 * was a transient; if they are close, the current is genuinely
 * continuous at that level. */
#define OPENLOOP_AVERAGE_FILTER_SHIFT 8U

/**
 * Prepare the module. Does not move anything.
 *
 * Must be called after control_init(), because it installs itself as the
 * control loop's function.
 */
void openloop_init(motor_t *m);

/**
 * Begin rotating the voltage vector.
 *
 * Enables all three phases, which charges each bootstrap capacitor in
 * turn, then starts the vector from whatever angle it last held.
 *
 * @param frequency_hz     electrical revolutions per second. Mechanical
 *                         speed is this divided by the pole pair count,
 *                         so a motor with 7 pole pairs turning at 7 Hz
 *                         electrical makes one mechanical revolution per
 *                         second. Clamped to OPENLOOP_FREQUENCY_CEILING.
 * @param amplitude        peak phase voltage as parts per thousand of the
 *                         bus. Clamped to OPENLOOP_AMPLITUDE_CEILING.
 *                         Start at 2 and increase gradually.
 * @return 1 on success, 0 if the control loop is not running or a fault
 *         is currently latched
 */
uint8_t openloop_start(uint16_t frequency_hz, uint16_t amplitude);

/**
 * Stop rotating and disable the bridge. The motor coasts.
 */
void openloop_stop(void);

/**
 * Change the rotation rate without stopping.
 *
 * The electrical angle is not reset, so the vector continues smoothly
 * from where it is -- changing frequency mid-spin does not jerk the
 * rotor.
 *
 * @param frequency_hz  new rate, clamped to the ceiling
 */
void openloop_set_frequency(uint16_t frequency_hz);

/**
 * Change the drive amplitude without stopping.
 *
 * @param amplitude  new amplitude in parts per thousand of bus voltage,
 *                   clamped to the ceiling
 */
void openloop_set_amplitude(uint16_t amplitude);

/**
 * @return 1 if the vector is currently rotating, 0 otherwise
 */
uint8_t openloop_is_running(void);

/**
 * Report what the module is doing, for the console.
 *
 * @param frequency_out       where to store the commanded frequency in
 *                            hertz. May be NULL.
 * @param amplitude_out       where to store the commanded amplitude in
 *                            parts per thousand. May be NULL.
 * @param electrical_angle_out  where to store the applied electrical
 *                            angle, 0 to 65535 for one electrical
 *                            revolution. May be NULL.
 * @param aborted_out         where to store 1 if the loop stopped itself
 *                            because of overcurrent, 0 otherwise. May be
 *                            NULL.
 */
void openloop_get_state(uint16_t *frequency_out,
                        uint16_t *amplitude_out,
                        uint16_t *electrical_angle_out,
                        uint8_t  *aborted_out);

/**
 * Report current extremes since the last start.
 *
 * @param peak_out   where to store the largest phase current magnitude
 *                   seen since the last start, in milliamps. This is the
 *                   number to watch when choosing an amplitude: it shows
 *                   how close the peak came to the abort threshold.
 *                   May be NULL.
 * @param abort_out  where to store the current that triggered an abort,
 *                   in milliamps, or 0 if none has occurred. A value
 *                   only slightly above OPENLOOP_ABORT_CURRENT_MA means
 *                   the limit is merely tight; a much larger one means
 *                   something genuinely went wrong. May be NULL.
 */
void openloop_get_currents(int32_t *peak_out, int32_t *abort_out);

/**
 * Filtered average of the phase current magnitude.
 *
 * Smoothed over roughly 8 milliseconds. Read alongside the peak from
 * openloop_get_currents() to tell a transient from a steady current: a
 * peak far above this average was a spike, while a peak close to it
 * means the current really is that high continuously.
 *
 * @return average magnitude in milliamps
 */
int32_t openloop_get_average_current(void);

/**
 * Change or remove the current abort limit.
 *
 * The limit exists to stop a runaway before it destroys a transistor.
 * Open-loop drive has nothing regulating current, so a stalled rotor or
 * a wrong amplitude draws whatever the applied voltage and the winding
 * resistance allow, and on a 36 milliohm winding that reaches damaging
 * levels in a fraction of a second.
 *
 * Switching it off is sometimes the right call during bring-up -- when
 * the limit is firing on currents you deliberately asked for, and you
 * are watching the supply and the motor yourself. Just know that with it
 * off, nothing in this module will stop anything. The protection
 * supervisor in the main loop is unaffected and still applies, unless
 * that is disabled too.
 *
 * @param limit_ma  new limit in milliamps, or 0 to disable the check
 *                  entirely
 */
void openloop_set_current_limit(int32_t limit_ma);

/**
 * @return the current abort limit in milliamps, or 0 if disabled
 */
int32_t openloop_get_current_limit(void);

#endif /* OPENLOOP_H_ */
