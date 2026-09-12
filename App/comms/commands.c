#include "commands.h"

#include "apex_board.h"
#include "cobs.h"
#include "calibration.h"
#include "loop.h"
#include "crc16.h"
#include "currentloop.h"
#include "encoder.h"
#include "estimate.h"
#include "motor.h"
#include "gate_driver.h"
#include "main.h"
#include "openloop.h"
#include "packet.h"
#include "protection.h"
#include "protocol_binary.h"
#include "sensors.h"
#include "units.h"
#include "telemetry.h"

#include <string.h>

/* The motor the handlers report on and configure. */
static motor_t *motor;

void commands_init(motor_t *m)
{
    motor = m;
}

/* ------------------------------------------------------------------
 * System
 * ------------------------------------------------------------------ */

static void command_ping(const protocol_args_t *args)
{
    (void)args;
    protocol_reply_begin(PROTOCOL_STATUS_OK, "ping");
    protocol_reply_end();
}

static void command_id(const protocol_args_t *args)
{
    (void)args;
    protocol_reply_begin(PROTOCOL_STATUS_OK, "id");
    protocol_reply_text("name", "apex");
    protocol_reply_text("fw", "0.2.0");
    protocol_reply_end();
}

static void command_up(const protocol_args_t *args)
{
    (void)args;
    protocol_reply_begin(PROTOCOL_STATUS_OK, "up");
    protocol_reply_uint("ms", HAL_GetTick());
    protocol_reply_end();
}

static void command_echo(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 2u) {
        protocol_reply_error("echo", "usage_echo_0_or_1");
        return;
    }
    protocol_set_echo((protocol_arg_int(args, 1u, 0) != 0) ? 1u : 0u);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "echo");
    protocol_reply_uint("on", protocol_get_echo());
    protocol_reply_end();
}

/* Switch the format replies are written in.
 *
 * The reply to this command is written in the OLD format, because it is
 * built before the switch takes effect. That is deliberate: a host that
 * asked for binary can still read the acknowledgement it was expecting,
 * and only what follows changes. */
static void command_mode(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 2u) {
        protocol_reply_error("mode", "usage_mode_text_or_binary");
        return;
    }

    uint8_t requested;

    if (protocol_arg_matches(args, 1u, "text", 0) != 0u) {
        requested = PROTOCOL_MODE_TEXT;
    } else if (protocol_arg_matches(args, 1u, "binary", 1) != 0u) {
        requested = PROTOCOL_MODE_BINARY;
    } else {
        protocol_reply_error("mode", "bad_value");
        return;
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "mode");
    protocol_reply_uint("mode", requested);
    protocol_reply_end();

    protocol_set_mode(requested);
}

static void command_help(const protocol_args_t *args);

/* Verify the transport primitives on the target itself.
 *
 * The same checks pass on a PC, so a failure here means the compiler or
 * the machine differs from where they were developed -- worth knowing
 * before the protocol is trusted rather than after. */
static void command_selftest(const protocol_args_t *args)
{
    static const uint8_t crc_check_input[] = "123456789";
    static const uint8_t plain[]           = { 0x11u, 0x22u, 0x00u, 0x33u };

    uint8_t encoded[COBS_ENCODED_MAX(sizeof plain)];
    uint8_t decoded[sizeof plain];

    uint8_t zero_found    = 0u;
    uint8_t round_trip_ok = 0u;

    (void)args;

    uint16_t crc_result = crc16_compute(crc_check_input, 9u);

    size_t encoded_length = cobs_encode(plain, sizeof plain,
                                        encoded, sizeof encoded);
    if (encoded_length > 0u) {
        for (size_t i = 0u; i < encoded_length; i++) {
            if (encoded[i] == 0u) {
                zero_found = 1u;
            }
        }
        size_t decoded_length = cobs_decode(encoded, encoded_length,
                                            decoded, sizeof decoded);
        if (decoded_length == sizeof plain) {
            round_trip_ok = (memcmp(decoded, plain, sizeof plain) == 0)
                              ? 1u : 0u;
        }
    }

    uint32_t accepted;
    uint32_t rejected;
    protocol_binary_get_counts(&accepted, &rejected);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "selftest");
    protocol_reply_uint("crc",     crc_result);
    protocol_reply_uint("crc_ok",  (crc_result == CRC16_CHECK_VALUE) ? 1u : 0u);
    protocol_reply_uint("nozero",  (zero_found == 0u) ? 1u : 0u);
    protocol_reply_uint("trip_ok", round_trip_ok);
    protocol_reply_uint("acc",     accepted);
    protocol_reply_uint("rej",     rejected);
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Sensors
 * ------------------------------------------------------------------ */

static void command_sense(const protocol_args_t *args)
{
    int32_t current_a;
    int32_t current_b;

    (void)args;
    sensors_get_currents(&current_a, &current_b);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "sense");
    protocol_reply_int("ia_ma", current_a);
    protocol_reply_int("ib_ma", current_b);
    protocol_reply_uint("vbus_mv", sensors_get_bus_mv());
    /* Temperature is still raw counts: the thermistor is off-board on
     * connector J5 and its curve is unknown, so there is no honest way
     * to convert it yet. A reading near full scale means nothing is
     * plugged in. */
    protocol_reply_uint("temp_raw", sensors_get_temperature_counts());
    protocol_reply_end();
}

static void command_angle(const protocol_args_t *args)
{
    uint16_t angle;

    (void)args;
    if (encoder_read_angle(&angle) == 0u) {
        protocol_reply_error("angle", "read_failed");
        return;
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "angle");
    protocol_reply_int("pos_mrad", units_counts_to_mrad(angle));
    protocol_reply_uint("raw", angle);
    protocol_reply_end();
}

static void command_encdiag(const protocol_args_t *args)
{
    encoder_diagnostics_t diagnostics;

    (void)args;
    if (encoder_read_diagnostics(&diagnostics) == 0u) {
        protocol_reply_error("encdiag", "read_failed");
        return;
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "encdiag");
    protocol_reply_uint("agc",    diagnostics.automatic_gain);
    protocol_reply_uint("mag",    diagnostics.field_magnitude);
    protocol_reply_uint("weak",   diagnostics.magnet_too_weak);
    protocol_reply_uint("strong", diagnostics.magnet_too_strong);
    protocol_reply_uint("cof",    diagnostics.cordic_overflow);
    protocol_reply_uint("lf",     diagnostics.offset_ready);
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Bridge
 * ------------------------------------------------------------------ */

/* Read a phase argument.
 *
 * Accepts a letter in text mode and a number in binary, so one handler
 * serves both.
 *
 * @param args       the argument block
 * @param index      which argument
 * @param phase_out  where to store the phase constant
 * @return 1 if the argument named a phase, 0 otherwise */
static uint8_t read_phase(const protocol_args_t *args,
                          uint32_t               index,
                          uint8_t               *phase_out)
{
    if (protocol_arg_matches(args, index, "a", 0) != 0u) {
        *phase_out = GATE_DRIVER_PHASE_A;
    } else if (protocol_arg_matches(args, index, "b", 1) != 0u) {
        *phase_out = GATE_DRIVER_PHASE_B;
    } else if (protocol_arg_matches(args, index, "c", 2) != 0u) {
        *phase_out = GATE_DRIVER_PHASE_C;
    } else {
        return 0u;
    }
    return 1u;
}

static void command_duty(const protocol_args_t *args)
{
    uint8_t phase;

    if (protocol_arg_count(args) != 3u) {
        protocol_reply_error("duty", "usage_duty_phase_value");
        return;
    }
    if (read_phase(args, 1u, &phase) == 0u) {
        protocol_reply_error("duty", "bad_phase");
        return;
    }

    int32_t requested = protocol_arg_int(args, 2u, 0);
    if (requested < 0) {
        protocol_reply_error("duty", "negative_value");
        return;
    }

    gate_driver_set_duty(phase, (uint16_t)requested);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "duty");
    protocol_reply_uint("duty", gate_driver_get_duty(phase));
    protocol_reply_end();
}

static void command_en(const protocol_args_t *args)
{
    uint8_t phase;

    if (protocol_arg_count(args) != 3u) {
        protocol_reply_error("en", "usage_en_phase_0_or_1");
        return;
    }
    if (read_phase(args, 1u, &phase) == 0u) {
        protocol_reply_error("en", "bad_phase");
        return;
    }

    if (protocol_arg_int(args, 2u, 0) != 0) {
        gate_driver_enable_phase(phase);
    } else {
        gate_driver_disable_phase(phase);
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "en");
    protocol_reply_uint("on", gate_driver_is_enabled(phase));
    protocol_reply_end();
}

static void command_stop(const protocol_args_t *args)
{
    (void)args;
    openloop_stop();
    gate_driver_disable_all();

    protocol_reply_begin(PROTOCOL_STATUS_OK, "stop");
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Control loop
 * ------------------------------------------------------------------ */

static void command_loop(const protocol_args_t *args)
{
    int32_t current_a;
    int32_t current_b;

    (void)args;
    sensors_get_currents(&current_a, &current_b);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "loop");
    /* init separates "never set up" from "set up but stopped". A zero
     * here means the timer is not triggering the ADC, and nothing that
     * depends on current measurement will work. */
    protocol_reply_uint("init",  loop_is_initialised());
    protocol_reply_uint("run",   loop_is_running());
    protocol_reply_uint("iters", loop_get_iteration_count());
    protocol_reply_uint("over",  loop_get_overrun_count());
    protocol_reply_uint("us",    loop_get_duration_us());
    protocol_reply_int("ia_ma",  current_a);
    protocol_reply_int("ib_ma",  current_b);
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Open-loop motion
 * ------------------------------------------------------------------ */

static void command_spin(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 3u) {
        protocol_reply_error("spin", "usage_spin_hz_amplitude");
        return;
    }

    int32_t frequency = protocol_arg_int(args, 1u, -1);
    int32_t amplitude = protocol_arg_int(args, 2u, -1);

    if ((frequency < 0) || (amplitude < 0)) {
        protocol_reply_error("spin", "bad_value");
        return;
    }

    /* Starting open-loop drive always takes over the bridge from
     * closed-loop current control if that was left running -- otherwise
     * its stale "running" state would keep reporting itself as active
     * even though the loop's installed function has moved on. */
    currentloop_stop();

    if (openloop_start((uint16_t)frequency, (uint16_t)amplitude) == 0u) {
        protocol_reply_error("spin", "control_loop_not_running");
        return;
    }

    uint16_t applied_frequency;
    uint16_t applied_amplitude;
    openloop_get_state(&applied_frequency, &applied_amplitude, NULL, NULL);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "spin");
    protocol_reply_uint("hz",  applied_frequency);
    protocol_reply_uint("amp", applied_amplitude);
    protocol_reply_end();
}

static void command_amp(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 2u) {
        protocol_reply_error("amp", "usage_amp_value");
        return;
    }

    int32_t amplitude = protocol_arg_int(args, 1u, -1);
    if (amplitude < 0) {
        protocol_reply_error("amp", "bad_value");
        return;
    }
    openloop_set_amplitude((uint16_t)amplitude);

    uint16_t applied_amplitude;
    openloop_get_state(NULL, &applied_amplitude, NULL, NULL);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "amp");
    protocol_reply_uint("amp", applied_amplitude);
    protocol_reply_end();
}

static void command_hz(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 2u) {
        protocol_reply_error("hz", "usage_hz_value");
        return;
    }

    int32_t frequency = protocol_arg_int(args, 1u, -1);
    if (frequency < 0) {
        protocol_reply_error("hz", "bad_value");
        return;
    }
    openloop_set_frequency((uint16_t)frequency);

    uint16_t applied_frequency;
    openloop_get_state(&applied_frequency, NULL, NULL, NULL);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "hz");
    protocol_reply_uint("hz", applied_frequency);
    protocol_reply_end();
}

static void command_spinstat(const protocol_args_t *args)
{
    uint16_t frequency;
    uint16_t amplitude;
    uint16_t electrical_angle;
    uint8_t  aborted;
    int32_t  peak_current;
    int32_t  abort_current;
    int32_t  current_a;
    int32_t  current_b;
    uint16_t encoder_angle = 0u;

    (void)args;

    openloop_get_state(&frequency, &amplitude, &electrical_angle, &aborted);
    openloop_get_currents(&peak_current, &abort_current);
    sensors_get_currents(&current_a, &current_b);
    (void)encoder_read_angle(&encoder_angle);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "spinstat");
    protocol_reply_uint("run",    openloop_is_running());
    protocol_reply_uint("hz",     frequency);
    protocol_reply_uint("amp",    amplitude);
    protocol_reply_uint("eangle", electrical_angle);
    protocol_reply_uint("abort",  aborted);
    protocol_reply_int("ia_ma",   current_a);
    protocol_reply_int("ib_ma",   current_b);
    /* peak is a single-sample maximum; avg is filtered over about eight
     * milliseconds. A peak far above avg was a transient, while a peak
     * close to avg means the current really is that high. */
    protocol_reply_int("peak_ma", peak_current);
    protocol_reply_int("avg_ma",  openloop_get_average_current());
    protocol_reply_int("abort_ma", abort_current);
    protocol_reply_uint("enc",    encoder_angle);
    protocol_reply_end();
}

static void command_limit(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 2u) {
        protocol_reply_error("limit", "usage_limit_ma_or_off");
        return;
    }

    if (protocol_arg_matches(args, 1u, "off", 0) != 0u) {
        openloop_set_current_limit(0);
    } else {
        int32_t milliamps = protocol_arg_int(args, 1u, -1);
        if (milliamps < 0) {
            protocol_reply_error("limit", "bad_value");
            return;
        }
        openloop_set_current_limit(milliamps);
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "limit");
    protocol_reply_int("lim_ma", openloop_get_current_limit());
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Closed-loop current control
 * ------------------------------------------------------------------ */

static void command_iloop(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 3u) {
        protocol_reply_error("iloop", "usage_iloop_id_ma_iq_ma");
        return;
    }

    int32_t id_ma = protocol_arg_int(args, 1u, 0);
    int32_t iq_ma = protocol_arg_int(args, 2u, 0);

    if (currentloop_is_running() != 0u) {
        /* Already regulating -- just move the targets, which keeps the
         * integrators and the charged bootstrap capacitors rather than
         * restarting from rest. */
        currentloop_set_targets(id_ma, iq_ma);
    } else {
        /* Starting closed-loop current control always takes over the
         * bridge from open-loop drive if that was left running --
         * otherwise its stale "running" state would keep reporting
         * itself as active even though the loop's installed function
         * has moved on. */
        openloop_stop();

        if (currentloop_start(id_ma, iq_ma) == 0u) {
            protocol_reply_error("iloop", "not_ready");
            return;
        }
    }

    int32_t id_target_ma;
    int32_t iq_target_ma;
    currentloop_get_state(&id_target_ma, &iq_target_ma, NULL, NULL);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "iloop");
    protocol_reply_int("id_ma", id_target_ma);
    protocol_reply_int("iq_ma", iq_target_ma);
    protocol_reply_end();
}

static void command_iloopstop(const protocol_args_t *args)
{
    (void)args;
    currentloop_stop();

    protocol_reply_begin(PROTOCOL_STATUS_OK, "iloopstop");
    protocol_reply_end();
}

static void command_iloopstat(const protocol_args_t *args)
{
    int32_t id_target_ma;
    int32_t iq_target_ma;
    int32_t id_measured_ma;
    int32_t iq_measured_ma;

    (void)args;
    currentloop_get_state(&id_target_ma, &iq_target_ma,
                          &id_measured_ma, &iq_measured_ma);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "iloopstat");
    protocol_reply_uint("run",       currentloop_is_running());
    protocol_reply_int("id_ma",      id_target_ma);
    protocol_reply_int("iq_ma",      iq_target_ma);
    protocol_reply_int("id_meas_ma", id_measured_ma);
    protocol_reply_int("iq_meas_ma", iq_measured_ma);
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Protection
 * ------------------------------------------------------------------ */

static void command_fault(const protocol_args_t *args)
{
    uint8_t faults = protection_get_faults();

    (void)args;
    protocol_reply_begin(PROTOCOL_STATUS_OK, "fault");
    protocol_reply_uint("bits", faults);

    if (faults == PROTECTION_FAULT_NONE) {
        protocol_reply_text("reason", "none");
    } else {
        /* Several faults can be latched together, so every set bit is
         * reported rather than only the first. */
        for (uint8_t bit = 0x01u; bit != 0u; bit = (uint8_t)(bit << 1)) {
            if ((faults & bit) != 0u) {
                protocol_reply_text("reason", protection_fault_text(bit));
            }
        }
    }
    protocol_reply_end();
}

static void command_clear(const protocol_args_t *args)
{
    (void)args;
    if (protection_clear_faults() == 0u) {
        protocol_reply_error("clear", "condition_still_present");
        return;
    }
    protocol_reply_begin(PROTOCOL_STATUS_OK, "clear");
    protocol_reply_end();
}

static void command_safety(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 2u) {
        protocol_reply_error("safety", "usage_safety_0_or_1");
        return;
    }
    protection_set_enabled((protocol_arg_int(args, 1u, 1) != 0) ? 1u : 0u);

    protocol_reply_begin(PROTOCOL_STATUS_OK, "safety");
    protocol_reply_uint("on",   protection_is_enabled());
    protocol_reply_uint("bits", protection_get_faults());
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Telemetry
 * ------------------------------------------------------------------ */

/* tel <on|off>
 *
 * Starts or stops the continuous sample stream. Samples arrive as "dat"
 * lines or packets, which marks them as unsolicited so a host can tell
 * them from an answer to something it asked for. */
static void command_tel(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 2u) {
        protocol_reply_error("tel", "usage_tel_on_or_off");
        return;
    }

    if (protocol_arg_matches(args, 1u, "on", 1) != 0u) {
        telemetry_start();
    } else if (protocol_arg_matches(args, 1u, "off", 0) != 0u) {
        telemetry_stop();
    } else {
        protocol_reply_error("tel", "bad_value");
        return;
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "tel");
    protocol_reply_uint("run",  telemetry_is_running());
    protocol_reply_uint("bits", telemetry_get_channels());
    protocol_reply_uint("duty", telemetry_get_divider());
    protocol_reply_end();
}

/* telcfg <channel_mask> <divider>
 *
 * The mask selects which signals are streamed, one bit each. The divider
 * takes one loop iteration in N, so 32 gives 1 kHz from the 32 kHz
 * control loop.
 *
 * Sending faster than the link can carry does not fail visibly -- it
 * fills the ring and starts dropping samples, which telstat reports. */
static void command_telcfg(const protocol_args_t *args)
{
    if (protocol_arg_count(args) != 3u) {
        protocol_reply_error("telcfg", "usage_telcfg_mask_divider");
        return;
    }

    int32_t mask    = protocol_arg_int(args, 1u, -1);
    int32_t divider = protocol_arg_int(args, 2u, -1);

    if ((mask < 0) || (divider < 0)) {
        protocol_reply_error("telcfg", "bad_value");
        return;
    }
    if (telemetry_configure((uint32_t)mask, (uint16_t)divider) == 0u) {
        protocol_reply_error("telcfg", "no_channels_selected");
        return;
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "telcfg");
    protocol_reply_uint("bits", telemetry_get_channels());
    protocol_reply_uint("duty", telemetry_get_divider());
    /* The resulting sample rate, so the caller does not have to work it
     * out from the loop rate and the divider. */
    protocol_reply_uint("hz",
        LOOP_RATE_HZ / telemetry_get_divider());
    protocol_reply_end();
}

/* Report how the stream is doing.
 *
 * A non-zero drop count means the plot has gaps. Either the host is not
 * reading fast enough or the divider is too small for the link -- and a
 * gap nobody knows about is worse than no data, because a step that is
 * really a missing millisecond looks like a step that happened. */
static void command_telstat(const protocol_args_t *args)
{
    (void)args;
    protocol_reply_begin(PROTOCOL_STATUS_OK, "telstat");
    protocol_reply_uint("run",  telemetry_is_running());
    protocol_reply_uint("bits", telemetry_get_channels());
    protocol_reply_uint("duty", telemetry_get_divider());
    protocol_reply_uint("t",    telemetry_get_sent());
    protocol_reply_uint("over", telemetry_get_dropped());
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Parameter estimation
 * ------------------------------------------------------------------ */

/* eres
 *
 * Measures each phase's resistance from three line-to-line runs, taking
 * the slope between two operating points in each, which cancels the
 * fixed voltage the dead time loses. A single point cannot separate the
 * two and reports a resistance several times too high. */
static void command_eres(const protocol_args_t *args)
{
    estimate_resistance_result_t result;

    (void)args;

    uint8_t outcome = estimate_resistance(motor, &result);

    if (outcome != ESTIMATE_OK) {
        /* The reason alone does not say whether the drive never got
         * going or ran away, and those want opposite fixes -- so report
         * where it stopped and how far the current had got by then.
         * Which pair stopped narrows it further: a bad connection on one
         * phase fails the two runs that touch it and passes the third. */
        protocol_reply_begin(PROTOCOL_STATUS_ERROR, "eres");
        protocol_reply_text("reason",  estimate_result_text(outcome));
        protocol_reply_text("at_pair", estimate_pair_text(result.fault_pair));
        protocol_reply_uint("at_duty", result.fault_duty);
        protocol_reply_int("at_ma",    result.fault_current_ma);
        protocol_reply_end();
        return;
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "eres");
    /* Per phase, which is what the question was, followed by the three
     * line-to-line measurements they were solved from so the arithmetic
     * can be checked by hand: R_a = (R_ab + R_ca - R_bc) / 2. */
    protocol_reply_uint("a_mohm",  result.a_mohm);
    protocol_reply_uint("b_mohm",  result.b_mohm);
    protocol_reply_uint("c_mohm",  result.c_mohm);
    protocol_reply_uint("ab_mohm", result.ab_mohm);
    protocol_reply_uint("bc_mohm", result.bc_mohm);
    protocol_reply_uint("ca_mohm", result.ca_mohm);
    /* The mean, which is the one number the control loop is configured
     * with, and the spread the averaging hides. */
    protocol_reply_uint("mohm",    result.phase_mohm);
    protocol_reply_uint("imbal",   result.imbalance_percent);
    protocol_reply_uint("vbus_mv", result.bus_mv);
    protocol_reply_end();
}

/* eind
 *
 * Measures inductance along and across the rotor's magnet axis,
 * independently of one another.
 *
 * The ratio between them decides whether rotor position can be estimated
 * by injecting a high-frequency signal: the injected current responds
 * differently with position only because the inductance does, so with no
 * difference there is nothing to detect. */
static void command_eind(const protocol_args_t *args)
{
    estimate_inductance_result_t result;

    (void)args;

    uint8_t outcome = estimate_inductance(motor, &result);

    if (outcome != ESTIMATE_OK) {
        /* The current change each axis did manage is reported even on a
         * failure: "too small" with a change of nearly nothing means the
         * drive never got going, while one just under the threshold
         * means only the pulse needs to be bigger. */
        protocol_reply_begin(PROTOCOL_STATUS_ERROR, "eind");
        protocol_reply_text("reason",  estimate_result_text(outcome));
        protocol_reply_int("dchg_ma",  result.d_current_change_ma);
        protocol_reply_int("qchg_ma",  result.q_current_change_ma);
        protocol_reply_uint("ld_nh",   result.inductance_d_nh);
        protocol_reply_end();
        return;
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "eind");
    protocol_reply_uint("ld_nh",  result.inductance_d_nh);
    protocol_reply_uint("lq_nh",  result.inductance_q_nh);
    /* Saliency as a percentage: how much larger the across-axis
     * inductance is. Under about 110 means injection will not work
     * reliably on this motor. */
    protocol_reply_uint("sal",    result.saliency_percent);
    protocol_reply_int("dchg_ma", result.d_current_change_ma);
    protocol_reply_int("qchg_ma", result.q_current_change_ma);
    protocol_reply_uint("d_step", result.d_duty_used);
    protocol_reply_uint("q_step", result.q_duty_used);
    /* The series resistance each axis was solved against. The answer
     * depends on it, so it is reported rather than left implicit --
     * a resistance measured on a cold motor and an inductance taken
     * after a long run will not agree. */
    protocol_reply_uint("dr_mohm", result.d_series_mohm);
    protocol_reply_uint("qr_mohm", result.q_series_mohm);
    protocol_reply_end();
}

/* Report every measured parameter and what is still missing. */
static void command_param(const protocol_args_t *args)
{
    (void)args;

    /* The motor holds SI units; the protocol reports milliohms and
     * nanohenries, so the conversion happens here at the boundary. The
     * inductances are in nanohenries rather than microhenries because a
     * motor this board drives has single-figure microhenries per phase,
     * and rounding that to an integer would throw away a tenth of the
     * answer -- see estimate.h. */
    protocol_reply_begin(PROTOCOL_STATUS_OK, "param");
    protocol_reply_uint("mohm",
        (uint32_t)(motor->resistance_ohm * 1000.0f));
    protocol_reply_uint("ld_nh",
        (uint32_t)(motor->inductance_d_h * 1e9f));
    protocol_reply_uint("lq_nh",
        (uint32_t)(motor->inductance_q_h * 1e9f));
    protocol_reply_uint("poles", motor_get_pole_pairs(motor));
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * Calibration and commutation
 * ------------------------------------------------------------------ */

/* calib [amplitude]
 *
 * Measures direction, pole pairs and offset in one pass and applies
 * them, so closed-loop control can be started immediately afterwards.
 *
 * Takes several seconds and turns the rotor, which must be free. The
 * amplitude defaults to something small enough to be safe on a low
 * resistance winding and large enough to overcome cogging. */
static void command_calib(const protocol_args_t *args)
{
    calibration_result_t result;

    /* Each unit of amplitude is worth roughly 330 milliamps on this
     * motor, so 15 is around five amps.
     *
     * That is more than the torque alone needs, and the reason is the
     * dead time. It costs 24 parts per thousand of the bus, so an
     * amplitude below that is dominated by the correction rather than by
     * the sine, and the rotating field comes out badly shaped. Above it,
     * the sine leads and the rotor follows properly.
     *
     * This is a symptom of a very low resistance winding on a 12 volt
     * bus: the voltage range that produces sensible current sits below
     * the dead time. Current control removes the problem, since it
     * commands amps and lets the loop absorb the correction -- but that
     * needs the constants this command exists to measure. */
    int32_t amplitude = protocol_arg_int(args, 1u, 15);

    if ((amplitude <= 0) || (amplitude > 60)) {
        protocol_reply_error("calib", "bad_amplitude");
        return;
    }

    uint8_t outcome = calibration_run(motor, (uint16_t)amplitude, &result);

    if (outcome != CALIBRATION_OK) {
        /* The partial measurements are still reported, because they say
         * a great deal about what went wrong: no movement at all is a
         * different problem from movement that did not repeat. */
        protocol_reply_begin(PROTOCOL_STATUS_ERROR, "calib");
        protocol_reply_text("reason", calibration_result_text(outcome));
        protocol_reply_int("fwd",     result.forward_counts);
        protocol_reply_int("rev",     result.reverse_counts);
        protocol_reply_int("peak_ma", result.peak_current_ma);
        protocol_reply_end();
        return;
    }

    protocol_reply_begin(PROTOCOL_STATUS_OK, "calib");
    protocol_reply_uint("poles",  result.pole_pairs);
    protocol_reply_uint("dir",    result.direction_forward);
    /* The offset in milliradians rather than encoder counts: counts are
     * an accident of the sensor, radians are the actual angle. */
    protocol_reply_int("off_mrad",
        units_counts_to_mrad(result.offset_counts));
    /* How far the pole pair count had to be rounded, in hundredths. Near
     * zero means a clean fit; anything large means the rotor was not
     * following faithfully and the count is doubtful. */
    /* fit is how far the count had to be rounded, in hundredths, and
     * raw is the measurement before rounding. A raw of 730 with a fit of
     * 30 says seven pole pairs measured three percent high, which is
     * what a slightly off-centre encoder magnet produces. */
    protocol_reply_uint("fit",     result.pole_pair_error_percent);
    protocol_reply_uint("raw",     result.pole_pairs_hundredths);
    protocol_reply_uint("spread",  result.offset_spread);
    protocol_reply_int("peak_ma",  result.peak_current_ma);
    protocol_reply_end();
}

/* comm [0|1]
 *
 * With no argument, reports the constants and the rotor position. With
 * an argument, turns the per-period encoder read on or off.
 *
 * That read costs real time inside the control interrupt, which sits
 * above USB in priority -- enough of it and the link stops working. So
 * it is off unless something needs the angle. */
static void command_comm(const protocol_args_t *args)
{
    if (protocol_arg_count(args) == 2u) {
        motor_set_reading_enabled(motor,
            (protocol_arg_int(args, 1u, 0) != 0) ? 1u : 0u);
    }
    protocol_reply_begin(PROTOCOL_STATUS_OK, "comm");
    protocol_reply_uint("on",    motor_reading_enabled(motor));
    protocol_reply_uint("poles", motor_get_pole_pairs(motor));
    protocol_reply_uint("dir",   encoder_get_direction(motor->encoder));
    protocol_reply_int("off_mrad",
        units_counts_to_mrad(encoder_get_offset(motor->encoder)));
    /* Shaft position and rotor electrical angle, both in milliradians of
     * their own kind of revolution. */
    protocol_reply_int("pos_mrad",
        units_counts_to_mrad(encoder_get_raw_count(motor->encoder)));
    protocol_reply_int("eangle_mrad",
        units_angle_to_mrad(motor_get_electrical_angle(motor)));
    protocol_reply_end();
}

/* ------------------------------------------------------------------
 * The table
 *
 * Numbers are written out rather than derived from position, so that
 * inserting a command does not renumber the ones after it and break a
 * host built against an older firmware.
 * ------------------------------------------------------------------ */

static const struct {
    const char *name;
    uint8_t     id;
    void      (*handler)(const protocol_args_t *args);
} command_table[] = {
    /* system */
    { "ping",     0x01u, command_ping     },
    { "id",       0x02u, command_id       },
    { "up",       0x03u, command_up       },
    { "echo",     0x04u, command_echo     },
    { "mode",     0x05u, command_mode     },
    { "help",     0x06u, command_help     },
    { "selftest", 0x07u, command_selftest },
    /* sensors */
    { "sense",    0x10u, command_sense    },
    { "angle",    0x11u, command_angle    },
    { "encdiag",  0x12u, command_encdiag  },
    /* bridge */
    { "duty",     0x20u, command_duty     },
    { "en",       0x21u, command_en       },
    { "stop",     0x22u, command_stop     },
    /* control loop */
    { "loop",     0x30u, command_loop     },
    /* open-loop motion */
    { "spin",     0x50u, command_spin     },
    { "amp",      0x51u, command_amp      },
    { "hz",       0x52u, command_hz       },
    { "spinstat", 0x53u, command_spinstat },
    { "limit",    0x54u, command_limit    },
    /* closed-loop current control */
    { "iloop",     0x90u, command_iloop     },
    { "iloopstop", 0x91u, command_iloopstop },
    { "iloopstat", 0x92u, command_iloopstat },
    /* calibration and commutation */
    { "calib",    0x80u, command_calib    },
    { "comm",     0x81u, command_comm     },
    { "eres",     0x85u, command_eres     },
    { "eind",     0x86u, command_eind     },
    { "param",    0x87u, command_param    },
    /* telemetry */
    { "tel",      0x70u, command_tel      },
    { "telcfg",   0x71u, command_telcfg   },
    { "telstat",  0x72u, command_telstat  },
    /* protection */
    { "fault",    0x60u, command_fault    },
    { "clear",    0x61u, command_clear    },
    { "safety",   0x62u, command_safety   },
};

#define COMMAND_COUNT (sizeof command_table / sizeof command_table[0])

/* List the command names.
 *
 * The reply buffer is finite and this list grows with every command
 * added, so the list is truncated rather than allowed to overflow. In
 * binary mode the names are of little use anyway -- a host works from
 * numbers -- so only the count is reported there. */
static void command_help(const protocol_args_t *args)
{
    (void)args;

    protocol_reply_begin(PROTOCOL_STATUS_OK, "help");
    protocol_reply_uint("bits", COMMAND_COUNT);

    if (protocol_get_mode() == PROTOCOL_MODE_TEXT) {
        for (uint32_t i = 0u; i < COMMAND_COUNT; i++) {
            protocol_reply_text("c", command_table[i].name);
        }
    }
    protocol_reply_end();
}

void commands_dispatch(const char *name, const protocol_args_t *args)
{
    for (uint32_t i = 0u; i < COMMAND_COUNT; i++) {
        if (strcmp(name, command_table[i].name) == 0) {
            command_table[i].handler(args);
            return;
        }
    }
    /* Every command that arrives gets exactly one reply, so a host can
     * always match a response to its request instead of waiting on a
     * timeout to discover the name was wrong. */
    protocol_reply_error(name, "unknown_command");
}

uint8_t commands_id_for(const char *name)
{
    for (uint32_t i = 0u; i < COMMAND_COUNT; i++) {
        if (strcmp(name, command_table[i].name) == 0) {
            return command_table[i].id;
        }
    }
    return 0u;
}

const char *commands_name_for(uint8_t id)
{
    for (uint32_t i = 0u; i < COMMAND_COUNT; i++) {
        if (command_table[i].id == id) {
            return command_table[i].name;
        }
    }
    return NULL;
}

uint32_t commands_count(void)
{
    return COMMAND_COUNT;
}

const char *commands_name_at(uint32_t index)
{
    if (index >= COMMAND_COUNT) {
        return NULL;
    }
    return command_table[index].name;
}
