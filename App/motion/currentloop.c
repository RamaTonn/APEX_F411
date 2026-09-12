#include "currentloop.h"

#include <stddef.h>

#include "encoder.h"
#include "gate_driver.h"
#include "pid.h"
#include "sensors.h"
#include "telemetry.h"

/* Milliamps to amps, and millivolts to volts. */
#define MILLI_TO_UNIT 0.001f

/* Radians per second per hertz. */
#define TWO_PI 6.28318530718f

/* The motor this loop regulates. */
static motor_t *motor;

static pid_t id_pid;
static pid_t iq_pid;

/* Targets and the most recent measurement, in amps. Kept in amps rather
 * than milliamps since that is what pid_update() and motor_get_dq_currents()
 * both work in -- the milliamp conversion happens only at the console
 * boundary, in currentloop_start()/set_targets()/get_state(). */
static volatile float target_id_a;
static volatile float target_iq_a;
static volatile float measured_id_a;
static volatile float measured_iq_a;

static volatile uint8_t running;

/* ------------------------------------------------------------------
 * The control function
 *
 * Installed into the loop module and called once per PWM period in
 * interrupt context, with both measured currents already converted to
 * milliamps and the rotor angle already refreshed by motor_update().
 * ------------------------------------------------------------------ */
static void currentloop_step(int32_t current_a_ma, int32_t current_b_ma)
{
    if (running == 0u) {
        return;
    }

    uint16_t electrical_angle = motor_get_electrical_angle(motor);
    float    angle_rad        = (float)electrical_angle * MOTOR_ANGLE_TO_RAD;

    float id_a;
    float iq_a;
    motor_get_dq_currents(motor, current_a_ma, current_b_ma, angle_rad,
                          &id_a, &iq_a);
    measured_id_a = id_a;
    measured_iq_a = iq_a;

    /* Recomputed every period from the measured bus -- see OUTPUT
     * LIMITS TRACK THE MEASURED BUS in currentloop.h for why a fixed
     * limit chosen at init time would undermine anti-windup. */
    uint16_t bus_mv          = sensors_get_bus_mv();
    float    bus_volts       = (float)bus_mv * MILLI_TO_UNIT;
    float    output_limit    = bus_volts
                              * ((float)CURRENTLOOP_OUTPUT_LIMIT_PER_MILLE
                                 * MILLI_TO_UNIT);
    pid_set_output_limits(&id_pid, -output_limit, output_limit);
    pid_set_output_limits(&iq_pid, -output_limit, output_limit);

    float v_d = pid_update(&id_pid, target_id_a, id_a);
    float v_q = pid_update(&iq_pid, target_iq_a, iq_a);

    motor_apply_dq(motor, v_d, v_q, angle_rad,
                   current_a_ma, current_b_ma, bus_mv);

    /* Published every period so telemetry can attach it to this
     * sample -- unlike openloop.c, the encoder angle is real here
     * rather than a placeholder, since closed-loop control already
     * pays for reading it. */
    telemetry_set_angles(electrical_angle, encoder_get_raw_count(motor->encoder));
}

/* ------------------------------------------------------------------
 * Public interface
 * ------------------------------------------------------------------ */

void currentloop_init(motor_t *m)
{
    motor = m;

    target_id_a   = 0.0f;
    target_iq_a   = 0.0f;
    measured_id_a = 0.0f;
    measured_iq_a = 0.0f;
    running       = 0u;
}

uint8_t currentloop_start(int32_t id_target_ma, int32_t iq_target_ma)
{
    /* Without the control loop running, the function above would never
     * be called and the bridge would sit at whatever duty it was left
     * at -- same reasoning as openloop_start(). */
    if (loop_is_running() == 0u) {
        return 0u;
    }

    /* Gains computed from zero would silently produce zero gains
     * rather than an error, which looks like a current loop doing
     * nothing rather than one that was never configured. */
    if ((motor->resistance_ohm <= 0.0f)
            || (motor->inductance_d_h <= 0.0f)
            || (motor->inductance_q_h <= 0.0f)) {
        return 0u;
    }

    float angular_bandwidth = TWO_PI * (float)CURRENTLOOP_BANDWIDTH_HZ;
    float sample_time_s     = 1.0f / (float)LOOP_RATE_HZ;

    /* Kp = L*wc, Ki = R*wc -- see GAIN COMPUTATION in currentloop.h.
     * Output limits are placeholders here; currentloop_step() sets the
     * real ones from the measured bus before the first pid_update(). */
    pid_init(&id_pid,
            motor->inductance_d_h * angular_bandwidth,
            motor->resistance_ohm * angular_bandwidth,
            0.0f, sample_time_s, 0.0f, 0.0f);
    pid_init(&iq_pid,
            motor->inductance_q_h * angular_bandwidth,
            motor->resistance_ohm * angular_bandwidth,
            0.0f, sample_time_s, 0.0f, 0.0f);

    target_id_a = (float)id_target_ma * MILLI_TO_UNIT;
    target_iq_a = (float)iq_target_ma * MILLI_TO_UNIT;

    /* Claim the loop's installed function now rather than at
     * currentloop_init() time, so starting this mode always takes over
     * from whatever mode (if any) was running before -- openloop.c does
     * the same in openloop_start(). */
    loop_set_function(currentloop_step);

    /* Closed-loop control needs the real rotor angle every period,
     * unlike open-loop drive -- see motor_set_reading_enabled() for why
     * this is off unless something asks for it. */
    motor_set_reading_enabled(motor, 1u);

    /* Every phase starts at the resting point, so enabling cannot apply
     * a leftover duty from a previous run. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_set_duty(phase, 500u);
    }

    /* Enabling charges each bootstrap capacitor in turn, blocking for a
     * few milliseconds per phase. Done before the loop starts producing
     * a real command so that the first one is actually deliverable. */
    for (uint8_t phase = 0u; phase < GATE_DRIVER_PHASE_COUNT; phase++) {
        gate_driver_enable_phase(phase);
    }

    running = 1u;
    return 1u;
}

void currentloop_set_targets(int32_t id_target_ma, int32_t iq_target_ma)
{
    target_id_a = (float)id_target_ma * MILLI_TO_UNIT;
    target_iq_a = (float)iq_target_ma * MILLI_TO_UNIT;
}

void currentloop_stop(void)
{
    running = 0u;
    gate_driver_disable_all();
}

uint8_t currentloop_is_running(void)
{
    return running;
}

void currentloop_get_state(int32_t *id_target_ma_out,
                           int32_t *iq_target_ma_out,
                           int32_t *id_measured_ma_out,
                           int32_t *iq_measured_ma_out)
{
    if (id_target_ma_out != NULL) {
        *id_target_ma_out = (int32_t)(target_id_a * 1000.0f);
    }
    if (iq_target_ma_out != NULL) {
        *iq_target_ma_out = (int32_t)(target_iq_a * 1000.0f);
    }
    if (id_measured_ma_out != NULL) {
        *id_measured_ma_out = (int32_t)(measured_id_a * 1000.0f);
    }
    if (iq_measured_ma_out != NULL) {
        *iq_measured_ma_out = (int32_t)(measured_iq_a * 1000.0f);
    }
}
