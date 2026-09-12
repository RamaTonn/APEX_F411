#include "pid.h"

void pid_init(pid_t *p,
             float kp, float ki, float kd,
             float sample_time_s,
             float output_min, float output_max)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;

    p->sample_time_s = sample_time_s;

    p->output_min = output_min;
    p->output_max = output_max;

    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integrator           = 0.0f;
    p->previous_measurement = 0.0f;
}

void pid_set_gains(pid_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

void pid_set_output_limits(pid_t *p, float output_min, float output_max)
{
    p->output_min = output_min;
    p->output_max = output_max;
}

float pid_update(pid_t *p, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    float proportional = p->kp * error;

    /* Derivative on measurement -- see the note at the top of pid.h for
     * why this is not derivative on error. */
    float derivative = -p->kd * (measurement - p->previous_measurement)
                       / p->sample_time_s;
    p->previous_measurement = measurement;

    /* What the integrator would become if this step is committed. Used
     * to decide, below, whether committing it keeps the output inside
     * its limits. */
    float integrator_candidate = p->integrator
                                + (p->ki * error * p->sample_time_s);

    float output = proportional + integrator_candidate + derivative;

    /* Conditional integration: the new integrator value is only kept if
     * doing so does not push the output further past a limit it has
     * already reached. Otherwise the integrator holds at its previous
     * value. This is what stops it from winding up while saturated --
     * see the note at the top of pid.h for why that matters here. */
    if (output > p->output_max) {
        output = p->output_max;
        if (integrator_candidate < p->integrator) {
            p->integrator = integrator_candidate;
        }
    } else if (output < p->output_min) {
        output = p->output_min;
        if (integrator_candidate > p->integrator) {
            p->integrator = integrator_candidate;
        }
    } else {
        p->integrator = integrator_candidate;
    }

    return output;
}
