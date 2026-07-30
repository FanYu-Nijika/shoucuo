#include "pid.h"

#include "math_utils.h"

void cc_pid_init(cc_pid_t *pid, float kp, float ki, float kd,
                 float output_min, float output_max)
{
    if (pid == 0) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = output_min;
    pid->integral_max = output_max;
    cc_pid_reset(pid);
}

void cc_pid_set_integral_limits(cc_pid_t *pid, float integral_min, float integral_max)
{
    if (pid == 0) return;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->integral = cc_math_clamp_f32(pid->integral, integral_min, integral_max);
}

void cc_pid_reset(cc_pid_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0;
    pid->previous_error = 0.0;
    pid->last_output = 0.0;
    pid->initialized = 0;
}

float cc_pid_update(cc_pid_t *pid, float setpoint, float measurement, float dt)
{
    return cc_pid_update_error(pid, setpoint - measurement, dt);
}

float cc_pid_update_error(cc_pid_t *pid, float error, float dt)
{
    float derivative = 0.0;
    float output;
    if (pid == 0) return 0.0;
    if (dt <= 0.0) return pid->last_output;
    if (pid->initialized != 0) derivative = (error - pid->previous_error) / dt;
    pid->integral += error * dt;
    pid->integral = cc_math_clamp_f32(pid->integral, pid->integral_min, pid->integral_max);
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = cc_math_clamp_f32(output, pid->output_min, pid->output_max);
    pid->previous_error = error;
    pid->last_output = output;
    pid->initialized = 1;
    return output;
}

float cc_pid_output(const cc_pid_t *pid)
{
    return pid == 0 ? 0.0 : pid->last_output;
}

