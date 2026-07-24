#include "pid.h"

#include "math_utils.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_pid_init 功能实现
// 返回类型          void
// 使用示例          cc_pid_init(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_pid_set_integral_limits 功能实现
// 返回类型          void
// 使用示例          cc_pid_set_integral_limits(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_pid_set_integral_limits(cc_pid_t *pid, float integral_min, float integral_max)
{
    if (pid == 0) return;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->integral = cc_math_clamp_f32(pid->integral, integral_min, integral_max);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_pid_reset 功能实现
// 返回类型          void
// 使用示例          cc_pid_reset(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_pid_reset(cc_pid_t *pid)
{
    if (pid == 0) return;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->last_output = 0.0f;
    pid->initialized = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_pid_update 功能实现
// 返回类型          float
// 使用示例          cc_pid_update(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_pid_update(cc_pid_t *pid, float setpoint, float measurement, float dt)
{
    return cc_pid_update_error(pid, setpoint - measurement, dt);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_pid_update_error 功能实现
// 返回类型          float
// 使用示例          cc_pid_update_error(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_pid_update_error(cc_pid_t *pid, float error, float dt)
{
    float derivative = 0.0f;
    float output;
    if (pid == 0) return 0.0f;
    if (dt <= 0.0f) return pid->last_output;
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_pid_output 功能实现
// 返回类型          float
// 使用示例          cc_pid_output(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_pid_output(const cc_pid_t *pid)
{
    return pid == 0 ? 0.0f : pid->last_output;
}

