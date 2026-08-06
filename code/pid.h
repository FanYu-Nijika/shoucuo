#ifndef CC_PID_H
#define CC_PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float integral;
    float previous_error;
    float last_output;
    uint8_t initialized;
} cc_pid_t;

void cc_pid_init(cc_pid_t *pid, float kp, float ki, float kd,
                 float output_min, float output_max);
void cc_pid_set_integral_limits(cc_pid_t *pid, float integral_min, float integral_max);
void cc_pid_reset(cc_pid_t *pid);
float cc_pid_update(cc_pid_t *pid, float setpoint, float measurement, float dt);
float cc_pid_update_error(cc_pid_t *pid, float error, float dt);
float cc_pid_output(const cc_pid_t *pid);

#endif

