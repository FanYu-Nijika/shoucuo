#ifndef CODE_BALANCE_CONTROL_H_
#define CODE_BALANCE_CONTROL_H_

#include <stdint.h>

typedef enum {
    BALANCE_FAULT_NONE = 0,
    BALANCE_FAULT_STOPPED,
    BALANCE_FAULT_IMU,
    BALANCE_FAULT_TILT,
    BALANCE_FAULT_NUMERIC,
    BALANCE_FAULT_TIMING,
    BALANCE_FAULT_CALIBRATION
} balance_fault_t;

typedef struct {
    float balance_kp;
    float balance_kd;
    float middle_angle;
    float accel_weight;
    float follow_speed;
    int16_t pwm_limit;
    int16_t turn_pwm_limit;
    int8_t turn_direction;
    /* Keep these slots for V8 Flash profiles; motor dead zones are not used by the balance loop. */
    uint16_t left_motor_deadzone;
    uint16_t right_motor_deadzone;
} balance_params_t;

typedef struct {
    float angle;
    float accel_angle;
    float gyro_rate;
    float angle_error;
    float p_output;
    float d_output;
    float balance_output;
    float common_pwm;
    float target_angle;
    float follow_angle_applied;
    float turn_pwm_command;
    float turn_output;
    int32_t left_count;
    int32_t right_count;
    uint16_t control_period_ms;
    int16_t left_control_pwm;
    int16_t right_control_pwm;
    int16_t left_pwm;
    int16_t right_pwm;
    uint8_t running;
    uint8_t fault;
    uint8_t imu_ready;
    uint8_t output_updated;
} balance_state_t;

extern balance_params_t balance_params;
extern volatile balance_state_t balance_state;
extern const balance_params_t balance_default_params;

void balance_init(void);
void balance_params_reset(void);
void balance_params_flash_load(uint8_t gear);
void balance_params_flash_switch(uint8_t gear);
uint8_t balance_params_flash_save(uint8_t gear);
uint8_t balance_start(void);
uint8_t balance_seed_attitude(float angle);
void balance_stop(void);
uint8_t balance_apply_params(void);
uint8_t balance_apply_live_params(const balance_params_t *params);
uint8_t balance_params_are_valid(const balance_params_t *params);

/* Encoders remain diagnostic-only. Four 5 ms samples form the displayed 20 ms count. */
void balance_sample_encoders_5ms(int16_t left_count, int16_t right_count);
/* FOLLOW speed is an open-loop -100..100 command, mapped to -5..5 degrees. */
void balance_set_follow_command(float follow_speed, float turn_pwm);
void balance_cancel_follow_command(void);
void balance_update_5ms(float accel_angle, float gyro_rate, uint8_t valid);

#endif
