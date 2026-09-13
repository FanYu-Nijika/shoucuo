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
    float speed_kp;
    float speed_ki;
    float speed_filter;
    float speed_integral_limit;
    float speed_output_limit;
    float q_angle;
    float q_bias;
    float r_angle;
    int16_t pwm_limit;
    uint8_t speed_enabled;
    float target_speed; /* Sum of both wheels, counts per 10 ms. */
    int16_t turn_limit;
    int8_t turn_direction;
} balance_params_t;

typedef struct {
    float angle;
    float gyro_rate;
    float gyro_bias;
    float balance_output;
    float speed_output; /* Applied target-angle correction in degrees. */
    float target_angle;
    float speed_integral;
    float speed_error;
    float speed_filtered;
    float effective_target_speed;
    float turn_output;
    int32_t left_count;
    int32_t right_count;
    uint16_t control_period_ms;
    uint16_t speed_period_ms;
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
void balance_params_flash_save(uint8_t gear);
uint8_t balance_start(void);
uint8_t balance_seed_attitude(float angle);
void balance_stop(void);
uint8_t balance_apply_params(void);
uint8_t balance_params_are_valid(const balance_params_t *params);

/* Sample encoders on every tick, including a transient invalid IMU sample.
 * Counts in the state are full 20 ms increments; filtered speed is counts/10 ms. */
void balance_sample_encoders_5ms(int16_t left_count, int16_t right_count);
void balance_set_motion_target(float speed, float turn);
/* Count arguments retained for source compatibility; use the sampler above. */
void balance_update_5ms(float accel_angle, float gyro_rate, int16_t left_count, int16_t right_count, uint8_t valid);

#endif
