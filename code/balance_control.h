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
    int16_t left_count;
    int16_t right_count;
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
void balance_stop(void);
uint8_t balance_apply_params(void);
uint8_t balance_params_are_valid(const balance_params_t *params);

/* Counts are signed incremental encoder counts from one 5 ms sample. The
 * state count fields publish the completed two-sample (10 ms) sum. */
void balance_update_5ms(float accel_angle, float gyro_rate, int16_t left_count, int16_t right_count, uint8_t valid);

#endif
