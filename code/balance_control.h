#ifndef CODE_BALANCE_CONTROL_H_
#define CODE_BALANCE_CONTROL_H_

#include <stdint.h>

#define BALANCE_CONTROL_PERIOD_MS (2)

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
    float accel_weight; /* Menu/Flash weight remains referenced to 5 ms; applying parameters converts it to 2 ms. */
    float target_speed_mps;
    int16_t pwm_limit; /* Legacy Flash slot; upright output no longer uses this limit. */
    int16_t turn_pwm_limit;
    int8_t turn_direction;
    /* Keep these slots for V8 Flash profiles; motor dead zones are not used by the balance loop. */
    uint16_t left_motor_deadzone;
    uint16_t right_motor_deadzone;
    float speed_kp; /* Degrees per (m/s). */
    float speed_ki; /* Degrees per (m/s) per second, not a gain per PI update. */
    float speed_angle_limit;
    uint8_t speed_enabled;
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
    float speed_target_mps;
    float speed_target_applied_mps;
    float speed_raw_mps;
    float speed_filtered_mps;
    float speed_feedback_mps; /* Mean of ten 2 ms speeds, held from the latest PI calculation. */
    float speed_error_mps;
    float speed_p_angle;
    float speed_integral_angle;
    float speed_angle_applied;
    float turn_pwm_command;
    float turn_output;
    int32_t left_count; /* Latest signed 2 ms count, also used for physical velocity. */
    int32_t right_count;
    int32_t encoder_average_count; /* One coherent signed 20 ms mean for CPU1; no wheel calibration required. */
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    uint16_t control_period_ms;
    int16_t left_control_pwm;
    int16_t right_control_pwm;
    int16_t left_pwm;
    int16_t right_pwm;
    uint8_t running;
    uint8_t speed_pid_active;
    uint8_t fault;
    uint8_t imu_ready;
    uint8_t output_updated;
    uint8_t encoder_calibration_valid;
    uint8_t encoder_calibration_state;
} balance_state_t;

enum {
    BALANCE_ENCODER_CAL_IDLE = 0,
    BALANCE_ENCODER_CAL_LEFT,
    BALANCE_ENCODER_CAL_RIGHT,
    BALANCE_ENCODER_CAL_SAVED,
    BALANCE_ENCODER_CAL_ERROR
};

extern balance_params_t balance_params;
extern volatile balance_state_t balance_state;
extern const balance_params_t balance_default_params;

void balance_init(void);
void balance_params_reset(void);
void balance_params_flash_load(uint8_t gear);
void balance_params_flash_switch(uint8_t gear);
uint8_t balance_params_flash_save(uint8_t gear);
uint8_t balance_params_flash_copy(uint8_t source, uint8_t target);
uint8_t balance_start(void);
uint8_t balance_seed_attitude(float angle);
void balance_stop(void);
uint8_t balance_apply_params(void);
uint8_t balance_apply_live_params(const balance_params_t *params);
uint8_t balance_params_are_valid(const balance_params_t *params);

/* Measure velocity every 2 ms; update speed PI and the vision count window every ten samples. */
void balance_sample_encoders_2ms(int16_t left_count, int16_t right_count);
/* RUN and the FOLLOW upright hold leave speed PI off; tracking enables it separately from SPEED ENABLE. */
void balance_set_speed_control_active(uint8_t active);
/* Blend the launch target with the normal speed-loop target without a reference-angle step. */
void balance_set_follow_command(float target_speed_mps, float turn_pwm, float lean_angle, float lean_weight);
void balance_cancel_follow_command(void);
uint8_t balance_encoder_calibrate_toggle(uint8_t wheel);
void balance_update_2ms(float accel_angle, float gyro_rate, uint8_t valid, uint8_t accel_valid);

#endif
