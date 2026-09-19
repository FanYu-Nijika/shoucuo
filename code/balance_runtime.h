#ifndef CODE_BALANCE_RUNTIME_H_
#define CODE_BALANCE_RUNTIME_H_

#include "zf_common_typedef.h"

#define BALANCE_IMU_WARMUP_SAMPLES  (40)
#define BALANCE_CAL_STATIC_SAMPLES  (100)
#define BALANCE_CALIBRATION_SAMPLES (1000)

typedef enum {
    BALANCE_IMU_CAL_REQUIRED = 0,
    BALANCE_IMU_WARMUP,
    BALANCE_IMU_WAIT_STATIC,
    BALANCE_IMU_COLLECTING,
    BALANCE_IMU_SAVING,
    BALANCE_IMU_READY,
    BALANCE_IMU_FAILED
} balance_imu_calibration_state_t;

typedef enum {
    BALANCE_CAL_ERROR_NONE = 0,
    BALANCE_CAL_ERROR_SAMPLES,
    BALANCE_CAL_ERROR_GRAVITY,
    BALANCE_CAL_ERROR_AXIS,
    BALANCE_CAL_ERROR_AXIS_CONFLICT,
    BALANCE_CAL_ERROR_START,
    BALANCE_CAL_ERROR_IMU,
    BALANCE_CAL_ERROR_FLASH
} balance_imu_calibration_error_t;

typedef enum {
    BALANCE_CAL_SOURCE_NONE = 0,
    BALANCE_CAL_SOURCE_FLASH,
    BALANCE_CAL_SOURCE_MANUAL
} balance_imu_calibration_source_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    uint16 static_samples;
    uint16 calibration_samples;
    uint16 calibration_valid_samples;
    uint32 invalid_sample_total;
    uint8 calibration_state;
    uint8 calibration_error;
    uint8 calibration_source;
    uint8 flash_calibration_valid;
    uint8 stationary;
    uint8 trim_active;
    uint8 vertical_axis;
    uint8 invalid_sample_streak;
    int8 vertical_sign;
} balance_imu_status_t;

extern volatile uint32 car_balance_max_us;
extern volatile uint32 car_balance_period_us;
extern volatile uint32 car_balance_overruns;
extern volatile uint32 car_uptime_ms;
extern volatile uint8 car_balance_timing_reset;
extern volatile uint8 balance_start_pending;
extern volatile balance_imu_status_t balance_imu_status;

void balance_runtime_init(void);
void balance_runtime_task(void);
void balance_runtime_request_start(void);
uint8 balance_runtime_request_calibration(void);
void balance_runtime_update_5ms(void);
void balance_runtime_stop(void);
void balance_runtime_set_motor_test(uint8 enabled);

#endif
