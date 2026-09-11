#include "balance_runtime.h"

#include <math.h>

#include "balance_control.h"
#include "board_pins.h"
#include "car.h"
#include "car_menu.h"
#include "car_menu_port.h"
#include "qd_device_icm42688.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "zf_driver_timer.h"

volatile uint32 car_balance_max_us;
volatile uint32 car_balance_period_us;
volatile uint32 car_balance_overruns;
volatile uint32 car_uptime_ms;
volatile uint8 car_balance_timing_reset = 1;
volatile balance_imu_status_t balance_imu_status;

#define BALANCE_CALIBRATION_SAMPLES            (600)
#define BALANCE_CALIBRATION_VALID_MIN          (570)
#define BALANCE_CALIBRATION_GRAVITY_MIN        (0.7f)
#define BALANCE_CALIBRATION_GRAVITY_MAX        (1.3f)
#define BALANCE_CALIBRATION_AXIS_MIN           (0.65f)
#define BALANCE_CALIBRATION_AXIS_MARGIN        (0.25f)
#define BALANCE_CALIBRATION_ACCEL_VARIANCE_MAX (0.01f)
#define BALANCE_CALIBRATION_GYRO_VARIANCE_MAX  (25.0f)
#define BALANCE_IMU_INVALID_STREAK_LIMIT        (3)
#define BALANCE_IMU_RETRY_INTERVAL_MS           (500)

static uint8 imu_initialized;
static uint32 imu_retry_ms;
static float calibration_accel_sum[3];
static float calibration_accel_square_sum[3];
static float calibration_gyro_sum[3];
static float calibration_gyro_square_sum[3];

static void balance_runtime_reset_calibration(void)
{
    uint8 axis;

    for (axis = 0; axis < 3; axis++) {
        calibration_accel_sum[axis] = 0.0f;
        calibration_accel_square_sum[axis] = 0.0f;
        calibration_gyro_sum[axis] = 0.0f;
        calibration_gyro_square_sum[axis] = 0.0f;
    }
    balance_imu_status.accel_x = 0.0f;
    balance_imu_status.accel_y = 0.0f;
    balance_imu_status.accel_z = 0.0f;
    balance_imu_status.gyro_x = 0.0f;
    balance_imu_status.gyro_y = 0.0f;
    balance_imu_status.gyro_z = 0.0f;
    balance_imu_status.gyro_bias_x = 0.0f;
    balance_imu_status.gyro_bias_y = 0.0f;
    balance_imu_status.gyro_bias_z = 0.0f;
    balance_imu_status.calibration_samples = 0;
    balance_imu_status.calibration_valid_samples = 0;
    balance_imu_status.invalid_sample_total = 0;
    balance_imu_status.calibration_state = BALANCE_IMU_CALIBRATING;
    balance_imu_status.calibration_error = BALANCE_CAL_ERROR_NONE;
    balance_imu_status.calibration_quality = BALANCE_CAL_QUALITY_GOOD;
    balance_imu_status.vertical_axis = BOARD_BALANCE_ACCEL_VERTICAL_AXIS;
    balance_imu_status.invalid_sample_streak = 0;
    balance_imu_status.vertical_sign = BOARD_BALANCE_ACCEL_VERTICAL_SIGN;
    /* Calibration is a normal startup phase; CAL STATE already reports it, so it is not a fault. */
    balance_state.fault = BALANCE_FAULT_NONE;
}

static uint8 balance_runtime_probe_imu(void)
{
    uint8 model = 0;
    uint8 attempt;

    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPI_SPEED, ICM42688_SPC_PIN, ICM42688_SDI_PIN, ICM42688_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM42688_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    for (attempt = 0; attempt < 50; attempt++) {
        gpio_low(ICM42688_CS_PIN);
        spi_read_8bit_registers(ICM42688_SPI, 0x75 | 0x80, &model, 1);
        gpio_high(ICM42688_CS_PIN);
        if (model == 0x47) return 1;
        system_delay_ms(10);
    }
    return 0;
}

static uint8 balance_runtime_probe_imu_once(void)
{
    uint8 model = 0;

    gpio_low(ICM42688_CS_PIN);
    spi_read_8bit_registers(ICM42688_SPI, 0x75 | 0x80, &model, 1);
    gpio_high(ICM42688_CS_PIN);
    return model == 0x47 ? 1 : 0;
}

static uint8 balance_runtime_read_imu(float *angle, float *rate)
{
    float accel[3];
    float gyro[3];
    float accel_norm_square;

    if (imu_initialized == 0) return 0;
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    accel[0] = icm42688_acc_x;
    accel[1] = icm42688_acc_y;
    accel[2] = icm42688_acc_z;
    gyro[0] = icm42688_gyro_x;
    gyro[1] = icm42688_gyro_y;
    gyro[2] = icm42688_gyro_z;
    balance_imu_status.accel_x = accel[0];
    balance_imu_status.accel_y = accel[1];
    balance_imu_status.accel_z = accel[2];
    balance_imu_status.gyro_x = gyro[0];
    balance_imu_status.gyro_y = gyro[1];
    balance_imu_status.gyro_z = gyro[2];
    accel_norm_square = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
    if (!(accel_norm_square >= 0.04f && accel_norm_square <= 4.0f)) return 0;
    if (!(gyro[0] >= -2000.0f && gyro[0] <= 2000.0f && gyro[1] >= -2000.0f && gyro[1] <= 2000.0f &&
        gyro[2] >= -2000.0f && gyro[2] <= 2000.0f)) return 0;
    *angle = atan2f(BOARD_BALANCE_ACCEL_SIGN * accel[BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS],
        balance_imu_status.vertical_sign * accel[balance_imu_status.vertical_axis]) * 57.2957795f;
    gyro[0] -= balance_imu_status.gyro_bias_x;
    gyro[1] -= balance_imu_status.gyro_bias_y;
    gyro[2] -= balance_imu_status.gyro_bias_z;
    *rate = BOARD_BALANCE_GYRO_SIGN * gyro[BOARD_BALANCE_GYRO_AXIS];
    return 1;
}

static void balance_runtime_collect_calibration(uint8 valid)
{
    float accel[3];
    float gyro[3];
    uint8 axis;

    if (balance_imu_status.calibration_samples < BALANCE_CALIBRATION_SAMPLES) balance_imu_status.calibration_samples++;
    if (valid == 0) return;
    accel[0] = balance_imu_status.accel_x;
    accel[1] = balance_imu_status.accel_y;
    accel[2] = balance_imu_status.accel_z;
    gyro[0] = balance_imu_status.gyro_x;
    gyro[1] = balance_imu_status.gyro_y;
    gyro[2] = balance_imu_status.gyro_z;
    balance_imu_status.calibration_valid_samples++;
    for (axis = 0; axis < 3; axis++) {
        calibration_accel_sum[axis] += accel[axis];
        calibration_accel_square_sum[axis] += accel[axis] * accel[axis];
        calibration_gyro_sum[axis] += gyro[axis];
        calibration_gyro_square_sum[axis] += gyro[axis] * gyro[axis];
    }
}

static void balance_runtime_retry_calibration(void)
{
    uint8 axis;

    for (axis = 0; axis < 3; axis++) {
        calibration_accel_sum[axis] = 0.0f;
        calibration_accel_square_sum[axis] = 0.0f;
        calibration_gyro_sum[axis] = 0.0f;
        calibration_gyro_square_sum[axis] = 0.0f;
    }
    balance_imu_status.calibration_samples = 0;
    balance_imu_status.calibration_valid_samples = 0;
    balance_imu_status.calibration_state = BALANCE_IMU_CALIBRATING;
    balance_imu_status.calibration_error = BALANCE_CAL_ERROR_NONE;
    balance_imu_status.calibration_quality = BALANCE_CAL_QUALITY_GOOD;
    car_running = 1;
    balance_stop();
    balance_state.fault = BALANCE_FAULT_NONE;
    car_left_command = 0;
    car_right_command = 0;
    cc_tc264_menu_motor_stop();
}

static uint8 balance_runtime_finish_calibration(void)
{
    float accel_mean[3];
    float gyro_mean[3];
    float accel_variance;
    float gyro_variance;
    float gravity_norm_square = 0.0f;
    float dominant = 0.0f;
    float second = 0.0f;
    float magnitude;
    float sample_scale;
    float initial_angle;
    uint8 vertical_axis = 0;
    uint8 axis;

    if (balance_imu_status.calibration_valid_samples < BALANCE_CALIBRATION_VALID_MIN) {
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_SAMPLES;
        return 0;
    }
    sample_scale = 1.0f / balance_imu_status.calibration_valid_samples;
    for (axis = 0; axis < 3; axis++) {
        accel_mean[axis] = calibration_accel_sum[axis] * sample_scale;
        gyro_mean[axis] = calibration_gyro_sum[axis] * sample_scale;
        accel_variance = calibration_accel_square_sum[axis] * sample_scale - accel_mean[axis] * accel_mean[axis];
        gyro_variance = calibration_gyro_square_sum[axis] * sample_scale - gyro_mean[axis] * gyro_mean[axis];
        if (accel_variance > BALANCE_CALIBRATION_ACCEL_VARIANCE_MAX || gyro_variance > BALANCE_CALIBRATION_GYRO_VARIANCE_MAX) {
            balance_imu_status.calibration_quality = BALANCE_CAL_QUALITY_MOVING;
        }
        gravity_norm_square += accel_mean[axis] * accel_mean[axis];
        magnitude = fabsf(accel_mean[axis]);
        if (magnitude > dominant) {
            second = dominant;
            dominant = magnitude;
            vertical_axis = axis;
        } else if (magnitude > second) second = magnitude;
    }
    balance_imu_status.vertical_axis = vertical_axis;
    balance_imu_status.vertical_sign = accel_mean[vertical_axis] >= 0.0f ? 1 : -1;
    if (gravity_norm_square < BALANCE_CALIBRATION_GRAVITY_MIN * BALANCE_CALIBRATION_GRAVITY_MIN ||
        gravity_norm_square > BALANCE_CALIBRATION_GRAVITY_MAX * BALANCE_CALIBRATION_GRAVITY_MAX) {
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_GRAVITY;
        return 0;
    }
    if (dominant < BALANCE_CALIBRATION_AXIS_MIN || dominant - second < BALANCE_CALIBRATION_AXIS_MARGIN) {
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_AXIS;
        return 0;
    }
    if (vertical_axis == BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS || vertical_axis == BOARD_BALANCE_GYRO_AXIS ||
        BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS == BOARD_BALANCE_GYRO_AXIS) {
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_AXIS_CONFLICT;
        return 0;
    }

    balance_imu_status.gyro_bias_x = gyro_mean[0];
    balance_imu_status.gyro_bias_y = gyro_mean[1];
    balance_imu_status.gyro_bias_z = gyro_mean[2];
    balance_imu_status.calibration_state = BALANCE_IMU_READY;
    balance_imu_status.calibration_error = BALANCE_CAL_ERROR_NONE;
    initial_angle = atan2f(BOARD_BALANCE_ACCEL_SIGN * accel_mean[BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS],
        balance_imu_status.vertical_sign * accel_mean[vertical_axis]) * 57.2957795f;
    balance_update_5ms(initial_angle, 0.0f, 0, 0, 1);
    if (balance_start() == 0) {
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_START;
        return 0;
    }
    car_running = 1;
    car_balance_timing_reset = 1;
    return 1;
}

void balance_runtime_init(void)
{
    balance_init();
    balance_params_flash_load(car_menu_current_profile());
    cc_tc264_balance_encoder_init();
    imu_initialized = 0;
    imu_retry_ms = 0;
    balance_runtime_reset_calibration();
    if (balance_runtime_probe_imu() == 0) {
        car_running = 0;
        balance_update_5ms(0.0f, 0.0f, 0, 0, 0);
        balance_imu_status.calibration_state = BALANCE_IMU_FAILED;
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_IMU;
        cc_tc264_menu_motor_stop();
        return;
    }
    Init_ICM42688();
    imu_initialized = 1;
    car_running = 1;
    car_balance_timing_reset = 1;
    cc_tc264_menu_motor_stop();
}

void balance_runtime_task(void)
{
    uint32 now;

    if (imu_initialized != 0 || balance_imu_status.calibration_error != BALANCE_CAL_ERROR_IMU) return;
    now = system_getval_ms();
    if (now - imu_retry_ms < BALANCE_IMU_RETRY_INTERVAL_MS) return;
    imu_retry_ms = now;
    if (balance_runtime_probe_imu_once() == 0) return;

    Init_ICM42688();
    imu_initialized = 1;
    balance_runtime_reset_calibration();
    balance_state.fault = BALANCE_FAULT_NONE;
    car_running = 1;
    car_balance_timing_reset = 1;
}

void balance_runtime_request_start(void)
{
    if (imu_initialized == 0) {
        balance_imu_status.calibration_state = BALANCE_IMU_FAILED;
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_IMU;
        car_running = 0;
        return;
    }
    if (balance_imu_status.calibration_state != BALANCE_IMU_READY) {
        balance_runtime_reset_calibration();
        balance_state.fault = BALANCE_FAULT_NONE;
        car_running = 1;
        car_balance_timing_reset = 1;
        return;
    }
    car_running = balance_start();
    if (car_running != 0) car_balance_timing_reset = 1;
}

void balance_runtime_update_5ms(void)
{
    float angle = 0.0f;
    float rate = 0.0f;
    int16 left_count = 0;
    int16 right_count = 0;
    uint8 was_running = balance_state.running;
    uint8 valid;

    cc_tc264_balance_encoder_read(&left_count, &right_count);
    valid = balance_runtime_read_imu(&angle, &rate);
    if (balance_imu_status.calibration_state == BALANCE_IMU_CALIBRATING) {
        balance_runtime_collect_calibration(valid);
        car_left_command = 0;
        car_right_command = 0;
        cc_tc264_menu_motor_stop();
        if (balance_imu_status.calibration_samples >= BALANCE_CALIBRATION_SAMPLES &&
            balance_runtime_finish_calibration() == 0) balance_runtime_retry_calibration();
        return;
    }
    if (balance_imu_status.calibration_state != BALANCE_IMU_READY) {
        car_running = 0;
        car_left_command = 0;
        car_right_command = 0;
        cc_tc264_menu_motor_stop();
        return;
    }

    /* One bad SPI sample must not stop balancing. Keep the last command for at most 10 ms;
       a sustained failure stops safely, then the main loop reinitializes and recalibrates. */
    if (valid == 0) {
        if (balance_imu_status.invalid_sample_streak < 255) balance_imu_status.invalid_sample_streak++;
        balance_imu_status.invalid_sample_total++;
        if (balance_imu_status.invalid_sample_streak < BALANCE_IMU_INVALID_STREAK_LIMIT) return;
        balance_update_5ms(angle, rate, left_count, right_count, 0);
        imu_initialized = 0;
        imu_retry_ms = system_getval_ms();
        balance_imu_status.calibration_state = BALANCE_IMU_FAILED;
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_IMU;
        car_running = 0;
        car_left_command = 0;
        car_right_command = 0;
        cc_tc264_menu_motor_stop();
        return;
    } else {
        balance_imu_status.invalid_sample_streak = 0;
    }
    balance_update_5ms(angle, rate, left_count, right_count, valid);

    if (was_running != 0 && balance_state.running == 0) {
        car_running = 0;
    } else if (car_running != 0 && balance_state.running == 0) {
        car_running = balance_start();
        if (car_running != 0) car_balance_timing_reset = 1;
    } else if (car_running == 0 && balance_state.running != 0) {
        balance_stop();
    }

    car_left_command = balance_state.left_pwm;
    car_right_command = balance_state.right_pwm;
    if (balance_state.running != 0 && balance_state.output_updated != 0) {
        cc_tc264_menu_motor_write(car_left_command, car_right_command);
    } else if (balance_state.running == 0) {
        cc_tc264_menu_motor_stop();
    }
}

void balance_runtime_stop(void)
{
    if (balance_imu_status.calibration_state == BALANCE_IMU_CALIBRATING) {
        balance_imu_status.calibration_state = BALANCE_IMU_FAILED;
    }
    car_running = 0;
    balance_stop();
    car_left_command = 0;
    car_right_command = 0;
    cc_tc264_menu_motor_stop();
}
