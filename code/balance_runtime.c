#include "balance_runtime.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "balance_control.h"
#include "board_pins.h"
#include "car.h"
#include "car_menu.h"
#include "car_menu_port.h"
#include "qd_device_icm42688.h"
#include "zf_driver_flash.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "zf_driver_timer.h"

volatile uint32 car_balance_max_us;
volatile uint32 car_balance_period_us;
volatile uint32 car_balance_overruns;
volatile uint32 car_uptime_ms;
volatile uint8 car_balance_timing_reset = 1;
volatile uint8 balance_start_pending;
volatile balance_imu_status_t balance_imu_status;

#define BALANCE_IMU_FLASH_SECTOR            (0)
#define BALANCE_IMU_FLASH_PAGE              (10)
#define BALANCE_IMU_FLASH_MAGIC             (0x494d5531u)
#define BALANCE_IMU_FLASH_VERSION           (1u)
#define BALANCE_CAL_GRAVITY_MIN             (0.7f)
#define BALANCE_CAL_GRAVITY_MAX             (1.3f)
#define BALANCE_CAL_GYRO_BIAS_MAX           (100.0f)
#define BALANCE_MANUAL_STATIC_ACCEL_MIN     (0.95f)
#define BALANCE_MANUAL_STATIC_ACCEL_MAX     (1.05f)
#define BALANCE_MANUAL_STATIC_GYRO_MAX      (5.0f)
#define BALANCE_TRIM_STATIC_GYRO_MAX        (1.5f)
#define BALANCE_TRIM_ALPHA                  (0.00040012f)
#define BALANCE_TRIM_WAIT_SAMPLES           (500)
#define BALANCE_IMU_INVALID_STREAK_LIMIT    (8)
#define BALANCE_IMU_RETRY_INTERVAL_MS       (500)

typedef struct {
    uint32 magic;
    uint32 version;
    float gyro_bias[3];
    uint8 vertical_axis;
    int8 vertical_sign;
    uint8 reserved[2];
    uint32 checksum;
} balance_imu_flash_record_t;

static uint8 imu_initialized;
static volatile uint8 motor_test_active;
static volatile uint8 calibration_save_pending;
static uint32 imu_retry_ms;
static uint16 trim_static_samples;
static float calibration_accel_sum[3];
static float calibration_gyro_sum[3];
static float warmup_angle_sum;
static float calibration_upright_pending;
static uint8 calibration_profile_pending;
static balance_params_t calibration_params_pending;

static uint8 balance_runtime_value_finite(float value)
{
    uint32 bits;

    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static uint32 balance_runtime_record_checksum(const balance_imu_flash_record_t *record)
{
    const uint8 *bytes = (const uint8 *)record;
    uint32 checksum = 2166136261u;
    uint32 index;

    for (index = 0; index < offsetof(balance_imu_flash_record_t, checksum); index++) {
        checksum ^= bytes[index];
        checksum *= 16777619u;
    }
    return checksum;
}

static uint8 balance_runtime_record_valid(const balance_imu_flash_record_t *record)
{
    uint8 axis;

    if (record->magic != BALANCE_IMU_FLASH_MAGIC || record->version != BALANCE_IMU_FLASH_VERSION) return 0;
    if (record->checksum != balance_runtime_record_checksum(record)) return 0;
    if (record->vertical_axis > 2 || (record->vertical_sign != -1 && record->vertical_sign != 1)) return 0;
    if (record->vertical_axis == BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS ||
        record->vertical_axis == BOARD_BALANCE_GYRO_AXIS ||
        BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS == BOARD_BALANCE_GYRO_AXIS) return 0;
    for (axis = 0; axis < 3; axis++) {
        if (!balance_runtime_value_finite(record->gyro_bias[axis]) ||
            fabsf(record->gyro_bias[axis]) > BALANCE_CAL_GYRO_BIAS_MAX) return 0;
    }
    return 1;
}

static void balance_runtime_clear_calibration_sums(void)
{
    uint8 axis;

    for (axis = 0; axis < 3; axis++) {
        calibration_accel_sum[axis] = 0.0f;
        calibration_gyro_sum[axis] = 0.0f;
    }
}

static void balance_runtime_reset_status(void)
{
    memset((void *)&balance_imu_status, 0, sizeof(balance_imu_status));
    balance_imu_status.calibration_state = BALANCE_IMU_CAL_REQUIRED;
    balance_imu_status.calibration_source = BALANCE_CAL_SOURCE_NONE;
    balance_imu_status.vertical_axis = BOARD_BALANCE_ACCEL_VERTICAL_AXIS;
    balance_imu_status.vertical_sign = BOARD_BALANCE_ACCEL_VERTICAL_SIGN;
    balance_imu_status.calibrated_upright_angle = balance_params.middle_angle;
    balance_imu_status.calibration_profile = car_menu_current_profile();
    calibration_upright_pending = balance_params.middle_angle;
    calibration_profile_pending = car_menu_current_profile();
    calibration_params_pending = balance_params;
    balance_runtime_clear_calibration_sums();
    warmup_angle_sum = 0.0f;
    trim_static_samples = 0;
}

static uint8 balance_runtime_load_calibration(void)
{
    balance_imu_flash_record_t record;

    flash_read_page_to_buffer(BALANCE_IMU_FLASH_SECTOR, BALANCE_IMU_FLASH_PAGE);
    memcpy(&record, &flash_union_buffer[0], sizeof(record));
    if (!balance_runtime_record_valid(&record)) {
        balance_imu_status.flash_calibration_valid = 0;
        return 0;
    }
    balance_imu_status.gyro_bias_x = record.gyro_bias[0];
    balance_imu_status.gyro_bias_y = record.gyro_bias[1];
    balance_imu_status.gyro_bias_z = record.gyro_bias[2];
    balance_imu_status.vertical_axis = record.vertical_axis;
    balance_imu_status.vertical_sign = record.vertical_sign;
    balance_imu_status.flash_calibration_valid = 1;
    balance_imu_status.calibration_source = BALANCE_CAL_SOURCE_FLASH;
    return 1;
}

static uint8 balance_runtime_save_calibration(void)
{
    balance_imu_flash_record_t record;
    balance_imu_flash_record_t verify;
    uint32 index;

    memset(&record, 0, sizeof(record));
    record.magic = BALANCE_IMU_FLASH_MAGIC;
    record.version = BALANCE_IMU_FLASH_VERSION;
    record.gyro_bias[0] = balance_imu_status.gyro_bias_x;
    record.gyro_bias[1] = balance_imu_status.gyro_bias_y;
    record.gyro_bias[2] = balance_imu_status.gyro_bias_z;
    record.vertical_axis = balance_imu_status.vertical_axis;
    record.vertical_sign = balance_imu_status.vertical_sign;
    record.checksum = balance_runtime_record_checksum(&record);
    if (!balance_runtime_record_valid(&record)) return 0;

    for (index = 0; index < EEPROM_PAGE_LENGTH; index++) flash_union_buffer[index].uint32_type = 0xffffffffu;
    memcpy(&flash_union_buffer[0], &record, sizeof(record));
    if (flash_write_page_from_buffer(BALANCE_IMU_FLASH_SECTOR, BALANCE_IMU_FLASH_PAGE) != 0) return 0;
    flash_read_page_to_buffer(BALANCE_IMU_FLASH_SECTOR, BALANCE_IMU_FLASH_PAGE);
    memcpy(&verify, &flash_union_buffer[0], sizeof(verify));
    return balance_runtime_record_valid(&verify);
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

static uint8 balance_runtime_read_imu(float *angle, float *rate, uint8 *accel_valid)
{
    float accel[3];
    float gyro[3];
    float accel_norm_square;

    *accel_valid = 0;
    if (!imu_initialized) return 0;
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
    if (!balance_runtime_value_finite(accel_norm_square)) return 0;
    if (!(gyro[0] >= -2000.0f && gyro[0] <= 2000.0f && gyro[1] >= -2000.0f && gyro[1] <= 2000.0f &&
        gyro[2] >= -2000.0f && gyro[2] <= 2000.0f)) return 0;
    gyro[0] -= balance_imu_status.gyro_bias_x;
    gyro[1] -= balance_imu_status.gyro_bias_y;
    gyro[2] -= balance_imu_status.gyro_bias_z;
    *rate = BOARD_BALANCE_GYRO_SIGN * gyro[BOARD_BALANCE_GYRO_AXIS];
    if (!balance_runtime_value_finite(*rate)) return 0;
    /* A bump can invalidate the gravity angle without breaking the gyro reading. */
    if (accel_norm_square < 0.04f || accel_norm_square > 4.0f) return 1;
    *angle = atan2f(BOARD_BALANCE_ACCEL_SIGN * accel[BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS],
        balance_imu_status.vertical_sign * accel[balance_imu_status.vertical_axis]) * 57.2957795f;
    if (!balance_runtime_value_finite(*angle)) return 0;
    *accel_valid = 1;
    return 1;
}

static uint8 balance_runtime_is_static(float gyro_limit)
{
    float accel_norm_square = balance_imu_status.accel_x * balance_imu_status.accel_x +
        balance_imu_status.accel_y * balance_imu_status.accel_y +
        balance_imu_status.accel_z * balance_imu_status.accel_z;
    float gyro_x = balance_imu_status.gyro_x - balance_imu_status.gyro_bias_x;
    float gyro_y = balance_imu_status.gyro_y - balance_imu_status.gyro_bias_y;
    float gyro_z = balance_imu_status.gyro_z - balance_imu_status.gyro_bias_z;
    float gyro_norm_square = gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z;

    if (accel_norm_square < BALANCE_MANUAL_STATIC_ACCEL_MIN * BALANCE_MANUAL_STATIC_ACCEL_MIN ||
        accel_norm_square > BALANCE_MANUAL_STATIC_ACCEL_MAX * BALANCE_MANUAL_STATIC_ACCEL_MAX) return 0;
    return gyro_norm_square < gyro_limit * gyro_limit ? 1 : 0;
}

static void balance_runtime_hold_stopped(void)
{
    car_running = 0;
    car_left_command = 0;
    car_right_command = 0;
    if (balance_state.running) balance_stop();
    cc_tc264_menu_motor_stop();
}

static void balance_runtime_begin_warmup(void)
{
    balance_runtime_hold_stopped();
    balance_state.imu_ready = 0;
    balance_imu_status.calibration_state = BALANCE_IMU_WARMUP;
    balance_imu_status.calibration_error = BALANCE_CAL_ERROR_NONE;
    balance_imu_status.calibration_source = BALANCE_CAL_SOURCE_FLASH;
    balance_imu_status.static_samples = 0;
    balance_imu_status.calibration_samples = 0;
    balance_imu_status.calibration_valid_samples = 0;
    balance_imu_status.stationary = 0;
    balance_imu_status.trim_active = 0;
    warmup_angle_sum = 0.0f;
    trim_static_samples = 0;
}

static void balance_runtime_set_cal_required(uint8 error)
{
    balance_runtime_hold_stopped();
    balance_state.imu_ready = 0;
    balance_imu_status.calibration_state = BALANCE_IMU_CAL_REQUIRED;
    balance_imu_status.calibration_error = error;
    balance_imu_status.calibration_source = BALANCE_CAL_SOURCE_NONE;
    if (error == BALANCE_CAL_ERROR_NONE) {
        balance_imu_status.static_samples = 0;
        balance_imu_status.calibration_samples = 0;
        balance_imu_status.calibration_valid_samples = 0;
    }
    balance_imu_status.stationary = 0;
    balance_imu_status.trim_active = 0;
    trim_static_samples = 0;
}

static void balance_runtime_reset_manual_wait(void)
{
    balance_runtime_clear_calibration_sums();
    balance_imu_status.calibration_state = BALANCE_IMU_WAIT_STATIC;
    balance_imu_status.calibration_error = BALANCE_CAL_ERROR_NONE;
    balance_imu_status.calibration_source = BALANCE_CAL_SOURCE_MANUAL;
    balance_imu_status.static_samples = 0;
    balance_imu_status.calibration_samples = 0;
    balance_imu_status.calibration_valid_samples = 0;
    balance_imu_status.stationary = 0;
    balance_imu_status.trim_active = 0;
    trim_static_samples = 0;
}

static void balance_runtime_collect_manual_sample(void)
{
    calibration_accel_sum[0] += balance_imu_status.accel_x;
    calibration_accel_sum[1] += balance_imu_status.accel_y;
    calibration_accel_sum[2] += balance_imu_status.accel_z;
    calibration_gyro_sum[0] += balance_imu_status.gyro_x;
    calibration_gyro_sum[1] += balance_imu_status.gyro_y;
    calibration_gyro_sum[2] += balance_imu_status.gyro_z;
    balance_imu_status.calibration_samples++;
    balance_imu_status.calibration_valid_samples++;
}

static uint8 balance_runtime_finish_manual_calibration(void)
{
    float accel_mean[3];
    float gyro_mean[3];
    float gravity_norm_square = 0.0f;
    float initial_angle;
    float scale;
    uint8 axis;

    if (balance_imu_status.calibration_valid_samples != BALANCE_CALIBRATION_SAMPLES) {
        balance_runtime_set_cal_required(BALANCE_CAL_ERROR_SAMPLES);
        return 0;
    }
    scale = 1.0f / BALANCE_CALIBRATION_SAMPLES;
    for (axis = 0; axis < 3; axis++) {
        accel_mean[axis] = calibration_accel_sum[axis] * scale;
        gyro_mean[axis] = calibration_gyro_sum[axis] * scale;
        gravity_norm_square += accel_mean[axis] * accel_mean[axis];
        if (!balance_runtime_value_finite(gyro_mean[axis]) || fabsf(gyro_mean[axis]) > BALANCE_CAL_GYRO_BIAS_MAX) {
            balance_runtime_set_cal_required(BALANCE_CAL_ERROR_SAMPLES);
            return 0;
        }
    }
    if (gravity_norm_square < BALANCE_CAL_GRAVITY_MIN * BALANCE_CAL_GRAVITY_MIN ||
        gravity_norm_square > BALANCE_CAL_GRAVITY_MAX * BALANCE_CAL_GRAVITY_MAX) {
        balance_runtime_set_cal_required(BALANCE_CAL_ERROR_GRAVITY);
        return 0;
    }
    if (BOARD_BALANCE_ACCEL_VERTICAL_AXIS == BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS ||
        BOARD_BALANCE_ACCEL_VERTICAL_AXIS == BOARD_BALANCE_GYRO_AXIS ||
        BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS == BOARD_BALANCE_GYRO_AXIS) {
        balance_runtime_set_cal_required(BALANCE_CAL_ERROR_AXIS_CONFLICT);
        return 0;
    }

    balance_imu_status.gyro_bias_x = gyro_mean[0];
    balance_imu_status.gyro_bias_y = gyro_mean[1];
    balance_imu_status.gyro_bias_z = gyro_mean[2];
    balance_imu_status.vertical_axis = BOARD_BALANCE_ACCEL_VERTICAL_AXIS;
    balance_imu_status.vertical_sign = BOARD_BALANCE_ACCEL_VERTICAL_SIGN;
    initial_angle = atan2f(BOARD_BALANCE_ACCEL_SIGN * accel_mean[BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS],
        BOARD_BALANCE_ACCEL_VERTICAL_SIGN * accel_mean[BOARD_BALANCE_ACCEL_VERTICAL_AXIS]) * 57.2957795f;
    if (!balance_runtime_value_finite(initial_angle)) {
        balance_runtime_set_cal_required(BALANCE_CAL_ERROR_START);
        return 0;
    }
    calibration_upright_pending = initial_angle;
    balance_imu_status.calibrated_upright_angle = initial_angle;
    balance_stop();
    if (!balance_seed_attitude(initial_angle)) {
        balance_runtime_set_cal_required(BALANCE_CAL_ERROR_START);
        return 0;
    }
    balance_imu_status.calibration_state = BALANCE_IMU_SAVING;
    balance_imu_status.calibration_error = BALANCE_CAL_ERROR_NONE;
    balance_imu_status.calibration_source = BALANCE_CAL_SOURCE_MANUAL;
    balance_imu_status.stationary = 1;
    calibration_save_pending = 1;
    return 1;
}

static void balance_runtime_update_warmup(float angle)
{
    warmup_angle_sum += angle;
    balance_imu_status.calibration_samples++;
    balance_imu_status.calibration_valid_samples++;
    if (balance_imu_status.calibration_valid_samples < BALANCE_IMU_WARMUP_SAMPLES) return;
    if (!balance_seed_attitude(warmup_angle_sum / BALANCE_IMU_WARMUP_SAMPLES)) {
        balance_runtime_set_cal_required(BALANCE_CAL_ERROR_START);
        return;
    }
    balance_imu_status.calibration_state = BALANCE_IMU_READY;
    balance_imu_status.calibration_error = BALANCE_CAL_ERROR_NONE;
    balance_imu_status.calibration_source = BALANCE_CAL_SOURCE_FLASH;
    balance_stop();
    car_balance_timing_reset = 1;
}

static void balance_runtime_update_manual_calibration(uint8 valid)
{
    uint8 stationary = valid && balance_runtime_is_static(BALANCE_MANUAL_STATIC_GYRO_MAX);

    balance_runtime_hold_stopped();
    balance_imu_status.stationary = stationary;
    if (balance_imu_status.calibration_state == BALANCE_IMU_WAIT_STATIC) {
        if (!stationary) {
            balance_imu_status.static_samples = 0;
            return;
        }
        if (++balance_imu_status.static_samples >= BALANCE_CAL_STATIC_SAMPLES) {
            balance_runtime_clear_calibration_sums();
            balance_imu_status.calibration_state = BALANCE_IMU_COLLECTING;
            balance_imu_status.calibration_samples = 0;
            balance_imu_status.calibration_valid_samples = 0;
        }
        return;
    }
    if (!stationary) {
        /* Any movement discards the whole window so gyro bias never includes a handling transient. */
        balance_runtime_reset_manual_wait();
        return;
    }
    balance_runtime_collect_manual_sample();
    if (balance_imu_status.calibration_samples >= BALANCE_CALIBRATION_SAMPLES) {
        balance_runtime_finish_manual_calibration();
    }
}

static void balance_runtime_update_bias_trim(void)
{
    if (balance_state.running || motor_test_active || !balance_imu_status.flash_calibration_valid) {
        trim_static_samples = 0;
        balance_imu_status.static_samples = 0;
        balance_imu_status.stationary = 0;
        balance_imu_status.trim_active = 0;
        return;
    }
    balance_imu_status.stationary = balance_runtime_is_static(BALANCE_TRIM_STATIC_GYRO_MAX);
    if (!balance_imu_status.stationary) {
        trim_static_samples = 0;
        balance_imu_status.static_samples = 0;
        balance_imu_status.trim_active = 0;
        return;
    }
    if (trim_static_samples < BALANCE_TRIM_WAIT_SAMPLES) trim_static_samples++;
    balance_imu_status.static_samples = trim_static_samples;
    if (trim_static_samples < BALANCE_TRIM_WAIT_SAMPLES) {
        balance_imu_status.trim_active = 0;
        return;
    }
    balance_imu_status.trim_active = 1;
    balance_imu_status.gyro_bias_x += BALANCE_TRIM_ALPHA *
        (balance_imu_status.gyro_x - balance_imu_status.gyro_bias_x);
    balance_imu_status.gyro_bias_y += BALANCE_TRIM_ALPHA *
        (balance_imu_status.gyro_y - balance_imu_status.gyro_bias_y);
    balance_imu_status.gyro_bias_z += BALANCE_TRIM_ALPHA *
        (balance_imu_status.gyro_z - balance_imu_status.gyro_bias_z);
}

static void balance_runtime_imu_failed(void)
{
    balance_update_2ms(0.0f, 0.0f, 0, 0);
    /* While balancing, retain the last motor command and resume on the next valid sample. */
    if (balance_state.running) return;
    imu_initialized = 0;
    imu_retry_ms = system_getval_ms();
    balance_imu_status.calibration_state = BALANCE_IMU_FAILED;
    balance_imu_status.calibration_error = BALANCE_CAL_ERROR_IMU;
    balance_imu_status.stationary = 0;
    balance_imu_status.trim_active = 0;
    balance_runtime_hold_stopped();
}

void balance_runtime_init(void)
{
    uint8 calibration_loaded;

    balance_init();
    motor_test_active = 0;
    calibration_save_pending = 0;
    balance_start_pending = 0;
    balance_params_flash_load(car_menu_current_profile());
    cc_tc264_balance_encoder_init();
    imu_initialized = 0;
    imu_retry_ms = 0;
    balance_runtime_reset_status();
    calibration_loaded = balance_runtime_load_calibration();
    if (!balance_runtime_probe_imu()) {
        balance_imu_status.calibration_state = BALANCE_IMU_FAILED;
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_IMU;
        balance_runtime_hold_stopped();
        return;
    }
    Init_ICM42688();
    imu_initialized = 1;
    if (calibration_loaded) balance_runtime_begin_warmup();
    else balance_runtime_set_cal_required(BALANCE_CAL_ERROR_NONE);
    car_balance_timing_reset = 1;
}

void balance_runtime_task(void)
{
    uint32 now;
    balance_params_t new_params;
    uint8 imu_saved;

    if (calibration_save_pending) {
        calibration_save_pending = 0;
        imu_saved = balance_runtime_save_calibration();
        new_params = calibration_params_pending;
        new_params.middle_angle = calibration_upright_pending;
        if (imu_saved && balance_apply_live_params(&new_params) && balance_params_flash_save(calibration_profile_pending)) {
            balance_imu_status.flash_calibration_valid = 1;
            balance_imu_status.calibration_state = BALANCE_IMU_READY;
            balance_imu_status.calibration_error = BALANCE_CAL_ERROR_NONE;
            balance_imu_status.calibration_source = BALANCE_CAL_SOURCE_MANUAL;
            balance_imu_status.calibrated_upright_angle = calibration_upright_pending;
            balance_imu_status.calibration_profile = calibration_profile_pending;
            balance_imu_status.static_samples = 0;
            balance_imu_status.trim_active = 0;
            balance_state.fault = BALANCE_FAULT_NONE;
            trim_static_samples = 0;
        } else {
            balance_imu_status.flash_calibration_valid = 0;
            balance_runtime_set_cal_required(BALANCE_CAL_ERROR_FLASH);
        }
    }
    if (motor_test_active || imu_initialized || balance_imu_status.calibration_error != BALANCE_CAL_ERROR_IMU) return;
    now = system_getval_ms();
    if (now - imu_retry_ms < BALANCE_IMU_RETRY_INTERVAL_MS) return;
    imu_retry_ms = now;
    if (!balance_runtime_probe_imu_once()) return;

    Init_ICM42688();
    imu_initialized = 1;
    balance_imu_status.invalid_sample_streak = 0;
    if (balance_runtime_load_calibration()) balance_runtime_begin_warmup();
    else balance_runtime_set_cal_required(BALANCE_CAL_ERROR_NONE);
    car_balance_timing_reset = 1;
}

void balance_runtime_request_start(void)
{
    if (motor_test_active) return;
    balance_start_pending = 1;
    if (!imu_initialized) {
        balance_imu_status.calibration_state = BALANCE_IMU_FAILED;
        balance_imu_status.calibration_error = BALANCE_CAL_ERROR_IMU;
        balance_state.fault = BALANCE_FAULT_IMU;
        return;
    }
    if (balance_imu_status.calibration_state != BALANCE_IMU_READY ||
        !balance_imu_status.flash_calibration_valid) {
        balance_state.fault = BALANCE_FAULT_CALIBRATION;
        return;
    }
}

uint8 balance_runtime_request_calibration(void)
{
    if (!imu_initialized || motor_test_active || balance_state.running || car_running) return 0;
    calibration_profile_pending = car_menu_current_profile();
    calibration_params_pending = balance_params;
    calibration_upright_pending = balance_params.middle_angle;
    balance_runtime_hold_stopped();
    balance_state.imu_ready = 0;
    balance_runtime_reset_manual_wait();
    balance_imu_status.calibration_profile = calibration_profile_pending;
    balance_imu_status.calibrated_upright_angle = calibration_upright_pending;
    balance_state.fault = BALANCE_FAULT_NONE;
    return 1;
}

void balance_runtime_update_2ms(void)
{
    float angle = 0.0f;
    float rate = 0.0f;
    int16 left_count = 0;
    int16 right_count = 0;
    uint8 was_running = balance_state.running;
    uint8 valid;
    uint8 accel_valid;

    cc_tc264_balance_encoder_read(&left_count, &right_count);
    balance_sample_encoders_2ms(left_count, right_count);
    if (motor_test_active) {
        balance_start_pending = 0;
        car_running = 0;
        if (balance_state.running) balance_stop();
        car_left_command = 0;
        car_right_command = 0;
        trim_static_samples = 0;
        balance_imu_status.static_samples = 0;
        balance_imu_status.stationary = 0;
        balance_imu_status.trim_active = 0;
        return;
    }
    if (!imu_initialized) {
        balance_runtime_hold_stopped();
        return;
    }
    valid = balance_runtime_read_imu(&angle, &rate, &accel_valid);
    if (!valid) {
        if (balance_imu_status.invalid_sample_streak < 255) balance_imu_status.invalid_sample_streak++;
        balance_imu_status.invalid_sample_total++;
        if (balance_imu_status.calibration_state == BALANCE_IMU_WAIT_STATIC ||
            balance_imu_status.calibration_state == BALANCE_IMU_COLLECTING) {
            balance_runtime_update_manual_calibration(0);
        }
        if (balance_imu_status.invalid_sample_streak >= BALANCE_IMU_INVALID_STREAK_LIMIT) balance_runtime_imu_failed();
        return;
    }
    balance_imu_status.invalid_sample_streak = 0;

    if (balance_imu_status.calibration_state == BALANCE_IMU_WARMUP) {
        balance_runtime_hold_stopped();
        if (accel_valid) balance_runtime_update_warmup(angle);
        return;
    }
    if (balance_imu_status.calibration_state == BALANCE_IMU_WAIT_STATIC ||
        balance_imu_status.calibration_state == BALANCE_IMU_COLLECTING) {
        balance_runtime_update_manual_calibration(accel_valid);
        return;
    }
    if (balance_imu_status.calibration_state != BALANCE_IMU_READY) {
        balance_imu_status.stationary = balance_runtime_is_static(BALANCE_MANUAL_STATIC_GYRO_MAX);
        balance_runtime_hold_stopped();
        return;
    }

    balance_runtime_update_bias_trim();
    balance_update_2ms(angle, rate, 1, accel_valid);
    /* WAIT_DELAY must keep the motors off; only request RUN after the two-second stop. */
    if (car_follow_status == CAR_FOLLOW_WAIT_RUN && !balance_state.running) balance_start_pending = 1;
    if (balance_start_pending && !balance_state.running) {
        balance_start_pending = 0;
        /* Warmup seeds the filter once; while stopped it keeps tracking attitude, so RUN must not replace it with one noisy frame. */
        car_running = balance_start();
        if (car_running) car_balance_timing_reset = 1;
    }
    if (was_running && !balance_state.running) {
        car_running = 0;
    } else if (!car_running && balance_state.running) balance_stop();

    car_left_command = balance_state.left_pwm;
    car_right_command = balance_state.right_pwm;
    if (balance_state.running && balance_state.output_updated) {
        cc_tc264_menu_motor_write(car_left_command, car_right_command);
    } else if (!balance_state.running) cc_tc264_menu_motor_stop();
}

void balance_runtime_stop(void)
{
    car_follow_stop();
    balance_start_pending = 0;
    car_running = 0;
    balance_stop();
    car_left_command = 0;
    car_right_command = 0;
    cc_tc264_menu_motor_stop();
}

void balance_runtime_set_motor_test(uint8 enabled)
{
    motor_test_active = enabled ? 1 : 0;
    if (!motor_test_active) return;
    if (balance_imu_status.calibration_state == BALANCE_IMU_WAIT_STATIC ||
        balance_imu_status.calibration_state == BALANCE_IMU_COLLECTING) balance_runtime_reset_manual_wait();
    if (balance_imu_status.calibration_state == BALANCE_IMU_WARMUP) {
        warmup_angle_sum = 0.0f;
        balance_imu_status.calibration_samples = 0;
        balance_imu_status.calibration_valid_samples = 0;
    }
    balance_start_pending = 0;
    car_running = 0;
    balance_stop();
    car_left_command = 0;
    car_right_command = 0;
    cc_tc264_menu_motor_stop();
}
