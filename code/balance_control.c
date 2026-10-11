#include "balance_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "board_pins.h"
#include "zf_common_interrupt.h"
#include "zf_driver_flash.h"

static const float balance_dt_s = BALANCE_CONTROL_PERIOD_MS * 0.001f;
/* The driver defines 10000 as 100% duty; clamp before narrowing to int16_t. */
static const float balance_motor_pwm_max = 10000.0f;
static const float speed_dt_s = 0.02f;
static const float wheel_circumference_m = 0.065f * 3.14159265f;
/* Favor the current 2 ms measurement so speed feedback responds promptly. */
static const float speed_filter_weight = 0.9f;

#define BALANCE_PARAMS_FLASH_SECTOR        (0)
#define BALANCE_PARAMS_FLASH_PAGE          (9)
#define BALANCE_PARAMS_FLASH_MAGIC         (0x42414C31u)
#define BALANCE_PARAMS_FLASH_VERSION       (10u)
#define BALANCE_PARAMS_FLASH_PROFILE_COUNT (4u)
#define BALANCE_PARAMS_FLASH_PROFILE_START (2u)
#define BALANCE_PARAMS_FLASH_PROFILE_WORDS ((sizeof(balance_params_t) + 3u) / 4u)
#define BALANCE_PARAMS_V1_WORDS            (12u)
#define BALANCE_PARAMS_V4_WORDS            (14u)
#define BALANCE_PARAMS_V5_WORDS            (15u)
#define BALANCE_PARAMS_V7_WORDS            (16u)
#define BALANCE_ENCODER_FLASH_PAGE         (11u)
#define BALANCE_ENCODER_FLASH_MAGIC        (0x454E4331u)
#define BALANCE_ENCODER_FLASH_VERSION      (2u)
#define BALANCE_ENCODER_MAX_COUNTS         (1000000u)

/* Fixed V7 layout is kept only for migration. Removed speed/Kalman/rate-limit fields never enter active control. */
typedef struct {
    float angle_kp;
    float rate_kp;
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
    float target_speed;
    int16_t turn_limit;
    int8_t turn_direction;
    uint16_t left_motor_deadzone;
    uint16_t right_motor_deadzone;
    float rate_limit;
} balance_params_v7_t;

typedef struct {
    float balance_kp;
    float balance_kd;
    float middle_angle;
    float accel_weight;
    float follow_angle;
    int16_t pwm_limit;
    int16_t turn_pwm_limit;
    int8_t turn_direction;
    uint16_t left_motor_deadzone;
    uint16_t right_motor_deadzone;
} balance_params_v8_t;

typedef struct {
    float balance_kp;
    float balance_kd;
    float middle_angle;
    float accel_weight;
    float follow_speed;
    int16_t pwm_limit;
    int16_t turn_pwm_limit;
    int8_t turn_direction;
    uint16_t left_motor_deadzone;
    uint16_t right_motor_deadzone;
} balance_params_v9_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t left_counts_per_rev;
    uint32_t right_counts_per_rev;
    uint32_t checksum;
} balance_encoder_flash_t;

typedef char balance_v7_target_check[offsetof(balance_params_v7_t, target_speed) == 48 ? 1 : -1];
typedef char balance_v7_deadzone_check[offsetof(balance_params_v7_t, left_motor_deadzone) == 56 ? 1 : -1];
typedef char balance_v7_size_check[sizeof(balance_params_v7_t) == 64 ? 1 : -1];
typedef char balance_v8_size_check[sizeof(balance_params_v8_t) == sizeof(balance_params_v9_t) ? 1 : -1];
typedef char balance_v9_target_check[offsetof(balance_params_v9_t, follow_speed) == offsetof(balance_params_t, target_speed_mps) ? 1 : -1];
typedef char balance_flash_capacity_check[2 + 4 * BALANCE_PARAMS_FLASH_PROFILE_WORDS <= EEPROM_PAGE_LENGTH ? 1 : -1];

const balance_params_t balance_default_params = {
    .balance_kp = 1000.0f,
    .balance_kd = 25.0556f,
    .middle_angle = 0.0f,
    .accel_weight = 0.01f,
    .target_speed_mps = 1.5f,
    .pwm_limit = 5000,
    .turn_pwm_limit = 1400,
    .turn_direction = 1,
    .left_motor_deadzone = 1100,
    .right_motor_deadzone = 600,
    .speed_kp = 2.0f,
    .speed_ki = 0.3f,
    .speed_angle_limit = 5.0f,
    .speed_enabled = 1
};

static const balance_params_v7_t balance_v7_defaults = {
    .angle_kp = 1000.0f,
    .rate_kp = 25.0556f,
    .middle_angle = 0.0f,
    .speed_kp = 0.004f,
    .speed_ki = 0.00002f,
    .speed_filter = 0.16f,
    .speed_integral_limit = 30000.0f,
    .speed_output_limit = 1.5f,
    .q_angle = 0.001f,
    .q_bias = 0.003f,
    .r_angle = 0.5f,
    .pwm_limit = 5000,
    .speed_enabled = 0,
    .target_speed = 0.0f,
    .turn_limit = 100,
    .turn_direction = 1,
    .left_motor_deadzone = 1100,
    .right_motor_deadzone = 600,
    .rate_limit = 200.0f
};

balance_params_t balance_params;
volatile balance_state_t balance_state;
static balance_params_t applied_params;
static uint8_t complementary_initialized;
static int32_t left_count_sum;
static int32_t right_count_sum;
static uint32_t left_count_total;
static uint32_t right_count_total;
static uint32_t encoder_calibration_start_count;
static uint8_t encoder_sample_phase;
static volatile uint8_t speed_control_active;
static float requested_speed_mps;
static float requested_turn_pwm;
static float requested_lean_angle;
static float requested_lean_weight;

static uint8_t balance_is_finite(float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static float balance_clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float balance_accel_weight_2ms(float weight_5ms)
{
    /* Preserve the filter time constant for existing Flash profiles; run powf only when applying parameters. */
    return 1.0f - powf(1.0f - weight_5ms, balance_dt_s / 0.005f);
}

static uint8_t balance_params_profile_valid(uint8_t gear)
{
    return gear >= 1 && gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT ? 1 : 0;
}

static uint32_t balance_params_profile_offset(uint8_t gear)
{
    return BALANCE_PARAMS_FLASH_PROFILE_START + (gear - 1u) * BALANCE_PARAMS_FLASH_PROFILE_WORDS;
}

static uint32_t balance_v7_profile_offset(uint8_t gear, uint32_t version)
{
    uint32_t words = BALANCE_PARAMS_V7_WORDS;

    if (version < 3u) words = BALANCE_PARAMS_V1_WORDS;
    else if (version < 5u) words = BALANCE_PARAMS_V4_WORDS;
    else if (version < 6u) words = BALANCE_PARAMS_V5_WORDS;
    return BALANCE_PARAMS_FLASH_PROFILE_START + (gear - 1u) * words;
}

uint8_t balance_params_are_valid(const balance_params_t *params)
{
    if (params == 0) return 0;
    if (!balance_is_finite(params->balance_kp) || !balance_is_finite(params->balance_kd) ||
        !balance_is_finite(params->middle_angle) || !balance_is_finite(params->accel_weight) ||
        !balance_is_finite(params->target_speed_mps) || !balance_is_finite(params->speed_kp) ||
        !balance_is_finite(params->speed_ki) || !balance_is_finite(params->speed_angle_limit)) return 0;
    /* Signed gains allow direction tuning; finite values and output limits are checked separately. */
    if (fabsf(params->middle_angle) > 180.0f ||
        params->accel_weight < 0.0f || params->accel_weight > 1.0f ||
        params->target_speed_mps < 0.0f || params->target_speed_mps > 3.0f ||
        params->speed_angle_limit < 0.0f || params->speed_angle_limit > 20.0f || params->speed_enabled > 1u ||
        params->pwm_limit < 1 || params->pwm_limit > 10000 ||
        params->turn_pwm_limit < 0 || params->turn_pwm_limit > 3000 ||
        (params->turn_direction != -1 && params->turn_direction != 1) ||
        params->left_motor_deadzone > 3000 || params->right_motor_deadzone > 3000) return 0;
    return 1;
}

static balance_params_t balance_params_from_v7(const balance_params_v7_t *old, uint32_t version)
{
    balance_params_t migrated = balance_default_params;
    float old_kp = old->angle_kp;
    float old_kd = old->rate_kp;

    if (version == 6u && balance_is_finite(old_kp) && balance_is_finite(old_kd)) old_kp *= old_kd;
    if (balance_is_finite(old_kp)) migrated.balance_kp = old_kp;
    if (balance_is_finite(old_kd)) migrated.balance_kd = old_kd;
    if (balance_is_finite(old->middle_angle) && fabsf(old->middle_angle) <= 180.0f) migrated.middle_angle = old->middle_angle;
    if (old->pwm_limit >= 1 && old->pwm_limit <= 10000) migrated.pwm_limit = old->pwm_limit;
    if (old->turn_direction == -1 || old->turn_direction == 1) migrated.turn_direction = old->turn_direction;
    if (old->left_motor_deadzone <= 3000) migrated.left_motor_deadzone = old->left_motor_deadzone;
    if (old->right_motor_deadzone <= 3000) migrated.right_motor_deadzone = old->right_motor_deadzone;
    return migrated;
}

static balance_params_t balance_params_from_v9(const balance_params_v9_t *old)
{
    balance_params_t migrated = balance_default_params;

    if (balance_is_finite(old->balance_kp)) migrated.balance_kp = old->balance_kp;
    if (balance_is_finite(old->balance_kd)) migrated.balance_kd = old->balance_kd;
    if (balance_is_finite(old->middle_angle) && fabsf(old->middle_angle) <= 180.0f) migrated.middle_angle = old->middle_angle;
    if (balance_is_finite(old->accel_weight) && old->accel_weight >= 0.0f && old->accel_weight <= 1.0f)
        migrated.accel_weight = old->accel_weight;
    if (old->pwm_limit >= 1 && old->pwm_limit <= 10000) migrated.pwm_limit = old->pwm_limit;
    if (old->turn_pwm_limit >= 0 && old->turn_pwm_limit <= 3000) migrated.turn_pwm_limit = old->turn_pwm_limit;
    if (old->turn_direction == -1 || old->turn_direction == 1) migrated.turn_direction = old->turn_direction;
    if (old->left_motor_deadzone <= 3000) migrated.left_motor_deadzone = old->left_motor_deadzone;
    if (old->right_motor_deadzone <= 3000) migrated.right_motor_deadzone = old->right_motor_deadzone;
    /* The old FOLLOW SPEED was an angle command, not a physical velocity. */
    return migrated;
}

/* Build a complete V10 page in RAM. Only saving or copying writes the migration. */
static void balance_params_flash_prepare(void)
{
    balance_params_t profiles[BALANCE_PARAMS_FLASH_PROFILE_COUNT];
    balance_params_v7_t old_profiles[BALANCE_PARAMS_FLASH_PROFILE_COUNT];
    balance_params_v9_t old_v9;
    uint32_t version = flash_union_buffer[1].uint32_type;
    uint32_t offset;
    uint32_t copy_bytes;
    uint32_t index;
    uint8_t gear;

    for (gear = 0; gear < BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear++) profiles[gear] = balance_default_params;
    if (flash_union_buffer[0].uint32_type == BALANCE_PARAMS_FLASH_MAGIC && version == BALANCE_PARAMS_FLASH_VERSION) {
        for (gear = 1; gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear++) {
            offset = balance_params_profile_offset(gear);
            memcpy(&profiles[gear - 1u], &flash_union_buffer[offset], sizeof(balance_params_t));
            if (!balance_params_are_valid(&profiles[gear - 1u])) profiles[gear - 1u] = balance_default_params;
        }
    } else if (flash_union_buffer[0].uint32_type == BALANCE_PARAMS_FLASH_MAGIC && (version == 8u || version == 9u)) {
        for (gear = 1; gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear++) {
            offset = BALANCE_PARAMS_FLASH_PROFILE_START + (gear - 1u) * (sizeof(balance_params_v9_t) / 4u);
            memcpy(&old_v9, &flash_union_buffer[offset], sizeof(old_v9));
            profiles[gear - 1u] = balance_params_from_v9(&old_v9);
            if (!balance_params_are_valid(&profiles[gear - 1u])) profiles[gear - 1u] = balance_default_params;
        }
    } else if (flash_union_buffer[0].uint32_type == BALANCE_PARAMS_FLASH_MAGIC && version >= 1u && version <= 7u) {
        if (version < 3u) copy_bytes = BALANCE_PARAMS_V1_WORDS * 4u;
        else if (version < 5u) copy_bytes = BALANCE_PARAMS_V4_WORDS * 4u;
        else if (version < 6u) copy_bytes = BALANCE_PARAMS_V5_WORDS * 4u;
        else copy_bytes = sizeof(balance_params_v7_t);
        for (gear = 1; gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear++) {
            old_profiles[gear - 1u] = balance_v7_defaults;
            offset = balance_v7_profile_offset(gear, version);
            memcpy(&old_profiles[gear - 1u], &flash_union_buffer[offset], copy_bytes);
        }
        for (gear = 1; gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear++) {
            profiles[gear - 1u] = balance_params_from_v7(&old_profiles[gear - 1u], version);
        }
    }

    for (index = 0; index < EEPROM_PAGE_LENGTH; index++) flash_union_buffer[index].uint32_type = 0xffffffffu;
    flash_union_buffer[0].uint32_type = BALANCE_PARAMS_FLASH_MAGIC;
    flash_union_buffer[1].uint32_type = BALANCE_PARAMS_FLASH_VERSION;
    for (gear = 1; gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear++) {
        offset = balance_params_profile_offset(gear);
        memcpy(&flash_union_buffer[offset], &profiles[gear - 1u], sizeof(balance_params_t));
    }
}

static void balance_reset_speed_controller(void)
{
    speed_control_active = 0;
    requested_speed_mps = 0.0f;
    balance_state.speed_target_mps = 0.0f;
    balance_state.speed_target_applied_mps = 0.0f;
    balance_state.speed_feedback_mps = 0.0f;
    balance_state.speed_error_mps = 0.0f;
    balance_state.speed_p_angle = 0.0f;
    balance_state.speed_integral_angle = 0.0f;
    balance_state.speed_pid_active = 0;
    balance_state.speed_angle_applied = 0.0f;
    balance_state.target_angle = applied_params.middle_angle;
}

void balance_cancel_follow_command(void)
{
    requested_speed_mps = 0.0f;
    requested_turn_pwm = 0.0f;
    requested_lean_angle = 0.0f;
    requested_lean_weight = 0.0f;
    balance_state.speed_target_mps = 0.0f;
    balance_state.turn_pwm_command = 0.0f;
    balance_state.turn_output = 0.0f;
    balance_reset_speed_controller();
}

static void balance_clear_outputs(void)
{
    balance_cancel_follow_command();
    balance_state.angle_error = 0.0f;
    balance_state.p_output = 0.0f;
    balance_state.d_output = 0.0f;
    balance_state.balance_output = 0.0f;
    balance_state.common_pwm = 0.0f;
    balance_state.left_control_pwm = 0;
    balance_state.right_control_pwm = 0;
    balance_state.left_pwm = 0;
    balance_state.right_pwm = 0;
    balance_state.output_updated = 0;
}

static int16_t balance_pwm_from_float(float value)
{
    if (value >= balance_motor_pwm_max) return 10000;
    if (value <= -balance_motor_pwm_max) return -10000;
    return (int16_t)value;
}

static int16_t balance_apply_right_trim(float control)
{
    float magnitude = fabsf(control) - BOARD_BALANCE_RIGHT_PWM_TRIM;

    /* The wheel asymmetry needs a smaller right PWM; the board trim is tuned separately. */
    if (magnitude <= 0.0f) return 0;
    return balance_pwm_from_float(control > 0.0f ? magnitude : -magnitude);
}

static uint8_t balance_update_complementary(float accel_angle, float gyro_rate, uint8_t accel_valid)
{
    float next_angle;
    float predicted_angle;

    if (!balance_is_finite(gyro_rate) || (accel_valid && !balance_is_finite(accel_angle))) return 0;
    if (accel_valid) balance_state.accel_angle = accel_angle;
    balance_state.gyro_rate = gyro_rate;
    if (!complementary_initialized) {
        if (!accel_valid) return 0;
        next_angle = accel_angle;
        complementary_initialized = 1;
    } else {
        predicted_angle = balance_state.angle + gyro_rate * balance_dt_s;
        next_angle = predicted_angle;
        if (accel_valid) next_angle = applied_params.accel_weight * accel_angle +
            (1.0f - applied_params.accel_weight) * predicted_angle;
    }
    if (!balance_is_finite(next_angle)) return 0;
    balance_state.angle = next_angle;
    return 1;
}

static uint8_t balance_run_control(void)
{
    float common;
    float turn;
    float turn_headroom;
    float right_control;

    /* Blend both targets so a nonzero speed correction cannot cause a step at either end of the launch lean. */
    balance_state.target_angle = applied_params.middle_angle +
        (1.0f - requested_lean_weight) * balance_state.speed_angle_applied + requested_lean_weight * requested_lean_angle;
    balance_state.angle_error = balance_state.angle - balance_state.target_angle;
    balance_state.p_output = applied_params.balance_kp * balance_state.angle_error;
    if (!balance_is_finite(balance_state.p_output)) return 0;
    balance_state.d_output = applied_params.balance_kd * balance_state.gyro_rate;
    /* Reverse upright feedback for the current IMU mounting without changing the stored angle reference. */
    balance_state.balance_output = -(balance_state.p_output + balance_state.d_output);
    if (!balance_is_finite(balance_state.balance_output)) return 0;
    common = balance_state.balance_output;
    balance_state.common_pwm = common;
    balance_state.turn_pwm_command = balance_clamp(requested_turn_pwm, applied_params.turn_pwm_limit);
    /* Steering only uses the PWM left after upright control, so it cannot clip one wheel's balance command. */
    turn_headroom = balance_motor_pwm_max - fabsf(common);
    if (turn_headroom < 0.0f) turn_headroom = 0.0f;
    turn = balance_clamp(balance_state.turn_pwm_command, turn_headroom);
    right_control = common - turn;
    balance_state.left_control_pwm = balance_pwm_from_float(common + turn);
    balance_state.right_control_pwm = balance_pwm_from_float(right_control);
    balance_state.left_pwm = balance_state.left_control_pwm;
    balance_state.right_pwm = balance_apply_right_trim(right_control);
    balance_state.turn_output = (balance_state.left_pwm - balance_state.right_pwm) * 0.5f;
    return 1;
}

void balance_params_reset(void)
{
    balance_params = balance_default_params;
    if (!balance_state.running) {
        applied_params = balance_default_params;
        applied_params.accel_weight = balance_accel_weight_2ms(balance_default_params.accel_weight);
    }
}

void balance_params_flash_switch(uint8_t gear)
{
    uint32_t offset;
    balance_params_t stored_params;

    if (!balance_params_profile_valid(gear) || balance_state.running) return;
    flash_read_page_to_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE);
    balance_params_flash_prepare();
    offset = balance_params_profile_offset(gear);
    memcpy(&stored_params, &flash_union_buffer[offset], sizeof(balance_params_t));
    if (!balance_params_are_valid(&stored_params)) stored_params = balance_default_params;
    balance_params = stored_params;
    applied_params = stored_params;
    applied_params.accel_weight = balance_accel_weight_2ms(stored_params.accel_weight);
    balance_clear_outputs();
}

void balance_params_flash_load(uint8_t gear)
{
    if (!balance_params_profile_valid(gear)) gear = 1;
    balance_params_flash_switch(gear);
}

uint8_t balance_params_flash_save(uint8_t gear)
{
    uint32_t offset;
    balance_params_t stored_params;

    if (!balance_params_profile_valid(gear) || balance_state.running || !balance_params_are_valid(&balance_params)) return 0;
    stored_params = balance_params;
    flash_read_page_to_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE);
    balance_params_flash_prepare();
    offset = balance_params_profile_offset(gear);
    memcpy(&flash_union_buffer[offset], &stored_params, sizeof(balance_params_t));
    if (flash_write_page_from_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE) != 0) return 0;
    flash_read_page_to_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE);
    return memcmp(&flash_union_buffer[offset], &stored_params, sizeof(stored_params)) == 0 ? 1 : 0;
}

uint8_t balance_params_flash_copy(uint8_t source, uint8_t target)
{
    uint32_t source_offset;
    uint32_t target_offset;
    balance_params_t stored_params;

    if (!balance_params_profile_valid(source) || !balance_params_profile_valid(target) || balance_state.running) return 0;
    if (source == target) return 1;
    flash_read_page_to_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE);
    if (flash_union_buffer[0].uint32_type != BALANCE_PARAMS_FLASH_MAGIC ||
        flash_union_buffer[1].uint32_type == 0 ||
        flash_union_buffer[1].uint32_type > BALANCE_PARAMS_FLASH_VERSION) return 0;
    /* Old saved profiles use the same migration as SWITCH GEAR. */
    if (flash_union_buffer[1].uint32_type != BALANCE_PARAMS_FLASH_VERSION) balance_params_flash_prepare();
    source_offset = balance_params_profile_offset(source);
    target_offset = balance_params_profile_offset(target);
    memcpy(&stored_params, &flash_union_buffer[source_offset], sizeof(stored_params));
    if (!balance_params_are_valid(&stored_params)) return 0;
    memcpy(&flash_union_buffer[target_offset], &stored_params, sizeof(stored_params));
    if (flash_write_page_from_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE) != 0) return 0;
    flash_read_page_to_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE);
    return memcmp(&flash_union_buffer[target_offset], &stored_params, sizeof(stored_params)) == 0;
}

static uint32_t balance_encoder_checksum(const balance_encoder_flash_t *record)
{
    return record->magic ^ record->version ^ record->left_counts_per_rev ^ record->right_counts_per_rev ^ 0xa5a5a5a5u;
}

static void balance_encoder_flash_load(void)
{
    balance_encoder_flash_t record;

    flash_read_page_to_buffer(0, BALANCE_ENCODER_FLASH_PAGE);
    memcpy(&record, flash_union_buffer, sizeof(record));
    if (record.magic != BALANCE_ENCODER_FLASH_MAGIC || (record.version != 1u && record.version != BALANCE_ENCODER_FLASH_VERSION) ||
        record.left_counts_per_rev == 0 || record.left_counts_per_rev > BALANCE_ENCODER_MAX_COUNTS ||
        record.right_counts_per_rev == 0 || record.right_counts_per_rev > BALANCE_ENCODER_MAX_COUNTS ||
        record.checksum != balance_encoder_checksum(&record)) return;
    /* V1 followed the old camera direction; the new front swaps the physical wheels. */
    balance_state.left_counts_per_rev = record.version == 1u ? record.right_counts_per_rev : record.left_counts_per_rev;
    balance_state.right_counts_per_rev = record.version == 1u ? record.left_counts_per_rev : record.right_counts_per_rev;
    balance_state.encoder_calibration_valid = 1;
}

static uint8_t balance_encoder_flash_save(void)
{
    balance_encoder_flash_t record;
    uint32_t index;

    record.magic = BALANCE_ENCODER_FLASH_MAGIC;
    record.version = BALANCE_ENCODER_FLASH_VERSION;
    record.left_counts_per_rev = balance_state.left_counts_per_rev;
    record.right_counts_per_rev = balance_state.right_counts_per_rev;
    record.checksum = balance_encoder_checksum(&record);
    for (index = 0; index < EEPROM_PAGE_LENGTH; index++) flash_union_buffer[index].uint32_type = 0xffffffffu;
    memcpy(flash_union_buffer, &record, sizeof(record));
    if (flash_write_page_from_buffer(0, BALANCE_ENCODER_FLASH_PAGE) != 0) return 0;
    flash_read_page_to_buffer(0, BALANCE_ENCODER_FLASH_PAGE);
    return memcmp(flash_union_buffer, &record, sizeof(record)) == 0 ? 1 : 0;
}

uint8_t balance_encoder_calibrate_toggle(uint8_t wheel)
{
    uint32_t total;
    uint32_t count;

    if (balance_state.running || (wheel != BALANCE_ENCODER_CAL_LEFT && wheel != BALANCE_ENCODER_CAL_RIGHT)) return 0;
    if (balance_state.encoder_calibration_state == BALANCE_ENCODER_CAL_LEFT ||
        balance_state.encoder_calibration_state == BALANCE_ENCODER_CAL_RIGHT) {
        if (balance_state.encoder_calibration_state != wheel) return 0;
        total = wheel == BALANCE_ENCODER_CAL_LEFT ? left_count_total : right_count_total;
        count = total - encoder_calibration_start_count;
        /* One revolution calibrates magnitude; either rotation direction is valid. */
        if (count > 0x7fffffffu) count = 0u - count;
        if (count == 0 || count > BALANCE_ENCODER_MAX_COUNTS) {
            balance_state.encoder_calibration_state = BALANCE_ENCODER_CAL_ERROR;
            return 0;
        }
        if (wheel == BALANCE_ENCODER_CAL_LEFT) balance_state.left_counts_per_rev = count;
        else balance_state.right_counts_per_rev = count;
        balance_state.encoder_calibration_state = BALANCE_ENCODER_CAL_IDLE;
        if (balance_state.left_counts_per_rev && balance_state.right_counts_per_rev) {
            if (!balance_encoder_flash_save()) {
                balance_state.encoder_calibration_valid = 0;
                balance_state.encoder_calibration_state = BALANCE_ENCODER_CAL_ERROR;
                return 0;
            }
            balance_state.encoder_calibration_valid = 1;
            balance_state.encoder_calibration_state = BALANCE_ENCODER_CAL_SAVED;
        }
        return 1;
    }
    encoder_calibration_start_count = wheel == BALANCE_ENCODER_CAL_LEFT ? left_count_total : right_count_total;
    balance_state.encoder_calibration_state = wheel;
    return 1;
}

void balance_set_speed_control_active(uint8_t active)
{
    uint32 interrupt_state = interrupt_global_disable();

    active = active && balance_state.running;
    /* Phase changes start with a clean PI; keep the measured speed for the next FOLLOW update. */
    if (speed_control_active != active) {
        balance_reset_speed_controller();
        speed_control_active = active;
    }
    interrupt_global_enable(interrupt_state);
}

void balance_set_follow_command(float target_speed_mps, float turn_pwm, float lean_angle, float lean_weight)
{
    if (!balance_state.running) {
        balance_cancel_follow_command();
        return;
    }
    if (!balance_is_finite(target_speed_mps) || !balance_is_finite(turn_pwm) || !balance_is_finite(lean_angle) ||
        !balance_is_finite(lean_weight) || lean_weight < 0.0f || lean_weight > 1.0f) {
        balance_state.fault = BALANCE_FAULT_NUMERIC;
        balance_cancel_follow_command();
        return;
    }
    if (target_speed_mps < 0.0f) target_speed_mps = 0.0f;
    if (target_speed_mps > 3.0f) target_speed_mps = 3.0f;
    requested_speed_mps = target_speed_mps;
    balance_state.speed_target_mps = target_speed_mps;
    requested_turn_pwm = turn_pwm;
    requested_lean_angle = lean_angle;
    requested_lean_weight = lean_weight;
}

static void balance_update_speed_20ms(void)
{
    float error;
    float pi_output;
    float angle_limit = applied_params.speed_angle_limit;

    if (balance_state.fault != BALANCE_FAULT_NONE) {
        balance_reset_speed_controller();
        return;
    }
    if (!balance_state.running || !speed_control_active || !applied_params.speed_enabled || !balance_state.encoder_calibration_valid) {
        balance_state.speed_target_applied_mps = 0.0f;
        balance_state.speed_feedback_mps = 0.0f;
        balance_state.speed_error_mps = 0.0f;
        balance_state.speed_p_angle = 0.0f;
        balance_state.speed_integral_angle = 0.0f;
        balance_state.speed_angle_applied = 0.0f;
        balance_state.speed_pid_active = 0;
        return;
    }

    balance_state.speed_target_applied_mps = requested_speed_mps;
    balance_state.speed_pid_active = 1;
    balance_state.speed_feedback_mps = balance_state.speed_filtered_mps;
    error = balance_state.speed_target_applied_mps - balance_state.speed_feedback_mps;
    balance_state.speed_error_mps = error;
    balance_state.speed_p_angle = applied_params.speed_kp * error;
    balance_state.speed_integral_angle += applied_params.speed_ki * error * speed_dt_s;
    balance_state.speed_integral_angle = balance_clamp(balance_state.speed_integral_angle, angle_limit);
    pi_output = balance_state.speed_p_angle + balance_state.speed_integral_angle;
    /* PI goes straight to the angle reference; the configured angle limit still bounds its magnitude. */
    balance_state.speed_angle_applied = BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_clamp(pi_output, angle_limit);
}

void balance_sample_encoders_2ms(int16_t left_count, int16_t right_count)
{
    balance_state.left_count = left_count;
    balance_state.right_count = right_count;
    left_count_sum += left_count;
    right_count_sum += right_count;
    left_count_total += (uint32_t)(int32_t)left_count;
    right_count_total += (uint32_t)(int32_t)right_count;
    if (balance_state.encoder_calibration_valid && balance_state.left_counts_per_rev > 0 &&
        balance_state.left_counts_per_rev <= BALANCE_ENCODER_MAX_COUNTS && balance_state.right_counts_per_rev > 0 &&
        balance_state.right_counts_per_rev <= BALANCE_ENCODER_MAX_COUNTS) {
        balance_state.speed_raw_mps = wheel_circumference_m *
            ((float)balance_state.left_count / balance_state.left_counts_per_rev +
             (float)balance_state.right_count / balance_state.right_counts_per_rev) / (2.0f * balance_dt_s);
        if (!balance_is_finite(balance_state.speed_filtered_mps)) balance_state.speed_filtered_mps = 0.0f;
        balance_state.speed_filtered_mps += speed_filter_weight *
            (balance_state.speed_raw_mps - balance_state.speed_filtered_mps);
    } else {
        /* Missing distance calibration means unknown m/s, not a stationary wheel. */
        balance_state.encoder_calibration_valid = 0;
        balance_state.speed_raw_mps = NAN;
        balance_state.speed_filtered_mps = NAN;
    }
    if (++encoder_sample_phase < 10) return;
    encoder_sample_phase = 0;
    /* Vision still uses a 20 ms window so its existing 250/20 preview thresholds keep their units. */
    balance_state.encoder_average_count = (left_count_sum + right_count_sum) / 2;
    left_count_sum = 0;
    right_count_sum = 0;
    balance_update_speed_20ms();
}

void balance_init(void)
{
    balance_params = balance_default_params;
    applied_params = balance_default_params;
    applied_params.accel_weight = balance_accel_weight_2ms(balance_default_params.accel_weight);
    memset((void *)&balance_state, 0, sizeof(balance_state));
    balance_state.speed_raw_mps = NAN;
    balance_state.speed_filtered_mps = NAN;
    balance_state.fault = BALANCE_FAULT_NONE;
    balance_state.control_period_ms = BALANCE_CONTROL_PERIOD_MS;
    complementary_initialized = 0;
    left_count_sum = 0;
    right_count_sum = 0;
    left_count_total = 0;
    right_count_total = 0;
    encoder_calibration_start_count = 0;
    encoder_sample_phase = 0;
    balance_clear_outputs();
    balance_encoder_flash_load();
}

uint8_t balance_seed_attitude(float angle)
{
    if (balance_state.running || !balance_is_finite(angle)) return 0;
    balance_state.angle = angle;
    balance_state.accel_angle = angle;
    balance_state.gyro_rate = 0.0f;
    balance_state.imu_ready = 1;
    complementary_initialized = 1;
    return 1;
}

uint8_t balance_start(void)
{
    if (balance_state.running) return 1;
    if (!balance_params_are_valid(&applied_params)) {
        balance_state.fault = BALANCE_FAULT_NUMERIC;
        return 0;
    }
    if (!balance_state.imu_ready || !balance_is_finite(balance_state.angle) || !balance_is_finite(balance_state.gyro_rate)) {
        balance_state.fault = BALANCE_FAULT_IMU;
        return 0;
    }
    balance_clear_outputs();
    balance_state.running = 1;
    balance_state.fault = BALANCE_FAULT_NONE;
    return 1;
}

void balance_stop(void)
{
    balance_state.running = 0;
    balance_state.fault = BALANCE_FAULT_NONE;
    balance_clear_outputs();
}

uint8_t balance_apply_params(void)
{
    if (balance_state.running) return 0;
    if (!balance_params_are_valid(&balance_params)) {
        balance_state.fault = BALANCE_FAULT_NUMERIC;
        return 0;
    }
    applied_params = balance_params;
    applied_params.accel_weight = balance_accel_weight_2ms(balance_params.accel_weight);
    balance_clear_outputs();
    /* Release rejected tuning once valid parameters and an initialized attitude are available. */
    if (balance_state.fault == BALANCE_FAULT_NUMERIC && balance_state.imu_ready) balance_state.fault = BALANCE_FAULT_NONE;
    return 1;
}

uint8_t balance_apply_live_params(const balance_params_t *params)
{
    uint32 interrupt_state;
    float accel_weight;

    if (!balance_params_are_valid(params)) return 0;
    accel_weight = balance_accel_weight_2ms(params->accel_weight);
    /* Calculate the filter conversion before disabling interrupts to keep the 2 ms ISR responsive. */
    interrupt_state = interrupt_global_disable();
    balance_params = *params;
    applied_params = *params;
    applied_params.accel_weight = accel_weight;
    if (!balance_state.running) balance_state.target_angle = applied_params.middle_angle;
    interrupt_global_enable(interrupt_state);
    return 1;
}

void balance_update_2ms(float accel_angle, float gyro_rate, uint8_t valid, uint8_t accel_valid)
{
    balance_state.output_updated = 0;
    if (!valid) {
        balance_state.imu_ready = 0;
        if (!balance_state.running) complementary_initialized = 0;
        balance_state.fault = BALANCE_FAULT_IMU;
        balance_cancel_follow_command();
        return;
    }
    if (!balance_update_complementary(accel_angle, gyro_rate, accel_valid)) {
        balance_state.imu_ready = 0;
        if (!balance_state.running) complementary_initialized = 0;
        balance_state.fault = BALANCE_FAULT_NUMERIC;
        balance_cancel_follow_command();
        return;
    }
    balance_state.imu_ready = 1;
    if (balance_state.running) {
        if (!balance_run_control()) {
            balance_state.fault = BALANCE_FAULT_NUMERIC;
            balance_cancel_follow_command();
            return;
        }
        balance_state.fault = BALANCE_FAULT_NONE;
        balance_state.output_updated = 1;
    }
}
