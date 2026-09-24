#include "balance_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "board_pins.h"
#include "zf_common_interrupt.h"
#include "zf_driver_flash.h"

static const float balance_dt_s = 0.005f;
static const float follow_angle_per_speed = 0.05f;
static const float follow_angle_rise_step = 0.01f;
static const float follow_angle_fall_step = 0.02f;
static const float follow_p_pwm_fraction = 0.6f;

#define BALANCE_PARAMS_FLASH_SECTOR        (0)
#define BALANCE_PARAMS_FLASH_PAGE          (9)
#define BALANCE_PARAMS_FLASH_MAGIC         (0x42414C31u)
#define BALANCE_PARAMS_FLASH_VERSION       (9u)
#define BALANCE_PARAMS_FLASH_PROFILE_COUNT (4u)
#define BALANCE_PARAMS_FLASH_PROFILE_START (2u)
#define BALANCE_PARAMS_FLASH_PROFILE_WORDS ((sizeof(balance_params_t) + 3u) / 4u)
#define BALANCE_PARAMS_V1_WORDS            (12u)
#define BALANCE_PARAMS_V4_WORDS            (14u)
#define BALANCE_PARAMS_V5_WORDS            (15u)
#define BALANCE_PARAMS_V7_WORDS            (16u)

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

typedef char balance_v7_target_check[offsetof(balance_params_v7_t, target_speed) == 48 ? 1 : -1];
typedef char balance_v7_deadzone_check[offsetof(balance_params_v7_t, left_motor_deadzone) == 56 ? 1 : -1];
typedef char balance_v7_size_check[sizeof(balance_params_v7_t) == 64 ? 1 : -1];
typedef char balance_v8_size_check[sizeof(balance_params_v8_t) == sizeof(balance_params_t) ? 1 : -1];
typedef char balance_v8_follow_check[offsetof(balance_params_v8_t, follow_angle) == offsetof(balance_params_t, follow_speed) ? 1 : -1];
typedef char balance_flash_capacity_check[2 + 4 * BALANCE_PARAMS_FLASH_PROFILE_WORDS <= EEPROM_PAGE_LENGTH ? 1 : -1];

const balance_params_t balance_default_params = {
    .balance_kp = 1000.0f,
    .balance_kd = 25.0556f,
    .middle_angle = 0.0f,
    .accel_weight = 0.01f,
    .follow_speed = 0.0f,
    .pwm_limit = 5000,
    .turn_pwm_limit = 1400,
    .turn_direction = 1,
    .left_motor_deadzone = 1100,
    .right_motor_deadzone = 600
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
static uint8_t encoder_sample_phase;
static float requested_follow_speed;
static float requested_turn_pwm;
static uint8_t follow_active;

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
        !balance_is_finite(params->follow_speed)) return 0;
    /* Kp/Kd deliberately have no tuning ceiling. Numeric and PWM checks contain unsafe values later. */
    if (params->balance_kp < 0.0f || params->balance_kd < 0.0f || fabsf(params->middle_angle) > 180.0f ||
        params->accel_weight < 0.0f || params->accel_weight > 1.0f ||
        params->follow_speed < -100.0f || params->follow_speed > 100.0f ||
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
    if (balance_is_finite(old_kp) && old_kp >= 0.0f) migrated.balance_kp = old_kp;
    if (balance_is_finite(old_kd) && old_kd >= 0.0f) migrated.balance_kd = old_kd;
    if (balance_is_finite(old->middle_angle) && fabsf(old->middle_angle) <= 180.0f) migrated.middle_angle = old->middle_angle;
    if (old->pwm_limit >= 1 && old->pwm_limit <= 10000) migrated.pwm_limit = old->pwm_limit;
    if (old->turn_direction == -1 || old->turn_direction == 1) migrated.turn_direction = old->turn_direction;
    if (old->left_motor_deadzone <= 3000) migrated.left_motor_deadzone = old->left_motor_deadzone;
    if (old->right_motor_deadzone <= 3000) migrated.right_motor_deadzone = old->right_motor_deadzone;
    return migrated;
}

/* Build a complete V9 page in RAM. Migration is written only when the user chooses SAVE FLASH. */
static void balance_params_flash_prepare(void)
{
    balance_params_t profiles[BALANCE_PARAMS_FLASH_PROFILE_COUNT];
    balance_params_v7_t old_profiles[BALANCE_PARAMS_FLASH_PROFILE_COUNT];
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
    } else if (flash_union_buffer[0].uint32_type == BALANCE_PARAMS_FLASH_MAGIC && version == 8u) {
        for (gear = 1; gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear++) {
            offset = balance_params_profile_offset(gear);
            memcpy(&profiles[gear - 1u], &flash_union_buffer[offset], sizeof(balance_params_v8_t));
            /* The old angle is not a speed command; start each migrated profile from zero drive. */
            profiles[gear - 1u].follow_speed = 0.0f;
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

void balance_cancel_follow_command(void)
{
    requested_follow_speed = 0.0f;
    requested_turn_pwm = 0.0f;
    follow_active = 0;
    balance_state.follow_angle_applied = 0.0f;
    balance_state.turn_pwm_command = 0.0f;
    balance_state.target_angle = applied_params.middle_angle;
    balance_state.angle_error = 0.0f;
    balance_state.turn_output = 0.0f;
}

static void balance_clear_outputs(void)
{
    balance_cancel_follow_command();
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
    if (value >= (float)applied_params.pwm_limit) return applied_params.pwm_limit;
    if (value <= -(float)applied_params.pwm_limit) return (int16_t)-applied_params.pwm_limit;
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
    float follow_target;
    float follow_step;
    float turn;
    float right_control;

    follow_target = requested_follow_speed * follow_angle_per_speed;
    /* Reverse commands pass through upright before applying lean in the opposite direction. */
    if (balance_state.follow_angle_applied * follow_target < 0.0f) follow_target = 0.0f;
    follow_step = fabsf(follow_target) > fabsf(balance_state.follow_angle_applied) ?
        follow_angle_rise_step : follow_angle_fall_step;
    balance_state.follow_angle_applied += balance_clamp(follow_target - balance_state.follow_angle_applied, follow_step);
    if (fabsf(balance_state.follow_angle_applied) < 0.0001f) balance_state.follow_angle_applied = 0.0f;
    balance_state.target_angle = applied_params.middle_angle - BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_state.follow_angle_applied;
    balance_state.angle_error = balance_state.angle - balance_state.target_angle;
    balance_state.p_output = applied_params.balance_kp * balance_state.angle_error;
    if (!balance_is_finite(balance_state.p_output)) return 0;
    /* Limit FOLLOW acceleration before gyro damping is added; RUN retains its original upright PD. */
    if (follow_active && balance_state.follow_angle_applied != 0.0f) {
        balance_state.p_output = balance_clamp(balance_state.p_output, applied_params.pwm_limit * follow_p_pwm_fraction);
    }
    balance_state.d_output = applied_params.balance_kd * balance_state.gyro_rate;
    balance_state.balance_output = balance_state.p_output + balance_state.d_output;
    if (!balance_is_finite(balance_state.balance_output)) return 0;
    common = balance_clamp(balance_state.balance_output, applied_params.pwm_limit);
    balance_state.common_pwm = common;
    balance_state.turn_pwm_command = balance_clamp(requested_turn_pwm, applied_params.turn_pwm_limit);
    /* Steering only uses the PWM left after upright control, so it cannot clip one wheel's balance command. */
    turn = balance_clamp(balance_state.turn_pwm_command, applied_params.pwm_limit - fabsf(common));
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
    if (!balance_state.running) applied_params = balance_default_params;
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

void balance_set_follow_command(float follow_speed, float turn_pwm)
{
    if (!balance_state.running) {
        balance_cancel_follow_command();
        return;
    }
    if (!balance_is_finite(follow_speed) || !balance_is_finite(turn_pwm)) {
        balance_state.fault = BALANCE_FAULT_NUMERIC;
        balance_cancel_follow_command();
        return;
    }
    requested_follow_speed = balance_clamp(follow_speed, 100.0f);
    requested_turn_pwm = turn_pwm;
    follow_active = 1;
}

void balance_sample_encoders_5ms(int16_t left_count, int16_t right_count)
{
    left_count_sum += left_count;
    right_count_sum += right_count;
    if (++encoder_sample_phase < 4) return;
    encoder_sample_phase = 0;
    balance_state.left_count = left_count_sum;
    balance_state.right_count = right_count_sum;
    left_count_sum = 0;
    right_count_sum = 0;
}

void balance_init(void)
{
    balance_params = balance_default_params;
    applied_params = balance_default_params;
    memset((void *)&balance_state, 0, sizeof(balance_state));
    balance_state.fault = BALANCE_FAULT_NONE;
    balance_state.control_period_ms = 5;
    complementary_initialized = 0;
    left_count_sum = 0;
    right_count_sum = 0;
    encoder_sample_phase = 0;
    balance_clear_outputs();
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
    balance_clear_outputs();
    return 1;
}

uint8_t balance_apply_live_params(const balance_params_t *params)
{
    uint32 interrupt_state;

    if (!balance_params_are_valid(params)) return 0;
    /* Copy both views in one short critical section so the 5 ms ISR never sees a mixed parameter set. */
    interrupt_state = interrupt_global_disable();
    balance_params = *params;
    applied_params = *params;
    if (!balance_state.running) balance_state.target_angle = applied_params.middle_angle;
    interrupt_global_enable(interrupt_state);
    return 1;
}

void balance_update_5ms(float accel_angle, float gyro_rate, uint8_t valid, uint8_t accel_valid)
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
