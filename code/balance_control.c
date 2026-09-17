#include "balance_control.h"

#include <string.h>
#include <stddef.h>
#include <math.h>

#include "board_pins.h"
#include "zf_driver_flash.h"

static const float balance_dt_s = 0.005f;
static const float balance_tilt_limit_deg = 40.0f;

#define BALANCE_PARAMS_FLASH_SECTOR        (0)
#define BALANCE_PARAMS_FLASH_PAGE          (9)
#define BALANCE_PARAMS_FLASH_MAGIC         (0x42414C31u)
#define BALANCE_PARAMS_FLASH_VERSION       (5u)
#define BALANCE_PARAMS_FLASH_OLD_WORDS     (12u)
#define BALANCE_PARAMS_FLASH_V4_WORDS      (14u)
#define BALANCE_PARAMS_FLASH_PROFILE_COUNT (4u)
#define BALANCE_PARAMS_FLASH_PROFILE_START (2u)
#define BALANCE_PARAMS_FLASH_PROFILE_WORDS ((sizeof(balance_params_t) + 3u) / 4u)

/* V1/V2 were 11 floats + int16 PWM + uint8 enable + one padding byte. */
typedef char balance_legacy_layout_check[offsetof(balance_params_t, target_speed) == 48 ? 1 : -1];
typedef char balance_v4_layout_check[offsetof(balance_params_t, left_motor_deadzone) == 56 ? 1 : -1];
typedef char balance_flash_capacity_check[2 + 4 * BALANCE_PARAMS_FLASH_PROFILE_WORDS <= EEPROM_PAGE_LENGTH ? 1 : -1];

/* The speed loop follows the reference controller's count-sample units.
 * These small values are unverified tuning starting points: speed_kp
 * is degrees/count per 20 ms and speed_ki is degrees/(count-sample). */
const balance_params_t balance_default_params = {
    .balance_kp = 1000.0f,
    .balance_kd = 25.0556f,
    .middle_angle = 0.0f,
    .speed_kp = 0.00025f,
    .speed_ki = 0.000005f,
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
    .right_motor_deadzone = 600
};

balance_params_t balance_params;
volatile balance_state_t balance_state;
static balance_params_t applied_params;

static float kalman_p00;
static float kalman_p01;
static float kalman_p10;
static float kalman_p11;
static uint8_t kalman_initialized;
static int32_t left_count_sum;
static int32_t right_count_sum;
static uint8_t speed_sample_phase;
static float requested_speed;
static float requested_turn_speed;

static uint8_t balance_params_profile_valid(uint8_t gear)
{
    return gear >= 1 && gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT ? 1 : 0;
}

static uint32_t balance_params_profile_offset(uint8_t gear)
{
    return BALANCE_PARAMS_FLASH_PROFILE_START + (gear - 1u) * BALANCE_PARAMS_FLASH_PROFILE_WORDS;
}

static uint8_t balance_is_finite(float value)
{
    uint32_t bits;

    /* TC264 uses IEEE-754 binary32; bit inspection also works with fast FP mode. */
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static void balance_reset_kalman(void)
{
    kalman_p00 = 1.0f;
    kalman_p01 = 0.0f;
    kalman_p10 = 0.0f;
    kalman_p11 = 1.0f;
    kalman_initialized = 0;
}

static void balance_reset_speed(void)
{
    left_count_sum = 0;
    right_count_sum = 0;
    speed_sample_phase = 0;
    requested_speed = 0;
    requested_turn_speed = 0;
    balance_state.effective_target_speed = 0;
    balance_state.left_target_speed = 0;
    balance_state.right_target_speed = 0;
    balance_state.left_speed_filtered = 0;
    balance_state.right_speed_filtered = 0;
    balance_state.left_speed_error = 0;
    balance_state.right_speed_error = 0;
    balance_state.left_speed_integral = 0;
    balance_state.right_speed_integral = 0;
    balance_state.left_speed_output = 0;
    balance_state.right_speed_output = 0;
    balance_state.left_target_angle = applied_params.middle_angle;
    balance_state.right_target_angle = applied_params.middle_angle;
    balance_state.turn_speed_target = 0;
    balance_state.turn_output = 0;
    balance_state.left_control_pwm = 0;
    balance_state.right_control_pwm = 0;
    balance_state.speed_output = 0.0f;
    balance_state.target_angle = applied_params.middle_angle;
    balance_state.speed_integral = 0.0f;
    balance_state.speed_error = 0.0f;
    balance_state.speed_filtered = 0.0f;
}

static void balance_clear_outputs(void)
{
    balance_state.target_angle = applied_params.middle_angle;
    balance_state.balance_output = 0.0f;
    balance_state.speed_output = 0.0f;
    balance_state.speed_integral = 0.0f;
    balance_state.speed_error = 0.0f;
    balance_state.speed_filtered = 0.0f;
    balance_state.left_target_speed = 0.0f;
    balance_state.right_target_speed = 0.0f;
    balance_state.left_speed_filtered = 0.0f;
    balance_state.right_speed_filtered = 0.0f;
    balance_state.left_speed_error = 0.0f;
    balance_state.right_speed_error = 0.0f;
    balance_state.left_speed_integral = 0.0f;
    balance_state.right_speed_integral = 0.0f;
    balance_state.left_speed_output = 0.0f;
    balance_state.right_speed_output = 0.0f;
    balance_state.left_target_angle = applied_params.middle_angle;
    balance_state.right_target_angle = applied_params.middle_angle;
    balance_state.turn_speed_target = 0.0f;
    balance_state.left_control_pwm = 0;
    balance_state.right_control_pwm = 0;
    balance_state.left_pwm = 0;
    balance_state.right_pwm = 0;
    balance_state.output_updated = 0;
    balance_state.effective_target_speed = 0;
    balance_state.turn_output = 0;
}

static void balance_latch_fault(balance_fault_t fault)
{
    balance_state.running = 0;
    balance_state.fault = (uint8_t)fault;
    balance_clear_outputs();
    balance_reset_speed();
}

uint8_t balance_params_are_valid(const balance_params_t *params)
{
    if (params == 0) return 0;
    if (!balance_is_finite(params->balance_kp) || !balance_is_finite(params->balance_kd) ||
        !balance_is_finite(params->middle_angle) || !balance_is_finite(params->speed_kp) ||
        !balance_is_finite(params->speed_ki) || !balance_is_finite(params->speed_filter) ||
        !balance_is_finite(params->speed_integral_limit) || !balance_is_finite(params->speed_output_limit) ||
        !balance_is_finite(params->q_angle) || !balance_is_finite(params->q_bias) ||
        !balance_is_finite(params->r_angle) || !balance_is_finite(params->target_speed)) return 0;
    if (params->balance_kp < 0 || params->balance_kp > 2000 || params->balance_kd < 0 ||
        params->balance_kd > 200 || fabsf(params->middle_angle) > 30 ||
        params->speed_kp < 0 || params->speed_kp > 0.1f || params->speed_ki < 0 || params->speed_ki > 0.01f ||
        params->speed_filter < 0 || params->speed_filter > 1 || params->speed_integral_limit <= 0 ||
        params->speed_integral_limit > 100000 || params->speed_output_limit <= 0 ||
        params->speed_output_limit > 5 || params->q_angle <= 0 || params->q_angle > 1 ||
        params->q_bias <= 0 || params->q_bias > 1 || params->r_angle <= 0 || params->r_angle > 100 ||
        params->pwm_limit <= 0 || params->pwm_limit > 10000 || params->speed_enabled > 1 ||
        params->target_speed < 0 || params->target_speed > 2000 || params->turn_limit < 0 ||
        params->turn_limit > 1000 || (params->turn_direction != -1 && params->turn_direction != 1) ||
        params->left_motor_deadzone > 3000 || params->right_motor_deadzone > 3000) return 0;
    return 1;
}

/* Prepare all four profiles in the shared buffer; only explicit SAVE writes Flash. */
static void balance_params_flash_prepare(void)
{
    uint32_t index;
    uint32_t offset;
    uint8_t gear;
    uint32_t version = flash_union_buffer[1].uint32_type;
    balance_params_t stored;

    if (flash_union_buffer[0].uint32_type == BALANCE_PARAMS_FLASH_MAGIC &&
        (version == 1u || version == 2u || version == 3u || version == 4u || version == BALANCE_PARAMS_FLASH_VERSION)) {
        /* Expand backwards so a current profile cannot overwrite the next old profile. */
        for (gear = BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear > 0; gear--) {
            offset = balance_params_profile_offset(gear);
            stored = balance_default_params;
            if (version < 3u) {
                index = BALANCE_PARAMS_FLASH_PROFILE_START + (gear - 1u) * BALANCE_PARAMS_FLASH_OLD_WORDS;
                memcpy(&stored, &flash_union_buffer[index], BALANCE_PARAMS_FLASH_OLD_WORDS * 4u);
            } else if (version < 5u) {
                index = BALANCE_PARAMS_FLASH_PROFILE_START + (gear - 1u) * BALANCE_PARAMS_FLASH_V4_WORDS;
                memcpy(&stored, &flash_union_buffer[index], BALANCE_PARAMS_FLASH_V4_WORDS * 4u);
            } else {
                memcpy(&stored, &flash_union_buffer[offset], sizeof(stored));
            }
            if (version == 1u) {
                if (balance_is_finite(stored.balance_kp) && stored.balance_kp > 0 &&
                    balance_is_finite(stored.speed_kp) && stored.speed_kp >= 0 && stored.speed_kp <= 1000 &&
                    balance_is_finite(stored.speed_ki) && stored.speed_ki >= 0 && stored.speed_ki <= 1000 &&
                    balance_is_finite(stored.speed_output_limit) &&
                    stored.speed_output_limit > 0 && stored.speed_output_limit <= 10000) {
                    stored.speed_kp /= stored.balance_kp;
                    stored.speed_ki /= stored.balance_kp;
                    stored.speed_output_limit /= stored.balance_kp;
                    if (stored.speed_output_limit > 1.5f) stored.speed_output_limit = 1.5f;
                } else {
                    stored.speed_kp = balance_default_params.speed_kp;
                    stored.speed_ki = balance_default_params.speed_ki;
                    stored.speed_output_limit = balance_default_params.speed_output_limit;
                }
                if (!balance_is_finite(stored.speed_kp) || stored.speed_kp > 0.1f ||
                    !balance_is_finite(stored.speed_ki) || stored.speed_ki > 0.01f) {
                    stored.speed_kp = balance_default_params.speed_kp;
                    stored.speed_ki = balance_default_params.speed_ki;
                    stored.speed_output_limit = balance_default_params.speed_output_limit;
                }
                stored.speed_enabled = 0;
            }
            /* V1-V3 stored a PWM turn limit. V4 uses per-wheel encoder counts/20 ms. */
            if (version < 4u) stored.turn_limit = balance_default_params.turn_limit;
            if (version < 5u) {
                stored.left_motor_deadzone = balance_default_params.left_motor_deadzone;
                stored.right_motor_deadzone = balance_default_params.right_motor_deadzone;
            }
            if (!balance_params_are_valid(&stored)) stored = balance_default_params;
            memcpy(&flash_union_buffer[offset], &stored, sizeof(stored));
        }
        flash_union_buffer[1].uint32_type = BALANCE_PARAMS_FLASH_VERSION;
        return;
    }

    for (index = 0; index < EEPROM_PAGE_LENGTH; index++) flash_union_buffer[index].uint32_type = 0xFFFFFFFFu;
    flash_union_buffer[0].uint32_type = BALANCE_PARAMS_FLASH_MAGIC;
    flash_union_buffer[1].uint32_type = BALANCE_PARAMS_FLASH_VERSION;
    for (gear = 1; gear <= BALANCE_PARAMS_FLASH_PROFILE_COUNT; gear++) {
        offset = balance_params_profile_offset(gear);
        memcpy(&flash_union_buffer[offset], &balance_default_params, sizeof(balance_params_t));
    }
}

void balance_params_reset(void)
{
    balance_params = balance_default_params;
    if (balance_state.running == 0) applied_params = balance_default_params;
}

void balance_params_flash_switch(uint8_t gear)
{
    uint32_t offset;
    balance_params_t stored_params;

    if (balance_params_profile_valid(gear) == 0 || balance_state.running != 0) return;
    flash_read_page_to_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE);
    balance_params_flash_prepare();
    offset = balance_params_profile_offset(gear);
    memcpy(&stored_params, &flash_union_buffer[offset], sizeof(balance_params_t));
    if (balance_params_are_valid(&stored_params) == 0) stored_params = balance_default_params;
    balance_params = stored_params;
    applied_params = stored_params;
    balance_clear_outputs();
    balance_reset_speed();
}

void balance_params_flash_load(uint8_t gear)
{
    if (balance_params_profile_valid(gear) == 0) gear = 1;
    balance_params_flash_switch(gear);
}

void balance_params_flash_save(uint8_t gear)
{
    uint32_t offset;

    if (balance_params_profile_valid(gear) == 0 || balance_state.running != 0) return;
    if (balance_params_are_valid(&balance_params) == 0) return;
    flash_read_page_to_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE);
    balance_params_flash_prepare();
    offset = balance_params_profile_offset(gear);
    memcpy(&flash_union_buffer[offset], &balance_params, sizeof(balance_params_t));
    flash_write_page_from_buffer(BALANCE_PARAMS_FLASH_SECTOR, BALANCE_PARAMS_FLASH_PAGE);
}

static uint8_t balance_update_kalman(float accel_angle, float gyro_rate)
{
    float rate;
    float p00_dot;
    float p01_dot;
    float p10_dot;
    float p11_dot;
    float innovation;
    float innovation_covariance;
    float kalman_gain_angle;
    float kalman_gain_bias;
    float old_p00;
    float old_p01;

    if (!balance_is_finite(accel_angle) || !balance_is_finite(gyro_rate)) {
        return 0;
    }

    if (!kalman_initialized) {
        balance_state.angle = accel_angle;
        balance_state.gyro_rate = gyro_rate;
        balance_state.gyro_bias = 0.0f;
        kalman_p00 = 1.0f;
        kalman_p01 = 0.0f;
        kalman_p10 = 0.0f;
        kalman_p11 = 1.0f;
        kalman_initialized = 1;
        return 1;
    }

    rate = gyro_rate - balance_state.gyro_bias;
    balance_state.angle += rate * balance_dt_s;

    p00_dot = applied_params.q_angle + balance_dt_s * kalman_p11 - kalman_p01 - kalman_p10;
    p01_dot = -kalman_p11;
    p10_dot = -kalman_p11;
    p11_dot = applied_params.q_bias;
    kalman_p00 += p00_dot * balance_dt_s;
    kalman_p01 += p01_dot * balance_dt_s;
    kalman_p10 += p10_dot * balance_dt_s;
    kalman_p11 += p11_dot * balance_dt_s;

    innovation = accel_angle - balance_state.angle;
    innovation_covariance = kalman_p00 + applied_params.r_angle;
    if (!balance_is_finite(innovation_covariance) || innovation_covariance <= 0.0f) {
        return 0;
    }

    kalman_gain_angle = kalman_p00 / innovation_covariance;
    kalman_gain_bias = kalman_p10 / innovation_covariance;
    old_p00 = kalman_p00;
    old_p01 = kalman_p01;
    kalman_p00 = old_p00 - kalman_gain_angle * old_p00;
    kalman_p01 = old_p01 - kalman_gain_angle * old_p01;
    kalman_p10 = kalman_p10 - kalman_gain_bias * old_p00;
    kalman_p11 = kalman_p11 - kalman_gain_bias * old_p01;
    balance_state.angle += kalman_gain_angle * innovation;
    balance_state.gyro_bias += kalman_gain_bias * innovation;
    /* The D term must use the residual-bias-corrected rate estimated by Kalman. */
    balance_state.gyro_rate = gyro_rate - balance_state.gyro_bias;

    if (!balance_is_finite(balance_state.angle) || !balance_is_finite(balance_state.gyro_rate) ||
        !balance_is_finite(balance_state.gyro_bias) ||
        !balance_is_finite(kalman_p00) || !balance_is_finite(kalman_p01) ||
        !balance_is_finite(kalman_p10) || !balance_is_finite(kalman_p11)) {
        return 0;
    }
    return 1;
}

static float balance_clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static int16_t balance_pwm_from_float(float value)
{
    if (value >= (float)applied_params.pwm_limit) {
        return applied_params.pwm_limit;
    }
    if (value <= -(float)applied_params.pwm_limit) {
        return (int16_t)-applied_params.pwm_limit;
    }
    return (int16_t)value;
}

static int16_t balance_apply_motor_deadzone(float control, uint16_t wheel_deadzone)
{
    uint16_t reference_deadzone = applied_params.left_motor_deadzone > applied_params.right_motor_deadzone ?
        applied_params.left_motor_deadzone : applied_params.right_motor_deadzone;
    float magnitude = fabsf(control) - (reference_deadzone - wheel_deadzone);

    /* Align the two measured start thresholds without scaling the entire speed range. */
    if (magnitude <= 0.0f) return 0;
    return balance_pwm_from_float(control > 0.0f ? magnitude : -magnitude);
}

static uint8_t balance_update_wheel_speed(uint8_t left_wheel, float error, float upright_output)
{
    float candidate_integral;
    float candidate_output;
    float integral_delta_output;
    float clipped_speed;
    float integral;
    float speed_output;
    float wheel_output;
    float pwm_delta;

    if (left_wheel) {
        integral = balance_state.left_speed_integral;
        speed_output = balance_state.left_speed_output;
    } else {
        integral = balance_state.right_speed_integral;
        speed_output = balance_state.right_speed_output;
    }
    candidate_integral = balance_clamp(integral + error * 2.0f, applied_params.speed_integral_limit);
    candidate_output = BOARD_BALANCE_SPEED_ANGLE_SIGN *
        (applied_params.speed_kp * error + applied_params.speed_ki * candidate_integral);
    integral_delta_output = BOARD_BALANCE_SPEED_ANGLE_SIGN * applied_params.speed_ki * (candidate_integral - integral);
    clipped_speed = balance_clamp(candidate_output, applied_params.speed_output_limit);
    clipped_speed = speed_output + balance_clamp(clipped_speed - speed_output, 0.04f);
    wheel_output = upright_output - applied_params.balance_kp * clipped_speed;
    pwm_delta = -applied_params.balance_kp * integral_delta_output;
    if (!balance_is_finite(error) || !balance_is_finite(candidate_integral) ||
        !balance_is_finite(candidate_output) || !balance_is_finite(wheel_output)) return 0;

    /* Each wheel freezes only integration that would push its own angle or PWM farther into saturation. */
    if ((integral_delta_output > 0 && candidate_output > clipped_speed) ||
        (integral_delta_output < 0 && candidate_output < clipped_speed) ||
        (pwm_delta > 0 && wheel_output > applied_params.pwm_limit) ||
        (pwm_delta < 0 && wheel_output < -applied_params.pwm_limit)) {
        candidate_integral = integral;
        candidate_output = BOARD_BALANCE_SPEED_ANGLE_SIGN *
            (applied_params.speed_kp * error + applied_params.speed_ki * candidate_integral);
    }
    clipped_speed = balance_clamp(candidate_output, applied_params.speed_output_limit);
    speed_output += balance_clamp(clipped_speed - speed_output, 0.04f);
    if (left_wheel) {
        balance_state.left_speed_integral = candidate_integral;
        balance_state.left_speed_output = speed_output;
    } else {
        balance_state.right_speed_integral = candidate_integral;
        balance_state.right_speed_output = speed_output;
    }
    return balance_is_finite(candidate_integral) && balance_is_finite(speed_output);
}

static uint8_t balance_run_speed(void)
{
    float filter = 1.0f - (1.0f - applied_params.speed_filter) * (1.0f - applied_params.speed_filter);
    float upright_output;

    if (!applied_params.speed_enabled) {
        balance_state.left_target_speed = 0;
        balance_state.right_target_speed = 0;
        balance_state.left_speed_filtered = 0;
        balance_state.right_speed_filtered = 0;
        balance_state.left_speed_error = 0;
        balance_state.right_speed_error = 0;
        balance_state.left_speed_integral = 0;
        balance_state.right_speed_integral = 0;
        balance_state.left_speed_output = 0;
        balance_state.right_speed_output = 0;
        balance_state.left_target_angle = applied_params.middle_angle;
        balance_state.right_target_angle = applied_params.middle_angle;
        balance_state.turn_speed_target = 0;
        balance_state.speed_error = 0;
        balance_state.speed_integral = 0;
        balance_state.speed_output = 0;
        balance_state.speed_filtered = 0;
        balance_state.effective_target_speed = 0;
    } else {
        /* The base target is ramped; camera differential changes without an extra slow ramp. */
        balance_state.effective_target_speed += balance_clamp(requested_speed - balance_state.effective_target_speed, 2.0f);
        balance_state.turn_speed_target = balance_clamp(requested_turn_speed, applied_params.turn_limit);
        balance_state.left_target_speed = balance_clamp(balance_state.effective_target_speed + balance_state.turn_speed_target, 2000.0f);
        balance_state.right_target_speed = balance_clamp(balance_state.effective_target_speed - balance_state.turn_speed_target, 2000.0f);
        balance_state.left_speed_filtered += filter * (balance_state.left_count - balance_state.left_speed_filtered);
        balance_state.right_speed_filtered += filter * (balance_state.right_count - balance_state.right_speed_filtered);
        balance_state.left_speed_error = balance_state.left_target_speed - balance_state.left_speed_filtered;
        balance_state.right_speed_error = balance_state.right_target_speed - balance_state.right_speed_filtered;
        upright_output = applied_params.balance_kp * (balance_state.angle - applied_params.middle_angle) +
            applied_params.balance_kd * balance_state.gyro_rate;
        if (!balance_is_finite(upright_output)) return 0;
        if (!balance_update_wheel_speed(1, balance_state.left_speed_error, upright_output)) return 0;
        if (!balance_update_wheel_speed(0, balance_state.right_speed_error, upright_output)) return 0;
        balance_state.left_target_angle = applied_params.middle_angle + balance_state.left_speed_output;
        balance_state.right_target_angle = applied_params.middle_angle + balance_state.right_speed_output;
        balance_state.speed_filtered = (balance_state.left_speed_filtered + balance_state.right_speed_filtered) * 0.5f;
        balance_state.speed_error = (balance_state.left_speed_error + balance_state.right_speed_error) * 0.5f;
        balance_state.speed_integral = (balance_state.left_speed_integral + balance_state.right_speed_integral) * 0.5f;
        balance_state.speed_output = (balance_state.left_speed_output + balance_state.right_speed_output) * 0.5f;
    }
    balance_state.target_angle = (balance_state.left_target_angle + balance_state.right_target_angle) * 0.5f;
    return balance_is_finite(balance_state.speed_filtered) && balance_is_finite(balance_state.target_angle) &&
        balance_is_finite(balance_state.turn_speed_target);
}

static uint8_t balance_run_control(void)
{
    float left_raw;
    float right_raw;
    float common;
    float diff;
    float left_control;
    float right_control;

    left_raw = applied_params.balance_kp * (balance_state.angle - balance_state.left_target_angle) +
        applied_params.balance_kd * balance_state.gyro_rate;
    right_raw = applied_params.balance_kp * (balance_state.angle - balance_state.right_target_angle) +
        applied_params.balance_kd * balance_state.gyro_rate;
    if (!balance_is_finite(left_raw) || !balance_is_finite(right_raw)) return 0;
    common = (left_raw + right_raw) * 0.5f;
    diff = (left_raw - right_raw) * 0.5f;
    balance_state.balance_output = common;
    common = balance_clamp(common, applied_params.pwm_limit);
    /* Preserve common balancing torque before allocating the remaining PWM to wheel-speed difference. */
    diff = balance_clamp(diff, applied_params.pwm_limit - fabsf(common));
    left_control = common + diff;
    right_control = common - diff;
    balance_state.left_control_pwm = balance_pwm_from_float(left_control);
    balance_state.right_control_pwm = balance_pwm_from_float(right_control);
    balance_state.left_pwm = balance_apply_motor_deadzone(left_control, applied_params.left_motor_deadzone);
    balance_state.right_pwm = balance_apply_motor_deadzone(right_control, applied_params.right_motor_deadzone);
    balance_state.turn_output = (balance_state.left_pwm - balance_state.right_pwm) * 0.5f;
    return 1;
}

void balance_set_motion_target(float speed, float turn)
{
    if (!balance_state.running) {
        requested_speed = 0;
        requested_turn_speed = 0;
        return;
    }
    if (!balance_is_finite(speed) || !balance_is_finite(turn)) {
        balance_latch_fault(BALANCE_FAULT_NUMERIC);
        return;
    }
    requested_speed = speed < 0 ? 0 : (speed > 2000 ? 2000 : speed);
    requested_turn_speed = balance_clamp(turn, 2000.0f);
}

void balance_sample_encoders_5ms(int16_t left_count, int16_t right_count)
{
    left_count_sum += left_count;
    right_count_sum += right_count;
    if (++speed_sample_phase < 4) return;
    speed_sample_phase = 0;
    balance_state.left_count = left_count_sum;
    balance_state.right_count = right_count_sum;
    left_count_sum = 0;
    right_count_sum = 0;
    if (balance_state.running) {
        if (!balance_run_speed()) balance_latch_fault(BALANCE_FAULT_NUMERIC);
    }
}

void balance_init(void)
{
    balance_params = balance_default_params;
    applied_params = balance_default_params;
    balance_state.angle = 0.0f;
    balance_state.gyro_rate = 0.0f;
    balance_state.gyro_bias = 0.0f;
    balance_state.balance_output = 0.0f;
    balance_state.speed_output = 0.0f;
    balance_state.speed_integral = 0.0f;
    balance_state.speed_error = 0.0f;
    balance_state.speed_filtered = 0.0f;
    balance_state.left_count = 0;
    balance_state.right_count = 0;
    balance_state.left_pwm = 0;
    balance_state.right_pwm = 0;
    balance_state.running = 0;
    balance_state.fault = BALANCE_FAULT_STOPPED;
    balance_state.imu_ready = 0;
    balance_state.output_updated = 0;
    balance_state.control_period_ms = 5;
    balance_state.speed_period_ms = 20;
    balance_reset_kalman();
    balance_reset_speed();
}

uint8_t balance_seed_attitude(float angle)
{
    if (balance_state.running) return 0;
    balance_reset_kalman();
    balance_state.imu_ready = balance_update_kalman(angle, 0.0f);
    return balance_state.imu_ready;
}

uint8_t balance_start(void)
{
    if (balance_state.running) {
        return 1;
    }
    /* Only this explicit request may release a latched fault, after fresh safety checks. */
    if (!balance_params_are_valid(&applied_params)) {
        balance_latch_fault(BALANCE_FAULT_NUMERIC);
        return 0;
    }
    if (!balance_state.imu_ready || !balance_is_finite(balance_state.angle) ||
        !balance_is_finite(balance_state.gyro_rate) || !balance_is_finite(balance_state.gyro_bias)) {
        balance_latch_fault(BALANCE_FAULT_IMU);
        return 0;
    }
    if (fabsf(balance_state.angle - applied_params.middle_angle) >= balance_tilt_limit_deg) {
        balance_latch_fault(BALANCE_FAULT_TILT);
        return 0;
    }
    balance_state.running = 1;
    balance_state.fault = BALANCE_FAULT_NONE;
    balance_state.output_updated = 0;
    balance_clear_outputs();
    balance_reset_speed();
    return 1;
}

void balance_stop(void)
{
    balance_state.running = 0;
    balance_state.fault = BALANCE_FAULT_STOPPED;
    balance_clear_outputs();
    balance_reset_speed();
}

uint8_t balance_apply_params(void)
{
    if (balance_state.running) return 0;
    if (!balance_params_are_valid(&balance_params)) {
        balance_latch_fault(BALANCE_FAULT_NUMERIC);
        return 0;
    }
    applied_params = balance_params;
    balance_clear_outputs();
    balance_reset_speed();
    return 1;
}

void balance_update_5ms(float accel_angle, float gyro_rate, int16_t left_count, int16_t right_count, uint8_t valid)
{
    (void)left_count;
    (void)right_count;
    balance_state.output_updated = 0;
    if (!valid) {
        balance_state.imu_ready = 0;
        balance_latch_fault(BALANCE_FAULT_IMU);
        balance_reset_kalman();
        return;
    }
    if (!balance_update_kalman(accel_angle, gyro_rate)) {
        balance_state.imu_ready = 0;
        balance_state.angle = 0;
        balance_state.gyro_rate = 0;
        balance_state.gyro_bias = 0;
        balance_reset_kalman();
        balance_latch_fault(BALANCE_FAULT_NUMERIC);
        return;
    }
    balance_state.imu_ready = 1;
    if (balance_state.running && fabsf(balance_state.angle - applied_params.middle_angle) >= balance_tilt_limit_deg) {
        balance_latch_fault(BALANCE_FAULT_TILT);
        return;
    }

    if (balance_state.running) {
        if (!balance_run_control()) {
            balance_latch_fault(BALANCE_FAULT_NUMERIC);
            return;
        }
        balance_state.output_updated = 1;
    }
}
