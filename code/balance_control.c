#include "balance_control.h"

#include <string.h>
#include <math.h>

static const float balance_dt_s = 0.005f;
static const float balance_tilt_limit_deg = 40.0f;

/* The speed loop follows the reference controller's count-sample units.
 * These small values are unverified tuning starting points: speed_kp
 * is PWM/count per 10 ms and speed_ki is PWM/(count-sample) per 10 ms. */
const balance_params_t balance_default_params = {
    .balance_kp = 375.0f,
    .balance_kd = 25.0556f,
    .middle_angle = 0.0f,
    .speed_kp = 0.25f,
    .speed_ki = 0.005f,
    .speed_filter = 0.16f,
    .speed_integral_limit = 30000.0f,
    .speed_output_limit = 1500.0f,
    .q_angle = 0.001f,
    .q_bias = 0.003f,
    .r_angle = 0.5f,
    .pwm_limit = 5000,
    .speed_enabled = 0
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
    balance_state.speed_output = 0.0f;
    balance_state.speed_integral = 0.0f;
    balance_state.speed_error = 0.0f;
    balance_state.speed_filtered = 0.0f;
}

static void balance_clear_outputs(void)
{
    balance_state.balance_output = 0.0f;
    balance_state.speed_output = 0.0f;
    balance_state.speed_integral = 0.0f;
    balance_state.speed_error = 0.0f;
    balance_state.speed_filtered = 0.0f;
    balance_state.left_pwm = 0;
    balance_state.right_pwm = 0;
    balance_state.output_updated = 0;
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
        !balance_is_finite(params->r_angle)) return 0;
    if (params->balance_kp < 0 || params->balance_kp > 2000 || params->balance_kd < 0 ||
        params->balance_kd > 200 || fabsf(params->middle_angle) > 30 ||
        params->speed_kp < 0 || params->speed_kp > 1000 || params->speed_ki < 0 || params->speed_ki > 1000 ||
        params->speed_filter < 0 || params->speed_filter > 1 || params->speed_integral_limit <= 0 ||
        params->speed_integral_limit > 100000 || params->speed_output_limit <= 0 ||
        params->speed_output_limit > 10000 || params->q_angle <= 0 || params->q_angle > 1 ||
        params->q_bias <= 0 || params->q_bias > 1 || params->r_angle <= 0 || params->r_angle > 100 ||
        params->pwm_limit <= 0 || params->pwm_limit > 10000 || params->speed_enabled > 1) return 0;
    return 1;
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

    balance_state.gyro_rate = gyro_rate;
    if (!kalman_initialized) {
        balance_state.angle = accel_angle;
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

    if (!balance_is_finite(balance_state.angle) || !balance_is_finite(balance_state.gyro_bias) ||
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

static uint8_t balance_run_control(void)
{
    float filtered_error;
    float candidate_integral;
    float candidate_output;
    float integral_delta_output;
    float combined_output;
    float clipped_speed;

    balance_state.balance_output = applied_params.balance_kp * (balance_state.angle - applied_params.middle_angle) +
        applied_params.balance_kd * balance_state.gyro_rate;
    if (!balance_is_finite(balance_state.balance_output)) return 0;

    if (!applied_params.speed_enabled) {
        balance_state.speed_error = 0;
        balance_state.speed_filtered = 0;
        balance_state.speed_integral = 0;
        balance_state.speed_output = 0;
    } else {
        balance_state.speed_error = -(left_count_sum + right_count_sum);
        filtered_error = balance_state.speed_filtered * (1.0f - applied_params.speed_filter) +
            balance_state.speed_error * applied_params.speed_filter;
        candidate_integral = balance_clamp(balance_state.speed_integral + filtered_error, applied_params.speed_integral_limit);
        candidate_output = -(applied_params.speed_kp * filtered_error + applied_params.speed_ki * candidate_integral);
        integral_delta_output = -applied_params.speed_ki * (candidate_integral - balance_state.speed_integral);
        clipped_speed = balance_clamp(candidate_output, applied_params.speed_output_limit);
        combined_output = balance_state.balance_output + clipped_speed;
        if (!balance_is_finite(filtered_error) || !balance_is_finite(candidate_integral) ||
            !balance_is_finite(candidate_output) || !balance_is_finite(combined_output)) return 0;
        /* Freeze only integration that makes saturation worse; always allow unwinding. */
        if ((integral_delta_output > 0 && (candidate_output > applied_params.speed_output_limit ||
             combined_output > applied_params.pwm_limit)) ||
            (integral_delta_output < 0 && (candidate_output < -applied_params.speed_output_limit ||
             combined_output < -applied_params.pwm_limit))) {
            candidate_integral = balance_state.speed_integral;
            candidate_output = -(applied_params.speed_kp * filtered_error + applied_params.speed_ki * candidate_integral);
        }
        balance_state.speed_filtered = filtered_error;
        balance_state.speed_integral = candidate_integral;
        balance_state.speed_output = balance_clamp(candidate_output, applied_params.speed_output_limit);
    }
    combined_output = balance_state.balance_output + balance_state.speed_output;
    if (!balance_is_finite(combined_output)) return 0;
    balance_state.left_pwm = balance_pwm_from_float(combined_output);
    balance_state.right_pwm = balance_state.left_pwm;
    return 1;
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
    balance_reset_kalman();
    balance_reset_speed();
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

    left_count_sum += left_count;
    right_count_sum += right_count;
    speed_sample_phase++;
    if (speed_sample_phase < 2) return;
    speed_sample_phase = 0;
    if (left_count_sum > 32767 || left_count_sum < -32768 || right_count_sum > 32767 || right_count_sum < -32768) {
        balance_latch_fault(BALANCE_FAULT_NUMERIC);
        return;
    }
    /* Publish complete 10 ms counts even when stopped, for hand-turn direction checks. */
    balance_state.left_count = left_count_sum;
    balance_state.right_count = right_count_sum;
    if (balance_state.running) {
        if (!balance_run_control()) {
            balance_latch_fault(BALANCE_FAULT_NUMERIC);
            return;
        }
        balance_state.output_updated = 1;
    } else {
        balance_clear_outputs();
    }
    left_count_sum = 0;
    right_count_sum = 0;
}
