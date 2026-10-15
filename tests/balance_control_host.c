#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include "balance_control.h"
#include "board_pins.h"

flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];
static flash_data_union host_encoder_flash_page[EEPROM_PAGE_LENGTH];
static uint8 host_encoder_flash_valid;
static uint32 host_encoder_flash_read_count;
static uint32 host_encoder_flash_write_count;

typedef struct {
    uint32 magic;
    uint32 version;
    uint32 left_counts_per_rev;
    uint32 right_counts_per_rev;
    uint32 checksum;
} host_encoder_flash_record_t;

#define HOST_ENCODER_FLASH_MAGIC (0x454E4331u)

static uint32 host_encoder_flash_checksum(const host_encoder_flash_record_t *record)
{
    return record->magic ^ record->version ^ record->left_counts_per_rev ^ record->right_counts_per_rev ^ 0xa5a5a5a5u;
}

static void host_set_encoder_flash(uint32 version, uint32 left_counts, uint32 right_counts, uint8 valid_checksum)
{
    host_encoder_flash_record_t record;

    record.magic = HOST_ENCODER_FLASH_MAGIC;
    record.version = version;
    record.left_counts_per_rev = left_counts;
    record.right_counts_per_rev = right_counts;
    record.checksum = host_encoder_flash_checksum(&record);
    if (!valid_checksum) record.checksum ^= 1u;
    memset(host_encoder_flash_page, 0xff, sizeof(host_encoder_flash_page));
    memcpy(host_encoder_flash_page, &record, sizeof(record));
    host_encoder_flash_valid = 1;
}

uint32 interrupt_global_disable(void) { return 0; }
void interrupt_global_enable(uint32 state) { (void)state; }
void flash_read_page_to_buffer(uint32 sector, uint32 page)
{
    if (sector == 0 && page == 11) {
        host_encoder_flash_read_count++;
        if (host_encoder_flash_valid) memcpy(flash_union_buffer, host_encoder_flash_page, sizeof(flash_union_buffer));
    }
}

uint8 flash_write_page_from_buffer(uint32 sector, uint32 page)
{
    if (sector == 0 && page == 11) {
        host_encoder_flash_write_count++;
        memcpy(host_encoder_flash_page, flash_union_buffer, sizeof(flash_union_buffer));
        host_encoder_flash_valid = 1;
    }
    return 0;
}

static void near_value(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

static void test_2ms_filter_and_pd(void)
{
    balance_params_t params;
    int sample;

    balance_init();
    assert(balance_state.control_period_ms == 2);
    params = balance_params;
    params.accel_weight = 0.0f;
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(0.0f));
    for (sample = 0; sample < 50; sample++) balance_update_2ms(0.0f, 10.0f, 1, 0);
    near_value(balance_state.angle, 1.0f);

    params.accel_weight = 0.01f;
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(0.0f));
    for (sample = 0; sample < 500; sample++) balance_update_2ms(10.0f, 0.0f, 1, 1);
    assert(fabsf(balance_state.angle - 10.0f * (1.0f - powf(0.99f, 200.0f))) < 0.001f);
    near_value(balance_params.accel_weight, 0.01f);

    params.accel_weight = 1.0f;
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(1.0f));
    assert(balance_start());
    balance_update_2ms(1.0f, 2.0f, 1, 1);
    near_value(balance_state.p_output, 1000.0f);
    near_value(balance_state.d_output, balance_default_params.balance_kd * 2.0f);
    near_value(balance_state.balance_output, -(1000.0f + balance_default_params.balance_kd * 2.0f));
    balance_stop();
    balance_params_reset();
    assert(balance_seed_attitude(0.0f));
    balance_update_2ms(10.0f, 0.0f, 1, 1);
    near_value(balance_state.angle, 10.0f * (1.0f - powf(0.99f, 0.4f)));
    balance_init();
}

static void test_2ms_encoder_speed_and_20ms_pi(void)
{
    const float circumference = 0.065f * 3.14159265f;
    const float filter_weight = 0.9f;
    float expected_speed;
    float previous_filtered;
    float retained_speed;
    float expected_feedback = 0.0f;
    balance_params_t params;
    int sample;

    balance_init();
    expected_speed = -circumference * (-10.0f - 10.0f) / (2.0f * BALANCE_ENCODER_COUNTS_PER_REV * 0.002f);
    balance_sample_encoders_2ms(-10, -10);
    assert(balance_state.left_count == -10 && balance_state.right_count == -10);
    assert(balance_state.encoder_average_count == 0);
    near_value(balance_state.speed_raw_mps, expected_speed);
    near_value(balance_state.speed_filtered_mps, filter_weight * expected_speed);

    previous_filtered = balance_state.speed_filtered_mps;
    expected_speed = -circumference * -5.0f / (2.0f * BALANCE_ENCODER_COUNTS_PER_REV * 0.002f);
    balance_sample_encoders_2ms(0, -5);
    assert(balance_state.left_count == 0 && balance_state.right_count == -5);
    near_value(balance_state.speed_raw_mps, expected_speed);
    near_value(balance_state.speed_filtered_mps, previous_filtered + filter_weight * (expected_speed - previous_filtered));

    for (sample = 0; sample < 8; sample++) balance_sample_encoders_2ms(-1, -3);
    assert(balance_state.left_count == -1 && balance_state.right_count == -3);
    assert(balance_state.encoder_average_count == 28);

    balance_init();
    params = balance_params;
    params.speed_kp = 1.0f;
    params.speed_ki = 2.0f;
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_set_speed_control_active(1);
    balance_set_follow_command(0.5f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 9; sample++) {
        balance_sample_encoders_2ms(0, 0);
        near_value(balance_state.speed_target_applied_mps, 0.0f);
        near_value(balance_state.speed_integral_angle, 0.0f);
    }
    balance_sample_encoders_2ms(0, 0);
    near_value(balance_state.speed_target_applied_mps, 0.5f);
    near_value(balance_state.speed_error_mps, 0.5f);
    assert(balance_state.speed_pid_active);
    near_value(balance_state.speed_feedback_mps, 0.0f);
    near_value(balance_state.speed_p_angle, 0.5f);
    near_value(balance_state.speed_integral_angle, 2.0f * 0.5f * 0.02f);
    near_value(balance_state.speed_angle_applied, BOARD_FOLLOW_FORWARD_PWM_SIGN * 0.52f);
    assert(fabsf(balance_state.speed_angle_applied) > 0.04f);

    balance_set_follow_command(0.0f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 9; sample++) {
        balance_sample_encoders_2ms(-30, -30);
        expected_feedback += balance_state.speed_filtered_mps;
        near_value(balance_state.speed_target_applied_mps, 0.5f);
        near_value(balance_state.speed_error_mps, 0.5f);
        /* New 2 ms measurements must not rewrite the feedback/P used by the last PI step. */
        near_value(balance_state.speed_feedback_mps, 0.0f);
        near_value(balance_state.speed_p_angle, 0.5f);
        assert(balance_state.speed_filtered_mps > balance_state.speed_feedback_mps);
        near_value(balance_state.speed_integral_angle, 0.02f);
        near_value(balance_state.speed_angle_applied, BOARD_FOLLOW_FORWARD_PWM_SIGN * 0.52f);
    }
    balance_sample_encoders_2ms(-30, -30);
    expected_feedback += balance_state.speed_filtered_mps;
    assert(balance_state.speed_target_applied_mps == 0.0f);
    assert(balance_state.speed_error_mps < 0.0f);
    near_value(balance_state.speed_feedback_mps, expected_feedback / 10.0f);
    near_value(balance_state.speed_p_angle, balance_state.speed_error_mps);
    near_value(balance_state.speed_integral_angle, 0.02f + 2.0f * balance_state.speed_error_mps * 0.02f);
    near_value(balance_state.speed_angle_applied,
        BOARD_FOLLOW_FORWARD_PWM_SIGN * (balance_state.speed_error_mps + balance_state.speed_integral_angle));
    assert(balance_state.speed_angle_applied > 0.04f);

    balance_sample_encoders_2ms(-1, -1);
    retained_speed = balance_state.speed_filtered_mps;
    assert(retained_speed > 0.0f);
    balance_stop();
    assert(!balance_state.speed_pid_active);
    near_value(balance_state.speed_p_angle, 0.0f);
    near_value(balance_state.speed_filtered_mps, retained_speed);
    params = balance_params;
    params.speed_kp = 1.0f;
    assert(balance_apply_params());
    near_value(balance_state.speed_filtered_mps, retained_speed);
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    near_value(balance_state.speed_filtered_mps, retained_speed);
    balance_update_2ms(0.0f, 0.0f, 0, 0);
    assert(balance_state.fault == BALANCE_FAULT_IMU);
    near_value(balance_state.speed_filtered_mps, retained_speed);

    balance_init();
    params = balance_params;
    params.speed_kp = 0.0f;
    params.speed_ki = -1000.0f;
    params.speed_angle_limit = 0.1f;
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_set_speed_control_active(1);
    balance_set_follow_command(1.0f, 0.0f, 0.0f, 0.0f);
    balance_state.balance_output = -10000.0f;
    balance_state.common_pwm = -10000.0f;
    for (sample = 0; sample < 9; sample++) balance_sample_encoders_2ms(0, 0);
    near_value(balance_state.speed_integral_angle, 0.0f);
    balance_sample_encoders_2ms(0, 0);
    near_value(balance_state.speed_target_applied_mps, 1.0f);
    near_value(balance_state.speed_error_mps, 1.0f);
    near_value(balance_state.speed_integral_angle, -0.1f);
    near_value(balance_state.speed_angle_applied, 0.1f);
    assert(balance_state.speed_angle_applied > 0.04f);
    balance_init();
}

static void test_speed_pi_uses_whole_encoder_window(void)
{
    const float pulse_speed = 0.065f * 3.14159265f * 10.0f / (BALANCE_ENCODER_COUNTS_PER_REV * 0.002f);
    balance_params_t params;
    int sample;

    balance_init();
    params = balance_params;
    params.speed_kp = 1.0f;
    params.speed_ki = 0.0f;
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_set_speed_control_active(1);
    balance_set_follow_command(0.0f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 9; sample++) balance_sample_encoders_2ms(0, 0);
    balance_sample_encoders_2ms(-10, -10);
    near_value(balance_state.speed_raw_mps, pulse_speed);
    near_value(balance_state.speed_filtered_mps, 0.9f * pulse_speed);
    near_value(balance_state.speed_feedback_mps, 0.09f * pulse_speed);
    near_value(balance_state.speed_p_angle, -0.09f * pulse_speed);

    /* A pulse in the first sample must still affect PI even when the last sample is zero. */
    balance_init();
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_set_speed_control_active(1);
    balance_sample_encoders_2ms(-10, -10);
    for (sample = 0; sample < 9; sample++) balance_sample_encoders_2ms(0, 0);
    near_value(balance_state.speed_raw_mps, 0.0f);
    near_value(balance_state.speed_feedback_mps, pulse_speed / 10.0f);
    assert(balance_state.speed_p_angle < 0.0f);
    balance_init();
}

static void test_speed_integral_retains_drift_correction(void)
{
    const float measured_speed = 0.065f * 3.14159265f * 10.0f / (BALANCE_ENCODER_COUNTS_PER_REV * 0.002f);
    balance_params_t params;
    float slow_integral = 0.0f;
    float retained_integral;
    int test_case;
    int sample;

    for (test_case = 0; test_case < 2; test_case++) {
        balance_init();
        params = balance_params;
        params.speed_kp = 0.0f;
        params.speed_ki = test_case == 0 ? 0.01f : 0.5f;
        params.speed_angle_limit = 15.0f;
        assert(balance_apply_live_params(&params));
        assert(balance_seed_attitude(0.0f));
        assert(balance_start());
        balance_set_speed_control_active(1);
        balance_set_follow_command(0.5f, 0.0f, 0.0f, 0.0f);
        for (sample = 0; sample < 5000; sample++) balance_sample_encoders_2ms(-10, -10);
        assert(balance_state.speed_integral_angle < 0.0f);
        assert(balance_state.speed_angle_applied > 0.0f);
        if (test_case == 0) {
            slow_integral = balance_state.speed_integral_angle;
            assert(slow_integral < -0.05f && slow_integral > -0.06f);
        } else {
            assert(balance_state.speed_integral_angle < -2.5f && balance_state.speed_integral_angle > -3.0f);
            assert(fabsf(balance_state.speed_integral_angle / slow_integral - 50.0f) < 0.01f);
        }

        /* At target speed, the correction must remain available to cancel a persistent bias. */
        retained_integral = balance_state.speed_integral_angle;
        balance_set_follow_command(measured_speed, 0.0f, 0.0f, 0.0f);
        for (sample = 0; sample < 100; sample++) balance_sample_encoders_2ms(-10, -10);
        near_value(balance_state.speed_error_mps, 0.0f);
        near_value(balance_state.speed_integral_angle, retained_integral);
        balance_stop();
        near_value(balance_state.speed_integral_angle, 0.0f);
        near_value(balance_state.speed_angle_applied, 0.0f);
    }
    balance_init();
}

static void test_speed_pi_follow_gate(void)
{
    balance_params_t params;
    int sample;

    balance_init();
    params = balance_params;
    params.middle_angle = 2.5f;
    params.speed_kp = 1.0f;
    params.speed_ki = 2.0f;
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    near_value(balance_state.target_angle, params.middle_angle);

    balance_set_follow_command(0.5f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(-1, -1);
    assert(balance_state.speed_raw_mps > 0.0f && balance_state.speed_filtered_mps > 0.0f);
    near_value(balance_state.speed_target_applied_mps, 0.0f);
    near_value(balance_state.speed_integral_angle, 0.0f);
    near_value(balance_state.speed_angle_applied, 0.0f);
    balance_update_2ms(0.0f, 0.0f, 1, 1);
    near_value(balance_state.target_angle, params.middle_angle);

    balance_set_speed_control_active(1);
    balance_set_follow_command(0.5f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(-1, -1);
    near_value(balance_state.speed_target_applied_mps, 0.5f);
    assert(balance_state.speed_integral_angle > 0.0f);
    assert(BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_state.speed_angle_applied > 0.0f);

    balance_set_follow_command(0.0f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(-10, -10);
    assert(balance_state.speed_error_mps < 0.0f);
    assert(BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_state.speed_angle_applied < 0.0f);

    balance_set_speed_control_active(0);
    assert(!balance_state.speed_pid_active);
    near_value(balance_state.speed_p_angle, 0.0f);
    near_value(balance_state.speed_integral_angle, 0.0f);
    near_value(balance_state.speed_angle_applied, 0.0f);
    near_value(balance_state.speed_target_applied_mps, 0.0f);
    near_value(balance_state.target_angle, params.middle_angle);

    balance_set_speed_control_active(1);
    balance_set_follow_command(0.5f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(-1, -1);
    assert(balance_state.speed_angle_applied != 0.0f);
    balance_cancel_follow_command();
    near_value(balance_state.speed_integral_angle, 0.0f);
    near_value(balance_state.speed_angle_applied, 0.0f);
    near_value(balance_state.speed_target_applied_mps, 0.0f);
    near_value(balance_state.target_angle, params.middle_angle);

    balance_set_follow_command(0.5f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(-1, -1);
    near_value(balance_state.speed_integral_angle, 0.0f);
    near_value(balance_state.speed_angle_applied, 0.0f);
    balance_stop();
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_set_follow_command(0.5f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(-1, -1);
    near_value(balance_state.speed_integral_angle, 0.0f);
    near_value(balance_state.speed_angle_applied, 0.0f);

    params.speed_enabled = 0;
    assert(balance_apply_live_params(&params));
    balance_set_speed_control_active(1);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(-1, -1);
    near_value(balance_state.speed_target_applied_mps, 0.0f);
    near_value(balance_state.speed_integral_angle, 0.0f);
    near_value(balance_state.speed_angle_applied, 0.0f);
    balance_init();
}

static void test_encoder_scale_is_fixed_and_ignores_legacy_flash(void)
{
    int sample;

    host_set_encoder_flash(4u, 111u, 222u, 1);
    host_encoder_flash_read_count = 0;
    host_encoder_flash_write_count = 0;
    balance_init();
    assert(balance_state.encoder_calibration_valid);
    assert(balance_state.left_counts_per_rev == BALANCE_ENCODER_COUNTS_PER_REV);
    assert(balance_state.right_counts_per_rev == BALANCE_ENCODER_COUNTS_PER_REV);
    assert(balance_state.speed_raw_mps == 0.0f && balance_state.speed_filtered_mps == 0.0f);
    assert(host_encoder_flash_read_count == 0 && host_encoder_flash_write_count == 0);

    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_set_speed_control_active(1);
    balance_set_follow_command(1.5f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(-10, -10);
    assert(balance_state.speed_raw_mps > 0.0f && balance_state.speed_filtered_mps > 0.0f);
    assert(balance_state.speed_error_mps < 1.5f);
    balance_init();
    assert(balance_state.left_counts_per_rev == BALANCE_ENCODER_COUNTS_PER_REV);
    assert(balance_state.right_counts_per_rev == BALANCE_ENCODER_COUNTS_PER_REV);
    assert(host_encoder_flash_read_count == 0 && host_encoder_flash_write_count == 0);
}

static void test_signed_control_gains(void)
{
    balance_params_t params;
    int direction;
    int sample;

    balance_init();
    params = balance_default_params;
    params.balance_kp = -FLT_MAX;
    params.balance_kd = FLT_MAX;
    params.speed_kp = FLT_MAX;
    params.speed_ki = -FLT_MAX;
    assert(balance_params_are_valid(&params));
    assert(balance_apply_live_params(&params));
    params.balance_kp = FLT_MAX;
    params.balance_kd = -FLT_MAX;
    params.speed_kp = -FLT_MAX;
    params.speed_ki = FLT_MAX;
    assert(balance_params_are_valid(&params));
    params = balance_default_params;
    params.speed_kp = NAN;
    assert(!balance_params_are_valid(&params));
    params.speed_kp = 0.0f;
    params.speed_ki = INFINITY;
    assert(!balance_params_are_valid(&params));
    assert(balance_seed_attitude(0.0f));
    balance_params = params;
    assert(!balance_apply_params());
    assert(balance_state.fault == BALANCE_FAULT_NUMERIC);
    balance_params.speed_kp = -2000.0f;
    balance_params.speed_ki = -500.0f;
    assert(balance_apply_params());
    assert(balance_state.fault == BALANCE_FAULT_NONE);
    balance_state.imu_ready = 0;
    balance_state.fault = BALANCE_FAULT_NUMERIC;
    assert(balance_apply_params());
    assert(balance_state.fault == BALANCE_FAULT_NUMERIC);
    balance_state.fault = BALANCE_FAULT_IMU;
    assert(balance_apply_params());
    assert(balance_state.fault == BALANCE_FAULT_IMU);

    for (direction = -1; direction <= 1; direction += 2) {
        balance_init();
        /* Keep gain-sign checks below the PWM stop boundary so they test validation, not saturation. */
        balance_params.balance_kp = 0.01f;
        balance_params.speed_kp = direction * 2000.0f;
        balance_params.speed_ki = direction * 500.0f;
        balance_params.speed_angle_limit = 20.0f;
        assert(balance_apply_params());
        assert(balance_seed_attitude(0.0f));
        assert(balance_start());
        balance_set_speed_control_active(1);
        balance_set_follow_command(1.5f, 0.0f, 0.0f, 0.0f);
        for (sample = 0; sample < 1000; sample++) {
            balance_sample_encoders_2ms(0, 0);
            balance_update_2ms(0.0f, 0.0f, 1, 1);
            assert(balance_state.fault == BALANCE_FAULT_NONE);
            assert(isfinite(balance_state.balance_output));
            assert(fabsf(balance_state.speed_integral_angle) <= 20.0f);
            assert(fabsf(balance_state.speed_angle_applied) <= 20.0f);
        }
        assert(direction * BOARD_FOLLOW_FORWARD_PWM_SIGN * balance_state.speed_angle_applied > 0.0f);
    }
    balance_init();
}

static void test_pwm_fullscale_stop(void)
{
    balance_params_t params;
    int sample;
    int wheel;

    assert(BALANCE_FAULT_OSCILLATION == 7);
    balance_init();
    params = balance_params;
    params.balance_kp = 1.0f;
    params.balance_kd = 0.0f;
    params.accel_weight = 1.0f;
    params.turn_pwm_limit = 3000;
    params.speed_kp = 0.0f;
    params.speed_ki = 1.0f;
    params.speed_angle_limit = 1.0f;
    assert(balance_apply_live_params(&params));
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());

    /* RUN and the startup grace period must allow even simultaneous negative full duty. */
    balance_update_2ms(10000.0f, 0.0f, 1, 1);
    assert(balance_state.running && balance_state.output_updated);
    assert(balance_state.left_pwm == -10000 && balance_state.right_pwm == -10000);
    balance_set_speed_control_active(1);
    for (sample = 0; sample < 50; sample++) balance_update_2ms(10000.0f, 0.0f, 1, 1);
    assert(balance_state.running && balance_state.fault == BALANCE_FAULT_NONE);

    balance_set_oscillation_detection(1);
    for (sample = 0; sample < 50; sample++) balance_update_2ms(-10000.0f, 0.0f, 1, 1);
    assert(balance_state.running && balance_state.output_updated);
    assert(balance_state.left_pwm == 10000 && balance_state.right_pwm == 10000);
    balance_update_2ms(9999.0f, 0.0f, 1, 1);
    assert(balance_state.running && balance_state.fault == BALANCE_FAULT_NONE);
    assert(balance_state.left_pwm == -9999 && balance_state.right_pwm == -9999);

    /* Either wheel remaining at -9999 prevents the two-wheel negative-limit STOP. */
    for (wheel = 0; wheel < 2; wheel++) {
        balance_set_follow_command(0.0f, wheel == 0 ? -0.5f : 0.5f, 0.0f, 0.0f);
        balance_update_2ms(9999.5f, 0.0f, 1, 1);
        assert(balance_state.running && balance_state.output_updated);
        assert(balance_state.fault == BALANCE_FAULT_NONE);
        if (wheel == 0) {
            assert(balance_state.left_pwm == -10000 && balance_state.right_pwm == -9999);
        } else {
            assert(balance_state.left_pwm == -9999 && balance_state.right_pwm == -10000);
        }
    }
    balance_set_follow_command(1.0f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(0, 0);
    assert(balance_state.speed_pid_active && balance_state.speed_integral_angle > 0.0f);
    balance_update_2ms(11000.0f, 0.0f, 1, 1);
    assert(!balance_state.running && !balance_state.output_updated);
    assert(balance_state.fault == BALANCE_FAULT_OSCILLATION);
    assert(balance_state.left_pwm == 0 && balance_state.right_pwm == 0);
    assert(balance_state.left_control_pwm == 0 && balance_state.right_control_pwm == 0);
    assert(balance_state.balance_output == 0.0f && balance_state.common_pwm == 0.0f);
    assert(balance_state.speed_target_mps == 0.0f && balance_state.turn_pwm_command == 0.0f);
    assert(balance_state.speed_target_applied_mps == 0.0f && balance_state.speed_feedback_mps == 0.0f);
    assert(balance_state.speed_integral_angle == 0.0f && balance_state.speed_angle_applied == 0.0f);
    assert(!balance_state.speed_pid_active);
    balance_update_2ms(0.0f, 0.0f, 1, 1);
    assert(!balance_state.running && balance_state.fault == BALANCE_FAULT_OSCILLATION);

    /* Restart must leave detection disabled until FOLLOW enables it again after the grace period. */
    assert(balance_start());
    balance_set_speed_control_active(1);
    balance_update_2ms(10000.0f, 0.0f, 1, 1);
    assert(balance_state.running && balance_state.fault == BALANCE_FAULT_NONE);
    balance_set_oscillation_detection(1);
    balance_update_2ms(10000.0f, 0.0f, 1, 1);
    assert(!balance_state.running && balance_state.fault == BALANCE_FAULT_OSCILLATION);
    balance_init();
}

static void test_reverse_speed_feedback(void)
{
    int motion;
    int sample;

    /* Forward movement is negative; reverse movement is positive and reports negative speed. */
    for (motion = -1; motion <= 1; motion += 2) {
        balance_init();
        assert(balance_seed_attitude(0.0f));
        assert(balance_start());
        balance_set_speed_control_active(1);
        for (sample = 0; sample < 10; sample++) {
            balance_sample_encoders_2ms(20 * motion, 20 * motion);
        }
        assert(motion * balance_state.speed_raw_mps < 0.0f);
        assert(balance_state.encoder_average_count == -200 * motion);
        assert(motion * balance_state.speed_error_mps > 0.0f);
        balance_update_2ms(0.0f, 0.0f, 1, 1);
        /* At zero target, braking PWM follows the raw encoder direction. */
        assert(motion * balance_state.left_pwm > 0 && motion * balance_state.right_pwm > 0);
    }
    balance_init();
}

int main(void)
{
    balance_params_t old_params;
    balance_params_t params;
    int sample;

    test_speed_pi_uses_whole_encoder_window();
    test_speed_integral_retains_drift_correction();
    test_encoder_scale_is_fixed_and_ignores_legacy_flash();
    balance_init();
    memset(flash_union_buffer, 0xff, sizeof(flash_union_buffer));
    flash_union_buffer[0].uint32_type = 0x42414C31u;
    flash_union_buffer[1].uint32_type = 10u;
    old_params = balance_default_params;
    old_params.balance_kp = -1234.0f;
    old_params.balance_kd = -9.0f;
    old_params.target_speed_mps = 1.2f;
    old_params.speed_kp = -2000.0f;
    old_params.speed_ki = -500.0f;
    old_params.speed_angle_limit = 4.0f;
    old_params.speed_enabled = 1;
    memcpy(&flash_union_buffer[2], &old_params, sizeof(old_params));
    balance_params_flash_switch(1);
    near_value(balance_params.balance_kp, -1234.0f);
    near_value(balance_params.balance_kd, -9.0f);
    near_value(balance_params.target_speed_mps, 1.2f);
    near_value(balance_params.speed_kp, -2000.0f);
    near_value(balance_params.speed_ki, -500.0f);
    near_value(balance_params.speed_angle_limit, 4.0f);
    assert(balance_params.speed_enabled == 1);
    assert(flash_union_buffer[1].uint32_type == 10u);
    assert(balance_seed_attitude(0.0f));
    balance_update_2ms(10.0f, 0.0f, 1, 1);
    near_value(balance_state.angle, 10.0f * (1.0f - powf(0.99f, 0.4f)));
    near_value(balance_params.accel_weight, old_params.accel_weight);

    balance_init();
    near_value(balance_params.target_speed_mps, 1.5f);
    near_value(balance_params.speed_kp, 2.0f);
    near_value(balance_params.speed_ki, 0.3f);
    near_value(balance_params.speed_angle_limit, 5.0f);
    assert(balance_params.speed_enabled == 1);
    params = balance_params;
    assert(balance_params_are_valid(&params));
    params.target_speed_mps = -0.1f;
    assert(!balance_params_are_valid(&params));
    params.target_speed_mps = 3.1f;
    assert(!balance_params_are_valid(&params));
    params = balance_params;
    params.speed_kp = -0.1f;
    assert(balance_params_are_valid(&params));
    test_signed_control_gains();
    test_2ms_filter_and_pd();
    test_2ms_encoder_speed_and_20ms_pi();
    test_speed_pi_follow_gate();
    test_reverse_speed_feedback();
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_update_2ms(0.0f, 0.0f, 1, 1);
    assert(balance_state.output_updated && balance_state.left_pwm == 0 && balance_state.right_pwm == 0);

    balance_cancel_follow_command();
    assert(balance_state.speed_target_mps == 0.0f);
    balance_update_2ms(0.0f, 0.0f, 1, 1);
    assert(balance_state.speed_target_mps == 0.0f);

    balance_stop();
    assert(balance_seed_attitude(1.0f));
    assert(balance_start());
    balance_update_2ms(1.0f, 0.0f, 1, 1);
    near_value(balance_state.p_output, 1000.0f);
    near_value(balance_state.d_output, 0.0f);

    params = balance_params;
    params.accel_weight = 1.0f;
    params.turn_pwm_limit = 300;
    assert(balance_apply_live_params(&params));
    balance_set_follow_command(0.0f, 1000.0f, 0.0f, 0.0f);
    balance_update_2ms(0.0f, 0.0f, 1, 1);
    assert(balance_state.turn_pwm_command == 300.0f);

    balance_set_follow_command(0.0f, 20.0f * BOARD_FOLLOW_TURN_PWM_SIGN, 0.0f, 0.0f);
    balance_update_2ms(params.middle_angle - 1.0f, 0.0f, 1, 1);
    assert(balance_state.left_pwm == 980 && balance_state.right_pwm == 1020);
    balance_set_follow_command(0.0f, -20.0f * BOARD_FOLLOW_TURN_PWM_SIGN, 0.0f, 0.0f);
    balance_update_2ms(params.middle_angle - 1.0f, 0.0f, 1, 1);
    assert(balance_state.left_pwm == 1020 && balance_state.right_pwm == 980);
    balance_set_follow_command(0.0f, 0.0f, 0.0f, 0.0f);

    balance_update_2ms(params.middle_angle + 5.0f, 0.0f, 1, 1);
    near_value(balance_state.balance_output, -5000.0f);
    assert(balance_state.left_pwm == -5000 && balance_state.right_pwm == -5000);
    balance_update_2ms(params.middle_angle - 5.0f, 0.0f, 1, 1);
    near_value(balance_state.balance_output, 5000.0f);
    assert(balance_state.left_pwm == 5000 && balance_state.right_pwm == 5000);

    balance_update_2ms(0.0f, 0.0f, 0, 0);
    assert(!balance_state.output_updated && balance_state.fault == BALANCE_FAULT_IMU);
    assert(balance_state.running && balance_state.left_pwm == 5000 && balance_state.right_pwm == 5000);
    balance_stop();
    assert(!balance_state.running && balance_state.left_pwm == 0 && balance_state.right_pwm == 0);
    test_pwm_fullscale_stop();

    balance_init();
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_set_speed_control_active(1);
    balance_set_follow_command(1.5f, 0.0f, 0.0f, 0.0f);
    for (sample = 0; sample < 10; sample++) balance_sample_encoders_2ms(0, 0);
    assert(balance_state.speed_angle_applied < 0.0f);
    balance_update_2ms(0.0f, 0.0f, 1, 1);
    assert(balance_state.left_pwm < 0 && balance_state.right_pwm < 0);
    return 0;
}
