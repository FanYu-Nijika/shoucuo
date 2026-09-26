#include <assert.h>
#include <math.h>
#include <string.h>

#include "balance_control.h"
#include "board_pins.h"

flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];

uint32 interrupt_global_disable(void) { return 0; }
void interrupt_global_enable(uint32 state) { (void)state; }
void flash_read_page_to_buffer(uint32 sector, uint32 page) { (void)sector; (void)page; }
uint8 flash_write_page_from_buffer(uint32 sector, uint32 page) { (void)sector; (void)page; return 0; }

static void near_value(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

static float speed_filter_step(float previous, float raw)
{
    return previous + 0.25f * (raw - previous);
}

static float speed_angle_step(float previous, float target, float measured, float *integral)
{
    float error = target - measured;
    float requested;

    *integral += error * 0.02f;
    requested = 2.0f * error + 0.3f * *integral;
    if (requested > 5.0f) requested = 5.0f;
    if (requested < -5.0f) requested = -5.0f;
    if (requested - previous > 0.04f) return previous + 0.04f;
    if (requested - previous < -0.04f) return previous - 0.04f;
    return requested;
}

static void test_speed_loop_math(void)
{
    float integral = 0.0f;
    float filtered = speed_filter_step(0.0f, 1.0f);
    float angle = speed_angle_step(0.0f, 1.5f, filtered, &integral);

    near_value(filtered, 0.25f);
    assert(angle > 0.0f);
    integral = 0.0f;
    angle = speed_angle_step(0.0f, 0.0f, 1.0f, &integral);
    assert(angle < 0.0f);
    integral = 12.0f;
    angle = speed_angle_step(0.0f, 0.0f, 0.0f, &integral);
    assert(angle > 0.0f);
    integral = 0.0f;
    angle = speed_angle_step(0.0f, 10.0f, 0.0f, &integral);
    near_value(angle, 0.04f);
}

int main(void)
{
    balance_params_t old_params;
    balance_params_t params;

    balance_init();
    memset(flash_union_buffer, 0xff, sizeof(flash_union_buffer));
    flash_union_buffer[0].uint32_type = 0x42414C31u;
    flash_union_buffer[1].uint32_type = 10u;
    old_params = balance_default_params;
    old_params.balance_kp = 1234.0f;
    old_params.balance_kd = 9.0f;
    old_params.target_speed_mps = 1.2f;
    old_params.speed_kp = 2.5f;
    old_params.speed_ki = 0.4f;
    old_params.speed_angle_limit = 4.0f;
    old_params.speed_enabled = 1;
    memcpy(&flash_union_buffer[2], &old_params, sizeof(old_params));
    balance_params_flash_switch(1);
    near_value(balance_params.balance_kp, 1234.0f);
    near_value(balance_params.balance_kd, 9.0f);
    near_value(balance_params.target_speed_mps, 1.2f);
    near_value(balance_params.speed_kp, 2.5f);
    near_value(balance_params.speed_ki, 0.4f);
    near_value(balance_params.speed_angle_limit, 4.0f);
    assert(balance_params.speed_enabled == 1);
    assert(flash_union_buffer[1].uint32_type == 10u);

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
    assert(!balance_params_are_valid(&params));
    test_speed_loop_math();
    assert(balance_seed_attitude(0.0f));
    assert(balance_start());
    balance_update_5ms(0.0f, 0.0f, 1, 1);
    assert(balance_state.output_updated && balance_state.left_pwm == 0 && balance_state.right_pwm == 0);

    balance_cancel_follow_command();
    assert(balance_state.speed_target_mps == 0.0f);
    balance_update_5ms(0.0f, 0.0f, 1, 1);
    assert(balance_state.speed_target_mps == 0.0f);

    balance_stop();
    assert(balance_seed_attitude(1.0f));
    assert(balance_start());
    balance_update_5ms(1.0f, 0.0f, 1, 1);
    near_value(balance_state.p_output, 1000.0f);
    near_value(balance_state.d_output, 0.0f);

    params = balance_params;
    params.turn_pwm_limit = 300;
    assert(balance_apply_live_params(&params));
    balance_set_follow_command(0.0f, 1000.0f);
    balance_update_5ms(0.0f, 0.0f, 1, 1);
    assert(balance_state.turn_pwm_command == 300.0f);

    balance_update_5ms(0.0f, 1000.0f, 1, 1);
    assert(balance_state.common_pwm == 5000.0f);
    assert(balance_state.turn_output == BOARD_BALANCE_RIGHT_PWM_TRIM * 0.5f);
    assert(balance_state.left_pwm == 5000 && balance_state.right_pwm == 5000 - BOARD_BALANCE_RIGHT_PWM_TRIM);

    balance_update_5ms(0.0f, 0.0f, 0, 0);
    assert(!balance_state.output_updated && balance_state.fault == BALANCE_FAULT_IMU);
    assert(balance_state.left_pwm == 5000 && balance_state.right_pwm == 5000 - BOARD_BALANCE_RIGHT_PWM_TRIM);
    balance_stop();
    assert(!balance_state.running && balance_state.left_pwm == 0 && balance_state.right_pwm == 0);
    return 0;
}
