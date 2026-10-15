#include "car.h"

#include <string.h>
#include "balance_control.h"
#include "balance_runtime.h"
#include "board_pins.h"
#include "math_utils.h"

volatile uint8 car_running;
uint8 car_camera_ready;
uint16 car_lost_count;
int16 car_left_command;
int16 car_right_command;
float steering_kp = CAR_STEERING_KP_DEFAULT;
float steering_kd = CAR_STEERING_KD_DEFAULT;
float curve_steering_kp = CAR_CURVE_STEERING_KP_DEFAULT;
float curve_steering_kd = CAR_CURVE_STEERING_KD_DEFAULT;
uint8 car_curve_mode;
volatile uint8 car_follow_status;
volatile uint8 car_follow_enabled;
volatile float car_turn_request;

/* Time each stage from its actual entry so a delayed RUN cannot shorten upright hold or acceleration. */
#define CAR_FOLLOW_WAIT_DELAY_MS (2000)
#define CAR_FOLLOW_UPRIGHT_MS (500)
#define CAR_FOLLOW_SPEED_RAMP_MS (2000)
#define CAR_FOLLOW_OSCERR_DELAY_MS (5000)

static uint8 derivative_ready;
static uint32 follow_stage_started_ms;
static uint32 follow_started_ms;
static uint32 last_control_capture_ms;
static float last_error;
static float curve_blend;
static float curve_exit_threshold = CAR_CURVE_EXIT_THRESHOLD_DEFAULT;
static float curve_enter_threshold = CAR_CURVE_ENTER_THRESHOLD_DEFAULT;
static const uint32 visual_timeout_ms = 120;

static uint8 car_value_finite(float value)
{
    uint32 bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static void car_reset_turn(void)
{
    derivative_ready = 0;
    last_error = 0;
    curve_blend = 0;
    car_curve_mode = 0;
    car_turn_request = 0;
}

void car_apply_menu_params(const volatile car_params_t *params)
{
    if (params == 0) return;
    steering_kp = params->steering_kp;
    steering_kd = params->steering_kd;
    curve_steering_kp = params->curve_steering_kp;
    curve_steering_kd = params->curve_steering_kd;
    curve_exit_threshold = cc_math_clamp_f32(params->curve_exit_threshold, 0, 1);
    curve_enter_threshold = cc_math_clamp_f32(params->curve_enter_threshold, curve_exit_threshold, 1);
}

void car_init(void)
{
    /* Actuator port initializes the motor channels once. */
    car_running = 0;
    car_follow_enabled = 0;
    car_follow_status = CAR_FOLLOW_IDLE;
    follow_stage_started_ms = 0;
    follow_started_ms = 0;
    car_reset_turn();
}

void car_set_camera_ready(uint8 ready)
{
    car_camera_ready = ready;
}

void car_set_running(uint8 running)
{
    if (running) balance_runtime_request_start();
    else car_stop();
}

void car_stop(void)
{
    balance_runtime_stop();
}

uint8 car_follow_start(void)
{
    balance_runtime_stop();
    follow_stage_started_ms = system_getval_ms();
    /* Keep the OSCERR grace period tied to the button press, independent of stage transitions. */
    follow_started_ms = follow_stage_started_ms;
    car_follow_status = CAR_FOLLOW_WAIT_DELAY;
    return 1;
}

void car_follow_stop(void)
{
    car_follow_enabled = 0;
    car_follow_status = CAR_FOLLOW_IDLE;
    follow_stage_started_ms = 0;
    follow_started_ms = 0;
    car_reset_turn();
    balance_cancel_follow_command();
}

void car_track_update(float error, float curvature, uint8 valid, uint8 new_result, uint32 capture_ms, uint32 now_ms)
{
    float kp;
    float kd;
    float d_error = 0;
    float turn;
    float target_speed_mps;
    uint32 elapsed_ms;
    uint8 visual_fresh = capture_ms != 0 && now_ms - capture_ms <= visual_timeout_ms;
    uint8 visual_valid = visual_fresh && valid && car_value_finite(error) && car_value_finite(curvature);

    if (car_follow_status != CAR_FOLLOW_IDLE && balance_state.fault != BALANCE_FAULT_NONE) {
        car_follow_stop();
        return;
    }
    if (car_follow_status == CAR_FOLLOW_WAIT_DELAY) {
        if (now_ms - follow_stage_started_ms < CAR_FOLLOW_WAIT_DELAY_MS) {
            car_reset_turn();
            balance_set_speed_control_active(0);
            balance_set_follow_command(0.0f, 0.0f, 0.0f, 0.0f);
            return;
        }
        car_follow_status = CAR_FOLLOW_WAIT_RUN;
        balance_runtime_request_start();
    }
    if (car_follow_status != CAR_FOLLOW_IDLE && car_follow_status != CAR_FOLLOW_WAIT_RUN && car_running == 0) {
        car_follow_enabled = 0;
        car_follow_status = CAR_FOLLOW_WAIT_RUN;
        follow_stage_started_ms = 0;
    }
    if (car_follow_status == CAR_FOLLOW_WAIT_RUN && car_running != 0) {
        follow_stage_started_ms = now_ms;
        car_follow_status = CAR_FOLLOW_RUN_UPRIGHT;
        car_follow_enabled = 1;
    }
    if (car_follow_status == CAR_FOLLOW_WAIT_RUN) {
        car_reset_turn();
        balance_set_speed_control_active(0);
        balance_set_follow_command(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
    if (car_follow_status == CAR_FOLLOW_RUN_UPRIGHT && now_ms - follow_stage_started_ms >= CAR_FOLLOW_UPRIGHT_MS) {
        follow_stage_started_ms = now_ms;
        car_follow_status = CAR_FOLLOW_SPEED_RAMP;
    }

    if (!visual_valid) {
        /* A missing or invalid camera result must not leave the last curve command driving the wheels. */
        if (new_result && car_lost_count < 65535) car_lost_count++;
        car_reset_turn();
    } else if (new_result) {
        car_lost_count = 0;
        if (!car_curve_mode && curvature >= curve_enter_threshold) {
            car_curve_mode = 1;
            derivative_ready = 0;
        } else if (car_curve_mode && curvature <= curve_exit_threshold) {
            car_curve_mode = 0;
            derivative_ready = 0;
        }
        curve_blend = cc_math_clamp_f32(curve_blend + (car_curve_mode ? 0.5f : -0.5f), 0, 1);
        elapsed_ms = capture_ms - last_control_capture_ms;
        if (derivative_ready && elapsed_ms > 0 && elapsed_ms <= 100) d_error = (error - last_error) * 20.0f / elapsed_ms;
        last_error = error;
        last_control_capture_ms = capture_ms;
        derivative_ready = 1;
        kp = steering_kp + (curve_steering_kp - steering_kp) * curve_blend;
        kd = steering_kd + (curve_steering_kd - steering_kd) * curve_blend;
        turn = (kp * error + kd * d_error) * balance_params.turn_direction * BOARD_FOLLOW_TURN_PWM_SIGN;
        if (car_value_finite(turn))
            car_turn_request = cc_math_clamp_f32(turn, -balance_params.turn_pwm_limit, balance_params.turn_pwm_limit);
    }

    if (!car_follow_enabled) {
        car_follow_status = CAR_FOLLOW_IDLE;
        balance_set_speed_control_active(0);
        balance_set_follow_command(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }
    /* Reuse the existing curve flag: one fixed cornering speed, otherwise the menu speed. */
    target_speed_mps = balance_params.target_speed_mps;
    if (car_curve_mode) target_speed_mps = 1.5f;
    /* Start PI at zero target after upright hold; ramp its reference, not its output or gains. */
    if (car_follow_status == CAR_FOLLOW_RUN_UPRIGHT) target_speed_mps = 0.0f;
    elapsed_ms = now_ms - follow_stage_started_ms;
    if (car_follow_status == CAR_FOLLOW_SPEED_RAMP) {
        if (elapsed_ms >= CAR_FOLLOW_SPEED_RAMP_MS) {
            follow_stage_started_ms = 0;
            car_follow_status = CAR_FOLLOW_ACTIVE;
        } else target_speed_mps *= elapsed_ms / (float)CAR_FOLLOW_SPEED_RAMP_MS;
    }
    balance_set_speed_control_active(car_follow_status != CAR_FOLLOW_RUN_UPRIGHT);
    balance_set_oscillation_detection(now_ms - follow_started_ms >= CAR_FOLLOW_OSCERR_DELAY_MS);
    balance_set_follow_command(target_speed_mps, car_turn_request, 0.0f, 0.0f);
}
