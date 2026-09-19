#include "car.h"

#include <string.h>
#include "balance_control.h"
#include "balance_runtime.h"
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
volatile float car_turn_request;

/* Main posts commands; the ISR owns follow/PD history. Line loss disarms
 * following until another explicit START, even when valid frames return. */
static volatile uint8 follow_command;
static uint8 following;
static volatile uint8 valid_frames;
static volatile uint32 last_valid_capture_ms;
static uint8 derivative_ready;
static uint32 last_control_capture_ms;
static float last_error;
static float curve_blend;
static float curve_exit_threshold = CAR_CURVE_EXIT_THRESHOLD_DEFAULT;
static float curve_enter_threshold = CAR_CURVE_ENTER_THRESHOLD_DEFAULT;

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
    car_follow_stop();
}

void car_init(void)
{
    /* Actuator port initializes the motor channels once. */
    car_running = 0;
    following = 0;
    follow_command = 0;
    valid_frames = 0;
    car_follow_status = CAR_FOLLOW_IDLE;
    car_reset_turn();
}

void car_set_camera_ready(uint8 ready)
{
    car_camera_ready = ready;
    if (!ready) car_follow_stop();
}

void car_set_running(uint8 running)
{
    if (running) balance_runtime_request_start();
    else car_stop();
}

void car_stop(void)
{
    car_follow_stop();
    balance_runtime_stop();
}

static uint8 car_follow_check(void)
{
    if (!balance_state.running) return CAR_FOLLOW_NO_BALANCE;
    if (!car_camera_ready || valid_frames < 3 || system_getval_ms() - last_valid_capture_ms > 100)
        return CAR_FOLLOW_NO_LINE;
    return CAR_FOLLOW_ACTIVE;
}

uint8 car_follow_start(void)
{
    uint8 status = car_follow_check();
    if (status != CAR_FOLLOW_ACTIVE) {
        car_follow_status = status;
        return 0;
    }
    follow_command = 1;
    return 1;
}

void car_follow_stop(void)
{
    follow_command = 2;
}

void car_track_update(float error, float curvature, uint8 valid, uint8 new_result, uint32 capture_ms, uint32 now_ms)
{
    float kp;
    float kd;
    float d_error = 0;
    float turn;
    uint32 elapsed_ms;
    uint8 command = follow_command;

    follow_command = 0;
    if (command == 2 || !balance_state.running) {
        following = 0;
        car_reset_turn();
        car_follow_status = CAR_FOLLOW_IDLE;
    }
    valid = valid && car_camera_ready && now_ms - capture_ms <= 100 &&
        car_value_finite(error) && car_value_finite(curvature);
    if (new_result && valid) {
        if (valid_frames != 0 && capture_ms - last_valid_capture_ms > 100) valid_frames = 0;
        last_valid_capture_ms = capture_ms;
        if (valid_frames < 3) valid_frames++;
        car_lost_count = 0;
    } else if (!valid || now_ms - last_valid_capture_ms > 100) {
        valid_frames = 0;
        if (new_result && car_lost_count < 65535) car_lost_count++;
        if (following) car_follow_status = CAR_FOLLOW_LINE_LOST;
        following = 0;
        car_reset_turn();
    }
    if (command == 1) {
        car_follow_status = car_follow_check();
        following = car_follow_status == CAR_FOLLOW_ACTIVE;
        car_reset_turn();
    }
    if (!following) {
        balance_set_follow_command(0, 0);
        return;
    }
    if (new_result && valid) {
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
        /* With no wheel-speed loop, camera PD directly requests PWM differential. */
        turn = (kp * error + kd * d_error) * balance_params.turn_direction;
        if (!car_value_finite(turn)) {
            following = 0;
            car_follow_status = CAR_FOLLOW_LINE_LOST;
            car_reset_turn();
            balance_set_follow_command(0, 0);
            return;
        }
        car_turn_request = cc_math_clamp_f32(turn, -balance_params.turn_pwm_limit, balance_params.turn_pwm_limit);
    }
    balance_set_follow_command(balance_params.follow_angle, car_turn_request);
}
