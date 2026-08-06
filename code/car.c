#include "car.h"

#include "board_pins.h"
#include "image.h"
#include "math_utils.h"

#define CAR_SERVO_MIN_DUTY (600)
#define CAR_SERVO_CENTER_DUTY (700)
#define CAR_SERVO_MAX_DUTY (800)

uint8 car_running = 0;
uint8 car_camera_ready = 0;
uint8 servo_reverse = 0;
uint8 left_motor_reverse = 0;
uint8 right_motor_reverse = 0;

uint16 servo_center_duty = CAR_SERVO_CENTER_DUTY;
uint16 servo_min_duty = CAR_SERVO_MIN_DUTY;
uint16 servo_max_duty = CAR_SERVO_MAX_DUTY;
uint16 car_servo_duty = CAR_SERVO_CENTER_DUTY;
uint16 lost_stop_frames = CAR_LOST_STOP_FRAMES;
uint16 car_lost_count = 0;

int16 motor_base_duty = 2800;
int16 motor_limit = 5000;
int16 curve_slowdown = 0;
int16 car_left_command = 0;
int16 car_right_command = 0;

float steering_kp = CAR_STEERING_KP_DEFAULT;
float steering_kd = CAR_STEERING_KD_DEFAULT;

static uint8 had_valid_track = 0;
static int16 last_error = 0;
static int16 last_valid_error = 0;
static const uint8 car_line_loss_protection_enabled = 0;

void car_apply_menu_params(const volatile car_params_t *params)
{
    uint32 center_duty;
    int32 minimum_duty;
    int32 maximum_duty;

    if (params == 0) return;

    motor_base_duty = params->base_speed;
    motor_limit = params->pwm_limit;
    curve_slowdown = params->curve_slowdown;
    lost_stop_frames = params->lost_stop_frames == 0 ? 1 : params->lost_stop_frames;
    steering_kp = params->steering_kp;
    steering_kd = params->steering_kd;
    servo_reverse = params->servo_reverse;
    left_motor_reverse = params->left_direction < 0 ? 1 : 0;
    right_motor_reverse = params->right_direction < 0 ? 1 : 0;

    center_duty = (uint32)(params->servo_center_us < 1000 ? 1000 : params->servo_center_us) * 10000 / 20000;
    minimum_duty = (int32)center_duty - (int32)params->servo_travel_us * 10000 / 20000;
    maximum_duty = (int32)center_duty + (int32)params->servo_travel_us * 10000 / 20000;
    servo_center_duty = (uint16)cc_math_clamp_i32(center_duty, CAR_SERVO_MIN_DUTY, CAR_SERVO_MAX_DUTY);
    servo_min_duty = (uint16)cc_math_clamp_i32(minimum_duty, CAR_SERVO_MIN_DUTY, CAR_SERVO_MAX_DUTY);
    servo_max_duty = (uint16)cc_math_clamp_i32(maximum_duty, CAR_SERVO_MIN_DUTY, CAR_SERVO_MAX_DUTY);
    if (servo_min_duty > servo_center_duty) servo_min_duty = servo_center_duty;
    if (servo_max_duty < servo_center_duty) servo_max_duty = servo_center_duty;

}


static void car_set_servo(uint16 duty)
{
    duty = cc_i16_max(duty, servo_min_duty);
    duty = cc_i16_min(duty, servo_max_duty);
    car_servo_duty = duty;
    pwm_set_duty(BOARD_SERVO_PWM_PIN, car_servo_duty);
}

static void car_set_one_motor(pwm_channel_enum forward_pin, pwm_channel_enum reverse_pin, int16 command, uint8 reverse)
{
    if (reverse) command = -command;
    if (command > motor_limit) command = motor_limit;
    if (command < -motor_limit) command = -motor_limit;
    command = cc_i16_max(command, -motor_limit);
    command = cc_i16_min(command, motor_limit);

    if (command >= 0) {
        pwm_set_duty(forward_pin, command);
        pwm_set_duty(reverse_pin, 0);
    } else {
        pwm_set_duty(forward_pin, 0);
        pwm_set_duty(reverse_pin, -command);
    }
}

static void car_set_motor(int16 left, int16 right)
{
    if (left > motor_limit) left = motor_limit;
    if (left < -motor_limit) left = -motor_limit;
    if (right > motor_limit) right = motor_limit;
    if (right < -motor_limit) right = -motor_limit;

    car_left_command = left;
    car_right_command = right;
    car_set_one_motor(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, left, left_motor_reverse);
    car_set_one_motor(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, right, right_motor_reverse);
}

void car_init(void)
{
    pwm_init(BOARD_SERVO_PWM_PIN, 50, servo_center_duty);
    pwm_init(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, 17000, 0);
    pwm_init(BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, 17000, 0);
    pwm_init(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, 17000, 0);
    pwm_init(BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, 17000, 0);
    car_stop();
}

void car_set_camera_ready(uint8 ready)
{
    car_camera_ready = ready;
    if (!ready) car_stop();
}

void car_set_running(uint8 running)
{
    if (running && car_camera_ready) {
        car_running = 1;
        car_lost_count = 0;
        had_valid_track = 0;
        last_error = 0;
        last_valid_error = 0;
    } else {
        car_stop();
    }
}

void car_toggle_running(void)
{
    car_set_running(!car_running);
}

void car_stop(void)
{
    car_running = 0;
    car_lost_count = 0;
    had_valid_track = 0;
    last_error = 0;
    last_valid_error = 0;
    car_set_motor(0, 0);
    car_set_servo(servo_center_duty);
}

void car_track_update(int16 error, uint8 valid, uint8 new_result)
{
    int16 speed;
    int16 error_abs;
    float steering;
    float servo_command;

    if (!car_running) {
        car_set_motor(0, 0);
        car_set_servo(servo_center_duty);
        return;
    }

    if (new_result == 0) return;

    if (!valid && car_line_loss_protection_enabled != 0) {
        car_lost_count++;
        if (car_lost_count >= lost_stop_frames) {
            car_running = 0;
            car_set_motor(0, 0);
            car_set_servo(servo_center_duty);
            return;
        }

        if (!had_valid_track) {
            car_set_motor(0, 0);
            car_set_servo(servo_center_duty);
            return;
        }

        steering = steering_kp * last_valid_error;
        if (servo_reverse) steering = -steering;
        servo_command = servo_center_duty + steering;
        servo_command = cc_f64_max(servo_command, servo_min_duty);
        servo_command = cc_f64_min(servo_command, servo_max_duty);
        car_set_servo(servo_command);

        speed = motor_base_duty / 2;
        car_set_motor(speed, speed);
        return;
    }

    if (!valid) valid = 1;

    car_lost_count = 0;
    had_valid_track = 1;
    last_valid_error = error;
    steering = steering_kp * error + steering_kd * (error - last_error);
    last_error = error;
    if (servo_reverse) steering = -steering;

    servo_command = servo_center_duty + steering;
    servo_command = cc_f64_max(servo_command, servo_min_duty);
    servo_command = cc_f64_min(servo_command, servo_max_duty);
    if (servo_command > servo_max_duty) servo_command = servo_max_duty;
    car_set_servo(servo_command);

    error_abs = error >= 0 ? error : -error;
    speed = motor_base_duty - error_abs * curve_slowdown;
    if (speed < 0) speed = 0;
    if (speed > motor_limit) speed = motor_limit;
    car_set_motor(speed, speed);
}
