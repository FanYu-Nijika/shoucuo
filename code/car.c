#include "car.h"

#include "board_pins.h"

uint8 car_running = 0;
uint8 car_camera_ready = 0;
uint8 servo_reverse = 0;
uint8 left_motor_reverse = 0;
uint8 right_motor_reverse = 0;

uint16 servo_center_duty = 830;
uint16 servo_min_duty = 700;
uint16 servo_max_duty = 1000;
uint16 car_servo_duty = 830;
uint16 lost_stop_frames = 5;
uint16 car_lost_count = 0;

int16 motor_base_duty = 1800;
int16 motor_limit = 5000;
int16 curve_slowdown = 20;
int16 car_left_command = 0;
int16 car_right_command = 0;

float steering_kp = 2.0f;
float steering_kd = 4.0f;

static uint8 had_valid_track = 0;
static int16 last_error = 0;
static int16 last_valid_error = 0;

static void car_set_servo(uint16 duty)
{
    if (duty < servo_min_duty) duty = servo_min_duty;
    if (duty > servo_max_duty) duty = servo_max_duty;
    car_servo_duty = duty;
    pwm_set_duty(BOARD_SERVO_PWM_PIN, car_servo_duty);
}

static void car_set_one_motor(pwm_channel_enum forward_pin, pwm_channel_enum reverse_pin, int16 command, uint8 reverse)
{
    if (reverse) command = -command;
    if (command > motor_limit) command = motor_limit;
    if (command < -motor_limit) command = -motor_limit;

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
    if (!car_camera_ready) car_stop();
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

void car_track_update(int16 error, uint8 valid)
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

    if (!valid) {
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
        if (servo_command < servo_min_duty) servo_command = servo_min_duty;
        if (servo_command > servo_max_duty) servo_command = servo_max_duty;
        car_set_servo(servo_command);

        speed = motor_base_duty / 2;
        car_set_motor(speed, speed);
        return;
    }

    car_lost_count = 0;
    had_valid_track = 1;
    last_valid_error = error;
    steering = steering_kp * error + steering_kd * (error - last_error);
    last_error = error;
    if (servo_reverse) steering = -steering;

    servo_command = servo_center_duty + steering;
    if (servo_command < servo_min_duty) servo_command = servo_min_duty;
    if (servo_command > servo_max_duty) servo_command = servo_max_duty;
    car_set_servo(servo_command);

    error_abs = error >= 0 ? error : -error;
    speed = motor_base_duty - error_abs * curve_slowdown;
    if (speed < 0) speed = 0;
    if (speed > motor_limit) speed = motor_limit;
    car_set_motor(speed, speed);
}
