#include "car_menu_port.h"

#include "board_pins.h"
#include "math_utils.h"
#include "zf_common_headfile.h"

#define CC_MOTOR_PWM_FREQUENCY_HZ   (17000)
#define CC_SERVO_PWM_FREQUENCY_HZ   (200)
#define CC_SERVO_PERIOD_US          (5000)

/* Servo calibration for PWM_DUTY_MAX=10000 at 200 Hz:
 * 2480 -> lower limit -> 1240 us
 * 3040 -> center      -> 1520 us
 * 3600 -> upper limit -> 1800 us
 */
#define CC_SERVO_MIN_DUTY           (2480)
#define CC_SERVO_CENTER_DUTY        (3040)
#define CC_SERVO_MAX_DUTY           (3600)

static uint8_t camera_online;
static uint8_t actuator_ready;
static int8_t left_last_direction;
static int8_t right_last_direction;
static uint16_t left_encoder_previous;
static uint16_t right_encoder_previous;

#if BOARD_SERVO_ENABLE
static uint32_t servo_duty(int16_t pulse_us)
{
    uint32_t duty = (uint32_t)cc_math_clamp_i32(pulse_us, 1000, 3000) * PWM_DUTY_MAX / CC_SERVO_PERIOD_US;
    return cc_math_clamp_i32(duty, CC_SERVO_MIN_DUTY, CC_SERVO_MAX_DUTY);
}

#endif

static void write_motor_pair(pwm_channel_enum forward_pin, pwm_channel_enum reverse_pin, int32_t command, int8_t *last_direction)
{
    int32_t value = cc_math_clamp_i32(command, -PWM_DUTY_MAX, PWM_DUTY_MAX);
    uint32_t duty;
    int8_t direction;

    /* Disable both H-bridge inputs before changing direction. */
    pwm_set_duty(forward_pin, 0);
    pwm_set_duty(reverse_pin, 0);
    if (value == 0) return;

    direction = value > 0 ? 1 : -1;
    /* Duty writes use shadow registers: allow one 17 kHz period before reversing. */
    if (*last_direction != 0 && direction != *last_direction) system_delay_us(65);
    *last_direction = direction;
    duty = value > 0 ? value : -value;

    if (value > 0) pwm_set_duty(forward_pin, duty);
    else pwm_set_duty(reverse_pin, duty);
}

void cc_tc264_board_init(void)
{

    gpio_init(BOARD_LED1_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(BOARD_LED2_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(BOARD_BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(BOARD_KEY_UP_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_DOWN_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_LEFT_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_RIGHT_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    pwm_init(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, CC_MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, CC_MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, CC_MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, CC_MOTOR_PWM_FREQUENCY_HZ, 0);
#if BOARD_SERVO_ENABLE
    pwm_init(BOARD_SERVO_PWM_PIN, CC_SERVO_PWM_FREQUENCY_HZ, CC_SERVO_CENTER_DUTY);
#else
    gpio_init(BOARD_SERVO_AUX_GPIO_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
#endif

    actuator_ready = 1;
    cc_tc264_menu_motor_stop();
}

uint8_t cc_tc264_menu_key_mask(void)
{
    uint8_t mask = 0;

    if (gpio_get_level(BOARD_KEY_UP_PIN) == GPIO_LOW) mask |= CC_KEY_UP_MASK;
    if (gpio_get_level(BOARD_KEY_DOWN_PIN) == GPIO_LOW) mask |= CC_KEY_DOWN_MASK;
    if (gpio_get_level(BOARD_KEY_LEFT_PIN) == GPIO_LOW) mask |= CC_KEY_LEFT_MASK;
    if (gpio_get_level(BOARD_KEY_RIGHT_PIN) == GPIO_LOW) mask |= CC_KEY_RIGHT_MASK;
#if BOARD_KEY_CENTER_ENABLE
    if (gpio_get_level(BOARD_KEY_CENTER_PIN) == GPIO_LOW) mask |= CC_KEY_CENTER_MASK;
#endif
    return mask;
}

uint8_t cc_tc264_camera_init(void)
{
#if BOARD_CAMERA_ENABLE
    camera_online = mt9v03x_init() == 0 ? 1 : 0;
#else
    camera_online = 0;
#endif
    return camera_online;
}

uint8_t cc_tc264_camera_ready(void)
{
    return camera_online;
}

uint8_t cc_tc264_camera_set_exposure(uint16_t exposure)
{
#if BOARD_CAMERA_ENABLE
    if (camera_online == 0) return 0;
    return mt9v03x_set_exposure_time(exposure) == 0 ? 1 : 0;
#else
    (void)exposure;
    return 0;
#endif
}

uint8_t cc_tc264_camera_set_gain(uint8_t gain)
{
#if BOARD_CAMERA_ENABLE
    if (camera_online == 0) return 0;
    return mt9v03x_set_reg(0x35, gain) == 0 ? 1 : 0;
#else
    (void)gain;
    return 0;
#endif
}

uint8_t cc_tc264_motor_ready(void)
{
    return actuator_ready;
}

void cc_tc264_menu_motor_write(int16_t left, int16_t right)
{
    if (actuator_ready == 0) return;
    write_motor_pair(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, left * BOARD_BALANCE_LEFT_MOTOR_SIGN, &left_last_direction);
    write_motor_pair(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, right * BOARD_BALANCE_RIGHT_MOTOR_SIGN, &right_last_direction);
}

void cc_tc264_menu_motor_stop(void)
{
    if (actuator_ready == 0) return;
    write_motor_pair(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, 0, &left_last_direction);
    write_motor_pair(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, 0, &right_last_direction);
}

void cc_tc264_menu_servo_write_us(int16_t pulse_us)
{
#if BOARD_SERVO_ENABLE
    if (actuator_ready == 0) return;
    pwm_set_duty(BOARD_SERVO_PWM_PIN, servo_duty(pulse_us));
#else
    (void)pulse_us;
#endif
}

void cc_tc264_balance_encoder_init(void)
{
    /* TIM6 cannot use this library's quadrature mode; use rising-edge + direction on both wheels. */
    encoder_dir_init(BOARD_LEFT_ENCODER_INDEX, BOARD_LEFT_ENCODER_CH1, BOARD_LEFT_ENCODER_CH2);
    encoder_dir_init(BOARD_RIGHT_ENCODER_INDEX, BOARD_RIGHT_ENCODER_CH1, BOARD_RIGHT_ENCODER_CH2);
    left_encoder_previous = encoder_get_count(BOARD_LEFT_ENCODER_INDEX);
    right_encoder_previous = encoder_get_count(BOARD_RIGHT_ENCODER_INDEX);
}

void cc_tc264_balance_encoder_read(int16_t *left, int16_t *right)
{
    uint16_t current_left = encoder_get_count(BOARD_LEFT_ENCODER_INDEX);
    uint16_t current_right = encoder_get_count(BOARD_RIGHT_ENCODER_INDEX);
    int32_t delta_left = (int16_t)(uint16_t)(current_left - left_encoder_previous);
    int32_t delta_right = (int16_t)(uint16_t)(current_right - right_encoder_previous);

    /* Modular subtraction avoids losing edges between a read and clearing the timer. */
    left_encoder_previous = current_left;
    right_encoder_previous = current_right;
    delta_left *= BOARD_BALANCE_LEFT_ENCODER_SIGN;
    delta_right *= BOARD_BALANCE_RIGHT_ENCODER_SIGN;
    *left = cc_math_clamp_i32(delta_left, -32768, 32767);
    *right = cc_math_clamp_i32(delta_right, -32768, 32767);
}
