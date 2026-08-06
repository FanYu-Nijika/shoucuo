#include "car_menu_port.h"

#include "board_pins.h"
#include "math_utils.h"
#include "zf_common_headfile.h"

#define CC_MOTOR_PWM_FREQUENCY_HZ   (17000)
#define CC_SERVO_PWM_FREQUENCY_HZ   (50)
#define CC_SERVO_PERIOD_US          (20000)

/* Servo calibration for PWM_DUTY_MAX=10000 at 50 Hz:
 * 600 -> lower limit -> 1200 us
 * 700 -> center      -> 1400 us
 * 800 -> upper limit -> 1600 us
 */
#define CC_SERVO_MIN_DUTY           (600)
#define CC_SERVO_CENTER_DUTY        (700)
#define CC_SERVO_MAX_DUTY           (800)

static uint8_t camera_online;
static uint8_t actuator_ready;

static uint32_t servo_duty(int16_t pulse_us)
{
    uint32_t duty = (uint32_t)cc_math_clamp_i32(pulse_us, 1000, 3000) * PWM_DUTY_MAX / CC_SERVO_PERIOD_US;
    return cc_math_clamp_i32(duty, CC_SERVO_MIN_DUTY, CC_SERVO_MAX_DUTY);
}

static void write_motor_pair(pwm_channel_enum forward_pin, pwm_channel_enum reverse_pin, int16_t command)
{
    int32_t value = cc_math_clamp_i32(command, -PWM_DUTY_MAX, PWM_DUTY_MAX);
    uint32_t duty;

    /* Disable both H-bridge inputs before changing direction. */
    pwm_set_duty(forward_pin, 0);
    pwm_set_duty(reverse_pin, 0);
    if (value == 0) return;

    duty = (uint32_t)(value > 0 ? value : -value);

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
    gpio_init(BOARD_KEY_CENTER_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_AUX1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_AUX2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    pwm_init(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, CC_MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, CC_MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, CC_MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, CC_MOTOR_PWM_FREQUENCY_HZ, 0);
    pwm_init(BOARD_SERVO_PWM_PIN, CC_SERVO_PWM_FREQUENCY_HZ, CC_SERVO_CENTER_DUTY);

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
    if (gpio_get_level(BOARD_KEY_CENTER_PIN) == GPIO_LOW) mask |= CC_KEY_CENTER_MASK;
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
    write_motor_pair(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, left);
    write_motor_pair(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, right);
}

void cc_tc264_menu_motor_stop(void)
{
    if (actuator_ready == 0) return;
    write_motor_pair(BOARD_LEFT_MOTOR_FORWARD_PWM_PIN, BOARD_LEFT_MOTOR_REVERSE_PWM_PIN, 0);
    write_motor_pair(BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN, BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN, 0);
}

void cc_tc264_menu_servo_write_us(int16_t pulse_us)
{
    if (actuator_ready == 0) return;
    pwm_set_duty(BOARD_SERVO_PWM_PIN, servo_duty(pulse_us));
}

int32_t cc_tc264_encoder_read(uint8_t wheel_index)
{
    /* The current board has no encoder input. */
    (void)wheel_index;
    return 0;
}
