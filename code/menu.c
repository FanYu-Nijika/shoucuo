#include "menu.h"

#include "board_pins.h"
#include "car.h"
#include "image.h"

enum {
    MENU_HOME = 0,
    MENU_DRIVE,
    MENU_STEERING,
    MENU_VISION,
    MENU_MOTOR,
    MENU_TELEMETRY,
    MENU_DIAGNOSTICS,
    MENU_CAMERA
};

static uint8 menu_page = MENU_HOME;
static uint8 menu_item = 0;
static uint8 diagnostic_led_on = 0;
static uint8 diagnostic_buzzer_on = 0;

static uint8 key_up_last = 0;
static uint8 key_down_last = 0;
static uint8 key_left_last = 0;
static uint8 key_right_last = 0;
static uint8 key_center_last = 0;
static uint8 key_aux1_last = 0;
static uint8 key_aux2_last = 0;

static uint8 menu_key_pressed(gpio_pin_enum pin, uint8 *last)
{
    uint8 pressed = !gpio_get_level(pin);
    uint8 event = pressed && !*last;
    *last = pressed;
    return event;
}

static uint8 menu_item_count(void)
{
    switch (menu_page) {
        case MENU_HOME: return 7;
        case MENU_DRIVE: return 6;
        case MENU_STEERING: return 7;
        case MENU_VISION: return 6;
        case MENU_MOTOR: return 5;
        case MENU_DIAGNOSTICS: return 4;
        default: return 0;
    }
}

static void menu_reset_defaults(void)
{
    car_stop();

    servo_center_duty = 830;
    servo_min_duty = 700;
    servo_max_duty = 1000;
    car_servo_duty = servo_center_duty;
    steering_kp = 2.0f;
    steering_kd = 4.0f;
    servo_reverse = 0;

    motor_base_duty = 1800;
    motor_limit = 5000;
    curve_slowdown = 20;
    left_motor_reverse = 0;
    right_motor_reverse = 0;
    lost_stop_frames = 5;

    image_auto_threshold = 1;
    threshold = 100;
    image_scan_start_col = 64;
    image_scan_end_col = 124;
    image_check_row = 80;
    image_search_start_row = 85;

    pwm_set_duty(BOARD_SERVO_PWM_PIN, servo_center_duty);
}

static void menu_change_value(int8 direction)
{
    int value;

    if (menu_page == MENU_DRIVE) {
        if (menu_item == 1) {
            motor_base_duty += direction * 100;
            if (motor_base_duty < 0) motor_base_duty = 0;
            if (motor_base_duty > motor_limit) motor_base_duty = motor_limit;
        } else if (menu_item == 2) {
            curve_slowdown += direction;
            if (curve_slowdown < 0) curve_slowdown = 0;
            if (curve_slowdown > 100) curve_slowdown = 100;
        } else if (menu_item == 3) {
            value = lost_stop_frames + direction;
            if (value < 1) value = 1;
            if (value > 100) value = 100;
            lost_stop_frames = value;
        }
    } else if (menu_page == MENU_STEERING) {
        if (menu_item == 0) {
            value = servo_center_duty + direction * 5;
            if (value < servo_min_duty) value = servo_min_duty;
            if (value > servo_max_duty) value = servo_max_duty;
            servo_center_duty = value;
            if (!car_running) {
                car_servo_duty = servo_center_duty;
                pwm_set_duty(BOARD_SERVO_PWM_PIN, servo_center_duty);
            }
        } else if (menu_item == 1) {
            value = servo_min_duty + direction * 5;
            if (value < 500) value = 500;
            if (value > servo_center_duty) value = servo_center_duty;
            servo_min_duty = value;
        } else if (menu_item == 2) {
            value = servo_max_duty + direction * 5;
            if (value < servo_center_duty) value = servo_center_duty;
            if (value > 1200) value = 1200;
            servo_max_duty = value;
        } else if (menu_item == 3) {
            steering_kp += direction * 0.1f;
            if (steering_kp < 0.0f) steering_kp = 0.0f;
            if (steering_kp > 20.0f) steering_kp = 20.0f;
        } else if (menu_item == 4) {
            steering_kd += direction * 0.1f;
            if (steering_kd < 0.0f) steering_kd = 0.0f;
            if (steering_kd > 50.0f) steering_kd = 50.0f;
        } else if (menu_item == 5) {
            servo_reverse = !servo_reverse;
        }
    } else if (menu_page == MENU_VISION) {
        if (menu_item == 0) {
            image_auto_threshold = !image_auto_threshold;
        } else if (menu_item == 1) {
            value = threshold + direction;
            if (value < 1) value = 1;
            if (value > 254) value = 254;
            threshold = value;
        } else if (menu_item == 2) {
            value = image_scan_start_col + direction;
            if (value < 0) value = 0;
            if (value > image_scan_end_col - 3) value = image_scan_end_col - 3;
            image_scan_start_col = value;
        } else if (menu_item == 3) {
            value = image_scan_end_col + direction;
            if (value < image_scan_start_col + 3) value = image_scan_start_col + 3;
            if (value > MT9V03X_W) value = MT9V03X_W;
            image_scan_end_col = value;
        } else if (menu_item == 4) {
            value = image_check_row + direction;
            if (value < 2) value = 2;
            if (value > MT9V03X_H - 1) value = MT9V03X_H - 1;
            image_check_row = value;
            if (image_search_start_row < image_check_row) image_search_start_row = image_check_row;
        } else if (menu_item == 5) {
            value = image_search_start_row + direction;
            if (value < image_check_row) value = image_check_row;
            if (value > MT9V03X_H - 1) value = MT9V03X_H - 1;
            image_search_start_row = value;
        }
    } else if (menu_page == MENU_MOTOR) {
        if (menu_item == 0) {
            motor_limit += direction * 100;
            if (motor_limit < 0) motor_limit = 0;
            if (motor_limit > 10000) motor_limit = 10000;
            if (motor_base_duty > motor_limit) motor_base_duty = motor_limit;
        } else if (menu_item == 1) {
            left_motor_reverse = !left_motor_reverse;
        } else if (menu_item == 2) {
            right_motor_reverse = !right_motor_reverse;
        }
    } else if (menu_page == MENU_DIAGNOSTICS) {
        if (menu_item == 1) {
            diagnostic_led_on = !diagnostic_led_on;
            gpio_set_level(BOARD_LED2_PIN, diagnostic_led_on ? GPIO_LOW : GPIO_HIGH);
        } else if (menu_item == 2) {
            diagnostic_buzzer_on = !diagnostic_buzzer_on;
            gpio_set_level(BOARD_BUZZER_PIN, diagnostic_buzzer_on ? GPIO_HIGH : GPIO_LOW);
        }
    }
}

static void menu_show_header(const char *title)
{
    ips200_full(RGB565_BLACK);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(8, 8, title);
    ips200_show_string(144, 8, car_camera_ready ? "CAM OK " : "CAM ERR");
    ips200_show_string(248, 8, car_running ? "RUN " : "STOP");
}

static void menu_show_selector(uint8 row)
{
    ips200_show_char(8, 48 + row * 24, menu_item == row ? '>' : ' ');
}

static void menu_show_bool(uint16 x, uint16 y, uint8 value)
{
    ips200_show_string(x, y, value ? "ON " : "OFF");
}

static void menu_display_home(void)
{
    menu_show_header("CAMERA4");

    for (uint8 row = 0; row < 7; row++) menu_show_selector(row);
    ips200_show_string(24, 48, "Drive");
    ips200_show_string(24, 72, "Steering");
    ips200_show_string(24, 96, "Vision");
    ips200_show_string(24, 120, "Rear motor");
    ips200_show_string(24, 144, "Telemetry");
    ips200_show_string(24, 168, "Diagnostics");
    ips200_show_string(24, 192, "Camera view");
    ips200_show_string(8, 216, "CENTER ENTER  AUX1 BACK  AUX2 RUN/STOP");
}

static void menu_display_drive(void)
{
    menu_show_header("DRIVE");

    for (uint8 row = 0; row < 6; row++) menu_show_selector(row);
    ips200_show_string(24, 48, "Run");
    menu_show_bool(232, 48, car_running);
    ips200_show_string(24, 72, "Base PWM");
    ips200_show_int(232, 72, motor_base_duty, 5);
    ips200_show_string(24, 96, "Curve slow");
    ips200_show_int(232, 96, curve_slowdown, 4);
    ips200_show_string(24, 120, "Lost frames");
    ips200_show_uint(232, 120, lost_stop_frames, 3);
    ips200_show_string(24, 144, "Track error");
    ips200_show_int(232, 144, track_error, 4);
    ips200_show_string(24, 168, "Track state");
    ips200_show_string(232, 168, track_valid ? "OK  " : "LOST");
    ips200_show_string(8, 216, "LEFT/RIGHT CHANGE  AUX1 BACK");
}

static void menu_display_steering(void)
{
    menu_show_header("STEERING");

    for (uint8 row = 0; row < 7; row++) menu_show_selector(row);
    ips200_show_string(24, 48, "Center duty");
    ips200_show_uint(232, 48, servo_center_duty, 4);
    ips200_show_string(24, 72, "Minimum duty");
    ips200_show_uint(232, 72, servo_min_duty, 4);
    ips200_show_string(24, 96, "Maximum duty");
    ips200_show_uint(232, 96, servo_max_duty, 4);
    ips200_show_string(24, 120, "Kp");
    ips200_show_float(232, 120, steering_kp, 4, 1);
    ips200_show_string(24, 144, "Kd");
    ips200_show_float(232, 144, steering_kd, 4, 1);
    ips200_show_string(24, 168, "Reverse");
    menu_show_bool(232, 168, servo_reverse);
    ips200_show_string(24, 192, "Servo duty");
    ips200_show_uint(232, 192, car_servo_duty, 4);
    ips200_show_string(8, 216, "CENTER/LEFT/RIGHT CHANGE  AUX1 BACK");
}

static void menu_display_vision(void)
{
    menu_show_header("VISION");

    for (uint8 row = 0; row < 6; row++) menu_show_selector(row);
    ips200_show_string(24, 48, "Auto threshold");
    menu_show_bool(232, 48, image_auto_threshold);
    ips200_show_string(24, 72, "Threshold");
    ips200_show_uint(232, 72, threshold, 3);
    ips200_show_string(24, 96, "Scan left");
    ips200_show_uint(232, 96, image_scan_start_col, 3);
    ips200_show_string(24, 120, "Scan right");
    ips200_show_uint(232, 120, image_scan_end_col, 3);
    ips200_show_string(24, 144, "Check row");
    ips200_show_uint(232, 144, image_check_row, 3);
    ips200_show_string(24, 168, "Search row");
    ips200_show_uint(232, 168, image_search_start_row, 3);
    ips200_show_string(8, 216, "LEFT/RIGHT CHANGE  AUX1 BACK");
}

static void menu_display_motor(void)
{
    menu_show_header("REAR MOTOR");

    for (uint8 row = 0; row < 5; row++) menu_show_selector(row);
    ips200_show_string(24, 48, "PWM limit");
    ips200_show_int(232, 48, motor_limit, 5);
    ips200_show_string(24, 72, "Left reverse");
    menu_show_bool(232, 72, left_motor_reverse);
    ips200_show_string(24, 96, "Right reverse");
    menu_show_bool(232, 96, right_motor_reverse);
    ips200_show_string(24, 120, "Left command");
    ips200_show_int(232, 120, car_left_command, 5);
    ips200_show_string(24, 144, "Right command");
    ips200_show_int(232, 144, car_right_command, 5);
    ips200_show_string(8, 216, "CENTER/LEFT/RIGHT CHANGE  AUX1 BACK");
}

static void menu_display_telemetry(void)
{
    menu_show_header("TELEMETRY");

    ips200_show_string(24, 48, "Track state");
    ips200_show_string(232, 48, track_valid ? "OK  " : "LOST");
    ips200_show_string(24, 72, "Threshold");
    ips200_show_uint(232, 72, threshold, 3);
    ips200_show_string(24, 96, "Best col");
    ips200_show_uint(232, 96, best_col, 3);
    ips200_show_string(24, 120, "Best row");
    ips200_show_uint(232, 120, best_row, 3);
    ips200_show_string(24, 144, "Track error");
    ips200_show_int(232, 144, track_error, 4);
    ips200_show_string(24, 168, "Servo duty");
    ips200_show_uint(232, 168, car_servo_duty, 4);
    ips200_show_string(24, 192, "Motor L/R");
    ips200_show_int(208, 192, car_left_command, 5);
    ips200_show_int(264, 192, car_right_command, 5);
    ips200_show_string(8, 216, "AUX1 BACK  AUX2 RUN/STOP");
}

static void menu_display_diagnostics(void)
{
    menu_show_header("DIAGNOSTICS");

    for (uint8 row = 0; row < 4; row++) menu_show_selector(row);
    ips200_show_string(24, 48, "Emergency stop");
    ips200_show_string(232, 48, "ENTER");
    ips200_show_string(24, 72, "LED test");
    menu_show_bool(232, 72, diagnostic_led_on);
    ips200_show_string(24, 96, "Buzzer test");
    menu_show_bool(232, 96, diagnostic_buzzer_on);
    ips200_show_string(24, 120, "Reset defaults");
    ips200_show_string(232, 120, "ENTER");
    ips200_show_string(24, 160, "CPU0 frame");
    menu_show_bool(232, 160, cpu0_done);
    ips200_show_string(24, 184, "CPU1 result");
    menu_show_bool(232, 184, cpu1_done);
    ips200_show_string(8, 216, "CENTER ACTION  AUX1 BACK");
}

void menu_init(void)
{
    gpio_init(BOARD_KEY_UP_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_DOWN_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_LEFT_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_RIGHT_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_CENTER_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_AUX1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOARD_KEY_AUX2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    key_up_last = !gpio_get_level(BOARD_KEY_UP_PIN);
    key_down_last = !gpio_get_level(BOARD_KEY_DOWN_PIN);
    key_left_last = !gpio_get_level(BOARD_KEY_LEFT_PIN);
    key_right_last = !gpio_get_level(BOARD_KEY_RIGHT_PIN);
    key_center_last = !gpio_get_level(BOARD_KEY_CENTER_PIN);
    key_aux1_last = !gpio_get_level(BOARD_KEY_AUX1_PIN);
    key_aux2_last = !gpio_get_level(BOARD_KEY_AUX2_PIN);

    menu_page = MENU_HOME;
    menu_item = 0;
    menu_display();
}

void menu_task(void)
{
    uint8 up = menu_key_pressed(BOARD_KEY_UP_PIN, &key_up_last);
    uint8 down = menu_key_pressed(BOARD_KEY_DOWN_PIN, &key_down_last);
    uint8 left = menu_key_pressed(BOARD_KEY_LEFT_PIN, &key_left_last);
    uint8 right = menu_key_pressed(BOARD_KEY_RIGHT_PIN, &key_right_last);
    uint8 center = menu_key_pressed(BOARD_KEY_CENTER_PIN, &key_center_last);
    uint8 aux1 = menu_key_pressed(BOARD_KEY_AUX1_PIN, &key_aux1_last);
    uint8 aux2 = menu_key_pressed(BOARD_KEY_AUX2_PIN, &key_aux2_last);
    uint8 count = menu_item_count();

    if (aux2) car_toggle_running();

    if (aux1) {
        diagnostic_led_on = 0;
        diagnostic_buzzer_on = 0;
        gpio_set_level(BOARD_LED1_PIN, car_camera_ready ? GPIO_LOW : GPIO_HIGH);
        gpio_set_level(BOARD_LED2_PIN, GPIO_HIGH);
        gpio_set_level(BOARD_BUZZER_PIN, GPIO_LOW);
        menu_page = MENU_HOME;
        menu_item = 0;
        return;
    }

    if (menu_page == MENU_CAMERA || menu_page == MENU_TELEMETRY) return;

    if (up && count) {
        if (menu_item == 0) menu_item = count - 1;
        else menu_item--;
    }

    if (down && count) {
        menu_item++;
        if (menu_item >= count) menu_item = 0;
    }

    if (center) {
        if (menu_page == MENU_HOME) {
            menu_page = menu_item + 1;
            menu_item = 0;
        } else if (menu_page == MENU_DRIVE && menu_item == 0) {
            car_toggle_running();
        } else if (menu_page == MENU_STEERING && menu_item == 5) {
            servo_reverse = !servo_reverse;
        } else if (menu_page == MENU_VISION && menu_item == 0) {
            image_auto_threshold = !image_auto_threshold;
        } else if (menu_page == MENU_MOTOR && menu_item == 1) {
            left_motor_reverse = !left_motor_reverse;
        } else if (menu_page == MENU_MOTOR && menu_item == 2) {
            right_motor_reverse = !right_motor_reverse;
        } else if (menu_page == MENU_DIAGNOSTICS && menu_item == 0) {
            car_stop();
        } else if (menu_page == MENU_DIAGNOSTICS && menu_item == 1) {
            menu_change_value(1);
        } else if (menu_page == MENU_DIAGNOSTICS && menu_item == 2) {
            menu_change_value(1);
        } else if (menu_page == MENU_DIAGNOSTICS && menu_item == 3) {
            menu_reset_defaults();
        }
    }

    if (left) menu_change_value(-1);
    if (right) menu_change_value(1);
}

void menu_display(void)
{
    if (menu_page == MENU_CAMERA) {
        image_display();
    } else if (menu_page == MENU_HOME) {
        menu_display_home();
    } else if (menu_page == MENU_DRIVE) {
        menu_display_drive();
    } else if (menu_page == MENU_STEERING) {
        menu_display_steering();
    } else if (menu_page == MENU_VISION) {
        menu_display_vision();
    } else if (menu_page == MENU_MOTOR) {
        menu_display_motor();
    } else if (menu_page == MENU_TELEMETRY) {
        menu_display_telemetry();
    } else if (menu_page == MENU_DIAGNOSTICS) {
        menu_display_diagnostics();
    }
}
