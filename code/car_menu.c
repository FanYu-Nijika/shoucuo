#include "car_menu.h"

#include <stdint.h>

#include "board_pins.h"
#include "car.h"
#include "car_menu_port.h"
#include "car_params.h"
#include "car_shared.h"
#include "car_uart_stream.h"
#include "longest_white.h"
#include "zf_common_headfile.h"

/* image.c owns this buffer. The menu only displays it after the frame channel is idle. */
extern uint8 *binary_image;

enum {
    CAR_MENU_ROOT = 0U,
    CAR_MENU_PAGE_RUN = 1U,
    CAR_MENU_PAGE_STEERING = 2U,
    CAR_MENU_PAGE_VISION = 3U,
    CAR_MENU_PAGE_MOTORS = 4U,
    CAR_MENU_PAGE_TELEMETRY = 5U,
    CAR_MENU_PAGE_IMAGE = 6U,
    CAR_MENU_PAGE_TOOLS = 7U
};

enum {
    CAR_MENU_ACTION_RUN = 1U,
    CAR_MENU_ACTION_STOP,
    CAR_MENU_ACTION_LCD_TEST,
    CAR_MENU_ACTION_RESET_DEFAULTS,
    CAR_MENU_ACTION_UART
};

enum {
    CAR_MENU_ITEM_PAGE = 1U,
    CAR_MENU_ITEM_VALUE,
    CAR_MENU_ITEM_INFO,
    CAR_MENU_ITEM_ACTION
};

enum {
    CAR_MENU_VALUE_NONE = 0U,
    CAR_MENU_VALUE_I16,
    CAR_MENU_VALUE_U16,
    CAR_MENU_VALUE_U8,
    CAR_MENU_VALUE_I8,
    CAR_MENU_VALUE_U32,
    CAR_MENU_VALUE_FLOAT,
    CAR_MENU_VALUE_BOOL,
    CAR_MENU_VALUE_DIRECTION
};

enum {
    CAR_MENU_APPLY_NONE = 0U,
    CAR_MENU_APPLY_PARAMS,
    CAR_MENU_APPLY_SERVO_CENTER,
    CAR_MENU_APPLY_EXPOSURE,
    CAR_MENU_APPLY_GAIN
};

enum {
    CAR_MENU_ERROR_NONE = 0U,
    CAR_MENU_ERROR_CAMERA,
    CAR_MENU_ERROR_MOTOR,
    CAR_MENU_ERROR_RUNNING,
    CAR_MENU_ERROR_LINE_LOST
};

#define CAR_MENU_VISIBLE_ROWS     (8U)
#define CAR_MENU_REFRESH_MS       (100U)
#define CAR_MENU_LCD_TEST_MS      (1500U)

typedef struct {
    uint16_t id;
    uint16_t parent;
    const char *name;
    uint8_t item_type;
    uint8_t value_type;
    volatile void *address;
    float minimum;
    float maximum;
    float step;
    uint8_t decimals;
    uint8_t apply;
    uint8_t action;
} car_menu_item_t;

/*
 * Menu maintenance is intentionally a flat list. Add or remove one row here;
 * no second value table or synchronization code needs to be updated.
 */
static const car_menu_item_t car_menu_items[] = {
    {CAR_MENU_PAGE_RUN, CAR_MENU_ROOT, "RUN CONTROL", CAR_MENU_ITEM_PAGE, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, 0, 0},
    {CAR_MENU_PAGE_STEERING, CAR_MENU_ROOT, "STEERING", CAR_MENU_ITEM_PAGE, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, 0, 0},
    {CAR_MENU_PAGE_VISION, CAR_MENU_ROOT, "VISION", CAR_MENU_ITEM_PAGE, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, 0, 0},
    {CAR_MENU_PAGE_MOTORS, CAR_MENU_ROOT, "MOTORS", CAR_MENU_ITEM_PAGE, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, 0, 0},
    {CAR_MENU_PAGE_TELEMETRY, CAR_MENU_ROOT, "TELEMETRY", CAR_MENU_ITEM_PAGE, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, 0, 0},
    {CAR_MENU_PAGE_IMAGE, CAR_MENU_ROOT, "IMAGE VIEW", CAR_MENU_ITEM_PAGE, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, 0, 0},
    {CAR_MENU_PAGE_TOOLS, CAR_MENU_ROOT, "TOOLS", CAR_MENU_ITEM_PAGE, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, 0, 0},

    {0, CAR_MENU_PAGE_RUN, "RUN", CAR_MENU_ITEM_ACTION, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, CAR_MENU_APPLY_NONE, CAR_MENU_ACTION_RUN},
    {0, CAR_MENU_PAGE_RUN, "STOP", CAR_MENU_ITEM_ACTION, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, CAR_MENU_APPLY_NONE, CAR_MENU_ACTION_STOP},
    {0, CAR_MENU_PAGE_RUN, "RUNNING", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_BOOL, &car_running, 0, 1, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_RUN, "BASE SPEED", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_I16, &car_params.base_speed, 0, 10000, 100, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_RUN, "PWM LIMIT", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_I16, &car_params.pwm_limit, 0, 10000, 100, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_RUN, "CURVE SLOW", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_I16, &car_params.curve_slowdown, 0, 1000, 5, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_RUN, "LOST STOP", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U16, &car_params.lost_stop_frames, 1, 65535, 1, 0, CAR_MENU_APPLY_PARAMS, 0},

    {0, CAR_MENU_PAGE_STEERING, "STEERING KP", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_FLOAT, &car_params.steering_kp, 0, 50, 0.1f, 1, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_STEERING, "STEERING KD", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_FLOAT, &car_params.steering_kd, 0, 100, 0.1f, 1, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_STEERING, "SERVO CENTER", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_I16, &car_params.servo_center_us, 1000, 3000, 5, 0, CAR_MENU_APPLY_SERVO_CENTER, 0},
    {0, CAR_MENU_PAGE_STEERING, "SERVO TRAVEL", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_I16, &car_params.servo_travel_us, 0, 800, 5, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_STEERING, "SERVO REVERSE", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_BOOL, &car_params.servo_reverse, 0, 1, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_STEERING, "SERVO COMMAND", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.servo_command_us, 0, 3000, 1, 0, CAR_MENU_APPLY_NONE, 0},

    {0, CAR_MENU_PAGE_VISION, "AUTO THRESHOLD", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_BOOL, &car_params.automatic_threshold, 0, 1, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "THRESHOLD", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.threshold, 0, 255, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "EXPOSURE", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U16, &car_params.exposure, 1, 4000, 16, 0, CAR_MENU_APPLY_EXPOSURE, 0},
    {0, CAR_MENU_PAGE_VISION, "GAIN", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.gain, 0, 64, 1, 0, CAR_MENU_APPLY_GAIN, 0},

    {0, CAR_MENU_PAGE_MOTORS, "LEFT DIRECTION", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_DIRECTION, &car_params.left_direction, -1, 1, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_MOTORS, "RIGHT DIRECTION", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_DIRECTION, &car_params.right_direction, -1, 1, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_MOTORS, "LEFT COMMAND", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.left_command, -10000, 10000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_MOTORS, "RIGHT COMMAND", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.right_command, -10000, 10000, 1, 0, CAR_MENU_APPLY_NONE, 0},

    {0, CAR_MENU_PAGE_TELEMETRY, "LINE VALID", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_BOOL, &car_result.line_valid, 0, 1, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "LOST COUNT", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U16, &car_result.lost_count, 0, 65535, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "THRESHOLD USED", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U8, &car_result.threshold_used, 0, 255, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "CENTER X", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_FLOAT, &car_result.center_x, 0, 188, 1, 1, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "ERROR PIXELS", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_FLOAT, &car_result.error_pixels, -188, 188, 1, 1, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "SERVO US", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.servo_command_us, 0, 3000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "LEFT CMD", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.left_command, -10000, 10000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "RIGHT CMD", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.right_command, -10000, 10000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "PROCESS US", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_result.processing_time_us, 0, 1000000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "PERIOD MS", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_frame_period_ms, 0, 1000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "FRAME SEQ", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_frame_sequence, 0, 4294967295.0f, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "WHITE LEFT LEN", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &Longest_White_Column_Left[0], 0, 120, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "WHITE LEFT COL", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &Longest_White_Column_Left[1], 0, 188, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "WHITE RIGHT LEN", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &Longest_White_Column_Right[0], 0, 120, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "WHITE RIGHT COL", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &Longest_White_Column_Right[1], 0, 188, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "CAMERA", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_BOOL, &car_camera_ready, 0, 1, 1, 0, CAR_MENU_APPLY_NONE, 0},

    {0, CAR_MENU_PAGE_TOOLS, "LCD TEST", CAR_MENU_ITEM_ACTION, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, CAR_MENU_APPLY_NONE, CAR_MENU_ACTION_LCD_TEST},
    {0, CAR_MENU_PAGE_TOOLS, "RESET DEFAULTS", CAR_MENU_ITEM_ACTION, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, CAR_MENU_APPLY_NONE, CAR_MENU_ACTION_RESET_DEFAULTS},
    {0, CAR_MENU_PAGE_TOOLS, "UART STREAM", CAR_MENU_ITEM_ACTION, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, CAR_MENU_APPLY_NONE, CAR_MENU_ACTION_UART}
};

static uint8_t car_menu_ready;
static uint8_t car_menu_gray_valid;
static uint8_t car_menu_new_frame;
static uint8_t car_menu_dirty;
static uint8_t car_menu_editing;
static uint8_t car_menu_last_keys;
static uint8_t car_menu_error;
static uint8_t car_menu_lcd_test_drawn;
static uint16_t car_menu_page;
static uint16_t car_menu_selected;
static uint16_t car_menu_first_visible;
static uint16_t car_menu_edit_item_number;
static float car_menu_edit_value;
static float car_menu_original_value;
static uint32_t car_menu_last_display_ms;
static uint32_t car_menu_lcd_test_until_ms;

static uint16_t car_menu_item_count(uint16_t parent)
{
    uint16_t count = 0U;
    uint16_t index;

    for (index = 0U; index < sizeof(car_menu_items) / sizeof(car_menu_items[0]); index++) {
        if (car_menu_items[index].parent == parent) count++;
    }
    return count;
}

static const car_menu_item_t *car_menu_item_at(uint16_t parent, uint16_t ordinal)
{
    uint16_t index;

    for (index = 0U; index < sizeof(car_menu_items) / sizeof(car_menu_items[0]); index++) {
        if (car_menu_items[index].parent != parent) continue;
        if (ordinal == 0U) return &car_menu_items[index];
        ordinal--;
    }
    return 0;
}

static uint16_t car_menu_page_parent(uint16_t page)
{
    uint16_t index;

    if (page == CAR_MENU_ROOT) return CAR_MENU_ROOT;
    for (index = 0U; index < sizeof(car_menu_items) / sizeof(car_menu_items[0]); index++) {
        if (car_menu_items[index].id == page && car_menu_items[index].item_type == CAR_MENU_ITEM_PAGE) {
            return car_menu_items[index].parent;
        }
    }
    return CAR_MENU_ROOT;
}

static const char *car_menu_page_name(uint16_t page)
{
    uint16_t index;

    if (page == CAR_MENU_ROOT) return "CAMERA4";
    for (index = 0U; index < sizeof(car_menu_items) / sizeof(car_menu_items[0]); index++) {
        if (car_menu_items[index].id == page && car_menu_items[index].item_type == CAR_MENU_ITEM_PAGE) {
            return car_menu_items[index].name;
        }
    }
    return "MENU";
}

static float car_menu_read_value(const car_menu_item_t *item)
{
    if (item == 0 || item->address == 0) return 0.0f;
    switch (item->value_type) {
        case CAR_MENU_VALUE_I16: return *(volatile int16_t *)item->address;
        case CAR_MENU_VALUE_U16: return *(volatile uint16_t *)item->address;
        case CAR_MENU_VALUE_U8: return *(volatile uint8_t *)item->address;
        case CAR_MENU_VALUE_I8: return *(volatile int8_t *)item->address;
        case CAR_MENU_VALUE_U32: return *(volatile uint32_t *)item->address;
        case CAR_MENU_VALUE_FLOAT: return *(volatile float *)item->address;
        case CAR_MENU_VALUE_BOOL: return *(volatile uint8_t *)item->address != 0U ? 1.0f : 0.0f;
        case CAR_MENU_VALUE_DIRECTION: return *(volatile int8_t *)item->address < 0 ? -1.0f : 1.0f;
        default: return 0.0f;
    }
}

static float car_menu_limit_value(const car_menu_item_t *item, float value)
{
    if (item == 0) return value;
    if (value < item->minimum) value = item->minimum;
    if (value > item->maximum) value = item->maximum;
    return value;
}

static void car_menu_write_value(const car_menu_item_t *item, float value)
{
    if (item == 0 || item->address == 0) return;
    value = car_menu_limit_value(item, value);
    switch (item->value_type) {
        case CAR_MENU_VALUE_I16: *(volatile int16_t *)item->address = (int16_t)value; break;
        case CAR_MENU_VALUE_U16: *(volatile uint16_t *)item->address = (uint16_t)value; break;
        case CAR_MENU_VALUE_U8: *(volatile uint8_t *)item->address = (uint8_t)value; break;
        case CAR_MENU_VALUE_I8: *(volatile int8_t *)item->address = (int8_t)value; break;
        case CAR_MENU_VALUE_U32: *(volatile uint32_t *)item->address = (uint32_t)value; break;
        case CAR_MENU_VALUE_FLOAT: *(volatile float *)item->address = value; break;
        case CAR_MENU_VALUE_BOOL: *(volatile uint8_t *)item->address = value >= 0.5f ? 1U : 0U; break;
        case CAR_MENU_VALUE_DIRECTION: *(volatile int8_t *)item->address = value < 0.0f ? -1 : 1; break;
        default: break;
    }
}

static void car_menu_apply_value(const car_menu_item_t *item, float value)
{
    if (item == 0) return;
    car_menu_write_value(item, value);
    if (item->apply == CAR_MENU_APPLY_NONE) return;

    car_apply_menu_params(&car_params);
    if (item->apply == CAR_MENU_APPLY_SERVO_CENTER && car_running == 0U) {
        cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    } else if (item->apply == CAR_MENU_APPLY_EXPOSURE) {
        cc_tc264_camera_set_exposure(car_params.exposure);
    } else if (item->apply == CAR_MENU_APPLY_GAIN) {
        cc_tc264_camera_set_gain(car_params.gain);
    }
}

static uint8_t car_menu_can_edit(const car_menu_item_t *item)
{
    return item != 0 && item->item_type == CAR_MENU_ITEM_VALUE && car_running == 0U ? 1U : 0U;
}

static void car_menu_begin_edit(void)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, car_menu_selected);

    if (car_menu_can_edit(item) == 0U) return;
    car_menu_editing = 1U;
    car_menu_edit_item_number = car_menu_selected;
    car_menu_edit_value = car_menu_read_value(item);
    car_menu_original_value = car_menu_edit_value;
    car_menu_dirty = 1U;
}

static void car_menu_adjust_edit(int8_t direction)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, car_menu_edit_item_number);
    float value;

    if (car_menu_can_edit(item) == 0U) return;
    value = car_menu_edit_value;
    if (item->value_type == CAR_MENU_VALUE_BOOL) value = value >= 0.5f ? 0.0f : 1.0f;
    else if (item->value_type == CAR_MENU_VALUE_DIRECTION) value = value < 0.0f ? 1.0f : -1.0f;
    else value += item->step * direction;
    value = car_menu_limit_value(item, value);
    car_menu_edit_value = value;
    car_menu_apply_value(item, value);
    car_menu_dirty = 1U;
}

static void car_menu_finish_edit(uint8_t save)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, car_menu_edit_item_number);

    if (save == 0U && item != 0) car_menu_apply_value(item, car_menu_original_value);
    car_menu_editing = 0U;
    car_menu_edit_item_number = 0U;
    car_menu_dirty = 1U;
}

static void car_menu_stop(void)
{
    car_stop();
    car_params.running = 0U;
    cc_tc264_menu_motor_stop();
    cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_dirty = 1U;
}

static void car_menu_start(void)
{
    if (cc_tc264_motor_ready() == 0U) {
        car_menu_error = CAR_MENU_ERROR_MOTOR;
        car_menu_dirty = 1U;
        return;
    }
    if (cc_tc264_camera_ready() == 0U) {
        car_menu_error = CAR_MENU_ERROR_CAMERA;
        car_menu_dirty = 1U;
        return;
    }
    car_apply_menu_params(&car_params);
    car_set_running(1U);
    if (car_running == 0U) {
        car_menu_error = CAR_MENU_ERROR_CAMERA;
        car_menu_dirty = 1U;
        return;
    }
    car_params.running = 1U;
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_dirty = 1U;
}

static void car_menu_reset_defaults(void)
{
    if (car_running != 0U) {
        car_menu_error = CAR_MENU_ERROR_RUNNING;
        car_menu_dirty = 1U;
        return;
    }
    car_menu_stop();
    car_params_reset();
    car_apply_menu_params(&car_params);
    cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    if (cc_tc264_camera_ready() != 0U) {
        cc_tc264_camera_set_exposure(car_params.exposure);
        cc_tc264_camera_set_gain(car_params.gain);
    }
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_dirty = 1U;
}

static void car_menu_execute_action(uint8_t action)
{
    if (action == CAR_MENU_ACTION_RUN) car_menu_start();
    else if (action == CAR_MENU_ACTION_STOP) car_menu_stop();
    else if (action == CAR_MENU_ACTION_RESET_DEFAULTS) car_menu_reset_defaults();
    else if (action == CAR_MENU_ACTION_LCD_TEST) {
        car_menu_lcd_test_until_ms = system_getval_ms() + CAR_MENU_LCD_TEST_MS;
        car_menu_lcd_test_drawn = 0U;
        car_menu_dirty = 1U;
    } else if (action == CAR_MENU_ACTION_UART) {
        if (car_running != 0U) car_menu_error = CAR_MENU_ERROR_RUNNING;
        else if (car_uart_stream_is_enabled() != 0U) car_uart_stream_stop();
        else car_uart_stream_start();
        car_menu_dirty = 1U;
    }
}

static void car_menu_emergency_stop(void)
{
    car_menu_stop();
}

static void car_menu_move(int8_t direction)
{
    uint16_t count = car_menu_item_count(car_menu_page);

    if (count == 0U) return;
    if (direction < 0) car_menu_selected = car_menu_selected == 0U ? count - 1U : car_menu_selected - 1U;
    else {
        car_menu_selected++;
        if (car_menu_selected >= count) car_menu_selected = 0U;
    }
    if (car_menu_selected < car_menu_first_visible) car_menu_first_visible = car_menu_selected;
    if (car_menu_selected >= car_menu_first_visible + CAR_MENU_VISIBLE_ROWS) {
        car_menu_first_visible = car_menu_selected - CAR_MENU_VISIBLE_ROWS + 1U;
    }
    car_menu_dirty = 1U;
}

static void car_menu_enter(void)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, car_menu_selected);

    if (item == 0) return;
    if (item->item_type == CAR_MENU_ITEM_PAGE) {
        car_menu_page = item->id;
        car_menu_selected = 0U;
        car_menu_first_visible = 0U;
        car_menu_dirty = 1U;
    } else if (item->item_type == CAR_MENU_ITEM_VALUE) car_menu_begin_edit();
    else if (item->item_type == CAR_MENU_ITEM_ACTION) car_menu_execute_action(item->action);
}

static void car_menu_back(void)
{
    if (car_menu_page == CAR_MENU_ROOT) return;
    car_menu_page = car_menu_page_parent(car_menu_page);
    car_menu_selected = 0U;
    car_menu_first_visible = 0U;
    car_menu_dirty = 1U;
}

static void car_menu_handle_input(uint8_t keys)
{
    uint8_t center = (keys & CC_KEY_CENTER_MASK) != 0U ? 1U : 0U;
    uint8_t left = (keys & CC_KEY_LEFT_MASK) != 0U ? 1U : 0U;
    uint8_t right = (keys & CC_KEY_RIGHT_MASK) != 0U ? 1U : 0U;
    uint8_t up = (keys & CC_KEY_UP_MASK) != 0U ? 1U : 0U;
    uint8_t down = (keys & CC_KEY_DOWN_MASK) != 0U ? 1U : 0U;

    if (car_running != 0U && center != 0U) {
        car_menu_emergency_stop();
        return;
    }
    if (car_menu_lcd_test_until_ms != 0U) {
        if (left != 0U || center != 0U || right != 0U || up != 0U || down != 0U) {
            car_menu_lcd_test_until_ms = 0U;
            car_menu_dirty = 1U;
        }
        return;
    }
    if (car_menu_editing != 0U) {
        if (up != 0U) car_menu_adjust_edit(1);
        if (down != 0U) car_menu_adjust_edit(-1);
        if (right != 0U || center != 0U) car_menu_finish_edit(1U);
        if (left != 0U) car_menu_finish_edit(0U);
        return;
    }
    if (left != 0U) car_menu_back();
    if (up != 0U) car_menu_move(-1);
    if (down != 0U) car_menu_move(1);
    if (right != 0U || center != 0U) car_menu_enter();
}

static const char *car_menu_status_text(void)
{
    if (car_running != 0U) return "RUN";
    if (car_menu_error != CAR_MENU_ERROR_NONE) return "ERROR";
    return "STOP";
}

static const char *car_menu_error_text(void)
{
    if (car_menu_error == CAR_MENU_ERROR_CAMERA) return "CAMERA OFFLINE";
    if (car_menu_error == CAR_MENU_ERROR_MOTOR) return "MOTOR NOT READY";
    if (car_menu_error == CAR_MENU_ERROR_RUNNING) return "STOP FIRST";
    if (car_menu_error == CAR_MENU_ERROR_LINE_LOST) return "LINE LOST";
    return "READY";
}

static void car_menu_fill(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    uint16_t row;

    for (row = 0U; row < height; row++) ips200_draw_line(x, y + row, x + width - 1U, y + row, color);
}

static void car_menu_text(uint16_t x, uint16_t y, const char *text, uint16_t foreground, uint16_t background)
{
    ips200_set_color(foreground, background);
    ips200_show_string(x, y, text);
}

static void car_menu_draw_header(void)
{
    const char *camera = car_camera_ready != 0U ? "CAM" : "OFF";

    car_menu_fill(0U, 0U, 320U, 24U, RGB565_BLACK);
    car_menu_text(4U, 4U, car_menu_page_name(car_menu_page), RGB565_WHITE, RGB565_BLACK);
    car_menu_text(224U, 4U, car_menu_status_text(), RGB565_WHITE, RGB565_BLACK);
    car_menu_text(280U, 4U, camera, RGB565_WHITE, RGB565_BLACK);
}

static void car_menu_draw_value(const car_menu_item_t *item, float value, uint16_t x, uint16_t y, uint16_t foreground, uint16_t background)
{
    if (item == 0) return;
    ips200_set_color(foreground, background);
    if (item->value_type == CAR_MENU_VALUE_BOOL) {
        ips200_show_string(x, y, value >= 0.5f ? "ON" : "OFF");
    } else if (item->value_type == CAR_MENU_VALUE_DIRECTION) {
        ips200_show_string(x, y, value < 0.0f ? "REV" : "FWD");
    } else if (item->value_type == CAR_MENU_VALUE_FLOAT) {
        ips200_show_float(x, y, value, 7U, item->decimals);
    } else if (item->value_type == CAR_MENU_VALUE_I16 || item->value_type == CAR_MENU_VALUE_I8) {
        ips200_show_int(x, y, (int32_t)value, 7U);
    } else {
        ips200_show_uint(x, y, (uint32_t)value, 7U);
    }
}

static void car_menu_draw_footer(void)
{
    const char *text;

    car_menu_fill(0U, 216U, 320U, 24U, RGB565_BLACK);
    if (car_menu_error != CAR_MENU_ERROR_NONE && car_menu_editing == 0U) text = car_menu_error_text();
    else if (car_menu_editing != 0U) text = "UP/DN ADJUST RIGHT SAVE LEFT CANCEL";
    else if (car_menu_page == CAR_MENU_PAGE_IMAGE) text = "LEFT BACK";
    else if (car_running != 0U) text = "CENTER EMERGENCY STOP LEFT BACK";
    else text = "UP/DN MOVE RIGHT/CENTER ENTER LEFT BACK";
    car_menu_text(2U, 220U, text, RGB565_WHITE, RGB565_BLACK);
}

static void car_menu_draw_list(void)
{
    uint16_t row;
    uint16_t y;
    const car_menu_item_t *item;
    uint16_t foreground;
    uint16_t background;
    float value;

    ips200_full(RGB565_BLACK);
    car_menu_draw_header();
    for (row = 0U; row < CAR_MENU_VISIBLE_ROWS; row++) {
        item = car_menu_item_at(car_menu_page, car_menu_first_visible + row);
        if (item == 0) break;
        y = 28U + row * 23U;
        background = row + car_menu_first_visible == car_menu_selected ? RGB565_BLUE : RGB565_BLACK;
        foreground = RGB565_WHITE;
        car_menu_fill(0U, y, 320U, 21U, background);
        car_menu_text(6U, y + 3U, row + car_menu_first_visible == car_menu_selected ? ">" : " ", foreground, background);
        car_menu_text(20U, y + 3U, item->name, foreground, background);
        if (item->item_type == CAR_MENU_ITEM_PAGE) {
            car_menu_text(304U, y + 3U, ">", foreground, background);
        } else if (item->item_type == CAR_MENU_ITEM_VALUE || item->item_type == CAR_MENU_ITEM_INFO) {
            value = car_menu_editing != 0U && row + car_menu_first_visible == car_menu_edit_item_number ? car_menu_edit_value : car_menu_read_value(item);
            car_menu_draw_value(item, value, 228U, y + 3U, foreground, background);
        } else if (item->item_type == CAR_MENU_ITEM_ACTION) {
            car_menu_text(304U, y + 3U, ">", foreground, background);
        }
    }
    car_menu_draw_footer();
}

static void car_menu_draw_image(void)
{
    ips200_full(RGB565_BLACK);
    car_menu_draw_header();
    car_menu_text(4U, 27U, "RAW", RGB565_WHITE, RGB565_BLACK);
    car_menu_text(168U, 27U, "BINARY", RGB565_WHITE, RGB565_BLACK);
    if (car_menu_gray_valid != 0U) {
        ips200_show_gray_image(4U, 44U, &car_gray_frame[0][0], CAR_IMAGE_WIDTH, CAR_IMAGE_HEIGHT, 148U, 94U, 0U);
    } else {
        car_menu_text(32U, 84U, "NO RAW FRAME", RGB565_WHITE, RGB565_BLACK);
    }
    if (binary_image != 0) {
        ips200_show_binary_image(168U, 44U, binary_image, CAR_IMAGE_WIDTH, CAR_IMAGE_HEIGHT, 148U, 94U);
    } else {
        car_menu_text(194U, 84U, "NO BINARY", RGB565_WHITE, RGB565_BLACK);
    }
    car_menu_text(4U, 146U, "GRAY/BINARY FRAME", RGB565_WHITE, RGB565_BLACK);
    car_menu_draw_footer();
}

static void car_menu_draw_lcd_test(void)
{
    car_menu_fill(0U, 0U, 64U, 240U, 0xF800U);
    car_menu_fill(64U, 0U, 64U, 240U, 0x07E0U);
    car_menu_fill(128U, 0U, 64U, 240U, 0x001FU);
    car_menu_fill(192U, 0U, 64U, 240U, 0xFFFFU);
    car_menu_fill(256U, 0U, 64U, 240U, RGB565_BLACK);
    car_menu_text(92U, 108U, "LCD TEST", RGB565_WHITE, RGB565_BLACK);
}

void car_menu_init(void)
{
    uint8_t camera_ready;

    cc_tc264_board_init();
    car_init();
    car_shared_clear();
    car_params_reset();
    car_apply_menu_params(&car_params);
    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);
    car_uart_stream_init();

    camera_ready = cc_tc264_camera_init();
    car_set_camera_ready(camera_ready);
    if (camera_ready != 0U) {
        cc_tc264_camera_set_exposure(car_params.exposure);
        cc_tc264_camera_set_gain(car_params.gain);
    }
    cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    gpio_set_level(BOARD_LED1_PIN, camera_ready != 0U ? GPIO_LOW : GPIO_HIGH);
    gpio_set_level(BOARD_LED2_PIN, GPIO_HIGH);

    car_menu_page = CAR_MENU_ROOT;
    car_menu_selected = 0U;
    car_menu_first_visible = 0U;
    car_menu_gray_valid = 0U;
    car_menu_new_frame = 0U;
    car_menu_editing = 0U;
    car_menu_edit_item_number = 0U;
    car_menu_edit_value = 0.0f;
    car_menu_original_value = 0.0f;
    car_menu_last_display_ms = 0U;
    car_menu_lcd_test_until_ms = 0U;
    car_menu_lcd_test_drawn = 0U;
    car_menu_last_keys = cc_tc264_menu_key_mask();
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_ready = 1U;
    car_menu_dirty = 1U;
    car_menu_display();
}

void car_menu_frame_accepted(uint32_t frame_period_ms)
{
    (void)frame_period_ms;
    if (car_menu_ready == 0U) return;
    car_menu_gray_valid = 1U;
}

void car_menu_result_accepted(void)
{
    if (car_menu_ready == 0U) return;
    car_menu_new_frame = 1U;
    car_menu_dirty = 1U;
}

void car_menu_task(void)
{
    uint8_t key_mask;
    uint8_t pressed;
    uint32_t now;

    if (car_menu_ready == 0U) return;
    key_mask = cc_tc264_menu_key_mask();
    pressed = key_mask & (uint8_t)~car_menu_last_keys;
    car_menu_last_keys = key_mask;
    now = system_getval_ms();

    if (car_menu_lcd_test_until_ms != 0U && now >= car_menu_lcd_test_until_ms) {
        car_menu_lcd_test_until_ms = 0U;
        car_menu_dirty = 1U;
    }
    car_menu_handle_input(pressed);
    if (car_params.running != 0U && car_running == 0U) {
        car_params.running = 0U;
        car_menu_error = CAR_MENU_ERROR_LINE_LOST;
        car_menu_dirty = 1U;
    }
    if (car_menu_new_frame != 0U && car_menu_lcd_test_until_ms == 0U && car_frame_ready == 0U && car_result_ready == 0U) {
        if (car_uart_stream_is_enabled() != 0U) car_uart_stream_send_frame(&car_gray_frame[0][0]);
        car_menu_new_frame = 0U;
    }
    if (car_menu_lcd_test_until_ms == 0U && now - car_menu_last_display_ms >= CAR_MENU_REFRESH_MS) car_menu_dirty = 1U;
    car_menu_display();
}

void car_menu_display(void)
{
    uint32_t now;

    if (car_menu_ready == 0U) return;
    if (car_frame_ready != 0U || car_result_ready != 0U) return;
    now = system_getval_ms();
    if (car_menu_lcd_test_until_ms != 0U) {
        if (car_menu_lcd_test_drawn == 0U) {
            car_menu_draw_lcd_test();
            car_menu_lcd_test_drawn = 1U;
        }
        car_menu_last_display_ms = now;
        car_menu_dirty = 0U;
        return;
    }
    car_menu_lcd_test_drawn = 0U;
    if (car_menu_dirty == 0U) return;
    if (car_menu_page == CAR_MENU_PAGE_IMAGE) car_menu_draw_image();
    else car_menu_draw_list();
    car_menu_last_display_ms = now;
    car_menu_dirty = 0U;
}
