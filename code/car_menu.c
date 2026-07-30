#include "car_menu.h"

#include <stdint.h>
#include <string.h>

#include "board_pins.h"
#include "car.h"
#include "car_menu_port.h"
#include "car_params.h"
#include "car_shared.h"
#include "car_uart_stream.h"
#include "longest_white.h"
#include "zf_common_headfile.h"

enum {
    CAR_MENU_ROOT = 0,
    CAR_MENU_PAGE_RUN = 1,
    CAR_MENU_PAGE_STEERING = 2,
    CAR_MENU_PAGE_VISION = 3,
    CAR_MENU_PAGE_MOTORS = 4,
    CAR_MENU_PAGE_TELEMETRY = 5,
    CAR_MENU_PAGE_IMAGE = 6,
    CAR_MENU_PAGE_TOOLS = 7
};

enum {
    CAR_MENU_ACTION_RUN = 1,
    CAR_MENU_ACTION_STOP,
    CAR_MENU_ACTION_LCD_TEST,
    CAR_MENU_ACTION_RESET_DEFAULTS,
    CAR_MENU_ACTION_UART
};

enum {
    CAR_MENU_ITEM_PAGE = 1,
    CAR_MENU_ITEM_VALUE,
    CAR_MENU_ITEM_INFO,
    CAR_MENU_ITEM_ACTION
};

enum {
    CAR_MENU_VALUE_NONE = 0,
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
    CAR_MENU_APPLY_NONE = 0,
    CAR_MENU_APPLY_PARAMS,
    CAR_MENU_APPLY_SERVO_CENTER,
    CAR_MENU_APPLY_EXPOSURE,
    CAR_MENU_APPLY_GAIN
};

enum {
    CAR_MENU_ERROR_NONE = 0,
    CAR_MENU_ERROR_CAMERA,
    CAR_MENU_ERROR_MOTOR,
    CAR_MENU_ERROR_RUNNING,
    CAR_MENU_ERROR_LINE_LOST
};

#define CAR_MENU_VISIBLE_ROWS             (8)
#define CAR_MENU_KEY_COUNT                (5)
#define CAR_MENU_KEY_SCAN_MS              (10)
#define CAR_MENU_LONG_PRESS_MS            (1000)
#define CAR_MENU_KEY_REPEAT_MS            (100)
#define CAR_MENU_IMAGE_REFRESH_MS         (50)
#define CAR_MENU_TELEMETRY_REFRESH_MS     (50)
#define CAR_MENU_LCD_TEST_MS              (1500)
#define CAR_MENU_START_DELAY_MS           (3000)

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

    {0, CAR_MENU_PAGE_STEERING, "STEERING KP", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_FLOAT, &car_params.steering_kp, 0, 10000, 0.1, 1, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_STEERING, "STEERING KD", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_FLOAT, &car_params.steering_kd, 0, 10000, 0.1, 1, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_STEERING, "SERVO CENTER", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_I16, &car_params.servo_center_us, 1400, 1600, 5, 0, CAR_MENU_APPLY_SERVO_CENTER, 0},
    {0, CAR_MENU_PAGE_STEERING, "SERVO TRAVEL", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_I16, &car_params.servo_travel_us, 0, 200, 5, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_STEERING, "SERVO REVERSE", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_BOOL, &car_params.servo_reverse, 0, 1, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_STEERING, "SERVO COMMAND", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.servo_command_us, 0, 3000, 1, 0, CAR_MENU_APPLY_NONE, 0},

    {0, CAR_MENU_PAGE_VISION, "AUTO THRESHOLD", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_BOOL, &car_params.automatic_threshold, 0, 1, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "CROSS ENABLE", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_BOOL, &car_params.cross_enabled, 0, 1, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "CROSS LOST MIN", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.cross_min_both_lost, 1, 120, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "CROSS COLUMN MIN", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U16, &car_params.cross_min_white_column, 5, 120, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "CROSS EDGE DIFF", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.cross_edge_stable_diff, 0, 30, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "CROSS TEAR FIRST", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.cross_tear_diff_first, 0, 80, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "CROSS TEAR SECOND", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.cross_tear_diff_second, 0, 120, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "CROSS ROW GAP", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.cross_corner_row_gap_max, 1, 120, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "CROSS LOST MAX", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U16, &car_params.cross_max_lost_rows, 1, 120, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "THRESHOLD", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.threshold, 0, 255, 1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_VISION, "EXPOSURE", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U16, &car_params.exposure, 1, 4000, 16, 0, CAR_MENU_APPLY_EXPOSURE, 0},
    {0, CAR_MENU_PAGE_VISION, "GAIN", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_U8, &car_params.gain, 0, 64, 1, 0, CAR_MENU_APPLY_GAIN, 0},

    {0, CAR_MENU_PAGE_MOTORS, "LEFT DIRECTION", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_DIRECTION, &car_params.left_direction, -1, 1, -1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_MOTORS, "RIGHT DIRECTION", CAR_MENU_ITEM_VALUE, CAR_MENU_VALUE_DIRECTION, &car_params.right_direction, -1, 1, -1, 0, CAR_MENU_APPLY_PARAMS, 0},
    {0, CAR_MENU_PAGE_MOTORS, "LEFT COMMAND", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.left_command, -10000, 10000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_MOTORS, "RIGHT COMMAND", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.right_command, -10000, 10000, 1, 0, CAR_MENU_APPLY_NONE, 0},

    {0, CAR_MENU_PAGE_TELEMETRY, "LINE VALID", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_BOOL, &car_result.line_valid, 0, 1, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "CROSS FLAG", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_BOOL, &car_result.cross_detected, 0, 1, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "LOST COUNT", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U16, &car_result.lost_count, 0, 65535, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "THRESHOLD USED", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U8, &car_result.threshold_used, 0, 255, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "CENTER X", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_FLOAT, &car_result.center_x, 0, 188, 1, 1, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "ERROR PIXELS", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_FLOAT, &car_result.error_pixels, -188, 188, 1, 1, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "SERVO US", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.servo_command_us, 0, 3000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "LEFT CMD", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.left_command, -10000, 10000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "RIGHT CMD", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &car_result.right_command, -10000, 10000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "PROCESS US", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_result.processing_time_us, 0, 1000000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "PERIOD MS", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_frame_period_ms, 0, 1000, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "FRAME SEQ", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_frame_sequence, 0, 4294967295.0, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "WHITE LEFT LEN", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &Longest_White_Column_Left[0], 0, 120, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "WHITE LEFT COL", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &Longest_White_Column_Left[1], 0, 188, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "WHITE RIGHT LEN", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &Longest_White_Column_Right[0], 0, 120, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "WHITE RIGHT COL", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_I16, &Longest_White_Column_Right[1], 0, 188, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "CAPTURE DROP", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_capture_drop_count, 0, 4294967295.0, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "PROCESS DROP", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_processing_drop_count, 0, 4294967295.0, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "DISPLAY DROP", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_display_drop_count, 0, 4294967295.0, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "RESULT DROP", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_result_drop_count, 0, 4294967295.0, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "VISION MAX US", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_vision_max_us, 0, 4294967295.0, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "CAM FRAME CNT", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_camera_frame_count, 0, 4294967295.0, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "CAM AGE MS", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_U32, &car_camera_age_ms, 0, 4294967295.0, 1, 0, CAR_MENU_APPLY_NONE, 0},
    {0, CAR_MENU_PAGE_TELEMETRY, "CAMERA", CAR_MENU_ITEM_INFO, CAR_MENU_VALUE_BOOL, &car_camera_ready, 0, 1, 1, 0, CAR_MENU_APPLY_NONE, 0},

    {0, CAR_MENU_PAGE_TOOLS, "LCD TEST", CAR_MENU_ITEM_ACTION, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, CAR_MENU_APPLY_NONE, CAR_MENU_ACTION_LCD_TEST},
    {0, CAR_MENU_PAGE_TOOLS, "RESET DEFAULTS", CAR_MENU_ITEM_ACTION, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, CAR_MENU_APPLY_NONE, CAR_MENU_ACTION_RESET_DEFAULTS},
    {0, CAR_MENU_PAGE_TOOLS, "UART STREAM", CAR_MENU_ITEM_ACTION, CAR_MENU_VALUE_NONE, 0, 0, 0, 0, 0, CAR_MENU_APPLY_NONE, CAR_MENU_ACTION_UART}
};

static uint8_t car_menu_ready;
static uint8_t car_menu_new_frame;
static uint8_t car_menu_image_pending;
static uint8_t car_menu_dirty;
static uint8_t car_menu_row_dirty;
static uint8_t car_menu_footer_dirty;
static uint8_t car_menu_editing;
static uint8_t car_menu_previous_keys;
static uint8_t car_menu_long_latched[CAR_MENU_KEY_COUNT];
static uint8_t car_menu_error;
static uint8_t car_menu_lcd_test_drawn;
static uint16_t car_menu_page;
static uint16_t car_menu_selected;
static uint16_t car_menu_first_visible;
static uint16_t car_menu_old_selected;
static uint16_t car_menu_edit_item_number;
static float car_menu_edit_value;
static float car_menu_original_value;
static uint32_t car_menu_key_pressed_ms[CAR_MENU_KEY_COUNT];
static uint32_t car_menu_key_repeat_ms[CAR_MENU_KEY_COUNT];
static uint32_t car_menu_last_key_scan_ms;
static uint32_t car_menu_last_image_display_ms;
static uint32_t car_menu_last_telemetry_display_ms;
static uint32_t car_menu_lcd_test_until_ms;
static uint8_t car_menu_start_pending;
static uint32_t car_menu_start_ms;

static uint16_t car_menu_item_count(uint16_t parent)
{
    uint16_t count = 0;
    uint16_t index;

    for (index = 0; index < sizeof(car_menu_items) / sizeof(car_menu_items[0]); index++) {
        if (car_menu_items[index].parent == parent) count++;
    }
    return count;
}

static const car_menu_item_t *car_menu_item_at(uint16_t parent, uint16_t ordinal)
{
    uint16_t index;

    for (index = 0; index < sizeof(car_menu_items) / sizeof(car_menu_items[0]); index++) {
        if (car_menu_items[index].parent != parent) continue;
        if (ordinal == 0) return &car_menu_items[index];
        ordinal--;
    }
    return 0;
}

static uint16_t car_menu_page_parent(uint16_t page)
{
    uint16_t index;

    if (page == CAR_MENU_ROOT) return CAR_MENU_ROOT;
    for (index = 0; index < sizeof(car_menu_items) / sizeof(car_menu_items[0]); index++) {
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
    for (index = 0; index < sizeof(car_menu_items) / sizeof(car_menu_items[0]); index++) {
        if (car_menu_items[index].id == page && car_menu_items[index].item_type == CAR_MENU_ITEM_PAGE) {
            return car_menu_items[index].name;
        }
    }
    return "MENU";
}

static float car_menu_read_value(const car_menu_item_t *item)
{
    if (item == 0 || item->address == 0) return 0.0;
    switch (item->value_type) {
        case CAR_MENU_VALUE_I16: return *(volatile int16_t *)item->address;
        case CAR_MENU_VALUE_U16: return *(volatile uint16_t *)item->address;
        case CAR_MENU_VALUE_U8: return *(volatile uint8_t *)item->address;
        case CAR_MENU_VALUE_I8: return *(volatile int8_t *)item->address;
        case CAR_MENU_VALUE_U32: return *(volatile uint32_t *)item->address;
        case CAR_MENU_VALUE_FLOAT: return *(volatile float *)item->address;
        case CAR_MENU_VALUE_BOOL: return *(volatile uint8_t *)item->address != 0 ? 1.0 : 0.0;
        case CAR_MENU_VALUE_DIRECTION: return *(volatile int8_t *)item->address < 0 ? -1.0 : 1.0;
        default: return 0.0;
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
        case CAR_MENU_VALUE_BOOL: *(volatile uint8_t *)item->address = value >= 0.5 ? 1 : 0; break;
        case CAR_MENU_VALUE_DIRECTION: *(volatile int8_t *)item->address = value < 0.0 ? -1 : 1; break;
        default: break;
    }
}

static void car_menu_apply_value(const car_menu_item_t *item, float value)
{
    if (item == 0) return;
    car_menu_write_value(item, value);
    if (item->apply == CAR_MENU_APPLY_NONE) return;

    car_apply_menu_params(&car_params);
    if (item->apply == CAR_MENU_APPLY_SERVO_CENTER && car_running == 0) {
        cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    } else if (item->apply == CAR_MENU_APPLY_EXPOSURE) {
        cc_tc264_camera_set_exposure(car_params.exposure);
    } else if (item->apply == CAR_MENU_APPLY_GAIN) {
        cc_tc264_camera_set_gain(car_params.gain);
    }
}

static uint8_t car_menu_can_edit(const car_menu_item_t *item)
{
    return item != 0 && item->item_type == CAR_MENU_ITEM_VALUE && car_running == 0 && car_menu_start_pending == 0 ? 1 : 0;
}

static void car_menu_begin_edit(void)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, car_menu_selected);

    if (car_menu_can_edit(item) == 0) return;
    car_menu_editing = 1;
    car_menu_edit_item_number = car_menu_selected;
    car_menu_edit_value = car_menu_read_value(item);
    car_menu_original_value = car_menu_edit_value;
    car_menu_dirty = 1;
}

static void car_menu_adjust_edit(int8_t direction)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, car_menu_edit_item_number);
    float value;

    if (car_menu_can_edit(item) == 0) return;
    value = car_menu_edit_value;
    if (item->value_type == CAR_MENU_VALUE_BOOL) value = value >= 0.5 ? 0.0 : 1.0;
    else if (item->value_type == CAR_MENU_VALUE_DIRECTION) value = value < 0.0 ? 1.0 : -1.0;
    else value += item->step * direction;
    value = car_menu_limit_value(item, value);
    car_menu_edit_value = value;
    car_menu_apply_value(item, value);
    car_menu_old_selected = car_menu_selected;
    car_menu_row_dirty = 1;
}

static void car_menu_finish_edit(uint8_t save)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, car_menu_edit_item_number);

    if (save == 0 && item != 0) car_menu_apply_value(item, car_menu_original_value);
    car_menu_editing = 0;
    car_menu_edit_item_number = 0;
    car_menu_row_dirty = 1;
    car_menu_footer_dirty = 1;
}

static void car_menu_stop(void)
{
    car_menu_start_pending = 0;
    car_control_clear_latched_result();
    car_stop();
    car_params.running = 0;
    cc_tc264_menu_motor_stop();
    cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_dirty = 1;
}

static void car_menu_start(void)
{
    if (car_running != 0) {
        car_menu_error = CAR_MENU_ERROR_RUNNING;
        car_menu_dirty = 1;
        return;
    }
    if (cc_tc264_motor_ready() == 0) {
        car_menu_error = CAR_MENU_ERROR_MOTOR;
        car_menu_dirty = 1;
        return;
    }
    if (cc_tc264_camera_ready() == 0) {
        car_menu_error = CAR_MENU_ERROR_CAMERA;
        car_menu_dirty = 1;
        return;
    }
    car_apply_menu_params(&car_params);
    car_control_clear_latched_result();
    car_stop();
    car_menu_start_pending = 1;
    car_menu_start_ms = system_getval_ms();
    car_params.running = 0;
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_dirty = 1;
}

static void car_menu_update_start(uint32_t now)
{
    if (car_menu_start_pending == 0) return;
    if (cc_tc264_motor_ready() == 0) {
        car_menu_start_pending = 0;
        car_control_clear_latched_result();
        car_stop();
        car_menu_error = CAR_MENU_ERROR_MOTOR;
        car_menu_dirty = 1;
        return;
    }
    if (cc_tc264_camera_ready() == 0) {
        car_menu_start_pending = 0;
        car_control_clear_latched_result();
        car_stop();
        car_menu_error = CAR_MENU_ERROR_CAMERA;
        car_menu_dirty = 1;
        return;
    }
    if (now - car_menu_start_ms < CAR_MENU_START_DELAY_MS) return;

    car_menu_start_pending = 0;
    car_apply_menu_params(&car_params);
    car_control_clear_latched_result();
    car_set_running(1);
    if (car_running == 0) {
        car_control_clear_latched_result();
        car_menu_error = CAR_MENU_ERROR_CAMERA;
        car_params.running = 0;
        car_menu_dirty = 1;
        return;
    }
    car_params.running = 1;
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_dirty = 1;
}

static void car_menu_reset_defaults(void)
{
    if (car_running != 0) {
        car_menu_error = CAR_MENU_ERROR_RUNNING;
        car_menu_dirty = 1;
        return;
    }
    car_menu_stop();
    car_params_reset();
    car_apply_menu_params(&car_params);
    cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    if (cc_tc264_camera_ready() != 0) {
        cc_tc264_camera_set_exposure(car_params.exposure);
        cc_tc264_camera_set_gain(car_params.gain);
    }
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_dirty = 1;
}

static void car_menu_execute_action(uint8_t action)
{
    if (action == CAR_MENU_ACTION_RUN) car_menu_start();
    else if (action == CAR_MENU_ACTION_STOP) car_menu_stop();
    else if (action == CAR_MENU_ACTION_RESET_DEFAULTS) car_menu_reset_defaults();
    else if (action == CAR_MENU_ACTION_LCD_TEST) {
        car_menu_lcd_test_until_ms = system_getval_ms() + CAR_MENU_LCD_TEST_MS;
        car_menu_lcd_test_drawn = 0;
        car_menu_dirty = 1;
    } else if (action == CAR_MENU_ACTION_UART) {
        if (car_running != 0) car_menu_error = CAR_MENU_ERROR_RUNNING;
        else if (car_uart_stream_is_enabled() != 0) car_uart_stream_stop();
        else car_uart_stream_start();
        car_menu_dirty = 1;
    }
}

static void car_menu_emergency_stop(void)
{
    car_menu_stop();
}

static void car_menu_move(int8_t direction)
{
    uint16_t count = car_menu_item_count(car_menu_page);
    uint16_t old_first_visible;

    if (count == 0) return;
    car_menu_old_selected = car_menu_selected;
    old_first_visible = car_menu_first_visible;
    if (direction < 0) car_menu_selected = car_menu_selected == 0 ? count - 1 : car_menu_selected - 1;
    else {
        car_menu_selected++;
        if (car_menu_selected >= count) car_menu_selected = 0;
    }
    if (car_menu_selected < car_menu_first_visible) car_menu_first_visible = car_menu_selected;
    if (car_menu_selected >= car_menu_first_visible + CAR_MENU_VISIBLE_ROWS) {
        car_menu_first_visible = car_menu_selected - CAR_MENU_VISIBLE_ROWS + 1;
    }
    if (old_first_visible != car_menu_first_visible) car_menu_dirty = 1;
    else car_menu_row_dirty = 1;
}

static void car_menu_enter(void)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, car_menu_selected);

    if (item == 0) return;
    if (item->item_type == CAR_MENU_ITEM_PAGE) {
        car_menu_page = item->id;
        car_menu_selected = 0;
        car_menu_first_visible = 0;
        car_menu_dirty = 1;
    } else if (item->item_type == CAR_MENU_ITEM_VALUE) car_menu_begin_edit();
    else if (item->item_type == CAR_MENU_ITEM_ACTION) car_menu_execute_action(item->action);
}

static void car_menu_back(void)
{
    if (car_menu_page == CAR_MENU_ROOT) return;
    car_menu_page = car_menu_page_parent(car_menu_page);
    car_menu_selected = 0;
    car_menu_first_visible = 0;
    car_menu_old_selected = 0;
    car_menu_dirty = 1;
}

static void car_menu_handle_input(uint8_t short_keys, uint8_t long_keys, uint8_t repeat_keys)
{
    uint8_t action_keys = short_keys | long_keys;
    uint8_t adjust_keys = short_keys | repeat_keys;
    uint8_t center = action_keys & CC_KEY_CENTER_MASK;
    uint8_t left = action_keys & CC_KEY_LEFT_MASK;
    uint8_t right = action_keys & CC_KEY_RIGHT_MASK;
    uint8_t up = adjust_keys & CC_KEY_UP_MASK;
    uint8_t down = adjust_keys & CC_KEY_DOWN_MASK;

    if ((car_running != 0 || car_menu_start_pending != 0) && center != 0) {
        car_menu_emergency_stop();
        return;
    }
    if (car_menu_lcd_test_until_ms != 0) {
        if (action_keys != 0 || adjust_keys != 0) {
            car_menu_lcd_test_until_ms = 0;
            car_menu_dirty = 1;
        }
        return;
    }
    if (car_menu_editing != 0) {
        if (up != 0) car_menu_adjust_edit(1);
        if (down != 0) car_menu_adjust_edit(-1);
        if (right != 0 || center != 0) car_menu_finish_edit(1);
        if (left != 0) car_menu_finish_edit(0);
        return;
    }
    if (left != 0) car_menu_back();
    if (up != 0) car_menu_move(-1);
    if (down != 0) car_menu_move(1);
    if (right != 0 || center != 0) car_menu_enter();
}

static void car_menu_process_key_scan(uint32_t now)
{
    uint8_t key_mask;
    uint8_t short_keys = 0;
    uint8_t long_keys = 0;
    uint8_t repeat_keys = 0;
    uint8_t key_bit;
    uint8_t index;

    if (now - car_menu_last_key_scan_ms < CAR_MENU_KEY_SCAN_MS) return;
    car_menu_last_key_scan_ms = now;
    key_mask = cc_tc264_menu_key_mask();

    for (index = 0; index < CAR_MENU_KEY_COUNT; index++) {
        key_bit = (uint8_t)(1 << index);
        if ((key_mask & key_bit) != 0 && (car_menu_previous_keys & key_bit) == 0) {
            car_menu_key_pressed_ms[index] = now;
            car_menu_key_repeat_ms[index] = now;
            car_menu_long_latched[index] = 0;
        } else if ((key_mask & key_bit) == 0 && (car_menu_previous_keys & key_bit) != 0) {
            if (car_menu_long_latched[index] == 0) short_keys |= key_bit;
            car_menu_long_latched[index] = 0;
        } else if ((key_mask & key_bit) != 0 && now - car_menu_key_pressed_ms[index] >= CAR_MENU_LONG_PRESS_MS) {
            if (index < 2) {
                if (now - car_menu_key_repeat_ms[index] >= CAR_MENU_KEY_REPEAT_MS) {
                    repeat_keys |= key_bit;
                    car_menu_key_repeat_ms[index] = now;
                    car_menu_long_latched[index] = 1;
                }
            } else if (car_menu_long_latched[index] == 0) {
                long_keys |= key_bit;
                car_menu_long_latched[index] = 1;
            }
        }
    }

    car_menu_previous_keys = key_mask;
    car_menu_handle_input(short_keys, long_keys, repeat_keys);
}

static const char *car_menu_status_text(void)
{
    if (car_running != 0) return "RUN";
    if (car_menu_start_pending != 0) return "ARMING";
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
    ips200_fill_rect(x, y, width, height, color);
}

static void car_menu_text(uint16_t x, uint16_t y, const char *text, uint16_t foreground, uint16_t background)
{
    ips200_set_color(foreground, background);
    ips200_show_string(x, y, text);
}

static void car_menu_draw_header(void)
{
    const char *camera = car_camera_ready != 0 ? "CAM" : "OFF";

    car_menu_fill(0, 0, 320, 24, RGB565_BLACK);
    car_menu_text(4, 4, car_menu_page_name(car_menu_page), RGB565_WHITE, RGB565_BLACK);
    car_menu_text(224, 4, car_menu_status_text(), RGB565_WHITE, RGB565_BLACK);
    car_menu_text(280, 4, camera, RGB565_WHITE, RGB565_BLACK);
}

static void car_menu_draw_value(const car_menu_item_t *item, float value, uint16_t x, uint16_t y, uint16_t foreground, uint16_t background)
{
    if (item == 0) return;
    ips200_set_color(foreground, background);
    if (item->value_type == CAR_MENU_VALUE_BOOL) {
        ips200_show_string(x, y, value >= 0.5 ? "ON" : "OFF");
    } else if (item->value_type == CAR_MENU_VALUE_DIRECTION) {
        ips200_show_string(x, y, value < 0.0 ? "REV" : "FWD");
    } else if (item->value_type == CAR_MENU_VALUE_FLOAT) {
        ips200_show_float(x, y, value, 7, item->decimals);
    } else if (item->value_type == CAR_MENU_VALUE_I16 || item->value_type == CAR_MENU_VALUE_I8) {
        ips200_show_int(x, y, (int32_t)value, 7);
    } else {
        ips200_show_uint(x, y, (uint32_t)value, 7);
    }
}

static void car_menu_draw_footer(void)
{
    const char *text;

    car_menu_fill(0, 216, 320, 24, RGB565_BLACK);
    if (car_menu_error != CAR_MENU_ERROR_NONE && car_menu_editing == 0) text = car_menu_error_text();
    else if (car_menu_editing != 0) text = "UP/DN ADJUST RIGHT SAVE LEFT CANCEL";
    else if (car_menu_page == CAR_MENU_PAGE_IMAGE) text = "LEFT BACK";
    else if (car_running != 0) text = "CENTER EMERGENCY STOP LEFT BACK";
    else if (car_menu_start_pending != 0) text = "WAIT 3S CENTER STOP LEFT BACK";
    else text = "UP/DN MOVE RIGHT/CENTER ENTER LEFT BACK";
    car_menu_text(2, 220, text, RGB565_WHITE, RGB565_BLACK);
}

static void car_menu_draw_row_item(uint16_t item_number)
{
    const car_menu_item_t *item = car_menu_item_at(car_menu_page, item_number);
    uint16_t row;
    uint16_t y;
    uint16_t foreground;
    uint16_t background;
    float value;

    if (item == 0 || item_number < car_menu_first_visible || item_number >= car_menu_first_visible + CAR_MENU_VISIBLE_ROWS) return;
    row = item_number - car_menu_first_visible;
    y = 28 + row * 23;
    background = item_number == car_menu_selected ? RGB565_BLUE : RGB565_BLACK;
    foreground = RGB565_WHITE;
    car_menu_fill(0, y, 320, 21, background);
    car_menu_text(6, y + 3, item_number == car_menu_selected ? ">" : " ", foreground, background);
    car_menu_text(20, y + 3, item->name, foreground, background);
    if (item->item_type == CAR_MENU_ITEM_PAGE || item->item_type == CAR_MENU_ITEM_ACTION) {
        car_menu_text(304, y + 3, ">", foreground, background);
    } else if (item->item_type == CAR_MENU_ITEM_VALUE || item->item_type == CAR_MENU_ITEM_INFO) {
        value = car_menu_editing != 0 && item_number == car_menu_edit_item_number ? car_menu_edit_value : car_menu_read_value(item);
        car_menu_draw_value(item, value, 228, y + 3, foreground, background);
    }
}

static void car_menu_draw_list(void)
{
    uint16_t row;

    car_menu_fill(0, 24, 320, 192, RGB565_BLACK);
    car_menu_draw_header();
    for (row = 0; row < CAR_MENU_VISIBLE_ROWS; row++) car_menu_draw_row_item(car_menu_first_visible + row);
    car_menu_draw_footer();
}

static void car_menu_draw_list_delta(void)
{
    car_menu_draw_row_item(car_menu_old_selected);
    car_menu_draw_row_item(car_menu_selected);
}

static void car_menu_draw_telemetry_values(void)
{
    uint16_t row;
    uint16_t item_number;
    uint16_t y;
    uint16_t background;
    const car_menu_item_t *item;
    float value;

    for (row = 0; row < CAR_MENU_VISIBLE_ROWS; row++) {
        item_number = car_menu_first_visible + row;
        item = car_menu_item_at(car_menu_page, item_number);
        if (item == 0) continue;
        y = 28 + row * 23;
        background = item_number == car_menu_selected ? RGB565_BLUE : RGB565_BLACK;
        car_menu_fill(220, y, 100, 21, background);
        if (item->item_type == CAR_MENU_ITEM_VALUE || item->item_type == CAR_MENU_ITEM_INFO) {
            value = car_menu_read_value(item);
            car_menu_draw_value(item, value, 228, y + 3, RGB565_WHITE, background);
        }
    }
}

static uint8_t car_menu_acquire_display_slot(void)
{
    uint8_t slot = car_display_slot;

    if (slot >= CAR_FRAME_SLOT_COUNT || car_frame_state[slot] != CAR_FRAME_DISPLAY_READY) return CAR_FRAME_SLOT_NONE;
    car_display_read_slot = slot;
    __dsync();
    car_frame_state[slot] = CAR_FRAME_DISPLAY_READING;
    __dsync();
    return slot;
}

static void car_menu_release_display_slot(uint8_t slot)
{
    if (slot >= CAR_FRAME_SLOT_COUNT) return;
    if (car_display_pending_free_slot == slot) {
        car_frame_state[slot] = CAR_FRAME_FREE;
        car_display_pending_free_slot = CAR_FRAME_SLOT_NONE;
    } else if (car_frame_state[slot] == CAR_FRAME_DISPLAY_READING) {
        car_frame_state[slot] = CAR_FRAME_DISPLAY_READY;
    }
    __dsync();
    car_display_read_slot = CAR_FRAME_SLOT_NONE;
    __dsync();
}

static void car_menu_draw_image_pixels(void)
{
    uint8_t slot = car_menu_acquire_display_slot();

    if (slot < CAR_FRAME_SLOT_COUNT) {
        ips200_show_gray_image(4, 44, &car_gray_frames[slot][0][0], CAR_IMAGE_WIDTH, CAR_IMAGE_HEIGHT, 148, 94, 0);
        ips200_show_gray_image(168, 44, &car_binary_frames[slot][0][0], CAR_IMAGE_WIDTH, CAR_IMAGE_HEIGHT, 148, 94, 1);
        car_menu_release_display_slot(slot);
    } else {
        car_menu_fill(5, 45, 146, 92, RGB565_BLACK);
        car_menu_fill(169, 45, 146, 92, RGB565_BLACK);
        car_menu_text(32, 84, "NO RAW FRAME", RGB565_WHITE, RGB565_BLACK);
        car_menu_text(194, 84, "NO BINARY", RGB565_WHITE, RGB565_BLACK);
    }
}

static void car_menu_draw_image(void)
{
    car_menu_fill(0, 24, 320, 192, RGB565_BLACK);
    car_menu_draw_header();
    car_menu_text(4, 27, "RAW", RGB565_WHITE, RGB565_BLACK);
    car_menu_text(168, 27, "BINARY", RGB565_WHITE, RGB565_BLACK);
    car_menu_fill(5, 45, 146, 92, RGB565_BLACK);
    car_menu_fill(169, 45, 146, 92, RGB565_BLACK);
    car_menu_draw_image_pixels();
    car_menu_text(4, 146, "GRAY/BINARY FRAME", RGB565_WHITE, RGB565_BLACK);
    car_menu_draw_footer();
}

static void car_menu_draw_lcd_test(void)
{
    car_menu_fill(0, 0, 64, 240, 0xF800);
    car_menu_fill(64, 0, 64, 240, 0x07E0);
    car_menu_fill(128, 0, 64, 240, 0x001F);
    car_menu_fill(192, 0, 64, 240, 0xFFFF);
    car_menu_fill(256, 0, 64, 240, RGB565_BLACK);
    car_menu_text(92, 108, "LCD TEST", RGB565_WHITE, RGB565_BLACK);
}

void car_menu_init(void)
{
    uint8_t camera_ready;
    uint8_t key_mask;
    uint32_t now;

    cc_tc264_board_init();
    car_init();
    car_shared_clear();
    car_params_reset();
    car_apply_menu_params(&car_params);
    ips200_set_dir(IPS200_CROSSWISE_180);
    ips200_init(IPS200_TYPE_SPI);
    car_uart_stream_init();

    camera_ready = cc_tc264_camera_init();
    car_set_camera_ready(camera_ready);
    if (camera_ready != 0) {
        cc_tc264_camera_set_exposure(car_params.exposure);
        cc_tc264_camera_set_gain(car_params.gain);
    }
    cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    gpio_set_level(BOARD_LED1_PIN, camera_ready != 0 ? GPIO_LOW : GPIO_HIGH);
    gpio_set_level(BOARD_LED2_PIN, GPIO_HIGH);

    car_menu_page = CAR_MENU_ROOT;
    car_menu_selected = 0;
    car_menu_first_visible = 0;
    car_menu_old_selected = 0;
    car_menu_new_frame = 0;
    car_menu_image_pending = 0;
    car_menu_editing = 0;
    car_menu_row_dirty = 0;
    car_menu_footer_dirty = 0;
    car_menu_edit_item_number = 0;
    car_menu_edit_value = 0.0;
    car_menu_original_value = 0.0;
    now = system_getval_ms();
    car_menu_last_key_scan_ms = now;
    car_menu_last_image_display_ms = now;
    car_menu_last_telemetry_display_ms = now;
    car_menu_lcd_test_until_ms = 0;
    car_menu_lcd_test_drawn = 0;
    car_menu_start_pending = 0;
    car_menu_start_ms = 0;
    car_control_clear_latched_result();
    key_mask = cc_tc264_menu_key_mask();
    car_menu_previous_keys = key_mask;
    memset(car_menu_long_latched, 0, sizeof(car_menu_long_latched));
    memset(car_menu_key_pressed_ms, 0, sizeof(car_menu_key_pressed_ms));
    memset(car_menu_key_repeat_ms, 0, sizeof(car_menu_key_repeat_ms));
    car_menu_error = CAR_MENU_ERROR_NONE;
    car_menu_ready = 1;
    car_menu_dirty = 1;
    car_menu_display();
}

void car_menu_frame_accepted(uint32_t frame_period_ms)
{
    (void)frame_period_ms;
}

void car_menu_result_accepted(void)
{
    if (car_menu_ready == 0) return;
    car_menu_new_frame = 1;
    car_menu_image_pending = 1;
}

void car_menu_task(void)
{
    uint32_t now;

    if (car_menu_ready == 0) return;
    now = system_getval_ms();

    if (car_menu_lcd_test_until_ms != 0 && now >= car_menu_lcd_test_until_ms) {
        car_menu_lcd_test_until_ms = 0;
        car_menu_dirty = 1;
    }
    car_menu_process_key_scan(now);
    car_menu_update_start(now);
    if (car_menu_start_pending == 0 && car_params.running != 0 && car_running == 0) {
        car_control_clear_latched_result();
        car_params.running = 0;
        car_menu_error = CAR_MENU_ERROR_LINE_LOST;
        car_menu_dirty = 1;
    }
    if (car_menu_new_frame != 0 && car_menu_lcd_test_until_ms == 0 && car_result_ready == 0 &&
        car_display_slot < CAR_FRAME_SLOT_COUNT && car_frame_state[car_display_slot] == CAR_FRAME_DISPLAY_READY) {
        uint8_t slot = car_menu_acquire_display_slot();

        if (slot < CAR_FRAME_SLOT_COUNT) {
            if (car_uart_stream_is_enabled() != 0) car_uart_stream_send_frame(&car_gray_frames[slot][0][0]);
            car_menu_release_display_slot(slot);
            car_menu_new_frame = 0;
        }
    }
    car_menu_display();
}

void car_menu_display(void)
{
    uint32_t now;

    if (car_menu_ready == 0) return;
    now = system_getval_ms();
    if (car_menu_lcd_test_until_ms != 0) {
        if (car_menu_lcd_test_drawn == 0) {
            car_menu_draw_lcd_test();
            car_menu_lcd_test_drawn = 1;
        }
        car_menu_dirty = 0;
        return;
    }
    car_menu_lcd_test_drawn = 0;
    if (car_menu_page == CAR_MENU_PAGE_IMAGE) {
        if (car_menu_dirty != 0) {
            car_menu_draw_image();
            car_menu_last_image_display_ms = now;
            car_menu_image_pending = 0;
            car_menu_dirty = 0;
        } else if (car_menu_image_pending != 0 && now - car_menu_last_image_display_ms >= CAR_MENU_IMAGE_REFRESH_MS) {
            car_menu_draw_image_pixels();
            car_menu_last_image_display_ms = now;
            car_menu_image_pending = 0;
        }
        return;
    }
    if (car_menu_page == CAR_MENU_PAGE_TELEMETRY) {
        if (car_menu_dirty != 0) {
            car_menu_draw_list();
            car_menu_last_telemetry_display_ms = now;
            car_menu_dirty = 0;
            car_menu_row_dirty = 0;
            car_menu_footer_dirty = 0;
        } else if (car_menu_row_dirty != 0) {
            car_menu_draw_list_delta();
            car_menu_row_dirty = 0;
            if (car_menu_footer_dirty != 0) {
                car_menu_draw_footer();
                car_menu_footer_dirty = 0;
            }
        } else if (car_menu_footer_dirty != 0) {
            car_menu_draw_footer();
            car_menu_footer_dirty = 0;
        } else if (now - car_menu_last_telemetry_display_ms >= CAR_MENU_TELEMETRY_REFRESH_MS) {
            car_menu_draw_telemetry_values();
            car_menu_last_telemetry_display_ms = now;
        }
        return;
    }
    if (car_menu_dirty != 0) {
        car_menu_draw_list();
        car_menu_dirty = 0;
        car_menu_row_dirty = 0;
        car_menu_footer_dirty = 0;
    } else if (car_menu_row_dirty != 0) {
        car_menu_draw_list_delta();
        car_menu_row_dirty = 0;
        if (car_menu_footer_dirty != 0) {
            car_menu_draw_footer();
            car_menu_footer_dirty = 0;
        }
    } else if (car_menu_footer_dirty != 0) {
        car_menu_draw_footer();
        car_menu_footer_dirty = 0;
    }
}
