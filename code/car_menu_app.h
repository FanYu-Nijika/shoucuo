#ifndef CC_CAR_MENU_APP_H
#define CC_CAR_MENU_APP_H

#include <stdint.h>

#include "menu_core.h"

typedef enum {
    CC_CAR_SAFE_LOCKED = 0,
    CC_CAR_SAFE_ARMING,
    CC_CAR_SAFE_RUNNING,
    CC_CAR_SAFE_FAULT
} cc_car_safety_state_t;

typedef enum {
    CC_CAR_ERROR_NONE = 0,
    CC_CAR_ERROR_MOTOR_LOCKED,
    CC_CAR_ERROR_CAMERA_OFFLINE,
    CC_CAR_ERROR_BAD_FRAME,
    CC_CAR_ERROR_BAD_PERIOD,
    CC_CAR_ERROR_LINE_LOST,
    CC_CAR_ERROR_WAIT_RESULT
} cc_car_error_t;

typedef enum {
    CC_CAR_ACTION_RUN = 1,
    CC_CAR_ACTION_STOP,
    CC_CAR_ACTION_LCD_TEST,
    CC_CAR_ACTION_RESET_DEFAULTS
} cc_car_action_id_t;

/* Pages and action items use fixed IDs; normal data IDs remain sequential. */
typedef enum {
    CC_CAR_PAGE_ROOT = 1000,
    CC_CAR_PAGE_DASHBOARD = 1100,
    CC_CAR_PAGE_DRIVE = 1200,
    CC_CAR_ITEM_RUN = 1201,
    CC_CAR_ITEM_STOP = 1202,
    CC_CAR_PAGE_STEERING = 1300,
    CC_CAR_PAGE_VISION = 1400,
    CC_CAR_PAGE_MOTORS = 1500,
    CC_CAR_PAGE_TELEMETRY = 1600,
    CC_CAR_PAGE_DIAGNOSTICS = 1700,
    CC_CAR_ITEM_LCD_TEST = 1701,
    CC_CAR_PAGE_SYSTEM = 1800,
    CC_CAR_ITEM_RESET_DEFAULTS = 1801
} cc_car_item_id_t;

typedef enum {
    CC_CAR_DATA_BASE_SPEED = 1,
    CC_CAR_DATA_MAX_STEERING,
    CC_CAR_DATA_CURVE_VARIANCE_THRESHOLD,
    CC_CAR_DATA_LOST_SPEED,
    CC_CAR_DATA_LOST_FRAMES,
    CC_CAR_DATA_STANLEY_GAIN,
    CC_CAR_DATA_STANLEY_SOFT,
    CC_CAR_DATA_STANLEY_HEADING_GAIN,
    CC_CAR_DATA_CURVE_FEEDFORWARD_GAIN,
    CC_CAR_DATA_CURVE_PREVIEW_BASE_CM,
    CC_CAR_DATA_CURVE_PREVIEW_SPEED_GAIN_CM,
    CC_CAR_DATA_AUTO_THRESHOLD,
    CC_CAR_DATA_THRESHOLD,
    CC_CAR_DATA_DARK_LINE,
    CC_CAR_DATA_MIN_PIXELS,
    CC_CAR_DATA_ROW_STEP,
    CC_CAR_DATA_ROI_TOP,
    CC_CAR_DATA_ROI_BOTTOM,
    CC_CAR_DATA_CENTER_OFFSET,
    CC_CAR_DATA_OTSU_ROW_STEP,
    CC_CAR_DATA_OTSU_COLUMN_STEP,
    CC_CAR_DATA_EXPOSURE,
    CC_CAR_DATA_GAIN,
    CC_CAR_DATA_PWM_LIMIT,
    CC_CAR_DATA_DEADZONE,
    CC_CAR_DATA_LEFT_DIRECTION,
    CC_CAR_DATA_RIGHT_DIRECTION,
    CC_CAR_DATA_LEFT_COMMAND,
    CC_CAR_DATA_RIGHT_COMMAND,
    CC_CAR_DATA_LEFT_ENCODER,
    CC_CAR_DATA_RIGHT_ENCODER,
    CC_CAR_DATA_LINE_VALID,
    CC_CAR_DATA_LEFT_EDGE,
    CC_CAR_DATA_RIGHT_EDGE,
    CC_CAR_DATA_LINE_WIDTH,
    CC_CAR_DATA_LINE_CENTER,
    CC_CAR_DATA_LINE_ERROR,
    CC_CAR_DATA_NEAR_ERROR_CM,
    CC_CAR_DATA_HEADING_ERROR,
    CC_CAR_DATA_LINE_STRENGTH,
    CC_CAR_DATA_LOST_COUNT,
    CC_CAR_DATA_THRESHOLD_USED,
    CC_CAR_DATA_FRAME_RATE,
    CC_CAR_DATA_LOOP_TIME,
    CC_CAR_DATA_CAMERA_READY,
    CC_CAR_DATA_MOTOR_READY,
    CC_CAR_DATA_SAFETY_STATE,
    CC_CAR_DATA_SERVO_CENTER_US,
    CC_CAR_DATA_SERVO_TRAVEL_US,
    CC_CAR_DATA_SERVO_REVERSE,
    CC_CAR_DATA_SERVO_COMMAND_US,
    CC_CAR_DATA_MIN_SPEED,
    CC_CAR_DATA_CURVE_SLOWDOWN,
    CC_CAR_DATA_CONFIDENCE_MIN,
    CC_CAR_DATA_WHEELBASE_CM,
    CC_CAR_DATA_WHEEL_ANGLE_DEG,
    CC_CAR_DATA_SEARCH_WINDOW,
    CC_CAR_DATA_EDGE_GRADIENT,
    CC_CAR_DATA_TRACK_WIDTH_FAR,
    CC_CAR_DATA_TRACK_WIDTH_NEAR,
    CC_CAR_DATA_PREVIEW_CURVATURE,
    CC_CAR_DATA_CURVE_PREVIEW_USED,
    CC_CAR_DATA_REPAIRED_ROWS,
    CC_CAR_DATA_PATH_WEIGHT_FAR,
    CC_CAR_DATA_PATH_WEIGHT_MIDDLE,
    CC_CAR_DATA_PATH_WEIGHT_NEAR,
    CC_CAR_DATA_STANLEY_FEEDBACK_US,
    CC_CAR_DATA_CURVATURE_FEEDFORWARD_US,
    CC_CAR_DATA_CURVE_STEERING_KP,
    CC_CAR_DATA_CURVE_STEERING_KD,
    CC_CAR_DATA_CURVE_PREVIEW_SETTING_CM,
    CC_CAR_DATA_CURVE_PREVIEW_SPEED_SETTING_CM
} cc_car_data_id_t;

typedef struct {
    cc_menu_t menu;
    cc_car_safety_state_t safety_state;
    cc_car_error_t last_error;
    uint8_t camera_ready;
    uint8_t lcd_test_requested;
    uint32_t last_frame_ms;
    uint32_t frame_count;
    uint32_t frame_rate;
} cc_car_menu_app_t;

cc_menu_result_t cc_car_menu_init(cc_car_menu_app_t *app);
void cc_car_menu_process_events(cc_car_menu_app_t *app);
void cc_car_menu_note_frame(cc_car_menu_app_t *app, uint32_t frame_period_ms);
void cc_car_menu_accept_result(cc_car_menu_app_t *app);
void cc_car_menu_sync_telemetry(cc_car_menu_app_t *app);
void cc_car_menu_activate(cc_car_menu_app_t *app, uint16_t item_id,
                          uint8_t long_press);
void cc_car_menu_emergency_stop(cc_car_menu_app_t *app);
uint8_t cc_car_menu_can_edit(const cc_car_menu_app_t *app);
const char *cc_car_menu_safety_text(const cc_car_menu_app_t *app);
const char *cc_car_menu_error_text(const cc_car_menu_app_t *app);

#endif

