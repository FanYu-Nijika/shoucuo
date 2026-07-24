#include "car_menu_app.h"

#include <string.h>

#include "car_params.h"
#include "car.h"
#include "car_menu_port.h"
#include "car_shared.h"
#include "zf_common_headfile.h"

enum {
    CC_CAR_MENU_NORMAL = CC_MENU_NODE_VISIBLE | CC_MENU_NODE_ENABLED
};

/*
 * Menu maintenance:
 * 1. Add or remove rows in cc_car_menu_items.
 * 2. Keep adjustable data in cc_car_menu_values and cc_car_apply_value.
 * 3. Handle actions in cc_car_menu_process_events.
 */
static const cc_menu_node_spec_t cc_car_menu_items[] = {
    {CC_CAR_PAGE_ROOT, CC_MENU_ID_NONE, "Rear Drive Car", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {CC_CAR_PAGE_DASHBOARD, CC_CAR_PAGE_ROOT, "Dashboard", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {CC_CAR_PAGE_DRIVE, CC_CAR_PAGE_ROOT, "Drive", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {CC_CAR_PAGE_STEERING, CC_CAR_PAGE_ROOT, "Steering", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {CC_CAR_PAGE_VISION, CC_CAR_PAGE_ROOT, "Vision", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {CC_CAR_PAGE_MOTORS, CC_CAR_PAGE_ROOT, "Rear Drive", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {CC_CAR_PAGE_TELEMETRY, CC_CAR_PAGE_ROOT, "Telemetry", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {CC_CAR_PAGE_DIAGNOSTICS, CC_CAR_PAGE_ROOT, "Diagnostics", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {CC_CAR_PAGE_SYSTEM, CC_CAR_PAGE_ROOT, "System", CC_MENU_ITEM_PAGE, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},

    {2000U, CC_CAR_PAGE_DASHBOARD, "Safety", CC_MENU_ITEM_INFO, CC_CAR_DATA_SAFETY_STATE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2001U, CC_CAR_PAGE_DASHBOARD, "Camera", CC_MENU_ITEM_INFO, CC_CAR_DATA_CAMERA_READY, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2002U, CC_CAR_PAGE_DASHBOARD, "Frame rate", CC_MENU_ITEM_INFO, CC_CAR_DATA_FRAME_RATE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},

    {CC_CAR_ITEM_RUN, CC_CAR_PAGE_DRIVE, "Run", CC_MENU_ITEM_ACTION, CC_MENU_ID_NONE, CC_CAR_ACTION_RUN, CC_CAR_MENU_NORMAL},
    {CC_CAR_ITEM_STOP, CC_CAR_PAGE_DRIVE, "Stop", CC_MENU_ITEM_ACTION, CC_MENU_ID_NONE, CC_CAR_ACTION_STOP, CC_CAR_MENU_NORMAL},
    {2003U, CC_CAR_PAGE_DRIVE, "Base speed", CC_MENU_ITEM_VALUE, CC_CAR_DATA_BASE_SPEED, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2047U, CC_CAR_PAGE_DRIVE, "Min speed", CC_MENU_ITEM_VALUE, CC_CAR_DATA_MIN_SPEED, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2048U, CC_CAR_PAGE_DRIVE, "Curve slowdown", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_SLOWDOWN, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2049U, CC_CAR_PAGE_DRIVE, "Min confidence", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CONFIDENCE_MIN, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2005U, CC_CAR_PAGE_DRIVE, "Lost speed", CC_MENU_ITEM_VALUE, CC_CAR_DATA_LOST_SPEED, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2006U, CC_CAR_PAGE_DRIVE, "Lost frames", CC_MENU_ITEM_VALUE, CC_CAR_DATA_LOST_FRAMES, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},

    {2007U, CC_CAR_PAGE_STEERING, "Servo center us", CC_MENU_ITEM_VALUE, CC_CAR_DATA_SERVO_CENTER_US, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2008U, CC_CAR_PAGE_STEERING, "Servo travel us", CC_MENU_ITEM_VALUE, CC_CAR_DATA_SERVO_TRAVEL_US, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2009U, CC_CAR_PAGE_STEERING, "Servo reverse", CC_MENU_ITEM_VALUE, CC_CAR_DATA_SERVO_REVERSE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2010U, CC_CAR_PAGE_STEERING, "Servo command", CC_MENU_ITEM_INFO, CC_CAR_DATA_SERVO_COMMAND_US, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2050U, CC_CAR_PAGE_STEERING, "Wheelbase cm", CC_MENU_ITEM_VALUE, CC_CAR_DATA_WHEELBASE_CM, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2051U, CC_CAR_PAGE_STEERING, "Wheel angle deg", CC_MENU_ITEM_VALUE, CC_CAR_DATA_WHEEL_ANGLE_DEG, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2011U, CC_CAR_PAGE_STEERING, "PD Kp us/pixel", CC_MENU_ITEM_VALUE, CC_CAR_DATA_STANLEY_GAIN, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2012U, CC_CAR_PAGE_STEERING, "PD Kd us/pixel", CC_MENU_ITEM_VALUE, CC_CAR_DATA_STANLEY_SOFT, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2013U, CC_CAR_PAGE_STEERING, "Max steering us", CC_MENU_ITEM_VALUE, CC_CAR_DATA_MAX_STEERING, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2014U, CC_CAR_PAGE_STEERING, "Heading gain", CC_MENU_ITEM_VALUE, CC_CAR_DATA_STANLEY_HEADING_GAIN, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2070U, CC_CAR_PAGE_STEERING, "Curve Kp", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_STEERING_KP, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2071U, CC_CAR_PAGE_STEERING, "Curve Kd", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_STEERING_KD, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2004U, CC_CAR_PAGE_STEERING, "Curve var limit", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_VARIANCE_THRESHOLD, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2072U, CC_CAR_PAGE_STEERING, "Curve FF gain", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_FEEDFORWARD_GAIN, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2073U, CC_CAR_PAGE_STEERING, "Curve preview cm", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_PREVIEW_SETTING_CM, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2074U, CC_CAR_PAGE_STEERING, "Preview speed cm", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_PREVIEW_SPEED_SETTING_CM, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2052U, CC_CAR_PAGE_STEERING, "Control row far", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_PREVIEW_BASE_CM, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2053U, CC_CAR_PAGE_STEERING, "Control row near", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CURVE_PREVIEW_SPEED_GAIN_CM, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2063U, CC_CAR_PAGE_STEERING, "CAR_PATH_WEIGHT_FAR", CC_MENU_ITEM_VALUE, CC_CAR_DATA_PATH_WEIGHT_FAR, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2064U, CC_CAR_PAGE_STEERING, "CAR_PATH_WEIGHT_MIDDLE", CC_MENU_ITEM_VALUE, CC_CAR_DATA_PATH_WEIGHT_MIDDLE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2065U, CC_CAR_PAGE_STEERING, "CAR_PATH_WEIGHT_NEAR", CC_MENU_ITEM_VALUE, CC_CAR_DATA_PATH_WEIGHT_NEAR, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},

    {2016U, CC_CAR_PAGE_VISION, "Auto threshold", CC_MENU_ITEM_VALUE, CC_CAR_DATA_AUTO_THRESHOLD, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2017U, CC_CAR_PAGE_VISION, "Threshold", CC_MENU_ITEM_VALUE, CC_CAR_DATA_THRESHOLD, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2018U, CC_CAR_PAGE_VISION, "Dark line", CC_MENU_ITEM_VALUE, CC_CAR_DATA_DARK_LINE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2019U, CC_CAR_PAGE_VISION, "Min pixels", CC_MENU_ITEM_VALUE, CC_CAR_DATA_MIN_PIXELS, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2020U, CC_CAR_PAGE_VISION, "Row step", CC_MENU_ITEM_VALUE, CC_CAR_DATA_ROW_STEP, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2056U, CC_CAR_PAGE_VISION, "Search window", CC_MENU_ITEM_VALUE, CC_CAR_DATA_SEARCH_WINDOW, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2057U, CC_CAR_PAGE_VISION, "Edge gradient", CC_MENU_ITEM_VALUE, CC_CAR_DATA_EDGE_GRADIENT, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2058U, CC_CAR_PAGE_VISION, "Far width", CC_MENU_ITEM_VALUE, CC_CAR_DATA_TRACK_WIDTH_FAR, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2059U, CC_CAR_PAGE_VISION, "Near width", CC_MENU_ITEM_VALUE, CC_CAR_DATA_TRACK_WIDTH_NEAR, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2021U, CC_CAR_PAGE_VISION, "ROI top", CC_MENU_ITEM_VALUE, CC_CAR_DATA_ROI_TOP, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2022U, CC_CAR_PAGE_VISION, "ROI bottom", CC_MENU_ITEM_VALUE, CC_CAR_DATA_ROI_BOTTOM, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2023U, CC_CAR_PAGE_VISION, "Center offset", CC_MENU_ITEM_VALUE, CC_CAR_DATA_CENTER_OFFSET, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2024U, CC_CAR_PAGE_VISION, "Otsu row step", CC_MENU_ITEM_VALUE, CC_CAR_DATA_OTSU_ROW_STEP, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2025U, CC_CAR_PAGE_VISION, "Otsu col step", CC_MENU_ITEM_VALUE, CC_CAR_DATA_OTSU_COLUMN_STEP, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2026U, CC_CAR_PAGE_VISION, "Exposure", CC_MENU_ITEM_VALUE, CC_CAR_DATA_EXPOSURE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2027U, CC_CAR_PAGE_VISION, "Gain", CC_MENU_ITEM_VALUE, CC_CAR_DATA_GAIN, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},

    {2028U, CC_CAR_PAGE_MOTORS, "PWM limit", CC_MENU_ITEM_VALUE, CC_CAR_DATA_PWM_LIMIT, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2029U, CC_CAR_PAGE_MOTORS, "Deadzone", CC_MENU_ITEM_VALUE, CC_CAR_DATA_DEADZONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2030U, CC_CAR_PAGE_MOTORS, "Left direction", CC_MENU_ITEM_VALUE, CC_CAR_DATA_LEFT_DIRECTION, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2031U, CC_CAR_PAGE_MOTORS, "Right direction", CC_MENU_ITEM_VALUE, CC_CAR_DATA_RIGHT_DIRECTION, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2032U, CC_CAR_PAGE_MOTORS, "Left command", CC_MENU_ITEM_INFO, CC_CAR_DATA_LEFT_COMMAND, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2033U, CC_CAR_PAGE_MOTORS, "Right command", CC_MENU_ITEM_INFO, CC_CAR_DATA_RIGHT_COMMAND, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},

    {2034U, CC_CAR_PAGE_TELEMETRY, "Left encoder", CC_MENU_ITEM_INFO, CC_CAR_DATA_LEFT_ENCODER, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2035U, CC_CAR_PAGE_TELEMETRY, "Right encoder", CC_MENU_ITEM_INFO, CC_CAR_DATA_RIGHT_ENCODER, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2036U, CC_CAR_PAGE_TELEMETRY, "Servo us", CC_MENU_ITEM_INFO, CC_CAR_DATA_SERVO_COMMAND_US, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2037U, CC_CAR_PAGE_TELEMETRY, "Near error cm", CC_MENU_ITEM_INFO, CC_CAR_DATA_NEAR_ERROR_CM, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2038U, CC_CAR_PAGE_TELEMETRY, "Near heading", CC_MENU_ITEM_INFO, CC_CAR_DATA_HEADING_ERROR, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2039U, CC_CAR_PAGE_TELEMETRY, "Strength", CC_MENU_ITEM_INFO, CC_CAR_DATA_LINE_STRENGTH, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2040U, CC_CAR_PAGE_TELEMETRY, "Lost count", CC_MENU_ITEM_INFO, CC_CAR_DATA_LOST_COUNT, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2041U, CC_CAR_PAGE_TELEMETRY, "Threshold used", CC_MENU_ITEM_INFO, CC_CAR_DATA_THRESHOLD_USED, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2042U, CC_CAR_PAGE_TELEMETRY, "Loop us", CC_MENU_ITEM_INFO, CC_CAR_DATA_LOOP_TIME, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2060U, CC_CAR_PAGE_TELEMETRY, "Preview curvature", CC_MENU_ITEM_INFO, CC_CAR_DATA_PREVIEW_CURVATURE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2061U, CC_CAR_PAGE_TELEMETRY, "Curve preview cm", CC_MENU_ITEM_INFO, CC_CAR_DATA_CURVE_PREVIEW_USED, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2067U, CC_CAR_PAGE_TELEMETRY, "Line variance", CC_MENU_ITEM_INFO, CC_CAR_DATA_STANLEY_FEEDBACK_US, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2068U, CC_CAR_PAGE_TELEMETRY, "Curvature FF us", CC_MENU_ITEM_INFO, CC_CAR_DATA_CURVATURE_FEEDFORWARD_US, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2062U, CC_CAR_PAGE_TELEMETRY, "Repaired rows", CC_MENU_ITEM_INFO, CC_CAR_DATA_REPAIRED_ROWS, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},

    {CC_CAR_ITEM_LCD_TEST, CC_CAR_PAGE_DIAGNOSTICS, "LCD color test", CC_MENU_ITEM_ACTION, CC_MENU_ID_NONE, CC_CAR_ACTION_LCD_TEST, CC_CAR_MENU_NORMAL},
    {2043U, CC_CAR_PAGE_DIAGNOSTICS, "Camera ready", CC_MENU_ITEM_INFO, CC_CAR_DATA_CAMERA_READY, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2044U, CC_CAR_PAGE_DIAGNOSTICS, "Rear drive ready", CC_MENU_ITEM_INFO, CC_CAR_DATA_MOTOR_READY, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},

    {CC_CAR_ITEM_RESET_DEFAULTS, CC_CAR_PAGE_SYSTEM, "Reset defaults", CC_MENU_ITEM_ACTION, CC_MENU_ID_NONE, CC_CAR_ACTION_RESET_DEFAULTS, CC_CAR_MENU_NORMAL},
    {2045U, CC_CAR_PAGE_SYSTEM, "Theme: Dark Cyan", CC_MENU_ITEM_INFO, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL},
    {2046U, CC_CAR_PAGE_SYSTEM, "TC264 / ST7789", CC_MENU_ITEM_INFO, CC_MENU_ID_NONE, CC_MENU_ID_NONE, CC_CAR_MENU_NORMAL}
};

static const cc_menu_data_spec_t cc_car_menu_values[] = {
    {CC_CAR_DATA_BASE_SPEED, CC_MENU_DATA_INT32, -1000.0f, 10000.0f, 10.0f, 0U, 0U},
    {CC_CAR_DATA_MIN_SPEED, CC_MENU_DATA_INT32, 0.0f, 10000.0f, 10.0f, 0U, 0U},
    {CC_CAR_DATA_CURVE_SLOWDOWN, CC_MENU_DATA_INT32, 0.0f, 1000.0f, 10.0f, 0U, 0U},
    {CC_CAR_DATA_CONFIDENCE_MIN, CC_MENU_DATA_FLOAT, 0.05f, 0.95f, 0.05f, 2U, 0U},
    {CC_CAR_DATA_MAX_STEERING, CC_MENU_DATA_INT32, 50.0f, 200.0f, 10.0f, 0U, 0U},
    {CC_CAR_DATA_CURVE_VARIANCE_THRESHOLD, CC_MENU_DATA_FLOAT, 0.5f, 50.0f, 0.5f, 1U, 0U},
    {CC_CAR_DATA_LOST_SPEED, CC_MENU_DATA_INT32, 0.0f, 10000.0f, 10.0f, 0U, 0U},
    {CC_CAR_DATA_LOST_FRAMES, CC_MENU_DATA_UINT32, 1.0f, 65535.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_STANLEY_GAIN, CC_MENU_DATA_FLOAT, -1000.0f, 1000.0f, 0.10f, 2U, 0U},
    {CC_CAR_DATA_STANLEY_SOFT, CC_MENU_DATA_FLOAT, -1000.0f, 1000.0f, 0.10f, 2U, 0U},
    {CC_CAR_DATA_STANLEY_HEADING_GAIN, CC_MENU_DATA_FLOAT, 0.0f, 2.0f, 0.10f, 2U, 0U},
    {CC_CAR_DATA_CURVE_FEEDFORWARD_GAIN, CC_MENU_DATA_FLOAT, 0.0f, 3.0f, 0.10f, 2U, 0U},
    {CC_CAR_DATA_CURVE_PREVIEW_BASE_CM, CC_MENU_DATA_UINT32, 20.0f, 72.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_CURVE_PREVIEW_SPEED_GAIN_CM, CC_MENU_DATA_UINT32, 28.0f, 115.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_CURVE_STEERING_KP, CC_MENU_DATA_FLOAT, -1000.0f, 1000.0f, 0.10f, 2U, 0U},
    {CC_CAR_DATA_CURVE_STEERING_KD, CC_MENU_DATA_FLOAT, -1000.0f, 1000.0f, 0.10f, 2U, 0U},
    {CC_CAR_DATA_CURVE_PREVIEW_SETTING_CM, CC_MENU_DATA_FLOAT, 4.0f, 16.0f, 1.0f, 1U, 0U},
    {CC_CAR_DATA_CURVE_PREVIEW_SPEED_SETTING_CM, CC_MENU_DATA_FLOAT, 0.0f, 16.0f, 1.0f, 1U, 0U},
    {CC_CAR_DATA_WHEELBASE_CM, CC_MENU_DATA_FLOAT, 10.0f, 40.0f, 0.5f, 1U, 0U},
    {CC_CAR_DATA_WHEEL_ANGLE_DEG, CC_MENU_DATA_FLOAT, 10.0f, CAR_WHEEL_ANGLE_MAX_DEG, 1.0f, 1U, 0U},
    {CC_CAR_DATA_PATH_WEIGHT_FAR, CC_MENU_DATA_FLOAT, 0.05f, 3.0f, 0.05f, 2U, 0U},
    {CC_CAR_DATA_PATH_WEIGHT_MIDDLE, CC_MENU_DATA_FLOAT, 0.05f, 3.0f, 0.05f, 2U, 0U},
    {CC_CAR_DATA_PATH_WEIGHT_NEAR, CC_MENU_DATA_FLOAT, 0.05f, 3.0f, 0.05f, 2U, 0U},
    {CC_CAR_DATA_AUTO_THRESHOLD, CC_MENU_DATA_BOOL, 0.0f, 1.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_THRESHOLD, CC_MENU_DATA_UINT32, 0.0f, 255.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_DARK_LINE, CC_MENU_DATA_BOOL, 0.0f, 1.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_MIN_PIXELS, CC_MENU_DATA_UINT32, 1.0f, 64.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_ROW_STEP, CC_MENU_DATA_UINT32, 1.0f, 16.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_ROI_TOP, CC_MENU_DATA_UINT32, 0.0f, 119.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_ROI_BOTTOM, CC_MENU_DATA_UINT32, 0.0f, 120.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_CENTER_OFFSET, CC_MENU_DATA_INT32, -94.0f, 94.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_OTSU_ROW_STEP, CC_MENU_DATA_UINT32, 1.0f, 16.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_OTSU_COLUMN_STEP, CC_MENU_DATA_UINT32, 1.0f, 16.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_SEARCH_WINDOW, CC_MENU_DATA_UINT32, 4.0f, 40.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_EDGE_GRADIENT, CC_MENU_DATA_UINT32, 0.0f, 128.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_TRACK_WIDTH_FAR, CC_MENU_DATA_UINT32, 8.0f, 188.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_TRACK_WIDTH_NEAR, CC_MENU_DATA_UINT32, 20.0f, 188.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_EXPOSURE, CC_MENU_DATA_UINT32, 1.0f, 4000.0f, 16.0f, 0U, 0U},
    {CC_CAR_DATA_GAIN, CC_MENU_DATA_UINT32, 16.0f, 64.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_PWM_LIMIT, CC_MENU_DATA_INT32, 0.0f, 10000.0f, 10.0f, 0U, 0U},
    {CC_CAR_DATA_DEADZONE, CC_MENU_DATA_INT32, 0.0f, 500.0f, 5.0f, 0U, 0U},
    {CC_CAR_DATA_LEFT_DIRECTION, CC_MENU_DATA_ENUM, 0.0f, 1.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_RIGHT_DIRECTION, CC_MENU_DATA_ENUM, 0.0f, 1.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_LEFT_COMMAND, CC_MENU_DATA_INT32, -10000.0f, 10000.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_RIGHT_COMMAND, CC_MENU_DATA_INT32, -10000.0f, 10000.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LEFT_ENCODER, CC_MENU_DATA_INT32, -1000000.0f, 1000000.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_RIGHT_ENCODER, CC_MENU_DATA_INT32, -1000000.0f, 1000000.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LINE_VALID, CC_MENU_DATA_BOOL, 0.0f, 1.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LEFT_EDGE, CC_MENU_DATA_UINT32, 0.0f, 188.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_RIGHT_EDGE, CC_MENU_DATA_UINT32, 0.0f, 188.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LINE_WIDTH, CC_MENU_DATA_UINT32, 0.0f, 188.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LINE_CENTER, CC_MENU_DATA_FLOAT, 0.0f, 188.0f, 1.0f, 1U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LINE_ERROR, CC_MENU_DATA_FLOAT, -1.5f, 1.5f, 0.01f, 2U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_NEAR_ERROR_CM, CC_MENU_DATA_FLOAT, -100.0f, 100.0f, 0.01f, 2U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_HEADING_ERROR, CC_MENU_DATA_FLOAT, -2.0f, 2.0f, 0.01f, 2U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LINE_STRENGTH, CC_MENU_DATA_FLOAT, 0.0f, 1.0f, 0.01f, 2U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LOST_COUNT, CC_MENU_DATA_UINT32, 0.0f, 65535.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_THRESHOLD_USED, CC_MENU_DATA_UINT32, 0.0f, 255.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_FRAME_RATE, CC_MENU_DATA_UINT32, 0.0f, 200.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_LOOP_TIME, CC_MENU_DATA_UINT32, 0.0f, 1000000.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_CAMERA_READY, CC_MENU_DATA_BOOL, 0.0f, 1.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_MOTOR_READY, CC_MENU_DATA_BOOL, 0.0f, 1.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_SAFETY_STATE, CC_MENU_DATA_ENUM, 0.0f, 3.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_SERVO_CENTER_US, CC_MENU_DATA_INT32, 1400.0f, 2000.0f, 5.0f, 0U, 0U},
    {CC_CAR_DATA_SERVO_TRAVEL_US, CC_MENU_DATA_INT32, 50.0f, 300.0f, 5.0f, 0U, 0U},
    {CC_CAR_DATA_SERVO_REVERSE, CC_MENU_DATA_BOOL, 0.0f, 1.0f, 1.0f, 0U, 0U},
    {CC_CAR_DATA_SERVO_COMMAND_US, CC_MENU_DATA_INT32, 1400.0f, 2000.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_PREVIEW_CURVATURE, CC_MENU_DATA_FLOAT, -1.0f, 1.0f, 0.001f, 4U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_CURVE_PREVIEW_USED, CC_MENU_DATA_FLOAT, 0.0f, 20.0f, 0.1f, 1U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_STANLEY_FEEDBACK_US, CC_MENU_DATA_FLOAT, -1000.0f, 1000.0f, 0.1f, 1U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_CURVATURE_FEEDFORWARD_US, CC_MENU_DATA_FLOAT, -1000.0f, 1000.0f, 0.1f, 1U, CC_MENU_DATA_READ_ONLY},
    {CC_CAR_DATA_REPAIRED_ROWS, CC_MENU_DATA_UINT32, 0.0f, 120.0f, 1.0f, 0U, CC_MENU_DATA_READ_ONLY}
};

static void cc_car_apply_value(cc_car_menu_app_t *app, uint16_t id,
                               cc_menu_value_t value)
{
    switch (id) {
        case CC_CAR_DATA_BASE_SPEED: car_params.base_speed = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_MIN_SPEED: car_params.minimum_speed = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_CURVE_SLOWDOWN: car_params.curve_slowdown = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_CONFIDENCE_MIN: car_params.confidence_minimum = value.float_value; break;
        case CC_CAR_DATA_MAX_STEERING: car_params.maximum_steering = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_CURVE_VARIANCE_THRESHOLD:
            car_params.curve_variance_threshold = value.float_value;
            break;
        case CC_CAR_DATA_LOST_SPEED: car_params.lost_speed = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_LOST_FRAMES: car_params.lost_stop_frames = (uint16_t)value.uint32_value; break;
        case CC_CAR_DATA_STANLEY_GAIN:
            car_params.steering_kp = value.float_value;
            break;
        case CC_CAR_DATA_STANLEY_SOFT:
            car_params.steering_kd = value.float_value;
            break;
        case CC_CAR_DATA_STANLEY_HEADING_GAIN: car_params.stanley_heading_gain = value.float_value; break;
        case CC_CAR_DATA_CURVE_FEEDFORWARD_GAIN: car_params.curve_feedforward_gain = value.float_value; break;
        case CC_CAR_DATA_CURVE_PREVIEW_BASE_CM:
            car_params.control_row_far = (uint8_t)value.uint32_value;
            break;
        case CC_CAR_DATA_CURVE_PREVIEW_SPEED_GAIN_CM:
            car_params.control_row_near = (uint8_t)value.uint32_value;
            break;
        case CC_CAR_DATA_CURVE_STEERING_KP:
            car_params.curve_steering_kp = value.float_value;
            break;
        case CC_CAR_DATA_CURVE_STEERING_KD:
            car_params.curve_steering_kd = value.float_value;
            break;
        case CC_CAR_DATA_CURVE_PREVIEW_SETTING_CM:
            car_params.curve_preview_base_cm = value.float_value;
            break;
        case CC_CAR_DATA_CURVE_PREVIEW_SPEED_SETTING_CM:
            car_params.curve_preview_speed_gain_cm = value.float_value;
            break;
        case CC_CAR_DATA_WHEELBASE_CM: car_params.wheelbase_cm = value.float_value; break;
        case CC_CAR_DATA_WHEEL_ANGLE_DEG: car_params.wheel_angle_deg = value.float_value; break;
        case CC_CAR_DATA_PATH_WEIGHT_FAR: car_params.path_weight_far = value.float_value; break;
        case CC_CAR_DATA_PATH_WEIGHT_MIDDLE: car_params.path_weight_middle = value.float_value; break;
        case CC_CAR_DATA_PATH_WEIGHT_NEAR: car_params.path_weight_near = value.float_value; break;
        case CC_CAR_DATA_SERVO_CENTER_US:
            car_params.servo_center_us = (int16_t)value.int32_value;
            if (app->safety_state != CC_CAR_SAFE_RUNNING) {
                cc_tc264_menu_servo_write_us(car_params.servo_center_us);
            }
            break;
        case CC_CAR_DATA_SERVO_TRAVEL_US: car_params.servo_travel_us = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_SERVO_REVERSE: car_params.servo_reverse = value.boolean; break;
        case CC_CAR_DATA_AUTO_THRESHOLD: car_params.automatic_threshold = value.boolean; break;
        case CC_CAR_DATA_THRESHOLD: car_params.threshold = (uint8_t)value.uint32_value; break;
        case CC_CAR_DATA_DARK_LINE: car_params.dark_is_line = value.boolean; break;
        case CC_CAR_DATA_MIN_PIXELS: car_params.minimum_line_pixels = (uint8_t)value.uint32_value; break;
        case CC_CAR_DATA_ROW_STEP: car_params.row_step = (uint8_t)value.uint32_value; break;
        case CC_CAR_DATA_ROI_TOP: car_params.roi_top = (uint16_t)value.uint32_value; break;
        case CC_CAR_DATA_ROI_BOTTOM: car_params.roi_bottom = (uint16_t)value.uint32_value; break;
        case CC_CAR_DATA_CENTER_OFFSET: car_params.center_offset_pixels = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_OTSU_ROW_STEP: car_params.otsu_row_step = (uint8_t)value.uint32_value; break;
        case CC_CAR_DATA_OTSU_COLUMN_STEP: car_params.otsu_column_step = (uint8_t)value.uint32_value; break;
        case CC_CAR_DATA_SEARCH_WINDOW: car_params.search_window = (uint8_t)value.uint32_value; break;
        case CC_CAR_DATA_EDGE_GRADIENT: car_params.edge_gradient = (uint8_t)value.uint32_value; break;
        case CC_CAR_DATA_TRACK_WIDTH_FAR: car_params.track_width_far = (uint16_t)value.uint32_value; break;
        case CC_CAR_DATA_TRACK_WIDTH_NEAR: car_params.track_width_near = (uint16_t)value.uint32_value; break;
        case CC_CAR_DATA_EXPOSURE:
            car_params.exposure = (uint16_t)value.uint32_value;
            cc_tc264_camera_set_exposure(car_params.exposure);
            break;
        case CC_CAR_DATA_GAIN:
            car_params.gain = (uint8_t)value.uint32_value;
            cc_tc264_camera_set_gain(car_params.gain);
            break;
        case CC_CAR_DATA_PWM_LIMIT: car_params.pwm_limit = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_DEADZONE: car_params.deadzone = (int16_t)value.int32_value; break;
        case CC_CAR_DATA_LEFT_DIRECTION: car_params.left_direction = value.enum_value != 0 ? 1 : -1; break;
        case CC_CAR_DATA_RIGHT_DIRECTION: car_params.right_direction = value.enum_value != 0 ? 1 : -1; break;
        default: break;
    }
    car_apply_menu_params(&car_params);
}

static uint8_t cc_car_get_param_value(const car_params_t *params, uint16_t id, cc_menu_value_t *value)
{
    if (params == 0 || value == 0) return 0U;
    memset(value, 0, sizeof(*value));

    switch (id) {
        case CC_CAR_DATA_BASE_SPEED: value->int32_value = params->base_speed; break;
        case CC_CAR_DATA_MIN_SPEED: value->int32_value = params->minimum_speed; break;
        case CC_CAR_DATA_CURVE_SLOWDOWN: value->int32_value = params->curve_slowdown; break;
        case CC_CAR_DATA_CONFIDENCE_MIN: value->float_value = params->confidence_minimum; break;
        case CC_CAR_DATA_MAX_STEERING: value->int32_value = params->maximum_steering; break;
        case CC_CAR_DATA_CURVE_VARIANCE_THRESHOLD:
            value->float_value = params->curve_variance_threshold;
            break;
        case CC_CAR_DATA_LOST_SPEED: value->int32_value = params->lost_speed; break;
        case CC_CAR_DATA_LOST_FRAMES: value->uint32_value = params->lost_stop_frames; break;
        case CC_CAR_DATA_STANLEY_GAIN: value->float_value = params->steering_kp; break;
        case CC_CAR_DATA_STANLEY_SOFT: value->float_value = params->steering_kd; break;
        case CC_CAR_DATA_STANLEY_HEADING_GAIN: value->float_value = params->stanley_heading_gain; break;
        case CC_CAR_DATA_CURVE_FEEDFORWARD_GAIN: value->float_value = params->curve_feedforward_gain; break;
        case CC_CAR_DATA_CURVE_PREVIEW_BASE_CM: value->uint32_value = params->control_row_far; break;
        case CC_CAR_DATA_CURVE_PREVIEW_SPEED_GAIN_CM: value->uint32_value = params->control_row_near; break;
        case CC_CAR_DATA_CURVE_STEERING_KP: value->float_value = params->curve_steering_kp; break;
        case CC_CAR_DATA_CURVE_STEERING_KD: value->float_value = params->curve_steering_kd; break;
        case CC_CAR_DATA_CURVE_PREVIEW_SETTING_CM: value->float_value = params->curve_preview_base_cm; break;
        case CC_CAR_DATA_CURVE_PREVIEW_SPEED_SETTING_CM: value->float_value = params->curve_preview_speed_gain_cm; break;
        case CC_CAR_DATA_WHEELBASE_CM: value->float_value = params->wheelbase_cm; break;
        case CC_CAR_DATA_WHEEL_ANGLE_DEG: value->float_value = params->wheel_angle_deg; break;
        case CC_CAR_DATA_PATH_WEIGHT_FAR: value->float_value = params->path_weight_far; break;
        case CC_CAR_DATA_PATH_WEIGHT_MIDDLE: value->float_value = params->path_weight_middle; break;
        case CC_CAR_DATA_PATH_WEIGHT_NEAR: value->float_value = params->path_weight_near; break;
        case CC_CAR_DATA_SERVO_CENTER_US: value->int32_value = params->servo_center_us; break;
        case CC_CAR_DATA_SERVO_TRAVEL_US: value->int32_value = params->servo_travel_us; break;
        case CC_CAR_DATA_SERVO_REVERSE: value->boolean = params->servo_reverse; break;
        case CC_CAR_DATA_AUTO_THRESHOLD: value->boolean = params->automatic_threshold; break;
        case CC_CAR_DATA_THRESHOLD: value->uint32_value = params->threshold; break;
        case CC_CAR_DATA_DARK_LINE: value->boolean = params->dark_is_line; break;
        case CC_CAR_DATA_MIN_PIXELS: value->uint32_value = params->minimum_line_pixels; break;
        case CC_CAR_DATA_ROW_STEP: value->uint32_value = params->row_step; break;
        case CC_CAR_DATA_ROI_TOP: value->uint32_value = params->roi_top; break;
        case CC_CAR_DATA_ROI_BOTTOM: value->uint32_value = params->roi_bottom; break;
        case CC_CAR_DATA_CENTER_OFFSET: value->int32_value = params->center_offset_pixels; break;
        case CC_CAR_DATA_OTSU_ROW_STEP: value->uint32_value = params->otsu_row_step; break;
        case CC_CAR_DATA_OTSU_COLUMN_STEP: value->uint32_value = params->otsu_column_step; break;
        case CC_CAR_DATA_SEARCH_WINDOW: value->uint32_value = params->search_window; break;
        case CC_CAR_DATA_EDGE_GRADIENT: value->uint32_value = params->edge_gradient; break;
        case CC_CAR_DATA_TRACK_WIDTH_FAR: value->uint32_value = params->track_width_far; break;
        case CC_CAR_DATA_TRACK_WIDTH_NEAR: value->uint32_value = params->track_width_near; break;
        case CC_CAR_DATA_EXPOSURE: value->uint32_value = params->exposure; break;
        case CC_CAR_DATA_GAIN: value->uint32_value = params->gain; break;
        case CC_CAR_DATA_PWM_LIMIT: value->int32_value = params->pwm_limit; break;
        case CC_CAR_DATA_DEADZONE: value->int32_value = params->deadzone; break;
        case CC_CAR_DATA_LEFT_DIRECTION: value->enum_value = params->left_direction > 0 ? 1 : 0; break;
        case CC_CAR_DATA_RIGHT_DIRECTION: value->enum_value = params->right_direction > 0 ? 1 : 0; break;
        default: return 0U;
    }
    return 1U;
}

static void cc_car_sync_params_to_menu(cc_car_menu_app_t *app)
{
    car_params_t params;
    uint16_t index;

    if (app == 0) return;
    params = car_params;

    for (index = 0U; index < app->menu.data_count; index++) {
        cc_menu_data_t *data = &app->menu.data[index];
        cc_menu_value_t value;

        if ((data->flags & CC_MENU_DATA_READ_ONLY) != 0U) continue;
        if (cc_car_get_param_value(&params, data->id, &value) != 0U) {
            cc_menu_data_set_runtime(&app->menu, data->id, value);
        }
    }
}

static cc_menu_result_t cc_car_load_menu(cc_car_menu_app_t *app)
{
    return cc_menu_init(
        &app->menu,
        cc_car_menu_items,
        (uint16_t)(sizeof(cc_car_menu_items) / sizeof(cc_car_menu_items[0])),
        cc_car_menu_values,
        (uint16_t)(sizeof(cc_car_menu_values) / sizeof(cc_car_menu_values[0])));
}

static void cc_car_reset_defaults(cc_car_menu_app_t *app)
{
    if (app == 0) return;
    car_params.running = 0U;
    __dsync();
    car_stop();
    cc_tc264_menu_motor_stop();
    app->safety_state = CC_CAR_SAFE_LOCKED;
    app->last_error = CC_CAR_ERROR_NONE;

    car_params_reset();
    car_apply_menu_params(&car_params);
    cc_car_sync_params_to_menu(app);
    cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    if (cc_tc264_camera_ready() != 0U) {
        cc_tc264_camera_set_exposure(car_params.exposure);
        cc_tc264_camera_set_gain(car_params.gain);
    }
    cc_car_menu_sync_telemetry(app);
}

cc_menu_result_t cc_car_menu_init(cc_car_menu_app_t *app)
{
    cc_menu_result_t result;

    if (app == 0) return CC_MENU_ERROR_ARGUMENT;
    memset(app, 0, sizeof(*app));
    car_shared_clear();
    car_params_reset();
    car_apply_menu_params(&car_params);
    car_result.servo_command_us = car_params.servo_center_us;
    __dsync();
    app->safety_state = CC_CAR_SAFE_LOCKED;
    app->last_error = CC_CAR_ERROR_NONE;
    app->camera_ready = cc_tc264_camera_ready();

    result = cc_car_load_menu(app);
    if (result == CC_MENU_OK) {
        cc_car_sync_params_to_menu(app);
        cc_car_menu_sync_telemetry(app);
    }
    return result;
}

void cc_car_menu_process_events(cc_car_menu_app_t *app)
{
    cc_menu_event_t event;

    if (app == 0 || cc_menu_take_event(&app->menu, &event) == 0U) return;
    if (event.type == CC_MENU_EVENT_VALUE_CHANGED) {
        if (cc_car_menu_can_edit(app) != 0U) {
            cc_car_apply_value(app, event.data_id, event.value);
        }
    } else if (event.type == CC_MENU_EVENT_ACTION) {
        if (event.action_id == CC_CAR_ACTION_STOP) {
            cc_car_menu_emergency_stop(app);
        } else if (event.action_id == CC_CAR_ACTION_LCD_TEST) {
            app->lcd_test_requested = 1U;
        } else if (event.action_id == CC_CAR_ACTION_RESET_DEFAULTS &&
                   cc_car_menu_can_edit(app) != 0U) {
            cc_car_reset_defaults(app);
        }
    }
}

void cc_car_menu_note_frame(cc_car_menu_app_t *app, uint32_t frame_period_ms)
{
    uint32_t now;

    if (app == 0) return;
    now = system_getval_ms();
    app->frame_count++;
    if (app->last_frame_ms == 0U) app->last_frame_ms = now;
    if ((now - app->last_frame_ms) >= 1000U) {
        app->frame_rate = app->frame_count * 1000U / (now - app->last_frame_ms);
        app->frame_count = 0U;
        app->last_frame_ms = now;
    }

    if (app->safety_state == CC_CAR_SAFE_RUNNING &&
        (frame_period_ms == 0U || frame_period_ms > 250U)) {
        app->last_error = CC_CAR_ERROR_BAD_PERIOD;
        app->safety_state = CC_CAR_SAFE_FAULT;
        car_params.running = 0U;
        cc_tc264_menu_motor_stop();
        cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    }
}

void cc_car_menu_accept_result(cc_car_menu_app_t *app)
{
    car_result_t result;

    if (app == 0 || car_result_ready == 0U) return;
    __dsync();
    result = car_result;
    car_result_ready = 0U;
    __dsync();

    if (app->safety_state != CC_CAR_SAFE_RUNNING) return;
    if (result.lost_count > car_params.lost_stop_frames) {
        app->last_error = CC_CAR_ERROR_LINE_LOST;
        app->safety_state = CC_CAR_SAFE_FAULT;
        car_params.running = 0U;
        cc_tc264_menu_motor_stop();
        cc_tc264_menu_servo_write_us(car_params.servo_center_us);
        return;
    }

    cc_tc264_menu_servo_write_us(result.servo_command_us);
    cc_tc264_menu_motor_write(result.left_command, result.right_command);
}

static void cc_car_set_i32(cc_car_menu_app_t *app, uint16_t id, int32_t value)
{
    cc_menu_value_t data;
    data.int32_value = value;
    cc_menu_data_set_runtime(&app->menu, id, data);
}

static void cc_car_set_u32(cc_car_menu_app_t *app, uint16_t id, uint32_t value)
{
    cc_menu_value_t data;
    data.uint32_value = value;
    cc_menu_data_set_runtime(&app->menu, id, data);
}

static void cc_car_set_float(cc_car_menu_app_t *app, uint16_t id, float value)
{
    cc_menu_value_t data;
    data.float_value = value;
    cc_menu_data_set_runtime(&app->menu, id, data);
}

static void cc_car_set_bool(cc_car_menu_app_t *app, uint16_t id, uint8_t value)
{
    cc_menu_value_t data;
    data.boolean = value != 0U ? 1U : 0U;
    cc_menu_data_set_runtime(&app->menu, id, data);
}

static void cc_car_set_enum(cc_car_menu_app_t *app, uint16_t id, int32_t value)
{
    cc_menu_value_t data;
    data.enum_value = value;
    cc_menu_data_set_runtime(&app->menu, id, data);
}

void cc_car_menu_sync_telemetry(cc_car_menu_app_t *app)
{
    car_result_t result;

    if (app == 0) return;
    result = car_result;
    app->camera_ready = cc_tc264_camera_ready();
    cc_car_set_i32(app, CC_CAR_DATA_SERVO_COMMAND_US, result.servo_command_us);
    cc_car_set_i32(app, CC_CAR_DATA_LEFT_COMMAND, result.left_command);
    cc_car_set_i32(app, CC_CAR_DATA_RIGHT_COMMAND, result.right_command);
    cc_car_set_i32(app, CC_CAR_DATA_LEFT_ENCODER, cc_tc264_encoder_read(0U));
    cc_car_set_i32(app, CC_CAR_DATA_RIGHT_ENCODER, cc_tc264_encoder_read(1U));
    cc_car_set_bool(app, CC_CAR_DATA_LINE_VALID, result.line_valid);
    cc_car_set_u32(app, CC_CAR_DATA_LEFT_EDGE, result.left_edge);
    cc_car_set_u32(app, CC_CAR_DATA_RIGHT_EDGE, result.right_edge);
    cc_car_set_u32(app, CC_CAR_DATA_LINE_WIDTH, result.line_width);
    cc_car_set_float(app, CC_CAR_DATA_LINE_CENTER, result.center_x);
    cc_car_set_float(app, CC_CAR_DATA_LINE_ERROR, result.error_normalized);
    cc_car_set_float(app, CC_CAR_DATA_NEAR_ERROR_CM, result.near_error_cm);
    cc_car_set_float(app, CC_CAR_DATA_HEADING_ERROR, result.heading_error);
    cc_car_set_float(app, CC_CAR_DATA_LINE_STRENGTH, result.strength);
    cc_car_set_u32(app, CC_CAR_DATA_LOST_COUNT, result.lost_count);
    cc_car_set_u32(app, CC_CAR_DATA_THRESHOLD_USED, result.threshold_used);
    cc_car_set_u32(app, CC_CAR_DATA_FRAME_RATE, app->frame_rate);
    cc_car_set_u32(app, CC_CAR_DATA_LOOP_TIME, result.processing_time_us);
    cc_car_set_float(app, CC_CAR_DATA_PREVIEW_CURVATURE, result.curvature);
    cc_car_set_float(app, CC_CAR_DATA_CURVE_PREVIEW_USED, result.curve_preview_cm);
    cc_car_set_float(app, CC_CAR_DATA_STANLEY_FEEDBACK_US, result.curve_variance);
    cc_car_set_float(app, CC_CAR_DATA_CURVATURE_FEEDFORWARD_US,
                     result.curvature_feedforward_us);
    cc_car_set_u32(app, CC_CAR_DATA_REPAIRED_ROWS, result.repaired_rows);
    cc_car_set_bool(app, CC_CAR_DATA_CAMERA_READY, app->camera_ready);
    cc_car_set_bool(app, CC_CAR_DATA_MOTOR_READY, cc_tc264_motor_ready());
    cc_car_set_enum(app, CC_CAR_DATA_SAFETY_STATE, app->safety_state);
}

void cc_car_menu_activate(cc_car_menu_app_t *app, uint16_t item_id,
                          uint8_t long_press)
{
    const cc_menu_node_t *node;

    if (app == 0) return;
    node = cc_menu_find(&app->menu, item_id);
    if (node == 0 || node->type != CC_MENU_ITEM_ACTION) return;

    if (node->action_id == CC_CAR_ACTION_RUN) {
        (void)long_press;
        if (cc_tc264_motor_ready() == 0U) {
            app->last_error = CC_CAR_ERROR_MOTOR_LOCKED;
            app->safety_state = CC_CAR_SAFE_LOCKED;
            cc_tc264_menu_motor_stop();
        } else if (cc_tc264_camera_ready() == 0U) {
            app->last_error = CC_CAR_ERROR_CAMERA_OFFLINE;
            app->safety_state = CC_CAR_SAFE_LOCKED;
            cc_tc264_menu_motor_stop();
        } else {
            app->last_error = CC_CAR_ERROR_NONE;
            car_apply_menu_params(&car_params);
            car_set_running(1U);
            car_params.running = 1U;
            app->safety_state = CC_CAR_SAFE_RUNNING;
            __dsync();
        }
    } else {
        cc_menu_emit_action(&app->menu, item_id);
    }
}

void cc_car_menu_emergency_stop(cc_car_menu_app_t *app)
{
    if (app == 0) return;
    car_stop();
    car_params.running = 0U;
    cc_tc264_menu_motor_stop();
    cc_tc264_menu_servo_write_us(car_params.servo_center_us);
    app->safety_state = CC_CAR_SAFE_LOCKED;
}

uint8_t cc_car_menu_can_edit(const cc_car_menu_app_t *app)
{
    return app != 0 && app->safety_state != CC_CAR_SAFE_RUNNING ? 1U : 0U;
}

const char *cc_car_menu_safety_text(const cc_car_menu_app_t *app)
{
    if (app == 0) return "FAULT";
    if (app->safety_state == CC_CAR_SAFE_LOCKED) return "LOCKED";
    if (app->safety_state == CC_CAR_SAFE_ARMING) return "ARMING";
    if (app->safety_state == CC_CAR_SAFE_RUNNING) return "RUNNING";
    return "FAULT";
}

const char *cc_car_menu_error_text(const cc_car_menu_app_t *app)
{
    if (app == 0) return "APP ERROR";
    if (app->safety_state == CC_CAR_SAFE_ARMING && app->last_error == CC_CAR_ERROR_NONE) return "WAIT RESULT";
    if (app->last_error == CC_CAR_ERROR_NONE) return "READY";
    if (app->last_error == CC_CAR_ERROR_MOTOR_LOCKED) return "MOTOR LOCK";
    if (app->last_error == CC_CAR_ERROR_CAMERA_OFFLINE) return "CAM OFFLINE";
    if (app->last_error == CC_CAR_ERROR_BAD_FRAME) return "BAD FRAME";
    if (app->last_error == CC_CAR_ERROR_BAD_PERIOD) return "BAD PERIOD";
    if (app->last_error == CC_CAR_ERROR_LINE_LOST) return "LINE LOST";
    if (app->last_error == CC_CAR_ERROR_WAIT_RESULT) return "WAIT RESULT";
    return "APP ERROR";
}
