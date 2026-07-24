#include "car_params.h"

#include "zf_common_headfile.h"

/* Defaults stay in read-only memory; runtime parameters live in shared RAM. */
const car_params_t car_default_params = {
    .base_speed = 1800,
    .minimum_speed = 0,
    .curve_slowdown = 20,
    .maximum_steering = 170,
    .lost_search_steering = 180,
    .lost_speed = 900,
    .lost_stop_frames = 5,

    /* Simple PD units: us/pixel and us/pixel/frame. */
    .steering_kp = 2.0f,
    .steering_ki = 0.1,
    .steering_kd = 4.0f,
    .curve_variance_threshold = CAR_CURVE_VARIANCE_DEFAULT,
    .curve_steering_kp = CAR_CURVE_STEERING_KP_DEFAULT,
    .curve_steering_kd = CAR_CURVE_STEERING_KD_DEFAULT,
    .stanley_heading_gain = CAR_STANLEY_HEADING_GAIN_DEFAULT,
    .curve_feedforward_gain = CAR_CURVE_FEEDFORWARD_GAIN_DEFAULT,
    .curve_preview_base_cm = CAR_CURVE_PREVIEW_BASE_DEFAULT_CM,
    .curve_preview_speed_gain_cm = CAR_CURVE_PREVIEW_SPEED_DEFAULT_CM,
    .wheelbase_cm = 20.0f,
    .wheel_angle_deg = CAR_WHEEL_ANGLE_MAX_DEG,
    .path_weight_far = CAR_PATH_WEIGHT_FAR,
    .path_weight_middle = CAR_PATH_WEIGHT_MIDDLE,
    .path_weight_near = CAR_PATH_WEIGHT_NEAR,
    .confidence_minimum = 0.35f,
    .control_row_far = 56,
    .control_row_near = 84,

    /*
     * Measured with PWM_DUTY_MAX=10000 at 50 Hz:
     * right 820 -> 1640 us, center 890 -> 1780 us, left 940 -> 1880 us.
     * Positive image error means track is right, so servo output is reversed.
     */
    .servo_center_us = 1660,
    .servo_travel_us = 260,
    .servo_reverse = 0U,

    .automatic_threshold = 1U,
    .threshold = 100U,
    .dark_is_line = 0U,
    .minimum_line_pixels = 3U,
    .row_step = 1U,
    .roi_top = 32U,
    .roi_bottom = 0U,
    .center_offset_pixels = 0,
    .otsu_row_step = 2U,
    .otsu_column_step = 2U,
    .search_window = 30U,
    .edge_gradient = 12U,
    .track_width_far = 78U,
    .track_width_near = 185U,

    .exposure = 512U,
    .gain = 32U,
    .pwm_limit = 5000,
    .deadzone = 0,
    .left_direction = 1,
    .right_direction = 1,
    .running = 0U
};

#pragma section all "cpu1_dsram"

volatile car_params_t car_params;

#pragma section all restore

void car_params_reset(void)
{
    car_params.running = 0U;
    __dsync();
    car_params = car_default_params;
    __dsync();
}
