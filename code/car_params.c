#include "car_params.h"

#include "zf_common_headfile.h"

/* Defaults stay in read-only memory; runtime parameters live in shared RAM. */
const car_params_t car_default_params = {
    .base_speed = 2200,
    // .minimum_speed = 2700,
    .curve_slowdown = 0,
    // .cruise_speed = 2800,
    // .straight_speed = 2800,
    // .curve_speed = 2800,
    // .speed_rise_step = 2,
    // .speed_fall_step = 6,
    // .straight_confirm_frames = 3,
    // .maximum_steering = 280,
    // .lost_search_steering = 180,
    // .lost_speed = 2000,
    .lost_stop_frames = CAR_LOST_STOP_FRAMES,

    /* Simple PD units: us/pixel and us/pixel/frame. */
    .steering_kp = CAR_STEERING_KP_DEFAULT,
    // .steering_ki = 0.0,
    .steering_kd = CAR_STEERING_KD_DEFAULT,
    // .curve_variance_threshold = CAR_CURVE_VARIANCE_DEFAULT,
    // .curve_blend_threshold1 = CAR_CURVE_BLEND_THRESHOLD1_DEFAULT,
    // .curve_blend_threshold2 = CAR_CURVE_BLEND_THRESHOLD2_DEFAULT,
    // .curve_steering_kp = CAR_CURVE_STEERING_KP_DEFAULT,
    // .curve_steering_kd = CAR_CURVE_STEERING_KD_DEFAULT,
    // .stanley_heading_gain = CAR_STANLEY_HEADING_GAIN_DEFAULT,
    // .curve_feedforward_gain = CAR_CURVE_FEEDFORWARD_GAIN_DEFAULT,
    // .curve_preview_base_cm = CAR_CURVE_PREVIEW_BASE_DEFAULT_CM,
    // .curve_preview_speed_gain_cm = CAR_CURVE_PREVIEW_SPEED_DEFAULT_CM,
    // .perception_preview_cm = CAR_CURVE_PREVIEW_BASE_DEFAULT_CM,
    // .steering_preview_straight_cm = CAR_CURVE_PREVIEW_SPEED_DEFAULT_CM,
    // .steering_preview_curve_cm = CAR_CURVE_PREVIEW_SPEED_DEFAULT_CM,
    // .differential_gain = 1.0,
    // .differential_limit = 0,
    // .wheelbase_cm = 20.0,
    // .wheel_angle_deg = CAR_WHEEL_ANGLE_MAX_DEG,
    // .path_weight_far = CAR_PATH_WEIGHT_FAR,
    // .path_weight_middle = CAR_PATH_WEIGHT_MIDDLE,
    // .path_weight_near = CAR_PATH_WEIGHT_NEAR,
    // .confidence_minimum = 0.35,
    // .control_row_far = 56,
    .control_row_near = 84,

    /*
     * Measured with PWM_DUTY_MAX=10000 at 50 Hz:
     * PWM duty 600/700/800 corresponds to about 1200/1400/1600 us at 50 Hz.
     * Positive image error means track is right, so servo output is reversed.
     */
    .servo_center_us = 1400,
    .servo_travel_us = 200,
    .servo_reverse = 1,

    .automatic_threshold = 1,
    .cross_enabled = 1,
    .cross_min_both_lost = 10,
    .cross_min_white_column = 60,
    .cross_edge_stable_diff = 5,
    .cross_tear_diff_first = 8,
    .cross_tear_diff_second = 15,
    .cross_corner_row_gap_max = 30,
    .cross_max_lost_rows = 108,
    .threshold = 100,
    .minimum_line_pixels = 2,
    .center_offset_pixels = 0,
    // .dark_is_line = 0,
    // .row_step = 1,
    // .roi_top = 32,
    // .roi_bottom = 0,
    // .otsu_row_step = 2,
    // .otsu_column_step = 2,
    // .search_window = 12,
    // .edge_gradient = 12,
    // .track_width_far = 78,
    // .track_width_near = 185,

    .exposure = 256,
    .gain = 4,
    .pwm_limit = 5000,
    // .deadzone = 15,
    .left_direction = 1,
    .right_direction = 1,
    .running = 0
};

#pragma section all "cpu1_dsram"

volatile car_params_t car_params;

#pragma section all restore

void car_params_reset(void)
{
    car_params.running = 0;
    __dsync();
    car_params = car_default_params;
    __dsync();
}
