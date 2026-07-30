#include "car_params.h"

#include "zf_common_headfile.h"

/* Defaults stay in read-only memory; runtime parameters live in shared RAM. */
const car_params_t car_default_params = {
    .base_speed = 2800,
    .minimum_speed = 2700,
    .curve_slowdown = 0,
    .cruise_speed = 2800,
    .straight_speed = 2800,
    .curve_speed = 2800,
    .speed_rise_step = 2U,
    .speed_fall_step = 6U,
    .straight_confirm_frames = 3U,
    .maximum_steering = 280,
    .lost_search_steering = 180,
    .lost_speed = 2000,
    .lost_stop_frames = CAR_LOST_STOP_FRAMES,

    /* Simple PD units: us/pixel and us/pixel/frame. */
    .steering_kp = CAR_STEERING_KP_DEFAULT,
    .steering_ki = 0.0f,
    .steering_kd = CAR_STEERING_KD_DEFAULT,
    .curve_variance_threshold = CAR_CURVE_VARIANCE_DEFAULT,
    .curve_blend_threshold1 = CAR_CURVE_BLEND_THRESHOLD1_DEFAULT,
    .curve_blend_threshold2 = CAR_CURVE_BLEND_THRESHOLD2_DEFAULT,
    .curve_steering_kp = CAR_CURVE_STEERING_KP_DEFAULT,
    .curve_steering_kd = CAR_CURVE_STEERING_KD_DEFAULT,
    .stanley_heading_gain = CAR_STANLEY_HEADING_GAIN_DEFAULT,
    .curve_feedforward_gain = CAR_CURVE_FEEDFORWARD_GAIN_DEFAULT,
    .curve_preview_base_cm = CAR_CURVE_PREVIEW_BASE_DEFAULT_CM,
    .curve_preview_speed_gain_cm = CAR_CURVE_PREVIEW_SPEED_DEFAULT_CM,
    .perception_preview_cm = CAR_CURVE_PREVIEW_BASE_DEFAULT_CM,
    .steering_preview_straight_cm = CAR_CURVE_PREVIEW_SPEED_DEFAULT_CM,
    .steering_preview_curve_cm = CAR_CURVE_PREVIEW_SPEED_DEFAULT_CM,
    .differential_gain = 1.0f,
    .differential_limit = 0,
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
     * PWM duty 600/700/800 corresponds to about 1200/1400/1600 us at 50 Hz.
     * Positive image error means track is right, so servo output is reversed.
     */
    .servo_center_us = 1400,
    .servo_travel_us = 200,
    .servo_reverse = 1U,

    .automatic_threshold = 1U,
    .cross_enabled = 1U,
    .threshold = 100U,
    .dark_is_line = 0U,
    .minimum_line_pixels = 2U,
    .row_step = 1U,
    .roi_top = 32U,
    .roi_bottom = 0U,
    .center_offset_pixels = 0,
    .otsu_row_step = 2U,
    .otsu_column_step = 2U,
    .search_window = 12U,
    .edge_gradient = 12U,
    .track_width_far = 78U,
    .track_width_near = 185U,

    .exposure = 512U,
    .gain = 32U,
    .pwm_limit = 5000,
    .deadzone = 15,
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
