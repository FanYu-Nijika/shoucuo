#ifndef CAR_PARAMS_H
#define CAR_PARAMS_H

#include <stdint.h>

#define CAR_PATH_WEIGHT_FAR       (0.30)
#define CAR_PATH_WEIGHT_MIDDLE    (1.00)
#define CAR_PATH_WEIGHT_NEAR      (1.50)
#define CAR_STEERING_KP_DEFAULT   (5.50)
#define CAR_STEERING_KD_DEFAULT   (18.88)
#define CAR_CURVE_STEERING_KP_DEFAULT         (5.50)
#define CAR_CURVE_STEERING_KD_DEFAULT         (680.00)
#define CAR_CURVE_VARIANCE_DEFAULT             (8.00)
#define CAR_CURVE_BLEND_THRESHOLD1_DEFAULT    (0.15)
#define CAR_CURVE_BLEND_THRESHOLD2_DEFAULT    (0.45)
#define CAR_STANLEY_HEADING_GAIN_DEFAULT      (0.35)
#define CAR_CURVE_FEEDFORWARD_GAIN_DEFAULT    (0.10)
#define CAR_CURVE_PREVIEW_BASE_DEFAULT_CM     (8.0)
#define CAR_CURVE_PREVIEW_SPEED_DEFAULT_CM    (4.0)
#define CAR_WHEEL_ANGLE_MAX_DEG   (40.0)
#define CAR_MAXIMUM_STEERING_US    (280)
#define CAR_LOST_STOP_FRAMES       (50)
#define CAR_CONTROL_ROW_FAR_DEFAULT  (56)
#define CAR_CONTROL_ROW_NEAR_DEFAULT (84)

/* Flat runtime parameters: CPU0 edits them and CPU1 reads the shared values directly. */
typedef struct {
    int16_t base_speed;
    int16_t minimum_speed;
    int16_t curve_slowdown;
    int16_t cruise_speed;
    int16_t straight_speed;
    int16_t curve_speed;
    uint8_t speed_rise_step;
    uint8_t speed_fall_step;
    uint8_t straight_confirm_frames;
    int16_t maximum_steering;
    int16_t lost_search_steering;
    int16_t lost_speed;
    uint16_t lost_stop_frames;

    /* Simple pixel PD; these are the first steering values to tune. */
    float steering_kp;
    float steering_ki;
    float steering_kd;
    float curve_variance_threshold;
    float curve_blend_threshold1;
    float curve_blend_threshold2;

    /* The bend uses separate PD values so straight tuning stays independent. */
    float curve_steering_kp;
    float curve_steering_kd;
    float stanley_heading_gain;
    float curve_feedforward_gain;
    float curve_preview_base_cm;
    float curve_preview_speed_gain_cm;
    float perception_preview_cm;
    float steering_preview_straight_cm;
    float steering_preview_curve_cm;
    float differential_gain;
    int16_t differential_limit;
    float wheelbase_cm;
    float wheel_angle_deg;
    float path_weight_far;
    float path_weight_middle;
    float path_weight_near;
    float confidence_minimum;

    /* The menu can move the pixel band used by the simple PD controller. */
    uint8_t control_row_far;
    uint8_t control_row_near;

    int16_t servo_center_us;
    int16_t servo_travel_us;
    uint8_t servo_reverse;

    uint8_t automatic_threshold;
    uint8_t cross_enabled;
    uint8_t cross_min_both_lost;
    uint16_t cross_min_white_column;
    uint8_t cross_edge_stable_diff;
    uint8_t cross_tear_diff_first;
    uint8_t cross_tear_diff_second;
    uint8_t cross_corner_row_gap_max;
    uint16_t cross_max_lost_rows;
    uint8_t threshold;
    uint8_t dark_is_line;
    uint8_t minimum_line_pixels;
    uint8_t row_step;
    uint16_t roi_top;
    uint16_t roi_bottom;
    int16_t center_offset_pixels;
    uint8_t otsu_row_step;
    uint8_t otsu_column_step;
    uint8_t search_window;
    uint8_t edge_gradient;
    uint16_t track_width_far;
    uint16_t track_width_near;
    uint16_t lookhead;

    uint16_t exposure;
    uint8_t gain;
    int16_t pwm_limit;
    int16_t deadzone;
    int8_t left_direction;
    int8_t right_direction;
    uint8_t running;
} car_params_t;

extern const car_params_t car_default_params;
extern volatile car_params_t car_params;

void car_params_reset(void);
uint8_t car_params_flash_load(void);
void car_params_flash_switch(uint8_t gear);
void car_params_flash_save(uint8_t gear);

#endif
