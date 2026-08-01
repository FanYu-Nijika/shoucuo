#include "car_params.h"

#include <string.h>

#include "zf_common_headfile.h"

#define CAR_PARAMS_FLASH_SECTOR          (0)
#define CAR_PARAMS_FLASH_PAGE            (8)
#define CAR_PARAMS_FLASH_FORMAT_VERSION  (2)
#define CAR_PARAMS_FLASH_PROFILE_COUNT   (4)
#define CAR_PARAMS_FLASH_PROFILE_START   (2)
#define CAR_PARAMS_FLASH_PROFILE_WORDS   ((sizeof(car_params_t) + 3) / 4)

static void car_params_flash_factory_init(void);

/* Defaults stay in read-only memory; runtime parameters live in shared RAM. */
const car_params_t car_default_params = {
    .base_speed = 2400,
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
    .steering_kp = 3.0,
    // .steering_ki = 0.0,
    .steering_kd = 3.8,
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
     * PWM duty 660/760/860 corresponds to about 1320/1520/1720 us at 50 Hz.
     * Positive image error means track is right, so servo output is reversed.
     */
    .servo_center_us = 1540,
    .servo_travel_us = 220,
    .servo_reverse = 1,

    .automatic_threshold = 1,
    .cross_enabled = 1,
    .cross_min_both_lost = 8,
    .cross_min_white_column = 30,
    .cross_edge_stable_diff = 5,
    .cross_tear_diff_first = 5,
    .cross_tear_diff_second = 10,
    .cross_corner_row_gap_max = 18,
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
    .lookhead = 90,

    .exposure = 256,
    .gain = 4,
    .pwm_limit = 5000,
    // .deadzone = 15,
    .left_direction = -1,
    .right_direction = -1,
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

static uint8_t car_params_flash_gear_valid(uint8_t gear)
{
    return gear >= 1 && gear <= CAR_PARAMS_FLASH_PROFILE_COUNT ? 1 : 0;
}

static uint32_t car_params_flash_profile_offset(uint8_t gear)
{
    return CAR_PARAMS_FLASH_PROFILE_START + (gear - 1) * CAR_PARAMS_FLASH_PROFILE_WORDS;
}

static uint8_t car_params_flash_buffer_empty(void)
{
    uint32_t index;
    uint8_t all_ff = 1;
    uint8_t all_zero = 1;

    for (index = 0; index < EEPROM_PAGE_LENGTH; index++) {
        if (flash_union_buffer[index].uint32_type != 0xFFFFFFFF) all_ff = 0;
        if (flash_union_buffer[index].uint32_type != 0) all_zero = 0;
    }

    return all_ff != 0 || all_zero != 0;
}

static uint8_t car_params_flash_profile_empty(uint32_t offset)
{
    uint32_t index;
    uint8_t all_ff = 1;
    uint8_t all_zero = 1;

    for (index = 0; index < CAR_PARAMS_FLASH_PROFILE_WORDS; index++) {
        if (flash_union_buffer[offset + index].uint32_type != 0xFFFFFFFF) all_ff = 0;
        if (flash_union_buffer[offset + index].uint32_type != 0) all_zero = 0;
    }

    return all_ff != 0 || all_zero != 0;
}

uint8_t car_params_flash_load(void)
{
    uint8_t gear = 1;
    flash_read_page_to_buffer(CAR_PARAMS_FLASH_SECTOR, CAR_PARAMS_FLASH_PAGE);
    if (car_params_flash_buffer_empty() != 0 ||
        flash_union_buffer[1].uint32_type != CAR_PARAMS_FLASH_FORMAT_VERSION) {
        car_params_flash_factory_init();
    } else {
        gear = flash_union_buffer[0].uint8_type;
        if (car_params_flash_gear_valid(gear) == 0) gear = 1;
    }

    car_params_flash_switch(gear);
    return gear;
}

void car_params_flash_switch(uint8_t gear)
{
    uint32_t profile_offset;
    car_params_t stored_params;

    if (car_params_flash_gear_valid(gear) == 0) return;
    profile_offset = car_params_flash_profile_offset(gear);
    if (profile_offset + CAR_PARAMS_FLASH_PROFILE_WORDS > EEPROM_PAGE_LENGTH) return;

    flash_read_page_to_buffer(CAR_PARAMS_FLASH_SECTOR, CAR_PARAMS_FLASH_PAGE);
    if (car_params_flash_profile_empty(profile_offset) != 0) {
        stored_params = car_default_params;
    } else {
        memcpy(&stored_params, &flash_union_buffer[profile_offset], sizeof(car_params_t));
        if (stored_params.servo_center_us == 1400 && stored_params.servo_travel_us == 200)
            stored_params.servo_center_us = 1520;
    }
    stored_params.running = 0;
    __dsync();
    car_params = stored_params;
    car_params.running = 0;
    __dsync();
}

void car_params_flash_save(uint8_t gear)
{
    uint32_t profile_offset;
    car_params_t stored_params = car_params;

    if (car_params_flash_gear_valid(gear) == 0) return;
    profile_offset = car_params_flash_profile_offset(gear);
    if (profile_offset + CAR_PARAMS_FLASH_PROFILE_WORDS > EEPROM_PAGE_LENGTH) return;

    stored_params.running = 0;
    flash_read_page_to_buffer(CAR_PARAMS_FLASH_SECTOR, CAR_PARAMS_FLASH_PAGE);
    flash_union_buffer[0].uint32_type = 0;
    flash_union_buffer[0].uint8_type = gear;
    flash_union_buffer[1].uint32_type = CAR_PARAMS_FLASH_FORMAT_VERSION;
    memcpy(&flash_union_buffer[profile_offset], &stored_params, sizeof(car_params_t));
    flash_write_page_from_buffer(CAR_PARAMS_FLASH_SECTOR, CAR_PARAMS_FLASH_PAGE);
}

static void car_params_flash_factory_init(void)
{
    uint8_t gear;
    uint32_t offset;

    flash_read_page_to_buffer(CAR_PARAMS_FLASH_SECTOR, CAR_PARAMS_FLASH_PAGE);

    flash_union_buffer[0].uint32_type = 0;
    flash_union_buffer[0].uint8_type = 1;
    flash_union_buffer[1].uint32_type = CAR_PARAMS_FLASH_FORMAT_VERSION;

    for (gear = 1; gear <= 4; gear++) {
        offset = car_params_flash_profile_offset(gear);
        memcpy(&flash_union_buffer[offset], &car_default_params, sizeof(car_params_t));
    }

    flash_write_page_from_buffer(CAR_PARAMS_FLASH_SECTOR, CAR_PARAMS_FLASH_PAGE);
}
