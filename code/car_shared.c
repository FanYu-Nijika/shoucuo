#include "car_shared.h"

#include <string.h>

#include "zf_common_headfile.h"

#pragma section all "cpu1_dsram"

uint8_t car_gray_frames[CAR_FRAME_SLOT_COUNT][CAR_IMAGE_HEIGHT][CAR_IMAGE_WIDTH];
volatile car_result_t car_result;
volatile uint32_t car_frame_sequence;
volatile uint32_t car_frame_period_ms;
volatile car_frame_state_t car_frame_state[CAR_FRAME_SLOT_COUNT];
volatile uint32_t car_frame_slot_sequence[CAR_FRAME_SLOT_COUNT];
volatile uint32_t car_frame_slot_period_ms[CAR_FRAME_SLOT_COUNT];
volatile uint8_t car_result_ready;
volatile uint32_t car_capture_drop_count;
volatile uint32_t car_processing_drop_count;
volatile uint32_t car_display_drop_count;
volatile uint32_t car_result_drop_count;
volatile uint32_t car_vision_max_us;

#pragma section all restore

#pragma section all "cpu0_dsram"

uint8_t car_binary_frames[CAR_FRAME_SLOT_COUNT][CAR_IMAGE_HEIGHT][CAR_IMAGE_WIDTH];
volatile uint8_t car_display_slot;
volatile uint8_t car_display_read_slot;
volatile uint8_t car_display_pending_free_slot;
volatile uint32_t car_camera_frame_count;
volatile uint32_t car_camera_last_frame_ms;
volatile uint32_t car_camera_age_ms;
volatile uint8_t car_center_stop_request;

#pragma section all restore

void car_shared_clear(void)
{
    uint8_t slot;

    memset((void *)car_gray_frames, 0, sizeof(car_gray_frames));
    memset((void *)car_binary_frames, 0, sizeof(car_binary_frames));
    memset((void *)&car_result, 0, sizeof(car_result_t));
    car_frame_sequence = 0;
    car_frame_period_ms = 20;
    car_display_slot = CAR_FRAME_SLOT_NONE;
    car_display_read_slot = CAR_FRAME_SLOT_NONE;
    car_display_pending_free_slot = CAR_FRAME_SLOT_NONE;
    car_result_ready = 0;
    car_capture_drop_count = 0;
    car_processing_drop_count = 0;
    car_display_drop_count = 0;
    car_result_drop_count = 0;
    car_vision_max_us = 0;
    car_camera_frame_count = 0;
    car_camera_last_frame_ms = 0;
    car_camera_age_ms = 0;
    car_center_stop_request = 0;
    for (slot = 0; slot < CAR_FRAME_SLOT_COUNT; slot++) {
        car_frame_state[slot] = CAR_FRAME_FREE;
        car_frame_slot_sequence[slot] = 0;
        car_frame_slot_period_ms[slot] = 20;
    }
    __dsync();
}

uint8_t car_shared_has_pending_frame(void)
{
    uint8_t slot;

    for (slot = 0; slot < CAR_FRAME_SLOT_COUNT; slot++) {
        if (car_frame_state[slot] == CAR_FRAME_READY || car_frame_state[slot] == CAR_FRAME_READING) return 1;
    }
    return 0;
}
