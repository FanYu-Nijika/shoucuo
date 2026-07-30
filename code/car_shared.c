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

#pragma section all restore

void car_shared_clear(void)
{
    uint8_t slot;

    memset((void *)car_gray_frames, 0, sizeof(car_gray_frames));
    memset((void *)car_binary_frames, 0, sizeof(car_binary_frames));
    memset((void *)&car_result, 0, sizeof(car_result));
    car_frame_sequence = 0U;
    car_frame_period_ms = 20U;
    car_display_slot = CAR_FRAME_SLOT_NONE;
    car_result_ready = 0U;
    car_capture_drop_count = 0U;
    car_processing_drop_count = 0U;
    car_display_drop_count = 0U;
    car_result_drop_count = 0U;
    car_vision_max_us = 0U;
    for (slot = 0U; slot < CAR_FRAME_SLOT_COUNT; slot++) {
        car_frame_state[slot] = CAR_FRAME_FREE;
        car_frame_slot_sequence[slot] = 0U;
        car_frame_slot_period_ms[slot] = 20U;
    }
    __dsync();
}

uint8_t car_shared_has_pending_frame(void)
{
    uint8_t slot;

    for (slot = 0U; slot < CAR_FRAME_SLOT_COUNT; slot++) {
        if (car_frame_state[slot] == CAR_FRAME_READY || car_frame_state[slot] == CAR_FRAME_READING) return 1U;
    }
    return 0U;
}
