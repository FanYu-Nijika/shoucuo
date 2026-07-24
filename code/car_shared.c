#include "car_shared.h"

#include <string.h>

#include "zf_common_headfile.h"

#pragma section all "cpu1_dsram"

uint8_t car_gray_frame[CAR_IMAGE_HEIGHT][CAR_IMAGE_WIDTH];
volatile car_result_t car_result;
volatile uint32_t car_frame_sequence;
volatile uint32_t car_frame_period_ms;
volatile uint8_t car_frame_ready;
volatile uint8_t car_result_ready;

#pragma section all restore

void car_shared_clear(void)
{
    memset((void *)&car_result, 0, sizeof(car_result));
    car_frame_sequence = 0U;
    car_frame_period_ms = 20U;
    car_frame_ready = 0U;
    car_result_ready = 0U;
    __dsync();
}

