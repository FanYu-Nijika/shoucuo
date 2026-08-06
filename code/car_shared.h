#ifndef CAR_SHARED_H
#define CAR_SHARED_H

#include <stdint.h>

#define CAR_IMAGE_WIDTH     (188)
#define CAR_IMAGE_HEIGHT    (120)
#define CAR_FRAME_SLOT_COUNT (2)
#define CAR_FRAME_SLOT_NONE (0xFF)

typedef enum {
    CAR_FRAME_FREE = 0,
    CAR_FRAME_READY,
    CAR_FRAME_READING,
    CAR_FRAME_DISPLAY_READY,
    CAR_FRAME_DISPLAY_READING
} car_frame_state_t;

/* CPU1 publishes one complete result; CPU0 only displays and applies it. */
typedef struct {
    uint8_t line_valid;
    uint8_t cross_detected;
    uint16_t valid_rows;
    uint16_t repaired_rows;
    uint16_t lost_count;
    uint16_t left_edge;
    uint16_t right_edge;
    uint16_t line_width;
    float center_x;
    float error_pixels;
    float near_error_cm;
    float error_normalized;
    float heading_error;
    float strength;
    uint8_t threshold_used;
    uint8_t threshold_far;
    uint8_t threshold_middle;
    uint8_t threshold_near;
    float curve_variance;
    float curvature;
    float curve_preview_cm;
    float stanley_feedback_us;
    float curvature_feedforward_us;
    int16_t servo_command_us;
    int16_t left_command;
    int16_t right_command;
    uint8_t frame_slot;
    uint32_t frame_sequence;
    uint32_t processing_time_us;
    // int16_t error_pixels;
} car_result_t;

extern uint8_t car_gray_frames[CAR_FRAME_SLOT_COUNT][CAR_IMAGE_HEIGHT][CAR_IMAGE_WIDTH];
extern uint8_t car_binary_frames[CAR_FRAME_SLOT_COUNT][CAR_IMAGE_HEIGHT][CAR_IMAGE_WIDTH];
extern volatile car_result_t car_result;
extern volatile uint32_t car_frame_sequence;
extern volatile uint32_t car_frame_period_ms;
extern volatile car_frame_state_t car_frame_state[CAR_FRAME_SLOT_COUNT];
extern volatile uint32_t car_frame_slot_sequence[CAR_FRAME_SLOT_COUNT];
extern volatile uint32_t car_frame_slot_period_ms[CAR_FRAME_SLOT_COUNT];
extern volatile uint8_t car_display_slot;
extern volatile uint8_t car_display_read_slot;
extern volatile uint8_t car_display_pending_free_slot;
extern volatile uint8_t car_result_ready;
extern volatile uint32_t car_capture_drop_count;
extern volatile uint32_t car_processing_drop_count;
extern volatile uint32_t car_display_drop_count;
extern volatile uint32_t car_result_drop_count;
extern volatile uint32_t car_vision_max_us;
extern volatile uint32_t car_camera_frame_count;
extern volatile uint32_t car_camera_last_frame_ms;
extern volatile uint32_t car_camera_age_ms;

void car_shared_clear(void);
uint8_t car_shared_has_pending_frame(void);

#endif
