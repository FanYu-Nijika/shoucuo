#ifndef CC_IMAGE_H
#define CC_IMAGE_H

#include "zf_common_headfile.h"

typedef struct {
    uint8 *data;
    uint16 width;
    uint16 height;
    uint16 stride;
} cc_image_u8_t;

cc_image_u8_t cc_image_u8_make(uint8 *data, uint16 width, uint16 height, uint16 stride);
uint8 cc_image_u8_is_valid(const cc_image_u8_t *image);
uint8 cc_image_u8_otsu_threshold(const cc_image_u8_t *image, uint16 roi_top,
                                 uint16 roi_bottom, uint8 row_step,
                                 uint8 column_step, uint8 fallback);
uint8 cc_image_u8_row_span(const cc_image_u8_t *image, uint16 row,
                           uint8 threshold, uint8 dark_is_line,
                           uint16 *left, uint16 *right, uint16 *pixel_count);

extern volatile uint8 cpu0_done;
extern volatile uint8 cpu1_done;
extern uint8 image_buffer[MT9V03X_H][MT9V03X_W];
extern volatile uint8 image_auto_threshold;
extern volatile uint8 threshold;
extern volatile uint8 track_valid;

extern volatile uint16 best_col;
extern volatile uint16 best_row;

extern volatile int16 track_error;

extern volatile uint16 image_scan_start_col;
extern volatile uint16 image_scan_end_col;
extern volatile uint16 image_check_row;
extern volatile uint16 image_search_start_row;

uint8 image_binary(uint8 *image);
void image_find_longest_white_line(uint8 *image);
int16 image_get_error(void);
void image_display(void);

#endif
