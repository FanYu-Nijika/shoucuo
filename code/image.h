#ifndef CC_IMAGE_H
#define CC_IMAGE_H

#include "zf_common_headfile.h"

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
