#ifndef CC_IMAGE_H
#define CC_IMAGE_H

#include "zf_common_headfile.h"

// 当前帧使用的大津法阈值
extern uint8 threshold;

extern uint16 best_col;
extern uint16 best_row;

extern int16 track_error;
uint8 image_binary(uint8 *image);
void image_find_longest_white_line(uint8 *image);
int16 image_get_error(void);
void image_display(void);

#endif