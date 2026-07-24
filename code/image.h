#ifndef CC_IMAGE_H
#define CC_IMAGE_H

#include "zf_common_headfile.h"

// CPU0复制完一帧图像后置1，CPU1处理完成后清零。
extern volatile uint8 cpu0_done;

// CPU1完成阈值、最长白列和偏差计算后置1，CPU0读取结果后清零。
extern volatile uint8 cpu1_done;

// CPU0复制给CPU1处理的完整灰度帧，避免摄像头DMA覆盖正在处理的数据。
extern uint8 image_buffer[MT9V03X_H][MT9V03X_W];

// 当前帧使用的大津法阈值
extern volatile uint8 image_auto_threshold;
extern volatile uint8 threshold;

// 最长白列是否有效
extern volatile uint8 track_valid;

extern volatile uint16 best_col;
extern volatile uint16 best_row;

extern volatile int16 track_error;

// 最长白列扫描范围和起始行，可在菜单中直接调整。
extern volatile uint16 image_scan_start_col;
extern volatile uint16 image_scan_end_col;
extern volatile uint16 image_check_row;
extern volatile uint16 image_search_start_row;

uint8 image_binary(uint8 *image);
void image_find_longest_white_line(uint8 *image);
int16 image_get_error(void);
void image_display(void);

#endif
