#ifndef CC_IMAGE_H
#define CC_IMAGE_H

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"
#define LOST_LINE 50
#define GrayScale 256

// struct LEFT_EDGE
// {
//     int16 row;  //行坐标
//     int16 col;  //列坐标
//     uint8 flag; //存在边界的标志
// };
// struct RIGHT_EDGE
// {
//     int16 row;  //行坐标
//     int16 col;  //列坐标
//     uint8 flag; //存在边界的标志
// };

extern uint8* binary_image;

void image_init();
void image_deal(uint8 start_y, uint8 end_y);
uint8 ostu_deal_threshold();
void threshold_update();

#endif
