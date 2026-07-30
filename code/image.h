#ifndef CC_IMAGE_H
#define CC_IMAGE_H

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"
#include "car_shared.h"

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

extern int16 Left_Line[MT9V03X_H];
extern int16 Right_Line[MT9V03X_H];

extern uint16 Search_Stop_Line;

extern uint8 Cross_Flag;
extern uint8 Cross_Count;

extern int16 Center_Line[MT9V03X_H];

void Center_Line_Calculate(void);
void image_init(void);
void image_deal(uint8 start_y, uint8 end_y, const uint8 *gray_frame, uint8 *binary_frame, car_result_t *result);
float Calculate_Error(void);
uint8 ostu_deal_threshold();
void threshold_update();

int Find_Left_Down_Point(int start,int end);
int Find_Right_Down_Point(int start,int end);
int Find_Left_Up_Point(int start,int end);
int Find_Right_Up_Point(int start,int end);

void shizibuxian(void);
void Add_Left_Line(int start,int end);
void Add_Right_Line(int start,int end);

void Lengthen_Left_Boundry(int start,int end);
void Lengthen_Right_Boundry(int start,int end);
void Draw_Line(int x1,int y1,int x2,int y2);
void Draw_Left_Line(int start,int end);
void Draw_Right_Line(int start,int end);

void Extend_Left_Line(int start,int end);
void Extend_Right_Line(int start,int end);

void Set_Binary_Point(int x,int y);

#endif
