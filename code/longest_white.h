#ifndef __LONGEST__
#define __LONGEST__

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"

extern int16 Longest_White_Column_Left[2];
extern int16 Longest_White_Column_Right[2];
extern uint16 White_Column[MT9V03X_W];
extern uint8 Left_Lost_Flag[MT9V03X_H];
extern uint8 Right_Lost_Flag[MT9V03X_H];
extern int16 Boundry_Start_Left;
extern int16 Boundry_Start_Right;
extern int16 Road_Wide[MT9V03X_H];
extern int16 Left_Lost_Time;
extern int16 Right_Lost_Time;
extern int16 Both_Lost_Time;
void Longest_White_Column(void);

#endif
