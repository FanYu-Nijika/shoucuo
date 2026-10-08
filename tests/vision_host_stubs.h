#ifndef VISION_HOST_STUBS_H
#define VISION_HOST_STUBS_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/* Keep host builds independent of the AURIX startup and peripheral headers. */
#define CODE_HEADFILE_H_
#define _zf_common_typedef_h_
#define _zf_device_mt9v03x_h_
#define _zf_common_headfile_h_
#define _zf_common_font_h
#define _zf_device_ips200_h_

typedef uint8_t uint8;
typedef int8_t int8;
typedef uint16_t uint16;
typedef int16_t int16;
typedef uint32_t uint32;

#define MT9V03X_W (188)
#define MT9V03X_H (120)
#define MT9V03X_IMAGE_SIZE (MT9V03X_W * MT9V03X_H)

extern uint8 *binary_image;
extern int16 Left_Line[MT9V03X_H];
extern int16 Right_Line[MT9V03X_H];
extern uint16 Search_Stop_Line;

void __dsync(void);

#endif
