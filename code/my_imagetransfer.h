#ifndef MY_IMAGETRANSFER_H
#define MY_IMAGETRANSFER_H

#include "zf_common_typedef.h"
#include "car_shared.h"

/* Keep the supplied example's packet layout, using this car's full image. */
#define Pixels_Len ((CAR_IMAGE_WIDTH * CAR_IMAGE_HEIGHT + 7) / 8)
#define Send_Data_Len (12 + 50 * 8 + 8 + CAR_IMAGE_HEIGHT * 4 + Pixels_Len)

extern uint8 My_Send_Data[Send_Data_Len];
extern uint32 my_blockNum;
extern volatile uint32 my_picture_count;
extern volatile uint32 my_export_count;
extern volatile uint8 my_sd_ready;
extern volatile uint8 my_sd_recording;
extern volatile uint8 my_sd_exporting;
extern volatile uint8 my_sd_busy;
extern volatile uint8 my_sd_error;

void My_SdCard_Init(void);
void My_Tranfer_Init(void);
void My_SdCard_Start(void);
void My_SdCard_Stop(void);
void My_SdCard_Save(const uint8 *picture, const car_result_t *result);
void My_SdCard_Read(void);
void My_SdCard_Export(void);
void My_SdCard_Task(uint8 vehicle_busy);

#endif
