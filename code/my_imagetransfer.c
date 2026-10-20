#include "my_imagetransfer.h"
#include "sd_card.h"
#include "hdu_dma.h"
#include "image.h"
#include "zf_common_headfile.h"

#include <string.h>

#pragma section all "cpu1_dsram"
uint8 My_Send_Data[Send_Data_Len];
uint32 my_blockNum;
volatile uint32 my_picture_count;
volatile uint32 my_export_count;
volatile uint8 my_sd_ready;
volatile uint8 my_sd_recording;
volatile uint8 my_sd_exporting;
volatile uint8 my_sd_busy;
volatile uint8 my_sd_error;

/* The example's raw layout: six image sectors, one parameter sector and one line sector. */
static uint8 Pixels_zip[6 * 512];
static float my_car_data[128];
static uint8 my_lines[512];
static uint32 last_save_ms;
static uint8 saved_first_frame;
static uint8 export_uart_ready;
#pragma section all restore

static uint8 line_byte(int16 value)
{
    if (value < 0 || value >= CAR_IMAGE_WIDTH) return 255;
    return value;
}

void My_SdCard_Init(void)
{
    if (my_sd_recording || my_sd_exporting || my_sd_busy) return;
    my_sd_ready = sd_init() == MSD_RESPONSE_NO_ERROR;
    my_sd_error = my_sd_ready == 0;
}

void My_Tranfer_Init(void)
{
    My_Data_Uart_Init();
}

void My_SdCard_Start(void)
{
    if (!my_sd_ready) {
        my_sd_error = 1;
        return;
    }
    if (!my_sd_ready || my_sd_recording || my_sd_busy || my_sd_exporting) return;
    my_blockNum = 0;
    my_picture_count = 0;
    my_export_count = 0;
    my_sd_error = 0;
    saved_first_frame = 0;
    __dsync();
    my_sd_recording = 1;
}

void My_SdCard_Stop(void)
{
    my_sd_recording = 0;
    __dsync();
}

void My_SdCard_Save(const uint8 *picture, const car_result_t *result)
{
    uint32 pixel;
    uint16 row;

    if (!my_sd_recording || !my_sd_ready || picture == 0 || result == 0) return;
    if (saved_first_frame && result->capture_time_ms - last_save_ms < 100) return;
    /* CPU1 owns all three buffers until this whole frame is stored. CPU0 waits before export. */
    my_sd_busy = 1;
    __dsync();
    if (!my_sd_recording) {
        my_sd_busy = 0;
        return;
    }
    memset(Pixels_zip, 0, sizeof(Pixels_zip));
    for (pixel = 0; pixel < CAR_IMAGE_WIDTH * CAR_IMAGE_HEIGHT; pixel++) {
        if (picture[pixel] != 0) Pixels_zip[pixel / 8] |= 1u << (7 - pixel % 8);
    }
    memset(my_car_data, 0, sizeof(my_car_data));
    my_car_data[0] = result->frame_sequence;
    my_car_data[1] = result->capture_time_ms;
    my_car_data[2] = result->error_pixels;
    my_car_data[3] = result->curvature;
    my_car_data[4] = result->strength;
    my_car_data[5] = result->threshold_used;
    my_car_data[6] = result->threshold_far;
    my_car_data[7] = result->threshold_middle;
    my_car_data[8] = result->threshold_near;
    my_car_data[9] = result->valid_rows;
    my_car_data[10] = result->lost_count;
    my_car_data[11] = result->repaired_rows;
    my_car_data[12] = result->preview_row;
    my_car_data[13] = result->center_x;
    my_car_data[14] = result->line_valid;
    my_car_data[15] = result->cross_detected;
    my_car_data[16] = result->processing_time_us;
    my_car_data[17] = result->near_error_cm;
    my_car_data[18] = result->curve_variance;
    my_car_data[19] = result->heading_error;
    my_car_data[20] = result->error_normalized;
    memset(my_lines, 255, sizeof(my_lines));
    for (row = 0; row < CAR_IMAGE_HEIGHT; row++) {
        my_lines[row * 4] = 255;
        my_lines[row * 4 + 1] = line_byte(Left_Line[row]);
        my_lines[row * 4 + 2] = line_byte(Right_Line[row]);
        my_lines[row * 4 + 3] = line_byte(Center_Line[row]);
    }
    if (sd_WriteDisk(Pixels_zip, my_blockNum, 6) != MSD_RESPONSE_NO_ERROR ||
        sd_WriteDisk((uint8 *)my_car_data, my_blockNum + 6, 1) != MSD_RESPONSE_NO_ERROR ||
        sd_WriteDisk(my_lines, my_blockNum + 7, 1) != MSD_RESPONSE_NO_ERROR) {
        my_sd_error = 1;
        my_sd_recording = 0;
    } else {
        my_blockNum += 8;
        my_picture_count++;
        last_save_ms = result->capture_time_ms;
        saved_first_frame = 1;
    }
    __dsync();
    my_sd_busy = 0;
}

static void Get_SendData(void)
{
    uint16 index;
    uint8 byte;
    uint32 bits;

    memset(My_Send_Data, 0, sizeof(My_Send_Data));
    for (index = 0; index < 8; index++) My_Send_Data[index] = index + 1;
    My_Send_Data[8] = CAR_IMAGE_WIDTH;
    My_Send_Data[9] = CAR_IMAGE_HEIGHT;
    My_Send_Data[10] = 50;
    My_Send_Data[11] = 1;
    for (index = 0; index < 50; index++) {
        memcpy(&bits, &my_car_data[index], sizeof(bits));
        for (byte = 0; byte < 4; byte++) {
            My_Send_Data[12 + index * 8 + byte] = bits >> (byte * 8);
            My_Send_Data[12 + index * 8 + 4 + byte] = ~bits >> (byte * 8);
        }
    }
    /* The unused fixed parameter is still a valid zero + inverted-zero pair. */
    memset(&My_Send_Data[416], 255, 4);
    memcpy(&My_Send_Data[420], my_lines, CAR_IMAGE_HEIGHT * 4);
    memcpy(&My_Send_Data[420 + CAR_IMAGE_HEIGHT * 4], Pixels_zip, Pixels_Len);
}

void My_SdCard_Read(void)
{
    if (sd_ReadDisk(Pixels_zip, my_blockNum, 6) != MSD_RESPONSE_NO_ERROR ||
        sd_ReadDisk((uint8 *)my_car_data, my_blockNum + 6, 1) != MSD_RESPONSE_NO_ERROR ||
        sd_ReadDisk(my_lines, my_blockNum + 7, 1) != MSD_RESPONSE_NO_ERROR) {
        my_sd_error = 1;
        return;
    }
    Get_SendData();
    send_to_computer();
    my_blockNum += 8;
    my_export_count++;
}

void My_SdCard_Export(void)
{
    if (!my_sd_ready || my_picture_count == 0) {
        my_sd_error = 1;
        return;
    }
    if (!my_sd_ready || my_sd_recording || my_sd_busy || my_sd_exporting || my_picture_count == 0) return;
    my_blockNum = 0;
    my_export_count = 0;
    my_sd_error = 0;
    export_uart_ready = 0;
    my_sd_exporting = 1;
}

void My_SdCard_Task(uint8 vehicle_busy)
{
    if (!my_sd_exporting) return;
    if (vehicle_busy) {
        my_sd_exporting = 0;
    } else if (!my_sd_busy) {
        if (!export_uart_ready) {
            My_Tranfer_Init();
            export_uart_ready = 1;
        }
        My_SdCard_Read();
        if (my_sd_error || my_export_count >= my_picture_count) my_sd_exporting = 0;
    }
    if (!my_sd_exporting && export_uart_ready) {
        My_Data_Uart_Restore();
        export_uart_ready = 0;
    }
}
