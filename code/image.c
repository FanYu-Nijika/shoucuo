#include "image.h"

#include <string.h>

volatile uint8 cpu0_done = 0;
volatile uint8 cpu1_done = 0;

uint8 image_buffer[MT9V03X_H][MT9V03X_W];
volatile uint8 image_auto_threshold = 1;
volatile uint8 threshold = 100;
volatile uint8 track_valid = 0;
volatile uint16 best_col = MT9V03X_W / 2;
volatile uint16 best_row = MT9V03X_H - 1;
volatile int16 track_error = 0;

volatile uint16 image_scan_start_col = 64;
volatile uint16 image_scan_end_col = 124;
volatile uint16 image_check_row = 80;
volatile uint16 image_search_start_row = 85;

static uint8 last_threshold = 100;
static uint32 PixelCount[256];

uint8 image_binary(uint8 *image)
{
    uint16 width = MT9V03X_W;
    uint16 height = MT9V03X_H - 4;
    uint32 PixelSum = 0;
    uint32 GraySum = 0;
    uint8 result_threshold = last_threshold;
    float u;
    float maxVariance = 0.0f;
    float w0 = 0.0f;
    float avgValue = 0.0f;

    memset(PixelCount, 0, sizeof(PixelCount));

    for (int row = 0; row < height; row += 2) {
        for (int col = 0; col < width; col += 2) {
            uint8 Gray = image[row * width + col];
            PixelCount[Gray]++;
            GraySum += Gray;
            PixelSum++;
        }
    }

    u = 1.*GraySum / PixelSum;

    for (int i = 0; i < 256; i++) {
        float pixel_probability = (float)PixelCount[i] / PixelSum;
        float gray_difference;
        float variance;

        w0 += pixel_probability;
        avgValue += i * pixel_probability;

        gray_difference = avgValue / w0 - u;
        variance = gray_difference * gray_difference * w0 / (1.0f - w0);
        if (variance > maxVariance) {
            maxVariance = variance;
            result_threshold = i;
        }
    }

    // 本帧阈值异常时继续使用上一帧阈值
    if (result_threshold > 40 && result_threshold < 200) {
        last_threshold = result_threshold;
    } else {
        result_threshold = last_threshold;
    }

    threshold = result_threshold;
    return threshold;
}

void image_find_longest_white_line(uint8 *image)
{
    uint32 best_col_sum = 0;
    uint16 best_col_count = 0;

    best_col = MT9V03X_W / 2;
    best_row = MT9V03X_H - 1;
    track_valid = 0;

    for (int col = image_scan_start_col; col < image_scan_end_col; col += 3) {
        int16 candidate_row = -1;

        for (int row = image_search_start_row; row >= 2; row--) {
            if (image[row * MT9V03X_W + col] >= threshold && image[(row - 1) * MT9V03X_W + col] < threshold && image[(row - 2) * MT9V03X_W + col] < threshold) {
                candidate_row = row;
                break;
            }

            if (row == 2 && image[row * MT9V03X_W + col] >= threshold) candidate_row = row;
        }

        if (candidate_row < 0) continue;

        // 图像行号越小，表示白色区域延伸得越高
        if (!track_valid || candidate_row < best_row) {
            best_row = candidate_row;
            best_col_sum = col;
            best_col_count = 1;
            track_valid = 1;
        } else if (candidate_row == best_row) {
            // 多列同样高时取平均，避免best_col在相邻扫描列之间抖动。
            best_col_sum += col;
            best_col_count++;
        }
    }

    if (track_valid && best_col_count) best_col = best_col_sum / best_col_count;
}

int16 image_get_error(void)
{
    if (track_valid) {
        track_error = best_col - MT9V03X_W / 2;
    } else {
        track_error = 0;
    }

    return track_error;
}

void image_display(void)
{
    ips200_full(RGB565_BLACK);
    ips200_show_gray_image(0, 0, image_buffer[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    if (track_valid) ips200_draw_point(best_col, best_row, RGB565_RED);

    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(196, 8, "CAMERA");
    ips200_show_string(196, 32, "THR");
    ips200_show_uint(244, 32, threshold, 3);
    ips200_show_string(196, 56, "COL");
    ips200_show_uint(244, 56, best_col, 3);
    ips200_show_string(196, 80, "ROW");
    ips200_show_uint(244, 80, best_row, 3);
    ips200_show_string(196, 104, "ERR");
    ips200_show_int(244, 104, track_error, 4);
    ips200_show_string(196, 128, track_valid ? "VALID" : "LOST");
    ips200_show_string(196, 216, "AUX1 BACK");
}
