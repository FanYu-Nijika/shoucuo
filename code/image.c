#include "image.h"

#include <string.h>

cc_image_u8_t cc_image_u8_make(uint8 *data, uint16 width, uint16 height, uint16 stride)
{
    cc_image_u8_t image;
    image.data = data;
    image.width = width;
    image.height = height;
    image.stride = stride < width ? width : stride;
    return image;
}

uint8 cc_image_u8_is_valid(const cc_image_u8_t *image)
{
    return image != 0 && image->data != 0 && image->width != 0 &&
           image->height != 0 && image->stride >= image->width;
}

uint8 cc_image_u8_otsu_threshold(const cc_image_u8_t *image, uint16 roi_top,
                                  uint16 roi_bottom, uint8 row_step,
                                  uint8 column_step, uint8 fallback)
{
    uint32 histogram[256] = {0};
    uint32 sample_count = 0;
    uint64 total_sum = 0;
    uint64 class_sum = 0;
    uint32 class_count = 0;
    uint32 row;
    uint32 column;
    uint16 value;
    uint8 best = fallback;
    float best_variance = -1.0f;

    if (!cc_image_u8_is_valid(image)) return fallback;
    if (row_step == 0U) row_step = 1U;
    if (column_step == 0U) column_step = 1U;
    if (roi_bottom == 0U || roi_bottom > image->height) roi_bottom = image->height;
    if (roi_top >= roi_bottom) return fallback;

    for (row = roi_top; row < roi_bottom; row += row_step) {
        for (column = 0U; column < image->width; column += column_step) {
            uint8 pixel = image->data[row * image->stride + column];
            histogram[pixel]++;
            total_sum += pixel;
            sample_count++;
        }
    }
    if (sample_count == 0U) return fallback;

    for (value = 0U; value < 255U; value++) {
        uint32 class1_count;
        float class0_mean;
        float class1_mean;
        float difference;
        float variance;

        class_count += histogram[value];
        class_sum += (uint64)value * histogram[value];
        class1_count = sample_count - class_count;
        if (class_count == 0U || class1_count == 0U) continue;
        class0_mean = (float)class_sum / (float)class_count;
        class1_mean = (float)(total_sum - class_sum) / (float)class1_count;
        difference = class0_mean - class1_mean;
        variance = (float)class_count * (float)class1_count * difference * difference;
        if (variance > best_variance) {
            best_variance = variance;
            best = (uint8)value;
        }
    }
    return best;
}

uint8 cc_image_u8_row_span(const cc_image_u8_t *image, uint16 row,
                            uint8 threshold_value, uint8 dark_is_line,
                            uint16 *left, uint16 *right, uint16 *pixel_count)
{
    uint16 column;
    uint16 current_start = 0U;
    uint16 current_count = 0U;
    uint16 best_start = 0U;
    uint16 best_count = 0U;

    if (left != 0) *left = 0U;
    if (right != 0) *right = 0U;
    if (pixel_count != 0) *pixel_count = 0U;
    if (!cc_image_u8_is_valid(image) || row >= image->height) return 0U;

    for (column = 0U; column < image->width; column++) {
        uint8 pixel = image->data[row * image->stride + column];
        uint8 is_line = dark_is_line != 0U ? pixel < threshold_value : pixel >= threshold_value;
        if (is_line != 0U) {
            if (current_count == 0U) current_start = column;
            current_count++;
        } else {
            if (current_count > best_count) {
                best_start = current_start;
                best_count = current_count;
            }
            current_count = 0U;
        }
    }
    if (current_count > best_count) {
        best_start = current_start;
        best_count = current_count;
    }
    if (best_count == 0U) return 0U;
    if (left != 0) *left = best_start;
    if (right != 0) *right = best_start + best_count - 1U;
    if (pixel_count != 0) *pixel_count = best_count;
    return 1U;
}

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
