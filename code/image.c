#include "image.h"

uint8 threshold = 100;
uint8 last_threshold = 100;

uint16 best_col = 94;
uint16 best_row = 119;

int16 track_error = 0;


/*
 * 输入：
 * 摄像头的一维灰度图像。
 *
 * 处理：
 * 隔行隔列统计灰度直方图，再使用大津法寻找阈值。
 *
 * 输出：
 * 返回当前帧使用的二值化阈值。
 */
uint8 image_binary(uint8 *image)
{
    uint16 width = MT9V03X_W;
    uint16 height = MT9V03X_H - 4;

    uint32 PixelCount[256] = {0};

    uint32 PixelSum = 0;
    uint32 GraySum = 0;

    uint8 result_threshold = last_threshold;

    // 隔行、隔列采样，减少大津法运行时间
    for (int row = 0; row < height; row += 2)
    {
        for (int col = 0; col < width; col += 2)
        {
            uint8 Gray = image[row * width + col];

            PixelCount[Gray]++;
            GraySum += Gray;
            PixelSum++;
        }
    }

    // 当前图像的平均灰度
    float u = (float)GraySum / PixelSum;

    float maxVariance = 0.0f;
    float w0 = 0.0f;
    float avgValue = 0.0f;

    /*
     * 每次把当前灰度值作为阈值。
     * 0~i作为一类，i+1~255作为另一类。
     * 类间方差最大时，说明两类分得最开。
     */
    for (int i = 0; i < 256; i++)
    {
        float pixel_probability =
            (float)PixelCount[i] / PixelSum;

        w0 += pixel_probability;
        avgValue += i * pixel_probability;

        /*
         * w0为0时没有前景。
         * w0为1时没有背景。
         * 这两种情况不能计算类间方差。
         */
        if (w0 <= 0.0f || w0 >= 1.0f)
        {
            continue;
        }

        float gray_difference = avgValue / w0 - u;

        float variance =
            gray_difference *
            gray_difference *
            w0 /
            (1.0f - w0);

        if (variance > maxVariance)
        {
            maxVariance = variance;
            result_threshold = (uint8)i;
        }
    }

    // 本帧阈值异常时继续使用上一帧阈值
    if (result_threshold > 40 && result_threshold < 200)
    {
        last_threshold = result_threshold;
    }
    else
    {
        result_threshold = last_threshold;
    }

    threshold = result_threshold;

    return threshold;
}


/*
 * 输入：
 * 摄像头灰度图和已经计算好的threshold。
 *
 * 处理：
 * 在第64列到第124列之间，每隔3列向上扫描。
 * 寻找每一列中“白、黑、黑”的跳变位置。
 *
 * 输出：
 * best_col保存最长白列所在列。
 * best_row保存这根白列达到的最高行。
 */
void image_find_longest_white_line(uint8 *image)
{
    best_col = 94;
    best_row = 119;

    for (int col = 64; col < 124; col += 3) {
        /*
         * 第80行是白色，才认为这一列可能存在
         * 从图像下方连续向上延伸的白色区域。
         */
        if (image[80 * MT9V03X_W + col] >= threshold) {
            for (int row = 85; row >= 2; row--) {
   
                if (image[row * MT9V03X_W + col] >= threshold &&
                    image[(row - 1) * MT9V03X_W + col] < threshold &&
                    image[(row - 2) * MT9V03X_W + col] < threshold) {
                    if (row < best_row) {
                        best_row = row;
                        best_col = col;
                    }

                    break;
                }

                if (row == 2) {
                    if (row < best_row) {
                        best_row = row;
                        best_col = col;
                    }
                }
            }
        }
    }
}


/*
 * 图像宽度为188时，图像中心是94。
 *
 * 结果为正：
 * 最长白列在图像右侧。
 *
 * 结果为负：
 * 最长白列在图像左侧。
 */
int16 image_get_error(void)
{
    track_error =
        (int16)best_col -
        (int16)(MT9V03X_W / 2);

    return track_error;
}


/*
 * 显示原始灰度图，并标记当前找到的最长白列最高点。
 *
 * 这个函数应该由CPU0调用。
 * CPU1只负责图像计算，不操作屏幕。
 */
void image_display(void)
{
    ips200_show_gray_image(
        0,
        0,
        mt9v03x_image[0],
        MT9V03X_W,
        MT9V03X_H,
        MT9V03X_W,
        MT9V03X_H,
        0
    );

    ips200_draw_point(best_col, best_row, RGB565_RED);

    ips200_show_int(
        0,
        MT9V03X_H + 5,
        threshold,
        3
    );

    ips200_show_int(
        50,
        MT9V03X_H + 5,
        best_col,
        3
    );

    ips200_show_int(
        100,
        MT9V03X_H + 5,
        best_row,
        3
    );

    ips200_show_int(
        150,
        MT9V03X_H + 5,
        track_error,
        4
    );
}