#include "image.h"
#include "car_shared.h"
#include "longest_white.h"
#include "car_params.h"
#include "algorithm.h"
#include "math_utils.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"
#include <string.h>

static uint16 width;
static uint16 height;
uint8* binary_image;
int16 os_threshold;
int16 lost_line_cnt;
uint8 already_line_lost;
uint8 left_edge[MT9V03X_H], left_index;
uint8 right_edge[MT9V03X_H], right_index;
uint16 Search_Stop_Line;
int16 Center_Line[MT9V03X_H];
int a, stop_cnt;
uint8 Cross_Flag;
uint8 Cross_Count;

uint8 ostu_deal_threshold(void);
void threshold_update(void);
float Calculate_Error(void);
uint8_t protect(const uint8 *gray_frame);

// struct LEFT_EDGE  L_edge[140];
// struct RIGHT_EDGE R_edge[140];
// uint8 L_edge_count=0, R_edge_count = 0;
int16 Left_Line[MT9V03X_H], Right_Line[MT9V03X_H];
uint8 dire_left,dire_right;                                 //记录上一个点的相对位置
uint8 L_search_amount = 140, R_search_amount = 140;  //左右边界搜点时最多允许的点

//int16 left_down_line = -1, right_down_line = -1, left_up_line = -1, right_up_line = -1;
// uint8 left_corner[4], right[4];
// uint8 left_corner_index = 0, right_corner_index = 0;

void image_init(void) {
    width = MT9V03X_W;
    height = MT9V03X_H;
    binary_image = 0;
    os_threshold = -1;
    lost_line_cnt = LOST_LINE;
    already_line_lost = 0;
    for (int i = 0; i < height; ++i) {
        left_edge[i] = right_edge[i] = 0;
        Left_Line[i] = -1;
        Right_Line[i] = MT9V03X_W - 1;
    }
    Cross_Flag = 0;
    Cross_Count = 0;
    left_index = right_index = 0;
    stop_cnt = 10;
    // Search_Stop_Line = 0;
}

void image_deal(uint8 start_y, uint8 end_y, const uint8 *gray_frame, uint8 *binary_frame, car_result_t *result)
{
    uint16 row = car_params.control_row_near;
    uint16 valid_rows = 0;
    uint16 index;
    int16 left;
    int16 right;
    uint8 threshold_lost = 0;
    float image_center = (width - 1) * 0.5;




    if (gray_frame == 0 || binary_frame == 0 || result == 0) return;
    memset(result, 0, sizeof(*result));
    binary_image = binary_frame;
    Cross_Flag = 0;
    Cross_Count = 0;
    (void)start_y;
    (void)end_y;
    memcpy(binary_image, gray_frame, MT9V03X_IMAGE_SIZE);
    if (car_params.automatic_threshold != 0 && ostu_deal_threshold() != 0) {
        if (--lost_line_cnt <= 0) {
            already_line_lost = 1;
            threshold_lost = 1;
        }
    } else {
        lost_line_cnt= LOST_LINE;
        already_line_lost = 0;
    }
    if (car_params.automatic_threshold == 0) os_threshold = car_params.threshold;



    threshold_update();
    if (protect(gray_frame) != 0) {
        if (car_params.running != 0 && stop_cnt <= 0) car_center_stop_request = 1;
    } else {
        stop_cnt = 10;
    }
    Longest_White_Column();
    if (Longest_White_Column_Left[0] < 10) already_line_lost = 1;
    // get_highest();
    // image_draw_rectan(binary_image);
    // search_neighborhood();
    // edge_real_update();



    if (threshold_lost == 0 && car_params.cross_enabled != 0 && //这些是啥啊？？？
        Both_Lost_Time >= car_params.cross_min_both_lost &&
        Search_Stop_Line >= car_params.cross_min_white_column &&
        Left_Lost_Time < car_params.cross_max_lost_rows &&
        Right_Lost_Time < car_params.cross_max_lost_rows)
        shizibuxian();



    Center_Line_Calculate();
    result->cross_detected = Cross_Flag;

    if (row >= height) row = height - 1;
    for (index = 0; index < height; index++) {
        if (Left_Lost_Flag[index] == 0 && Right_Lost_Flag[index] == 0) valid_rows++;
    }
    result->valid_rows = valid_rows;
    result->lost_count = Both_Lost_Time < 0 ? 0 : Both_Lost_Time;
    result->threshold_used = os_threshold < 0 ? 0 : os_threshold > 255 ? 255 : os_threshold;
    result->threshold_far = result->threshold_used;
    result->threshold_middle = result->threshold_used;
    result->threshold_near = result->threshold_used;
    result->strength = (float)valid_rows / height;

    left = Left_Line[row];
    right = Right_Line[row];
    if (left < 0) left = 0;
    if (right < 0) right = 0;
    if (left >= width) left = width - 1;
    if (right >= width) right = width - 1;
    if (right < left) right = left;
    result->left_edge = left;
    result->right_edge = right;
    result->line_width = right - left;
    result->error_pixels = Calculate_Error() + car_params.center_offset_pixels;
    result->center_x = image_center + result->error_pixels;
    result->error_normalized = result->error_pixels / (width * 0.5);
    result->near_error_cm = result->error_pixels * 40.0 / (result->line_width > 0 ? result->line_width : 1);
    if (already_line_lost != 0 || Left_Lost_Flag[row] != 0 || Right_Lost_Flag[row] != 0 ||
        Longest_White_Column_Left[0] < car_params.minimum_line_pixels) return;

    result->line_valid = 1;
}

uint8_t protect(const uint8 *gray_frame) {
    int i, sum = 0;
    int row = 115;
    if (gray_frame == 0) return 0;
    for (i = 0; i < 187; ++i) {
        if (binary_image[row * width + i] == 0 || gray_frame[row * width + i] < 180) {
            ++sum;
        }
    }
    if (sum >= width-18) {
        --stop_cnt;
        // car_stop();
        return 1;
    }
    return 0;
}

float Calculate_Error(void)
{
    int i;
    int end_row = car_params.lookhead;
    float sum = 0;
    float weight_sum = 0;

    if (end_row >= height) end_row = height - 1;
    if (end_row < 30) return 0;

    for (i = 30; i <= end_row; i += 5)
    {
        float weight = i - 20;
        // float weight = 1;
        // float weight;
        // if (end_row == 90) {
        //     weight = i-20;
        // }
        // else {
        //     if (end_row <= 60) {
        //         weight = i+20/(end_row-30)*i;
        //     }
        //     else {
        //         weight = i-20;
        //     }
        // }

        sum += Center_Line[i] * weight;
        weight_sum += weight;
    }

    if(weight_sum == 0) return 0;

    return sum / weight_sum - MT9V03X_W * 0.5;
}

void Center_Line_Calculate(void)
{
    int i;
    int16 temp;

    for (i = 0; i < MT9V03X_H; i++) {
        if (Left_Line[i] < 0) Left_Line[i] = 0;
        if (Left_Line[i] >= MT9V03X_W) Left_Line[i] = MT9V03X_W - 1;
        if (Right_Line[i] < 0) Right_Line[i] = 0;
        if (Right_Line[i] >= MT9V03X_W) Right_Line[i] = MT9V03X_W - 1;
        if (Right_Line[i] < Left_Line[i]) {
            temp = Left_Line[i];
            Left_Line[i] = Right_Line[i];
            Right_Line[i] = temp;
        }
        Center_Line[i] = (Left_Line[i] + Right_Line[i]) / 2;
        // binary_image[i*width+Center_Line[i]] = 0;
        if (binary_image != 0 && Center_Line[i] >= 0 && Center_Line[i] < MT9V03X_W)
            binary_image[i * MT9V03X_W + Center_Line[i]] = 0;
    }
}

// // Legacy edge-following helpers are disabled until their removed interfaces are restored.
// /*-------------------------------------------------------------------------------------------------------------------
//   @brief     左下角点检测
//   @param     起始行，终止行
//   @return    返回角点所在的行数，找不到返回0
//   Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
//   @note      角点检测阈值可根据实际值更改
// -------------------------------------------------------------------------------------------------------------------*/
static void cross_prepare_search_range(int *start, int *end)
{
    int temp;

    if (*start < *end) {
        temp = *start;
        *start = *end;
        *end = temp;
    }
    if (*start >= MT9V03X_H - 1 - 5) *start = MT9V03X_H - 1 - 5;
    if (*end <= MT9V03X_H - Search_Stop_Line) *end = MT9V03X_H - Search_Stop_Line;
    if (*end <= 5) *end = 5;
}

int Find_Left_Down_Point(int start,int end)
{
    int i;
    int left_down_line = 0;

    if (Left_Lost_Time >= car_params.cross_max_lost_rows) return left_down_line;
    cross_prepare_search_range(&start, &end);
    for (i = start; i >= end; i--) {
        if (abs(Left_Line[i] - Left_Line[i + 1]) <= car_params.cross_edge_stable_diff &&
            abs(Left_Line[i + 1] - Left_Line[i + 2]) <= car_params.cross_edge_stable_diff &&
            abs(Left_Line[i + 2] - Left_Line[i + 3]) <= car_params.cross_edge_stable_diff &&
            Left_Line[i] - Left_Line[i - 2] >= car_params.cross_tear_diff_first &&
            Left_Line[i] - Left_Line[i - 3] >= car_params.cross_tear_diff_second &&
            Left_Line[i] - Left_Line[i - 4] >= car_params.cross_tear_diff_second) {
            left_down_line = i;
            break;
        }
    }
    return left_down_line;
}

int Find_Right_Down_Point(int start,int end)
{
    int i;
    int right_down_line = 0;

    if (Right_Lost_Time >= car_params.cross_max_lost_rows) return right_down_line;
    cross_prepare_search_range(&start, &end);
    for (i = start; i >= end; i--) {
        if (abs(Right_Line[i] - Right_Line[i + 1]) <= car_params.cross_edge_stable_diff &&
            abs(Right_Line[i + 1] - Right_Line[i + 2]) <= car_params.cross_edge_stable_diff &&
            abs(Right_Line[i + 2] - Right_Line[i + 3]) <= car_params.cross_edge_stable_diff &&
            Right_Line[i - 2] - Right_Line[i] >= car_params.cross_tear_diff_first &&
            Right_Line[i - 3] - Right_Line[i] >= car_params.cross_tear_diff_second &&
            Right_Line[i - 4] - Right_Line[i] >= car_params.cross_tear_diff_second) {
            right_down_line = i;
            break;
        }
    }
    return right_down_line;
}

int Find_Left_Up_Point(int start,int end)
{
    int i;
    int left_up_line = 0;

    if (Left_Lost_Time >= car_params.cross_max_lost_rows) return left_up_line;
    cross_prepare_search_range(&start, &end);
    for (i = start; i >= end; i--) {
        if (abs(Left_Line[i] - Left_Line[i - 1]) <= car_params.cross_edge_stable_diff &&
            abs(Left_Line[i - 1] - Left_Line[i - 2]) <= car_params.cross_edge_stable_diff &&
            abs(Left_Line[i - 2] - Left_Line[i - 3]) <= car_params.cross_edge_stable_diff &&
            Left_Line[i] - Left_Line[i + 2] >= car_params.cross_tear_diff_first &&
            Left_Line[i] - Left_Line[i + 3] >= car_params.cross_tear_diff_second &&
            Left_Line[i] - Left_Line[i + 4] >= car_params.cross_tear_diff_second) {
            left_up_line = i;
            break;
        }
    }
    return left_up_line;
}

int Find_Right_Up_Point(int start,int end)
{
    int i;
    int right_up_line = 0;

    if (Right_Lost_Time >= car_params.cross_max_lost_rows) return right_up_line;
    cross_prepare_search_range(&start, &end);
    for (i = start; i >= end; i--) {
        if (abs(Right_Line[i] - Right_Line[i - 1]) <= car_params.cross_edge_stable_diff &&
            abs(Right_Line[i - 1] - Right_Line[i - 2]) <= car_params.cross_edge_stable_diff &&
            abs(Right_Line[i - 2] - Right_Line[i - 3]) <= car_params.cross_edge_stable_diff &&
            Right_Line[i + 2] - Right_Line[i] >= car_params.cross_tear_diff_first &&
            Right_Line[i + 3] - Right_Line[i] >= car_params.cross_tear_diff_second &&
            Right_Line[i + 4] - Right_Line[i] >= car_params.cross_tear_diff_second) {
            right_up_line = i;
            break;
        }
    }
    return right_up_line;
}


// void shizibuxian() {
//     int left_down_line = Find_Left_Down_Point(0, height);
//     int left_up_line = Find_Left_Up_Point(0, height);
//     int right_down_line = Find_Right_Down_Point(0, height);
//     int right_up_line = Find_Right_Up_Point(0, height);
// }
void Draw_Line(int x1, int y1, int x2, int y2)
{
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = x1 < x2 ? 1 : -1;
    int sy = y1 < y2 ? 1 : -1;
    int error = dx - dy;

    while (1) {
        Set_Binary_Point(x1, y1);
        if (x1 == x2 && y1 == y2) break;
        if (2 * error > -dy) {
            error -= dy;
            x1 += sx;
        }
        if (2 * error < dx) {
            error += dx;
            y1 += sy;
        }
    }
}
void Draw_Left_Line(int start,int end)
{
    Draw_Line(Left_Line[start],start,Left_Line[end],end);
}
void Draw_Right_Line(int start,int end)
{
    Draw_Line(Right_Line[start],start,Right_Line[end],end);
}
// void Extend_Left_Line(int start,int end)
// {
//     int x;
//     int dx;

//     if(start<4) {
//         Draw_Left_Line(start,end);
//         return;
//     }
//     x=Left_Line[start];
//     dx=Left_Line[start]-Left_Line[start-4];
//     for(int y=start;y<=end;y++)
//     {
//         x+=dx/4;
//         if(x<0) x=0;
//         if(x>=MT9V03X_W) x=MT9V03X_W-1;
//         Set_Binary_Point(x,y);
//     }
// }
// void Extend_Right_Line(int start,int end)
// {
//     int x;
//     int dx;

//     if(start<4) {
//         Draw_Right_Line(start,end);
//         return;
//     }

//     x=Right_Line[start];
//     dx=Right_Line[start]-Right_Line[start-4];
//     for(int y=start;y<=end;y++) {
//         x+=dx/4;
//         if(x<0) x=0;
//         if(x>=MT9V03X_W) x=MT9V03X_W-1;
//         Set_Binary_Point(x,y);
//     }
// }
static void cross_write_binary_point(int row,int column)
{
    if (binary_image == 0 || row < 0 || row >= MT9V03X_H) return;
    if (column < 0) column = 0;
    if (column >= MT9V03X_W) column = MT9V03X_W - 1;
    binary_image[row * MT9V03X_W + column] = 0;
}

void Add_Left_Line(int start,int end)
{
    int i;
    int temp;
    int x1;
    int x2;
    int value;

    if (binary_image == 0) return;
    if (start < 0) start = 0;
    if (end < 0) end = 0;
    if (start >= MT9V03X_H) start = MT9V03X_H - 1;
    if (end >= MT9V03X_H) end = MT9V03X_H - 1;
    if (start > end) {
        temp = start;
        start = end;
        end = temp;
    }

    x1 = Left_Line[start];
    x2 = Left_Line[end];
    if (x1 < 0) x1 = 0;
    if (x1 >= MT9V03X_W) x1 = MT9V03X_W - 1;
    if (x2 < 0) x2 = 0;
    if (x2 >= MT9V03X_W) x2 = MT9V03X_W - 1;

    if (start == end) {
        Left_Line[start] = x1;
        cross_write_binary_point(start, x1);
        return;
    }

    for (i = start; i <= end; i++) {
        value = x1 + (x2 - x1) * (i - start) / (end - start);
        if (value < 0) value = 0;
        if (value >= MT9V03X_W) value = MT9V03X_W - 1;
        Left_Line[i] = value;
        cross_write_binary_point(i, value);
    }
}

void Add_Right_Line(int start,int end)
{
    int i;
    int temp;
    int x1;
    int x2;
    int value;

    if (binary_image == 0) return;
    if (start < 0) start = 0;
    if (end < 0) end = 0;
    if (start >= MT9V03X_H) start = MT9V03X_H - 1;
    if (end >= MT9V03X_H) end = MT9V03X_H - 1;
    if (start > end) {
        temp = start;
        start = end;
        end = temp;
    }

    x1 = Right_Line[start];
    x2 = Right_Line[end];
    if (x1 < 0) x1 = 0;
    if (x1 >= MT9V03X_W) x1 = MT9V03X_W - 1;
    if (x2 < 0) x2 = 0;
    if (x2 >= MT9V03X_W) x2 = MT9V03X_W - 1;

    if (start == end) {
        Right_Line[start] = x1;
        cross_write_binary_point(start, x1);
        return;
    }

    for (i = start; i <= end; i++) {
        value = x1 + (x2 - x1) * (i - start) / (end - start);
        if (value < 0) value = 0;
        if (value >= MT9V03X_W) value = MT9V03X_W - 1;
        Right_Line[i] = value;
        cross_write_binary_point(i, value);
    }
}

void Lengthen_Left_Boundry(int start,int end)
{
    int i;
    int temp;
    int value;
    float k;

    if (binary_image == 0) return;
    if (start < 0) start = 0;
    if (end < 0) end = 0;
    if (start >= MT9V03X_H) start = MT9V03X_H - 1;
    if (end >= MT9V03X_H) end = MT9V03X_H - 1;
    if (start > end) {
        temp = start;
        start = end;
        end = temp;
    }
    if (start <= 5 && start <= end) {
        Add_Left_Line(start, end);
        return;
    }
    if (start > end) return;
    if (start == end) {
        if (Left_Line[start] < 0) Left_Line[start] = 0;
        if (Left_Line[start] >= MT9V03X_W) Left_Line[start] = MT9V03X_W - 1;
        cross_write_binary_point(start, Left_Line[start]);
        return;
    }

    k = (Left_Line[start] - Left_Line[start - 4]) / 5.0;
    for (i = start; i <= end; i++) {
        value = (i - start) * k + Left_Line[start];
        if (value < 0) value = 0;
        if (value >= MT9V03X_W) value = MT9V03X_W - 1;
        Left_Line[i] = value;
        cross_write_binary_point(i, value);
    }
}



void Lengthen_Right_Boundry(int start,int end)
{
    int i;
    int temp;
    int value;
    float k;

    if (binary_image == 0) return;
    if (start < 0) start = 0;
    if (end < 0) end = 0;
    if (start >= MT9V03X_H) start = MT9V03X_H - 1;
    if (end >= MT9V03X_H) end = MT9V03X_H - 1;
    if (start > end) {
        temp = start;
        start = end;
        end = temp;
    }
    if (start <= 5 && start <= end) {
        Add_Right_Line(start, end);
        return;
    }
    if (start > end) return;
    if (start == end) {
        if (Right_Line[start] < 0) Right_Line[start] = 0;
        if (Right_Line[start] >= MT9V03X_W) Right_Line[start] = MT9V03X_W - 1;
        cross_write_binary_point(start, Right_Line[start]);
        return;
    }

    k = (Right_Line[start] - Right_Line[start - 4]) / 5.0;
    for (i = start; i <= end; i++) {
        value = (i - start) * k + Right_Line[start];
        if (value < 0) value = 0;
        if (value >= MT9V03X_W) value = MT9V03X_W - 1;
        Right_Line[i] = value;
        cross_write_binary_point(i, value);
    }
}
void Set_Binary_Point(int x,int y)
{
    if(x<=1||x>=MT9V03X_W-2||y<0||y>=MT9V03X_H)
        return;

    binary_image[y*MT9V03X_W+x]=1;
    binary_image[y*MT9V03X_W+x-1]=1;
    binary_image[y*MT9V03X_W+x+1]=1;
}

static uint8 cross_pair_slope_ok(int row1,int x1,int row2,int x2)
{
    int dx = abs(x2 - x1);
    int dy = abs(row2 - row1);

    /* The straight-entry samples are below 0.83. A limit of 2 keeps margin
     * for pixel noise while rejecting a nearly horizontal false connection. */
    if (dy <= 0) return 0;
    return dx <= dy;
}

void shizibuxian(void)
{
    int left_up;
    int right_up;
    int left_down;
    int right_down;
    int search_start = MT9V03X_H - 6;
    int search_end = MT9V03X_H - Search_Stop_Line;
    int down_end;
    int center = (MT9V03X_W - 1) / 2;

    left_up = Find_Left_Up_Point(search_start, search_end);
    right_up = Find_Right_Up_Point(search_start, search_end);
    if (left_up <= 0 || right_up <= 0) return;
    if (Left_Line[left_up] >= center || Right_Line[right_up] <= center) return;
    if (abs(left_up - right_up) >= car_params.cross_corner_row_gap_max) return;

    down_end = (left_up > right_up ? left_up : right_up) + 2;
    left_down = Find_Left_Down_Point(search_start, down_end);
    right_down = Find_Right_Down_Point(search_start, down_end);
    if (left_down <= left_up) left_down = 0;
    if (right_down <= right_up) right_down = 0;

    Cross_Flag = 1;
    Cross_Count = 3;
    if (left_down > 0) {
        if (cross_pair_slope_ok(left_up, Left_Line[left_up], left_down, Left_Line[left_down]))
            Add_Left_Line(left_up, left_down);
    } else {
        Lengthen_Left_Boundry(left_up - 1, MT9V03X_H - 1);
    }
    if (right_down > 0) {
        if (cross_pair_slope_ok(right_up, Right_Line[right_up], right_down, Right_Line[right_down]))
            Add_Right_Line(right_up, right_down);
    } else {
        Lengthen_Right_Boundry(right_up - 1, MT9V03X_H - 1);
    }
}

// void edge_real_update() {
//     for (int i = 0; i < L_edge_count; ++i) {
//         if (L_edge[i].flag)
//             Left_Line[L_edge[i].col] = cc_i16_max(Left_Line[L_edge[i].col], L_edge[i].row);
//     }
//     for (int i = 0; i < R_edge_count; ++i) {
//         if (R_edge[i].flag)
//             Right_Line[R_edge[i].col] = cc_i16_min(Right_Line[R_edge[i].col], R_edge[i].row);
//     }
//     for (int i = 0; i < L_edge_count && i < R_edge_count; ++i) {
//         if (!L_edge[i].flag && !R_edge[i].flag) {
//             ++a;
//         }
//     }
// }


// /*---------------------------------------------------------------
//  【函    数】search_neighborhood
//  【功    能】八邻域找边界
//  【参    数】无
//  【返 回 值】无
//  【注意事项】
//  ----------------------------------------------------------------*/

// void search_neighborhood(void)
// {
//     L_edge_count = 0;//左边点个数清0
//     R_edge_count = 0;//右边点个数清0

//     if(left_findflag)//如果左边界点存在并找到,则开始爬线
//     {
//         //变量声明
//         L_edge[0].row = L_start_y;
//         L_edge[0].col = L_start_x;
//         L_edge[0].flag = 1;
//         int16 curr_row = L_start_y;//初始化行坐标
//         int16 curr_col = L_start_x;//初始化列坐标
//         dire_left = 0; //初始化上个边界点的来向
//         //开始搜线，最多取150个点，不会往下搜，共7个方位
//         for(int i = 1;i < L_search_amount; i++)    //最多搜索150个点
//         {
//             ////越界退出 行越界和列越界（向上向下向左向右）
//             if(curr_row+1 < Boundary_search_end || curr_row>IMAGE_H-1)  break;
//             if (curr_col-1 <= 0 && curr_col+1 < IMAGE_W) break;
//             //搜线过程
//             if(dire_left != 2&&image_use[curr_row-1][curr_col-1]==0&&image_use[curr_row-1][curr_col]==1)   //左上黑，2，右边白
//             {
//                 curr_row = curr_row -1;
//                 curr_col = curr_col -1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 7;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=3&&image_use[curr_row-1][curr_col+1]==0&&image_use[curr_row][curr_col+1]==1)    //右上黑，3，下边白
//             {
//                 curr_row = curr_row -1;
//                 curr_col = curr_col + 1;
//                 L_edge_count = L_edge_count + 1;
//                 dire_left = 6;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(image_use[curr_row-1][curr_col]==0&&image_use[curr_row-1][curr_col+1]==1)                  //正上黑，1，右白
//             {
//                 curr_row = curr_row - 1;
//                 L_edge_count = L_edge_count + 1;
//                 dire_left = 0;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=5&&image_use[curr_row][curr_col-1]==0&&image_use[curr_row-1][curr_col-1]==1)     //正左黑，5，上白
//             {
//                 curr_col = curr_col - 1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 4;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=4&&image_use[curr_row][curr_col+1]==0&&image_use[curr_row+1][curr_col+1]==1)  //正右黑，4，下白
//             {
//                 curr_col = curr_col + 1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 5;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=6&&image_use[curr_row+1][curr_col-1]==0&&image_use[curr_row][curr_col-1]==1)    //左下黑，6，上白
//             {
//                 curr_row = curr_row + 1;
//                 curr_col = curr_col -1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 3;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else if(dire_left!=7&&image_use[curr_row+1][curr_col+1]==0&&image_use[curr_row+1][curr_col]==1)    //右下黑，7，左白
//             {
//                 curr_row = curr_row + 1;
//                 curr_col = curr_col + 1;
//                 L_edge_count = L_edge_count +1;
//                 dire_left = 2;
//                 L_edge[i].row = curr_row;
//                 L_edge[i].col = curr_col;
//                 L_edge[i].flag = 1;
//             }
//             else
//                 break;
//         }
//     }

//     if(right_findflag)//如果右边界存在并搜到
//     {
//         R_edge[0].row = R_start_y;
//         R_edge[0].col = R_start_x;
//         R_edge[0].flag = 1;
//         int16 curr_row = R_start_y;
//         int16 curr_col = R_start_x;
//         dire_right = 0;
//         for(int i = 1;i<R_search_amount;i++)
//         {
//             ////越界退出 行越界和列越界（向上向下向左向右）
//             if(curr_row < Boundary_search_end || curr_row>IMAGE_H-1||curr_row+1<Boundary_search_end)  break;
//             //爬线过程
//             if(curr_col<IMAGE_W&&dire_right!=3&&image_use[curr_row-1][curr_col+1]==0&&image_use[curr_row-1][curr_col]==1)    //右上黑，3，左白
//             {
//                 curr_row = curr_row - 1;
//                 curr_col = curr_col + 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 6;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(dire_right!=2&&image_use[curr_row-1][curr_col-1]==0&&image_use[curr_row][curr_col-1]==1) //左上黑，2，下白
//             {
//                 curr_row = curr_row-1;
//                 curr_col = curr_col-1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 7;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(image_use[curr_row-1][curr_col]==0&&image_use[curr_row-1][curr_col-1]==1)                  //正上黑，1，左白
//             {
//                 curr_row = curr_row - 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 0;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(dire_right!=4&&image_use[curr_row][curr_col+1]==0&&image_use[curr_row-1][curr_col+1]==1)   //正右黑，4，上白
//             {
//                 curr_col = curr_col + 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 5;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(dire_right!=5&&image_use[curr_row][curr_col-1]==0&&image_use[curr_row+1][curr_col-1]==1)   //正左黑，5，下白
//             {
//                 curr_col = curr_col-1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 4;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }


//             else if(dire_right!=6&&image_use[curr_row+1][curr_col-1]==0&&image_use[curr_row+1][curr_col]==1)   //左下黑，6，右白
//             {
//                 curr_row = curr_row + 1;
//                 curr_col = curr_col - 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 3;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else if(dire_right!=7&&image_use[curr_row+1][curr_col+1]==0&&image_use[curr_row][curr_col+1]==1)   //右下黑，7，上白
//             {
//                 curr_row = curr_row + 1;
//                 curr_col = curr_col + 1;
//                 R_edge_count = R_edge_count + 1;
//                 dire_right = 2;
//                 R_edge[i].row = curr_row;
//                 R_edge[i].col = curr_col;
//                 R_edge[i].flag = 1;
//             }
//             else
//                 break;
//         }
//     }
// }


// void image_draw_rectan(uint8(*image)[width])
// {
//     uint8 i = 0;
//     for (i = 0; i < height; i++)
//     {
//         image[i][0] = 0;
//         image[i][1] = 0;
//         image[i][width - 1] = 0;
//         image[i][width - 2] = 0;
//     }
//     for (i = 0; i < width; i++)
//     {
//         image[0][i] = 0;
//         image[1][i] = 0;
//     }
// }



uint8 ostu_deal_threshold(void)
{
    uint32 white_cnt = 0;
    uint32 PixelSum = 0;
    int16 PixelMax = 0;
    int16 PixelMin = 255;
    int PixelCnt[GrayScale] = {0};
    float PixelPro[GrayScale] = {0};
    uint32 Graysum = 0;
    int i;
    int j;
    int GrayCur;
    float w0 = 0;
    float w1 = 0;
    float u0tmp = 0;
    float u1tmp = 0;
    float u0;
    float u1;
    float deltaTmp;
    float deltaMax = 0;

    os_threshold = car_params.threshold;
    for (i = 0; i < height; i += 2) {
        for (j = 0; j < width; j += 2) {
            GrayCur = binary_image[i * width + j];
            if (GrayCur > 200) white_cnt++;
            Graysum += GrayCur;
            PixelCnt[GrayCur]++;
            PixelSum++;
            if (GrayCur > PixelMax) PixelMax = GrayCur;
            if (GrayCur < PixelMin) PixelMin = GrayCur;
        }
    }

    if (PixelSum == 0) return 1;
    if (PixelMin >= PixelMax) return white_cnt < PixelSum * 0.1;

    for (i = PixelMin; i < PixelMax; i++) PixelPro[i] = (float)PixelCnt[i] / PixelSum;
    for (j = PixelMin; j < PixelMax; j++) {
        w0 += PixelPro[j];
        u0tmp += j * PixelPro[j];
        w1 = 1 - w0;
        if (w0 <= 0 || w1 <= 0) continue;
        u1tmp = (float)Graysum / PixelSum - u0tmp;
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        deltaTmp = w0 * w1 * (u0 - u1) * (u0 - u1);
        if (deltaTmp > deltaMax) {
            deltaMax = deltaTmp;
            os_threshold = j;
        }
    }
    return white_cnt < PixelSum * 0.1;
}

void threshold_update() {
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            if (binary_image[i*width+j] > os_threshold) {
                binary_image[i*width+j] = 1;
            }
            else {
                binary_image[i*width+j] = 0;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     寻找最长白列     CYH-NYY
// 参数说明
// 返回参数
// 使用示例     get_highest();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

int lwline = 94;   //最长白列所在x
int lw = 119;      //最长白列顶部y
int xxx = 0;


void get_highest(void)
{
    lwline = width / 2;
    lw = height;
    xxx = 0;


    // 从中间区域向左右搜索
    for(int x = width/3; x < width*2/3; x += 3)
    {
        // 最下面一行必须是白，否则不是道路入口
        if(binary_image[(height-1)*width+x] == 1)
        {
            for(int y = height-1; y >= 0; y--)
            {
                // 白黑黑跳变
                if(binary_image[y*width+x] == 1 &&
                   binary_image[(y-1)*width+x] == 0 &&
                   binary_image[(y-2)*width+x] == 0)
                {
                    // 找最长白列
                    if(y < lw)
                    {
                        lw = y;
                        lwline = x;
                    }

                    break;
                }


                // 防止越界
                if(y <= 2)
                {
                    if(y < lw)
                    {
                        lw = y;
                        lwline = x;
                    }
                    break;
                }
            }
        }
    }
}
