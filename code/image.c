#include "image.h"
#include "car_shared.h"
#include "longest_white.h"
#include "car_params.h"
#include "algorithm.h"
#include "math_utils.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"
#include <string.h>

#define CROSS_SKEW_SPAN_MIN 18
#define CROSS_SKEW_SPAN_MAX 60
#define CROSS_SKEW_LOST_MIN 3
#define CROSS_SKEW_CROSS_MIN 4
#define CROSS_SKEW_TURN_WINDOW 14
#define CROSS_SKEW_TURN_GAP 5
#define CROSS_SKEW_TURN_DIFF 6
#define DYNAMIC_PREVIEW_SPEED_THRESHOLD 4000
#define DYNAMIC_PREVIEW_ROW_OFFSET 18
#define DYNAMIC_PREVIEW_TIME_MS 1800
#define DYNAMIC_PREVIEW_CURVE_HOLD_FRAMES 15

// #define CROSS_SKEW_NONE 11
// #define CROSS_SKEW_LEFT 45
// #define CROSS_SKEW_RIGHT 14

static uint16 width;
static uint16 height;
uint8* binary_image;
int16 os_threshold;
int16 lost_line_cnt;
uint8 already_line_lost;
uint8 left_edge[MT9V03X_H], left_index;
uint8 right_edge[MT9V03X_H], right_index;
uint8 LCenter[MT9V03X_H];
uint16 Search_Stop_Line;
int16 Center_Line[MT9V03X_H];
static int16 Dynamic_Center_Line[MT9V03X_H];
int a, stop_cnt;
uint8 Cross_Flag;
uint8 Cross_Count;
uint8 xieru_type;
static int current_preview_row = CAR_CONTROL_ROW_NEAR_DEFAULT;
static int last_preview_base_row = -1;
static uint32 straight_preview_time_ms = 0;
static uint8 preview_curve_mode = 0;
static uint8 curve_preview_hold_frames = 0;
static int curve_preview_hold_row = CAR_CONTROL_ROW_NEAR_DEFAULT;
static uint8 curve_variance_valid = 0;

uint8 ostu_deal_threshold(void);
void threshold_update(void);
float Calculate_Error(void);
static int Update_Dynamic_Preview_Row(float curvature, uint8 line_valid);
static float Calculate_Preview_Weight(int row, int top_row, int middle_row, int near_row);
static float Calculate_Curve_Variance(void);
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
    current_preview_row = CAR_CONTROL_ROW_NEAR_DEFAULT;
    last_preview_base_row = -1;
    straight_preview_time_ms = 0;
    preview_curve_mode = 0;
    curve_preview_hold_frames = 0;
    curve_preview_hold_row = CAR_CONTROL_ROW_NEAR_DEFAULT;
    curve_variance_valid = 0;
    // Search_Stop_Line = 0;
}

void image_deal(uint8 start_y, uint8 end_y, uint8 *gray_frame, uint8 *binary_frame, car_result_t *result)
{
    uint16 row = car_params.control_row_near;
    uint16 valid_rows = 0;
    uint16 index;
    int16 left;
    int16 right;
    uint8 threshold_lost = 0;
    uint8 line_valid;
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
    // Longest_White_Column();
    Longest_White_Column();
    xieru_type = Judge_xierushizi_type();

    if (Longest_White_Column_Left[0] < 10) already_line_lost = 1;

    if (threshold_lost == 0 && car_params.cross_enabled != 0) {
        if (xieru_type != CROSS_SKEW_NONE) {
            shizibuxian(xieru_type);
        } else if (Both_Lost_Time >= car_params.cross_min_both_lost &&
                Search_Stop_Line >= car_params.cross_min_white_column &&
                Left_Lost_Time < car_params.cross_max_lost_rows &&
                Right_Lost_Time < car_params.cross_max_lost_rows) {
            shizibuxian(CROSS_SKEW_NONE);
        }
    }

    Center_Line_Calculate();
    if (Longest_White_Column_Left[0] < 10) already_line_lost = 1;
    // get_highest();
    // image_draw_rectan(binary_image);
    // search_neighborhood();
    // edge_real_update();



    // Center_Line_Calculate();
    result->cross_detected = Cross_Flag;

    if (row < 3) row = 3;
    if (row > height - 3) row = height - 3;
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
    line_valid = already_line_lost == 0 && Left_Lost_Flag[row] == 0 && Right_Lost_Flag[row] == 0 &&
        Longest_White_Column_Left[0] >= car_params.minimum_line_pixels;
    result->curve_variance = Calculate_Curve_Variance();
    if (car_params.curvature_scale > 0.0)
        result->curvature = result->curve_variance / car_params.curvature_scale;
    if (result->curvature < 0.0) result->curvature = 0.0;
    if (result->curvature > 1.0) result->curvature = 1.0;
    result->preview_row = Update_Dynamic_Preview_Row(result->curvature, line_valid);
    result->error_pixels = Calculate_Error() + car_params.center_offset_pixels;
    result->center_x = image_center + result->error_pixels;
    result->error_normalized = result->error_pixels / (width * 0.5);
    result->near_error_cm = result->error_pixels * 40.0 / (result->line_width > 0 ? result->line_width : 1);
    /* Draw after all calculations so the marker cannot alter current frame detection. */
    for (index = 0; index < MT9V03X_W - 1; index++)
        gray_frame[result->preview_row * MT9V03X_W + index] = 0;
    if (line_valid == 0) return;

    result->line_valid = 1;
}

// int Find_dynamic_middleSeed(uint8 type) //找真正的面向的中心,用于判断左右斜入
// {
//     int x, y, max_x = 0, max_length = 0, length, bottom_row = MT9V03X_H - 1, top_row = MT9V03X_H - 60;
//     int bottom_center = (Left_Line[bottom_row] + Right_Line[bottom_row]) / 2;

//     /*
//      * 在最下面一行左右边界之间，
//      * 找向上延伸最长的白色列。
//      */
//     for (x = Left_Line[bottom_row] + 1; x < Right_Line[bottom_row]; ++x) {
//         length = 0;
//         for (y = bottom_row - 1; y >= top_row; y--) {
//             if (binary_image[(y + 1) * width + x] != 0 && binary_image[y * width + x] == 0){
//                 length = bottom_row - y;
//                 // if (length > max_length) {
//                 //     max_length = length;
//                 //     max_x = x;
//                 // }
//                 break;
//             }
//             if (binary_image[(y + 1) * width + x] == 0 && binary_image[y * width + x] == 0){
//                 break;
//             }
//             if (y == top_row) {
//                 length = bottom_row - top_row;
//                 if (length > max_length) {
//                     max_length = length;
//                     max_x = x;
//                 }
//             }
//         }
//         if (length > max_length || (length == max_length && length > 0 && abs(x - bottom_center) < abs(max_x - bottom_center))) {
//             max_length = length;
//             max_x = x;
//         }
//     }

//     // if (max_x <= 0) max_x = 1;
//     if (max_x <= 0) max_x = bottom_center;

//     if (max_x >= width - 1) max_x = width - 2;

//     /*
//      * 从底部向上逐�?�扫描�?
//      * 每一行都�? max_x 为中心，分别寻找左右边界�?
//      */
//     for (y = bottom_row - 1; y > top_row; y--)
//     {
//         if (binary_image[y * width + max_x] == 0) {
//             break;
//         }
//         for (x = max_x; x < width - 1; x++) {
//             if (x == width - 2 || (binary_image[y * width + x] != 0 &&  binary_image[y * width + x - 1] != 0 &&  binary_image[y * width + x + 1] == 0)) {
//                 Right_Line[y] = x;
//                 break;
//             }
//         }
//         for (x = max_x; x > 0; x--) {
//             if (x == 1 || (binary_image[y * width + x] != 0 && binary_image[y * width + x - 1] == 0 && binary_image[y * width + x + 1] != 0)) {
//                 Left_Line[y] = x;
//                 break;
//             }
//         }

//         Center_Line[y] = (Left_Line[y] + Right_Line[y]) / 2;
//     }
// }
uint8 Build_Dynamic_Center_Line(void)
{
    int x, y, left, right, found_left, found_right, max_x = 0, max_length = 0, length, count = 0, bottom_row = MT9V03X_H - 1, top_row = MT9V03X_H - 60, bottom_center, value;
    float sum_y = 0, sum_x = 0, sum_yy = 0, sum_yx = 0, denominator, k, b;

    if (Left_Line[bottom_row] >= Right_Line[bottom_row]) return 0;

    bottom_center = (Left_Line[bottom_row] + Right_Line[bottom_row]) / 2;

    for (x = Left_Line[bottom_row] + 1; x < Right_Line[bottom_row]; x++) {
        length = 0;

        for (y = bottom_row - 1; y >= top_row; y--) {
            if (binary_image[(y + 1) * width + x] != 0 && binary_image[y * width + x] == 0) {
                length = bottom_row - y;
                break;
            }

            if (binary_image[(y + 1) * width + x] == 0 && binary_image[y * width + x] == 0) break;

            if (y == top_row) length = bottom_row - top_row;
        }

        if (length > max_length ||
            (length == max_length && length > 0 && abs(x - bottom_center) < abs(max_x - bottom_center))) {
            max_length = length;
            max_x = x;
        }
    }

    if (max_x <= 0) max_x = bottom_center;
    if (max_x >= width - 1) max_x = width - 2;

    for (y = bottom_row - 1; y > top_row; y--) {
        if (binary_image[y * width + max_x] == 0) break;

        left = max_x;
        right = max_x;
        found_left = 0;
        found_right = 0;

        for (x = max_x; x < width - 1; x++) {
            if (x == width - 2 ||
                (binary_image[y * width + x] != 0 &&
                 binary_image[y * width + x - 1] != 0 &&
                 binary_image[y * width + x + 1] == 0)) {
                right = x;
                found_right = 1;
                break;
            }
        }

        for (x = max_x; x > 0; x--) {
            if (x == 1 ||
                (binary_image[y * width + x] != 0 &&
                 binary_image[y * width + x - 1] == 0 &&
                 binary_image[y * width + x + 1] != 0)) {
                left = x;
                found_left = 1;
                break;
            }
        }

        if (found_left == 0 || found_right == 0 || left >= right) continue;

        value = (left + right) / 2;
        sum_y += y;
        sum_x += value;
        sum_yy += y * y;
        sum_yx += y * value;
        count++;
    }

    if (count < 8) {
        for (y = 0; y < MT9V03X_H; y++) Dynamic_Center_Line[y] = bottom_center;
        return 0;
    }

    denominator = count * sum_yy - sum_y * sum_y;

    if (denominator > -0.001f && denominator < 0.001f) {
        for (y = 0; y < MT9V03X_H; y++) Dynamic_Center_Line[y] = bottom_center;
        return 0;
    }

    k = (count * sum_yx - sum_y * sum_x) / denominator;
    b = (sum_x - k * sum_y) / count;

    for (y = 0; y < MT9V03X_H; y++) {
        value = k * y + b;

        if (value < 1) value = 1;
        if (value >= MT9V03X_W - 1) value = MT9V03X_W - 2;

        Dynamic_Center_Line[y] = value;
    }

    return 1;
}
int Find_xierushizi_up_point(uint8 type)
{
    int i;
    int search_bottom = 64;
    int search_top = MT9V03X_H - Search_Stop_Line;

    if (search_bottom > MT9V03X_H - 3) search_bottom = MT9V03X_H - 3;
    if (search_top < 5) search_top = 5;

    for (i = search_bottom; i >= search_top; i--) {
        if (type == CROSS_SKEW_LEFT && Left_Lost_Flag[i] == 0 &&
            Left_Line[i] - Left_Line[i + 1] > CROSS_SKEW_JUMP &&
            Left_Line[i] - Left_Line[i + 2] > CROSS_SKEW_JUMP) {
            return i;
        }

        if (type == CROSS_SKEW_RIGHT && Right_Lost_Flag[i] == 0 &&
            Right_Line[i] - Right_Line[i + 1] < -CROSS_SKEW_JUMP &&
            Right_Line[i] - Right_Line[i + 2] < -CROSS_SKEW_JUMP) {
            return i;
        }
    }

    return 0;
}

// #define CROSS_SKEW_NONE   0
// #define CROSS_SKEW_LEFT   1
// #define CROSS_SKEW_RIGHT  2


// uint8 Judge_xierushizi_type(void)
// {
//     int i, left_cnt = 0, right_cnt = 0;
//     int bottom_row = MT9V03X_H - 1;
//     int start_row = 15, end_row = 55;
//     int valid_top = MT9V03X_H - Search_Stop_Line;
//     int bottom_center;

//     if (Left_Line[bottom_row] >= Right_Line[bottom_row]) return CROSS_SKEW_NONE;

//     bottom_center = (Left_Line[bottom_row] + Right_Line[bottom_row]) / 2;

//     if (start_row < valid_top) start_row = valid_top;
//     if (end_row > MT9V03X_H - 2) end_row = MT9V03X_H - 2;

//     for (i = start_row; i < end_row; i++) {
//         if (Left_Lost_Flag[i] == 0 && Left_Line[i] > bottom_center + CROSS_SKEW_MARGIN) left_cnt++;
//         if (Right_Lost_Flag[i] == 0 && Right_Line[i] < bottom_center - CROSS_SKEW_MARGIN) right_cnt++;
//     }

//     if (left_cnt >= CROSS_SKEW_COUNT_MIN && right_cnt < CROSS_SKEW_COUNT_MIN) {
//         if (Find_xierushizi_up_point(CROSS_SKEW_LEFT) > 5) return CROSS_SKEW_LEFT;
//     }

//     if (right_cnt >= CROSS_SKEW_COUNT_MIN && left_cnt < CROSS_SKEW_COUNT_MIN) {
//         if (Find_xierushizi_up_point(CROSS_SKEW_RIGHT) > 5) return CROSS_SKEW_RIGHT;
//     }

//     return CROSS_SKEW_NONE;
// }
static uint8 Skew_Pair_Is_Valid(const int16 *line, const uint8 *lost_flag, int up_point, int down_point, uint8 left_side)
{
    int i;
    int expected;
    int deviation;
    int max_deviation = 0;
    int lost_count = 0;

    /* 顶部碎片也会形成短双点，真实补线还必须有足够跨度和正确的左右方向。 */
    if (up_point <= 5 || down_point - up_point < CROSS_SKEW_SPAN_MIN) return 0;
    if (down_point - up_point > CROSS_SKEW_SPAN_MAX) return 0;
    if (left_side != 0 && line[up_point] <= line[down_point] + CROSS_SKEW_MARGIN) return 0;
    if (left_side == 0 && line[up_point] >= line[down_point] - CROSS_SKEW_MARGIN) return 0;

    for (i = up_point + 1; i < down_point; i++) {
        if (lost_flag[i] != 0) lost_count++;
        expected = line[up_point] + (line[down_point] - line[up_point]) * (i - up_point) / (down_point - up_point);
        deviation = abs(line[i] - expected);
        if (deviation > max_deviation) max_deviation = deviation;
    }

    return lost_count >= CROSS_SKEW_LOST_MIN || max_deviation >= CROSS_SKEW_JUMP;
}

static int Find_Skew_Turn_Point(const int16 *line, const uint8 *lost_flag, int up_point, uint8 left_side)
{
    int i;
    int search_start = up_point + CROSS_SKEW_SPAN_MIN;
    int search_end = up_point + CROSS_SKEW_SPAN_MAX;
    int upper_start;
    int upper_end;
    int lower_end;
    int candidate = 0;
    int candidate_value = left_side != 0 ? -1 : MT9V03X_W;
    int upper_value = left_side != 0 ? MT9V03X_W : 0;
    int lower_value = left_side != 0 ? MT9V03X_W : 0;
    uint8 upper_found = 0;
    uint8 lower_found = 0;

    /* 斜入下口常是圆滑回头而不是边界撕裂，只有峰值两侧都明显退回时才接受。 */
    if (up_point <= 5) return 0;
    if (search_end > MT9V03X_H - 12) search_end = MT9V03X_H - 12;
    if (search_start > search_end) return 0;

    for (i = search_start; i <= search_end; i++) {
        if (lost_flag[i] != 0) continue;
        if ((left_side != 0 && line[i] > candidate_value) || (left_side == 0 && line[i] < candidate_value)) {
            candidate = i;
            candidate_value = line[i];
        }
    }
    if (candidate == 0) return 0;

    upper_start = candidate - CROSS_SKEW_TURN_WINDOW;
    if (upper_start < up_point + CROSS_SKEW_TURN_GAP) upper_start = up_point + CROSS_SKEW_TURN_GAP;
    upper_end = candidate - CROSS_SKEW_TURN_GAP;
    lower_end = candidate + CROSS_SKEW_TURN_WINDOW;
    if (lower_end > MT9V03X_H - 6) lower_end = MT9V03X_H - 6;

    for (i = upper_start; i <= upper_end; i++) {
        if (lost_flag[i] != 0) continue;
        if (left_side != 0 && line[i] < upper_value) upper_value = line[i];
        if (left_side == 0 && line[i] > upper_value) upper_value = line[i];
        upper_found = 1;
    }
    for (i = candidate + CROSS_SKEW_TURN_GAP; i <= lower_end; i++) {
        if (lost_flag[i] != 0) continue;
        if (left_side != 0 && line[i] < lower_value) lower_value = line[i];
        if (left_side == 0 && line[i] > lower_value) lower_value = line[i];
        lower_found = 1;
    }
    if (upper_found == 0 || lower_found == 0) return 0;

    if (left_side != 0 && line[candidate] - upper_value >= CROSS_SKEW_TURN_DIFF &&
            line[candidate] - lower_value >= CROSS_SKEW_TURN_DIFF) return candidate;
    if (left_side == 0 && upper_value - line[candidate] >= CROSS_SKEW_TURN_DIFF &&
            lower_value - line[candidate] >= CROSS_SKEW_TURN_DIFF) return candidate;
    return 0;
}

static int Find_Skew_Down_Point(const int16 *line, const uint8 *lost_flag, int up_point, int down_point, uint8 left_side)
{
    if (Skew_Pair_Is_Valid(line, lost_flag, up_point, down_point, left_side) != 0) return down_point;

    down_point = Find_Skew_Turn_Point(line, lost_flag, up_point, left_side);
    if (Skew_Pair_Is_Valid(line, lost_flag, up_point, down_point, left_side) == 0) return 0;
    return down_point;
}

uint8 Judge_xierushizi_type(void)
{
    int i;
    int start_row = 15;
    int end_row = 55;
    int valid_top = MT9V03X_H - Search_Stop_Line;
    int search_start = MT9V03X_H - 6;
    int bottom_row = MT9V03X_H - 1;
    int bottom_center;
    int normal_left_count = 0;
    int normal_right_count = 0;
    int left_cross_count = 0;
    int right_cross_count = 0;
    int left_up;
    int left_down;
    int right_up;
    int right_down;
    uint8 skew_signal;
    uint8 repair_left;
    uint8 repair_right;

    if (Left_Lost_Flag[bottom_row] == 0 && Right_Lost_Flag[bottom_row] == 0 &&
            Left_Line[bottom_row] < Right_Line[bottom_row]) {
        bottom_center = (Left_Line[bottom_row] + Right_Line[bottom_row]) / 2;
    } else {
        bottom_center = (MT9V03X_W - 1) / 2;
    }

    if (start_row < valid_top) start_row = valid_top;
    if (end_row > MT9V03X_H - 2) end_row = MT9V03X_H - 2;

    for (i = start_row; i < end_row; i++) {
        if (Left_Lost_Flag[i] == 0) {
            if (Left_Line[i] < bottom_center - CROSS_SKEW_MARGIN) normal_left_count++;
            if (Left_Line[i] > bottom_center) left_cross_count++;
        }
        if (Right_Lost_Flag[i] == 0) {
            if (Right_Line[i] > bottom_center + CROSS_SKEW_MARGIN) normal_right_count++;
            if (Right_Line[i] < bottom_center) right_cross_count++;
        }
    }

    left_up = Find_Left_Up_Point(search_start, valid_top);
    left_down = Find_Left_Down_Point(search_start, valid_top);
    right_up = Find_Right_Up_Point(search_start, valid_top);
    right_down = Find_Right_Down_Point(search_start, valid_top);

    /* 两侧上拐点齐全且边界分布均衡时，优先交给原来的正入十字逻辑。 */
    if (normal_left_count >= CROSS_SKEW_COUNT_MIN && normal_right_count >= CROSS_SKEW_COUNT_MIN &&
            abs(normal_left_count - normal_right_count) <= CROSS_SKEW_COUNT_MIN && left_up > 0 && right_up > 0) {
        return CROSS_SKEW_NONE;
    }

    left_down = Find_Skew_Down_Point(Left_Line, Left_Lost_Flag, left_up, left_down, 1);
    right_down = Find_Skew_Down_Point(Right_Line, Right_Lost_Flag, right_up, right_down, 0);
    /* 越线只证明存在斜入特征，最终补哪侧仍完全由本侧有效双锚点决定。 */
    skew_signal = left_cross_count >= CROSS_SKEW_CROSS_MIN || right_cross_count >= CROSS_SKEW_CROSS_MIN;
    repair_left = skew_signal != 0 && left_down > 0 && Left_Line[left_down] < bottom_center - CROSS_SKEW_MARGIN;
    repair_right = skew_signal != 0 && right_down > 0 && Right_Line[right_down] > bottom_center + CROSS_SKEW_MARGIN;

    if (repair_left == repair_right) return CROSS_SKEW_NONE;
    if (repair_right != 0) return CROSS_SKEW_LEFT;
    return CROSS_SKEW_RIGHT;
}

static uint8 Repair_Left_Skew(int skew_up)
{
    int i;
    int search_start = MT9V03X_H - 6;
    int search_end = MT9V03X_H - Search_Stop_Line;
    int skew_down = Find_Right_Down_Point(search_start, search_end);

    skew_down = Find_Skew_Down_Point(Right_Line, Right_Lost_Flag, skew_up, skew_down, 0);
    if (skew_down == 0) return 0;

    Add_Right_Line(skew_up, skew_down);
    for (i = skew_up; i <= skew_down; i++) Right_Lost_Flag[i] = 0;
    return 1;
}

static uint8 Repair_Right_Skew(int skew_up)
{
    int i;
    int search_start = MT9V03X_H - 6;
    int search_end = MT9V03X_H - Search_Stop_Line;
    int skew_down = Find_Left_Down_Point(search_start, search_end);

    skew_down = Find_Skew_Down_Point(Left_Line, Left_Lost_Flag, skew_up, skew_down, 1);
    if (skew_down == 0) return 0;

    Add_Left_Line(skew_up, skew_down);
    for (i = skew_up; i <= skew_down; i++) Left_Lost_Flag[i] = 0;
    return 1;
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

static int Calculate_Max_Preview_Length(void)
{
    int max_length = car_params.lookhead;

    if (max_length < 60) max_length = 60;
    if (max_length > height) max_length = height;
    return max_length;
}

static int Update_Dynamic_Preview_Row(float curvature, uint8 line_valid)
{
    int base_row = car_params.control_row_near;
    int far_row;
    int offset;
    uint32 frame_period_ms;

    if (base_row < 3) base_row = 3;
    if (base_row > height - 3) base_row = height - 3;
    far_row = base_row - DYNAMIC_PREVIEW_ROW_OFFSET;
    if (far_row < 3) far_row = 3;

    if (base_row != last_preview_base_row) {
        last_preview_base_row = base_row;
        straight_preview_time_ms = 0;
        preview_curve_mode = 0;
        curve_preview_hold_frames = 0;
        curve_preview_hold_row = base_row;
        current_preview_row = base_row;
    }

    if (car_params.running == 0 || car_params.base_speed <= DYNAMIC_PREVIEW_SPEED_THRESHOLD) {
        straight_preview_time_ms = 0;
        preview_curve_mode = 0;
        curve_preview_hold_frames = 0;
        curve_preview_hold_row = base_row;
        current_preview_row = base_row;
        return current_preview_row;
    }

    if (line_valid == 0 || curve_variance_valid == 0) {
        straight_preview_time_ms = 0;
        preview_curve_mode = 0;
        curve_preview_hold_frames = 0;
        curve_preview_hold_row = base_row;
        current_preview_row = base_row;
        return current_preview_row;
    }

    /* Hold the entry row for three valid frames before the curve switches back to the near row. */
    if (preview_curve_mode == 0 && curvature >= car_params.curve_enter_threshold) {
        preview_curve_mode = 1;
        straight_preview_time_ms = 0;
        curve_preview_hold_frames = DYNAMIC_PREVIEW_CURVE_HOLD_FRAMES;
        curve_preview_hold_row = current_preview_row;
        if (curve_preview_hold_row < far_row) curve_preview_hold_row = far_row;
        if (curve_preview_hold_row > base_row) curve_preview_hold_row = base_row;
    } else if (preview_curve_mode != 0 && curve_preview_hold_frames == 0 &&
        curvature <= car_params.curve_exit_threshold) {
        preview_curve_mode = 0;
    }

    if (curve_preview_hold_frames > 0) {
        straight_preview_time_ms = 0;
        current_preview_row = curve_preview_hold_row;
        curve_preview_hold_frames--;
        return current_preview_row;
    }

    if (preview_curve_mode != 0) {
        straight_preview_time_ms = 0;
        current_preview_row = base_row;
        return current_preview_row;
    }

    frame_period_ms = car_frame_period_ms;
    if (frame_period_ms == 0 || frame_period_ms > 250) frame_period_ms = 20;
    straight_preview_time_ms += frame_period_ms;
    if (straight_preview_time_ms > DYNAMIC_PREVIEW_TIME_MS) straight_preview_time_ms = DYNAMIC_PREVIEW_TIME_MS;
    offset = straight_preview_time_ms * DYNAMIC_PREVIEW_ROW_OFFSET / DYNAMIC_PREVIEW_TIME_MS;
    current_preview_row = base_row - offset;
    if (current_preview_row < far_row) current_preview_row = far_row;
    return current_preview_row;
}

float Calculate_Error(void)
{
    int i;
    int search_row;
    int pre_sight = current_preview_row;
    int start_row;
    int end_row;
    int valid_top = height - Search_Stop_Line;
    int valid_count = 0;
    int last_center = 0;
    uint8 has_last_center = 0;
    float center_sum = 0;

    if (pre_sight < 3) pre_sight = 3;
    if (pre_sight > height - 3) pre_sight = height - 3;
    if (valid_top < 0) valid_top = 0;
    start_row = pre_sight - 3;
    end_row = pre_sight + 3;

    /* 先找窗口下方最近的有效中线，窗口全部位于黑区时也能向上延长。 */
    for (search_row = end_row - 1; search_row < height; search_row++) {
        if (search_row < valid_top) continue;
        if (Left_Lost_Flag[search_row] != 0 && Right_Lost_Flag[search_row] != 0) continue;
        last_center = Center_Line[search_row];
        has_last_center = 1;
        break;
    }

    if (has_last_center == 0) return 0;

    /* 从近端向远端计算，无效行使用下方最近的有效中线，不修改原中线。 */
    for (i = end_row - 1; i >= start_row; i--) {
        if (i >= valid_top && (Left_Lost_Flag[i] == 0 || Right_Lost_Flag[i] == 0)) last_center = Center_Line[i];
        center_sum += last_center;
        valid_count++;
    }

    if (valid_count == 0) return 0;
    return center_sum / valid_count - MT9V03X_W * 0.5;
}

static float Calculate_Preview_Weight(int row, int top_row, int middle_row, int near_row)
{
    float weight;

    if (row <= middle_row) {
        if (middle_row > top_row) {
            weight = car_params.path_weight_far +
                (car_params.path_weight_middle - car_params.path_weight_far) *
                (row - top_row) / (middle_row - top_row);
        } else {
            weight = car_params.path_weight_near;
        }
    } else if (near_row > middle_row) {
        weight = car_params.path_weight_middle +
            (car_params.path_weight_near - car_params.path_weight_middle) *
            (row - middle_row) / (near_row - middle_row);
    } else {
        weight = car_params.path_weight_near;
    }

    if (weight < 0.0) weight = 0.0;
    return weight;
}

static float Calculate_Curve_Variance(void)
{
    int i;
    int max_length = Calculate_Max_Preview_Length();
    int top_row = height - max_length;
    int near_row = height - 30;
    int middle_row;
    int valid_top;
    int valid_count = 0;
    float weighted_sum = 0;
    float weight_sum = 0;
    float variance_sum = 0;
    float mean;

    curve_variance_valid = 0;
    if (near_row >= height) near_row = height - 1;
    if (near_row < 0) return 0;
    if (top_row < 0) top_row = 0;
    if (top_row > near_row) return 0;
    valid_top = height - Search_Stop_Line;
    if (valid_top < 0) valid_top = 0;
    middle_row = top_row + (near_row - top_row) / 2;

    for (i = top_row; i <= near_row; i += 5) {
        float weight;

        if (i < valid_top) continue;
        if (Left_Lost_Flag[i] != 0 && Right_Lost_Flag[i] != 0) continue;
        weight = Calculate_Preview_Weight(i, top_row, middle_row, near_row);
        if (weight <= 0.0) continue;
        weighted_sum += Center_Line[i] * weight;
        weight_sum += weight;
        valid_count++;
    }

    if (valid_count < 2 || weight_sum <= 0.0) return 0;
    mean = weighted_sum / weight_sum;

    for (i = top_row; i <= near_row; i += 5) {
        float weight;
        float difference;

        if (i < valid_top) continue;
        if (Left_Lost_Flag[i] != 0 && Right_Lost_Flag[i] != 0) continue;
        weight = Calculate_Preview_Weight(i, top_row, middle_row, near_row);
        if (weight <= 0.0) continue;
        difference = Center_Line[i] - mean;
        variance_sum += difference * difference * weight;
    }

    curve_variance_valid = 1;
    return variance_sum / weight_sum;
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

        binary_image[i * MT9V03X_W + Center_Line[i]] = 0;
    }
}

// // Legacy edge-following helpers are disabled until their removed interfaces are restored.
// /*-------------------------------------------------------------------------------------------------------------------
//   @brief     左下角点检�?
//   @param     起�?��?�，终�?��??
//   @return    返回角点所在的行数，找不到返回0
//   Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
//   @note      角点检测阈值可根据实际值更�?
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
        Left_Line[start] = (int16)x1;
        cross_write_binary_point(start, x1);
        return;
    }

    for (i = start; i <= end; i++) {
        value = x1 + (x2 - x1) * (i - start) / (end - start);
        if (value < 0) value = 0;
        if (value >= MT9V03X_W) value = MT9V03X_W - 1;
        Left_Line[i] = (int16)value;
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
        Right_Line[start] = (int16)x1;
        cross_write_binary_point(start, x1);
        return;
    }

    for (i = start; i <= end; i++) {
        value = x1 + (x2 - x1) * (i - start) / (end - start);
        if (value < 0) value = 0;
        if (value >= MT9V03X_W) value = MT9V03X_W - 1;
        Right_Line[i] = (int16)value;
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
        Left_Line[i] = (int16)value;
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
        Right_Line[i] = (int16)value;
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

void shizibuxian(uint8 xieru_type)
{
    int i;
    int left_up;
    int right_up;
    int left_down;
    int right_down;
    int skew_up;
    int search_start;
    int search_end;
    int down_end;
    int bottom_row;
    int center;

    search_start = MT9V03X_H - 6;
    search_end = MT9V03X_H - Search_Stop_Line;
    bottom_row = MT9V03X_H - 1;

    /* 斜入只连接确认侧的上下锚点，另一侧边界保持原样。 */
    if (xieru_type == CROSS_SKEW_LEFT)
    {
        skew_up = Find_Right_Up_Point(search_start, search_end);
        if (skew_up <= 5) return;
        if (Repair_Left_Skew(skew_up) == 0) return;

        Cross_Flag = 1;
        Cross_Count = 3;
        return;
    }

    if (xieru_type == CROSS_SKEW_RIGHT)
    {
        skew_up = Find_Left_Up_Point(search_start, search_end);
        if (skew_up <= 5) return;
        if (Repair_Right_Skew(skew_up) == 0) return;

        Cross_Flag = 1;
        Cross_Count = 3;
        return;
    }

    /*
     * 以下为�?�入十字�?
     */
    left_up = Find_Left_Up_Point(search_start, search_end);
    right_up = Find_Right_Up_Point(search_start, search_end);

    if (left_up <= 0 || right_up <= 0) return;

    /*
     * 用底部道�?�?心确认两�?上拐点确实分布在�?线两侧�?
     * �?3像素容差，避免边界抖动�?�致漏�?��?
     */
    if (Left_Lost_Flag[bottom_row] == 0 &&
        Right_Lost_Flag[bottom_row] == 0 &&
        Left_Line[bottom_row] < Right_Line[bottom_row])
    {
        center = (Left_Line[bottom_row] + Right_Line[bottom_row]) / 2;
    }
    else
    {
        center = (MT9V03X_W - 1) / 2;
    }

    if (Left_Line[left_up] > center + 3) return;
    if (Right_Line[right_up] < center - 3) return;

    /*
     * 正入时左右上拐点不能相差�?远�?
     */
    if (abs(left_up - right_up) >
        car_params.cross_corner_row_gap_max)
    {
        return;
    }

    /*
     * 下拐点必须在上拐点的近车方向，即行号更大�?
     */
    down_end = (left_up > right_up ? left_up : right_up) + 2;

    left_down =
        Find_Left_Down_Point(search_start, down_end);

    right_down =
        Find_Right_Down_Point(search_start, down_end);

    if (left_down <= left_up)
    {
        left_down = 0;
    }

    if (right_down <= right_up)
    {
        right_down = 0;
    }

    /*
     * 四个角点均存�?�?
     * 上下角点直接连线�?
     */
    if (left_down > 0 && right_down > 0)
    {
        Add_Left_Line(left_up, left_down);
        Add_Right_Line(right_up, right_down);
    }
    /*
     * �?有左下�?�点�?
     * 左侧连线，右侧延长�?
     */
    else if (left_down > 0)
    {
        Add_Left_Line(left_up, left_down);

        Lengthen_Right_Boundry(
            right_up - 1,
            MT9V03X_H - 1);
    }
    /*
     * �?有右下�?�点�?
     * 右侧连线，左侧延长�?
     */
    else if (right_down > 0)
    {
        Lengthen_Left_Boundry(
            left_up - 1,
            MT9V03X_H - 1);

        Add_Right_Line(right_up, right_down);
    }
    /*
     * 两个下�?�点都找不到�?
     * 才退化为延长两条上边界�?
     */
    else
    {
        Lengthen_Left_Boundry(
            left_up - 1,
            MT9V03X_H - 1);

        Lengthen_Right_Boundry(
            right_up - 1,
            MT9V03X_H - 1);
    }

    /*
     * 补线后的区域重新标为有效边界�?
     */
    for (i = left_up; i < MT9V03X_H; i++)
    {
        Left_Lost_Flag[i] = 0;
    }

    for (i = right_up; i < MT9V03X_H; i++)
    {
        Right_Lost_Flag[i] = 0;
    }

    Cross_Flag = 1;
    Cross_Count = 3;
}
// void shizibuxian(uint8 xieru_type)
// {
//     int i, left_up, right_up, left_down, right_down, skew_up;
//     int search_start = MT9V03X_H - 6, search_end = MT9V03X_H - Search_Stop_Line, down_end;
//     // int center = (MT9V03X_W - 1) / 2;
//     int bottom_row = MT9V03X_H - 1;
//     int center = (Left_Line[bottom_row] + Right_Line[bottom_row]) / 2;

//     // if (xieru_type == CROSS_SKEW_LEFT) {
//     //     skew_up = Find_xierushizi_up_point(CROSS_SKEW_LEFT);
//     //     if (skew_up <= 5) return;

//     //     Lengthen_Left_Boundry(skew_up - 1, MT9V03X_H - 1);
//     //     for (i = skew_up - 1; i < MT9V03X_H; i++) Left_Lost_Flag[i] = 0;

//     //     Cross_Flag = 1;
//     //     Cross_Count = 3;
//     //     return;
//     // }
//     if (xieru_type == CROSS_SKEW_LEFT) {
//         skew_up = Find_xierushizi_up_point(CROSS_SKEW_LEFT);
//         if (skew_up <= 5) return;

//         Repair_Left_Skew(skew_up);

//         Cross_Flag = 1;
//         Cross_Count = 3;
//         return;
//     }

//     // if (xieru_type == CROSS_SKEW_RIGHT) {
//     //     skew_up = Find_xierushizi_up_point(CROSS_SKEW_RIGHT);
//     //     if (skew_up <= 5) return;

//     //     Lengthen_Right_Boundry(skew_up - 1, MT9V03X_H - 1);
//     //     for (i = skew_up - 1; i < MT9V03X_H; i++) Right_Lost_Flag[i] = 0;

//     //     Cross_Flag = 1;
//     //     Cross_Count = 3;
//     //     return;
//     // }
//     if (xieru_type == CROSS_SKEW_RIGHT) {
//         skew_up = Find_xierushizi_up_point(CROSS_SKEW_RIGHT);
//         if (skew_up <= 5) return;

//         Repair_Right_Skew(skew_up);

//         Cross_Flag = 1;
//         Cross_Count = 3;
//         return;
//     }

//     left_up = Find_Left_Up_Point(search_start, search_end);
//     right_up = Find_Right_Up_Point(search_start, search_end);

//     if (left_up <= 0 || right_up <= 0) return;
//     // if (Left_Line[left_up] >= center || Right_Line[right_up] <= center) return;
//     if (Left_Line[left_up] > center || Right_Line[right_up] < center) return;
//     if (abs(left_up - right_up) >= car_params.cross_corner_row_gap_max) return;

//     down_end = (left_up > right_up ? left_up : right_up) + 2;
//     left_down = Find_Left_Down_Point(search_start, down_end);
//     right_down = Find_Right_Down_Point(search_start, down_end);

//     if (left_down <= left_up) left_down = 0;
//     if (right_down <= right_up) right_down = 0;

//     Lengthen_Left_Boundry(left_up - 1, MT9V03X_H - 1);
//     Lengthen_Right_Boundry(right_up - 1, MT9V03X_H - 1);

//     for (i = left_up - 1; i < MT9V03X_H; i++) Left_Lost_Flag[i] = 0;
//     for (i = right_up - 1; i < MT9V03X_H; i++) Right_Lost_Flag[i] = 0;

//     Cross_Flag = 1;
//     Cross_Count = 3;
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
    int16 GrayCur;
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
            os_threshold = (int16)j;
        }
    }
    return white_cnt < PixelSum * 0.1;
}

void threshold_update(void)
{
    int i, j;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            if (binary_image[i * width + j] > os_threshold) {
                binary_image[i * width + j] = 1;
            } else {
                binary_image[i * width + j] = 0;
            }
        }
    }
}

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
                // 白黑黑跳�?
                if(binary_image[y*width+x] == 1 &&
                   binary_image[(y-1)*width+x] == 0 &&
                   binary_image[(y-2)*width+x] == 0)
                {
                    // 找最长白�?
                    if(y < lw)
                    {
                        lw = y;
                        lwline = x;
                    }

                    break;
                }


                // 防�?�越�?
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

float Last_Curvature_Value = 0;     //防�??
/**
* 函数功能�?      计算�?线曲率，作为控制速度的一�?数据
* 特殊说明�?      �?
* �?  参：        uint8 *line      �?�?
*                 uint8 start      �?线相遇点的Y值，或最大有效�??
*                 uint8 end        �?线起始�?�（或图像最底�??�?
*
* 示例�?          Calculate_Curvature_2(C_Line, Y_Meet, L_Start_Point[1]);
* 返回值：        Last_Curvature_Value      所得曲�?
*/
// float Calculate_Curvature_2(uint8 *line, uint8 start, uint8 end)
// {
//     int16 Sum_Difference_Value = 0;     //用于X坐标�?值的�?�?
//     int16 Sum_Weight = 0;               //用于权值相�?
//     uint8 i = 0;
 
//     // uint8 Base_Curvature_Weight[60] = {0, 0, 1, 0, 1, 0, 1, 0, 1, 0,
//     //                                    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
//     //                                    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
//     //                                    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
//     //                                    1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
//     //                                    1, 0, 1, 0, 1, 0, 1, 0, 1, 0};       //固定权值，0,1,0,1间隔加权
//     uint8 Base_Curvature_Weight[120];
//     for (i = 0; i < 120; ++i) {
//         Base_Curvature_Weight[i] = i&1;
//     }
//     uint8 Trends_Curvature_Weight[19] = {4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 7, 7, 6, 6, 5, 5, 4};      //动态权值，高权值放�?�?
 
//     uint8 Start_Line = (uint8)((float)(Y_Meet - 2) / 18.0f * 16.0f + 24.0f);    //动态权替换固定权起始�?�，我图像为60行，最低从24行向上替�?，最高为40行向上替�?，Y_Meet最大值为20
 
//     //将动态权值赋值给固定权值数�?
//     for(i = 0; i < 19; i ++) {
//         Base_Curvature_Weight[Start_Line - i] = Trends_Curvature_Weight[i];
//     }
 
//     for(i = start; i < end - 1; i++)    //求差值和并求出权�?
//     {
//         Sum_Difference_Value += (int16)(My_ABS((int)line[i] - (int)line[i + 1]) * (int)Base_Curvature_Weight[i]);
//         Sum_Weight += (int16)Base_Curvature_Weight[i];
//     }
 
//     if(Sum_Weight != 0)     //防�?�权值为0，但好像不会出现，当时不知道为啥要写这个
//     {
//         Last_Curvature_Value = (float)Sum_Difference_Value / (float)Sum_Weight;     //求加权平均值，归一化放在了另一�?函数�?
//         return Last_Curvature_Value;
//     }
//     else
//     {
//         return Last_Curvature_Value;
//     }
// }
