#include "image.h"
#include "car_shared.h"
#include "longest_white.h"
#include "car_params.h"
#include "algorithm.h"
#include "math_utils.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"
#include <string.h>

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
static int current_preview_length = 60;
static int last_preview_max_length = 60;
static float filtered_preview_variance = 0;
static uint8 curve_variance_valid = 0;

uint8 ostu_deal_threshold(void);
void threshold_update(void);
float Calculate_Error(void);
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
    current_preview_length = 60;
    last_preview_max_length = 60;
    filtered_preview_variance = 0;
    curve_variance_valid = 0;
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
    // Longest_White_Column();
    Longest_White_Column();
    Build_Dynamic_Center_Line();
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
    result->curve_variance = Calculate_Curve_Variance();
    if (car_params.curvature_scale > 0.0)
        result->curvature = result->curve_variance / car_params.curvature_scale;
    if (result->curvature < 0.0) result->curvature = 0.0;
    if (result->curvature > 1.0) result->curvature = 1.0;
    if (curve_variance_valid != 0) {
        filtered_preview_variance += 0.5 * (result->curvature - filtered_preview_variance);
        if (filtered_preview_variance < 0.0) filtered_preview_variance = 0.0;
        if (filtered_preview_variance > 1.0) filtered_preview_variance = 1.0;
    }
    result->preview_length = current_preview_length;
    result->center_x = image_center + result->error_pixels;
    result->error_normalized = result->error_pixels / (width * 0.5);
    result->near_error_cm = result->error_pixels * 40.0 / (result->line_width > 0 ? result->line_width : 1);
    if (already_line_lost != 0 || Left_Lost_Flag[row] != 0 || Right_Lost_Flag[row] != 0 ||
        Longest_White_Column_Left[0] < car_params.minimum_line_pixels) return;

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
//      * 从底部向上逐行扫描。
//      * 每一行都以 max_x 为中心，分别寻找左右边界。
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
uint8 Judge_xierushizi_type(void)
{
    int i, left_cnt = 0, right_cnt = 0, start_row = 15, end_row = 55, valid_top = MT9V03X_H - Search_Stop_Line;

    if (start_row < valid_top) start_row = valid_top;
    if (end_row > MT9V03X_H - 2) end_row = MT9V03X_H - 2;

    for (i = start_row; i < end_row; i++) {
        if (Left_Lost_Flag[i] == 0 &&
            Left_Line[i] > Dynamic_Center_Line[i] + CROSS_SKEW_MARGIN) {
            left_cnt++;
        }

        if (Right_Lost_Flag[i] == 0 &&
            Right_Line[i] < Dynamic_Center_Line[i] - CROSS_SKEW_MARGIN) {
            right_cnt++;
        }
    }

    if (left_cnt >= CROSS_SKEW_COUNT_MIN && right_cnt < CROSS_SKEW_COUNT_MIN) {
        if (Find_xierushizi_up_point(CROSS_SKEW_LEFT) > 5) return CROSS_SKEW_LEFT;
    }

    if (right_cnt >= CROSS_SKEW_COUNT_MIN && left_cnt < CROSS_SKEW_COUNT_MIN) {
        if (Find_xierushizi_up_point(CROSS_SKEW_RIGHT) > 5) return CROSS_SKEW_RIGHT;
    }

    return CROSS_SKEW_NONE;
}

static void Repair_Left_Skew(int skew_up)
{
    int i;
    int left;
    int right;
    int start_row = MT9V03X_H - Search_Stop_Line;

    if (start_row < 5) start_row = 5;

    for (i = start_row; i < MT9V03X_H; i++) {
        if (i <= skew_up && Left_Lost_Flag[i] == 0 &&
            Left_Line[i] > Dynamic_Center_Line[i] + CROSS_SKEW_MARGIN) {
            right = Left_Line[i];
        } else {
            right = Right_Line[i];
        }

        if (right <= Dynamic_Center_Line[i]) right = Dynamic_Center_Line[i] + 1;
        if (right >= MT9V03X_W) right = MT9V03X_W - 1;

        left = 2 * Dynamic_Center_Line[i] - right;

        if (left < 0) left = 0;
        if (left >= Dynamic_Center_Line[i]) left = Dynamic_Center_Line[i] - 1;

        Left_Line[i] = left;
        Right_Line[i] = right;
        Left_Lost_Flag[i] = 0;
        Right_Lost_Flag[i] = 0;
    }
}
static void Repair_Right_Skew(int skew_up)
{
    int i;
    int left;
    int right;
    int start_row = MT9V03X_H - Search_Stop_Line;

    if (start_row < 5) start_row = 5;

    for (i = start_row; i < MT9V03X_H; i++) {
        if (i <= skew_up && Right_Lost_Flag[i] == 0 &&
            Right_Line[i] < Dynamic_Center_Line[i] - CROSS_SKEW_MARGIN) {
            left = Right_Line[i];
        } else {
            left = Left_Line[i];
        }

        if (left >= Dynamic_Center_Line[i]) left = Dynamic_Center_Line[i] - 1;
        if (left < 0) left = 0;

        right = 2 * Dynamic_Center_Line[i] - left;

        if (right >= MT9V03X_W) right = MT9V03X_W - 1;
        if (right <= Dynamic_Center_Line[i]) right = Dynamic_Center_Line[i] + 1;

        Left_Line[i] = left;
        Right_Line[i] = right;
        Left_Lost_Flag[i] = 0;
        Right_Lost_Flag[i] = 0;
    }
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

static int Calculate_Used_Preview_Length(int max_length)
{
    int min_length = max_length - 30;
    int used_length;
    float factor;
    float enter_threshold = car_params.curve_enter_threshold;
    float exit_threshold = car_params.curve_exit_threshold;

    if (min_length < 45) min_length = 45;
    if (min_length > max_length) min_length = max_length;
    if (enter_threshold < exit_threshold) enter_threshold = exit_threshold;

    if (enter_threshold > exit_threshold) {
        factor = (filtered_preview_variance - exit_threshold) / (enter_threshold - exit_threshold);
    } else if (filtered_preview_variance >= enter_threshold) {
        factor = 1.0;
    } else {
        factor = 0.0;
    }
    if (factor < 0.0) factor = 0.0;
    if (factor > 1.0) factor = 1.0;

    used_length = (int)(max_length - (max_length - min_length) * factor);
    used_length = used_length / 5 * 5;
    if (used_length < min_length) used_length = min_length;
    if (used_length > max_length) used_length = max_length;
    return used_length;
}

float Calculate_Error(void)
{
    int i;
    int max_length = Calculate_Max_Preview_Length();
    int top_row;
    int near_row = height - 30;
    int middle_row;
    int valid_top;
    float weighted_sum = 0;
    float weight_sum = 0;

    if (max_length != last_preview_max_length) {
        last_preview_max_length = max_length;
        filtered_preview_variance = 0;
    }
    current_preview_length = Calculate_Used_Preview_Length(max_length);
    top_row = height - current_preview_length;
    if (near_row >= height) near_row = height - 1;
    if (near_row < 0 || top_row < 0 || top_row > near_row) return 0;
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
    }

    if (weight_sum <= 0.0) return 0;
    return weighted_sum / weight_sum - MT9V03X_W * 0.5;
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

        if (xieru_type != CROSS_SKEW_NONE) {
            Center_Line[i] = Dynamic_Center_Line[i];
        } else {
            Center_Line[i] = (Left_Line[i] + Right_Line[i]) / 2;
        }

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

    /*
     * 斜入十字仍使用你现在的逻辑。
     */
    if (xieru_type == CROSS_SKEW_LEFT)
    {
        skew_up = Find_xierushizi_up_point(CROSS_SKEW_LEFT);
        if (skew_up <= 5) return;

        Repair_Left_Skew(skew_up);

        Cross_Flag = 1;
        Cross_Count = 3;
        return;
    }

    if (xieru_type == CROSS_SKEW_RIGHT)
    {
        skew_up = Find_xierushizi_up_point(CROSS_SKEW_RIGHT);
        if (skew_up <= 5) return;

        Repair_Right_Skew(skew_up);

        Cross_Flag = 1;
        Cross_Count = 3;
        return;
    }

    /*
     * 以下为正入十字。
     */
    left_up = Find_Left_Up_Point(search_start, search_end);
    right_up = Find_Right_Up_Point(search_start, search_end);

    if (left_up <= 0 || right_up <= 0) return;

    /*
     * 用底部道路中心确认两个上拐点确实分布在中线两侧。
     * 加3像素容差，避免边界抖动导致漏检。
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
     * 正入时左右上拐点不能相差太远。
     */
    if (abs(left_up - right_up) >
        car_params.cross_corner_row_gap_max)
    {
        return;
    }

    /*
     * 下拐点必须在上拐点的近车方向，即行号更大。
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
     * 四个角点均存在：
     * 上下角点直接连线。
     */
    if (left_down > 0 && right_down > 0)
    {
        Add_Left_Line(left_up, left_down);
        Add_Right_Line(right_up, right_down);
    }
    /*
     * 只有左下角点：
     * 左侧连线，右侧延长。
     */
    else if (left_down > 0)
    {
        Add_Left_Line(left_up, left_down);

        Lengthen_Right_Boundry(
            right_up - 1,
            MT9V03X_H - 1);
    }
    /*
     * 只有右下角点：
     * 右侧连线，左侧延长。
     */
    else if (right_down > 0)
    {
        Lengthen_Left_Boundry(
            left_up - 1,
            MT9V03X_H - 1);

        Add_Right_Line(right_up, right_down);
    }
    /*
     * 两个下角点都找不到：
     * 才退化为延长两条上边界。
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
     * 补线后的区域重新标为有效边界。
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

float Last_Curvature_Value = 0;     //防止
/**
* 函数功能：      计算中线曲率，作为控制速度的一个数据
* 特殊说明：      无
* 形  参：        uint8 *line      中线
*                 uint8 start      爬线相遇点的Y值，或最大有效行
*                 uint8 end        爬线起始行（或图像最底端）
*
* 示例：          Calculate_Curvature_2(C_Line, Y_Meet, L_Start_Point[1]);
* 返回值：        Last_Curvature_Value      所得曲率
*/
// float Calculate_Curvature_2(uint8 *line, uint8 start, uint8 end)
// {
//     int16 Sum_Difference_Value = 0;     //用于X坐标差值的累积
//     int16 Sum_Weight = 0;               //用于权值相加
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
//     uint8 Trends_Curvature_Weight[19] = {4, 5, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 7, 7, 6, 6, 5, 5, 4};      //动态权值，高权值放中间
 
//     uint8 Start_Line = (uint8)((float)(Y_Meet - 2) / 18.0f * 16.0f + 24.0f);    //动态权替换固定权起始行，我图像为60行，最低从24行向上替换，最高为40行向上替换，Y_Meet最大值为20
 
//     //将动态权值赋值给固定权值数组
//     for(i = 0; i < 19; i ++) {
//         Base_Curvature_Weight[Start_Line - i] = Trends_Curvature_Weight[i];
//     }
 
//     for(i = start; i < end - 1; i++)    //求差值和并求出权值
//     {
//         Sum_Difference_Value += (int16)(My_ABS((int)line[i] - (int)line[i + 1]) * (int)Base_Curvature_Weight[i]);
//         Sum_Weight += (int16)Base_Curvature_Weight[i];
//     }
 
//     if(Sum_Weight != 0)     //防止权值为0，但好像不会出现，当时不知道为啥要写这个
//     {
//         Last_Curvature_Value = (float)Sum_Difference_Value / (float)Sum_Weight;     //求加权平均值，归一化放在了另一个函数里
//         return Last_Curvature_Value;
//     }
//     else
//     {
//         return Last_Curvature_Value;
//     }
// }
