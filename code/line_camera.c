//#include "line_camera.h"
//
//#include "math_utils.h"
//
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介          cc_line_camera_init 功能实现
//// 返回类型          void
//// 使用示例          cc_line_camera_init(...);
//// 备注信息          参数含义请参考同名头文件声明。
////-------------------------------------------------------------------------------------------------------------------
//void cc_line_camera_init(cc_line_camera_t *camera, const cc_line_camera_config_t *config)
//{
//    if (camera == 0) return;
//    camera->config.threshold = 100;
//    camera->config.dark_is_line = 1;
//    camera->config.minimum_line_pixels = 2;
//    camera->config.row_step = 2;
//    camera->config.roi_top = 0;
//    camera->config.roi_bottom = 0;
//    camera->config.center_offset_pixels = 0;
//    camera->config.automatic_threshold = 0;
//    camera->config.otsu_row_step = 2;
//    camera->config.otsu_column_step = 2;
//    if (config != 0) camera->config = *config;
//    if (camera->config.minimum_line_pixels == 0) camera->config.minimum_line_pixels = 1;
//    if (camera->config.row_step == 0) camera->config.row_step = 1;
//    camera->lost_count = 0;
//    camera->last_result.valid = 0;
//    camera->last_result.valid_rows = 0;
//    camera->last_result.lost_count = 0;
//    camera->last_result.left_edge = 0;
//    camera->last_result.right_edge = 0;
//    camera->last_result.line_width = 0;
//    camera->last_result.center_x = 0.0;
//    camera->last_result.error_pixels = 0.0;
//    camera->last_result.error_normalized = 0.0;
//    camera->last_result.heading_error = 0.0;
//    camera->last_result.strength = 0.0;
//    camera->last_result.threshold_used = camera->config.threshold;
//}
//
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介          cc_line_camera_update 功能实现
//// 返回类型          cc_line_result_t
//// 使用示例          cc_line_camera_update(...);
//// 备注信息          参数含义请参考同名头文件声明。
////-------------------------------------------------------------------------------------------------------------------
//cc_line_result_t cc_line_camera_update(cc_line_camera_t *camera, const cc_image_u8_t *image)
//{
//    cc_line_result_t result;
//    uint16_t top;
//    uint16_t bottom;
//    uint32_t row;
//    uint16_t left;
//    uint16_t right;
//    uint16_t pixels;
//    uint16_t rows = 0;
//    uint16_t valid_rows = 0;
//    uint64_t weighted_center = 0;
//    uint64_t weight = 0;
//    uint64_t far_center_sum = 0;
//    uint64_t far_weight = 0;
//    uint64_t near_center_sum = 0;
//    uint64_t near_weight = 0;
//    uint16_t overall_left = 0;
//    uint16_t overall_right = 0;
//    uint8_t first_valid = 1;
//    float image_center;
//    float half_width;
//    uint8_t threshold;
//
//    if (camera == 0) {
//        result.valid = 0;
//        result.valid_rows = 0;
//        result.lost_count = 0;
//        result.left_edge = 0;
//        result.right_edge = 0;
//        result.line_width = 0;
//        result.center_x = 0.0;
//        result.error_pixels = 0.0;
//        result.error_normalized = 0.0;
//        result.heading_error = 0.0;
//        result.strength = 0.0;
//        result.threshold_used = 0;
//        return result;
//    }
//    result = camera->last_result;
//    result.valid = 0;
//    result.valid_rows = 0;
//    result.strength = 0.0;
//    if (!cc_image_u8_is_valid(image)) return result;
//
//    top = camera->config.roi_top < image->height ? camera->config.roi_top : 0;
//    bottom = camera->config.roi_bottom == 0 || camera->config.roi_bottom > image->height ? image->height : camera->config.roi_bottom;
//    if (top >= bottom) { top = 0; bottom = image->height; }
//    threshold = camera->config.threshold;
//    if (camera->config.automatic_threshold != 0) {
//        threshold = cc_image_u8_otsu_threshold(image, top, bottom,
//                                              camera->config.otsu_row_step,
//                                              camera->config.otsu_column_step,
//                                              threshold);
//    }
//    result.threshold_used = threshold;
//
//    for (row = top; row < bottom; row += camera->config.row_step) {
//        ++rows;
//        if (!cc_image_u8_row_span(image, (uint16_t)row, threshold, camera->config.dark_is_line, &left, &right, &pixels)) continue;
//        if (pixels < camera->config.minimum_line_pixels) continue;
//        weighted_center += (uint64_t)(left + right) * pixels;
//        weight += (uint64_t)2 * pixels;
//        if (row < ((uint32_t)top + bottom) / 2) {
//            far_center_sum += (uint64_t)(left + right) * pixels;
//            far_weight += (uint64_t)2 * pixels;
//        } else {
//            near_center_sum += (uint64_t)(left + right) * pixels;
//            near_weight += (uint64_t)2 * pixels;
//        }
//        if (first_valid != 0) { overall_left = left; overall_right = right; first_valid = 0; }
//        if (left < overall_left) overall_left = left;
//        if (right > overall_right) overall_right = right;
//        ++valid_rows;
//    }
//
//    if (valid_rows == 0 || weight == 0) {
//        ++camera->lost_count;
//        result.lost_count = camera->lost_count;
//        result.error_pixels = camera->last_result.error_pixels;
//        result.error_normalized = camera->last_result.error_normalized;
//        camera->last_result = result;
//        return result;
//    }
//
//    image_center = (float)(image->width - 1) * 0.5 + (float)camera->config.center_offset_pixels;
//    half_width = (float)image->width * 0.5;
//    result.valid = 1;
//    result.valid_rows = valid_rows;
//    result.lost_count = 0;
//    result.left_edge = overall_left;
//    result.right_edge = overall_right;
//    result.line_width = (uint16_t)(overall_right - overall_left + 1);
//    result.center_x = (float)weighted_center / (float)weight;
//    result.error_pixels = result.center_x - image_center;
//    result.error_normalized = cc_math_clamp_f32(result.error_pixels / half_width, -1.0, 1.0);
//    result.heading_error = 0.0;
//    if (far_weight != 0 && near_weight != 0) {
//        float far_center = (float)far_center_sum / (float)far_weight;
//        float near_center = (float)near_center_sum / (float)near_weight;
//        result.heading_error = cc_math_clamp_f32((near_center - far_center) / half_width,
//                                                -1.0, 1.0);
//    }
//    result.strength = (float)valid_rows / (float)rows;
//    camera->lost_count = 0;
//    camera->last_result = result;
//    return result;
//}
