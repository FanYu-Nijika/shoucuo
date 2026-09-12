//#ifndef CC_LINE_CAMERA_H
//#define CC_LINE_CAMERA_H
//
//#include <stdint.h>
//
//#include "image.h"
//
//typedef struct {
//    uint8_t threshold;
//    uint8_t dark_is_line;
//    uint8_t minimum_line_pixels;
//    uint8_t row_step;
//    uint16_t roi_top;
//    uint16_t roi_bottom;
//    int16_t center_offset_pixels;
//    uint8_t automatic_threshold;
//    uint8_t otsu_row_step;
//    uint8_t otsu_column_step;
//} cc_line_camera_config_t;
//
//typedef struct {
//    uint8_t valid;
//    uint16_t valid_rows;
//    uint16_t lost_count;
//    uint16_t left_edge;
//    uint16_t right_edge;
//    uint16_t line_width;
//    float center_x;
//    float error_pixels;
//    float error_normalized;
//    float heading_error;
//    float strength;
//    uint8_t threshold_used;
//} cc_line_result_t;
//
//typedef struct {
//    cc_line_camera_config_t config;
//    cc_line_result_t last_result;
//    uint16_t lost_count;
//} cc_line_camera_t;
//
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介          cc_line_camera_init 函数声明
//// 返回类型          void
//// 使用示例          cc_line_camera_init(...);
//// 备注信息          参数含义请参考同名源文件实现。
////-------------------------------------------------------------------------------------------------------------------
//void cc_line_camera_init(cc_line_camera_t *camera, const cc_line_camera_config_t *config);
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介          cc_line_camera_update 函数声明
//// 返回类型          cc_line_result_t
//// 使用示例          cc_line_camera_update(...);
//// 备注信息          参数含义请参考同名源文件实现。
////-------------------------------------------------------------------------------------------------------------------
//cc_line_result_t cc_line_camera_update(cc_line_camera_t *camera, const cc_image_u8_t *image);
//
//#endif
