#ifndef CC_TRACK_STATE_H
#define CC_TRACK_STATE_H

#include <stdint.h>

typedef enum {
    CC_TRACK_STATE_NORMAL = 0,
    CC_TRACK_STATE_LEFT_EDGE_LOST,
    CC_TRACK_STATE_RIGHT_EDGE_LOST,
    CC_TRACK_STATE_CROSS,
    CC_TRACK_STATE_ROUNDABOUT_LEFT,
    CC_TRACK_STATE_ROUNDABOUT_RIGHT,
    CC_TRACK_STATE_ZEBRA,
    CC_TRACK_STATE_LINE_LOST
} cc_track_state_t;

typedef struct {
    uint16_t row_count;
    uint16_t left_valid_rows;
    uint16_t right_valid_rows;
    uint16_t both_valid_rows;
    uint16_t stripe_transitions;
    float far_width;
    float middle_width;
    float near_width;
    float maximum_width;
} cc_track_features_t;

typedef struct {
    float line_lost_valid_ratio;
    float single_edge_valid_ratio;
    float cross_width_growth;
    float roundabout_width_growth;
    uint16_t zebra_min_transitions;
    uint8_t confirm_frames;
    uint8_t line_lost_confirm_frames;
} cc_track_state_config_t;

typedef struct {
    cc_track_state_config_t config;
    cc_track_state_t state;
    cc_track_state_t candidate;
    uint8_t candidate_frames;
} cc_track_state_detector_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          从左右边界提取赛道状态识别特征
// 返回类型          cc_track_features_t  提取出的统计特征
// 参数说明          left                左边界数组
// 参数说明          right               右边界数组
// 参数说明          row_count           数组行数
// 参数说明          image_width         图像宽度
// 参数说明          invalid_value       无效边界标记
// 参数说明          stripe_transitions  横向采样线上黑白跳变次数，用于斑马线识别
// 使用示例          f = cc_track_features_from_edges(left, right, h, w, -1, transitions);
// 备注信息          数组第 0 行按远端处理，最后一行按近端处理。
//-------------------------------------------------------------------------------------------------------------------
cc_track_features_t cc_track_features_from_edges(const int16_t *left, const int16_t *right,
                                                 uint16_t row_count, uint16_t image_width,
                                                 int16_t invalid_value,
                                                 uint16_t stripe_transitions);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          生成赛道状态机默认配置
// 返回类型          void
// 参数说明          config              要写入的配置结构体
// 使用示例          cc_track_state_config_default(&config);
// 备注信息          阈值均为比例或宽度增长倍数，不依赖固定图像分辨率。
//-------------------------------------------------------------------------------------------------------------------
void cc_track_state_config_default(cc_track_state_config_t *config);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          初始化赛道状态机
// 返回类型          void
// 参数说明          detector            状态机对象
// 参数说明          config              配置参数，传空指针时使用默认配置
// 使用示例          cc_track_state_init(&detector, &config);
// 备注信息          初始状态为普通赛道，所有候选状态都需连续帧确认后才能生效。
//-------------------------------------------------------------------------------------------------------------------
void cc_track_state_init(cc_track_state_detector_t *detector,
                         const cc_track_state_config_t *config);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          使用当前帧特征更新赛道状态机
// 返回类型          cc_track_state_t     去抖后的稳定状态
// 参数说明          detector            状态机对象
// 参数说明          features            当前帧统计特征
// 使用示例          state = cc_track_state_update(&detector, &features);
// 备注信息          转移顺序为丢线、斑马线、十字、环岛/单边丢失、普通赛道，避免特征冲突。
//-------------------------------------------------------------------------------------------------------------------
cc_track_state_t cc_track_state_update(cc_track_state_detector_t *detector,
                                       const cc_track_features_t *features);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          将赛道状态转换为便于调试显示的字符串
// 返回类型          const char *         静态字符串地址
// 参数说明          state               赛道状态
// 使用示例          debug_printf("%s", cc_track_state_name(state));
// 备注信息          返回值不需要释放。
//-------------------------------------------------------------------------------------------------------------------
const char *cc_track_state_name(cc_track_state_t state);

#endif
