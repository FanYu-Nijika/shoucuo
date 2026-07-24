#include "track_state.h"

static uint8_t cc_track_state_edge_valid(int16_t edge, uint16_t image_width, int16_t invalid_value)
{
    if (edge == invalid_value) return 0;
    return edge >= 0 && (uint16_t)edge < image_width;
}

static float cc_track_state_ratio(uint16_t value, uint16_t total)
{
    return total == 0 ? 0.0f : (float)value / (float)total;
}

static float cc_track_state_growth(float width, float base_width)
{
    return base_width > 1.0f ? width / base_width : 1.0f;
}

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
// 备注信息          使用分区平均而不是单行宽度，降低噪点对十字和环岛判断的影响。
//-------------------------------------------------------------------------------------------------------------------
cc_track_features_t cc_track_features_from_edges(const int16_t *left, const int16_t *right,
                                                 uint16_t row_count, uint16_t image_width,
                                                 int16_t invalid_value,
                                                 uint16_t stripe_transitions)
{
    cc_track_features_t features = {0, 0, 0, 0, 0, 0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t width_sum[3] = {0, 0, 0};
    uint16_t width_count[3] = {0, 0, 0};
    uint16_t row;
    features.row_count = row_count;
    features.stripe_transitions = stripe_transitions;
    if (left == 0 || right == 0 || row_count == 0 || image_width < 2) return features;

    for (row = 0; row < row_count; ++row) {
        uint8_t left_valid = cc_track_state_edge_valid(left[row], image_width, invalid_value);
        uint8_t right_valid = cc_track_state_edge_valid(right[row], image_width, invalid_value);
        uint8_t region = (uint8_t)(((uint32_t)row * 3u) / row_count);
        uint16_t width;
        if (region > 2) region = 2;
        if (left_valid) ++features.left_valid_rows;
        if (right_valid) ++features.right_valid_rows;
        if (!left_valid || !right_valid || left[row] >= right[row]) continue;
        ++features.both_valid_rows;
        width = (uint16_t)(right[row] - left[row]);
        width_sum[region] += width;
        ++width_count[region];
        if ((float)width > features.maximum_width) features.maximum_width = (float)width;
    }
    if (width_count[0] != 0) features.far_width = (float)width_sum[0] / width_count[0];
    if (width_count[1] != 0) features.middle_width = (float)width_sum[1] / width_count[1];
    if (width_count[2] != 0) features.near_width = (float)width_sum[2] / width_count[2];
    return features;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          生成赛道状态机默认配置
// 返回类型          void
// 参数说明          config              要写入的配置结构体
// 使用示例          cc_track_state_config_default(&config);
// 备注信息          默认要求连续 3 帧确认，丢线只需 2 帧以便更快进入保护状态。
//-------------------------------------------------------------------------------------------------------------------
void cc_track_state_config_default(cc_track_state_config_t *config)
{
    if (config == 0) return;
    config->line_lost_valid_ratio = 0.15f;
    config->single_edge_valid_ratio = 0.40f;
    config->cross_width_growth = 1.55f;
    config->roundabout_width_growth = 1.25f;
    config->zebra_min_transitions = 8;
    config->confirm_frames = 3;
    config->line_lost_confirm_frames = 2;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          初始化赛道状态机
// 返回类型          void
// 参数说明          detector            状态机对象
// 参数说明          config              配置参数，传空指针时使用默认配置
// 使用示例          cc_track_state_init(&detector, &config);
// 备注信息          状态转移采用候选状态计数，单帧噪声不会直接改变车辆策略。
//-------------------------------------------------------------------------------------------------------------------
void cc_track_state_init(cc_track_state_detector_t *detector,
                         const cc_track_state_config_t *config)
{
    cc_track_state_config_t default_config;
    if (detector == 0) return;
    cc_track_state_config_default(&default_config);
    detector->config = config == 0 ? default_config : *config;
    if (detector->config.confirm_frames == 0) detector->config.confirm_frames = 1;
    if (detector->config.line_lost_confirm_frames == 0) detector->config.line_lost_confirm_frames = 1;
    detector->state = CC_TRACK_STATE_NORMAL;
    detector->candidate = CC_TRACK_STATE_NORMAL;
    detector->candidate_frames = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          使用当前帧特征更新赛道状态机
// 返回类型          cc_track_state_t     去抖后的稳定状态
// 参数说明          detector            状态机对象
// 参数说明          features            当前帧统计特征
// 使用示例          state = cc_track_state_update(&detector, &features);
// 备注信息          环岛先表现为单边长期丢失并伴随宽度扩张；仅单边丢失时不会过早判成环岛。
//-------------------------------------------------------------------------------------------------------------------
cc_track_state_t cc_track_state_update(cc_track_state_detector_t *detector,
                                       const cc_track_features_t *features)
{
    cc_track_state_t candidate = CC_TRACK_STATE_NORMAL;
    float left_ratio;
    float right_ratio;
    float both_ratio;
    float far_growth;
    float maximum_growth;
    uint8_t required_frames;

    if (detector == 0 || features == 0 || features->row_count == 0) return CC_TRACK_STATE_LINE_LOST;
    left_ratio = cc_track_state_ratio(features->left_valid_rows, features->row_count);
    right_ratio = cc_track_state_ratio(features->right_valid_rows, features->row_count);
    both_ratio = cc_track_state_ratio(features->both_valid_rows, features->row_count);
    far_growth = cc_track_state_growth(features->far_width, features->near_width);
    maximum_growth = cc_track_state_growth(features->maximum_width, features->near_width);

    if (left_ratio < detector->config.line_lost_valid_ratio &&
        right_ratio < detector->config.line_lost_valid_ratio) {
        candidate = CC_TRACK_STATE_LINE_LOST;
    } else if (features->stripe_transitions >= detector->config.zebra_min_transitions) {
        candidate = CC_TRACK_STATE_ZEBRA;
    } else if (both_ratio >= detector->config.single_edge_valid_ratio &&
               far_growth >= detector->config.cross_width_growth) {
        candidate = CC_TRACK_STATE_CROSS;
    } else if (left_ratio < detector->config.single_edge_valid_ratio &&
               right_ratio >= detector->config.single_edge_valid_ratio) {
        candidate = maximum_growth >= detector->config.roundabout_width_growth
                    ? CC_TRACK_STATE_ROUNDABOUT_LEFT
                    : CC_TRACK_STATE_LEFT_EDGE_LOST;
    } else if (right_ratio < detector->config.single_edge_valid_ratio &&
               left_ratio >= detector->config.single_edge_valid_ratio) {
        candidate = maximum_growth >= detector->config.roundabout_width_growth
                    ? CC_TRACK_STATE_ROUNDABOUT_RIGHT
                    : CC_TRACK_STATE_RIGHT_EDGE_LOST;
    }

    if (candidate != detector->candidate) {
        detector->candidate = candidate;
        detector->candidate_frames = 1;
    } else if (detector->candidate_frames < 255) {
        ++detector->candidate_frames;
    }
    required_frames = candidate == CC_TRACK_STATE_LINE_LOST
                      ? detector->config.line_lost_confirm_frames
                      : detector->config.confirm_frames;
    if (detector->candidate_frames >= required_frames) detector->state = candidate;
    return detector->state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          将赛道状态转换为便于调试显示的字符串
// 返回类型          const char *         静态字符串地址
// 参数说明          state               赛道状态
// 使用示例          debug_printf("%s", cc_track_state_name(state));
// 备注信息          未知枚举返回 UNKNOWN，避免调试输出访问无效地址。
//-------------------------------------------------------------------------------------------------------------------
const char *cc_track_state_name(cc_track_state_t state)
{
    switch (state) {
        case CC_TRACK_STATE_NORMAL: return "NORMAL";
        case CC_TRACK_STATE_LEFT_EDGE_LOST: return "LEFT_EDGE_LOST";
        case CC_TRACK_STATE_RIGHT_EDGE_LOST: return "RIGHT_EDGE_LOST";
        case CC_TRACK_STATE_CROSS: return "CROSS";
        case CC_TRACK_STATE_ROUNDABOUT_LEFT: return "ROUNDABOUT_LEFT";
        case CC_TRACK_STATE_ROUNDABOUT_RIGHT: return "ROUNDABOUT_RIGHT";
        case CC_TRACK_STATE_ZEBRA: return "ZEBRA";
        case CC_TRACK_STATE_LINE_LOST: return "LINE_LOST";
        default: return "UNKNOWN";
    }
}
