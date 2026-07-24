#include "track_repair.h"

static uint8_t cc_track_edge_valid(int16_t edge, uint16_t image_width, int16_t invalid_value)
{
    if (edge == invalid_value) return 0;
    return edge >= 0 && (uint16_t)edge < image_width;
}

static uint8_t cc_track_pair_valid(int16_t left, int16_t right, uint16_t image_width,
                                   uint16_t minimum_width, uint16_t maximum_width,
                                   int16_t invalid_value)
{
    uint16_t width;
    if (!cc_track_edge_valid(left, image_width, invalid_value)) return 0;
    if (!cc_track_edge_valid(right, image_width, invalid_value)) return 0;
    if (left >= right) return 0;
    width = (uint16_t)(right - left);
    return width >= minimum_width && width <= maximum_width;
}

static int16_t cc_track_clamp_edge(int32_t edge, uint16_t image_width)
{
    if (edge < 0) return 0;
    if (edge >= image_width) return (int16_t)(image_width - 1u);
    return (int16_t)edge;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          生成补线算法默认配置
// 返回类型          void
// 参数说明          config              要写入的配置结构体
// 使用示例          cc_track_repair_config_default(&config);
// 备注信息          默认只补 8 行以内缺口，优先保证补线可信度而不是覆盖率。
//-------------------------------------------------------------------------------------------------------------------
void cc_track_repair_config_default(cc_track_repair_config_t *config)
{
    if (config == 0) return;
    config->invalid_value = -1;
    config->nominal_track_width = 0;
    config->minimum_track_width = 3;
    config->maximum_track_width = 0;
    config->maximum_gap_rows = 8;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          修补左右边界中的单边丢失和短距离双边缺口
// 返回类型          cc_track_repair_result_t  本次补线统计结果
// 参数说明          left                左边界数组，函数会原地修改
// 参数说明          right               右边界数组，函数会原地修改
// 参数说明          row_count           数组行数
// 参数说明          image_width         图像宽度
// 参数说明          config              补线配置，传空指针时使用默认配置
// 使用示例          result = cc_track_repair_edges(left, right, height, width, &config);
// 备注信息          先由可靠行估计赛道宽度，再补单边，最后仅在线性锚点充分时补双边缺口。
//-------------------------------------------------------------------------------------------------------------------
cc_track_repair_result_t cc_track_repair_edges(int16_t *left, int16_t *right,
                                               uint16_t row_count, uint16_t image_width,
                                               const cc_track_repair_config_t *config)
{
    cc_track_repair_config_t local_config;
    cc_track_repair_result_t result = {0, 0, 0, 0, 0.0f};
    uint32_t width_sum = 0;
    uint16_t width_count = 0;
    uint16_t maximum_width;
    uint16_t estimated_width;
    uint16_t row;

    cc_track_repair_config_default(&local_config);
    if (config != 0) local_config = *config;
    if (left == 0 || right == 0 || row_count == 0 || image_width < 2) {
        result.remaining_invalid_rows = row_count;
        return result;
    }
    maximum_width = local_config.maximum_track_width == 0 ||
                    local_config.maximum_track_width >= image_width
                    ? (uint16_t)(image_width - 1u)
                    : local_config.maximum_track_width;

    for (row = 0; row < row_count; ++row) {
        if (cc_track_pair_valid(left[row], right[row], image_width,
                                local_config.minimum_track_width, maximum_width,
                                local_config.invalid_value)) {
            width_sum += (uint16_t)(right[row] - left[row]);
            ++width_count;
            ++result.measured_rows;
        }
    }
    estimated_width = local_config.nominal_track_width;
    if (estimated_width == 0 && width_count != 0) estimated_width = (uint16_t)(width_sum / width_count);
    if (estimated_width != 0 && estimated_width < local_config.minimum_track_width) {
        estimated_width = local_config.minimum_track_width;
    }
    if (estimated_width > maximum_width) estimated_width = maximum_width;
    result.estimated_track_width = estimated_width;

    for (row = 0; row < row_count; ++row) {
        uint8_t left_valid = cc_track_edge_valid(left[row], image_width, local_config.invalid_value);
        uint8_t right_valid = cc_track_edge_valid(right[row], image_width, local_config.invalid_value);
        if (left_valid && right_valid) {
            uint16_t width = left[row] < right[row] ? (uint16_t)(right[row] - left[row]) : 0;
            if (width < local_config.minimum_track_width || width > maximum_width) {
                left_valid = 0;
                right_valid = 0;
                left[row] = local_config.invalid_value;
                right[row] = local_config.invalid_value;
            }
        }
        if (left_valid && !right_valid && estimated_width != 0) {
            right[row] = cc_track_clamp_edge((int32_t)left[row] + estimated_width, image_width);
            if (right[row] > left[row]) ++result.repaired_rows;
            else right[row] = local_config.invalid_value;
        } else if (!left_valid && right_valid && estimated_width != 0) {
            left[row] = cc_track_clamp_edge((int32_t)right[row] - estimated_width, image_width);
            if (left[row] < right[row]) ++result.repaired_rows;
            else left[row] = local_config.invalid_value;
        }
    }

    row = 0;
    while (row < row_count) {
        uint16_t gap_start;
        uint16_t gap_end;
        uint16_t gap_length;
        uint16_t index;
        if (cc_track_pair_valid(left[row], right[row], image_width, 1, maximum_width,
                                local_config.invalid_value)) {
            ++row;
            continue;
        }
        gap_start = row;
        while (row < row_count &&
               !cc_track_pair_valid(left[row], right[row], image_width, 1, maximum_width,
                                    local_config.invalid_value)) ++row;
        gap_end = row;
        gap_length = (uint16_t)(gap_end - gap_start);
        if (gap_length > local_config.maximum_gap_rows || gap_start == 0 || gap_end >= row_count) continue;
        if (!cc_track_pair_valid(left[gap_start - 1u], right[gap_start - 1u], image_width,
                                 1, maximum_width, local_config.invalid_value)) continue;
        if (!cc_track_pair_valid(left[gap_end], right[gap_end], image_width,
                                 1, maximum_width, local_config.invalid_value)) continue;

        for (index = 0; index < gap_length; ++index) {
            uint16_t numerator = (uint16_t)(index + 1u);
            uint16_t denominator = (uint16_t)(gap_length + 1u);
            int32_t left_delta = (int32_t)left[gap_end] - left[gap_start - 1u];
            int32_t right_delta = (int32_t)right[gap_end] - right[gap_start - 1u];
            left[gap_start + index] = (int16_t)(left[gap_start - 1u] +
                                                left_delta * numerator / denominator);
            right[gap_start + index] = (int16_t)(right[gap_start - 1u] +
                                                  right_delta * numerator / denominator);
            ++result.repaired_rows;
        }
    }

    for (row = 0; row < row_count; ++row) {
        if (!cc_track_pair_valid(left[row], right[row], image_width, 1, maximum_width,
                                 local_config.invalid_value)) ++result.remaining_invalid_rows;
    }
    result.confidence = (float)(row_count - result.remaining_invalid_rows) / (float)row_count;
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          根据已修补的左右边界生成中线
// 返回类型          uint16_t            成功生成中线的行数
// 参数说明          left                左边界数组
// 参数说明          right               右边界数组
// 参数说明          center              中线输出数组
// 参数说明          row_count           数组行数
// 参数说明          image_width         图像宽度
// 参数说明          invalid_value       无效行标记
// 使用示例          valid = cc_track_build_centerline(left, right, center, height, width, -1);
// 备注信息          只接受左右顺序正确的边界，防止错误中线进入转向控制。
//-------------------------------------------------------------------------------------------------------------------
uint16_t cc_track_build_centerline(const int16_t *left, const int16_t *right,
                                   int16_t *center, uint16_t row_count,
                                   uint16_t image_width, int16_t invalid_value)
{
    uint16_t row;
    uint16_t valid_rows = 0;
    if (left == 0 || right == 0 || center == 0) return 0;
    for (row = 0; row < row_count; ++row) {
        if (cc_track_edge_valid(left[row], image_width, invalid_value) &&
            cc_track_edge_valid(right[row], image_width, invalid_value) &&
            left[row] < right[row]) {
            center[row] = (int16_t)(((int32_t)left[row] + right[row]) / 2);
            ++valid_rows;
        } else {
            center[row] = invalid_value;
        }
    }
    return valid_rows;
}
