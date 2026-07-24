#ifndef CC_TRACK_REPAIR_H
#define CC_TRACK_REPAIR_H

#include <stdint.h>

typedef struct {
    int16_t invalid_value;
    uint16_t nominal_track_width;
    uint16_t minimum_track_width;
    uint16_t maximum_track_width;
    uint16_t maximum_gap_rows;
} cc_track_repair_config_t;

typedef struct {
    uint16_t measured_rows;
    uint16_t repaired_rows;
    uint16_t remaining_invalid_rows;
    uint16_t estimated_track_width;
    float confidence;
} cc_track_repair_result_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          生成补线算法默认配置
// 返回类型          void
// 参数说明          config              要写入的配置结构体
// 使用示例          cc_track_repair_config_default(&config);
// 备注信息          invalid_value 默认是 -1，边界数组必须使用 int16_t 保存。
//-------------------------------------------------------------------------------------------------------------------
void cc_track_repair_config_default(cc_track_repair_config_t *config);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          修补左右边界中的单边丢失和短距离双边缺口
// 返回类型          cc_track_repair_result_t  本次补线统计结果
// 参数说明          left                左边界数组，函数会原地修改
// 参数说明          right               右边界数组，函数会原地修改
// 参数说明          row_count           数组行数
// 参数说明          image_width         图像宽度
// 参数说明          config              补线配置，传空指针时使用默认配置
// 使用示例          result = cc_track_repair_edges(left, right, height, width, &config);
// 备注信息          长缺口不会强行补齐，因为虚构很长的赛道边界会误导后续状态识别。
//-------------------------------------------------------------------------------------------------------------------
cc_track_repair_result_t cc_track_repair_edges(int16_t *left, int16_t *right,
                                               uint16_t row_count, uint16_t image_width,
                                               const cc_track_repair_config_t *config);

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
// 备注信息          无效行的 center 也写入 invalid_value，调用方可以直接跳过。
//-------------------------------------------------------------------------------------------------------------------
uint16_t cc_track_build_centerline(const int16_t *left, const int16_t *right,
                                   int16_t *center, uint16_t row_count,
                                   uint16_t image_width, int16_t invalid_value);

#endif
