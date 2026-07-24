#ifndef CC_FUZZY_PID_H
#define CC_FUZZY_PID_H

#include <stdint.h>

#include "pid.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float kp_adjust_max;
    float ki_adjust_max;
    float kd_adjust_max;
    float error_full_scale;
    float error_rate_full_scale;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
} cc_fuzzy_pid_config_t;

typedef struct {
    cc_pid_t pid;
    cc_fuzzy_pid_config_t config;
    float previous_error;
    uint8_t initialized;
} cc_fuzzy_pid_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          生成一组可直接运行的模糊 PID 默认参数
// 返回类型          void
// 参数说明          config              要写入的配置结构体
// 使用示例          cc_fuzzy_pid_config_default(&config);
// 备注信息          默认值只是安全起点，实际车辆仍需根据转向机构和控制周期整定。
//-------------------------------------------------------------------------------------------------------------------
void cc_fuzzy_pid_config_default(cc_fuzzy_pid_config_t *config);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          初始化模糊 PID 控制器
// 返回类型          void
// 参数说明          controller          控制器对象
// 参数说明          config              配置参数，传入空指针时使用默认配置
// 使用示例          cc_fuzzy_pid_init(&steering_pid, &config);
// 备注信息          模糊层只调整 Kp、Ki、Kd，积分限幅和输出限幅仍由基础 PID 完成。
//-------------------------------------------------------------------------------------------------------------------
void cc_fuzzy_pid_init(cc_fuzzy_pid_t *controller, const cc_fuzzy_pid_config_t *config);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          清除模糊 PID 的历史误差和积分
// 返回类型          void
// 参数说明          controller          控制器对象
// 使用示例          cc_fuzzy_pid_reset(&steering_pid);
// 备注信息          车辆重新起步或控制模式切换时调用，避免沿用旧状态造成瞬时冲击。
//-------------------------------------------------------------------------------------------------------------------
void cc_fuzzy_pid_reset(cc_fuzzy_pid_t *controller);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          使用目标值和测量值更新模糊 PID
// 返回类型          float               限幅后的控制输出
// 参数说明          controller          控制器对象
// 参数说明          setpoint            目标值
// 参数说明          measurement         测量值
// 参数说明          dt                  控制周期，单位为秒
// 使用示例          output = cc_fuzzy_pid_update(&pid, 0.0f, line_error, 0.01f);
// 备注信息          dt 必须大于零，否则保持上一次输出。
//-------------------------------------------------------------------------------------------------------------------
float cc_fuzzy_pid_update(cc_fuzzy_pid_t *controller, float setpoint, float measurement, float dt);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          直接使用误差更新模糊 PID
// 返回类型          float               限幅后的控制输出
// 参数说明          controller          控制器对象
// 参数说明          error               当前误差
// 参数说明          dt                  控制周期，单位为秒
// 使用示例          output = cc_fuzzy_pid_update_error(&pid, error, 0.01f);
// 备注信息          误差大时优先增强比例，误差变化快时增强微分，误差小时增强积分。
//-------------------------------------------------------------------------------------------------------------------
float cc_fuzzy_pid_update_error(cc_fuzzy_pid_t *controller, float error, float dt);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          读取本周期模糊调度后的 PID 参数
// 返回类型          void
// 参数说明          controller          控制器对象
// 参数说明          kp                  Kp 输出地址，可传空指针
// 参数说明          ki                  Ki 输出地址，可传空指针
// 参数说明          kd                  Kd 输出地址，可传空指针
// 使用示例          cc_fuzzy_pid_get_gains(&pid, &kp, &ki, &kd);
// 备注信息          便于串口观察模糊规则是否符合实际车辆响应。
//-------------------------------------------------------------------------------------------------------------------
void cc_fuzzy_pid_get_gains(const cc_fuzzy_pid_t *controller, float *kp, float *ki, float *kd);

#endif
