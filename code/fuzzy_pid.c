#include "fuzzy_pid.h"

#include "math_utils.h"

/*
 * 三档隶属度比七档规则更容易在赛场上调整，也明显减少 TC264 每周期的计算量。
 * 表中数值是相对最大调整量的比例，不是最终 Kp、Ki、Kd。
 */
static const float cc_fuzzy_kp_rule[3][3] = {
    {-0.40f, 0.00f, 0.40f},
    { 0.00f, 0.45f, 0.75f},
    { 0.55f, 0.80f, 1.00f}
};

static const float cc_fuzzy_ki_rule[3][3] = {
    { 1.00f, 0.60f, 0.10f},
    { 0.40f, 0.10f,-0.40f},
    {-0.50f,-0.80f,-1.00f}
};

static const float cc_fuzzy_kd_rule[3][3] = {
    {-0.40f, 0.10f, 0.60f},
    { 0.00f, 0.40f, 0.80f},
    { 0.20f, 0.65f, 1.00f}
};

static float cc_fuzzy_abs(float value)
{
    return value < 0.0f ? -value : value;
}

static void cc_fuzzy_membership(float normalized, float membership[3])
{
    float value = cc_math_clamp_f32(normalized, 0.0f, 1.0f);
    membership[0] = value < 0.5f ? 1.0f - 2.0f * value : 0.0f;
    membership[1] = value < 0.5f ? 2.0f * value : 2.0f * (1.0f - value);
    membership[2] = value > 0.5f ? 2.0f * value - 1.0f : 0.0f;
}

static float cc_fuzzy_rule_output(const float error_membership[3],
                                  const float rate_membership[3],
                                  const float rule[3][3])
{
    uint8_t error_index;
    uint8_t rate_index;
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (error_index = 0; error_index < 3; ++error_index) {
        for (rate_index = 0; rate_index < 3; ++rate_index) {
            float weight = error_membership[error_index] * rate_membership[rate_index];
            numerator += weight * rule[error_index][rate_index];
            denominator += weight;
        }
    }
    return denominator > 0.0f ? numerator / denominator : 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          生成一组可直接运行的模糊 PID 默认参数
// 返回类型          void
// 参数说明          config              要写入的配置结构体
// 使用示例          cc_fuzzy_pid_config_default(&config);
// 备注信息          默认参数偏保守，防止未经整定时模糊调整量压过基础 PID。
//-------------------------------------------------------------------------------------------------------------------
void cc_fuzzy_pid_config_default(cc_fuzzy_pid_config_t *config)
{
    if (config == 0) return;
    config->kp = 1.0f;
    config->ki = 0.0f;
    config->kd = 0.0f;
    config->kp_adjust_max = 0.5f;
    config->ki_adjust_max = 0.1f;
    config->kd_adjust_max = 0.1f;
    config->error_full_scale = 1.0f;
    config->error_rate_full_scale = 5.0f;
    config->output_min = -1.0f;
    config->output_max = 1.0f;
    config->integral_min = -1.0f;
    config->integral_max = 1.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          初始化模糊 PID 控制器
// 返回类型          void
// 参数说明          controller          控制器对象
// 参数说明          config              配置参数，传入空指针时使用默认配置
// 使用示例          cc_fuzzy_pid_init(&steering_pid, &config);
// 备注信息          基础 PID 保留限幅和积分状态，模糊层只负责每周期调度增益。
//-------------------------------------------------------------------------------------------------------------------
void cc_fuzzy_pid_init(cc_fuzzy_pid_t *controller, const cc_fuzzy_pid_config_t *config)
{
    cc_fuzzy_pid_config_t default_config;
    if (controller == 0) return;
    cc_fuzzy_pid_config_default(&default_config);
    controller->config = config == 0 ? default_config : *config;
    if (controller->config.error_full_scale <= 0.0f) controller->config.error_full_scale = 1.0f;
    if (controller->config.error_rate_full_scale <= 0.0f) controller->config.error_rate_full_scale = 1.0f;
    cc_pid_init(&controller->pid,
                controller->config.kp,
                controller->config.ki,
                controller->config.kd,
                controller->config.output_min,
                controller->config.output_max);
    cc_pid_set_integral_limits(&controller->pid,
                               controller->config.integral_min,
                               controller->config.integral_max);
    controller->previous_error = 0.0f;
    controller->initialized = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          清除模糊 PID 的历史误差和积分
// 返回类型          void
// 参数说明          controller          控制器对象
// 使用示例          cc_fuzzy_pid_reset(&steering_pid);
// 备注信息          同时清除模糊层误差变化率和基础 PID 的积分状态。
//-------------------------------------------------------------------------------------------------------------------
void cc_fuzzy_pid_reset(cc_fuzzy_pid_t *controller)
{
    if (controller == 0) return;
    cc_pid_reset(&controller->pid);
    controller->previous_error = 0.0f;
    controller->initialized = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          使用目标值和测量值更新模糊 PID
// 返回类型          float               限幅后的控制输出
// 参数说明          controller          控制器对象
// 参数说明          setpoint            目标值
// 参数说明          measurement         测量值
// 参数说明          dt                  控制周期，单位为秒
// 使用示例          output = cc_fuzzy_pid_update(&pid, 0.0f, line_error, 0.01f);
// 备注信息          内部统一换算为 error=setpoint-measurement，避免两套规则产生方向差异。
//-------------------------------------------------------------------------------------------------------------------
float cc_fuzzy_pid_update(cc_fuzzy_pid_t *controller, float setpoint, float measurement, float dt)
{
    return cc_fuzzy_pid_update_error(controller, setpoint - measurement, dt);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          直接使用误差更新模糊 PID
// 返回类型          float               限幅后的控制输出
// 参数说明          controller          控制器对象
// 参数说明          error               当前误差
// 参数说明          dt                  控制周期，单位为秒
// 使用示例          output = cc_fuzzy_pid_update_error(&pid, error, 0.01f);
// 备注信息          使用加权平均解模糊，避免规则档位切换时控制量突跳。
//-------------------------------------------------------------------------------------------------------------------
float cc_fuzzy_pid_update_error(cc_fuzzy_pid_t *controller, float error, float dt)
{
    float rate = 0.0f;
    float error_membership[3];
    float rate_membership[3];
    float kp_adjust;
    float ki_adjust;
    float kd_adjust;

    if (controller == 0) return 0.0f;
    if (dt <= 0.0f) return controller->pid.last_output;
    if (controller->initialized != 0) rate = (error - controller->previous_error) / dt;

    cc_fuzzy_membership(cc_fuzzy_abs(error) / controller->config.error_full_scale,
                        error_membership);
    cc_fuzzy_membership(cc_fuzzy_abs(rate) / controller->config.error_rate_full_scale,
                        rate_membership);
    kp_adjust = cc_fuzzy_rule_output(error_membership, rate_membership, cc_fuzzy_kp_rule);
    ki_adjust = cc_fuzzy_rule_output(error_membership, rate_membership, cc_fuzzy_ki_rule);
    kd_adjust = cc_fuzzy_rule_output(error_membership, rate_membership, cc_fuzzy_kd_rule);

    controller->pid.kp = controller->config.kp + kp_adjust * controller->config.kp_adjust_max;
    controller->pid.ki = controller->config.ki + ki_adjust * controller->config.ki_adjust_max;
    controller->pid.kd = controller->config.kd + kd_adjust * controller->config.kd_adjust_max;
    if (controller->pid.kp < 0.0f) controller->pid.kp = 0.0f;
    if (controller->pid.ki < 0.0f) controller->pid.ki = 0.0f;
    if (controller->pid.kd < 0.0f) controller->pid.kd = 0.0f;

    controller->previous_error = error;
    controller->initialized = 1;
    return cc_pid_update_error(&controller->pid, error, dt);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          读取本周期模糊调度后的 PID 参数
// 返回类型          void
// 参数说明          controller          控制器对象
// 参数说明          kp                  Kp 输出地址，可传空指针
// 参数说明          ki                  Ki 输出地址，可传空指针
// 参数说明          kd                  Kd 输出地址，可传空指针
// 使用示例          cc_fuzzy_pid_get_gains(&pid, &kp, &ki, &kd);
// 备注信息          空输出地址会被跳过，便于只读取关心的参数。
//-------------------------------------------------------------------------------------------------------------------
void cc_fuzzy_pid_get_gains(const cc_fuzzy_pid_t *controller, float *kp, float *ki, float *kd)
{
    if (controller == 0) return;
    if (kp != 0) *kp = controller->pid.kp;
    if (ki != 0) *ki = controller->pid.ki;
    if (kd != 0) *kd = controller->pid.kd;
}
