#include "motor4.h"

#include "math_utils.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_limit_one 功能实现
// 返回类型          static int16_t
// 使用示例          cc_motor4_limit_one(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
static int16_t cc_motor4_limit_one(const cc_motor4_config_t *config, int32_t value)
{
    int32_t limited;
    int32_t deadzone;
    int32_t limit;
    if (config == 0) return (int16_t)value;
    limit = config->pwm_limit;
    if (limit < 0) limit = -limit;
    limited = cc_math_clamp_i32(value, -limit, limit);
    deadzone = config->deadzone;
    if (deadzone < 0) deadzone = -deadzone;
    if (limited != 0 && limited > -deadzone && limited < deadzone) limited = limited > 0 ? deadzone : -deadzone;
    return (int16_t)limited;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_config_default 功能实现
// 返回类型          void
// 使用示例          cc_motor4_config_default(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_config_default(cc_motor4_config_t *config)
{
    if (config == 0) return;
    config->pwm_limit = 1000;
    config->deadzone = 0;
    config->front_left_direction = 1;
    config->rear_left_direction = 1;
    config->front_right_direction = 1;
    config->rear_right_direction = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_stop 功能实现
// 返回类型          void
// 使用示例          cc_motor4_stop(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_stop(cc_motor4_command_t *command)
{
    if (command == 0) return;
    command->front_left = 0;
    command->rear_left = 0;
    command->front_right = 0;
    command->rear_right = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_mix 功能实现
// 返回类型          void
// 使用示例          cc_motor4_mix(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_mix(const cc_motor4_config_t *config, int16_t throttle,
                   int16_t steering, cc_motor4_command_t *command)
{
    int32_t left = (int32_t)throttle + steering;
    int32_t right = (int32_t)throttle - steering;
    if (config == 0 || command == 0) return;
    command->front_left = cc_motor4_limit_one(config, left * config->front_left_direction);
    command->rear_left = cc_motor4_limit_one(config, left * config->rear_left_direction);
    command->front_right = cc_motor4_limit_one(config, right * config->front_right_direction);
    command->rear_right = cc_motor4_limit_one(config, right * config->rear_right_direction);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_limit 功能实现
// 返回类型          void
// 使用示例          cc_motor4_limit(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_limit(const cc_motor4_config_t *config, cc_motor4_command_t *command)
{
    if (config == 0 || command == 0) return;
    command->front_left = cc_motor4_limit_one(config, command->front_left);
    command->rear_left = cc_motor4_limit_one(config, command->rear_left);
    command->front_right = cc_motor4_limit_one(config, command->front_right);
    command->rear_right = cc_motor4_limit_one(config, command->rear_right);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_approach_one 功能实现
// 返回类型          static int16_t
// 使用示例          cc_motor4_approach_one(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
static int16_t cc_motor4_approach_one(int16_t current, int16_t target, int16_t max_delta)
{
    int32_t difference = (int32_t)target - current;
    int32_t delta = max_delta;
    if (delta < 0) delta = -delta;
    if (difference > delta) return (int16_t)(current + delta);
    if (difference < -delta) return (int16_t)(current - delta);
    return target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_apply_slew 功能实现
// 返回类型          void
// 使用示例          cc_motor4_apply_slew(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_apply_slew(const cc_motor4_config_t *config,
                          const cc_motor4_command_t *target,
                          int16_t max_delta, cc_motor4_command_t *current)
{
    if (config == 0 || target == 0 || current == 0) return;
    current->front_left = cc_motor4_approach_one(current->front_left, target->front_left, max_delta);
    current->rear_left = cc_motor4_approach_one(current->rear_left, target->rear_left, max_delta);
    current->front_right = cc_motor4_approach_one(current->front_right, target->front_right, max_delta);
    current->rear_right = cc_motor4_approach_one(current->rear_right, target->rear_right, max_delta);
    cc_motor4_limit(config, current);
}

