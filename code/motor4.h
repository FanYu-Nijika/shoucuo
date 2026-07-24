#ifndef CC_MOTOR4_H
#define CC_MOTOR4_H

#include <stdint.h>

typedef enum {
    CC_MOTOR_FRONT_LEFT = 0,
    CC_MOTOR_REAR_LEFT = 1,
    CC_MOTOR_FRONT_RIGHT = 2,
    CC_MOTOR_REAR_RIGHT = 3
} cc_motor4_index_t;

typedef struct {
    int16_t front_left;
    int16_t rear_left;
    int16_t front_right;
    int16_t rear_right;
} cc_motor4_command_t;

typedef struct {
    int16_t pwm_limit;
    int16_t deadzone;
    int8_t front_left_direction;
    int8_t rear_left_direction;
    int8_t front_right_direction;
    int8_t rear_right_direction;
} cc_motor4_config_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_config_default 函数声明
// 返回类型          void
// 使用示例          cc_motor4_config_default(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_config_default(cc_motor4_config_t *config);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_stop 函数声明
// 返回类型          void
// 使用示例          cc_motor4_stop(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_stop(cc_motor4_command_t *command);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_mix 函数声明
// 返回类型          void
// 使用示例          cc_motor4_mix(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_mix(const cc_motor4_config_t *config, int16_t throttle,
                   int16_t steering, cc_motor4_command_t *command);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_apply_slew 函数声明
// 返回类型          void
// 使用示例          cc_motor4_apply_slew(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_apply_slew(const cc_motor4_config_t *config,
                          const cc_motor4_command_t *target,
                          int16_t max_delta, cc_motor4_command_t *current);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_motor4_limit 函数声明
// 返回类型          void
// 使用示例          cc_motor4_limit(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
void cc_motor4_limit(const cc_motor4_config_t *config, cc_motor4_command_t *command);

#endif

