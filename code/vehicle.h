#ifndef CC_VEHICLE_H
#define CC_VEHICLE_H

#include <stdint.h>

#include "line_camera.h"
#include "motor4.h"
#include "pid.h"

typedef enum {
    CC_VEHICLE_STOPPED = 0,
    CC_VEHICLE_RUNNING = 1,
    CC_VEHICLE_LINE_LOST = 2,
    CC_VEHICLE_FAULT = 3
} cc_vehicle_state_t;

typedef struct {
    cc_line_camera_config_t line_camera;
    cc_motor4_config_t motor;
    float steering_kp;
    float steering_ki;
    float steering_kd;
    int16_t base_speed;
    int16_t maximum_steering;
    int16_t lost_search_steering;
    int16_t lost_speed;
    uint16_t lost_stop_frames;
} cc_vehicle_config_t;

typedef struct {
    cc_line_camera_t line_camera;
    cc_pid_t steering_pid;
    cc_motor4_config_t motor_config;
    cc_vehicle_state_t state;
    cc_motor4_command_t command;
    cc_line_result_t line;
    int16_t base_speed;
    int16_t maximum_steering;
    int16_t lost_search_steering;
    int16_t lost_speed;
    uint16_t lost_stop_frames;
    uint8_t enabled;
} cc_vehicle_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_config_default 函数声明
// 返回类型          void
// 使用示例          cc_vehicle_config_default(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
void cc_vehicle_config_default(cc_vehicle_config_t *config);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_init 函数声明
// 返回类型          void
// 使用示例          cc_vehicle_init(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
void cc_vehicle_init(cc_vehicle_t *vehicle, const cc_vehicle_config_t *config);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_set_enabled 函数声明
// 返回类型          void
// 使用示例          cc_vehicle_set_enabled(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
void cc_vehicle_set_enabled(cc_vehicle_t *vehicle, uint8_t enabled);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_step 函数声明
// 返回类型          cc_vehicle_state_t
// 使用示例          cc_vehicle_step(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
cc_vehicle_state_t cc_vehicle_step(cc_vehicle_t *vehicle, const cc_image_u8_t *image,
                                   float dt, cc_motor4_command_t *command);
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_state 函数声明
// 返回类型          cc_vehicle_state_t
// 使用示例          cc_vehicle_state(...);
// 备注信息          参数含义请参考同名源文件实现。
//-------------------------------------------------------------------------------------------------------------------
cc_vehicle_state_t cc_vehicle_state(const cc_vehicle_t *vehicle);

#endif

