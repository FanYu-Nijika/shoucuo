#include "vehicle.h"

#include "math_utils.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_config_default 功能实现
// 返回类型          void
// 使用示例          cc_vehicle_config_default(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_vehicle_config_default(cc_vehicle_config_t *config)
{
    if (config == 0) return;
    config->line_camera.threshold = 100;
    config->line_camera.dark_is_line = 1;
    config->line_camera.minimum_line_pixels = 3;
    config->line_camera.row_step = 2;
    config->line_camera.roi_top = 0;
    config->line_camera.roi_bottom = 0;
    config->line_camera.center_offset_pixels = 0;
    config->line_camera.automatic_threshold = 1;
    config->line_camera.otsu_row_step = 2;
    config->line_camera.otsu_column_step = 2;
    cc_motor4_config_default(&config->motor);
    config->steering_kp = 600.0f;
    config->steering_ki = 0.0f;
    config->steering_kd = 25.0f;
    config->base_speed = 400;
    config->maximum_steering = 600;
    config->lost_search_steering = 250;
    config->lost_speed = 120;
    config->lost_stop_frames = 15;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_init 功能实现
// 返回类型          void
// 使用示例          cc_vehicle_init(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_vehicle_init(cc_vehicle_t *vehicle, const cc_vehicle_config_t *config)
{
    cc_vehicle_config_t defaults;
    if (vehicle == 0) return;
    cc_vehicle_config_default(&defaults);
    if (config != 0) defaults = *config;
    cc_line_camera_init(&vehicle->line_camera, &defaults.line_camera);
    cc_pid_init(&vehicle->steering_pid, defaults.steering_kp, defaults.steering_ki,
                defaults.steering_kd, -defaults.maximum_steering, defaults.maximum_steering);
    cc_pid_set_integral_limits(&vehicle->steering_pid, -defaults.maximum_steering, defaults.maximum_steering);
    vehicle->motor_config = defaults.motor;
    vehicle->state = CC_VEHICLE_STOPPED;
    cc_motor4_stop(&vehicle->command);
    vehicle->line.valid = 0;
    vehicle->line.valid_rows = 0;
    vehicle->line.lost_count = 0;
    vehicle->line.left_edge = 0;
    vehicle->line.right_edge = 0;
    vehicle->line.line_width = 0;
    vehicle->line.center_x = 0.0f;
    vehicle->line.error_pixels = 0.0f;
    vehicle->line.error_normalized = 0.0f;
    vehicle->line.heading_error = 0.0f;
    vehicle->line.strength = 0.0f;
    vehicle->line.threshold_used = defaults.line_camera.threshold;
    vehicle->base_speed = defaults.base_speed;
    vehicle->maximum_steering = defaults.maximum_steering;
    vehicle->lost_search_steering = defaults.lost_search_steering;
    vehicle->lost_speed = defaults.lost_speed;
    vehicle->lost_stop_frames = defaults.lost_stop_frames;
    vehicle->enabled = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_set_enabled 功能实现
// 返回类型          void
// 使用示例          cc_vehicle_set_enabled(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_vehicle_set_enabled(cc_vehicle_t *vehicle, uint8_t enabled)
{
    if (vehicle == 0) return;
    vehicle->enabled = enabled != 0 ? 1 : 0;
    if (vehicle->enabled == 0) {
        /* 禁止状态立刻清积分并清零输出，避免重新启动时沿用旧的转向量。 */
        cc_pid_reset(&vehicle->steering_pid);
        cc_motor4_stop(&vehicle->command);
        vehicle->state = CC_VEHICLE_STOPPED;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_step 功能实现
// 返回类型          cc_vehicle_state_t
// 使用示例          cc_vehicle_step(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
cc_vehicle_state_t cc_vehicle_step(cc_vehicle_t *vehicle, const cc_image_u8_t *image,
                                   float dt, cc_motor4_command_t *command)
{
    float steering_output;
    int16_t throttle;
    int16_t steering;

    if (vehicle == 0) return CC_VEHICLE_FAULT;
    if (vehicle->enabled == 0) {
        vehicle->state = CC_VEHICLE_STOPPED;
        cc_motor4_stop(&vehicle->command);
    } else if (!cc_image_u8_is_valid(image) || dt <= 0.0f) {
        vehicle->state = CC_VEHICLE_FAULT;
        cc_motor4_stop(&vehicle->command);
    } else {
        vehicle->line = cc_line_camera_update(&vehicle->line_camera, image);
        if (vehicle->line.valid != 0) {
            /* LINE_LOST -> RUNNING：重新看到足够长的线后恢复 PID 控制。 */
            vehicle->state = CC_VEHICLE_RUNNING;
            steering_output = cc_pid_update_error(&vehicle->steering_pid,
                                                  vehicle->line.error_normalized, dt);
            steering = (int16_t)cc_math_round_f32_to_i32(steering_output);
            throttle = vehicle->base_speed;
        } else if (vehicle->line.lost_count <= vehicle->lost_stop_frames) {
            /* RUNNING -> LINE_LOST：短暂丢线时低速按最后误差方向搜索。 */
            vehicle->state = CC_VEHICLE_LINE_LOST;
            cc_pid_reset(&vehicle->steering_pid);
            steering = vehicle->line.error_normalized >= 0.0f ? vehicle->lost_search_steering : (int16_t)-vehicle->lost_search_steering;
            throttle = vehicle->lost_speed;
        } else {
            /* LINE_LOST -> STOPPED：超过容错帧数后停车，防止冲出赛道。 */
            vehicle->state = CC_VEHICLE_STOPPED;
            cc_motor4_stop(&vehicle->command);
            steering = 0;
            throttle = 0;
        }
        if (vehicle->state == CC_VEHICLE_RUNNING || vehicle->state == CC_VEHICLE_LINE_LOST) {
            steering = (int16_t)cc_math_clamp_i32(steering, -vehicle->maximum_steering, vehicle->maximum_steering);
            cc_motor4_mix(&vehicle->motor_config, throttle, steering, &vehicle->command);
        }
    }
    if (command != 0) *command = vehicle->command;
    return vehicle->state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_vehicle_state 功能实现
// 返回类型          cc_vehicle_state_t
// 使用示例          cc_vehicle_state(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
cc_vehicle_state_t cc_vehicle_state(const cc_vehicle_t *vehicle)
{
    return vehicle == 0 ? CC_VEHICLE_FAULT : vehicle->state;
}
