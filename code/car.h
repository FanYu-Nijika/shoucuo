#ifndef CC_CAR_H
#define CC_CAR_H

#include "zf_common_headfile.h"
#include "car_params.h"

extern uint8 car_running;
extern uint8 car_camera_ready;
extern uint8 servo_reverse;
extern uint8 left_motor_reverse;
extern uint8 right_motor_reverse;

extern uint16 servo_center_duty;
extern uint16 servo_min_duty;
extern uint16 servo_max_duty;
extern uint16 car_servo_duty;
extern uint16 lost_stop_frames;
extern uint16 car_lost_count;

extern int16 motor_base_duty;
extern int16 motor_limit;
extern int16 curve_slowdown;
extern int16 car_left_command;
extern int16 car_right_command;

extern float steering_kp;
extern float steering_kd;

void car_init(void);
void car_set_camera_ready(uint8 ready);
void car_set_running(uint8 running);
void car_toggle_running(void);
void car_stop(void);
void car_track_update(float error, uint8 valid, uint8 new_result);
void car_apply_menu_params(const volatile car_params_t *params);
void car_control_clear_latched_result(void);

#endif
