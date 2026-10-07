#ifndef CC_CAR_H
#define CC_CAR_H

#include "zf_common_headfile.h"
#include "car_params.h"

/* START waits stopped for 2 s, waits for RUN, then holds zero-speed RUN for 2 s with camera tracking.
 * The remaining 300 ms / 2 s / 300 ms stage timers are retained; the launch offset is disabled (0 degrees).
 * STOP cancels every stage; a lost RUN without a fault restarts from WAIT_RUN and the full upright hold. */
enum {
    CAR_FOLLOW_IDLE = 0,
    CAR_FOLLOW_WAIT_RUN,
    CAR_FOLLOW_WAIT_DELAY,
    CAR_FOLLOW_ACTIVE,
    CAR_FOLLOW_RUN_UPRIGHT,
    CAR_FOLLOW_LEAN_IN,
    CAR_FOLLOW_LEAN_HOLD,
    CAR_FOLLOW_LEAN_OUT
};

extern volatile uint8 car_running;
extern uint8 car_camera_ready;
extern uint16 car_lost_count;
extern int16 car_left_command;
extern int16 car_right_command;
extern float steering_kp;
extern float steering_kd;
extern uint8 car_curve_mode;
extern float curve_steering_kp;
extern float curve_steering_kd;
extern volatile uint8 car_follow_status;
extern volatile uint8 car_follow_enabled;
extern volatile float car_turn_request;

void car_init(void);
void car_set_camera_ready(uint8 ready);
void car_set_running(uint8 running);
void car_stop(void);
uint8 car_follow_start(void);
void car_follow_stop(void);
void car_track_update(float error, float curvature, uint8 valid, uint8 new_result, uint32 capture_ms, uint32 now_ms);
void car_apply_menu_params(const volatile car_params_t *params);
void car_control_clear_latched_result(void);

#endif
