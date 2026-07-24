#include "soft_timer.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_soft_timer_init 功能实现
// 返回类型          void
// 使用示例          cc_soft_timer_init(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_soft_timer_init(cc_soft_timer_t *timer, uint32_t period_ms, uint8_t repeat)
{
    if (timer == 0) return;
    timer->period_ms = period_ms;
    timer->elapsed_ms = 0;
    timer->running = 0;
    timer->repeat = repeat != 0 ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_soft_timer_start 功能实现
// 返回类型          void
// 使用示例          cc_soft_timer_start(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_soft_timer_start(cc_soft_timer_t *timer)
{
    if (timer == 0) return;
    timer->elapsed_ms = 0;
    timer->running = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_soft_timer_stop 功能实现
// 返回类型          void
// 使用示例          cc_soft_timer_stop(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_soft_timer_stop(cc_soft_timer_t *timer)
{
    if (timer == 0) return;
    timer->running = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_soft_timer_update 功能实现
// 返回类型          uint8_t
// 使用示例          cc_soft_timer_update(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_soft_timer_update(cc_soft_timer_t *timer, uint32_t elapsed_ms)
{
    uint8_t due = 0;
    if (timer == 0 || timer->running == 0 || timer->period_ms == 0) return 0;
    timer->elapsed_ms += elapsed_ms;
    if (timer->elapsed_ms < timer->period_ms) return 0;
    due = 1;
    if (timer->repeat != 0) {
        timer->elapsed_ms %= timer->period_ms;
    } else {
        timer->elapsed_ms = 0;
        timer->running = 0;
    }
    return due;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_soft_timer_running 功能实现
// 返回类型          uint8_t
// 使用示例          cc_soft_timer_running(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_soft_timer_running(const cc_soft_timer_t *timer)
{
    return (uint8_t)(timer != 0 && timer->running != 0);
}

