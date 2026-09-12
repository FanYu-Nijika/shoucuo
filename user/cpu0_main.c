/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一�?基于官方 SDK 接口的�??三方开源库
* Copyright (c) 2022 SEEKFREE 逐�?��?�技
*
* �?文件�? TC264 开源库的一部分
*
* TC264 开源库 �?免费�?�?
* 您可以根�?�?由软件基金会发布�? GPL（GNU General Public License，即 GNU通用�?共�?�可证）的条�?
* �? GPL 的�??3版（�? GPL3.0）或（您选择的）任何后来的版�?，重新发布和/或修改它
*
* �?开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更�?�细节�?�参�? GPL
*
* 您应该在收到�?开源库的同时收到一�? GPL 的副�?
* 如果没有，�?�参�?<https://www.gnu.org/licenses/>
*
* 额�?�注明：
* �?开源库使用 GPL3.0 开源�?�可证协�? 以上许可申明为译文版�?
* 许可申明英文版在 libraries/doc 文件夹下�? GPL3_permission_statement.txt 文件�?
* 许可证副�?�? libraries 文件夹下 即�?�文件夹下的 LICENSE 文件
* 欢迎各位使用并传�?�?程序 但修改内容时必须保留逐�?��?�技的版权声明（即本声明�?
*
* 文件名称          cpu0_main
* �?司名�?          成都逐�?��?�技有限�?�?
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环�?          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* �?改�?�录
* 日期              作�?                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "isr_config.h"
#include <string.h>

#include "car_menu.h"
#include "car.h"
#include "car_menu_port.h"
#include "balance_control.h"
#include "balance_runtime.h"
#include "car_shared.h"
#include "image.h"

#pragma section all "cpu0_dsram"
// 将本�?句与#pragma section all restore�?句之间的全局变量都放在CPU0的RAM�?

// �?例程�?开源库空工�? �?用作移�?�或者测试各类内外�??
// �?例程�?开源库空工�? �?用作移�?�或者测试各类内外�??
// �?例程�?开源库空工�? �?用作移�?�或者测试各类内外�??

#define PIT_NUM                 (CCU60_CH0 )
#define CAR_CONTROL_PERIOD_MS   (5)


volatile uint32 car_time_ms = 0;
volatile float control_error = 0;
volatile float control_curvature = 0;
volatile uint8 control_valid = 0;
volatile uint8 control_result_new = 0;
volatile uint32 control_result_age_ms = 0;

void car_control_clear_latched_result(void)
{
    control_error = 0;
    control_curvature = 0;
    control_valid = 0;
    control_result_new = 0;
    control_result_age_ms = 0;
    __dsync();
}

static int8_t find_free_frame_slot(uint8_t preferred_slot)
{
    uint8_t offset;

    for (offset = 0; offset < CAR_FRAME_SLOT_COUNT; offset++) {
        uint8_t slot = (uint8_t)((preferred_slot + offset) % CAR_FRAME_SLOT_COUNT);

        if (car_frame_state[slot] == CAR_FRAME_FREE) return (int8_t)slot;
    }
    return -1;
}

static void update_display_slot(uint8_t slot)
{
    uint8_t old_slot = car_display_slot;

    if (old_slot != CAR_FRAME_SLOT_NONE && old_slot != slot && old_slot < CAR_FRAME_SLOT_COUNT) {
        if (car_frame_state[old_slot] == CAR_FRAME_DISPLAY_READING) {
            car_display_pending_free_slot = old_slot;
        } else if (car_frame_state[old_slot] == CAR_FRAME_DISPLAY_READY) {
            car_frame_state[old_slot] = CAR_FRAME_FREE;
            car_display_drop_count++;
        }
    }
    car_display_slot = slot;
    __dsync();
}

// **************************** 代码区域 ****************************
int core0_main(void)
{
    uint32 last_frame_ms = 0;
    uint8 next_frame_slot = 0;

    clock_init();                   // 获取时钟频率<务必保留>
    /* P11.11 drives the buzzer transistor; hold it low before slower peripheral initialization. */
    gpio_init(BOARD_BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    debug_init();                   // 初�?�化默�?�调试串�?
    // 此�?�编写用户代�? 例�?��?��?�初始化代码�?

    car_menu_init();


    // 此�?�编写用户代�? 例�?��?��?�初始化代码�?
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    /* All slow peripheral/UI initialization is complete before automatic balancing. */
    balance_runtime_init();
    pit_ms_init(PIT_NUM, CAR_CONTROL_PERIOD_MS);
    while (TRUE) {
        // 此�?�编写需要循�?执�?�的代码

        if (mt9v03x_finish_flag != 0) {
            uint32 now_ms = system_getval_ms();
            uint32 frame_period_ms = last_frame_ms == 0 ? 20 : now_ms - last_frame_ms;
            int8_t slot;

            car_camera_frame_count++;
            car_camera_last_frame_ms = now_ms;
            car_camera_age_ms = 0;

            if (frame_period_ms == 0 || frame_period_ms > 250) frame_period_ms = 20;
            last_frame_ms = now_ms;

            slot = find_free_frame_slot(next_frame_slot);
            if (slot >= 0) {
                memcpy(&car_gray_frames[(uint8_t)slot][0][0], &mt9v03x_image[0][0], sizeof(car_gray_frames[0]));
                car_frame_sequence++;
                car_frame_period_ms = frame_period_ms;
                car_frame_slot_sequence[(uint8_t)slot] = car_frame_sequence;
                car_frame_slot_period_ms[(uint8_t)slot] = frame_period_ms;
                __dsync();
                car_frame_state[(uint8_t)slot] = CAR_FRAME_READY;
                __dsync();
                next_frame_slot = (uint8_t)(((uint8_t)slot + 1) % CAR_FRAME_SLOT_COUNT);
                car_menu_frame_accepted(frame_period_ms);
            } else {
                car_capture_drop_count++;
            }
            mt9v03x_finish_flag = 0;
        }

        car_camera_age_ms = car_camera_frame_count == 0 ? 0 : system_getval_ms() - car_camera_last_frame_ms;

        if (car_result_ready != 0) {
            car_result_t result;

            __dsync();
            result = car_result;
            car_result_ready = 0;
            __dsync();

            control_error = result.error_pixels;
            control_curvature = result.curvature;
            control_valid = result.line_valid;
            control_result_age_ms = 0;
            control_result_new = 1;
            __dsync();

            if (result.frame_slot < CAR_FRAME_SLOT_COUNT)
                update_display_slot(result.frame_slot);

            car_menu_result_accepted();
        }
        if (car_center_stop_request != 0) {
            car_center_stop_request = 0;
            __dsync();
            car_menu_handle_center();
        }
        balance_runtime_task();
        car_menu_task();

        // 此�?�编写需要循�?执�?�的代码
    }
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    static uint32 previous_tick;
    static uint8 late_tick_count;
    uint32 start_ticks = system_getval();
    uint32 elapsed_us;

    pit_clear_flag(CCU60_CH0);
    if (car_balance_timing_reset != 0) {
        previous_tick = 0;
        late_tick_count = 0;
        car_balance_timing_reset = 0;
    }
    if (previous_tick != 0) {
        car_balance_period_us = (start_ticks - previous_tick) / 100;
        if (car_balance_period_us > 6000 && balance_state.running != 0) {
            car_balance_overruns++;
            if (++late_tick_count >= 3) {
                balance_runtime_stop();
                balance_state.fault = BALANCE_FAULT_TIMING;
            }
        } else late_tick_count = 0;
    }
    previous_tick = start_ticks;
    car_uptime_ms += CAR_CONTROL_PERIOD_MS;
    /* Sample emergency stop in the control ISR, not after a slow TFT frame. */
    if (car_running != 0 && (cc_tc264_menu_key_mask() & CC_KEY_CENTER_MASK) != 0) {
        balance_runtime_stop();
        car_center_stop_request = 1;
    }
    balance_runtime_update_5ms();
    if (car_running != 0) car_time_ms += CAR_CONTROL_PERIOD_MS;
    else car_time_ms = 0;
    gpio_set_level(BOARD_BUZZER_PIN, GPIO_LOW);
    elapsed_us = (system_getval() - start_ticks) / 100;
    if (elapsed_us > car_balance_max_us) car_balance_max_us = elapsed_us;
    if (elapsed_us >= 5000) {
        car_balance_overruns++;
        balance_runtime_stop();
        balance_state.fault = BALANCE_FAULT_TIMING;
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
