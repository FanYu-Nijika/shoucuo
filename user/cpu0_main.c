/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
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
#include "balance_vofa.h"
#include "car_shared.h"
#include "image.h"

#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程，可用作移植或者测试各类内外设
// 本例程是开源库空工程，可用作移植或者测试各类内外设
// 本例程是开源库空工程，可用作移植或者测试各类内外设

#define PIT_NUM                 (CCU60_CH0 )
#define CAR_CONTROL_PERIOD_MS   (BALANCE_CONTROL_PERIOD_MS)
#define CAMERA_DMA_RECOVER_MS   (80)
#define CAMERA_HARD_RESTART_MS  (600)
#define CAMERA_RETRY_MS         (2000)


volatile uint32 car_time_ms = 0;

void car_control_clear_latched_result(void)
{
    uint8 slot = (uint8)(car_control_snapshot_index ^ 1U);

    car_control_snapshots[slot].error = 0.0f;
    car_control_snapshots[slot].curvature = 0.0f;
    car_control_snapshots[slot].capture_time_ms = 0;
    car_control_snapshots[slot].sequence = 0;
    car_control_snapshots[slot].valid = 0;
    __dsync();
    car_control_snapshot_index = slot;
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
    uint32 camera_watchdog_ms = 0;
    uint32 camera_last_dma_recover_ms = 0;
    uint32 camera_last_restart_ms = 0;
    uint32 camera_watchdog_frame_count = 0;
    uint8 next_frame_slot = 0;

    clock_init();                   // 获取时钟频率<务必保留>
    /* Hold the buzzer low before slower peripheral initialization. */
    gpio_init(BOARD_BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码，例如外设初始化代码

    car_menu_init();

    // 此处编写用户代码，例如外设初始化代码
    /* All slow peripheral/UI initialization is complete before automatic balancing. */
    balance_runtime_init();
    pit_ms_init(PIT_NUM, CAR_CONTROL_PERIOD_MS);
    camera_watchdog_ms = system_getval_ms();
    camera_last_restart_ms = camera_watchdog_ms;
    wireless_uart_init();
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE) {
        // 此处编写需要循环执行的代码
        if (mt9v03x_finish_flag != 0) {
            uint32 now_ms = system_getval_ms();
            uint32 capture_time_ms = car_camera_completed_time_ms;
            uint32 frame_period_ms;
            int8_t slot;

            /* Consume completed frames before menu drawing so the next VSYNC can capture promptly. */
            if (capture_time_ms == 0) capture_time_ms = now_ms;
            frame_period_ms = last_frame_ms == 0 ? 20 : capture_time_ms - last_frame_ms;
            if (frame_period_ms == 0 || frame_period_ms > 250) frame_period_ms = 20;
            last_frame_ms = capture_time_ms;
            car_camera_frame_count++;
            car_camera_last_frame_ms = capture_time_ms;
            car_camera_age_ms = now_ms - capture_time_ms;

            slot = find_free_frame_slot(next_frame_slot);
            if (slot >= 0) {
                /* Publish pixels only after the full camera buffer has been copied. */
                memcpy(&car_gray_frames[(uint8_t)slot][0][0], &mt9v03x_image[0][0], sizeof(car_gray_frames[0]));
                car_frame_sequence++;
                car_frame_period_ms = frame_period_ms;
                car_frame_slot_sequence[(uint8_t)slot] = car_frame_sequence;
                car_frame_slot_period_ms[(uint8_t)slot] = frame_period_ms;
                car_frame_slot_capture_time_ms[(uint8_t)slot] = capture_time_ms;
                __dsync();
                car_frame_state[(uint8_t)slot] = CAR_FRAME_READY;
                __dsync();
                next_frame_slot = (uint8_t)(((uint8_t)slot + 1) % CAR_FRAME_SLOT_COUNT);
                car_menu_frame_accepted(frame_period_ms);
            } else {
                car_capture_drop_count++;
            }
            __dsync();
            mt9v03x_finish_flag = 0;
            __dsync();
        }

        car_camera_age_ms = car_camera_frame_count == 0 ? 0 : system_getval_ms() - car_camera_last_frame_ms;

        mt9v03x_dma_recovery_task();
        {
            uint32 now_ms = system_getval_ms();
            uint32 stale_ms;

            if (car_camera_frame_count != camera_watchdog_frame_count) {
                camera_watchdog_frame_count = car_camera_frame_count;
                camera_watchdog_ms = now_ms;
            }
            stale_ms = now_ms - camera_watchdog_ms;

            if (cc_tc264_camera_ready() != 0) {
                if (stale_ms >= CAMERA_DMA_RECOVER_MS && now_ms - camera_last_dma_recover_ms >= CAMERA_DMA_RECOVER_MS) {
                    mt9v03x_request_dma_recover();
                    camera_last_dma_recover_ms = now_ms;
                }
                if (stale_ms >= CAMERA_HARD_RESTART_MS && now_ms - camera_last_restart_ms >= CAMERA_HARD_RESTART_MS) {
                    uint8 camera_ready = cc_tc264_camera_restart();

                    camera_last_restart_ms = now_ms;
                    car_set_camera_ready(camera_ready);
                    if (camera_ready != 0) {
                        cc_tc264_camera_set_exposure(car_params.exposure);
                        cc_tc264_camera_set_gain(car_params.gain);
                        camera_watchdog_ms = system_getval_ms();
                    }
                }
            } else if (now_ms - camera_last_restart_ms >= CAMERA_RETRY_MS) {
                uint8 camera_ready = cc_tc264_camera_restart();

                camera_last_restart_ms = now_ms;
                car_set_camera_ready(camera_ready);
                if (camera_ready != 0) {
                    cc_tc264_camera_set_exposure(car_params.exposure);
                    cc_tc264_camera_set_gain(car_params.gain);
                    camera_watchdog_ms = system_getval_ms();
                }
            }
        }

        if (car_result_ready != 0) {
            car_result_t result;
            uint8 snapshot_slot;

            __dsync();
            result = car_result;
            car_result_ready = 0;
            __dsync();

            snapshot_slot = (uint8)(car_control_snapshot_index ^ 1U);
            car_control_snapshots[snapshot_slot].error = result.error_pixels;
            car_control_snapshots[snapshot_slot].curvature = result.curvature;
            car_control_snapshots[snapshot_slot].capture_time_ms = result.capture_time_ms;
            car_control_snapshots[snapshot_slot].sequence = result.frame_sequence;
            car_control_snapshots[snapshot_slot].valid = result.line_valid;
            __dsync();
            car_control_snapshot_index = snapshot_slot;
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
        balance_vofa_task();
        car_menu_task();

        // 此处编写需要循环执行的代码
    }
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    static uint32 previous_tick;
    static uint8 late_tick_count;
    static uint32 consumed_visual_sequence;
    uint32 start_ticks = system_getval();
    uint32 elapsed_us;
    uint32 now_ms;
    uint8 snapshot_slot;
    uint8 new_visual_result;
    car_control_snapshot_t visual_snapshot;

    pit_clear_flag(CCU60_CH0);
    if (car_balance_timing_reset != 0) {
        previous_tick = 0;
        late_tick_count = 0;
        car_balance_timing_reset = 0;
    }
    if (previous_tick != 0) {
        car_balance_period_us = (start_ticks - previous_tick) / 100;
        if (car_balance_period_us > CAR_CONTROL_PERIOD_MS * 1200 && balance_state.running != 0) {
            car_balance_overruns++;
            if (++late_tick_count >= 8) {
                balance_state.fault = BALANCE_FAULT_TIMING;
            }
        } else late_tick_count = 0;
    }
    previous_tick = start_ticks;
    car_uptime_ms += CAR_CONTROL_PERIOD_MS;
    __dsync();
    snapshot_slot = car_control_snapshot_index;
    visual_snapshot = car_control_snapshots[snapshot_slot];
    __dsync();
    new_visual_result = visual_snapshot.sequence != consumed_visual_sequence;
    if (new_visual_result != 0) consumed_visual_sequence = visual_snapshot.sequence;
    now_ms = system_getval_ms();
    car_track_update(visual_snapshot.error, visual_snapshot.curvature, visual_snapshot.valid,
                     new_visual_result, visual_snapshot.capture_time_ms, now_ms);
    balance_runtime_update_2ms();
    if (car_running != 0) car_time_ms += CAR_CONTROL_PERIOD_MS;
    else car_time_ms = 0;
    gpio_set_level(BOARD_BUZZER_PIN, car_curve_mode != 0 ? GPIO_HIGH : GPIO_LOW);
    elapsed_us = (system_getval() - start_ticks) / 100;
    if (elapsed_us > car_balance_max_us) car_balance_max_us = elapsed_us;
    if (elapsed_us >= CAR_CONTROL_PERIOD_MS * 1000) {
        car_balance_overruns++;
        balance_state.fault = BALANCE_FAULT_TIMING;
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
