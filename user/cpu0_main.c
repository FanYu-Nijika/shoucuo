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
#include "image.h"

#include <string.h>

#include "board_pins.h"
#include "car.h"
#include "menu.h"

#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

#define PIT_NUM                 (CCU60_CH0 )

// uint16 delay_time = 0;
// uint8 led_state = 0;

volatile uint32 car_time_ms = 0;

// **************************** 代码区域 ****************************
int core0_main(void)
{
    uint32 menu_time_ms = 0;
    uint32 display_time_ms = 0;
    uint8 camera_init_error;

    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

//    // 板载 LED 为低电平点亮，初始化为高电平可以避免上电误亮。
//    gpio_init(BOARD_LED1_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//    gpio_init(BOARD_LED2_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//    gpio_init(BOARD_KEY1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
//    gpio_init(BOARD_KEY2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
//    gpio_init(BOARD_SWITCH1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
//    gpio_init(BOARD_SWITCH2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    gpio_init(BOARD_LED1_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(BOARD_LED2_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(BOARD_BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    ips200_set_dir(IPS200_CROSSWISE);
    ips200_init(IPS200_TYPE_SPI);

    car_init();
    menu_init();

    camera_init_error = mt9v03x_init();
    car_set_camera_ready(!camera_init_error);
    gpio_set_level(BOARD_LED1_PIN, camera_init_error == 0 ? GPIO_LOW : GPIO_HIGH);
    gpio_set_level(BOARD_LED2_PIN, GPIO_HIGH);

    pit_ms_init(PIT_NUM, 5);

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE) {
        // 此处编写需要循环执行的代码
//
//        delay_time = 300;
//        if(!gpio_get_level(BOARD_SWITCH1_PIN)) delay_time /= 2;
//        if(!gpio_get_level(BOARD_SWITCH2_PIN)) delay_time /= 2;
//        if(!gpio_get_level(BOARD_KEY1_PIN) || !gpio_get_level(BOARD_KEY2_PIN))
//        {
//            gpio_set_level(BOARD_LED1_PIN, led_state);
//            gpio_set_level(BOARD_LED2_PIN, led_state);
//        }
//        else
//        {
//            gpio_set_level(BOARD_LED1_PIN, led_state);
//            gpio_set_level(BOARD_LED2_PIN, !led_state);
//        }
//        led_state = !led_state;
//        system_delay_ms(delay_time);

        if (mt9v03x_finish_flag) {
            mt9v03x_finish_flag = 0;

            // CPU1空闲时复制完整灰度帧，避免DMA采集下一帧时覆盖CPU1正在处理的数据。
            if (!cpu0_done && !cpu1_done) {
                memcpy(image_buffer[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
                __dsync();
                cpu0_done = 1;
                __dsync();
            }
        }

        // CPU1处理完成后，CPU0读取偏差并更新舵机和两个后轮电机。
        if (cpu1_done) {
            __dsync();
            car_track_update(track_error, track_valid);
            cpu1_done = 0;
            __dsync();
        }

        if (car_time_ms - menu_time_ms >= 20) {
            menu_time_ms = car_time_ms;
            menu_task();
        }
        if (car_time_ms - display_time_ms >= 100) {
            display_time_ms = car_time_ms;
            menu_display();
        }

        // 此处编写需要循环执行的代码
    }
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);
    car_time_ms += 5;
}

#pragma section all restore
// **************************** 代码区域 ****************************
