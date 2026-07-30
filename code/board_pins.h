/*
 * board_pins.h
 *
 * 说明：本文件依据 11111.SchDoc 集中记录整车主板的固定引脚。
 *       业务代码和设备驱动只引用这里的名称，换板时只需复核这一处。
 *
 * Camera4 board pin map.
 * Use this file as the only source of board-level pin assignments.
 */
#ifndef CODE_BOARD_PINS_H_
#define CODE_BOARD_PINS_H_

// LED1、LED2 的阳极由 3.3V 供电，MCU 输出低电平时点亮。
#define BOARD_LED1_PIN                     (P02_4)
#define BOARD_LED2_PIN                     (P02_5)

// SW2、SW3 以及五位拨码开关均接地有效，使用时应配置为上拉输入。
#define BOARD_KEY_UP_PIN                   (P11_10)
#define BOARD_KEY_DOWN_PIN                 (P11_11)
#define BOARD_KEY_LEFT_PIN                 (P11_9)
#define BOARD_KEY_RIGHT_PIN                (P13_3)
#define BOARD_KEY_CENTER_PIN               (P11_12)
#define BOARD_KEY_AUX1_PIN                 (P11_2)
#define BOARD_KEY_AUX2_PIN                 (P11_3)

// 保留旧名称，避免原有测试代码失效。
#define BOARD_KEY1_PIN                     (BOARD_KEY_AUX1_PIN)
#define BOARD_KEY2_PIN                     (BOARD_KEY_AUX2_PIN)
#define BOARD_SWITCH1_PIN                  (BOARD_KEY_DOWN_PIN)
#define BOARD_SWITCH2_PIN                  (BOARD_KEY_CENTER_PIN)
#define BOARD_SWITCH3_PIN                  (BOARD_KEY_RIGHT_PIN)
#define BOARD_SWITCH4_PIN                  (BOARD_KEY_UP_PIN)
#define BOARD_SWITCH5_PIN                  (BOARD_KEY_LEFT_PIN)

// P13.0 经 R12 驱动 Q1，输出高电平时导通蜂鸣器驱动级。
#define BOARD_BUZZER_PIN                   (P13_0)

// 五路信号均先经过 74LV245 缓冲，再送往舵机插座。
// 当前四路P21.2-P21.5用于两个后轮电机正反转PWM，P21.6用于前轮舵机PWM。
#define BOARD_SERVO1_GPIO_PIN              (P21_2)
#define BOARD_SERVO2_GPIO_PIN              (P21_3)
#define BOARD_SERVO3_GPIO_PIN              (P21_4)
#define BOARD_SERVO4_GPIO_PIN              (P21_5)
#define BOARD_SERVO_AUX_GPIO_PIN           (P21_6)
#define BOARD_SERVO1_PWM_PIN               (ATOM0_CH0_P21_2)
#define BOARD_SERVO2_PWM_PIN               (ATOM0_CH1_P21_3)
#define BOARD_SERVO3_PWM_PIN               (ATOM0_CH2_P21_4)
#define BOARD_SERVO4_PWM_PIN               (ATOM0_CH3_P21_5)
#define BOARD_SERVO_AUX_PWM_PIN            (ATOM0_CH4_P21_6)

#define BOARD_SERVO_PWM_PIN                (BOARD_SERVO_AUX_PWM_PIN)
#define BOARD_LEFT_MOTOR_REVERSE_PWM_PIN   (BOARD_SERVO1_PWM_PIN)
#define BOARD_LEFT_MOTOR_FORWARD_PWM_PIN   (BOARD_SERVO2_PWM_PIN)
#define BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN  (BOARD_SERVO3_PWM_PIN)
#define BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN  (BOARD_SERVO4_PWM_PIN)

#define BOARD_CAMERA_ENABLE                (1)

// D0-D7 必须占用同一 GPIO 端口内连续的八位，以便 DMA 一次读取完整像素。
#define BOARD_CAMERA_D0_PIN                (P00_0)
#define BOARD_CAMERA_D1_PIN                (P00_1)
#define BOARD_CAMERA_D2_PIN                (P00_2)
#define BOARD_CAMERA_D3_PIN                (P00_3)
#define BOARD_CAMERA_D4_PIN                (P00_4)
#define BOARD_CAMERA_D5_PIN                (P00_5)
#define BOARD_CAMERA_D6_PIN                (P00_6)
#define BOARD_CAMERA_D7_PIN                (P00_7)
#define BOARD_CAMERA_DATA_PIN              (BOARD_CAMERA_D0_PIN)
#define BOARD_CAMERA_PCLK_PIN              (ERU_CH2_REQ14_P02_1)
#define BOARD_CAMERA_VSYNC_PIN             (ERU_CH3_REQ6_P02_0)
#define BOARD_CAMERA_CONFIG_SCL_PIN        (P02_3)
#define BOARD_CAMERA_CONFIG_SDA_PIN        (P02_2)
#define BOARD_CAMERA_CONFIG_UART_TX_PIN    (UART1_RX_P02_3)
#define BOARD_CAMERA_CONFIG_UART_RX_PIN    (UART1_TX_P02_2)

// 屏幕没有 MISO；P15.4 仅作为硬件 SPI 初始化所需的占位输入，不接屏幕插座。
#define BOARD_DISPLAY_SPI                  (SPI_2)
#define BOARD_DISPLAY_SCL_GPIO_PIN         (P15_3)
#define BOARD_DISPLAY_SDA_GPIO_PIN         (P15_5)
#define BOARD_DISPLAY_SCL_SPI_PIN          (SPI2_SCLK_P15_3)
#define BOARD_DISPLAY_SDA_SPI_PIN          (SPI2_MOSI_P15_5)
#define BOARD_DISPLAY_MISO_SPI_PIN         (SPI2_MISO_P15_4)
#define BOARD_DISPLAY_RESET_PIN            (P15_1)
#define BOARD_DISPLAY_DC_PIN               (P15_0)
#define BOARD_DISPLAY_CS_PIN               (P15_2)
#define BOARD_DISPLAY_BACKLIGHT_PIN        (P20_14)

#define BOARD_HOST_UART                    (UART_0)
#define BOARD_HOST_UART_TX_PIN             (UART0_TX_P14_0)
#define BOARD_HOST_UART_RX_PIN             (UART0_RX_P14_1)

// 该插座与逐飞无线串口/WiFi 模块的 TX、RX、RTS、RST 定义一致。
#define BOARD_WIRELESS_UART_TX_PIN         (UART2_TX_P10_5)
#define BOARD_WIRELESS_UART_RX_PIN         (UART2_RX_P10_6)
#define BOARD_WIRELESS_TX_GPIO_PIN         (P10_5)
#define BOARD_WIRELESS_RX_GPIO_PIN         (P10_6)
#define BOARD_WIRELESS_RTS_PIN             (P10_2)
#define BOARD_WIRELESS_RESET_PIN           (P11_6)

#define BOARD_ANALOG_AN0_PIN               (ADC0_CH0_A0)

#endif /* CODE_BOARD_PINS_H_ */
