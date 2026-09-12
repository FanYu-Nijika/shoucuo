/*
 * board_pins.h
 *
 * 说明：本文件以 SCH_Schematic1_2026-09-11.pdf 集中记录整车主板的固定引脚。
 *       业务代码和设备驱动只引用这里的名称，换板时只需复核这一处。
 *
 * TC264 two-wheel balance board pin map.
 * Use this file as the only source of board-level pin assignments.
 */
#ifndef CODE_BOARD_PINS_H_
#define CODE_BOARD_PINS_H_

// 原理图“测试灯”LED8、LED9，均为 GPIO 输出低电平点亮。
#define BOARD_LED1_PIN                     (P02_4)
#define BOARD_LED2_PIN                     (P02_5)

// SW7 原厂定义：1=上、2=左、3=下、4=公共端、5=右、6=中心。
// 原理图把 6 脚接地，因此软件将 4 脚持续拉低作为公共端；CENTER 无法读取，按用户要求禁用。
#define BOARD_KEY_UP_PIN                   (P20_6)
#define BOARD_KEY_LEFT_PIN                 (P20_7)
#define BOARD_KEY_DOWN_PIN                 (P20_8)
#define BOARD_KEY_COMMON_PIN               (P20_10)
#define BOARD_KEY_RIGHT_PIN                (P20_9)
#define BOARD_KEY_CENTER_PIN               (BOARD_KEY_COMMON_PIN)
#define BOARD_KEY_CENTER_ENABLE            (0)

// SW2、SW5 是原理图中的两个独立按键。
#define BOARD_KEY1_PIN                     (P22_0)
#define BOARD_KEY2_PIN                     (P22_1)
#define BOARD_SWITCH1_PIN                  (BOARD_KEY_UP_PIN)
#define BOARD_SWITCH2_PIN                  (BOARD_KEY_LEFT_PIN)
#define BOARD_SWITCH3_PIN                  (BOARD_KEY_DOWN_PIN)
#define BOARD_SWITCH4_PIN                  (BOARD_KEY_COMMON_PIN)
#define BOARD_SWITCH5_PIN                  (BOARD_KEY_RIGHT_PIN)

// P23.1 经 R2 驱动 Q2，输出高电平时导通蜂鸣器驱动级。
#define BOARD_BUZZER_PIN                   (P23_1)
#define BOARD_BUZZER_ENABLE                (0)

// U8 缓冲器把 P33.5-P33.8 依次送到 PWM1-PWM4；P33.4 单独输出 PWM_SI。
#define BOARD_MOTOR1_PWM_PIN               (ATOM2_CH1_P33_5)
#define BOARD_MOTOR2_PWM_PIN               (ATOM2_CH2_P33_6)
#define BOARD_MOTOR3_PWM_PIN               (ATOM2_CH3_P33_7)
#define BOARD_MOTOR4_PWM_PIN               (ATOM2_CH4_P33_8)
#define BOARD_SERVO_AUX_GPIO_PIN           (P33_4)
#define BOARD_SERVO_PWM_PIN                (ATOM3_CH0_P33_4)
#define BOARD_LEFT_MOTOR_REVERSE_PWM_PIN   (BOARD_MOTOR1_PWM_PIN)
#define BOARD_LEFT_MOTOR_FORWARD_PWM_PIN   (BOARD_MOTOR2_PWM_PIN)
#define BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN  (BOARD_MOTOR3_PWM_PIN)
#define BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN  (BOARD_MOTOR4_PWM_PIN)

// 轴编号固定为 0=X、1=Y、2=Z；符号用于上车后快速校正安装方向。
#define BOARD_BALANCE_AXIS_X               (0)
#define BOARD_BALANCE_AXIS_Y               (1)
#define BOARD_BALANCE_AXIS_Z               (2)
#define BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS (BOARD_BALANCE_AXIS_X)
#define BOARD_BALANCE_ACCEL_VERTICAL_AXIS  (BOARD_BALANCE_AXIS_Z)
#define BOARD_BALANCE_GYRO_AXIS            (BOARD_BALANCE_AXIS_Y)
#define BOARD_BALANCE_ACCEL_SIGN           (-1)
#define BOARD_BALANCE_ACCEL_VERTICAL_SIGN  (1)
#define BOARD_BALANCE_GYRO_SIGN            (1)
#define BOARD_BALANCE_LEFT_MOTOR_SIGN      (1)
#define BOARD_BALANCE_RIGHT_MOTOR_SIGN     (1)
#define BOARD_BALANCE_LEFT_ENCODER_SIGN    (1)
#define BOARD_BALANCE_RIGHT_ENCODER_SIGN   (1)

// 原理图编码器 H7=P20.3/P20.0，H4=P02.6/P02.7；平衡车按 H7 左、H4 右使用。
#define BOARD_ENCODER_H7_INDEX             (TIM6_ENCODER)
#define BOARD_ENCODER_H7_CH1               (TIM6_ENCODER_CH1_P20_3)
#define BOARD_ENCODER_H7_CH2               (TIM6_ENCODER_CH2_P20_0)
#define BOARD_ENCODER_H4_INDEX             (TIM3_ENCODER)
#define BOARD_ENCODER_H4_CH1               (TIM3_ENCODER_CH1_P02_6)
#define BOARD_ENCODER_H4_CH2               (TIM3_ENCODER_CH2_P02_7)
#define BOARD_LEFT_ENCODER_INDEX           (BOARD_ENCODER_H7_INDEX)
#define BOARD_LEFT_ENCODER_CH1             (BOARD_ENCODER_H7_CH1)
#define BOARD_LEFT_ENCODER_CH2             (BOARD_ENCODER_H7_CH2)
#define BOARD_RIGHT_ENCODER_INDEX          (BOARD_ENCODER_H4_INDEX)
#define BOARD_RIGHT_ENCODER_CH1            (BOARD_ENCODER_H4_CH1)
#define BOARD_RIGHT_ENCODER_CH2            (BOARD_ENCODER_H4_CH2)

#define BOARD_IMU_SPI                     (SPI_0)
#define BOARD_IMU_SCLK_PIN                (SPI0_SCLK_P20_11)
#define BOARD_IMU_MOSI_PIN                (SPI0_MOSI_P20_14)
#define BOARD_IMU_MISO_PIN                (SPI0_MISO_P20_12)
#define BOARD_IMU_CS_PIN                  (P20_13)
#define BOARD_IMU_INT2_PIN                (P15_4)

#define BOARD_CAMERA_ENABLE                (1)
#define BOARD_SERVO_ENABLE                 (0)

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

// 屏幕没有 MISO；P15.7 未连接，仅作为硬件 SPI 初始化所需的输入占位。
#define BOARD_DISPLAY_SPI                  (SPI_2)
#define BOARD_DISPLAY_SCL_GPIO_PIN         (P15_3)
#define BOARD_DISPLAY_SDA_GPIO_PIN         (P15_5)
#define BOARD_DISPLAY_SCL_SPI_PIN          (SPI2_SCLK_P15_3)
#define BOARD_DISPLAY_SDA_SPI_PIN          (SPI2_MOSI_P15_5)
#define BOARD_DISPLAY_MISO_SPI_PIN         (SPI2_MISO_P15_7)
#define BOARD_DISPLAY_RESET_PIN            (P15_1)
#define BOARD_DISPLAY_DC_PIN               (P15_0)
#define BOARD_DISPLAY_CS_PIN               (P15_2)
#define BOARD_DISPLAY_BACKLIGHT_PIN        (P15_6)

#define BOARD_HOST_UART                    (UART_0)
#define BOARD_HOST_UART_TX_PIN             (UART0_TX_P14_0)
#define BOARD_HOST_UART_RX_PIN             (UART0_RX_P14_1)

// 该插座与逐飞无线串口/WiFi 模块的 TX、RX、RTS、RST 定义一致。
#define BOARD_WIRELESS_UART_TX_PIN         (UART2_TX_P10_5)
#define BOARD_WIRELESS_UART_RX_PIN         (UART2_RX_P10_6)
#define BOARD_WIRELESS_TX_GPIO_PIN         (P10_5)
#define BOARD_WIRELESS_RX_GPIO_PIN         (P10_6)
#define BOARD_WIRELESS_RTS_PIN             (P11_12)
#define BOARD_WIRELESS_RESET_PIN           (P11_6)

#define BOARD_BATTERY_ADC_PIN              (ADC1_CH8_A24)
#define BOARD_ANALOG_AN0_PIN               (ADC0_CH0_A0)

// TF-015 SD 卡座使用 SPI0 的这组独立引脚。
#define BOARD_SD_SPI                       (SPI_1)
#define BOARD_SD_SCLK_PIN                  (SPI1_SCLK_P10_2)
#define BOARD_SD_MOSI_PIN                  (SPI1_MOSI_P10_3)
#define BOARD_SD_MISO_PIN                  (SPI1_MISO_P10_1)
#define BOARD_SD_CS_PIN                    (P11_11)

#endif /* CODE_BOARD_PINS_H_ */
