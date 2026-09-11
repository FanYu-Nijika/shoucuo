/*
 * board_pins.h
 *
 * 说明：本文件暂以 TC264_主板原理图_V3.1.1.0.pdf 集中记录整车主板的固定引脚。
 *       业务代码和设备驱动只引用这里的名称，换板时只需复核这一处。
 *
 * TC264 two-wheel balance board pin map.
 * Use this file as the only source of board-level pin assignments.
 */
#ifndef CODE_BOARD_PINS_H_
#define CODE_BOARD_PINS_H_

// RGB 接口的红、绿通道暂兼容原状态灯接口。
#define BOARD_LED1_PIN                     (P33_10)
#define BOARD_LED2_PIN                     (P33_11)

// S2-S5 是四个接地有效按键；原理图没有 CENTER 键。
#define BOARD_KEY_UP_PIN                   (P20_6)
#define BOARD_KEY_DOWN_PIN                 (P20_7)
#define BOARD_KEY_LEFT_PIN                 (P11_2)
#define BOARD_KEY_RIGHT_PIN                (P11_3)
#define BOARD_KEY_CENTER_ENABLE            (0)

// 保留旧名称，避免原有测试代码失效。
#define BOARD_KEY1_PIN                     (BOARD_KEY_UP_PIN)
#define BOARD_KEY2_PIN                     (BOARD_KEY_DOWN_PIN)
#define BOARD_SWITCH1_PIN                  (BOARD_KEY_DOWN_PIN)
#define BOARD_SWITCH2_PIN                  (BOARD_KEY_LEFT_PIN)
#define BOARD_SWITCH3_PIN                  (BOARD_KEY_RIGHT_PIN)
#define BOARD_SWITCH4_PIN                  (BOARD_KEY_UP_PIN)
#define BOARD_SWITCH5_PIN                  (BOARD_KEY_LEFT_PIN)

// P11.11 经 R29 驱动 Q2，输出高电平时导通蜂鸣器驱动级。
#define BOARD_BUZZER_PIN                   (P11_11)
#define BOARD_BUZZER_ENABLE                (0)

// 主板八路电机 PWM；平衡车暂用前四路驱动左右轮。
#define BOARD_MOTOR1_PWM_PIN               (ATOM0_CH0_P21_2)
#define BOARD_MOTOR2_PWM_PIN               (ATOM0_CH1_P21_3)
#define BOARD_MOTOR3_PWM_PIN               (ATOM0_CH2_P21_4)
#define BOARD_MOTOR4_PWM_PIN               (ATOM0_CH3_P21_5)
#define BOARD_MOTOR5_PWM_PIN               (ATOM0_CH4_P02_4)
#define BOARD_MOTOR6_PWM_PIN               (ATOM0_CH5_P02_5)
#define BOARD_MOTOR7_PWM_PIN               (ATOM0_CH6_P02_6)
#define BOARD_MOTOR8_PWM_PIN               (ATOM0_CH7_P02_7)
#define BOARD_SERVO_AUX_GPIO_PIN           (P33_9)
#define BOARD_SERVO_PWM_PIN                (ATOM1_CH1_P33_9)
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

// 四个编码器插座按位号定义；平衡车暂用相邻的 P15 作为左轮、P16 作为右轮。
#define BOARD_ENCODER_P13_INDEX            (TIM4_ENCODER)
#define BOARD_ENCODER_P13_CH1              (TIM4_ENCODER_CH1_P02_8)
#define BOARD_ENCODER_P13_CH2              (TIM4_ENCODER_CH2_P00_9)
#define BOARD_ENCODER_P14_INDEX            (TIM2_ENCODER)
#define BOARD_ENCODER_P14_CH1              (TIM2_ENCODER_CH1_P33_7)
#define BOARD_ENCODER_P14_CH2              (TIM2_ENCODER_CH2_P33_6)
#define BOARD_ENCODER_P15_INDEX            (TIM6_ENCODER)
#define BOARD_ENCODER_P15_CH1              (TIM6_ENCODER_CH1_P20_3)
#define BOARD_ENCODER_P15_CH2              (TIM6_ENCODER_CH2_P20_0)
#define BOARD_ENCODER_P16_INDEX            (TIM5_ENCODER)
#define BOARD_ENCODER_P16_CH1              (TIM5_ENCODER_CH1_P10_3)
#define BOARD_ENCODER_P16_CH2              (TIM5_ENCODER_CH2_P10_1)
#define BOARD_LEFT_ENCODER_INDEX           (BOARD_ENCODER_P15_INDEX)
#define BOARD_LEFT_ENCODER_CH1             (BOARD_ENCODER_P15_CH1)
#define BOARD_LEFT_ENCODER_CH2             (BOARD_ENCODER_P15_CH2)
#define BOARD_RIGHT_ENCODER_INDEX          (BOARD_ENCODER_P16_INDEX)
#define BOARD_RIGHT_ENCODER_CH1            (BOARD_ENCODER_P16_CH1)
#define BOARD_RIGHT_ENCODER_CH2            (BOARD_ENCODER_P16_CH2)

#define BOARD_IMU_SPI                     (SPI_0)
#define BOARD_IMU_SCLK_PIN                (SPI0_SCLK_P20_11)
#define BOARD_IMU_MOSI_PIN                (SPI0_MOSI_P20_14)
#define BOARD_IMU_MISO_PIN                (SPI0_MISO_P20_12)
#define BOARD_IMU_CS_PIN                  (P20_13)
#define BOARD_IMU_INT2_PIN                (P15_4)

#define BOARD_ANGLE_ENCODER_SPI            (SPI_0)
#define BOARD_ANGLE_ENCODER_SCLK_PIN       (SPI0_SCLK_P20_11)
#define BOARD_ANGLE_ENCODER_MOSI_PIN       (SPI0_MOSI_P20_14)
#define BOARD_ANGLE_ENCODER_MISO_PIN       (SPI0_MISO_P20_12)
#define BOARD_ANGLE_ENCODER_CS_PIN         (P00_8)

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

// 屏幕没有 MISO，背光直接接 3.3V；P15.4 仅作为硬件 SPI 初始化所需的输入占位。
#define BOARD_DISPLAY_SPI                  (SPI_2)
#define BOARD_DISPLAY_SCL_GPIO_PIN         (P15_3)
#define BOARD_DISPLAY_SDA_GPIO_PIN         (P15_5)
#define BOARD_DISPLAY_SCL_SPI_PIN          (SPI2_SCLK_P15_3)
#define BOARD_DISPLAY_SDA_SPI_PIN          (SPI2_MOSI_P15_5)
#define BOARD_DISPLAY_MISO_SPI_PIN         (SPI2_MISO_P15_4)
#define BOARD_DISPLAY_RESET_PIN            (P15_1)
#define BOARD_DISPLAY_DC_PIN               (P15_0)
#define BOARD_DISPLAY_CS_PIN               (P15_2)
#define BOARD_DISPLAY_BACKLIGHT_PIN        (P20_8)

#define BOARD_HOST_UART                    (UART_3)
#define BOARD_HOST_UART_TX_PIN             (UART3_TX_P15_6)
#define BOARD_HOST_UART_RX_PIN             (UART3_RX_P15_7)

#define BOARD_AUX_UART                     (UART_1)
#define BOARD_AUX_UART_TX_PIN              (UART1_TX_P20_10)
#define BOARD_AUX_UART_RX_PIN              (UART1_RX_P20_9)

// 该插座与逐飞无线串口/WiFi 模块的 TX、RX、RTS、RST 定义一致。
#define BOARD_WIRELESS_UART_TX_PIN         (UART2_TX_P10_5)
#define BOARD_WIRELESS_UART_RX_PIN         (UART2_RX_P10_6)
#define BOARD_WIRELESS_TX_GPIO_PIN         (P10_5)
#define BOARD_WIRELESS_RX_GPIO_PIN         (P10_6)
#define BOARD_WIRELESS_RTS_PIN             (P10_2)
#define BOARD_WIRELESS_RESET_PIN           (P11_6)

#define BOARD_BATTERY_ADC_PIN              (ADC0_CH11_A11)
#define BOARD_ANALOG_AN0_PIN               (ADC0_CH0_A0)

#define BOARD_TOF_SCL_PIN                  (P33_4)
#define BOARD_TOF_SDA_PIN                  (P33_5)
#define BOARD_TOF_IO_PIN                   (P13_0)

#define BOARD_RGB_RED_PIN                  (P33_10)
#define BOARD_RGB_GREEN_PIN                (P33_11)
#define BOARD_RGB_BLUE_PIN                 (P33_12)

#define BOARD_WIRELESS_SPI                 (SPI_3)
#define BOARD_WIRELESS_SPI_SCLK_PIN        (SPI3_SCLK_P22_3)
#define BOARD_WIRELESS_SPI_MOSI_PIN        (SPI3_MOSI_P22_0)
#define BOARD_WIRELESS_SPI_MISO_PIN        (SPI3_MISO_P22_1)
#define BOARD_WIRELESS_SPI_CS_PIN          (P22_2)
#define BOARD_WIRELESS_SPI_RESET_PIN       (P23_1)
#define BOARD_WIRELESS_SPI_INT_PIN         (P15_8)

#endif /* CODE_BOARD_PINS_H_ */
