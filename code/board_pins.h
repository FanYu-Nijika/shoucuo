/*
 * Camera4 board pin map.
 * Use this file as the only source of board-level pin assignments.
 */
#ifndef CC_BOARD_PINS_H
#define CC_BOARD_PINS_H

#define BOARD_LED1_PIN                     (P02_4)
#define BOARD_LED2_PIN                     (P02_5)

#define BOARD_KEY_UP_PIN                   (P11_10)
#define BOARD_KEY_DOWN_PIN                 (P11_11)
#define BOARD_KEY_LEFT_PIN                 (P11_9)
#define BOARD_KEY_RIGHT_PIN                (P13_3)
#define BOARD_KEY_CENTER_PIN               (P11_12)
#define BOARD_KEY_AUX1_PIN                 (P11_2)
#define BOARD_KEY_AUX2_PIN                 (P11_3)

#define BOARD_BUZZER_PIN                   (P13_0)

#define BOARD_SERVO_PWM_PIN                (ATOM0_CH4_P21_6)
#define BOARD_LEFT_MOTOR_FORWARD_PWM_PIN   (ATOM0_CH0_P21_2)
#define BOARD_LEFT_MOTOR_REVERSE_PWM_PIN   (ATOM0_CH1_P21_3)
#define BOARD_RIGHT_MOTOR_FORWARD_PWM_PIN  (ATOM0_CH2_P21_4)
#define BOARD_RIGHT_MOTOR_REVERSE_PWM_PIN  (ATOM0_CH3_P21_5)

#define BOARD_CAMERA_ENABLE                (1U)
#define BOARD_CAMERA_DATA_PIN              (P00_0)
#define BOARD_CAMERA_PCLK_PIN              (ERU_CH2_REQ14_P02_1)
#define BOARD_CAMERA_VSYNC_PIN             (ERU_CH3_REQ6_P02_0)
#define BOARD_CAMERA_CONFIG_SCL_PIN        (P02_3)
#define BOARD_CAMERA_CONFIG_SDA_PIN        (P02_2)
#define BOARD_CAMERA_CONFIG_UART_TX_PIN    (UART1_RX_P02_3)
#define BOARD_CAMERA_CONFIG_UART_RX_PIN    (UART1_TX_P02_2)

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
#define BOARD_WIRELESS_UART_TX_PIN         (UART2_TX_P10_5)
#define BOARD_WIRELESS_UART_RX_PIN         (UART2_RX_P10_6)
#define BOARD_WIRELESS_RTS_PIN             (P10_2)
#define BOARD_WIRELESS_RESET_PIN           (P11_6)

#define BOARD_ANALOG_AN0_PIN               (ADC0_CH0_A0)

#endif
