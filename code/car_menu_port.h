#ifndef CC_CAR_MENU_PORT_H
#define CC_CAR_MENU_PORT_H

#include <stdint.h>

enum {
    CC_KEY_UP_MASK = 0x01,
    CC_KEY_DOWN_MASK = 0x02,
    CC_KEY_LEFT_MASK = 0x04,
    CC_KEY_RIGHT_MASK = 0x08,
    CC_KEY_CENTER_MASK = 0x10
};

void cc_tc264_board_init(void);
uint8_t cc_tc264_menu_key_mask(void);

uint8_t cc_tc264_camera_init(void);
uint8_t cc_tc264_camera_ready(void);
uint8_t cc_tc264_camera_set_exposure(uint16_t exposure);
uint8_t cc_tc264_camera_set_gain(uint8_t gain);

uint8_t cc_tc264_motor_ready(void);
void cc_tc264_menu_motor_write(int16_t left, int16_t right);
void cc_tc264_menu_motor_stop(void);
void cc_tc264_menu_servo_write_us(int16_t pulse_us);

void cc_tc264_balance_encoder_init(void);
void cc_tc264_balance_encoder_read(int16_t *left, int16_t *right);

#endif
