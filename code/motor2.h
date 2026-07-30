#ifndef CC_MOTOR2_H
#define CC_MOTOR2_H

#include <stdint.h>

int16_t cc_motor2_output(int16_t command, int16_t pwm_limit,
                         int16_t deadzone, int8_t direction);
int16_t cc_motor2_slew(int16_t current, int16_t target,
                       int16_t maximum_change);

#endif
