#include "motor2.h"

#include "math_utils.h"

int16_t cc_motor2_output(int16_t command, int16_t pwm_limit, int16_t deadzone, int8_t direction)
{
    int16_t output = (int16_t)cc_math_clamp_i32(command, -pwm_limit, pwm_limit);

    if (output > 0 && output < deadzone) output = deadzone;
    if (output < 0 && output > -deadzone) output = (int16_t)-deadzone;
    return (int16_t)(output * direction);
}

int16_t cc_motor2_slew(int16_t current, int16_t target, int16_t maximum_change)
{
    int16_t difference = (int16_t)(target - current);

    if (maximum_change < 0) maximum_change = (int16_t)-maximum_change;
    if (difference > maximum_change) return (int16_t)(current + maximum_change);
    if (difference < -maximum_change) return (int16_t)(current - maximum_change);
    return target;
}
