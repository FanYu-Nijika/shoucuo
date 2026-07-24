#include "math_utils.h"

#include <math.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_clamp_i32 功能实现
// 返回类型          int32_t
// 使用示例          cc_math_clamp_i32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
int32_t cc_math_clamp_i32(int32_t value, int32_t lower, int32_t upper)
{
    if (lower > upper) { int32_t temporary = lower; lower = upper; upper = temporary; }
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_clamp_u32 功能实现
// 返回类型          uint32_t
// 使用示例          cc_math_clamp_u32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint32_t cc_math_clamp_u32(uint32_t value, uint32_t lower, uint32_t upper)
{
    if (lower > upper) { uint32_t temporary = lower; lower = upper; upper = temporary; }
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_clamp_f32 功能实现
// 返回类型          float
// 使用示例          cc_math_clamp_f32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_clamp_f32(float value, float lower, float upper)
{
    if (lower > upper) { float temporary = lower; lower = upper; upper = temporary; }
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_map_i32 功能实现
// 返回类型          int32_t
// 使用示例          cc_math_map_i32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
int32_t cc_math_map_i32(int32_t value, int32_t input_min, int32_t input_max,
                        int32_t output_min, int32_t output_max)
{
    int64_t numerator;
    int64_t denominator;
    if (input_min == input_max) return output_min;
    numerator = ((int64_t)value - input_min) * ((int64_t)output_max - output_min);
    denominator = (int64_t)input_max - input_min;
    return (int32_t)((int64_t)output_min + numerator / denominator);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_map_f32 功能实现
// 返回类型          float
// 使用示例          cc_math_map_f32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_map_f32(float value, float input_min, float input_max,
                      float output_min, float output_max)
{
    if (input_min == input_max) return output_min;
    return output_min + (value - input_min) * (output_max - output_min) / (input_max - input_min);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_deadzone_i32 功能实现
// 返回类型          int32_t
// 使用示例          cc_math_deadzone_i32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
int32_t cc_math_deadzone_i32(int32_t value, int32_t radius)
{
    if (radius < 0) radius = -radius;
    return value > -radius && value < radius ? 0 : value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_deadzone_f32 功能实现
// 返回类型          float
// 使用示例          cc_math_deadzone_f32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_deadzone_f32(float value, float radius)
{
    if (radius < 0.0f) radius = -radius;
    return fabsf(value) < radius ? 0.0f : value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_sign_i32 功能实现
// 返回类型          int32_t
// 使用示例          cc_math_sign_i32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
int32_t cc_math_sign_i32(int32_t value)
{
    if (value > 0) return 1;
    if (value < 0) return -1;
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_round_f32_to_i32 功能实现
// 返回类型          int32_t
// 使用示例          cc_math_round_f32_to_i32(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
int32_t cc_math_round_f32_to_i32(float value)
{
    return value >= 0.0f ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_lerp 功能实现
// 返回类型          float
// 使用示例          cc_math_lerp(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_lerp(float first, float second, float amount)
{
    return first + (second - first) * amount;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_inverse_lerp 功能实现
// 返回类型          float
// 使用示例          cc_math_inverse_lerp(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_inverse_lerp(float first, float second, float value)
{
    if (first == second) return 0.0f;
    return (value - first) / (second - first);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_smoothstep 功能实现
// 返回类型          float
// 使用示例          cc_math_smoothstep(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_smoothstep(float edge0, float edge1, float value)
{
    float amount = cc_math_clamp_f32(cc_math_inverse_lerp(edge0, edge1, value), 0.0f, 1.0f);
    return amount * amount * (3.0f - 2.0f * amount);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_approach 功能实现
// 返回类型          float
// 使用示例          cc_math_approach(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_approach(float current, float target, float step)
{
    float distance = target - current;
    if (step < 0.0f) step = -step;
    if (fabsf(distance) <= step) return target;
    return current + (distance > 0.0f ? step : -step);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_wrap_degrees 功能实现
// 返回类型          float
// 使用示例          cc_math_wrap_degrees(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_wrap_degrees(float degrees)
{
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_wrap_radians 功能实现
// 返回类型          float
// 使用示例          cc_math_wrap_radians(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_wrap_radians(float radians)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;
    while (radians > pi) radians -= two_pi;
    while (radians < -pi) radians += two_pi;
    return radians;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_normalize_angle_error 功能实现
// 返回类型          float
// 使用示例          cc_math_normalize_angle_error(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_normalize_angle_error(float target, float measured)
{
    return cc_math_wrap_degrees(target - measured);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_math_safe_divide 功能实现
// 返回类型          float
// 使用示例          cc_math_safe_divide(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_math_safe_divide(float numerator, float denominator, float fallback)
{
    if (fabsf(denominator) < 0.000001f) return fallback;
    return numerator / denominator;
}

