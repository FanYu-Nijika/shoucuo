#include "filters.h"

#include "math_utils.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_moving_average_init 功能实现
// 返回类型          void
// 使用示例          cc_moving_average_init(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_moving_average_init(cc_moving_average_t *filter, size_t capacity, float initial_value)
{
    size_t index;
    if (filter == 0) return;
    if (capacity == 0) capacity = 1;
    if (capacity > CC_FILTER_MAX_SAMPLES) capacity = CC_FILTER_MAX_SAMPLES;
    filter->capacity = capacity;
    filter->count = capacity;
    filter->next_index = 0;
    filter->sum = initial_value * (float)capacity;
    for (index = 0; index < capacity; ++index) filter->samples[index] = initial_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_moving_average_update 功能实现
// 返回类型          float
// 使用示例          cc_moving_average_update(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_moving_average_update(cc_moving_average_t *filter, float input)
{
    if (filter == 0 || filter->capacity == 0) return input;
    filter->sum -= filter->samples[filter->next_index];
    filter->samples[filter->next_index] = input;
    filter->sum += input;
    filter->next_index = (filter->next_index + 1) % filter->capacity;
    if (filter->count < filter->capacity) ++filter->count;
    return filter->sum / (float)filter->count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_moving_average_value 功能实现
// 返回类型          float
// 使用示例          cc_moving_average_value(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_moving_average_value(const cc_moving_average_t *filter)
{
    if (filter == 0 || filter->count == 0) return 0.0f;
    return filter->sum / (float)filter->count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_moving_average_reset 功能实现
// 返回类型          void
// 使用示例          cc_moving_average_reset(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_moving_average_reset(cc_moving_average_t *filter, float value)
{
    size_t index;
    if (filter == 0 || filter->capacity == 0) return;
    filter->sum = value * (float)filter->capacity;
    filter->count = filter->capacity;
    filter->next_index = 0;
    for (index = 0; index < filter->capacity; ++index) filter->samples[index] = value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ema_init 功能实现
// 返回类型          void
// 使用示例          cc_ema_init(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_ema_init(cc_ema_t *filter, float alpha, float initial_value)
{
    if (filter == 0) return;
    filter->alpha = cc_math_clamp_f32(alpha, 0.0f, 1.0f);
    filter->value = initial_value;
    filter->initialized = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ema_update 功能实现
// 返回类型          float
// 使用示例          cc_ema_update(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_ema_update(cc_ema_t *filter, float input)
{
    if (filter == 0) return input;
    if (filter->initialized == 0) {
        filter->value = input;
        filter->initialized = 1;
    } else {
        filter->value += filter->alpha * (input - filter->value);
    }
    return filter->value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ema_reset 功能实现
// 返回类型          void
// 使用示例          cc_ema_reset(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_ema_reset(cc_ema_t *filter, float value)
{
    if (filter == 0) return;
    filter->value = value;
    filter->initialized = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_median3 功能实现
// 返回类型          float
// 使用示例          cc_median3(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_median3(float first, float second, float third)
{
    if (first > second) { float temporary = first; first = second; second = temporary; }
    if (second > third) { float temporary = second; second = third; third = temporary; }
    if (first > second) { float temporary = first; first = second; second = temporary; }
    return second;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_median5 功能实现
// 返回类型          float
// 使用示例          cc_median5(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_median5(float first, float second, float third, float fourth, float fifth)
{
    float values[5] = { first, second, third, fourth, fifth };
    size_t index;
    size_t position;
    float value;
    for (index = 1; index < 5; ++index) {
        value = values[index];
        position = index;
        while (position > 0 && values[position - 1] > value) {
            values[position] = values[position - 1];
            --position;
        }
        values[position] = value;
    }
    return values[2];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_slew_filter_init 功能实现
// 返回类型          void
// 使用示例          cc_slew_filter_init(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_slew_filter_init(cc_slew_filter_t *filter, float rise_per_second,
                         float fall_per_second, float initial_value)
{
    if (filter == 0) return;
    filter->rise_per_second = rise_per_second < 0.0f ? -rise_per_second : rise_per_second;
    filter->fall_per_second = fall_per_second < 0.0f ? -fall_per_second : fall_per_second;
    filter->value = initial_value;
    filter->initialized = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_slew_filter_update 功能实现
// 返回类型          float
// 使用示例          cc_slew_filter_update(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
float cc_slew_filter_update(cc_slew_filter_t *filter, float input, float dt)
{
    float limit;
    if (filter == 0) return input;
    if (filter->initialized == 0) {
        filter->value = input;
        filter->initialized = 1;
        return input;
    }
    if (dt <= 0.0f) return filter->value;
    limit = (input >= filter->value ? filter->rise_per_second : filter->fall_per_second) * dt;
    filter->value = cc_math_approach(filter->value, input, limit);
    return filter->value;
}

