#include "ring_buffer.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_init 功能实现
// 返回类型          void
// 使用示例          cc_ring_buffer_init(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_ring_buffer_init(cc_ring_buffer_t *buffer, uint8_t *storage, uint16_t capacity)
{
    if (buffer == 0) return;
    buffer->data = storage;
    buffer->capacity = capacity;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_clear 功能实现
// 返回类型          void
// 使用示例          cc_ring_buffer_clear(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
void cc_ring_buffer_clear(cc_ring_buffer_t *buffer)
{
    if (buffer == 0) return;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_push 功能实现
// 返回类型          uint8_t
// 使用示例          cc_ring_buffer_push(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_ring_buffer_push(cc_ring_buffer_t *buffer, uint8_t value)
{
    if (buffer == 0 || buffer->data == 0 || buffer->capacity == 0 || buffer->count >= buffer->capacity) return 0;
    buffer->data[buffer->head] = value;
    buffer->head = (uint16_t)((buffer->head + 1u) % buffer->capacity);
    ++buffer->count;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_pop 功能实现
// 返回类型          uint8_t
// 使用示例          cc_ring_buffer_pop(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_ring_buffer_pop(cc_ring_buffer_t *buffer, uint8_t *value)
{
    if (buffer == 0 || value == 0 || buffer->count == 0) return 0;
    *value = buffer->data[buffer->tail];
    buffer->tail = (uint16_t)((buffer->tail + 1u) % buffer->capacity);
    --buffer->count;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_peek 功能实现
// 返回类型          uint8_t
// 使用示例          cc_ring_buffer_peek(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_ring_buffer_peek(const cc_ring_buffer_t *buffer, uint8_t *value)
{
    if (buffer == 0 || value == 0 || buffer->count == 0) return 0;
    *value = buffer->data[buffer->tail];
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_size 功能实现
// 返回类型          uint16_t
// 使用示例          cc_ring_buffer_size(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint16_t cc_ring_buffer_size(const cc_ring_buffer_t *buffer) { return buffer == 0 ? 0 : buffer->count; }
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_free 功能实现
// 返回类型          uint16_t
// 使用示例          cc_ring_buffer_free(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint16_t cc_ring_buffer_free(const cc_ring_buffer_t *buffer) { return buffer == 0 ? 0 : (uint16_t)(buffer->capacity - buffer->count); }
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_is_empty 功能实现
// 返回类型          uint8_t
// 使用示例          cc_ring_buffer_is_empty(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_ring_buffer_is_empty(const cc_ring_buffer_t *buffer) { return (uint8_t)(buffer == 0 || buffer->count == 0); }
//-------------------------------------------------------------------------------------------------------------------
// 函数简介          cc_ring_buffer_is_full 功能实现
// 返回类型          uint8_t
// 使用示例          cc_ring_buffer_is_full(...);
// 备注信息          参数含义请参考同名头文件声明。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_ring_buffer_is_full(const cc_ring_buffer_t *buffer) { return (uint8_t)(buffer != 0 && buffer->capacity != 0 && buffer->count == buffer->capacity); }

