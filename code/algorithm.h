#ifndef CC_ALGORITHM_H
#define CC_ALGORITHM_H

#include <stddef.h>
#include <stdint.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          通用内存操作
// 返回类型          void
// 使用示例          cc_mem_copy(destination, source, sizeof(data));
// 备注信息          只进行字节复制，不依赖 memcpy，适用于裸机工程。
//-------------------------------------------------------------------------------------------------------------------
void cc_mem_swap(void *first, void *second, size_t size);
void cc_mem_copy(void *destination, const void *source, size_t size);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          声明一组指定标量类型的基础算法
// 返回类型          由具体函数决定
// 使用示例          cc_i16_sort_asc(values, count);
// 备注信息          同一组接口覆盖有符号、无符号、char、float、double 等类型。
//-------------------------------------------------------------------------------------------------------------------
#define CC_ALGORITHM_DECLARE(prefix, type, sum_type, average_type) \
type cc_##prefix##_max(type first, type second); \
type cc_##prefix##_min(type first, type second); \
type cc_##prefix##_abs(type value); \
void cc_##prefix##_swap(type *first, type *second); \
void cc_##prefix##_fill(type *array, size_t count, type value); \
void cc_##prefix##_copy(type *destination, const type *source, size_t count); \
void cc_##prefix##_reverse(type *array, size_t count); \
int32_t cc_##prefix##_find(const type *array, size_t count, type value); \
size_t cc_##prefix##_count(const type *array, size_t count, type value); \
int32_t cc_##prefix##_min_index(const type *array, size_t count); \
int32_t cc_##prefix##_max_index(const type *array, size_t count); \
sum_type cc_##prefix##_sum(const type *array, size_t count); \
average_type cc_##prefix##_average(const type *array, size_t count); \
uint8_t cc_##prefix##_is_sorted_asc(const type *array, size_t count); \
uint8_t cc_##prefix##_is_sorted_desc(const type *array, size_t count); \
void cc_##prefix##_sort_asc(type *array, size_t count); \
void cc_##prefix##_sort_desc(type *array, size_t count); \
int32_t cc_##prefix##_lower_bound(const type *array, size_t count, type value); \
int32_t cc_##prefix##_upper_bound(const type *array, size_t count, type value); \
int32_t cc_##prefix##_binary_search(const type *array, size_t count, type value); \
size_t cc_##prefix##_unique(type *array, size_t count); \
void cc_##prefix##_rotate_left(type *array, size_t count, size_t shift); \
void cc_##prefix##_rotate_right(type *array, size_t count, size_t shift)

CC_ALGORITHM_DECLARE(char, char, int64_t, double);
CC_ALGORITHM_DECLARE(schar, signed char, int64_t, double);
CC_ALGORITHM_DECLARE(uchar, unsigned char, uint64_t, double);
CC_ALGORITHM_DECLARE(i8, int8_t, int64_t, double);
CC_ALGORITHM_DECLARE(u8, uint8_t, uint64_t, double);
CC_ALGORITHM_DECLARE(i16, int16_t, int64_t, double);
CC_ALGORITHM_DECLARE(u16, uint16_t, uint64_t, double);
CC_ALGORITHM_DECLARE(i32, int32_t, int64_t, float);
CC_ALGORITHM_DECLARE(u32, uint32_t, uint64_t, float);
CC_ALGORITHM_DECLARE(i64, int64_t, int64_t, double);
CC_ALGORITHM_DECLARE(u64, uint64_t, uint64_t, double);
CC_ALGORITHM_DECLARE(f32, float, float, float);
CC_ALGORITHM_DECLARE(f64, double, double, double);
CC_ALGORITHM_DECLARE(short, short, int64_t, double);
CC_ALGORITHM_DECLARE(ushort, unsigned short, uint64_t, double);
CC_ALGORITHM_DECLARE(int, int, int64_t, double);
CC_ALGORITHM_DECLARE(uint, unsigned int, uint64_t, double);
CC_ALGORITHM_DECLARE(long, long, int64_t, double);
CC_ALGORITHM_DECLARE(ulong, unsigned long, uint64_t, double);
CC_ALGORITHM_DECLARE(ll, long long, int64_t, double);
CC_ALGORITHM_DECLARE(ull, unsigned long long, uint64_t, double);
CC_ALGORITHM_DECLARE(float, float, float, float);
CC_ALGORITHM_DECLARE(double, double, double, double);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          有序 int32_t 数组的扩展算法
// 返回类型          由具体函数决定
// 使用示例          cc_i32_next_permutation(values, count);
// 备注信息          集合运算要求输入数组已经按升序排列。
//-------------------------------------------------------------------------------------------------------------------
void cc_i32_fill_range(int32_t *array, size_t first, size_t last, int32_t value);
void cc_i32_reverse_range(int32_t *array, size_t first, size_t last);
int32_t cc_i32_accumulate(const int32_t *array, size_t count, int32_t initial);
uint8_t cc_i32_next_permutation(int32_t *array, size_t count);
uint8_t cc_i32_prev_permutation(int32_t *array, size_t count);
size_t cc_i32_set_intersection(const int32_t *first, size_t first_count,
                               const int32_t *second, size_t second_count,
                               int32_t *output);
size_t cc_i32_set_union(const int32_t *first, size_t first_count,
                        const int32_t *second, size_t second_count,
                        int32_t *output);
size_t cc_i32_set_difference(const int32_t *first, size_t first_count,
                             const int32_t *second, size_t second_count,
                             int32_t *output);
int32_t cc_i32_gcd(int32_t first, int32_t second);
int32_t cc_i32_lcm(int32_t first, int32_t second);
uint32_t cc_u32_gcd(uint32_t first, uint32_t second);
uint32_t cc_u32_lcm(uint32_t first, uint32_t second);

#endif
