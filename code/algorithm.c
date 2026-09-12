#include "algorithm.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          交换任意内存区域
// 返回类型          void
// 使用示例          cc_mem_swap(first, second, sizeof(value));
// 备注信息          空指针或相同地址直接返回，避免破坏调用方数据。
//-------------------------------------------------------------------------------------------------------------------
void cc_mem_swap(void *first, void *second, size_t size)
{
    uint8_t *left = (uint8_t *)first;
    uint8_t *right = (uint8_t *)second;
    size_t index;
    uint8_t temporary;

    if (left == 0 || right == 0 || left == right) return;
    for (index = 0; index < size; ++index) {
        temporary = left[index];
        left[index] = right[index];
        right[index] = temporary;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          复制任意内存区域
// 返回类型          void
// 使用示例          cc_mem_copy(destination, source, sizeof(data));
// 备注信息          不调用标准库 memcpy，避免在裸机工程中引入额外运行库依赖。
//-------------------------------------------------------------------------------------------------------------------
void cc_mem_copy(void *destination, const void *source, size_t size)
{
    uint8_t *dst = (uint8_t *)destination;
    const uint8_t *src = (const uint8_t *)source;
    size_t index;

    if (dst == 0 || src == 0 || dst == src) return;
    for (index = 0; index < size; ++index) dst[index] = src[index];
}

/*
 * 这一组宏只生成普通的、无函数指针的 C 函数。调用方通过前缀选择类型，
 * 例如 cc_i16_sort_asc、cc_u64_sum、cc_f64_average，避免把算法限制在 32 位。
 */
#define CC_ALGORITHM_DEFINE(prefix, type, sum_type, average_type, abs_expression) \
type cc_##prefix##_max(type first, type second) \
{ return first > second ? first : second; } \
type cc_##prefix##_min(type first, type second) \
{ return first < second ? first : second; } \
type cc_##prefix##_abs(type value) \
{ return abs_expression; } \
void cc_##prefix##_swap(type *first, type *second) \
{ type temporary; if (first == 0 || second == 0 || first == second) return; temporary = *first; *first = *second; *second = temporary; } \
void cc_##prefix##_fill(type *array, size_t count, type value) \
{ size_t index; if (array == 0) return; for (index = 0; index < count; ++index) array[index] = value; } \
void cc_##prefix##_copy(type *destination, const type *source, size_t count) \
{ size_t index; if (destination == 0 || source == 0 || destination == source) return; for (index = 0; index < count; ++index) destination[index] = source[index]; } \
void cc_##prefix##_reverse(type *array, size_t count) \
{ size_t left = 0; size_t right; type temporary; if (array == 0 || count == 0) return; right = count - 1; while (left < right) { temporary = array[left]; array[left] = array[right]; array[right] = temporary; ++left; --right; } } \
int32_t cc_##prefix##_find(const type *array, size_t count, type value) \
{ size_t index; if (array == 0) return -1; for (index = 0; index < count; ++index) if (array[index] == value) return (int32_t)index; return -1; } \
size_t cc_##prefix##_count(const type *array, size_t count, type value) \
{ size_t index; size_t result = 0; if (array == 0) return 0; for (index = 0; index < count; ++index) if (array[index] == value) ++result; return result; } \
int32_t cc_##prefix##_min_index(const type *array, size_t count) \
{ size_t index; size_t result = 0; if (array == 0 || count == 0) return -1; for (index = 1; index < count; ++index) if (array[index] < array[result]) result = index; return (int32_t)result; } \
int32_t cc_##prefix##_max_index(const type *array, size_t count) \
{ size_t index; size_t result = 0; if (array == 0 || count == 0) return -1; for (index = 1; index < count; ++index) if (array[index] > array[result]) result = index; return (int32_t)result; } \
sum_type cc_##prefix##_sum(const type *array, size_t count) \
{ size_t index; sum_type result = (sum_type)0; if (array == 0) return result; for (index = 0; index < count; ++index) result += array[index]; return result; } \
average_type cc_##prefix##_average(const type *array, size_t count) \
{ if (array == 0 || count == 0) return (average_type)0; return (average_type)(cc_##prefix##_sum(array, count) / (average_type)count); } \
uint8_t cc_##prefix##_is_sorted_asc(const type *array, size_t count) \
{ size_t index; if (array == 0) return 0; for (index = 1; index < count; ++index) if (array[index] < array[index - 1]) return 0; return 1; } \
uint8_t cc_##prefix##_is_sorted_desc(const type *array, size_t count) \
{ size_t index; if (array == 0) return 0; for (index = 1; index < count; ++index) if (array[index] > array[index - 1]) return 0; return 1; } \
void cc_##prefix##_sort_asc(type *array, size_t count) \
{ size_t index; size_t position; type value; if (array == 0) return; for (index = 1; index < count; ++index) { value = array[index]; position = index; while (position > 0 && array[position - 1] > value) { array[position] = array[position - 1]; --position; } array[position] = value; } } \
void cc_##prefix##_sort_desc(type *array, size_t count) \
{ size_t index; size_t position; type value; if (array == 0) return; for (index = 1; index < count; ++index) { value = array[index]; position = index; while (position > 0 && array[position - 1] < value) { array[position] = array[position - 1]; --position; } array[position] = value; } } \
int32_t cc_##prefix##_lower_bound(const type *array, size_t count, type value) \
{ size_t first = 0; size_t last = count; size_t middle; if (array == 0) return -1; while (first < last) { middle = first + (last - first) / 2; if (array[middle] < value) first = middle + 1; else last = middle; } return (int32_t)first; } \
int32_t cc_##prefix##_upper_bound(const type *array, size_t count, type value) \
{ size_t first = 0; size_t last = count; size_t middle; if (array == 0) return -1; while (first < last) { middle = first + (last - first) / 2; if (array[middle] <= value) first = middle + 1; else last = middle; } return (int32_t)first; } \
int32_t cc_##prefix##_binary_search(const type *array, size_t count, type value) \
{ int32_t index = cc_##prefix##_lower_bound(array, count, value); if (index < 0 || (size_t)index >= count || array[index] != value) return -1; return index; } \
size_t cc_##prefix##_unique(type *array, size_t count) \
{ size_t read_index; size_t write_index; if (array == 0 || count == 0) return 0; write_index = 1; for (read_index = 1; read_index < count; ++read_index) if (array[read_index] != array[write_index - 1]) array[write_index++] = array[read_index]; return write_index; } \
void cc_##prefix##_rotate_left(type *array, size_t count, size_t shift) \
{ size_t left; size_t right; type temporary; if (array == 0 || count == 0) return; shift %= count; if (shift == 0) return; left = 0; right = shift - 1; while (left < right) { temporary = array[left]; array[left] = array[right]; array[right] = temporary; ++left; --right; } left = shift; right = count - 1; while (left < right) { temporary = array[left]; array[left] = array[right]; array[right] = temporary; ++left; --right; } left = 0; right = count - 1; while (left < right) { temporary = array[left]; array[left] = array[right]; array[right] = temporary; ++left; --right; } } \
void cc_##prefix##_rotate_right(type *array, size_t count, size_t shift) \
{ if (array == 0 || count == 0) return; shift %= count; if (shift == 0) return; cc_##prefix##_rotate_left(array, count, count - shift); }

CC_ALGORITHM_DEFINE(char, char, int64_t, double, value < (char)0 ? (char)-value : value)
CC_ALGORITHM_DEFINE(schar, signed char, int64_t, double, value < (signed char)0 ? (signed char)-value : value)
CC_ALGORITHM_DEFINE(uchar, unsigned char, uint64_t, double, value)
CC_ALGORITHM_DEFINE(i8, int8_t, int64_t, double, value < (int8_t)0 ? (int8_t)-value : value)
CC_ALGORITHM_DEFINE(u8, uint8_t, uint64_t, double, value)
CC_ALGORITHM_DEFINE(i16, int16_t, int64_t, double, value < (int16_t)0 ? (int16_t)-value : value)
CC_ALGORITHM_DEFINE(u16, uint16_t, uint64_t, double, value)
CC_ALGORITHM_DEFINE(i32, int32_t, int64_t, float, value < (int32_t)0 ? (int32_t)-value : value)
CC_ALGORITHM_DEFINE(u32, uint32_t, uint64_t, float, value)
CC_ALGORITHM_DEFINE(i64, int64_t, int64_t, double, value < (int64_t)0 ? (int64_t)-value : value)
CC_ALGORITHM_DEFINE(u64, uint64_t, uint64_t, double, value)
CC_ALGORITHM_DEFINE(f32, float, float, float, value < 0.0 ? -value : value)
CC_ALGORITHM_DEFINE(f64, double, double, double, value < 0.0 ? -value : value)
CC_ALGORITHM_DEFINE(short, short, int64_t, double, value < (short)0 ? (short)-value : value)
CC_ALGORITHM_DEFINE(ushort, unsigned short, uint64_t, double, value)
CC_ALGORITHM_DEFINE(int, int, int64_t, double, value < 0 ? -value : value)
CC_ALGORITHM_DEFINE(uint, unsigned int, uint64_t, double, value)
CC_ALGORITHM_DEFINE(long, long, int64_t, double, value < 0L ? -value : value)
CC_ALGORITHM_DEFINE(ulong, unsigned long, uint64_t, double, value)
CC_ALGORITHM_DEFINE(ll, long long, int64_t, double, value < 0LL ? -value : value)
CC_ALGORITHM_DEFINE(ull, unsigned long long, uint64_t, double, value)
CC_ALGORITHM_DEFINE(float, float, float, float, value < 0.0 ? -value : value)
CC_ALGORITHM_DEFINE(double, double, double, double, value < 0.0 ? -value : value)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          填充 int32_t 数组的半开区间
// 返回类型          void
// 使用示例          cc_i32_fill_range(values, 0, 4, 0);
// 备注信息          区间为 [first, last)，last 不参与写入。
//-------------------------------------------------------------------------------------------------------------------
void cc_i32_fill_range(int32_t *array, size_t first, size_t last, int32_t value)
{
    size_t index;
    if (array == 0) return;
    for (index = first; index < last; ++index) array[index] = value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          反转 int32_t 数组的半开区间
// 返回类型          void
// 使用示例          cc_i32_reverse_range(values, 2, 6);
// 备注信息          区间为 [first, last)。
//-------------------------------------------------------------------------------------------------------------------
void cc_i32_reverse_range(int32_t *array, size_t first, size_t last)
{
    int32_t temporary;
    if (array == 0 || first >= last) return;
    --last;
    while (first < last) {
        temporary = array[first]; array[first] = array[last]; array[last] = temporary;
        ++first; --last;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          计算 int32_t 数组的累加值
// 返回类型          int32_t
// 使用示例          total = cc_i32_accumulate(values, count, 0);
// 备注信息          累加溢出遵循目标编译器的 int32_t 运算规则。
//-------------------------------------------------------------------------------------------------------------------
int32_t cc_i32_accumulate(const int32_t *array, size_t count, int32_t initial)
{
    size_t index;
    int32_t result = initial;
    if (array == 0) return initial;
    for (index = 0; index < count; ++index) result += array[index];
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          生成 int32_t 数组的下一个排列
// 返回类型          uint8_t
// 使用示例          has_next = cc_i32_next_permutation(values, count);
// 备注信息          输入数组应为升序或已经处于某个排列状态。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_i32_next_permutation(int32_t *array, size_t count)
{
    size_t first; size_t second; int32_t temporary;
    if (array == 0 || count < 2) return 0;
    first = count - 2;
    while (array[first] >= array[first + 1]) { if (first == 0) { cc_i32_reverse(array, count); return 0; } --first; }
    second = count - 1;
    while (array[second] <= array[first]) --second;
    temporary = array[first]; array[first] = array[second]; array[second] = temporary;
    cc_i32_reverse_range(array, first + 1, count);
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          生成 int32_t 数组的上一个排列
// 返回类型          uint8_t
// 使用示例          has_previous = cc_i32_prev_permutation(values, count);
// 备注信息          已经是最小排列时会反转为最大排列并返回 0。
//-------------------------------------------------------------------------------------------------------------------
uint8_t cc_i32_prev_permutation(int32_t *array, size_t count)
{
    size_t first; size_t second; int32_t temporary;
    if (array == 0 || count < 2) return 0;
    first = count - 2;
    while (array[first] <= array[first + 1]) { if (first == 0) { cc_i32_reverse(array, count); return 0; } --first; }
    second = count - 1;
    while (array[second] >= array[first]) --second;
    temporary = array[first]; array[first] = array[second]; array[second] = temporary;
    cc_i32_reverse_range(array, first + 1, count);
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          计算两个有序 int32_t 数组的交集
// 返回类型          size_t
// 使用示例          size = cc_i32_set_intersection(first, first_count, second, second_count, output);
// 备注信息          output 需要由调用方提供足够空间，结果保持升序且不重复。
//-------------------------------------------------------------------------------------------------------------------
size_t cc_i32_set_intersection(const int32_t *first, size_t first_count,
                               const int32_t *second, size_t second_count,
                               int32_t *output)
{
    size_t a = 0; size_t b = 0; size_t out = 0;
    if (first == 0 || second == 0 || output == 0) return 0;
    while (a < first_count && b < second_count) {
        if (first[a] < second[b]) ++a;
        else if (second[b] < first[a]) ++b;
        else { output[out++] = first[a]; ++a; ++b; }
    }
    return out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          计算两个有序 int32_t 数组的并集
// 返回类型          size_t
// 使用示例          size = cc_i32_set_union(first, first_count, second, second_count, output);
// 备注信息          output 需要由调用方提供足够空间，结果保持升序且不重复。
//-------------------------------------------------------------------------------------------------------------------
size_t cc_i32_set_union(const int32_t *first, size_t first_count,
                        const int32_t *second, size_t second_count,
                        int32_t *output)
{
    size_t a = 0; size_t b = 0; size_t out = 0;
    if (first == 0 || second == 0 || output == 0) return 0;
    while (a < first_count && b < second_count) {
        if (first[a] < second[b]) output[out++] = first[a++];
        else if (second[b] < first[a]) output[out++] = second[b++];
        else { output[out++] = first[a]; ++a; ++b; }
    }
    while (a < first_count) output[out++] = first[a++];
    while (b < second_count) output[out++] = second[b++];
    return out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          计算有序 int32_t 数组的差集
// 返回类型          size_t
// 使用示例          size = cc_i32_set_difference(first, first_count, second, second_count, output);
// 备注信息          结果为 first 中不存在于 second 的元素，输入必须升序。
//-------------------------------------------------------------------------------------------------------------------
size_t cc_i32_set_difference(const int32_t *first, size_t first_count,
                             const int32_t *second, size_t second_count,
                             int32_t *output)
{
    size_t a = 0; size_t b = 0; size_t out = 0;
    if (first == 0 || second == 0 || output == 0) return 0;
    while (a < first_count && b < second_count) {
        if (first[a] < second[b]) output[out++] = first[a++];
        else if (second[b] < first[a]) ++b;
        else { ++a; ++b; }
    }
    while (a < first_count) output[out++] = first[a++];
    return out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          计算 int32_t 最大公约数
// 返回类型          int32_t
// 使用示例          gcd = cc_i32_gcd(first, second);
// 备注信息          参数符号不影响结果。
//-------------------------------------------------------------------------------------------------------------------
int32_t cc_i32_gcd(int32_t first, int32_t second)
{
    int32_t temporary;
    first = cc_i32_abs(first); second = cc_i32_abs(second);
    while (second != 0) { temporary = first % second; first = second; second = temporary; }
    return first;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          计算 int32_t 最小公倍数
// 返回类型          int32_t
// 使用示例          lcm = cc_i32_lcm(first, second);
// 备注信息          任一参数为 0 时返回 0。
//-------------------------------------------------------------------------------------------------------------------
int32_t cc_i32_lcm(int32_t first, int32_t second)
{
    int32_t gcd;
    if (first == 0 || second == 0) return 0;
    gcd = cc_i32_gcd(first, second);
    return cc_i32_abs((first / gcd) * second);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          计算 uint32_t 最大公约数
// 返回类型          uint32_t
// 使用示例          gcd = cc_u32_gcd(first, second);
// 备注信息          使用欧几里得算法，不需要额外内存。
//-------------------------------------------------------------------------------------------------------------------
uint32_t cc_u32_gcd(uint32_t first, uint32_t second)
{
    uint32_t temporary;
    while (second != 0) { temporary = first % second; first = second; second = temporary; }
    return first;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介          计算 uint32_t 最小公倍数
// 返回类型          uint32_t
// 使用示例          lcm = cc_u32_lcm(first, second);
// 备注信息          任一参数为 0 时返回 0。
//-------------------------------------------------------------------------------------------------------------------
uint32_t cc_u32_lcm(uint32_t first, uint32_t second)
{
    uint32_t gcd;
    if (first == 0 || second == 0) return 0;
    gcd = cc_u32_gcd(first, second);
    return (first / gcd) * second;
}
