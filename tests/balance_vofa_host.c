#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "balance_control.h"
#include "balance_vofa.h"
#include "balance_vofa_host_stubs.h"

#define HOST_FIFO_CAPACITY 256
#define CHANNEL_COUNT 22
#define CAPTURE_CAPACITY 1024

flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];
uint32 host_time_ms;
uint8 host_interrupts_enabled = 1;

static uint8 host_fifo[HOST_FIFO_CAPACITY];
static uint32 host_fifo_read;
static uint32 host_fifo_write;
static uint8 host_image_stream_enabled;
static char captured_line[CAPTURE_CAPACITY];
static unsigned captured_printf_count;
static uint8 printf_saw_interrupts_enabled;
static const double expected_speed_channels[CHANNEL_COUNT] = {
    -0.5, -0.25, -0.25, -1.5, -2.25, -4.1, 20.0, -0.375, -0.2, -0.3, -0.9,
    -0.125, -12.34, 321.0, -654.0, -2.0, 3.0, -1234.0, 2345.0, 1.0, 1.0, BALANCE_FAULT_NUMERIC
};

uint32 interrupt_global_disable(void)
{
    uint32 previous = host_interrupts_enabled;
    host_interrupts_enabled = 0;
    return previous;
}

void interrupt_global_enable(uint32 state)
{
    host_interrupts_enabled = (uint8)state;
}

void flash_read_page_to_buffer(uint32 sector, uint32 page)
{
    (void)sector;
    (void)page;
}

uint8 flash_write_page_from_buffer(uint32 sector, uint32 page)
{
    (void)sector;
    (void)page;
    return 0;
}

uint32 system_getval_ms(void)
{
    return host_time_ms;
}

uint32 debug_read_ring_buffer(uint8 *buffer, uint32 length)
{
    uint32 copied = 0;

    while (copied < length && host_fifo_read < host_fifo_write) {
        buffer[copied++] = host_fifo[host_fifo_read++ % HOST_FIFO_CAPACITY];
    }
    if (host_fifo_read == host_fifo_write) host_fifo_read = host_fifo_write = 0;
    return copied;
}

uint8 car_uart_stream_is_enabled(void)
{
    return host_image_stream_enabled;
}

int __wrap_printf(const char *format, ...)
{
    va_list args;
    int length;

    printf_saw_interrupts_enabled = host_interrupts_enabled;
    va_start(args, format);
    length = vsnprintf(captured_line, sizeof(captured_line), format, args);
    va_end(args);
    assert(length >= 0 && (size_t)length < sizeof(captured_line));
    captured_printf_count++;
    return length;
}

static void host_fifo_push(const char *text)
{
    while (*text != '\0') {
        assert(host_fifo_write - host_fifo_read < HOST_FIFO_CAPACITY);
        host_fifo[host_fifo_write++ % HOST_FIFO_CAPACITY] = (uint8)*text++;
    }
}

static void set_sample_state(void)
{
    balance_state.speed_target_applied_mps = -0.5f;
    balance_state.speed_raw_mps = -0.375f;
    balance_state.speed_filtered_mps = -0.25f;
    balance_state.speed_feedback_mps = -0.2f;
    balance_state.speed_error_mps = -0.3f;
    balance_state.speed_p_angle = -0.9f;
    balance_state.speed_integral_angle = -0.125f;
    balance_state.speed_angle_applied = -0.25f;
    balance_state.target_angle = -1.5f;
    balance_state.angle = -2.25f;
    balance_state.balance_output = -4.1f;
    balance_state.gyro_rate = -12.34f;
    balance_state.turn_pwm_command = 321.0f;
    balance_state.turn_output = -654.0f;
    balance_state.left_count = -2;
    balance_state.right_count = 3;
    balance_state.left_pwm = -1234;
    balance_state.right_pwm = 2345;
    balance_state.speed_pid_active = 1;
    balance_state.running = 1;
    balance_state.fault = BALANCE_FAULT_NUMERIC;
}

static double channel_value(unsigned channel)
{
    char *text = captured_line;
    char *end;
    unsigned index;
    double value;

    assert(strncmp(text, "speed:", 6) == 0);
    text += 6;
    for (index = 0; index < channel; index++) {
        text = strchr(text, ',');
        assert(text != NULL);
        text++;
    }
    value = strtod(text, &end);
    assert(end != text);
    if (channel < CHANNEL_COUNT - 1) assert(*end == ',');
    else assert(*end == '\n' && end[1] == '\0');
    return value;
}

static void test_firewater_channels(void)
{
    unsigned index;
    unsigned commas = 0;

    balance_init();
    set_sample_state();
    host_time_ms = 0;
    host_fifo_push("startup");
    balance_vofa_task();
    assert(captured_printf_count == 0);
    assert(host_fifo_read == host_fifo_write);

    host_time_ms = 19;
    balance_vofa_task();
    assert(captured_printf_count == 0);
    host_time_ms = 20;
    balance_vofa_task();
    assert(captured_printf_count == 1);
    assert(printf_saw_interrupts_enabled);
    for (index = 0; captured_line[index] != '\0'; index++) if (captured_line[index] == ',') commas++;
    assert(commas == CHANNEL_COUNT - 1);
    assert(captured_line[strlen(captured_line) - 1] == '\n');
    for (index = 0; index < CHANNEL_COUNT; index++) if (index != 6) assert(fabs(channel_value(index) - expected_speed_channels[index]) < 0.001);
    assert(channel_value(6) == 20.0);
    host_time_ms = 39;
    balance_vofa_task();
    assert(captured_printf_count == 1);
    host_time_ms = 40;
    balance_vofa_task();
    assert(captured_printf_count == 2);
    assert(channel_value(6) == 40.0);
    assert(printf_saw_interrupts_enabled);
}

static void test_image_stream_pause_and_recovery(void)
{
    unsigned index;

    host_image_stream_enabled = 1;
    host_time_ms = 41;
    host_fifo_push("discard while image stream is active\n");
    balance_vofa_task();
    assert(captured_printf_count == 2);
    assert(host_fifo_read == host_fifo_write);

    host_time_ms = 1000;
    balance_vofa_task();
    assert(captured_printf_count == 2);
    host_image_stream_enabled = 0;
    host_time_ms = 1019;
    balance_vofa_task();
    assert(captured_printf_count == 2);
    host_time_ms = 1020;
    balance_vofa_task();
    assert(captured_printf_count == 3);
    assert(printf_saw_interrupts_enabled);
    for (index = 0; index < CHANNEL_COUNT; index++) if (index != 6) assert(fabs(channel_value(index) - expected_speed_channels[index]) < 0.001);
    assert(channel_value(6) == 1020.0);
}

static void test_uncalibrated_speed_keeps_nan_channels(void)
{
    balance_init();
    set_sample_state();
    balance_state.speed_raw_mps = NAN;
    balance_state.speed_filtered_mps = NAN;
    host_time_ms = 1040;
    balance_vofa_task();
    assert(captured_printf_count == 4);
    assert(isnan(channel_value(1)));
    assert(isnan(channel_value(7)));
    assert(fabs(channel_value(0) - expected_speed_channels[0]) < 0.001);
    assert(fabs(channel_value(2) - expected_speed_channels[2]) < 0.001);
    assert(fabs(channel_value(3) - expected_speed_channels[3]) < 0.001);
    assert(fabs(channel_value(4) - expected_speed_channels[4]) < 0.001);
    assert(fabs(channel_value(5) - expected_speed_channels[5]) < 0.001);
}

static void test_signed_gain_commands(void)
{
    balance_init();
    host_time_ms = 1041;
    host_fifo_push("KP=-170000\nKD=-1000\n");
    balance_vofa_task();
    assert(balance_params.balance_kp == -1700.0f);
    assert(balance_params.balance_kd == -10.0f);
    host_fifo_push("KP=260000\nKD=50000\n");
    balance_vofa_task();
    assert(balance_params.balance_kp == 2600.0f);
    assert(balance_params.balance_kd == 500.0f);
    host_fifo_push("KP=invalid\nKD=2147483648\n");
    balance_vofa_task();
    assert(balance_params.balance_kp == 2600.0f);
    assert(balance_params.balance_kd == 500.0f);
}

static void test_timestamp_wraparound(void)
{
    host_time_ms = 0xfffffff0u;
    balance_vofa_task();
    assert(captured_printf_count == 5);
    assert(channel_value(6) == 4294967280.0);
    host_time_ms = 3;
    balance_vofa_task();
    assert(captured_printf_count == 5);
    host_time_ms = 4;
    balance_vofa_task();
    assert(captured_printf_count == 6);
    assert(channel_value(6) == 4.0);
}

int main(void)
{
    test_firewater_channels();
    test_image_stream_pause_and_recovery();
    test_uncalibrated_speed_keeps_nan_channels();
    test_signed_gain_commands();
    test_timestamp_wraparound();
    return 0;
}
