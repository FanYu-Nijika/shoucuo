#include "car_vofa_debug.h"

#include <float.h>
#include <stdint.h>
#include <string.h>

#include "balance_control.h"
#include "balance_runtime.h"
#include "board_pins.h"
#include "car_menu.h"
#include "car_vofa_protocol.h"
#include "zf_common_interrupt.h"
#include "zf_common_typedef.h"
#include "zf_driver_timer.h"
#include "zf_driver_uart.h"

#pragma section all "cpu0_dsram"

#define CAR_VOFA_UART_BAUD          (115200)
#define CAR_VOFA_PERIOD_MS          (20)
#define CAR_VOFA_CHANNEL_COUNT      (10)
#define CAR_VOFA_RX_BUFFER_SIZE     (256)
#define CAR_VOFA_TX_BUFFER_SIZE     (192)
#define CAR_VOFA_REPLY_FRAME_SIZE   (80)
#define CAR_VOFA_REPLY_QUEUE_COUNT  (64)
#define CAR_VOFA_FIXED_LIMIT        (1000000.0f)

static volatile uint16 rx_head;
static volatile uint16 rx_tail;
static volatile uint8 rx_overflow;
static uint8 rx_buffer[CAR_VOFA_RX_BUFFER_SIZE];
static car_vofa_line_t rx_line;
static uint8 rx_resync;
static volatile float samples[CAR_VOFA_CHANNEL_COUNT];
static uint32 last_send_ms;
static uint8 vofa_ready;
static char reply_queue[CAR_VOFA_REPLY_QUEUE_COUNT][CAR_VOFA_REPLY_FRAME_SIZE];
static uint16 reply_length[CAR_VOFA_REPLY_QUEUE_COUNT];
static uint8 reply_head;
static uint8 reply_tail;
static uint8 reply_count;
static char pending_line[CAR_VOFA_LINE_CAPACITY];
static uint8 pending_line_ready;

static uint8 car_vofa_append_char(char *message, uint16 *length, uint16 capacity, char value)
{
    if (*length + 1 >= capacity) return 0;
    message[*length] = value;
    (*length)++;
    message[*length] = '\0';
    return 1;
}

static uint8 car_vofa_append_text(char *message, uint16 *length, uint16 capacity, const char *text)
{
    while (*text != '\0') {
        if (!car_vofa_append_char(message, length, capacity, *text++)) return 0;
    }
    return 1;
}

/* Fixed point keeps telemetry independent of the target C library's float printf support. */
static uint8 car_vofa_append_fixed_decimals(char *message, uint16 *length, uint16 capacity, float value, uint8 decimals)
{
    char digits[20];
    uint64_t scale = 1;
    uint64_t scaled;
    uint64_t whole;
    uint64_t fraction;
    uint64_t divisor;
    uint8 digit_count = 0;
    uint8 index;
    uint8 negative = 0;

    for (index = 0; index < decimals; index++) scale *= 10;
    if (value < 0.0f) {
        negative = 1;
        value = -value;
    }

    scaled = (uint64_t)(value * (float)scale + 0.5f);
    whole = scaled / scale;
    fraction = scaled % scale;
    if (negative && !car_vofa_append_char(message, length, capacity, '-')) return 0;
    do {
        digits[digit_count++] = (char)('0' + whole % 10);
        whole /= 10;
    } while (whole != 0UL && digit_count < sizeof(digits));
    while (digit_count != 0) {
        if (!car_vofa_append_char(message, length, capacity, digits[--digit_count])) return 0;
    }
    if (decimals == 0) return 1;
    if (!car_vofa_append_char(message, length, capacity, '.')) return 0;
    divisor = scale / 10;
    for (index = 0; index < decimals; index++) {
        if (!car_vofa_append_char(message, length, capacity, (char)('0' + fraction / divisor))) return 0;
        fraction %= divisor;
        divisor /= 10;
    }
    return 1;
}

static uint8 car_vofa_append_fixed(char *message, uint16 *length, uint16 capacity, float value)
{
    if (!(value == value) || value > CAR_VOFA_FIXED_LIMIT || value < -CAR_VOFA_FIXED_LIMIT) return 0;
    return car_vofa_append_fixed_decimals(message, length, capacity, value, 3);
}

static uint8 car_vofa_append_value(char *message, uint16 *length, uint16 capacity, float value)
{
    uint8 digit_count = 0;
    uint8 negative = 0;
    char digits[11];
    int exponent = 0;

    if (!(value == value)) return car_vofa_append_text(message, length, capacity, "NAN");
    if (value > FLT_MAX || value < -FLT_MAX) return car_vofa_append_text(message, length, capacity, "OVF");
    if (value <= CAR_VOFA_FIXED_LIMIT && value >= -CAR_VOFA_FIXED_LIMIT)
        return car_vofa_append_fixed_decimals(message, length, capacity, value, 6);
    if (value < 0.0f) {
        negative = 1;
        value = -value;
    }
    while (value >= 10.0f && exponent < 38) {
        value *= 0.1f;
        exponent++;
    }
    while (value < 1.0f && exponent > -38) {
        value *= 10.0f;
        exponent--;
    }
    if (value >= 9.999999f) {
        value *= 0.1f;
        exponent++;
    }
    if (!car_vofa_append_fixed_decimals(message, length, capacity, negative ? -value : value, 6)) return 0;
    if (!car_vofa_append_char(message, length, capacity, 'e')) return 0;
    if (!car_vofa_append_char(message, length, capacity, exponent < 0 ? '-' : '+')) return 0;
    if (exponent < 0) exponent = -exponent;
    do {
        digits[digit_count++] = (char)('0' + exponent % 10);
        exponent /= 10;
    } while (exponent != 0 && digit_count < sizeof(digits));
    while (digit_count != 0) {
        if (!car_vofa_append_char(message, length, capacity, digits[--digit_count])) return 0;
    }
    return 1;
}

static uint8 car_vofa_queue_reply(const char *message, uint16 length)
{
    uint8 slot;

    if (!CAR_VOFA_TX_ENABLED) return 1;
    if (length == 0 || length >= CAR_VOFA_REPLY_FRAME_SIZE || reply_count >= CAR_VOFA_REPLY_QUEUE_COUNT) return 0;
    slot = reply_head;
    memcpy(reply_queue[slot], message, length);
    reply_length[slot] = length;
    reply_head++;
    if (reply_head == CAR_VOFA_REPLY_QUEUE_COUNT) reply_head = 0;
    reply_count++;
    return 1;
}

static uint8 car_vofa_queue_text(const char *text)
{
    uint16 length = 0;

    while (text[length] != '\0') {
        if (length + 1 >= CAR_VOFA_REPLY_FRAME_SIZE) return 0;
        length++;
    }
    return car_vofa_queue_reply(text, length);
}

static void car_vofa_flush_replies(void)
{
    uint8 slot;

    if (!CAR_VOFA_TX_ENABLED) return;
    while (reply_count != 0) {
        slot = reply_tail;
        if (!uart_write_buffer_nonblocking(BOARD_WIRELESS_UART, (const uint8 *)reply_queue[slot], reply_length[slot])) break;
        reply_tail++;
        if (reply_tail == CAR_VOFA_REPLY_QUEUE_COUNT) reply_tail = 0;
        reply_count--;
    }
}

static uint8 car_vofa_rx_pop(uint8 *data)
{
    uint16 tail = rx_tail;

    if (tail == rx_head) return 0;
    *data = rx_buffer[tail];
    tail++;
    if (tail == CAR_VOFA_RX_BUFFER_SIZE) tail = 0;
    rx_tail = tail;
    return 1;
}

static float car_vofa_param_value(const balance_params_t *params, car_vofa_param_t index)
{
    if (index == CAR_VOFA_PARAM_BALANCE_KP) return params->balance_kp;
    if (index == CAR_VOFA_PARAM_BALANCE_KD) return params->balance_kd;
    if (index == CAR_VOFA_PARAM_MIDDLE_ANGLE) return params->middle_angle;
    if (index == CAR_VOFA_PARAM_SPEED_KP) return params->speed_kp;
    if (index == CAR_VOFA_PARAM_SPEED_KI) return params->speed_ki;
    if (index == CAR_VOFA_PARAM_TARGET_SPEED_MPS) return params->target_speed_mps;
    return params->speed_angle_limit;
}

static void car_vofa_set_param(balance_params_t *params, car_vofa_param_t index, float value)
{
    if (index == CAR_VOFA_PARAM_BALANCE_KP) params->balance_kp = value;
    else if (index == CAR_VOFA_PARAM_BALANCE_KD) params->balance_kd = value;
    else if (index == CAR_VOFA_PARAM_MIDDLE_ANGLE) params->middle_angle = value;
    else if (index == CAR_VOFA_PARAM_SPEED_KP) params->speed_kp = value;
    else if (index == CAR_VOFA_PARAM_SPEED_KI) params->speed_ki = value;
    else if (index == CAR_VOFA_PARAM_TARGET_SPEED_MPS) params->target_speed_mps = value;
    else params->speed_angle_limit = value;
}

static void car_vofa_queue_param(const balance_params_t *params, car_vofa_param_t index)
{
    char message[CAR_VOFA_REPLY_FRAME_SIZE];
    uint16 length = 0;

    message[0] = '\0';
    if (!car_vofa_append_text(message, &length, sizeof(message), "PARAM ") ||
        !car_vofa_append_text(message, &length, sizeof(message), car_vofa_param_name(index)) ||
        !car_vofa_append_char(message, &length, sizeof(message), ' ') ||
        !car_vofa_append_value(message, &length, sizeof(message), car_vofa_param_value(params, index)) ||
        !car_vofa_append_char(message, &length, sizeof(message), '\n')) return;
    car_vofa_queue_reply(message, length);
}

static void car_vofa_queue_set_reply(car_vofa_param_t index, float value)
{
    char message[CAR_VOFA_REPLY_FRAME_SIZE];
    uint16 length = 0;

    message[0] = '\0';
    if (!car_vofa_append_text(message, &length, sizeof(message), "OK SET ") ||
        !car_vofa_append_text(message, &length, sizeof(message), car_vofa_param_name(index)) ||
        !car_vofa_append_char(message, &length, sizeof(message), ' ') ||
        !car_vofa_append_value(message, &length, sizeof(message), value) ||
        !car_vofa_append_char(message, &length, sizeof(message), '\n')) return;
    car_vofa_queue_reply(message, length);
}

static void car_vofa_process_line(const char *line)
{
    car_vofa_command_t command;
    car_vofa_parse_result_t result = car_vofa_parse_command(line, &command);
    balance_params_t params;
    car_vofa_param_t index;

    if (result == CAR_VOFA_PARSE_SYNTAX) {
        car_vofa_queue_text("ERR SYNTAX\n");
        return;
    }
    if (result == CAR_VOFA_PARSE_NAME) {
        car_vofa_queue_text("ERR NAME\n");
        return;
    }
    if (result == CAR_VOFA_PARSE_NUMBER) {
        car_vofa_queue_text("ERR NUMBER\n");
        return;
    }
    if (result != CAR_VOFA_PARSE_OK) {
        car_vofa_queue_text("ERR SYNTAX\n");
        return;
    }

    if (command.type == CAR_VOFA_CMD_GET) {
        params = balance_params;
        for (index = CAR_VOFA_PARAM_BALANCE_KP; index < CAR_VOFA_PARAM_COUNT; index++) car_vofa_queue_param(&params, index);
        car_vofa_queue_text("OK GET\n");
    } else if (command.type == CAR_VOFA_CMD_SAVE) {
        if (balance_state.running != 0 || balance_start_pending != 0) car_vofa_queue_text("ERR RUNNING\n");
        else if (balance_params_flash_save(car_menu_current_profile())) car_vofa_queue_text("OK SAVE\n");
        else car_vofa_queue_text("ERR SAVE\n");
    } else if (command.type == CAR_VOFA_CMD_SET) {
        params = balance_params;
        car_vofa_set_param(&params, command.index, command.value);
        if (!balance_apply_live_params(&params)) {
            car_vofa_queue_text("ERR RANGE\n");
            return;
        }
        car_vofa_queue_set_reply(command.index, command.value);
    } else {
        car_vofa_queue_text("ERR SYNTAX\n");
    }
}

static uint8 car_vofa_reply_slots_needed(const char *line)
{
    car_vofa_command_t command;

    if (car_vofa_parse_command(line, &command) == CAR_VOFA_PARSE_OK && command.type == CAR_VOFA_CMD_GET) return 8;
    return 1;
}

static uint8 car_vofa_process_line_if_possible(const char *line)
{
    uint8 slots = car_vofa_reply_slots_needed(line);

    if (CAR_VOFA_REPLY_QUEUE_COUNT - reply_count < slots) return 0;
    car_vofa_process_line(line);
    return 1;
}

static void car_vofa_process_rx(void)
{
    car_vofa_line_result_t result;
    uint32 interrupt_state;
    uint16 count;
    uint8 data;

    if (pending_line_ready != 0) {
        if (!car_vofa_process_line_if_possible(pending_line)) return;
        pending_line_ready = 0;
    }
    if (rx_overflow != 0) {
        interrupt_state = interrupt_global_disable();
        rx_tail = rx_head;
        rx_overflow = 0;
        interrupt_global_enable(interrupt_state);
        rx_line.length = 0;
        rx_line.discard = 1;
        rx_resync = 1;
        car_vofa_queue_text("ERR RX_OVERFLOW\n");
    }
    for (count = 0; count < CAR_VOFA_RX_BUFFER_SIZE && car_vofa_rx_pop(&data); count++) {
        result = car_vofa_line_feed(&rx_line, data);
        if (result == CAR_VOFA_LINE_READY) {
            if (!car_vofa_process_line_if_possible(rx_line.buffer)) {
                memcpy(pending_line, rx_line.buffer, sizeof(pending_line));
                pending_line_ready = 1;
                return;
            }
        }
        else if (result == CAR_VOFA_LINE_OVERFLOW) {
            if (rx_resync != 0) rx_resync = 0;
            else car_vofa_queue_text("ERR LINE_TOO_LONG\n");
        }
        if (rx_overflow != 0) break;
    }
}

static uint8 car_vofa_build_telemetry(const float *snapshot, char *message, uint16 *length)
{
    uint8 index;

    *length = 0;
    message[0] = '\0';
    if (!car_vofa_append_text(message, length, CAR_VOFA_TX_BUFFER_SIZE, "d:")) return 0;
    for (index = 0; index < CAR_VOFA_CHANNEL_COUNT; index++) {
        if (index != 0 && !car_vofa_append_char(message, length, CAR_VOFA_TX_BUFFER_SIZE, ',')) return 0;
        if (!car_vofa_append_fixed(message, length, CAR_VOFA_TX_BUFFER_SIZE, snapshot[index])) return 0;
    }
    return car_vofa_append_char(message, length, CAR_VOFA_TX_BUFFER_SIZE, '\n');
}

void car_vofa_debug_init(void)
{
    vofa_ready = 0;
    rx_head = 0;
    rx_tail = 0;
    rx_overflow = 0;
    car_vofa_line_init(&rx_line);
    rx_resync = 0;
    reply_head = 0;
    reply_tail = 0;
    reply_count = 0;
    pending_line_ready = 0;
    memset((void *)samples, 0, sizeof(samples));
    /* UART2 is dedicated to the wireless bridge, so the ISR calls this receiver directly. */
    uart_init(BOARD_WIRELESS_UART, CAR_VOFA_UART_BAUD, BOARD_WIRELESS_UART_TX_PIN, BOARD_WIRELESS_UART_RX_PIN);
    uart_rx_interrupt(BOARD_WIRELESS_UART, 1);
    if (CAR_VOFA_TX_ENABLED) uart_tx_interrupt(BOARD_WIRELESS_UART, 1);
    car_vofa_queue_text("VOFA READY\n");
    car_vofa_flush_replies();
    last_send_ms = system_getval_ms();
    vofa_ready = 1;
}

void car_vofa_debug_rx_isr(void)
{
    uint16 next;
    uint8 data;

    if (!uart_query_byte(BOARD_WIRELESS_UART, &data)) return;
    next = rx_head + 1;
    if (next == CAR_VOFA_RX_BUFFER_SIZE) next = 0;
    if (next == rx_tail) {
        rx_overflow = 1;
        return;
    }
    rx_buffer[rx_head] = data;
    __dsync();
    rx_head = next;
}

void car_vofa_debug_capture_5ms(void)
{
    if (vofa_ready == 0 || !CAR_VOFA_TX_ENABLED) return;
    samples[0] = balance_state.angle;
    samples[1] = balance_state.target_angle;
    samples[2] = balance_state.angle_error;
    samples[3] = balance_state.gyro_rate;
    samples[4] = balance_state.balance_output;
    samples[5] = balance_state.speed_target_applied_mps;
    samples[6] = balance_state.speed_filtered_mps;
    samples[7] = balance_state.speed_error_mps;
    samples[8] = balance_state.speed_angle_applied;
    samples[9] = balance_state.common_pwm;
}

void car_vofa_debug_task(void)
{
    float snapshot[CAR_VOFA_CHANNEL_COUNT];
    char message[CAR_VOFA_TX_BUFFER_SIZE];
    uint32 interrupt_state;
    uint32 now;
    uint16 length;
    uint8 index;

    if (vofa_ready == 0) return;
    car_vofa_flush_replies();
    car_vofa_process_rx();
    car_vofa_flush_replies();
    if (!CAR_VOFA_TX_ENABLED) return;
    now = system_getval_ms();
    if (now - last_send_ms < CAR_VOFA_PERIOD_MS || reply_count != 0) return;
    last_send_ms = now;
    /* Hold the 5 ms update briefly so one line represents one coherent sample. */
    interrupt_state = interrupt_global_disable();
    for (index = 0; index < CAR_VOFA_CHANNEL_COUNT; index++) snapshot[index] = samples[index];
    interrupt_global_enable(interrupt_state);
    if (!car_vofa_build_telemetry(snapshot, message, &length)) return;
    /* The UART driver checks the whole frame before writing, so a failed frame is never truncated. */
    uart_write_buffer_nonblocking(BOARD_WIRELESS_UART, (const uint8 *)message, length);
}

#pragma section all restore
