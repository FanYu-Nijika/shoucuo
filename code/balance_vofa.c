#include "balance_vofa.h"

#include "balance_control.h"
#include "car_uart_stream.h"
#include "zf_common_headfile.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BALANCE_VOFA_PERIOD_MS 100u
#define BALANCE_VOFA_COMMAND_LEN 32u

static char command[BALANCE_VOFA_COMMAND_LEN];
static uint8 command_length;
static uint8 command_overflow;
static uint8 initialized;
static uint32 last_send_ms;

static uint8 balance_vofa_parse_number(const char *text, int32_t *value)
{
    uint32_t magnitude = 0;
    uint32_t limit = 2147483647u;
    uint8 negative = 0;
    uint8 digit;

    if (*text == '-') {
        negative = 1;
        limit = 2147483648u;
        text++;
    }
    if (*text == '\0') return 0;
    while (*text != '\0') {
        if (*text < '0' || *text > '9') return 0;
        digit = (uint8)(*text - '0');
        if (magnitude > (limit - digit) / 10u) return 0;
        magnitude = magnitude * 10u + digit;
        text++;
    }
    if (negative && magnitude == 2147483648u) *value = INT32_MIN;
    else if (negative) *value = -(int32_t)magnitude;
    else *value = (int32_t)magnitude;
    return 1;
}

static void balance_vofa_apply_command(char *line)
{
    balance_params_t params;
    const char *name;
    const char *number;
    int32_t value;
    uint8 field;

    if (strncmp(line, "KP=", 3) == 0) {
        name = "KP";
        number = line + 3;
        field = 0;
    } else if (strncmp(line, "KD=", 3) == 0) {
        name = "KD";
        number = line + 3;
        field = 1;
    } else if (strncmp(line, "MID=", 4) == 0) {
        name = "MID";
        number = line + 4;
        field = 2;
    } else {
        printf("ERR\n");
        return;
    }
    if (!balance_vofa_parse_number(number, &value) || (field != 2 && value < 0)) {
        printf("ERR\n");
        return;
    }

    params = balance_params;
    if (field == 0) params.balance_kp = value / 100.0f;
    else if (field == 1) params.balance_kd = value / 100.0f;
    else params.middle_angle = value / 100.0f;
    if (!balance_apply_live_params(&params)) {
        printf("ERR\n");
        return;
    }
    printf("OK %s=%ld\n", name, (long)value);
}

static void balance_vofa_receive(void)
{
    uint8 byte;

    while (debug_read_ring_buffer(&byte, 1) == 1) {
        if (byte == '\n') {
            if (command_overflow) printf("ERR\n");
            else if (command_length != 0) {
                if (command[command_length - 1] == '\r') command_length--;
                if (command_length != 0) {
                    command[command_length] = '\0';
                    balance_vofa_apply_command(command);
                }
            }
            command_length = 0;
            command_overflow = 0;
        } else if (!command_overflow) {
            if ((byte < 32 && byte != '\r') || byte > 126 ||
                command_length >= BALANCE_VOFA_COMMAND_LEN - 1u) command_overflow = 1;
            else command[command_length++] = (char)byte;
        }
    }
}

static uint8 balance_vofa_scale(float input, long *output)
{
    if (input != input || input < -21474800.0f || input > 21474800.0f) return 0;
    *output = (long)(input * 100.0f);
    return 1;
}

static void balance_vofa_send(void)
{
    float samples[8];
    long values[8];
    uint32 interrupt_state;
    uint8 index;

    /* The PIT updates these fields separately; a short snapshot keeps one frame coherent. */
    interrupt_state = interrupt_global_disable();
    samples[0] = balance_state.angle;
    samples[1] = balance_state.target_angle;
    samples[2] = balance_state.angle_error;
    samples[3] = balance_state.gyro_rate;
    samples[4] = balance_state.p_output;
    samples[5] = balance_state.d_output;
    samples[6] = balance_state.balance_output;
    samples[7] = balance_state.common_pwm;
    interrupt_global_enable(interrupt_state);

    for (index = 0; index < 8; index++) {
        if (!balance_vofa_scale(samples[index], &values[index])) return;
    }
    printf("bal:%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
        values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7]);
}

void balance_vofa_task(void)
{
    uint32 now_ms = system_getval_ms();
    uint8 byte;

    if (!initialized) {
        /* Wireless UART auto-baud may leave binary startup bytes in the debug FIFO. */
        while (debug_read_ring_buffer(&byte, 1) == 1) {}
        last_send_ms = now_ms;
        initialized = 1;
        return;
    }
    if (car_uart_stream_is_enabled()) {
        while (debug_read_ring_buffer(&byte, 1) == 1) {}
        command_length = 0;
        command_overflow = 0;
        last_send_ms = now_ms;
        return;
    }

    balance_vofa_receive();
    if (now_ms - last_send_ms < BALANCE_VOFA_PERIOD_MS) return;
    last_send_ms = now_ms;
    balance_vofa_send();
}
