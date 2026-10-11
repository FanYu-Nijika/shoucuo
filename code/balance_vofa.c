#include "balance_vofa.h"

#include "balance_control.h"
#include "car_uart_stream.h"
#include "zf_common_headfile.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BALANCE_VOFA_PERIOD_MS 20u
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
    // const char *name;
    const char *number;
    int32_t value;
    uint8 field;

    if (strncmp(line, "KP=", 3) == 0) {
        // name = "KP";
        number = line + 3;
        field = 0;
    } else if (strncmp(line, "KD=", 3) == 0) {
        // name = "KD";
        number = line + 3;
        field = 1;
    } else if (strncmp(line, "MID=", 4) == 0) {
        // name = "MID";
        number = line + 4;
        field = 2;
    } else {
        // printf("ERR\n");
        return;
    }
    if (!balance_vofa_parse_number(number, &value)) {
        // printf("ERR\n");
        return;
    }

    params = balance_params;
    if (field == 0) params.balance_kp = value / 100.0f;
    else if (field == 1) params.balance_kd = value / 100.0f;
    else params.middle_angle = value / 100.0f;
    if (!balance_apply_live_params(&params)) {
        // printf("ERR\n");
        return;
    }
    // printf("OK %s=%ld\n", name, (long)value);
}

static void balance_vofa_receive(void)
{
    uint8 byte;

    while (debug_read_ring_buffer(&byte, 1) == 1) {
        if (byte == '\n') {
            if (command_overflow) {
                // printf("ERR\n");
            } else if (command_length != 0) {
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

static void balance_vofa_send(uint32 now_ms)
{
    balance_state_t state;
    uint32 interrupt_state;

    /* Snapshot the ISR state together, but keep serial formatting outside the critical section. */
    interrupt_state = interrupt_global_disable();
    state = balance_state;
    interrupt_global_enable(interrupt_state);
    /* Keep the original six channels first so existing scope plots keep their meaning. */
    printf("speed:%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.0f,%.0f,%ld,%ld,%d,%d,%u,%u,%u\n",
           state.speed_target_applied_mps, state.speed_filtered_mps, state.speed_angle_applied,
           state.target_angle, state.angle, state.balance_output, (unsigned long)now_ms,
           state.speed_raw_mps, state.speed_feedback_mps, state.speed_error_mps, state.speed_p_angle,
           state.speed_integral_angle, state.gyro_rate, state.turn_pwm_command, state.turn_output,
           (long)state.left_count, (long)state.right_count, state.left_pwm, state.right_pwm,
           (unsigned)state.speed_pid_active, (unsigned)state.running, (unsigned)state.fault);
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
    balance_vofa_send(now_ms);
}
