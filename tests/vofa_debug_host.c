/* Compile with tests/vofa_debug_host_stubs.h preincluded; this exercises the UART2 command path. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "balance_control.h"
#include "car_vofa_debug.h"

balance_params_t balance_params;
volatile balance_state_t balance_state;
volatile uint8 balance_start_pending;

static char sent[8192];
static uint32 sent_length;
static uint32 tx_capacity;
static uint8 pending_byte;
static uint8 pending_valid;
static uint32 now_ms;
static uint32 apply_count;
static uint32 save_count;
static uint8 save_success;

uint32 interrupt_global_disable(void) { return 0; }
void interrupt_global_enable(uint32 state) { (void)state; }
uint32 system_getval_ms(void) { return now_ms; }

void uart_init(int uart, uint32 baud, int tx_pin, int rx_pin)
{
    assert(uart == UART_2 && baud == 115200 && tx_pin == UART2_TX_P10_5 && rx_pin == UART2_RX_P10_6);
}

void uart_rx_interrupt(int uart, uint32 enabled)
{
    assert(uart == UART_2 && enabled == 1);
}

void uart_tx_interrupt(int uart, uint32 enabled)
{
    assert(uart == UART_2 && enabled == 1);
}

uint8 uart_query_byte(int uart, uint8 *data)
{
    assert(uart == UART_2);
    if (!pending_valid) return 0;
    *data = pending_byte;
    pending_valid = 0;
    return 1;
}

uint8 uart_write_buffer_nonblocking(int uart, const uint8 *data, uint32 length)
{
    assert(uart == UART_2);
    if (length > tx_capacity) return 0;
    assert(sent_length + length < sizeof(sent));
    memcpy(sent + sent_length, data, length);
    sent_length += length;
    sent[sent_length] = '\0';
    tx_capacity -= length;
    return 1;
}

uint8_t car_menu_current_profile(void) { return 2; }

uint8_t balance_apply_live_params(const balance_params_t *params)
{
    apply_count++;
    if (!isfinite(params->balance_kp) || params->balance_kp < 0 ||
        !isfinite(params->balance_kd) || params->balance_kd < 0 ||
        !isfinite(params->middle_angle) || fabsf(params->middle_angle) > 180 ||
        !isfinite(params->speed_kp) || params->speed_kp < 0 || params->speed_kp > 20 ||
        !isfinite(params->speed_ki) || params->speed_ki < 0 || params->speed_ki > 10 ||
        !isfinite(params->target_speed_mps) || params->target_speed_mps < 0 || params->target_speed_mps > 3 ||
        !isfinite(params->speed_angle_limit) || params->speed_angle_limit < 0 ||
        params->speed_angle_limit > 10) return 0;
    balance_params = *params;
    return 1;
}

uint8_t balance_params_flash_save(uint8_t gear)
{
    assert(gear == 2);
    save_count++;
    return save_success;
}

static void reset_test(void)
{
    memset(&balance_params, 0, sizeof(balance_params));
    memset((void *)&balance_state, 0, sizeof(balance_state));
    balance_params.balance_kp = 1000.0f;
    balance_params.balance_kd = 25.0f;
    balance_params.speed_kp = 2.0f;
    balance_params.speed_ki = 0.3f;
    balance_params.target_speed_mps = 1.5f;
    balance_params.speed_angle_limit = 5.0f;
    balance_start_pending = 0;
    sent_length = 0;
    tx_capacity = 4096;
    pending_valid = 0;
    now_ms = 0;
    apply_count = 0;
    save_count = 0;
    save_success = 1;
    car_vofa_debug_init();
    assert(strcmp(sent, "VOFA READY\n") == 0);
    sent_length = 0;
    sent[0] = '\0';
}

static void inject_text(const char *text)
{
    while (*text != '\0') {
        assert(pending_valid == 0);
        pending_byte = (uint8)*text++;
        pending_valid = 1;
        car_vofa_debug_rx_isr();
    }
}

static void test_live_commands(void)
{
    reset_test();
    balance_state.running = 1;
    inject_text("SET BALANCE_KP 1200\nSET BALANCE_KD 30\nSET MIDDLE_ANGLE -1.25\n");
    car_vofa_debug_task();
    inject_text("SET SPEED_KP 3\nSET SPEED_KI 0.4\nSET TARGET_SPEED_MPS 1.2\nSET SPEED_ANGLE_LIMIT 6\n");
    car_vofa_debug_task();
    assert(apply_count == 7);
    assert(balance_params.balance_kp == 1200.0f && balance_params.balance_kd == 30.0f);
    assert(balance_params.middle_angle == -1.25f);
    assert(balance_params.speed_kp == 3.0f && balance_params.speed_ki == 0.4f);
    assert(balance_params.target_speed_mps == 1.2f && balance_params.speed_angle_limit == 6.0f);
    assert(strstr(sent, "OK SET MIDDLE_ANGLE -1.250000\n") != NULL);
}

static void test_errors_and_get(void)
{
    reset_test();
    inject_text("SET SPEED_KP 21\nSET UNKNOWN 1\nSET SPEED_KI nan\nGET\r\n");
    car_vofa_debug_task();
    assert(balance_params.speed_kp == 2.0f && balance_params.speed_ki == 0.3f);
    assert(strstr(sent, "ERR RANGE\n") != NULL);
    assert(strstr(sent, "ERR NAME\n") != NULL);
    assert(strstr(sent, "ERR NUMBER\n") != NULL);
    assert(strstr(sent, "PARAM BALANCE_KP 1000.000000\n") != NULL);
    assert(strstr(sent, "PARAM SPEED_KI 0.300000\n") != NULL);
    assert(strstr(sent, "OK GET\n") != NULL);
}

static void test_save(void)
{
    reset_test();
    balance_state.running = 1;
    inject_text("SAVE\n");
    car_vofa_debug_task();
    assert(save_count == 0 && strstr(sent, "ERR RUNNING\n") != NULL);
    balance_state.running = 0;
    balance_start_pending = 1;
    inject_text("SAVE\n");
    car_vofa_debug_task();
    assert(save_count == 0);
    balance_start_pending = 0;
    inject_text("SAVE\n");
    car_vofa_debug_task();
    assert(save_count == 1 && strstr(sent, "OK SAVE\n") != NULL);
    save_success = 0;
    inject_text("SAVE\n");
    car_vofa_debug_task();
    assert(save_count == 2 && strstr(sent, "ERR SAVE\n") != NULL);
}

static void test_telemetry(void)
{
    float values[10];
    int consumed = 0;
    int parsed;

    reset_test();
    balance_state.angle = 1.0f;
    balance_state.target_angle = 2.0f;
    balance_state.angle_error = 3.0f;
    balance_state.gyro_rate = 4.0f;
    balance_state.balance_output = 5.0f;
    balance_state.speed_target_applied_mps = 6.0f;
    balance_state.speed_filtered_mps = 7.0f;
    balance_state.speed_error_mps = 8.0f;
    balance_state.speed_angle_applied = 9.0f;
    balance_state.common_pwm = 10.0f;
    car_vofa_debug_capture_5ms();
    now_ms = 19;
    car_vofa_debug_task();
    assert(sent_length == 0);
    now_ms = 20;
    car_vofa_debug_task();
    parsed = sscanf(sent, "d:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n%n", &values[0], &values[1], &values[2],
                    &values[3], &values[4], &values[5], &values[6], &values[7], &values[8], &values[9], &consumed);
    assert(parsed == 10 && consumed == (int)sent_length);
    assert(values[0] == 1.0f && values[9] == 10.0f);
}

static void test_receive_recovery(void)
{
    uint16 index;

    reset_test();
    for (index = 0; index < 70; index++) inject_text("X");
    inject_text("\nGET\n");
    car_vofa_debug_task();
    assert(strstr(sent, "ERR LINE_TOO_LONG\n") != NULL);
    assert(strstr(sent, "OK GET\n") != NULL);

    reset_test();
    for (index = 0; index < 300; index++) inject_text("X");
    car_vofa_debug_task();
    assert(strcmp(sent, "ERR RX_OVERFLOW\n") == 0);
    inject_text("\nGET\n");
    car_vofa_debug_task();
    assert(strstr(sent, "OK GET\n") != NULL);
    assert(strstr(sent, "ERR LINE_TOO_LONG\n") == NULL);
}

static void test_tx_backpressure_and_formatting(void)
{
    const char *cursor;
    uint8 get_count;

    reset_test();
    tx_capacity = 5;
    balance_state.angle = 1.0f;
    car_vofa_debug_capture_5ms();
    now_ms = 20;
    car_vofa_debug_task();
    assert(sent_length == 0);
    inject_text("SET SPEED_KP 3\nGET\n");
    car_vofa_debug_task();
    assert(sent_length == 0);
    tx_capacity = 4096;
    now_ms = 40;
    car_vofa_debug_task();
    assert(strstr(sent, "OK SET SPEED_KP 3.000000\n") != NULL);
    assert(strstr(sent, "OK GET\n") != NULL);

    reset_test();
    tx_capacity = 0;
    inject_text("GET\nGET\nGET\nGET\nGET\nGET\nGET\nGET\nGET\nGET\n");
    car_vofa_debug_task();
    tx_capacity = 4096;
    car_vofa_debug_task();
    cursor = sent;
    get_count = 0;
    while ((cursor = strstr(cursor, "OK GET\n")) != NULL) {
        get_count++;
        cursor += 7;
    }
    assert(get_count == 10);

    reset_test();
    balance_params.balance_kp = 100000000.0f;
    inject_text("GET\n");
    car_vofa_debug_task();
    assert(strstr(sent, "PARAM BALANCE_KP 1.000000e+8\n") != NULL);

    reset_test();
    balance_state.angle = NAN;
    car_vofa_debug_capture_5ms();
    now_ms = 20;
    car_vofa_debug_task();
    assert(sent_length == 0);
}

int main(void)
{
    test_live_commands();
    test_errors_and_get();
    test_save();
    test_telemetry();
    test_receive_recovery();
    test_tx_backpressure_and_formatting();
    return 0;
}
