#include "car_seekfree_debug.h"

#include <string.h>

#include "balance_control.h"
#include "balance_runtime.h"
#include "board_pins.h"
#include "car_menu.h"
#include "seekfree_assistant.h"
#include "zf_common_fifo.h"
#include "zf_common_typedef.h"
#include "zf_device_type.h"
#include "zf_driver_gpio.h"
#include "zf_driver_timer.h"
#include "zf_driver_uart.h"

#pragma section all "cpu0_dsram"

/* CH0..CH7: ACC angle, complementary angle, target angle, gyro rate, P, D, raw output, clamped common PWM. */
#define CAR_SEEKFREE_CHANNEL_COUNT   (8)
#define CAR_SEEKFREE_SEND_PERIOD_MS  (10)
#define CAR_SEEKFREE_UART_BAUD       (115200)
#define CAR_SEEKFREE_RX_BUFFER_SIZE  (64)

extern seekfree_assistant_transfer_callback_function seekfree_assistant_transfer_callback;
extern seekfree_assistant_receive_callback_function seekfree_assistant_receive_callback;

static seekfree_assistant_oscilloscope_struct assistant_scope;
static seekfree_assistant_debug_param_struct assistant_params;
static fifo_struct assistant_rx_fifo;
static float assistant_send_channels[CAR_SEEKFREE_CHANNEL_COUNT];
static volatile float assistant_snapshots[2][CAR_SEEKFREE_CHANNEL_COUNT];
static uint8 assistant_rx_buffer[CAR_SEEKFREE_RX_BUFFER_SIZE];
static volatile uint8 assistant_snapshot_index;
static uint32 assistant_last_send_ms;
static uint8 assistant_ready;

static uint8 car_seekfree_value_finite(float value)
{
    uint32 bits;

    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

static uint32 car_seekfree_transfer(const uint8 *buffer, uint32 length)
{
    if (buffer == 0 || gpio_get_level(BOARD_WIRELESS_RTS_PIN) != GPIO_LOW) return length;
    uart_write_buffer(BOARD_WIRELESS_UART, buffer, length);
    return 0;
}

static uint32 car_seekfree_receive(uint8 *buffer, uint32 length)
{
    uint32 read_length = length;

    if (buffer == 0) return 0;
    fifo_read_buffer(&assistant_rx_fifo, buffer, &read_length, FIFO_READ_AND_CLEAN);
    return read_length;
}

static void car_seekfree_uart_callback(void)
{
    uint8 data;

    if (uart_query_byte(BOARD_WIRELESS_UART, &data)) fifo_write_buffer(&assistant_rx_fifo, &data, 1);
}

static void car_seekfree_sync_param(uint8 index)
{
    /* P0..P6 tune balance values live; P7=1 saves the active profile while stopped. */
    if (index == 0) assistant_params.data[index] = balance_params.balance_kp;
    else if (index == 1) assistant_params.data[index] = balance_params.balance_kd;
    else if (index == 2) assistant_params.data[index] = balance_params.middle_angle;
    else if (index == 3) assistant_params.data[index] = balance_params.accel_weight;
    else if (index == 4) assistant_params.data[index] = balance_params.pwm_limit;
    else if (index == 5) assistant_params.data[index] = balance_params.left_motor_deadzone;
    else if (index == 6) assistant_params.data[index] = balance_params.right_motor_deadzone;
    else assistant_params.data[index] = 0.0f;
}

static uint8 car_seekfree_apply_param(uint8 index, float value)
{
    balance_params_t params = balance_params;

    if (!car_seekfree_value_finite(value)) return 0;
    if (index == 0) {
        if (value < 0.0f) return 0;
        params.balance_kp = value;
    } else if (index == 1) {
        if (value < 0.0f) return 0;
        params.balance_kd = value;
    } else if (index == 2) {
        if (value < -180.0f || value > 180.0f) return 0;
        params.middle_angle = value;
    } else if (index == 3) {
        if (value < 0.0f || value > 1.0f) return 0;
        params.accel_weight = value;
    } else if (index == 4) {
        if (value < 1.0f || value > 10000.0f) return 0;
        params.pwm_limit = value;
    } else if (index == 5) {
        if (value < 0.0f || value > 3000.0f) return 0;
        params.left_motor_deadzone = value;
    } else if (index == 6) {
        if (value < 0.0f || value > 3000.0f) return 0;
        params.right_motor_deadzone = value;
    } else return 0;
    return balance_apply_live_params(&params);
}

static void car_seekfree_process_params(void)
{
    uint8 index;

    seekfree_assistant_debug_param_analysis(&assistant_params);
    for (index = 0; index < 7; index++) {
        if (assistant_params.update_flag[index] == 0) continue;
        assistant_params.update_flag[index] = 0;
        if (!car_seekfree_apply_param(index, assistant_params.data[index])) car_seekfree_sync_param(index);
    }
    if (assistant_params.update_flag[7] != 0) {
        assistant_params.update_flag[7] = 0;
        /* Flash writes remain explicit and are rejected while the balance controller is running. */
        if (assistant_params.data[7] >= 0.5f && assistant_params.data[7] < 1.5f &&
            balance_state.running == 0 && balance_start_pending == 0) {
            balance_params_flash_save(car_menu_current_profile());
        }
        assistant_params.data[7] = 0.0f;
    }
}

void car_seekfree_debug_init(void)
{
    uint8 index;

    assistant_ready = 0;
    assistant_snapshot_index = 0;
    assistant_last_send_ms = 0;
    memset(&assistant_params, 0, sizeof(assistant_params));
    memset((void *)assistant_snapshots, 0, sizeof(assistant_snapshots));
    fifo_init(&assistant_rx_fifo, FIFO_DATA_8BIT, assistant_rx_buffer, sizeof(assistant_rx_buffer));
    for (index = 0; index < CAR_SEEKFREE_CHANNEL_COUNT; index++) car_seekfree_sync_param(index);
    seekfree_assistant_oscilloscope_config(&assistant_scope, CAR_SEEKFREE_CHANNEL_COUNT, assistant_send_channels);
    gpio_init(BOARD_WIRELESS_RTS_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    set_wireless_type(WIRELESS_UART, car_seekfree_uart_callback);
    uart_init(BOARD_WIRELESS_UART, CAR_SEEKFREE_UART_BAUD,
        BOARD_WIRELESS_UART_TX_PIN, BOARD_WIRELESS_UART_RX_PIN);
    uart_rx_interrupt(BOARD_WIRELESS_UART, 1);
    /* The board RTS pin differs from the stock driver, so bind the official assistant to this board-local transport. */
    seekfree_assistant_transfer_callback = car_seekfree_transfer;
    seekfree_assistant_receive_callback = car_seekfree_receive;
    assistant_ready = 1;
}

void car_seekfree_debug_capture_5ms(void)
{
    uint8 slot;

    if (assistant_ready == 0) return;
    slot = assistant_snapshot_index ^ 1U;
    assistant_snapshots[slot][0] = balance_state.accel_angle;
    assistant_snapshots[slot][1] = balance_state.angle;
    assistant_snapshots[slot][2] = balance_state.target_angle;
    assistant_snapshots[slot][3] = balance_state.gyro_rate;
    assistant_snapshots[slot][4] = balance_state.p_output;
    assistant_snapshots[slot][5] = balance_state.d_output;
    assistant_snapshots[slot][6] = balance_state.balance_output;
    assistant_snapshots[slot][7] = balance_state.common_pwm;
    __dsync();
    assistant_snapshot_index = slot;
    __dsync();
}

void car_seekfree_debug_task(void)
{
    uint32 now;
    uint8 index;
    uint8 slot;

    if (assistant_ready == 0) return;
    car_seekfree_process_params();
    now = system_getval_ms();
    if (now - assistant_last_send_ms < CAR_SEEKFREE_SEND_PERIOD_MS) return;
    if (gpio_get_level(BOARD_WIRELESS_RTS_PIN) != GPIO_LOW) return;
    assistant_last_send_ms = now;
    __dsync();
    slot = assistant_snapshot_index;
    for (index = 0; index < CAR_SEEKFREE_CHANNEL_COUNT; index++) {
        assistant_send_channels[index] = assistant_snapshots[slot][index];
    }
    __dsync();
    seekfree_assistant_oscilloscope_send(&assistant_scope);
}

#pragma section all restore
