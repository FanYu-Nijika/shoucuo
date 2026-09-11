#include "balance_runtime.h"

#include <math.h>

#include "balance_control.h"
#include "board_pins.h"
#include "car.h"
#include "car_menu_port.h"
#include "qd_device_icm42688.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"

volatile uint32 car_balance_max_us;
volatile uint32 car_balance_period_us;
volatile uint32 car_balance_overruns;
volatile uint32 car_uptime_ms;
volatile uint8 car_balance_timing_reset = 1;

static uint8 imu_initialized;

static uint8 balance_runtime_probe_imu(void)
{
    uint8 model = 0;
    uint8 attempt;

    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPI_SPEED, ICM42688_SPC_PIN, ICM42688_SDI_PIN, ICM42688_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM42688_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    for (attempt = 0; attempt < 50; attempt++) {
        gpio_low(ICM42688_CS_PIN);
        spi_read_8bit_registers(ICM42688_SPI, 0x75 | 0x80, &model, 1);
        gpio_high(ICM42688_CS_PIN);
        if (model == 0x47) return 1;
        system_delay_ms(10);
    }
    return 0;
}

static uint8 balance_runtime_read_imu(float *angle, float *rate)
{
    float accel[3];
    float gyro[3];
    float accel_norm_square;

    if (imu_initialized == 0) return 0;
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    accel[0] = icm42688_acc_x;
    accel[1] = icm42688_acc_y;
    accel[2] = icm42688_acc_z;
    gyro[0] = icm42688_gyro_x;
    gyro[1] = icm42688_gyro_y;
    gyro[2] = icm42688_gyro_z;
    accel_norm_square = accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2];
    if (accel_norm_square < 0.04f || accel_norm_square > 4.0f) return 0;
    *angle = atan2f(BOARD_BALANCE_ACCEL_SIGN * accel[BOARD_BALANCE_ACCEL_HORIZONTAL_AXIS],
        BOARD_BALANCE_ACCEL_VERTICAL_SIGN * accel[BOARD_BALANCE_ACCEL_VERTICAL_AXIS]) * 57.2957795f;
    *rate = BOARD_BALANCE_GYRO_SIGN * gyro[BOARD_BALANCE_GYRO_AXIS];
    return 1;
}

void balance_runtime_init(void)
{
    balance_init();
    cc_tc264_balance_encoder_init();
    if (balance_runtime_probe_imu() == 0) {
        car_running = 0;
        balance_update_5ms(0.0f, 0.0f, 0, 0, 0);
        cc_tc264_menu_motor_stop();
        return;
    }
    Init_ICM42688();
    imu_initialized = 1;
    car_running = 1;
    car_balance_timing_reset = 1;
    balance_runtime_update_5ms();
}

void balance_runtime_update_5ms(void)
{
    float angle = 0.0f;
    float rate = 0.0f;
    int16 left_count = 0;
    int16 right_count = 0;
    uint8 was_running = balance_state.running;
    uint8 valid;

    cc_tc264_balance_encoder_read(&left_count, &right_count);
    valid = balance_runtime_read_imu(&angle, &rate);
    balance_update_5ms(angle, rate, left_count, right_count, valid);

    if (was_running != 0 && balance_state.running == 0) {
        car_running = 0;
    } else if (car_running != 0 && balance_state.running == 0) {
        car_running = balance_start();
        if (car_running != 0) car_balance_timing_reset = 1;
    } else if (car_running == 0 && balance_state.running != 0) {
        balance_stop();
    }

    car_left_command = balance_state.left_pwm;
    car_right_command = balance_state.right_pwm;
    if (balance_state.running != 0 && balance_state.output_updated != 0) {
        cc_tc264_menu_motor_write(car_left_command, car_right_command);
    } else if (balance_state.running == 0) {
        cc_tc264_menu_motor_stop();
    }
}

void balance_runtime_stop(void)
{
    car_running = 0;
    balance_stop();
    car_left_command = 0;
    car_right_command = 0;
    cc_tc264_menu_motor_stop();
}
