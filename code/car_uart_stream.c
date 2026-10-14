#include "car_uart_stream.h"

#include "board_pins.h"
#include "car_shared.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

static uint8 stream_enabled;
static uint8 stream_line[CAR_IMAGE_WIDTH];
/*
 * Official TC264 V1 MT9V03X 8-bit image header:
 * AA 02 40 08 + little-endian width and height.
 *
 * The binary frame stores pixels as 0/1 for the vision code, so the stream
 * expands them to 0/255 for the upper computer's 8-bit grayscale display.
 */
static const uint8 seekfree_assistant_camera_header[] = {
    0xAA, 0x02, 0x40, 0x08, CAR_IMAGE_WIDTH, 0x00, CAR_IMAGE_HEIGHT, 0x00
};

#pragma section all restore

void car_uart_stream_init(void)
{
    /* The download connector is on UART0; debug/printf keep their independent UART2 connection. */
    uart_init(BOARD_HOST_UART, 115200, BOARD_HOST_UART_TX_PIN, BOARD_HOST_UART_RX_PIN);
    stream_enabled = 0;
}
void car_uart_stream_start(void)
{
    stream_enabled = 1;
}
void car_uart_stream_stop(void)
{
    stream_enabled = 0;
}
uint8 car_uart_stream_is_enabled(void)
{
    return stream_enabled;
}

void car_uart_stream_send_frame(uint8 *image_frame)
{
    uint16 row;
    uint16 column;

    if (stream_enabled == 0 || image_frame == 0) return;
    uart_write_buffer(BOARD_HOST_UART, seekfree_assistant_camera_header, sizeof(seekfree_assistant_camera_header));
    for (row = 0; row < CAR_IMAGE_HEIGHT; row++) {
        for (column = 0; column < CAR_IMAGE_WIDTH; column++) {
            stream_line[column] = 0;
            if (image_frame[row * CAR_IMAGE_WIDTH + column] != 0) stream_line[column] = 255;
        }
        uart_write_buffer(BOARD_HOST_UART, stream_line, CAR_IMAGE_WIDTH);
    }
}
