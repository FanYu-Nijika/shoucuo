#include "car_uart_stream.h"

#include "car_shared.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

static uint8 stream_enabled;
/*
 * Official TC264 V1 MT9V03X 8-bit image header:
 * AA 02 40 08 + little-endian width and height.
 *
 * The payload remains one byte per pixel. The stream sends the completed raw
 * gray frame; the TFT binary view uses a separate per-pixel buffer.
 */
static const uint8 seekfree_assistant_camera_header[] = {
    0xAA, 0x02, 0x40, 0x08, CAR_IMAGE_WIDTH, 0x00, CAR_IMAGE_HEIGHT, 0x00
};

#pragma section all restore

void car_uart_stream_init(void)
{
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
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
    if (stream_enabled == 0 || image_frame == 0) return;
    debug_send_buffer(seekfree_assistant_camera_header, sizeof(seekfree_assistant_camera_header));
    debug_send_buffer(image_frame, CAR_IMAGE_WIDTH * CAR_IMAGE_HEIGHT);
}
