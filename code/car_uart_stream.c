#include "car_uart_stream.h"

#include "car_shared.h"
#include "zf_common_headfile.h"

#pragma section all "cpu0_dsram"

static uint8 stream_enabled;
/*
 * Official TC264 V1 MT9V03X 8-bit image header:
 * AA 02 40 08 + little-endian width and height.
 *
 * The payload remains one byte per pixel. The caller now supplies a marked
 * binary image instead of the raw gray image, so the wire size is unchanged.
 */
static const uint8 seekfree_assistant_camera_header[] = {
    0xAAU, 0x02U, 0x40U, 0x08U, CAR_IMAGE_WIDTH, 0x00U, CAR_IMAGE_HEIGHT, 0x00U
};

#pragma section all restore

void car_uart_stream_init(void)
{
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    stream_enabled = 0U;
}

void car_uart_stream_start(void)
{
    stream_enabled = 1U;
}

void car_uart_stream_stop(void)
{
    stream_enabled = 0U;
}

uint8 car_uart_stream_is_enabled(void)
{
    return stream_enabled;
}

void car_uart_stream_send_frame(uint8 *image_frame)
{
    if (stream_enabled == 0U || image_frame == 0) return;
    debug_send_buffer(seekfree_assistant_camera_header, sizeof(seekfree_assistant_camera_header));
    debug_send_buffer(image_frame, CAR_IMAGE_WIDTH * CAR_IMAGE_HEIGHT);
}

