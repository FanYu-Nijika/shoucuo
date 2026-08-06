#ifndef CAR_UART_STREAM_H
#define CAR_UART_STREAM_H

#include "zf_common_typedef.h"

void car_uart_stream_init(void);
void car_uart_stream_start(void);
void car_uart_stream_stop(void);
uint8 car_uart_stream_is_enabled(void);
void car_uart_stream_send_frame(uint8 *image_frame);

#endif

