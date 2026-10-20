#include "hdu_dma.h"
#include "my_imagetransfer.h"
#include "board_pins.h"
#include "zf_driver_uart.h"
#include "zf_driver_delay.h"

void My_Data_Uart_Init(void)
{
    uart_init(BOARD_HOST_UART, 640000, BOARD_HOST_UART_TX_PIN, BOARD_HOST_UART_RX_PIN);
}

void send_to_computer(void)
{
    /* Export runs after STOP, with interrupts enabled; no extra DMA channel is needed. */
    uart_write_buffer(BOARD_HOST_UART, My_Send_Data, Send_Data_Len);
    /* Let the final hardware FIFO bytes leave before the last frame restores the baud rate. */
    system_delay_ms(1);
}

void My_Data_Uart_Restore(void)
{
    uart_init(BOARD_HOST_UART, 115200, BOARD_HOST_UART_TX_PIN, BOARD_HOST_UART_RX_PIN);
}
