#ifndef VOFA_DEBUG_HOST_STUBS_H_
#define VOFA_DEBUG_HOST_STUBS_H_

#include <stdint.h>

#define CODE_BALANCE_RUNTIME_H_
#define _zf_common_interrupt_h_
#define _zf_common_typedef_h_
#define _zf_driver_timer_h_
#define _zf_driver_uart_h_

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

#define UART_2 2
#define UART2_TX_P10_5 15
#define UART2_RX_P10_6 16
#define __dsync() ((void)0)

extern volatile uint8 balance_start_pending;
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 state);
uint32 system_getval_ms(void);
void uart_init(int uart, uint32 baud, int tx_pin, int rx_pin);
void uart_rx_interrupt(int uart, uint32 enabled);
void uart_tx_interrupt(int uart, uint32 enabled);
uint8 uart_query_byte(int uart, uint8 *data);
uint8 uart_write_buffer_nonblocking(int uart, const uint8 *data, uint32 length);

#endif
