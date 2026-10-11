#ifndef BALANCE_VOFA_HOST_STUBS_H_
#define BALANCE_VOFA_HOST_STUBS_H_

#define _zf_common_headfile_h_
#define _zf_common_typedef_h_

#include "balance_host_stubs.h"

extern uint32 host_time_ms;
extern uint8 host_interrupts_enabled;

uint32 system_getval_ms(void);
uint32 debug_read_ring_buffer(uint8 *buffer, uint32 length);
uint8 car_uart_stream_is_enabled(void);

#endif
