#ifndef CODE_BALANCE_RUNTIME_H_
#define CODE_BALANCE_RUNTIME_H_

#include "zf_common_typedef.h"

extern volatile uint32 car_balance_max_us;
extern volatile uint32 car_balance_period_us;
extern volatile uint32 car_balance_overruns;
extern volatile uint32 car_uptime_ms;
extern volatile uint8 car_balance_timing_reset;

void balance_runtime_init(void);
void balance_runtime_update_5ms(void);
void balance_runtime_stop(void);

#endif
