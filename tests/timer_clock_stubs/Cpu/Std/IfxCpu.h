#ifndef TIMER_CLOCK_IFXCPU_H
#define TIMER_CLOCK_IFXCPU_H

#include <stdint.h>

typedef uint32_t IfxCpu_Id;
typedef uint8_t boolean;

IfxCpu_Id IfxCpu_getCoreId(void);
boolean IfxCpu_disableInterrupts(void);
void IfxCpu_restoreInterrupts(boolean enabled);

#endif
