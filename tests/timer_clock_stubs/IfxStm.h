#ifndef TIMER_CLOCK_IFXSTM_H
#define TIMER_CLOCK_IFXSTM_H

#include <stdint.h>

typedef uint64_t uint64;
typedef uint32_t uint32;
typedef float float32;
typedef uint32_t IfxStm_Index;

typedef struct {
    struct { uint32 U; } TIM0;
    struct { uint32 U; } CAP;
} Ifx_STM;

Ifx_STM *IfxStm_getAddress(IfxStm_Index index);
uint32 IfxStm_getLower(Ifx_STM *stm);
float32 IfxStm_getFrequency(Ifx_STM *stm);
uint64 IfxStm_get(Ifx_STM *stm);

#endif
