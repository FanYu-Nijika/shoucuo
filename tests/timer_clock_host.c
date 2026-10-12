#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "IfxStm.h"
#include "Cpu/Std/IfxCpu.h"
#include "zf_driver_timer.h"

static Ifx_STM fake_stm[2];
static float32 fake_frequency[2];
static boolean fake_interrupts_enabled[2] = {TRUE, TRUE};
static IfxCpu_Id fake_core;
static uint32 fake_disable_calls;
static uint32 fake_restore_calls;
static uint32 fake_stm_reads;

Ifx_STM *IfxStm_getAddress(IfxStm_Index index)
{
    assert(index < 2);
    return &fake_stm[index];
}

uint32 IfxStm_getLower(Ifx_STM *stm)
{
    return stm->TIM0.U;
}

float32 IfxStm_getFrequency(Ifx_STM *stm)
{
    return fake_frequency[stm == &fake_stm[1]];
}

uint64 IfxStm_get(Ifx_STM *stm)
{
    assert(fake_interrupts_enabled[fake_core] == FALSE);
    fake_stm_reads++;
    return ((uint64)stm->CAP.U << 32) | stm->TIM0.U;
}

IfxCpu_Id IfxCpu_getCoreId(void)
{
    return fake_core;
}

boolean IfxCpu_disableInterrupts(void)
{
    boolean was_enabled = fake_interrupts_enabled[fake_core];
    fake_interrupts_enabled[fake_core] = FALSE;
    fake_disable_calls++;
    return was_enabled;
}

void IfxCpu_restoreInterrupts(boolean enabled)
{
    if (enabled != FALSE) {
        fake_interrupts_enabled[fake_core] = TRUE;
    }
    fake_restore_calls++;
}

static void set_ticks(IfxCpu_Id core, uint64 ticks)
{
    fake_core = core;
    fake_stm[core].TIM0.U = (uint32)ticks;
    fake_stm[core].CAP.U = (uint32)(ticks >> 32);
}

static void test_full_width_43_second_boundary(void)
{
    fake_frequency[0] = 100000000.0f;
    set_ticks(0, 4294900000ULL);
    assert(system_getval_ms() == 42949U);
    set_ticks(0, 4295100000ULL);
    assert(system_getval_ms() == 42951U);
    assert(system_getval() == 132704U);
}

static void test_common_clock_frequencies(void)
{
    fake_frequency[0] = 100000000.0f;
    set_ticks(0, 312500000ULL);
    assert(system_getval_ms() == 3125U);

    fake_frequency[0] = 200000000.0f;
    set_ticks(0, 625000000ULL);
    assert(system_getval_ms() == 3125U);

    fake_frequency[0] = 80500000.0f;
    set_ticks(0, 140875000ULL);
    assert(system_getval_ms() == 1750U);
}

static void test_start_origin_and_core_independence(void)
{
    fake_frequency[0] = 100000000.0f;
    set_ticks(0, 123456789ULL);
    system_start();
    set_ticks(0, 135802467ULL);
    assert(system_getval_ms() == 123U);

    fake_frequency[1] = 200000000.0f;
    set_ticks(1, 10000000000ULL);
    system_start();
    set_ticks(0, 235802467ULL);
    assert(system_getval_ms() == 1123U);
    set_ticks(1, 10500000000ULL);
    assert(system_getval_ms() == 2500U);
}

static void test_uint32_millisecond_wrap(void)
{
    fake_frequency[0] = 100000000.0f;
    set_ticks(0, 0);
    system_start();
    set_ticks(0, (uint64)4294967294ULL * 100000ULL);
    assert(system_getval_ms() == 4294967294U);
    set_ticks(0, (uint64)4294967295ULL * 100000ULL);
    assert(system_getval_ms() == 4294967295U);
    set_ticks(0, (uint64)4294967296ULL * 100000ULL);
    assert(system_getval_ms() == 0U);
}

static void test_interrupt_state_restoration(void)
{
    fake_frequency[0] = 100000000.0f;
    set_ticks(0, 0);
    system_start();
    set_ticks(0, 100000000ULL);
    fake_interrupts_enabled[0] = TRUE;
    assert(system_getval_ms() == 1000U);
    assert(fake_interrupts_enabled[0] == TRUE);

    fake_interrupts_enabled[0] = FALSE;
    assert(system_getval_ms() == 1000U);
    assert(fake_interrupts_enabled[0] == FALSE);

    fake_interrupts_enabled[0] = TRUE;
    set_ticks(0, 200000000ULL);
    system_start();
    assert(fake_interrupts_enabled[0] == TRUE);
}

int main(void)
{
    test_full_width_43_second_boundary();
    test_common_clock_frequencies();
    test_start_origin_and_core_independence();
    test_uint32_millisecond_wrap();
    test_interrupt_state_restoration();
    assert(fake_disable_calls == fake_restore_calls);
    assert(fake_stm_reads == fake_disable_calls);
    puts("timer clock tests passed");
    return 0;
}
