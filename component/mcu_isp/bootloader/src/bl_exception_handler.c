/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "fsl_device_registers.h"

#ifndef BL_FETAURE_USE_STD_EXCEPTION_HANDLER
#define BL_FETAURE_USE_STD_EXCEPTION_HANDLER (1)
#endif

typedef struct _stack_frame
{
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} stack_frame_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
extern void MemManage_Handler(void);
extern void BusFault_Handler(void);
extern void UsageFault_Handler(void);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if BL_FETAURE_USE_STD_EXCEPTION_HANDLER
//! @brief HardFault_Handler
void HardFault_Handler(void)
{
    // Reset MCU
    NVIC_SystemReset();
}
#endif

void bl_hardfault_handler(stack_frame_t *frame)
{
    // Show call stack of HardFault
    debug_printf("pc = %x\n", frame->pc);
    debug_printf("psr = %x\n", frame->psr);
    debug_printf("r0 = %x\n", frame->r0);
    debug_printf("r1 = %x\n", frame->r1);
    debug_printf("r2 = %x\n", frame->r2);
    debug_printf("r3 = %x\n", frame->r3);
    debug_printf("r12 = %x\n", frame->r12);
    debug_printf("lr = %x\n", frame->lr);

#if (__CORTEX_M >= 3)
    // Show Fault Flags
    debug_printf("SCB->HFSR=%x\n", SCB->HFSR);

    // If it is a force HardFault, usually, it means that the HardFault was escalated from other core exceptions
    if (SCB->HFSR & SCB_HFSR_FORCED_Msk)
    {
        debug_printf("SCB->CFSR=%x\n", SCB->CFSR);
        if (SCB->CFSR & SCB_CFSR_MEMFAULTSR_Msk)
        {
            debug_printf("HardFault was escalated from MemUsageFault\n");
            MemManage_Handler();
        }
        if (SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk)
        {
            debug_printf("HardFault was escalated from BusFault\n");
            BusFault_Handler();
        }
        if (SCB->CFSR & SCB_CFSR_USGFAULTSR_Msk)
        {
            debug_printf("HardFault was escalated from UsageFault\n");
            UsageFault_Handler();
        }
    }
#endif // #if (__CORTEX_M >= 3)

    // Reset MCU
    NVIC_SystemReset();
}

/* Functions below only apply to MCUs that belongs to ARM Cortex-M3 or later family */
#if (__CORTEX_M >= 3)
//! @brief MPU Fault Handler
void MemManage_Handler(void)
{
    uint32_t mmfsr = SCB->CFSR & 0xFF;
    debug_printf("SCB->CFSR[MMFSR]=%x\n", mmfsr);

    if (mmfsr & 0x01)
    {
        debug_printf("Instruction access violation flag is asserted\n");
    }
    if (mmfsr & 0x02)
    {
        debug_printf("IDtat access violation flag is asserted\n");
    }
    if (mmfsr & 0x04)
    {
        debug_printf("Instruction access violation flag is asserted\n");
    }
    if (mmfsr & 0x80)
    {
        debug_printf("SCB->MMFAR=%x\n", SCB->MMFAR);
    }

#if (__CORTEX_M == 7)
    uint32_t abfsr = SCB->ABFSR;
    debug_printf("SCB->ABFSR=%x\n", abfsr);

    if (abfsr & 0x01)
    {
        debug_printf("Async fault on ITCM interface\n");
    }
    if (abfsr & 0x02)
    {
        debug_printf("Async fault on DTCM interface\n");
    }
    if (abfsr & 0x04)
    {
        debug_printf("Async fault on AHBP interface\n");
    }
    if (abfsr & 0x08)
    {
        debug_printf("Async fault on AXIM interface\n");
    }
    if (abfsr & 0x10)
    {
        debug_printf("Async fault on EPPB interface\n");
    }
#endif

    // Reset MCU
    if (SCB->SHCSR & SCB_SHCSR_MEMFAULTACT_Msk)
    {
        NVIC_SystemReset();
    }
}

//! @brief Bus Fault Handler
void BusFault_Handler(void)
{
    uint32_t bfsr = (SCB->CFSR >> 8) & 0xFF;
    debug_printf("SCB->CFSR[BFSR]=%x\n", bfsr);

    if (bfsr & 0x01)
    {
        debug_printf("Instruction Bus Error\n");
    }
    if (bfsr & 0x02)
    {
        debug_printf("Precise Data Bus Error\n");
    }
    if (bfsr & 0x04)
    {
        debug_printf("Imprecise Data Bus Error\n");
    }
    if (bfsr & 0x80)
    {
        debug_printf("SCB->BFAR=%x\n", SCB->BFAR);
    }

    // Reset MCU
    if (SCB->SHCSR & SCB_SHCSR_BUSFAULTACT_Msk)
    {
        NVIC_SystemReset();
    }
}

//! @brief Usage Fault Handler
void UsageFault_Handler(void)
{
    uint32_t ufsr = (SCB->CFSR >> 16) & 0xFFFF;
    debug_printf("SCB->CFSR[UFSR]=%x\n", ufsr);

    if (ufsr & 0x01)
    {
        debug_printf("Undefined instruction\n");
    }
    if (ufsr & 0x02)
    {
        debug_printf("Invalid State\n");
    }
    if (ufsr & 0x04)
    {
        debug_printf("Invalid PC load UsageFault\n");
    }
    if (ufsr & 0x08)
    {
        debug_printf("No coprocessor\n");
    }
    if (ufsr & 0x100)
    {
        debug_printf("Unaliged access UsageFault\n");
    }

    // Reset MCU
    if (SCB->SHCSR & SCB_SHCSR_USGFAULTACT_Msk)
    {
        NVIC_SystemReset();
    }
}

#endif // #if (__CORTEX_M >= 3)

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
