/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "fsl_device_registers.h"
#include "bootloader/bl_cache.h"
#include "lmem/fsl_cache.h"
#include "bootloader_common.h"
#include "utilities/fsl_rtos_abstraction.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern bool is_cm4_boot(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static cache_context_t s_cacheContext BL_SECTION(".noinit");

/*******************************************************************************
 * Code
 ******************************************************************************/
void bl_cache_init(cache_context_t ctx)
{
    s_cacheContext.allowEnableDCache = ctx.allowEnableDCache;
    s_cacheContext.allowEnableICache = ctx.allowEnableICache;
}

void bl_icache_enable(void)
{
    if (s_cacheContext.allowEnableICache != kCache_EnableAllowed)
    {
        debug_printf("Bootloader: Trying to enable I-Cache, but it is not allowed eFUSE setting\n");
        return;
    }

    lock_acquire();
    if (is_cm4_boot())
    {
        debug_printf("Bootloader: Enabling Code Cache on CM4 core\n");
        if (!(LMEM->PCCCR & LMEM_PCCCR_ENCACHE_MASK))
        {
            L1CACHE_EnableCodeCache();
        }
    }
    else
    {
        debug_printf("Bootloader: Enabling I-Cache on CM7 core\n");
        if (!(SCB->CCR & SCB_CCR_IC_Msk))
        {
            SCB_EnableICache();
        }
    }
    lock_release();
}

void bl_icache_disable(void)
{
    lock_acquire();
    if (is_cm4_boot())
    {
        debug_printf("Bootloader: Disabling Code Cache on CM4 core\n");
        if (LMEM->PCCCR & LMEM_PCCCR_ENCACHE_MASK)
        {
            L1CACHE_DisableCodeCache();
        }
    }
    else
    {
        debug_printf("Bootloader: Disabling I-Cache on CM7 core\n");
        if (SCB->CCR & SCB_CCR_IC_Msk)
        {
            SCB_DisableICache();
        }
    }
    lock_release();
}

void bl_icache_invalidate(void)
{
    lock_acquire();
    if (is_cm4_boot())
    {
        if (LMEM->PCCCR & LMEM_PCCCR_ENCACHE_MASK)
        {
            L1CACHE_InvalidateCodeCache();
        }
    }
    else
    {
        if (SCB->CCR & SCB_CCR_IC_Msk)
        {
            SCB_InvalidateICache();
        }
    }
    lock_release();
}

void bl_dcache_enable(void)
{
    if (s_cacheContext.allowEnableDCache != kCache_EnableAllowed)
    {
        debug_printf("Bootloader: Trying to enable D-Cache, but it is not allowed eFUSE setting\n");
        return;
    }

    lock_acquire();
    if (is_cm4_boot())
    {
        debug_printf("Bootloader: Enabling System Cache on CM4 core\n");
        if (!(LMEM->PSCCR & LMEM_PSCCR_ENCACHE_MASK))
        {
            L1CACHE_EnableSystemCache();
        }

        // Enable Write Buffer
        if (!(LMEM->PSCCR & LMEM_PSCCR_ENWRBUF_MASK))
        {
            LMEM->PSCCR |= LMEM_PSCCR_ENWRBUF_MASK;
        }
    }
    else
    {
        debug_printf("Bootloader: Enabling D-Cache on CM7 core\n");
        if (!(SCB->CCR & SCB_CCR_DC_Msk))
        {
            SCB_EnableDCache();
        }
    }
    lock_release();
}

void bl_dcache_disable(void)
{
    lock_acquire();
    if (is_cm4_boot())
    {
        debug_printf("Bootloader: Disabling System Cache on CM4 core\n");
        if (LMEM->PSCCR & LMEM_PSCCR_ENCACHE_MASK)
        {
            L1CACHE_DisableSystemCache();
        }
    }
    else
    {
        debug_printf("Bootloader: Disabling D-Cache on CM7 core\n");
        if (SCB->CCR & SCB_CCR_DC_Msk)
        {
            SCB_DisableDCache();
        }
    }
    lock_release();
}

void bl_dcache_clean(void)
{
    lock_acquire();
    if (is_cm4_boot())
    {
        if (LMEM->PSCCR & LMEM_PSCCR_ENCACHE_MASK)
        {
            L1CACHE_CleanSystemCache();
        }
    }
    else
    {
        if (SCB->CCR & SCB_CCR_DC_Msk)
        {
            SCB_CleanDCache();
        }
    }
    lock_release();
}
void bl_dcache_invalidate(void)
{
    lock_acquire();
    if (is_cm4_boot())
    {
        if (LMEM->PSCCR & LMEM_PSCCR_ENCACHE_MASK)
        {
            L1CACHE_InvalidateSystemCache();
        }
    }
    else
    {
        if (SCB->CCR & SCB_CCR_DC_Msk)
        {
            SCB_InvalidateDCache();
        }
    }
    lock_release();
}

void bl_dcache_clean_invalidate(void)
{
    lock_acquire();
    if (is_cm4_boot())
    {
        if (LMEM->PSCCR & LMEM_PSCCR_ENCACHE_MASK)
        {
            debug_printf("Bootloader: Flushing System Cache on CM4 core\n");
            L1CACHE_CleanInvalidateSystemCache();
        }
    }
    else
    {
        if (SCB->CCR & SCB_CCR_DC_Msk)
        {
            debug_printf("Bootloader: Flushing D-Cache on CM7 core\n");
            SCB_CleanInvalidateDCache();
        }
    }
    lock_release();
}
