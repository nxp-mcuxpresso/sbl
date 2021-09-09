/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(BOOTLOADER_HOST)
#include "fsl_device_registers.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/vector_table_info.h"
#if BL_FEATURE_HAS_INTERNAL_FLASH
#include "fsl_iap.h"
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#include "microseconds/microseconds.h"
#include "bootloader_common.h"

#endif // BOOTLOADER_HOST

#include "bootloader/bl_shutdown_cleanup.h"
#include "bootloader/bl_context.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#if BL_FEATURE_HAS_INTERNAL_FLASH
void flash_cache_clear(flash_config_t *config);
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See bl_shutdown_cleanup.h for documentation of this function.
void shutdown_cleanup(shutdown_type_t shutdown)
{
#if !defined(BOOTLOADER_HOST)
    if (shutdown != kShutdownType_Reset)
    {
// Clear (flush) the flash cache.
#if BL_FEATURE_HAS_INTERNAL_FLASH
        flash_cache_clear(NULL);
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
    }

    if (shutdown != kShutdownType_Cleanup)
    {
        // Shutdown all peripherals because they could be active
        uint32_t i;
        for (i = 0; g_peripherals[i].typeMask != 0; i++)
        {
            if (g_peripherals[i].controlInterface->shutdown)
            {
                g_peripherals[i].controlInterface->shutdown(&g_peripherals[i]);
            }
        }
    }

    // If we are permanently exiting the bootloader, there are a few extra things to do.
    if (shutdown == kShutdownType_Shutdown)
    {
        // Turn off global interrupt
        lock_acquire();

        // Shutdown microseconds driver.
        microseconds_shutdown();

// Disable force ROM.
#if defined(RCM_FM_FORCEROM_MASK)
        RCM->FM = ((~RCM_FM_FORCEROM_MASK) & RCM->FM) | RCM_FM_FORCEROM(0);
#elif defined(SMC_FM_FORCECFG_MASK)
#if defined(SMC0)
        SMC0->FM = ((~SMC_FM_FORCECFG_MASK) & SMC0->FM) | SMC_FM_FORCECFG(0);
#else
        SMC->FM = ((~SMC_FM_FORCECFG_MASK) & SMC->FM) | SMC_FM_FORCECFG(0);
#endif
#endif // defined(RCM_FM_FORCEROM_MASK)

// Clear status register (bits are w1c).
#if defined(RCM_MR_BOOTROM_MASK)
        RCM->MR = ((~RCM_MR_BOOTROM_MASK) & RCM->MR) | RCM_MR_BOOTROM(3);
#elif defined(SMC_MR_BOOTCFG_MASK)
#if defined(SMC0)
        SMC0->MR = ((~SMC_MR_BOOTCFG_MASK) & SMC0->MR) | SMC_MR_BOOTCFG(3);
#else
        SMC->MR = ((~SMC_MR_BOOTCFG_MASK) & SMC->MR) | SMC_MR_BOOTCFG(3);
#endif
#endif // defined(RCM_MR_BOOTROM_MASK)

        init_interrupts();

        // Set the VTOR to default.
        SCB->VTOR = kDefaultVectorTableAddress;

        // Restore clock to default before leaving bootloader.
        configure_clocks(kClockOption_ExitBootloader);

        // De-initialize hardware such as disabling port clock gate
        deinit_hardware();

        // Restore global interrupt.
        __enable_irq();

#if BL_FEATURE_BYPASS_WATCHDOG
        // De-initialize watchdog
        bootloader_watchdog_deinit();
#endif // BL_FEATURE_BYPASS_WATCHDOG
    }

    // Memory barriers for good measure.
    __ISB();
    __DSB();
#endif
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
