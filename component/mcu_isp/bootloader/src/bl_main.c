/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <sbl.h>
#include <stdbool.h>
#include "utilities/fsl_assert.h"
#include "bootloader/bl_context.h"
#include "bootloader/bl_peripheral.h"
#include "bootloader/bl_shutdown_cleanup.h"
#include "bootloader_common.h"
#include "microseconds/microseconds.h"
#include "bootloader/bootloader.h"
#include "property/property.h"
#include "utilities/vector_table_info.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "memory/memory.h"
//! @addtogroup bl_core
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
static void bootloader_init(void);
static void bootloader_run(bool isInfiniteIsp);

int main(void);

//! @brief Initialize the bootloader and peripherals.
//!
//! This function initializes hardware and clocks, loads user configuration data, and initialzes
//! a number of drivers. It then enters the active peripheral detection phase by calling
//! get_active_peripheral(). Once the peripheral is detected, the packet and comand interfaces
//! are initialized.
//!
//! Note that this routine may not return if peripheral detection times out and the bootloader
//! jumps directly to the user application in flash.
static void bootloader_init(void)
{
    // Init the global irq lock
    lock_init();

#if BL_FEATURE_STACK_CHECK
    stack_check_init();
#endif

    // Init pinmux and other hardware setup.
    init_hardware();

    // Load the user configuration data so that we can configure the clocks
    g_bootloaderContext.propertyInterface->load_user_config();

    // Start the lifetime counter
    microseconds_init();

#if BL_FEATURE_BYPASS_WATCHDOG
    bootloader_watchdog_init();
#endif // BL_FEATURE_BYPASS_WATCHDOG

    // Init address range of flash array, SRAM_L and SRAM U.
    g_bootloaderContext.memoryInterface->init();

    // Fully init the property store.
    g_bootloaderContext.propertyInterface->init();
}

//! @brief BootMode selection.
//!
//! Go to different boot modes accoring to the boot mode selection.
static void bootloader_run(bool isInfiniteIsp)
{
    if (go_passive_boot(isInfiniteIsp))
    {
        // If active peripheral detected during passive boot, exit and go into isp mode.
        go_isp_boot();
    }
    else
    {
        // Shut down the bootloader and return to reset-type state prior to low
        // power entry
        shutdown_cleanup(kShutdownType_Shutdown);
    }
}

int go_passive_boot(bool isInfiniteIsp)
{
    uint32_t milliseconds = 0; // Must be initialized to zero.
    if (!isInfiniteIsp)
    {
        bootloader_configuration_data_t *configurationData =
            &g_bootloaderContext.propertyInterface->store->configurationData;

#if !BL_FEATURE_TIMEOUT
//        const uint64_t ticksPerMillisecond = microseconds_convert_to_ticks(1000);

        // If the boot to rom option is not set AND there is a valid jump application determine the timeout value
        if (!is_boot_pin_asserted())
        {
            // Calculate how many ticks we need to wait based on the bootloader config. Check to see if
            // there is a valid configuration data value for the timeout. If there's not, use the
            // default timeout value.
            if (configurationData->peripheralDetectionTimeoutMs != 0xFFFF)
            {
                milliseconds = configurationData->peripheralDetectionTimeoutMs;
            }
            else
            {
                milliseconds = (ISP_TIMEOUT * 1000);
            }
        }
#endif // !BL_FEATURE_TIMEOUT
    }

    // Wait for a peripheral to become active.
    g_bootloaderContext.activePeripheral = get_active_peripheral(milliseconds);

    return (int)(g_bootloaderContext.activePeripheral != NULL);
}

#if BL_FEATURE_ISP_BOOT
void go_isp_boot(void)
{
    // Validate required active peripheral interfaces.
    assert(g_bootloaderContext.activePeripheral->controlInterface);

    // Init the active peripheral.
    if (g_bootloaderContext.activePeripheral->byteInterface &&
        g_bootloaderContext.activePeripheral->byteInterface->init)
    {
        g_bootloaderContext.activePeripheral->byteInterface->init(g_bootloaderContext.activePeripheral);
    }
    if (g_bootloaderContext.activePeripheral->packetInterface &&
        g_bootloaderContext.activePeripheral->packetInterface->init)
    {
        g_bootloaderContext.activePeripheral->packetInterface->init(g_bootloaderContext.activePeripheral);
    }

    // Initialize the command processor component.
    g_bootloaderContext.commandInterface->init();

    // Infinitely calls the command interface and active peripheral control interface pump routines.
    const peripheral_descriptor_t *activePeripheral = g_bootloaderContext.activePeripheral;

    assert(g_bootloaderContext.commandInterface->pump);

    // Read and execute commands.
    while (1)
    {
        g_bootloaderContext.commandInterface->pump();

        // Pump the active peripheral.
        if (activePeripheral->controlInterface->pump)
        {
            activePeripheral->controlInterface->pump(activePeripheral);
        }
    }
}
#endif // #if BL_FEATURE_ISP_BOOT

//! @brief Entry point for the bootloader.
int isp_kboot_main(bool isInfiniteIsp)
{
    if (!isp_cleanup_exit(&isInfiniteIsp))
    {
        if ((!isInfiniteIsp) && (!ISP_TIMEOUT))
        {
            return 0;
        }
        bootloader_init();
        bootloader_run(isInfiniteIsp);
        isp_cleanup_enter(CLEANUP_ISP_TO_SBL);
    }
    
    return 0;
}

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
