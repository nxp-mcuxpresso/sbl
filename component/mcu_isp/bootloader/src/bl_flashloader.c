/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "utilities/vector_table_info.h"
#include "bootloader/flashloader_image.h"

#if DEBUG
#include "debug/flashloader_image.c"
#else
#include "release/flashloader_image.c"
#endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

void bootloader_cleanup(void);
void bootloader_run(void);
int main(void);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

// Not used, but needed to resolve reference in startup.s.
uint32_t g_bootloaderTree;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void bootloader_cleanup()
{
    // Turn off interrupts.
    __disable_irq();

    // Set the VTOR to default.
    SCB->VTOR = kDefaultVectorTableAddress;

    // Memory barriers for good measure.
    __ISB();
    __DSB();
}

// @brief Run the bootloader.
void bootloader_run(void)
{
    // Copy flashloader image to RAM.
    memcpy((void *)g_flashloaderBase, g_flashloaderImage, g_flashloaderSize);

    bootloader_cleanup();

    // Set main stack pointer and process stack pointer.
    __set_MSP(g_flashloaderStack);
    __set_PSP(g_flashloaderStack);

    // Jump to flashloader entry point, does not return.
    void (*entry)(void) = (void (*)(void))g_flashloaderEntry;
    entry();
}

// @brief Main bootloader entry point.
int main(void)
{
    bootloader_run();

    // Should never end up here.
    while (1)
        ;
}

// Needed to resolve library reference.
int __write(void)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
