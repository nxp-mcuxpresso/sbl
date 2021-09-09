/*
* Copyright 2014-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2018 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*
*/

#include "bootloader/bootloader.h"
#include "bootloader_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
#define SRC_SCR_CORE0_RST_MASK 0x3U
void target_reset(void)
{
#warning Can clean up other NVIC_SystemReset in the bootloader to use this function instead

    // Prepare for shutdown.
    shutdown_cleanup(kShutdownType_Reset);

#ifndef BL_TARGET_FPGA
    NVIC_SystemReset();
#else
    SRC->SCR |= SRC_SCR_CORE0_RST_MASK;
#endif

    // Make sure reset is performed.
    __ISB();

    // Does not get here.
    assert(0);
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
