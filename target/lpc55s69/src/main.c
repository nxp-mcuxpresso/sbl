/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <sbl.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if (defined(COMPONENT_MCU_ISP))
extern int isp_kboot_main(bool isInfiniteIsp);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    BOARD_InitBootClocks();

#if (defined(COMPONENT_MCU_ISP))
    bool isInfiniteIsp = false;
    (void)isp_kboot_main(isInfiniteIsp);
#endif

    BOARD_InitPins();
    BOARD_InitDebugConsole();

    PRINTF("hello sbl.\r\n");

    (void)sbl_boot_main();
    
    return 0;
}


void SBL_DisablePeripherals(void)
{
    DbgConsole_Deinit();
}



