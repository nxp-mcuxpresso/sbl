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
#if defined(FSL_FEATURE_SOC_DCP_COUNT) && (FSL_FEATURE_SOC_DCP_COUNT > 0)
#include "fsl_dcp.h"
#endif
#if defined(FSL_FEATURE_SOC_TRNG_COUNT) && (FSL_FEATURE_SOC_TRNG_COUNT > 0)
#include "fsl_trng.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifdef SOC_REMAP_ENABLE
#define REMAPADDRSTART  0x400AC078
#define REMAPADDREND    0x400AC07C
#define REMAPADDROFFSET 0x400AC080
#endif
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
#if (defined(COMPONENT_MCU_ISP))
    bool isInfiniteIsp = false;
    (void)isp_kboot_main(isInfiniteIsp);
#endif

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    SCB_DisableDCache();

    PRINTF("hello sbl.\r\n");

    (void)sbl_boot_main();
    
    return 0;
}

void SBL_DisablePeripherals(void)
{
    DbgConsole_Deinit();
#if defined(COMPONENT_MCUBOOT_SECURE)
#if defined(FSL_FEATURE_SOC_DCP_COUNT) && (FSL_FEATURE_SOC_DCP_COUNT > 0)
    DCP_Deinit(DCP);
#endif
#if defined(FSL_FEATURE_SOC_TRNG_COUNT) && (FSL_FEATURE_SOC_TRNG_COUNT > 0)
    TRNG_Deinit(TRNG);
#endif
#endif
}

#ifdef SOC_REMAP_ENABLE
void SBL_EnableRemap(uint32_t start_addr, uint32_t end_addr, uint32_t off)
{
    uint32_t * remap_start  = (uint32_t *)REMAPADDRSTART;
    uint32_t * remap_end    = (uint32_t *)REMAPADDREND;
    uint32_t * remap_offset = (uint32_t *)REMAPADDROFFSET;
    
    *remap_start = start_addr;
    *remap_end = end_addr;
    *remap_offset = off;
}

void SBL_DisableRemap(void)
{
    uint32_t * remap_start  = (uint32_t *)REMAPADDRSTART;
    uint32_t * remap_end    = (uint32_t *)REMAPADDREND;
    uint32_t * remap_offset = (uint32_t *)REMAPADDROFFSET;
    
    *remap_start = 0;
    *remap_end = 0;
    *remap_offset = 0;
}
#endif
