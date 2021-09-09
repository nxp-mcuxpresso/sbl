/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FLEXSPI_FLASH_CONFIG_H_
#define _FLEXSPI_FLASH_CONFIG_H_

/*${header:start}*/
#include "fsl_flexspi.h"
/*${header:end}*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI                        FLEXSPI
#define FLASH_SIZE_KB                          (COMPONENT_FLASHIAP_SIZE/1024) //0x10000
#define EXAMPLE_FLEXSPI_AMBA_BASE              FlexSPI_AMBA_BASE
#define FLASH_PAGE_SIZE                        FLASH_CONFIG_PAGESIZE  //512
#define SECTOR_SIZE                            (FLASH_CONFIG_SECTORSIZE)  //0x40000
#define EXAMPLE_FLEXSPI_CLOCK                  kCLOCK_FlexSpi
#define FLEXSPI_BASE_ADDRESS_MASK			   (FLASH_SIZE_KB * 0x400 -1)

#define FLASH_BUSY_STATUS_POL 1
#define FLASH_BUSY_STATUS_OFFSET 0
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);

static inline void flexspi_clock_init(void)
{
    // Set flexspi root clock to 166MHZ.
    const clock_usb_pll_config_t g_ccmConfigUsbPll = {.loopDivider = 0U};

    CLOCK_InitUsb1Pll(&g_ccmConfigUsbPll);
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd0, 26);   /* Set PLL3 PFD0 clock 332MHZ. */
    CLOCK_SetMux(kCLOCK_FlexspiMux, 0x3); /* Choose PLL3 PFD0 clock as flexspi source clock. */
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 3);   /* flexspi clock 83M, DDR mode, internal clock 42M. */
}

static inline void flexspi_clock_set(void)
{
    /* Set the clock to run the program operation. */
    /* Wait for bus idle before change flash configuration. */
    while (!FLEXSPI_GetBusIdleStatus(EXAMPLE_FLEXSPI))
    {
    }    
    FLEXSPI_Enable(EXAMPLE_FLEXSPI, false);
    CLOCK_DisableClock(EXAMPLE_FLEXSPI_CLOCK);
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 3); /* flexspi clock 332M, DDR mode, internal clock 166M. */
    CLOCK_EnableClock(EXAMPLE_FLEXSPI_CLOCK);
    FLEXSPI_Enable(EXAMPLE_FLEXSPI, true);
    /* Do software reset. */
    FLEXSPI_SoftwareReset(EXAMPLE_FLEXSPI);
}

static inline void flexspi_clock_update(void)
{
    /* Program finished, speed the clock to 166M. */
    /* Wait for bus idle before change flash configuration. */
    while (!FLEXSPI_GetBusIdleStatus(EXAMPLE_FLEXSPI))
    {
    }
    FLEXSPI_Enable(EXAMPLE_FLEXSPI, false);
    CLOCK_DisableClock(EXAMPLE_FLEXSPI_CLOCK);
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 0); /* flexspi clock 332M, DDR mode, internal clock 166M. */
    CLOCK_EnableClock(EXAMPLE_FLEXSPI_CLOCK);
    FLEXSPI_Enable(EXAMPLE_FLEXSPI, true);
    /* Do software reset. */
    FLEXSPI_SoftwareReset(EXAMPLE_FLEXSPI);
}
/*${prototype:end}*/
status_t sbl_flash_erase(uint32_t address, size_t len);
status_t sbl_flash_write(uint32_t dstAddr, const void *src, size_t len);
status_t sbl_flash_read(uint32_t dstAddr, void *buf, size_t len);
status_t sbl_flash_init(void);
status_t sbl_flash_read_ipc(uint32_t addr, void *buffer, size_t length);

#endif /* _FLEXSPI_FLASH_CONFIG_H_ */
