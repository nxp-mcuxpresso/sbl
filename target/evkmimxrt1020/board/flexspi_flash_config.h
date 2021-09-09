/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FLEXSPI_FLASH_CONFIG_H_
#define _FLEXSPI_FLASH_CONFIG_H_

#include "sbl.h"
#include "fsl_flexspi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI           FLEXSPI
#define FLASH_SIZE_KB             (COMPONENT_FLASHIAP_SIZE/1024)
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI_AMBA_BASE
#define FLASH_PAGE_SIZE           FLASH_CONFIG_PAGESIZE
#define SECTOR_SIZE               (FLASH_CONFIG_SECTORSIZE) /* 4K */
#define FLEXSPI_BASE_ADDRESS_MASK	(FLASH_SIZE_KB * 0x400 -1)
//
//#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL        7
//#define NOR_CMD_LUT_SEQ_IDX_READ_FAST          13
//#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD     0
//#define NOR_CMD_LUT_SEQ_IDX_READSTATUS         1
//#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE        2
//#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR        3
//#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE 6
//#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD   4
//#define NOR_CMD_LUT_SEQ_IDX_READID             8
//#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG     9
//#define NOR_CMD_LUT_SEQ_IDX_ENTERQPI           10
//#define NOR_CMD_LUT_SEQ_IDX_EXITQPI            11
//#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG      12
//#define NOR_CMD_LUT_SEQ_IDX_ERASECHIP          5
//
//
#define FLASH_QUAD_ENABLE        0x40
#define FLASH_BUSY_STATUS_POL    1
#define FLASH_BUSY_STATUS_OFFSET 0
#define FLASH_ERROR_STATUS_MASK  0x0e
//
//#define FLASH_ERASE_TYPE_SECTOR		0
//#define FLASH_ERASE_TYPE_BLOCK32KB	1
//#define FLASH_ERASE_TYPE_BLOCK64KB	2

/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);

static inline void flexspi_clock_init()
{
#if defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1)
    /* Switch to PLL2 for XIP to avoid hardfault during re-initialize clock. */
    CLOCK_InitSysPfd(kCLOCK_Pfd2, 24);    /* Set PLL2 PFD2 clock 396MHZ. */
    CLOCK_SetMux(kCLOCK_FlexspiMux, 0x2); /* Choose PLL2 PFD2 clock as flexspi source clock. */
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 2);   /* flexspi clock 133M. */
#else
    const clock_usb_pll_config_t g_ccmConfigUsbPll = {.loopDivider = 0U};

    CLOCK_InitUsb1Pll(&g_ccmConfigUsbPll);
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd0, 24);   /* Set PLL3 PFD0 clock 360MHZ. */
    CLOCK_SetMux(kCLOCK_FlexspiMux, 0x3); /* Choose PLL3 PFD0 clock as flexspi source clock. */
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 2);   /* flexspi clock 120M. */
#endif
}

status_t sbl_flash_erase(uint32_t address, size_t len);
status_t sbl_flash_write(uint32_t dstAddr, const void *src, size_t len);
status_t sbl_flash_read(uint32_t dstAddr, void *buf, size_t len);
status_t sbl_flash_init(void);
status_t sbl_flash_read_ipc(uint32_t address, void *buffer, size_t length);

/*${prototype:end}*/

#endif /* _FLEXSPI_FLASH_H_ */
