/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FLEXSPI_FLASH_CONFIG_H_
#define _FLEXSPI_FLASH_CONFIG_H_

#include "fsl_flexspi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI FLEXSPI1
#define FLASH_SIZE_KB (COMPONENT_FLASHIAP_SIZE/1024) /* 16Mb/KByte */
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI1_AMBA_BASE
#define FLASH_PAGE_SIZE FLASH_CONFIG_PAGESIZE
#define SECTOR_SIZE (FLASH_CONFIG_SECTORSIZE) /* 4K */
#define EXAMPLE_FLEXSPI_CLOCK kCLOCK_Flexspi1
#define FLEXSPI_BASE_ADDRESS_MASK	(FLASH_SIZE_KB * 0x400 -1)

//#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL 7
//#define NOR_CMD_LUT_SEQ_IDX_READ_FAST 13
//#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD 0
//#define NOR_CMD_LUT_SEQ_IDX_READSTATUS 1
//#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE 2
//#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR 3
//#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE 6
//#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD 4
//#define NOR_CMD_LUT_SEQ_IDX_READID 8
//#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG 9
//#define NOR_CMD_LUT_SEQ_IDX_ENTERQPI 10
//#define NOR_CMD_LUT_SEQ_IDX_EXITQPI 11
//#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG 12
//#define NOR_CMD_LUT_SEQ_IDX_ERASECHIP 5
//
//#define CUSTOM_LUT_LENGTH 60
#define FLASH_QUAD_ENABLE 0x40
#define FLASH_BUSY_STATUS_POL 1
#define FLASH_BUSY_STATUS_OFFSET 0

/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
static inline void flexspi_clock_init(void)
{
    /*Clock setting for flexspi1*/
    CLOCK_SetRootClockDiv(kCLOCK_Root_Flexspi1, 2);
    CLOCK_SetRootClockMux(kCLOCK_Root_Flexspi1, 0);
}

status_t sbl_flash_erase(uint32_t address, size_t len);
status_t sbl_flash_write(uint32_t dstAddr, const void *src, size_t len);
status_t sbl_flash_read(uint32_t dstAddr, void *buf, size_t len);
status_t sbl_flash_init(void);

/*${prototype:end}*/

#endif /* _FLEXSPI_FLASH_H_ */
