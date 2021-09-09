/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FLEXSPI_FLASH_CONFIG_H_
#define _FLEXSPI_FLASH_CONFIG_H_

/*${header:start}*/
#include "fsl_cache.h"
/*${header:end}*/
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI                 FLEXSPI0
#define FLASH_SIZE_KB                   (COMPONENT_FLASHIAP_SIZE/1024) //0x1000 /* 0x2000= 64Mb/KByte */
#define EXAMPLE_FLEXSPI_AMBA_BASE       FlexSPI0_AMBA_BASE
#define FLASH_PAGE_SIZE                 FLASH_CONFIG_PAGESIZE  //256
#define SECTOR_SIZE                     (FLASH_CONFIG_SECTORSIZE) //0x1000 /* 4K */
#define FLASH_PORT                      kFLEXSPI_PortA1
#define FLEXSPI_BASE_ADDRESS_MASK (FLASH_SIZE_KB * 0x400 -1)


#define FLASH_BUSY_STATUS_POL    1
#define FLASH_BUSY_STATUS_OFFSET 0
#define FLASH_ERROR_STATUS_MASK  0x0e
#define FLASH_ENABLE_OCTAL_CMD   0x02

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
status_t sbl_flash_erase(uint32_t address, size_t len);
status_t sbl_flash_write(uint32_t dstAddr, const void *src, size_t len);
status_t sbl_flash_read(uint32_t dstAddr, void *buf, size_t len);
status_t sbl_flash_init(void);
/*${prototype:end}*/

#endif /* _FLEXSPI_FLASH_CONFIG_H_ */
