/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _FLEXSPI_FLASH_H_
#define _FLEXSPI_FLASH_H_

#include "fsl_iap.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define FLASH_SIZE_KB                   (512)
#define EXAMPLE_FLEXSPI_AMBA_BASE       (0x00000000)
#define FLASH_PAGE_SIZE                 (512)
#define SECTOR_SIZE                     (32*1024) /* 32K */




/*${macro:end}*/


status_t sbl_flash_erase(uint32_t address, size_t len);
status_t sbl_flash_write(uint32_t dstAddr, const void *src, size_t len);
status_t sbl_flash_read(uint32_t dstAddr, void *buf, size_t len);
status_t sbl_flash_init(void);

/*${prototype:end}*/

#endif /* _FLEXSPI_FLASH_H_ */
