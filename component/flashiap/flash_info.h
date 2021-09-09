// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 */
#ifndef __FLASH_INFO_H__
#define __FLASH_INFO_H__

#include "sbl.h"

#ifdef SOC_IMXRTXXX_SERIES
#include "flexspi_flash.h"
#include "flexspi_flash_config.h"
#endif

#ifdef SOC_IMXRTYYYY_SERIES
#include "flexspi_flash.h"
#include "flexspi_flash_config.h"
#endif

#ifdef SOC_LPC_SERIES
#include "iap_flash_ops.h"
#endif

#define FLASH_AREA_ID(x) FLASH_AREA_ID_##x

#define FLASH_AREA_ID_image_0 1
#define FLASH_AREA_ID_image_1 2
#define FLASH_AREA_ID_image_scratch 3
#define FLASH_AREA_ID_image_2 4
#define FLASH_AREA_ID_image_3 5

#ifdef SOC_LPC_SERIES
#define FLASH_AREA_IMAGE_SECTOR_SIZE	        (SECTOR_SIZE)
#else
#define FLASH_AREA_IMAGE_SECTOR_SIZE	        (FLASH_CONFIG_SECTORSIZE)
#endif

#define FLASH_DEVICE_BASE_ADDR			BOOT_FLASH_BASE	//0x60000000
/* BOOT_MAX_IMG_SECTORS must be bigger than the image slot sector number */
#define FLASH_AREA_IMAGE_1_OFFSET		(BOOT_FLASH_ACT_APP - BOOT_FLASH_BASE)	//MCUboot occupies 512KB
#define FLASH_AREA_IMAGE_1_SIZE			(BOOT_FLASH_CAND_APP - BOOT_FLASH_ACT_APP)//image1 slot occupies 1.5MB
#define FLASH_AREA_IMAGE_2_OFFSET		(FLASH_AREA_IMAGE_1_OFFSET + FLASH_AREA_IMAGE_1_SIZE)
#define FLASH_AREA_IMAGE_2_SIZE			FLASH_AREA_IMAGE_1_SIZE	//image2 slot occupies 1.5MB
#define FLASH_AREA_IMAGE_3_OFFSET		(FLASH_AREA_IMAGE_2_OFFSET + FLASH_AREA_IMAGE_2_SIZE)
#define FLASH_AREA_IMAGE_3_SIZE			0x80000		//scratch slot occupies 512KB

#endif
