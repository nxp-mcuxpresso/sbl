/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 
#ifndef __SYSFLASH_H__
#define __SYSFLASH_H__

#include <string.h>
#include "flash_info.h"
#include "mcuboot_config.h"

#ifndef CONFIG_SINGLE_APPLICATION_SLOT

#if (MCUBOOT_IMAGE_NUMBER == 1)
/*
 * NOTE: the definition below returns the same values for true/false on
 * purpose, to avoid having to mark x as non-used by all callers when
 * running in single image mode.
 */
#define FLASH_AREA_IMAGE_PRIMARY(x)    (((x) == 0) ?                \
                                         FLASH_AREA_ID(image_0) : \
                                         FLASH_AREA_ID(image_0))
#define FLASH_AREA_IMAGE_SECONDARY(x)  (((x) == 0) ?                \
                                         FLASH_AREA_ID(image_1) : \
                                         FLASH_AREA_ID(image_1))
#elif (MCUBOOT_IMAGE_NUMBER == 2)
/* MCUBoot currently supports only up to 2 updateable firmware images.
 * If the number of the current image is greater than MCUBOOT_IMAGE_NUMBER - 1
 * then a dummy value will be assigned to the flash area macros.
 */
#define FLASH_AREA_IMAGE_PRIMARY(x)    (((x) == 0) ?                \
                                         FLASH_AREA_ID(image_0) : \
                                        ((x) == 1) ?                \
                                         FLASH_AREA_ID(image_2) : \
                                         255)
#define FLASH_AREA_IMAGE_SECONDARY(x)  (((x) == 0) ?                \
                                         FLASH_AREA_ID(image_1) : \
                                        ((x) == 1) ?                \
                                         FLASH_AREA_ID(image_3) : \
                                         255)
#else
#error "Image slot and flash area mapping is not defined"
#endif

#if !defined(CONFIG_BOOT_SWAP_USING_MOVE)
#define FLASH_AREA_IMAGE_SCRATCH    FLASH_AREA_ID(image_scratch)
#endif

#else /* CONFIG_SINGLE_APPLICATION_SLOT */

#define FLASH_AREA_IMAGE_PRIMARY(x)	FLASH_AREA_ID(image_0)
#define FLASH_AREA_IMAGE_SECONDARY(x)	FLASH_AREA_ID(image_0)
/* NOTE: Scratch parition is not used by single image DFU but some of
 * functions in common files reference it, so the definitions has been
 * provided to allow compilation of common units.
 */
#define FLASH_AREA_IMAGE_SCRATCH	0

#endif /* CONFIG_SINGLE_APPLICATION_SLOT */

#define IMAGE_SLOT_NUM				3

#define FLASH_DEVICE_ID				1

#define FLASH_AREA_INIT(i, j)						\
        {.fa_id = i,								\
		 .fa_device_id = j,							\
         .fa_off = FLASH_AREA_IMAGE_##i##_OFFSET,	\
         .fa_size = FLASH_AREA_IMAGE_##i##_SIZE,},

typedef status_t (*init_t)(void);
typedef status_t (*erase_t)(uint32_t, size_t);
typedef status_t (*read_t)(uint32_t, void*, size_t);
typedef status_t (*write_t)(uint32_t, const void*, size_t);

typedef struct {
	init_t flash_init;
	erase_t flash_erase;
	read_t flash_read;
	write_t flash_write;
	uint8_t align_val;
	uint8_t erased_val;
} flash_ops_s;

int flash_area_id_from_multi_image_slot(int image_index, int slot);

#endif /* __SYSFLASH_H__ */
