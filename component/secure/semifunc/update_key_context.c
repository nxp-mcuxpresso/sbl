/*
 * Copyright 2021 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "flexspi_flash.h"
#include "flexspi_flash_config.h"
#include "update_key_context.h"
#ifdef COMPONENT_MCU_ISP
#include "bl_context.h"
#endif

#if defined(CONFIG_BOOT_ENCRYPTED_XIP)

/* Assume that user encrypt SBL, copy new key inormation to second context,
 * or user should change the offset of key context
 */
#if defined(SOC_IMXRT1170_SERIES) || defined(SOC_IMXRT1010_SERIES) || \
    defined(SOC_IMXRTXXX_SERIES)
#define KEY_CONTEXT2_OFFSET_IN_SBL      (0x40u)
#define KEY_CONTEXT_SIZE                (0x40u)
#elif defined(SOC_IMXRT1060_SERIES) || defined(SOC_IMXRT1064_SERIES) || \
      defined(SOC_IMXRT1050_SERIES) || defined(SOC_IMXRT1020_SERIES)
#define KEY_CONTEXT2_OFFSET_IN_SBL      (0x800u)
#define KEY_CONTEXT_SIZE                (0x180u)
#endif

status_t update_key_context(uint32_t key_info_address)
{
    status_t status = kStatus_Fail;
    uint32_t key_info[KEY_CONTEXT_SIZE/4];
    uint8_t *tmpbuf;

    tmpbuf = malloc(FLASH_CONFIG_SECTORSIZE);
    if (!tmpbuf) {
        return kStatus_Fail;
    }

    do {
        status = sbl_flash_read(0, tmpbuf, FLASH_CONFIG_SECTORSIZE);
        if (status != kStatus_Success) {
            break;
        }

        status = sbl_flash_read(key_info_address, key_info, KEY_CONTEXT_SIZE);
        if (status != kStatus_Success) {
            break;
        }

        /* Assume that user encrypt SBL, copy new key inormation to second context,
         * or user should use the real destination address
         */
        memcpy(tmpbuf + KEY_CONTEXT2_OFFSET_IN_SBL, key_info, KEY_CONTEXT_SIZE);

        status = sbl_flash_erase(0, FLASH_CONFIG_SECTORSIZE);
        if (status != kStatus_Success) {
            break;
        }

        status = sbl_flash_write(0, tmpbuf, FLASH_CONFIG_SECTORSIZE);
        if (status != kStatus_Success) {
            break;
        }

    } while (0);
    
    return status;
}

#endif //defined(CONFIG_BOOT_ENCRYPTED_XIP)
