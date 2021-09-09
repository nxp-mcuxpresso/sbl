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

#if defined(SOC_IMXRTYYYY_SERIES) && defined(CONFIG_BOOT_ENCRYPTED_XIP)

/* Assume that user encrypt SBL, copy new key inormation to second context,
 * or user should change the offset of key context
 */
#if defined(SOC_IMXRT1170_SERIES) || defined(SOC_IMXRT1010_SERIES)
#define KEY_CONTEXT2_OFFSET_IN_SBL      (0x40u)
#define KEY_CONTEXT_SIZE                (0x40u)
#elif defined(SOC_IMXRT1060_SERIES) || defined(SOC_IMXRT1064_SERIES) || \
      defined(SOC_IMXRT1050_SERIES) || defined(SOC_IMXRT1020_SERIES)
#define KEY_CONTEXT2_OFFSET_IN_SBL      (0x800u)
#define KEY_CONTEXT_SIZE                (0x180u)
#endif

#if defined(COMPONENT_MCU_ISP)
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
        status = g_bootloaderContext.memoryInterface->read(BOOT_FLASH_BASE, FLASH_CONFIG_SECTORSIZE, tmpbuf, kGroup_External);
        if (status != kStatus_Success) {
            break;
        }

        /* Assume that user encrypt SBL, copy new key inormation to second context,
         * or user should use the real destination address
         */
        status = g_bootloaderContext.memoryInterface->read(key_info_address, KEY_CONTEXT_SIZE, (uint8_t *)key_info, kGroup_External);
        if (status != kStatus_Success) {
            break;
        }
        memcpy(tmpbuf + KEY_CONTEXT2_OFFSET_IN_SBL, key_info, KEY_CONTEXT_SIZE);

        status = g_bootloaderContext.memoryInterface->erase(BOOT_FLASH_BASE, FLASH_CONFIG_SECTORSIZE, kGroup_External);
        if (status != kStatus_Success) {
            break;
        }

        status = g_bootloaderContext.memoryInterface->write(BOOT_FLASH_BASE, FLASH_CONFIG_SECTORSIZE, tmpbuf, kGroup_External);
        if (status != kStatus_Success) {
            break;
        }
        
        status = g_bootloaderContext.memoryInterface->flush();
        if (status != kStatus_Success) {
            break;
        }

    } while (0);

    free(tmpbuf);
    
    return status;
}
#else
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
#endif

#endif //defined(SOC_IMXRTYYYY_SERIES) && defined(CONFIG_BOOT_ENCRYPTED_XIP)
