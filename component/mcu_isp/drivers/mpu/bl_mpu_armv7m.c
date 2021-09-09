/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "bl_mpu.h"
#include "bootloader_common.h"
#include "crc/crc32.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define kMpuCrcUpdate_Key (0xc35accf0U)

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint32_t s_mpuCrcUpdateKey = kMpuCrcUpdate_Key;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MPU_GetAlignedRegion(uint32_t *start, uint32_t *length);
static status_t MPU_VerifyChecksum(void);
//!@brief Update specified MPU region attribute
status_t MPU_UpdateRegionAttribute(uint32_t index, const mpu_region_config_t *mpuRegion);

static status_t MPU_IntegrityVerify(void);
/*******************************************************************************
 * Codes
 ******************************************************************************/

static status_t MPU_VerifyChecksum(void)
{
    status_t status = kStatus_MPU_Fail;

    do
    {
        crc32_data_t crcConfig = { 0UL };
        uint32_t calcChecksum = 0UL;
        crc32_init(&crcConfig);
        crc32_update(&crcConfig, (const uint8_t *)&g_mpuContext,
                     sizeof(g_mpuContext) - sizeof(g_mpuContext.crcChecksum));
        crc32_finalize(&crcConfig, &calcChecksum);

        if (calcChecksum == g_mpuContext.crcChecksum)
        {
            status = kStatus_MPU_Success;
        }

    } while (0);

    return status;
}

static status_t MPU_IntegrityVerify(void)
{
    status_t status = kStatus_MPU_Fail;

    do
    {
#if BL_FEATURE_HAB_SUPPORT
        if (kHabStatus_Close == get_hab_status())
        {
            if ((g_mpuContext.mpuMemoryMap[MPU_ENTRY_ITCM_INDEX].attribute != kMPU_RAMRegionAtrributeNonCacheableXN) ||
                (g_mpuContext.mpuMemoryMap[MPU_ENTRY_DTCM_INDEX].attribute != kMPU_RAMRegionAtrributeNonCacheableXN) ||
                (g_mpuContext.mpuMemoryMap[MPU_ENTRY_OCRAM_INDEX].attribute != kMPU_RAMRegionAttributeWriteBackXN))
            {
                break;
            }
        }
#endif

        status = MPU_VerifyChecksum();

    } while (0);

    return status;
}

status_t MPU_UpdateChecksum(void)
{
    status_t status = kStatus_MPU_Fail;

    do
    {
        if (kMpuCrcUpdate_Key == s_mpuCrcUpdateKey)
        {
            s_mpuCrcUpdateKey = 0UL;

            crc32_data_t crcConfig = { 0UL };
            uint32_t calcChecksum = 0UL;
            crc32_init(&crcConfig);
            crc32_update(&crcConfig, (const uint8_t *)&g_mpuContext,
                         sizeof(g_mpuContext) - sizeof(g_mpuContext.crcChecksum));
            crc32_finalize(&crcConfig, &calcChecksum);

            g_mpuContext.crcChecksum = calcChecksum;

            debug_printf("MPU context checksum=%x\n", g_mpuContext.crcChecksum);

            status = kStatus_MPU_Success;
        }
    } while (0);

    return status;
}

void MPU_GetAlignedRegion(uint32_t *start, uint32_t *length)
{
    uint32_t alignedLength = 0UL;
    uint32_t alignedStart = 0UL;

    uint32_t tmpLength = *length;
    for (uint64_t i = 256UL; i < 0xffffffffUL; i <<= 1)
    {
        if (tmpLength < i)
        {
            alignedLength = i;
            break;
        }
    }

    alignedStart = (*start) & ~(alignedLength - 1);
    *start = alignedStart;
    *length = alignedLength;
}

status_t MPU_Init(void)
{
    status_t status = kStatus_MPU_Fail;
    do
    {
        if (MPU_IntegrityVerify() != kStatus_MPU_Success)
        {
            break;
        }
        uint32_t maxRegionNumber = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
        const mpu_region_config_t *config = &g_mpuContext.mpuMemoryMap[0];
        MPU->CTRL = 0UL;
        __DSB();
        __ISB();
        bool hasInvalidConfig = false;
        uint32_t regionMapIndex = 0UL;
        uint32_t mpuReginIndex = 0UL;
        while (regionMapIndex < g_mpuContext.validEntries)
        {
            // The sizeInPower must be in allowed region
            if ((config->baseAddress > 0UL) && (config->attribute > 0UL) &&
                ((config->sizeInPower < kMinRegionSizeInPower) || (config->sizeInPower >= kRegionSize4GBytes)))
            {
                hasInvalidConfig = true;
                break;
            }

            // The base address must be aligned to the size of the region
            if (config->baseAddress & ((1UL << config->sizeInPower) - 1UL))
            {
                hasInvalidConfig = true;
                break;
            }

            // Fill MPU entries into MPU controller if the entry is valid
            if ((config->baseAddress > 0UL) || (config->sizeInPower >= kMinRegionSizeInPower))
            {
                MPU->RNR = mpuReginIndex++;
                MPU->RBAR = config->baseAddress & MPU_RBAR_ADDR_Msk;
                MPU->RASR = (config->attribute & MPU_RASR_ATTRS_Msk) |
                            (((config->sizeInPower - 1) << MPU_RASR_SIZE_Pos) & MPU_RASR_SIZE_Msk) |
                            MPU_RASR_ENABLE_Msk;
            }
            ++regionMapIndex;
            ++config;

            if (regionMapIndex > maxRegionNumber)
            {
                hasInvalidConfig = true;
                break;
            }
        }

        if (hasInvalidConfig)
        {
            break;
        }

        /* Enable MPU */
        MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_HFNMIENA_Msk | MPU_CTRL_PRIVDEFENA_Msk;
        __DSB();
        __ISB();

        status = kStatus_MPU_Success;

    } while (0);

    return status;
}

status_t MPU_Deinit(void)
{
    status_t status = kStatus_MPU_Fail;

    do
    {
        if (MPU_IntegrityVerify() != kStatus_MPU_Success)
        {
            break;
        }

        /* Disable MPU */
        MPU->CTRL = 0u;
        __DSB();
        __ISB();
        uint32_t regionIndex = 0UL;
        uint32_t maxRegionNumber = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
        while (regionIndex < maxRegionNumber)
        {
            /* Clear all configurations*/
            MPU->RNR = regionIndex++;
            MPU->RBAR = 0UL;
            MPU->RASR = 0UL;
        }
        MPU->RNR = 0u;
        __DSB();
        __ISB();

        status = kStatus_MPU_Success;

    } while (0);

    return status;
}

status_t MPU_UpdateRegionAttribute(uint32_t index, const mpu_region_config_t *mpuRegion)
{
    status_t status = kStatus_MPU_Fail;

    do
    {
        if (MPU_IntegrityVerify() != kStatus_MPU_Success)
        {
            break;
        }

        uint32_t maxRegionNumber = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
        if (index >= maxRegionNumber)
        {
            break;
        }

        if ((mpuRegion == NULL) || (mpuRegion->sizeInPower < kMinRegionSizeInPower) ||
            (mpuRegion->sizeInPower > kRegionSize4GBytes))
        {
            status = kStatus_MPU_InvalidArgument;
            break;
        }

        MPU->RNR = index;
        MPU->RBAR = mpuRegion->baseAddress & MPU_RBAR_ADDR_Msk;
        MPU->RASR = (mpuRegion->attribute & MPU_RASR_ATTRS_Msk) |
                    (((mpuRegion->sizeInPower - 1) << MPU_RASR_SIZE_Pos) & MPU_RASR_SIZE_Msk) | MPU_RASR_ENABLE_Msk;

        __DSB();
        __ISB();

        status = kStatus_MPU_Success;

    } while (0);

    return status;
}

status_t MPU_UpdateEntryFromRegion(uint32_t start, uint32_t length)
{
    status_t mpuStatus = kStatus_MPU_Fail;

    do
    {
        if (MPU_IntegrityVerify() != kStatus_MPU_Success)
        {
            break;
        }

        uint32_t mpuAlignedStart = start;
        uint32_t mpuAlignedLength = length;
        mpu_region_config_t *config = NULL;
        MPU_GetAlignedRegion(&mpuAlignedStart, &mpuAlignedLength);

        config = &g_mpuContext.mpuMemoryMap[MPU_ENTRY_IMG_MEM_INDEX];
        config->baseAddress = mpuAlignedStart;
        uint32_t sizeInPower = 0;
        for (sizeInPower = kRegionSize256Bytes; sizeInPower < kRegionSize4GBytes; sizeInPower++)
        {
            if ((mpuAlignedLength - 1) == ((1ul << sizeInPower) - 1UL))
            {
                break;
            }
        }
        // Invalid size, report an error
        if (sizeInPower >= kRegionSize4GBytes)
        {
            break;
        }

        config->sizeInPower = sizeInPower;
        // Always use the same MPU attribute
        config->attribute = kMPU_RAMRegionAttributeWriteBackXN;

        // Only allow to update MPU checksum during call this function
        s_mpuCrcUpdateKey = kMpuCrcUpdate_Key;
        if (MPU_UpdateChecksum() != kStatus_MPU_Success)
        {
            s_mpuCrcUpdateKey = 0UL;
            break;
        }

        debug_printf("Updated MPU start=%x sizeInPower=%x attr=%x\n", config->baseAddress, config->sizeInPower,
                     config->attribute);
        mpuStatus = MPU_Init();

    } while (0);

    return mpuStatus;
}
