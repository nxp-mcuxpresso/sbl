/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_flashiap_wrapper.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*! @brief Perform the cache clear to the flash*/
void flashiap_cache_clear(flashiap_config_t *config);

/*! @brief Validates the range and alignment of the given address range.*/
static status_t flashiap_check_range(flashiap_config_t *config,
                                     uint32_t startAddress,
                                     uint32_t lengthInBytes,
                                     uint32_t alignmentBaseline);

/*! @brief Validates the given user key for flash erase APIs.*/
static status_t flashiap_check_user_key(uint32_t key);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t FLASHIAP_Init(flashiap_config_t *config)
{
    if (config == NULL)
    {
        return kStatus_FLASHIAP_InvalidArgument;
    }

    /* fill out a few of the structure members */
    config->PFlashBlockBase = 0;
    config->PFlashTotalSize = FSL_FEATURE_SYSCON_FLASH_SIZE_BYTES;
    config->PFlashPageSize = FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES;
    config->PFlashSectorSize = FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES;

    config->PFlashCallback = NULL;

    return kStatus_FLASHIAP_Success;
}

status_t FLASHIAP_SetCallback(flashiap_config_t *config, flash_callback_t callback)
{
    if (config == NULL)
    {
        return kStatus_FLASHIAP_InvalidArgument;
    }

    config->PFlashCallback = callback;

    return kStatus_FLASHIAP_Success;
}

status_t FLASHIAP_Erase(flashiap_config_t *config, uint32_t start, uint32_t lengthInBytes, uint32_t key)
{
    uint32_t eraseStepSize;
    uint32_t endAddress;    /* storing end address */
    uint32_t numberOfPages; /* number of Pages calculated by endAddress */
    status_t returnCode;

    /* Check the supplied address range. */
    /* Note: From LPC RM, LPC device supports sector and page erase.*/
    returnCode = flashiap_check_range(config, start, lengthInBytes, config->PFlashPageSize);
    if (returnCode)
    {
        return returnCode;
    }

    /* calculating Flash end address */
    endAddress = start + lengthInBytes - 1;

    /* re-calculate the endAddress and align it to the start of the next page
     * which will be used in the comparison below */
    if (endAddress % config->PFlashPageSize)
    {
        numberOfPages = endAddress / config->PFlashPageSize + 1;
        endAddress = numberOfPages * config->PFlashPageSize - 1;
    }

    /* the start address will increment to the next page/sector address
     * until it reaches the endAdddress */
    while (start <= endAddress)
    {
        /* Validate the user key */
        returnCode = flashiap_check_user_key(key);
        if (returnCode)
        {
            return returnCode;
        }

        /* Note: From LPC RM, The "Prepare Sector(s) for Write Operation" command
         * should precede this command.*/
        uint32_t sectorOrderNumber = (start - config->PFlashBlockBase) / config->PFlashSectorSize;
        returnCode = FLASHIAP_PrepareSectorForWrite(sectorOrderNumber, sectorOrderNumber);
        if (kStatus_FLASHIAP_Success != returnCode)
        {
            break;
        }
        else
        {
            /* calling flashiap command sequence function to execute the command */
            if ((!(start % config->PFlashSectorSize)) && ((endAddress - start + 1) >= config->PFlashSectorSize))
            {
                eraseStepSize = config->PFlashSectorSize;
                returnCode = FLASHIAP_EraseSector(sectorOrderNumber, sectorOrderNumber, SystemCoreClock);
            }
            else
            {
                eraseStepSize = config->PFlashPageSize - (start % config->PFlashPageSize);
                uint32_t pageOrderNumber = (start - config->PFlashBlockBase) / config->PFlashPageSize;
                returnCode = FLASHIAP_ErasePage(pageOrderNumber, pageOrderNumber, SystemCoreClock);
            }
        }

        /* calling flash callback function if it is available */
        if (config->PFlashCallback)
        {
            config->PFlashCallback();
        }

        /* checking the success of command execution */
        if (kStatus_FLASHIAP_Success != returnCode)
        {
            break;
        }
        else
        {
            /* Increment to the next page/sector */
            start += eraseStepSize;
        }
    }

    flashiap_cache_clear(config);

    return (returnCode);
}

status_t FLASHIAP_Program(flashiap_config_t *config, uint32_t start, uint32_t *src, uint32_t lengthInBytes)
{
    status_t returnCode;

    if (src == NULL)
    {
        return kStatus_FLASHIAP_InvalidArgument;
    }

    /* Check the supplied address range. */
    /* Note: From LPC RM, The smallest amount of data that can be written to flash by the
     *  copy RAM to flash command is equal to the size of one page.*/
    returnCode = flashiap_check_range(config, start, lengthInBytes, config->PFlashPageSize);
    if (returnCode)
    {
        return returnCode;
    }

    while (lengthInBytes > 0)
    {
        /* Note: From LPC RM, The "Prepare Sector(s) for Write Operation" command
         * should precede this command.*/
        uint32_t sectorOrderNumber = (start - config->PFlashBlockBase) / config->PFlashSectorSize;
        returnCode = FLASHIAP_PrepareSectorForWrite(sectorOrderNumber, sectorOrderNumber);
        if (kStatus_FLASHIAP_Success != returnCode)
        {
            break;
        }
        else
        {
            /* calling flashiap command sequence function to execute the command */
            returnCode = FLASHIAP_CopyRamToFlash(start, src, config->PFlashPageSize, SystemCoreClock);
            src += config->PFlashPageSize;
        }

        /* calling flash callback function if it is available */
        if (config->PFlashCallback)
        {
            config->PFlashCallback();
        }

        /* checking for the success of command execution */
        if (kStatus_FLASHIAP_Success != returnCode)
        {
            break;
        }
        else
        {
            /* update start address for next iteration */
            start += config->PFlashPageSize;

            /* update lengthInBytes for next iteration */
            lengthInBytes -= config->PFlashPageSize;
        }
    }

    flashiap_cache_clear(config);

    return (returnCode);
}

status_t FLASHIAP_VerifyErase(flashiap_config_t *config, uint32_t start, uint32_t lengthInBytes)
{
    /* Check arguments. */
    status_t returnCode;

    returnCode = flashiap_check_range(config, start, lengthInBytes, kFLASHIAP_AlignementUnitVerifyErase);
    if (returnCode)
    {
        return returnCode;
    }

    /* calculating Flash end address */
    uint32_t endAddress = start + lengthInBytes - 1;

    bool isSectorStepSize = true;
    while (start <= endAddress)
    {
        if ((!(start % config->PFlashSectorSize)) && ((endAddress - start + 1) >= config->PFlashSectorSize))
        {
            uint32_t startSectorOrderNumber = (start - config->PFlashBlockBase) / config->PFlashSectorSize;

            /* calling flashiap command sequence function to execute the command */
            returnCode = FLASHIAP_BlankCheckSector(startSectorOrderNumber, startSectorOrderNumber);
            if (kStatus_FLASHIAP_Success != returnCode)
            {
                if (returnCode == kStatus_FLASHIAP_SectorNotblank)
                {
                    returnCode = kStatus_FLASHIAP_MemoryNotblank;
                }
                break;
            }
            else
            {
                /* Increment to the next sector */
                start += config->PFlashSectorSize;
            }
        }
        else
        {
            uint8_t blankValue[8];
            memset(blankValue, (uint8_t)kFLASHIAP_ErasedValue, sizeof(blankValue));
            assert(kFLASHIAP_AlignementUnitVerifyErase <= sizeof(blankValue));

            if (memcmp((uint8_t *)start, (uint8_t *)blankValue, kFLASHIAP_AlignementUnitVerifyErase) != 0)
            {
                returnCode = kStatus_FLASHIAP_MemoryNotblank;
                break;
            }
            else
            {
                start += kFLASHIAP_AlignementUnitVerifyErase;
            }
        }
    }

    return (returnCode);
}

status_t FLASHIAP_VerifyProgram(flashiap_config_t *config,
                                uint32_t start,
                                uint32_t lengthInBytes,
                                const uint32_t *expectedData,
                                uint32_t *failedAddress,
                                uint32_t *failedData)
{
    status_t returnCode;

    if (expectedData == NULL)
    {
        return kStatus_FLASHIAP_InvalidArgument;
    }

    returnCode = flashiap_check_range(config, start, lengthInBytes, kFLASHIAP_AlignementUnitVerifyProgram);
    if (returnCode)
    {
        return returnCode;
    }

    bool isPageStepSize = true;
    while (lengthInBytes)
    {
        uint32_t stepSizeInBytes;
        /* Note: From LPC RM, Compare result may not be correct when source or destination address contains any of the
         * first 512
         * bytes starting from address zero. First 512 bytes are re-mapped to boot ROM */
        if ((start >= kFLASHIAP_RemappedMemoryBase + kFLASHIAP_RemappedMemorySize) &&
            (lengthInBytes >= config->PFlashPageSize) && isPageStepSize)
        {
            stepSizeInBytes = config->PFlashPageSize;
            /* calling flashiap command sequence function to execute the command */
            returnCode = FLASHIAP_Compare(start, (uint32_t *)expectedData, stepSizeInBytes);
            /* checking for the success of command execution */
            if (kStatus_FLASHIAP_Success != returnCode)
            {
                isPageStepSize = false;
                continue;
            }
        }
        else
        {
            stepSizeInBytes = kFLASHIAP_AlignementUnitVerifyProgram;
            if (memcmp((uint8_t *)start, (uint8_t *)expectedData, stepSizeInBytes) != 0)
            {
                if (failedAddress)
                {
                    *failedAddress = start;
                }
                if (failedData)
                {
                    *failedData = 0;
                }
                break;
            }
        }

        lengthInBytes -= stepSizeInBytes;
        expectedData += stepSizeInBytes;
        start += stepSizeInBytes;
    }

    return (returnCode);
}

status_t FLASHIAP_GetProperty(flashiap_config_t *config, flash_property_tag_t whichProperty, uint32_t *value)
{
    if ((config == NULL) || (value == NULL))
    {
        return kStatus_FLASHIAP_InvalidArgument;
    }

    switch (whichProperty)
    {
        case kFLASHIAP_PropertyPflashSectorSize:
            *value = config->PFlashSectorSize;
            break;

        case kFLASHIAP_PropertyPflashTotalSize:
            *value = config->PFlashTotalSize;
            break;

        case kFLASHIAP_PropertyPflashBlockBaseAddr:
            *value = config->PFlashBlockBase;
            break;

        case kFLASHIAP_PropertyPflashPageSize:
            *value = config->PFlashPageSize;
            break;

        default: /* catch inputs that are not recognized */
            return kStatus_FLASHIAP_UnknownProperty;
    }

    return kStatus_FLASHIAP_Success;
}

/*! @brief Perform the cache clear to the flash*/
void flashiap_cache_clear(flashiap_config_t *config)
{
}

/*! @brief Validates the range and alignment of the given address range.*/
static status_t flashiap_check_range(flashiap_config_t *config,
                                     uint32_t startAddress,
                                     uint32_t lengthInBytes,
                                     uint32_t alignmentBaseline)
{
    if (config == NULL)
    {
        return kStatus_FLASHIAP_InvalidArgument;
    }

    /* Verify the start and length are alignmentBaseline aligned. */
    if ((startAddress & (alignmentBaseline - 1)) || (lengthInBytes & (alignmentBaseline - 1)))
    {
        return kStatus_FLASHIAP_AlignmentError;
    }

    /* check for valid range of the target addresses */
    if (((startAddress >= config->PFlashBlockBase) &&
         ((startAddress + lengthInBytes) <= (config->PFlashBlockBase + config->PFlashTotalSize))))
    {
        return kStatus_FLASHIAP_Success;
    }

    return kStatus_FLASHIAP_AddressError;
}

/*! @brief Validates the given user key for flash erase APIs.*/
static status_t flashiap_check_user_key(uint32_t key)
{
    /* Validate the user key */
    if (key != kFLASHIAP_ApiEraseKey)
    {
        return kStatus_FLASHIAP_EraseKeyError;
    }

    return kStatus_FLASHIAP_Success;
}
