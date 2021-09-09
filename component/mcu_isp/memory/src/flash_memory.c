/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "memory/memory.h"
#include "flash_memory.h"
#include "normal_memory.h"
#include "flash/fsl_flash.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/fsl_assert.h"
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// If the bootloader is running from flash, then we need to make sure that all
// interrupts are disabled during the execution of a flash operation, so that
// no code is unexpectedly run from flash (which would cause a hard fault).
//
// If we're running from ROM or RAM, then we neither need to nor want to disable
// interrupts during flash operations.
#if !BL_TARGET_FLASH
#define flash_lock_release() (void)sizeof(0)
#define flash_lock_acquire() (void)sizeof(0)
#endif // BL_TARGET_FLASH

//! @brief Flash Memory constants.
enum _flash_memory_constants
{
    kFlashMemory_ErasedValue = ~0
};

#if BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED
//! @brief FLASH section program memory context
//!
//! An instance is maintained in this file, will is used to keep key information for write and flush
//! operatations.
typedef struct _flash_section_program_info
{
    uint32_t startAddress;                      //!< This address is used to record the address which is used
                                                //!< to write the whole section into flash memory
    uint32_t storedBytes;                       //!< A variable which is used to indicate if the buffer is full.
    uint8_t buffer[kFLASH_AccelerationRamSize]; //!< A buffer which is used to buffer a full section of data
} flash_section_program_info_t;

flash_section_program_info_t s_flash_section_program_info[kFLASHCount] = { 0 };
#endif

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief flash memory array.
flash_config_t g_flashState[] = {
    // Main Flash
    { 0 },
#if BL_HAS_SECONDARY_INTERNAL_FLASH
    // Secondary Flash
    { 0 },
#endif
    { 0 }
};

//! @brief Current flash memory index.
static volatile uint8_t s_flashMemoryIndex = kFlashIndex_Main;

//! @brief Interface to main flash memory operations.
const memory_region_interface_t g_flashMemoryInterface = {
    .init = &main_flash_mem_init,
    .read = &main_flash_mem_read,
    .write = &main_flash_mem_write,
#if !BL_FEATURE_MIN_PROFILE || BL_FEATURE_FILL_MEMORY
    .fill = &main_flash_mem_fill,
#endif // !BL_FEATURE_MIN_PROFILE
#if BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED
    .flush = main_flash_mem_flush,
#else
    .flush = NULL,
#endif
    .erase = main_flash_mem_erase,
    .erase_all = flash_mem_erase_all,
};

//! @brief Interface to secondary flash memory operations.
#if BL_HAS_SECONDARY_INTERNAL_FLASH
const memory_region_interface_t g_secondaryFlashMemoryInterface = {
    .init = &secondary_flash_mem_init,
    .read = &secondary_flash_mem_read,
    .write = &secondary_flash_mem_write,
#if !BL_FEATURE_MIN_PROFILE || BL_FEATURE_FILL_MEMORY
    .fill = &secondary_flash_mem_fill,
#endif // !BL_FEATURE_MIN_PROFILE
#if BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED
    .flush = secondary_flash_mem_flush,
#else
    .flush = NULL,
#endif
    .erase = secondary_flash_mem_erase,
    .erase_all = flash_mem_erase_all,
};
#endif // BL_HAS_SECONDARY_INTERNAL_FLASH

#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
//! @brief It is used for indicating if an XA controlled region is unlocked to program state
bool isFlashRegionUnlocked = false;
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL

#if BL_TARGET_FLASH
static uint32_t s_regPrimask = 0U;
#endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
//! @brief check if a flash region is in an XA controlled region or contains an XA controlled region.
//         and try to open flash program state by calling verify_erase_all command if needed.
status_t flash_preprocess_execute_only_region(uint32_t address,
                                              uint32_t length,
                                              flash_execute_only_access_state_t *state);

status_t flash_check_access_before_programming(uint32_t address, uint32_t length, bool *verifyWrites);

static inline bool flash_check_is_erased(uint32_t address, uint32_t length);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if BL_TARGET_FLASH
static void flash_lock_acquire(void)
{
    // Ensure that the program operation cannots be interrupted.
    s_regPrimask = __get_PRIMASK();
    __disable_irq();
}

static void flash_lock_release(void)
{
    // Release lock after the write operation completes.
    __set_PRIMASK(s_regPrimask);
}
#endif

// See flash_memory.h for documentation on this function.
status_t main_flash_mem_init(void)
{
    s_flashMemoryIndex = kFlashIndex_Main;
    return flash_mem_init();
}

// See flash_memory.h for documentation on this function.
status_t main_flash_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
    s_flashMemoryIndex = kFlashIndex_Main;
    return flash_mem_read(address, length, buffer);
}

// See flash_memory.h for documentation on this function.
status_t main_flash_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    s_flashMemoryIndex = kFlashIndex_Main;
    return flash_mem_write(address, length, buffer);
}

// See flash_memory.h for documentation on this function.
status_t main_flash_mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    s_flashMemoryIndex = kFlashIndex_Main;
    return flash_mem_fill(address, length, pattern);
}

// See flash_memory.h for documentation on this function.
status_t main_flash_mem_flush(void)
{
    s_flashMemoryIndex = kFlashIndex_Main;
    return flash_mem_flush();
}

// See flash_memory.h for documentation on this function.
status_t main_flash_mem_erase(uint32_t address, uint32_t length)
{
    s_flashMemoryIndex = kFlashIndex_Main;
    return flash_mem_erase(address, length);
}

// See flash_memory.h for documentation on this function.
status_t secondary_flash_mem_init(void)
{
    s_flashMemoryIndex = kFlashIndex_Secondary;
    return flash_mem_init();
}

// See flash_memory.h for documentation on this function.
status_t secondary_flash_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
    s_flashMemoryIndex = kFlashIndex_Secondary;
    return flash_mem_read(address, length, buffer);
}

// See flash_memory.h for documentation on this function.
status_t secondary_flash_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    s_flashMemoryIndex = kFlashIndex_Secondary;
    return flash_mem_write(address, length, buffer);
}

// See flash_memory.h for documentation on this function.
status_t secondary_flash_mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    s_flashMemoryIndex = kFlashIndex_Secondary;
    return flash_mem_fill(address, length, pattern);
}

// See flash_memory.h for documentation on this function.
status_t secondary_flash_mem_flush(void)
{
    s_flashMemoryIndex = kFlashIndex_Secondary;
    return flash_mem_flush();
}

// See flash_memory.h for documentation on this function.
status_t secondary_flash_mem_erase(uint32_t address, uint32_t length)
{
    s_flashMemoryIndex = kFlashIndex_Secondary;
    return flash_mem_erase(address, length);
}

// See flash_memory.h for documentation on this function.
status_t flash_mem_init(void)
{
    // Update address range of flash
    memory_map_entry_t *map;
#if BL_HAS_SECONDARY_INTERNAL_FLASH
    if (s_flashMemoryIndex == kFlashIndex_Secondary)
    {
        map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexSenondaryFlashArray];
    }
    else
#endif
    {
        map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexFlashArray];
    }

    g_bootloaderContext.flashDriverInterface->flash_get_property(
        &g_bootloaderContext.allFlashState[s_flashMemoryIndex], kFLASH_PropertyPflashBlockBaseAddr, &map->startAddress);
    uint32_t tmp;
    g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[s_flashMemoryIndex],
                                                                 kFLASH_PropertyPflashTotalSize, &tmp);
    map->endAddress = map->startAddress + tmp - 1;

#if BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED
    s_flash_section_program_info[s_flashMemoryIndex].storedBytes = 0;
#endif

    return kStatus_Success;
}

// See flash_memory.h for documentation on this function.
status_t flash_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    flash_execute_only_access_state_t access_state;
    uint32_t alignedAddress =
        ALIGN_DOWN(address, g_bootloaderContext.allFlashState[s_flashMemoryIndex].PFlashAccessSegmentSize);
    uint32_t updatedLength = address - alignedAddress + length;
    uint32_t alignedLength =
        ALIGN_UP(updatedLength, g_bootloaderContext.allFlashState[s_flashMemoryIndex].PFlashAccessSegmentSize);
    status_t status = g_bootloaderContext.flashDriverInterface->flash_is_execute_only(
        &g_bootloaderContext.allFlashState[s_flashMemoryIndex], alignedAddress, alignedLength, &access_state);
    if (status != kStatus_Success)
    {
        return status;
    }

    if (access_state != kFLASH_AccessStateUnLimited)
    {
        return kStatus_FLASH_RegionExecuteOnly;
    }
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL

    return normal_mem_read(address, length, buffer);
}

#if !BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED
// See flash_memory.h for documentation on this function.
status_t flash_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_write().
    assert(length);
    assert(buffer);

    status_t status = kStatus_Success;
    uint32_t alignedLength;
    uint32_t extraBytes;
    uint32_t extraData[2];

    assert(sizeof(extraData) >= sizeof(uint8_t) * FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE);

    bool verifyWrites;
    status = flash_check_access_before_programming(address, length, &verifyWrites);
    if (status != kStatus_Success)
    {
        return status;
    }

    // Align length to whole words.
    alignedLength = ALIGN_DOWN(length, sizeof(uint8_t) * FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE);
    extraBytes = length - alignedLength;
    assert(extraBytes < sizeof(uint8_t) * FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE);

    // Pre-fill word buffer with flash erased value.
    extraData[0] = (uint32_t)kFlashMemory_ErasedValue;
    extraData[1] = (uint32_t)kFlashMemory_ErasedValue;
    if (extraBytes)
    {
        // Copy extra bytes to word buffer.
        memcpy((uint8_t *)extraData, &buffer[alignedLength], extraBytes);
    }

    flash_lock_acquire();
    // Program whole words from the user's buffer.
    if (alignedLength)
    {
        status = g_bootloaderContext.flashDriverInterface->flash_program(
            &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address, (uint32_t *)buffer, alignedLength);
    }
    if ((status == kStatus_Success) && extraBytes)
    {
        // Program trailing word.
        status = g_bootloaderContext.flashDriverInterface->flash_program(
            &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address + alignedLength, extraData,
            (uint32_t)FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE);
    }
    flash_lock_release();
    if (status != kStatus_Success)
    {
        return status;
    }

#if !BL_FEATURE_FLASH_VERIFY_DISABLE
    if (verifyWrites)
    {
        uint32_t failedAddress;
        uint32_t failedData;

        flash_lock_acquire();
        if (alignedLength)
        {
            status = g_bootloaderContext.flashDriverInterface->flash_verify_program(
                &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address, alignedLength, (uint32_t *)buffer,
                (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin, &failedAddress,
                &failedData);
        }
        if ((status == kStatus_Success) && extraBytes)
        {
            status = g_bootloaderContext.flashDriverInterface->flash_verify_program(
                &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address + alignedLength, sizeof(extraData),
                extraData, (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin,
                &failedAddress, &failedData);
        }
        flash_lock_release();
        if (status != kStatus_Success)
        {
            debug_printf("Error: flash verify failed at address: 0x%x\r\n", failedAddress);
            return status;
        }
    }
#endif // !BL_FEATURE_FLASH_VERIFY_DISABLE

    return kStatus_Success;
}

// See flash_memory.h for documentation on this function.
status_t flash_mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    // Note: the check for "length != 0"
    assert(length);

    status_t status = kStatus_Success;
    uint32_t patternBuffer[8];

    // Pre-fill pattern buffer with pattern.
    for (uint32_t i = 0; i < 8; i++)
    {
        patternBuffer[i] = pattern;
    }

    // Program patterns from the pattern buffer.
    while (length)
    {
        uint32_t bytes;
        if (length < sizeof(patternBuffer))
        {
            bytes = length;
        }
        else
        {
            bytes = sizeof(patternBuffer);
        }

        status = flash_mem_write(address, bytes, (uint8_t *)patternBuffer);
        if (status != kStatus_Success)
        {
            return status;
        }

        address += bytes;
        length -= bytes;
    }

    return kStatus_Success;
}

#else // BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED

// See flash_memory.h for documentation on this function.
status_t flash_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_write().
    assert(length);
    assert(buffer);

    status_t status = kStatus_Success;

    while (length)
    {
        // Set start address when storing first byte into section program buffer
        if ((!s_flash_section_program_info[s_flashMemoryIndex].storedBytes) &&
            (!s_flash_section_program_info[s_flashMemoryIndex].startAddress))
        {
            // Check address alignment
            if (address & (FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT - 1))
            {
                return kStatus_FLASH_AlignmentError;
            }
            s_flash_section_program_info[s_flashMemoryIndex].startAddress = address;
        }
        else
        {
            // Start section programming operation when meet discontinuous address
            if ((s_flash_section_program_info[s_flashMemoryIndex].startAddress +
                 s_flash_section_program_info[s_flashMemoryIndex].storedBytes) != address)
            {
                // flush cached data into target memory,
                status = flash_mem_flush();
                if (status != kStatus_Success)
                {
                    return status;
                }
                continue;
            }
        }

        uint32_t storeBytes;
        // Check to see if section program buffer will be filled with current data packet
        if ((s_flash_section_program_info[s_flashMemoryIndex].storedBytes + length) <=
            sizeof(s_flash_section_program_info[s_flashMemoryIndex].buffer))
        {
            storeBytes = length;
        }
        else
        {
            storeBytes = sizeof(s_flash_section_program_info[s_flashMemoryIndex].buffer) -
                         s_flash_section_program_info[s_flashMemoryIndex].storedBytes;
        }

        // Copy data to section program buffer
        if (buffer !=
            &s_flash_section_program_info[s_flashMemoryIndex]
                 .buffer[s_flash_section_program_info[s_flashMemoryIndex].storedBytes])
        {
            memcpy(&s_flash_section_program_info[s_flashMemoryIndex]
                        .buffer[s_flash_section_program_info[s_flashMemoryIndex].storedBytes],
                   buffer, storeBytes);
        }

        s_flash_section_program_info[s_flashMemoryIndex].storedBytes += storeBytes;
        buffer += storeBytes;
        address += storeBytes;
        length -= storeBytes;

        // Start section programming operation when section program buffer is full
        if (s_flash_section_program_info[s_flashMemoryIndex].storedBytes ==
            sizeof(s_flash_section_program_info[s_flashMemoryIndex].buffer))
        {
            // flush cached data into target memory,
            status = flash_mem_flush();
            if (status != kStatus_Success)
            {
                return status;
            }
        }
    }

    return kStatus_Success;
}

// See flash_memory.h for documentation on this function.
status_t flash_mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_fill().
    assert(length);

    status_t status;

    // Pre-fill section program buffer with pattern
    uint32_t *buffer = (uint32_t *)s_flash_section_program_info[s_flashMemoryIndex].buffer;
    uint32_t maxPatterns = sizeof(s_flash_section_program_info[s_flashMemoryIndex].buffer) >> 2;
    for (uint32_t i = 0; i < maxPatterns; i++)
    {
        *buffer++ = pattern;
    }

    while (length)
    {
        uint32_t bytes;

        s_flash_section_program_info[s_flashMemoryIndex].storedBytes = 0;

        // Check to see if remaining address range can hold whole section program buffer
        if (length < sizeof(s_flash_section_program_info[s_flashMemoryIndex].buffer))
        {
            bytes = length;
        }
        else
        {
            bytes = sizeof(s_flash_section_program_info[s_flashMemoryIndex].buffer);
        }

        // flush cached data into target memory,
        status = flash_mem_write(address, bytes, s_flash_section_program_info[s_flashMemoryIndex].buffer);
        if (status != kStatus_Success)
        {
            return status;
        }

        address += bytes;
        length -= bytes;
    }

    // flush cached data into target memory,
    status = flash_mem_flush();
    if (status != kStatus_Success)
    {
        return status;
    }

    return kStatus_Success;
}

// See memory.h for documentation on this function.
status_t flash_mem_flush(void)
{
    status_t status = kStatus_Success;

    if (s_flash_section_program_info[s_flashMemoryIndex].storedBytes)
    {
        uint32_t address = s_flash_section_program_info[s_flashMemoryIndex].startAddress;
        uint32_t length = s_flash_section_program_info[s_flashMemoryIndex].storedBytes;

        // Clear related states no matter following operations are executed successfully or not.
        s_flash_section_program_info[s_flashMemoryIndex].startAddress = 0;
        s_flash_section_program_info[s_flashMemoryIndex].storedBytes = 0;

        // Align length to section program unit
        uint32_t alignedLength = ALIGN_UP(length, (uint32_t)FSL_FEATURE_FLASH_PFLASH_SECTION_CMD_ADDRESS_ALIGMENT);

        bool verifyWrites;
        status = flash_check_access_before_programming(address, length, &verifyWrites);
        if (status != kStatus_Success)
        {
            return status;
        }

        // Fill unused region with oxFFs.
        assert(length <= sizeof(s_flash_section_program_info[s_flashMemoryIndex].buffer));
        if (length < alignedLength)
        {
            memset(&s_flash_section_program_info[s_flashMemoryIndex].buffer[length], 0xFF, alignedLength - length);
        }

        flash_lock_acquire();
        // Write data of aligned length to flash
        status =
            FLASH_ProgramSection(&g_bootloaderContext.allFlashState[s_flashMemoryIndex], address,
                                 (uint32_t *)s_flash_section_program_info[s_flashMemoryIndex].buffer, alignedLength);
        flash_lock_release();

        if (status != kStatus_Success)
        {
            return status;
        }

// Verify wether the data has been programmed to flash successfully.
#if !BL_FEATURE_FLASH_VERIFY_DISABLE
        // Verify flash program
        if (verifyWrites)
        {
            uint32_t failedAddress;
            uint32_t failedData;

            flash_lock_acquire();
            status = g_bootloaderContext.flashDriverInterface->flash_verify_program(
                &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address, alignedLength,
                (uint32_t *)&s_flash_section_program_info[s_flashMemoryIndex].buffer,
                (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin, &failedAddress,
                &failedData);
            flash_lock_release();
            if (status != kStatus_Success)
            {
                debug_printf("Error: flash verify failed at address: 0x%x\r\n", failedAddress);
                return status;
            }
        }
#endif // !BL_FEATURE_FLASH_VERIFY_DISABLE
    }

    return status;
}
#endif // !BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED

// See memory.h for documentation on this function.
status_t flash_mem_erase(uint32_t address, uint32_t length)
{
    status_t status;

    flash_lock_acquire();
    status = g_bootloaderContext.flashDriverInterface->flash_erase(
        &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address, length, kFLASH_ApiEraseKey);
    flash_lock_release();

#if !BL_FEATURE_FLASH_VERIFY_DISABLE
    if ((status == kStatus_Success) && (g_bootloaderContext.propertyInterface->store->verifyWrites))
    {
        flash_lock_acquire();
        status = g_bootloaderContext.flashDriverInterface->flash_verify_erase(
            &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address, length,
            (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin);
        flash_lock_release();
        if (status != kStatus_Success)
        {
            debug_printf("Error: flash_verify_erase failed\r\n");
            return status;
        }
    }
#endif // !BL_FEATURE_FLASH_VERIFY_DISABLE

    return status;
}

// See memory.h for documentation on this function.
#if BL_FEATURE_FAC_ERASE
status_t flash_mem_erase_all(flash_erase_all_option_t eraseOption)
#else
status_t flash_mem_erase_all(void)
#endif
{
    status_t status = kStatus_Success;

// Decompose the the flash erase all into two region erases.
#if BL_TARGET_FLASH
    reserved_region_t *reservedRegion =
        &g_bootloaderContext.propertyInterface->store->reservedRegions[kProperty_FlashReservedRegionIndex];
    const uint32_t eraseSize = g_bootloaderContext.propertyInterface->store->flashSectorSize[kFlashIndex_Main];

#if BL_FEATURE_FAC_ERASE
    if (eraseOption == kFlashEraseAllOption_ExecuteOnlySegments)
    {
        uint32_t address = ALIGN_DOWN(reservedRegion->startAddress, eraseSize);
        uint32_t length = ALIGN_UP(reservedRegion->endAddress, eraseSize) - address;
        flash_execute_only_access_state_t access_state;

        // If the reserved flash region is in an execute-only protected segment, then FAC erase command is not
        // allowed
        flash_lock_acquire();
        status = flash_preprocess_execute_only_region(address, length, &access_state);
        if (status != kStatus_Success)
        {
            flash_lock_release();
            return status;
        }
        flash_lock_release();

        if (access_state != kFLASH_AccessStateUnLimited)
        {
            return kStatusMemoryAppOverlapWithExecuteOnlyRegion;
        }
    }
    else if (eraseOption == kFlashEraseAllOption_Blocks)
#endif
    {
        memory_map_entry_t *map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexFlashArray];
        bool isReservedRegionEmpty =
            (reservedRegion->startAddress == map->startAddress) && (reservedRegion->endAddress == map->startAddress);
        if (!isReservedRegionEmpty)
        {
            // Erase the initial unreserved region, if any.
            if (reservedRegion->startAddress > map->startAddress)
            {
                uint32_t length = ALIGN_DOWN(reservedRegion->startAddress, eraseSize);
                if (length > 0)
                {
                    status = flash_mem_erase(map->startAddress, length);
                }
            }

            // Erase the final unreserved region, if any.
            if (status == kStatus_Success)
            {
                uint32_t start = ALIGN_UP(reservedRegion->endAddress, eraseSize);
                if (start < map->endAddress)
                {
                    status = flash_mem_erase(start, (map->endAddress + 1) - start);
                }
            }

            return status;
        }
    }

#endif // BL_TARGET_FLASH

    // Do full erase and verify.

    flash_lock_acquire();
#if BL_FEATURE_FAC_ERASE
    if (eraseOption == kFlashEraseAllOption_ExecuteOnlySegments)
    {
        status = g_bootloaderContext.flashDriverInterface->flash_erase_all_execute_only_segments(
            &g_bootloaderContext.allFlashState[s_flashMemoryIndex], kFLASH_ApiEraseKey);
    }
    else if (eraseOption == kFlashEraseAllOption_Blocks)
#endif
    {
        status = g_bootloaderContext.flashDriverInterface->flash_erase_all(
            &g_bootloaderContext.allFlashState[s_flashMemoryIndex], kFLASH_ApiEraseKey);

#if BL_FEATURE_ZEROIZE_SRAM
        zeroize_unreserved_sram();
#endif // BL_FEATURE_ZEROIZE_SRAM
    }
    flash_lock_release();

#if !BL_FEATURE_FLASH_VERIFY_DISABLE
    if ((status == kStatus_Success) && (g_bootloaderContext.propertyInterface->store->verifyWrites))
    {
        flash_lock_acquire();
#if BL_FEATURE_FAC_ERASE
        if (eraseOption == kFlashEraseAllOption_ExecuteOnlySegments)
        {
            status = g_bootloaderContext.flashDriverInterface->flash_verify_erase_all_execute_only_segments(
                &g_bootloaderContext.allFlashState[s_flashMemoryIndex],
                (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin);
        }
        else if (eraseOption == kFlashEraseAllOption_Blocks)
#endif
        {
            status = g_bootloaderContext.flashDriverInterface->flash_verify_erase_all(
                &g_bootloaderContext.allFlashState[s_flashMemoryIndex],
                (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin);
        }
        flash_lock_release();
        if (status != kStatus_Success)
        {
            debug_printf("Error: flash_verify_erase_all/all_execute_only_segments failed\r\n");
            return status;
        }
    }
#endif // !BL_FEATURE_FLASH_VERIFY_DISABLE

#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    if (status == kStatus_Success)
    {
        isFlashRegionUnlocked = true;
    }
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL

    return status;
}

// See memory.h for documentation on this function.
status_t flash_mem_erase_all_unsecure(void)
{
    status_t status;

    flash_lock_acquire();
    status = g_bootloaderContext.flashDriverInterface->flash_erase_all_unsecure(
        &g_bootloaderContext.allFlashState[s_flashMemoryIndex], kFLASH_ApiEraseKey);
    flash_lock_release();

#if BL_FEATURE_ZEROIZE_SRAM
    zeroize_unreserved_sram();
#endif // BL_FEATURE_ZEROIZE_SRAM

    return status;
}

//! @brief check if a flash region is in an XA controlled region or contains an XA controlled region.
//         and try to open flash program state by calling verify_erase_all command if needed.
status_t flash_preprocess_execute_only_region(uint32_t address,
                                              uint32_t length,
                                              flash_execute_only_access_state_t *state)
{
    status_t status = kStatus_Success;
    *state = kFLASH_AccessStateUnLimited;

#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    // If a target flash location is in an execute-only protected segment, these program commands are not
    // allowed unless a Read 1s All Blocks command is executed and returns with a pass code
    flash_execute_only_access_state_t access_state;

    status = g_bootloaderContext.flashDriverInterface->flash_is_execute_only(
        &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address, length, &access_state);
    if (status != kStatus_Success)
    {
        return status;
    }

    *state = access_state;

    if ((access_state != kFLASH_AccessStateUnLimited) && (!isFlashRegionUnlocked))
    {
#if BL_FEATURE_FAC_ERASE
        status = g_bootloaderContext.flashDriverInterface->flash_verify_erase_all_execute_only_segments(
            &g_bootloaderContext.allFlashState[s_flashMemoryIndex],
            (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin);
#else
        status = g_bootloaderContext.flashDriverInterface->flash_verify_erase_all(
            &g_bootloaderContext.allFlashState[s_flashMemoryIndex],
            (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin);
#endif
        if (status != kStatus_Success)
        {
            return kStatus_FLASH_RegionExecuteOnly;
        }

        isFlashRegionUnlocked = true;
    }
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    return status;
}

status_t flash_check_access_before_programming(uint32_t address, uint32_t length, bool *verifyWrites)
{
    status_t status = kStatus_Success;

#if !BL_FEATURE_FLASH_VERIFY_DISABLE
    *verifyWrites = g_bootloaderContext.propertyInterface->store->verifyWrites;
#endif // BL_FEATURE_FLASH_VERIFY_DISABLE

#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    // If a target flash location is in an execute-only protected segment, these program commands are not
    // allowed unless a Read 1s All Blocks command is executed and returns with a pass code
    flash_execute_only_access_state_t access_state;

    uint32_t actualLength = ALIGN_UP(length, (uint32_t)FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE);
    flash_lock_acquire();
    status = flash_preprocess_execute_only_region(address, actualLength, &access_state);
    if (status != kStatus_Success)
    {
        flash_lock_release();
        return status;
    }
    flash_lock_release();

#if !BL_FEATURE_FLASH_VERIFY_DISABLE
    if (access_state != kFLASH_AccessStateUnLimited)
    {
        *verifyWrites = false;
    }
#endif // BL_FEATURE_FLASH_VERIFY_DISABLE

#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL

// Do cumulative write check
#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE || FSL_FEATURE_SOC_FTFE_COUNT
    bool isCumulativeCheckNeeded = true;
#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    if (access_state != kFLASH_AccessStateUnLimited)
    {
        isCumulativeCheckNeeded = false;
    }
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    if (isCumulativeCheckNeeded)
    {
        uint32_t actualLength = ALIGN_UP(length, (uint32_t)FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE);
#if BL_FEATURE_ENABLE_BOOT_FROM_EITHER_CORE
        if (!flash_check_is_erased(address, actualLength))
#else
        if (!mem_is_erased(address, actualLength))
#endif
        {
            return kStatusMemoryCumulativeWrite;
        }
    }
#endif // BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE || FSL_FEATURE_SOC_FTFE_COUNT

    return status;
}

#if BL_FEATURE_ENABLE_BOOT_FROM_EITHER_CORE
static inline bool flash_check_is_erased(uint32_t address, uint32_t length)
{
    status_t status;

    uint64_t erasedData = (uint64_t)kFlashMemory_ErasedValue;

    // Not really used. Only check the returned status.
    uint32_t failedAddress;
    uint32_t failedData;

    uint8_t offset = 0;
    while (offset < length)
    {
        status = g_bootloaderContext.flashDriverInterface->flash_verify_program(
            &g_bootloaderContext.allFlashState[s_flashMemoryIndex], address + offset,
            FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE, (uint32_t *)&erasedData,
            (flash_margin_value_t)g_bootloaderContext.propertyInterface->store->flashReadMargin, &failedAddress,
            &failedData);
        if (status != kStatus_Success)
        {
            return false;
        }
        offset += FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE;
    }
    return true;
}
#endif
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
