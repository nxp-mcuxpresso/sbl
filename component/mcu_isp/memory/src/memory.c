/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "memory/memory.h"
#if !defined(BOOTLOADER_HOST)
#if BL_FEATURE_HAS_INTERNAL_FLASH
#include "fsl_iap.h"
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#include "fsl_device_registers.h"
#if BL_FEATURE_HAS_INTERNAL_FLASH
#include "flash_memory.h"
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#endif // BOOTLOADER_HOST
#include "utilities/fsl_assert.h"

//! @addtogroup memif
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

// Forward function declarations.
bool mem_is_block_reserved(uint32_t address, uint32_t length);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief This variable is used to do flush operation, it is bind to write operation.
#if BL_FEATURE_MEM_WRITE_ENABLE
static status_t (*s_flush)(void) = NULL;
#endif
#if BL_FEATURE_EXPAND_MEMORY
static status_t (*s_finalize)(void) = NULL;
#endif // BL_FEATURE_EXPAND_MEMORY

//! @brief Interface to generic memory operations.
const memory_interface_t g_memoryInterface = {
    .init = mem_init,
    .read = mem_read,
    .check = mem_check,
#if BL_FEATURE_MEM_WRITE_ENABLE
    .write = mem_write,
#if !BL_FEATURE_MIN_PROFILE || BL_FEATURE_FILL_MEMORY
    .fill = mem_fill,
#else
    .fill = NULL,
#endif // !BL_FEATURE_MIN_PROFILE
    .flush = mem_flush,
#if BL_FEATURE_EXPAND_MEMORY
    .finalize = mem_finalize,
#else
    .finalize = NULL,
#endif // BL_FEATURE_EXPAND_MEMORY
    .erase = mem_erase,
#endif // BL_FEATURE_MEM_WRITE_ENABLE
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See memory.h for documentation on this function.
status_t mem_read(uint32_t address, uint32_t length, uint8_t *buffer, uint32_t memoryId)
{
    if (length == 0)
    {
        return kStatus_Success;
    }

#if BL_FEATURE_EXPAND_MEMORY
    switch (GROUPID(memoryId))
    {
        case kGroup_Internal:
        {
#endif // BL_FEATURE_EXPAND_MEMORY
            const memory_map_entry_t *mapEntry;
            status_t status = find_map_entry(address, length, &mapEntry);
            if (status == kStatus_Success)
            {
                status = mapEntry->memoryInterface->read(address, length, buffer);
            }
            return status;
#if BL_FEATURE_EXPAND_MEMORY
        }
        // "break;" is not needed for always executing return.
        case kGroup_External:
        {
            const external_memory_map_entry_t *mapEntry;
            status_t status = find_external_map_entry(address, length, memoryId, &mapEntry);
            if (status == kStatus_Success)
            {
                s_finalize = mapEntry->memoryInterface->finalize;
                status = mapEntry->memoryInterface->read(address, length, buffer);
            }
            return status;
        }
        // "break;" is not needed for always executing return.
        default:
            return kStatusMemoryRangeInvalid;
    }
#endif // BL_FEATURE_EXPAND_MEMORY
}

// See memory.h for documentation on this function.
status_t mem_check(uint32_t address, uint32_t length, uint32_t memoryId)
{
    if (length == 0)
    {
        return kStatus_InvalidArgument;
    }

#if BL_FEATURE_EXPAND_MEMORY
    switch (GROUPID(memoryId))
    {
        case kGroup_Internal:
        {
#endif // BL_FEATURE_EXPAND_MEMORY
            if (mem_is_block_reserved(address, length))
            {
                return kStatusMemoryRangeInvalid;
            }

            const memory_map_entry_t *mapEntry;
            return find_map_entry(address, length, &mapEntry);
#if BL_FEATURE_EXPAND_MEMORY
        }
        // "break;" is not needed for always executing return.
        case kGroup_External:
        {
            const external_memory_map_entry_t *mapEntry;
            return find_external_map_entry(address, length, memoryId, &mapEntry);
        }
        // "break;" is not needed for always executing return.
        default:
            return kStatusMemoryRangeInvalid;
    }
#endif // BL_FEATURE_EXPAND_MEMORY
}

#if BL_FEATURE_MEM_WRITE_ENABLE

// See memory.h for documentation on this function.
status_t mem_write(uint32_t address, uint32_t length, const uint8_t *buffer, uint32_t memoryId)
{
    if (length == 0)
    {
        return kStatus_Success;
    }

#if BL_FEATURE_EXPAND_MEMORY
    switch (GROUPID(memoryId))
    {
        case kGroup_Internal:
        {
#endif // BL_FEATURE_EXPAND_MEMORY
            if (mem_is_block_reserved(address, length))
            {
                return kStatusMemoryRangeInvalid;
            }

            const memory_map_entry_t *mapEntry;
            status_t status = find_map_entry(address, length, &mapEntry);
            if (status == kStatus_Success)
            {
                status = mapEntry->memoryInterface->write(address, length, buffer);

                if (status == kStatus_Success)
                {
                    s_flush = mapEntry->memoryInterface->flush;
                }
                else
                {
                    s_flush = NULL;
                }
            }
            return status;
#if BL_FEATURE_EXPAND_MEMORY
        }
        // "break;" is not needed for always executing return.
        case kGroup_External:
        {
            const external_memory_map_entry_t *mapEntry;
            status_t status = find_external_map_entry(address, length, memoryId, &mapEntry);
            if (status == kStatus_Success)
            {
                s_finalize = mapEntry->memoryInterface->finalize;
                status = mapEntry->memoryInterface->write(address, length, buffer);
                if (status == kStatus_Success)
                {
                    s_flush = mapEntry->memoryInterface->flush;
                }
                else
                {
                    s_flush = NULL;
                }
            }
            return status;
        }
        // "break;" is not needed for always executing return.
        default:
            return kStatusMemoryRangeInvalid;
    }
#endif // BL_FEATURE_EXPAND_MEMORY
}

status_t mem_erase(uint32_t address, uint32_t length, uint32_t memoryId)
{
    status_t status = kStatus_Success;

#if !defined(BOOTLOADER_HOST)

#if BL_FEATURE_EXPAND_MEMORY
    switch (GROUPID(memoryId))
    {
        case kGroup_Internal:
        {
#endif // BL_FEATURE_EXPAND_MEMORY
            const memory_map_entry_t *mapEntry;
            status = find_map_entry(address, length, &mapEntry);
            if (status == kStatus_Success)
            {
                // In this case, it means that bootloader tries to erase a range of memory
                // which doesn't support erase operaton
                if (mapEntry->memoryInterface->erase == NULL)
                {
                    return kStatusMemoryUnsupportedCommand;
                }

                if (mem_is_block_reserved(address, length))
                {
                    return kStatusMemoryRangeInvalid;
                }

                status = mapEntry->memoryInterface->erase(address, length);
            }
            else if (length == 0)
            {
                // if length = 0, return kStatus_Success regardless of memory address
                return kStatus_Success;
            }
#if BL_FEATURE_EXPAND_MEMORY
        }
        break;
        case kGroup_External:
        {
            const external_memory_map_entry_t *mapEntry;
            status = find_external_map_entry(address, length, memoryId, &mapEntry);
            if (status == kStatus_Success)
            {
                if (mapEntry->memoryInterface->erase == NULL)
                {
                    return kStatusMemoryUnsupportedCommand;
                }
                status = mapEntry->memoryInterface->erase(address, length);
            }
            else if (length == 0)
            {
                return kStatus_Success;
            }
        }
        break;
        default:
            status = kStatusMemoryRangeInvalid;
    }
#endif // BL_FEATURE_EXPAND_MEMORY
#endif // BOOTLOADER_HOST
    return status;
}

// See memory.h for documentation on this function.
status_t mem_erase_all(uint32_t memoryId)
{
    status_t status = kStatus_InvalidArgument;
    bool isMemoryInterfaceFound = false;
    switch (GROUPID(memoryId))
    {
        case kGroup_Internal:
        {
#if BL_FEATURE_FAC_ERASE
            if (memoryId == kMemoryFlashExecuteOnly)
            {
                status = flash_mem_erase_all(kFlashEraseAllOption_ExecuteOnlySegments);
                break;
            }
            else if (memoryId == kMemoryInternal)
            {
                status = flash_mem_erase_all(kFlashEraseAllOption_Blocks);
                break;
            }
#endif // BL_FEATURE_FAC_ERASE
            const memory_map_entry_t *map = &g_bootloaderContext.memoryMap[0];
            while (map->memoryInterface != NULL)
            {
                if (map->memoryId == memoryId)
                {
                    isMemoryInterfaceFound = true;
                    break;
                }
                ++map;
            }
            if (isMemoryInterfaceFound)
            {
                if (map->memoryInterface->erase_all != NULL)
                {
                    status = map->memoryInterface->erase_all();
                }
            }
        }
        break;
#if BL_FEATURE_EXPAND_MEMORY
        case kGroup_External:
        {
            external_memory_map_entry_t *extMap =
                (external_memory_map_entry_t *)&g_bootloaderContext.externalMemoryMap[0];
            while (extMap->memoryInterface != NULL)
            {
                if (extMap->memoryId == memoryId)
                {
                    isMemoryInterfaceFound = true;
                    break;
                }
                ++extMap;
            }

            if (isMemoryInterfaceFound)
            {
                if (extMap->memoryInterface->erase_all != NULL)
                {
                    status = extMap->memoryInterface->erase_all();
                }
            }
        }
        break;
#endif // #if BL_FEATURE_EXPAND_MEMORY
    }

    return status;
}

// See memory.h for documentation on this function.
status_t mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    if (length == 0)
    {
        return kStatus_Success;
    }

    if (mem_is_block_reserved(address, length))
    {
        return kStatusMemoryRangeInvalid;
    }

    const memory_map_entry_t *mapEntry;
    status_t status = find_map_entry(address, length, &mapEntry);
    if (status == kStatus_Success)
    {
        if (mapEntry->memoryInterface->fill != NULL)
        {
            status = mapEntry->memoryInterface->fill(address, length, pattern);
        }
        else
        {
            status = kStatusMemoryUnsupportedCommand;
        }
    }
    return status;
}

//! @brief Flush buffered data into target memory
//! @note  1. This function should be called immediately after one write-memory command(either
//!        received in command packet or in sb file), only in this way, given data can be programmed
//!        at given address as expected.
//!
//!        2. So far, flush() is only implemented in qspi memory interface, for other memory
//!        interfaces, it is not available and mem_flush() just returns kStatus_Success if it is
//!        called.
//!
//!        3. This function is designed to flush buffered data into target memory, please call it
//!        only if it is required to do so. For example, write 128 bytes to qspi flash, while the
//!        page size is 256 bytes, that means data might not be written to qspi memory immediately,
//!        since the internal buffer of qspi memory interface is not full, if no data are expected
//!        to write to left area of the same page, this function can be used to force to write
//!        immediately, otherwise, keep in mind that any calls should be avoided. If users voilate
//!        this rules, it would make the left area of the same page cannot be programmed.
//!
//! @return An error code or kStatus_Success
status_t mem_flush(void)
{
    status_t status = kStatus_Success;

#if !defined(BOOTLOADER_HOST)
    if (s_flush)
    {
        status = s_flush();
        s_flush = NULL; // Clear this variable after performing flush operation
        return status;
    }
#endif // BOOTLOADER_HOST

    return status;
}

//! @brief Reset the state machine of memory interface when a read/write sequence is finished.
//!
//! @return An error code or kStatus_Success
#if BL_FEATURE_EXPAND_MEMORY
status_t mem_finalize(void)
{
    status_t status = kStatus_Success;

#if !defined(BOOTLOADER_HOST)
    if (s_finalize)
    {
        status = s_finalize();
        s_finalize = NULL; // Clear this variable after performing finalize operation
        return status;
    }
#endif // BOOTLOADER_HOST

    return status;
}
#endif // BL_FEATURE_EXPAND_MEMORY

#endif // BL_FEATURE_MEM_WRITE_ENABLE

//! @brief Find a map entry that matches address and length.
//!
//! @param address Start address for the memory operation.
//! @param length Number of bytes on which the operation will act.
//! @param map The matching map entry is returned through this pointer if the return status
//!     is #kStatus_Success.
//!
//! @retval #kStatus_Success A valid map entry was found and returned through @a map.
//! @retval #kStatusMemoryRangeInvalid The requested address range does not match an entry, or
//!     the length extends past the matching entry's end address.
status_t find_map_entry(uint32_t address, uint32_t length, const memory_map_entry_t **map)
{
    status_t status = kStatusMemoryRangeInvalid;

    do
    {
        if (map == NULL)
        {
            break;
        }

        // Set starting entry.
        *map = &g_bootloaderContext.memoryMap[0];

        uint64_t end = address + length - 1;
        if (end > UINT32_MAX)
        {
            break;
        }

        // Scan memory map array looking for a match.
        while ((length > 0) && map && *map)
        {
            if (((*map)->startAddress == 0) && ((*map)->endAddress == 0) && ((*map)->memoryInterface == NULL))
            {
                break;
            }
            // Check if the start address is within this entry's address range.
            if ((address >= (*map)->startAddress) && (address <= (*map)->endAddress))
            {
                // Check that the length fits in this entry's address range.
                if (end <= (*map)->endAddress)
                {
                    status = kStatus_Success;
                }
                break;
            }
            ++(*map);
        }
    } while (0);

    return status;
}

// See memory.h for documentation on this function.
#if BL_FEATURE_EXPAND_MEMORY
status_t find_external_map_entry(uint32_t address,
                                 uint32_t length,
                                 uint32_t memory_id,
                                 const external_memory_map_entry_t **map)
{
    status_t status = kStatusMemoryRangeInvalid;

    // Set starting entry.
    assert(map);
    if (map)
    {
        *map = &g_bootloaderContext.externalMemoryMap[0];
    }

    // Scan memory map array looking for a match.
    while ((length > 0) && map && *map)
    {
        if (((*map)->memoryId == 0) && ((*map)->status == 0) && ((*map)->basicUnitCount == 0) &&
            ((*map)->basicUnitSize == 0) && ((*map)->memoryInterface == NULL))
        {
            break;
        }

        // Check if the memory id is matched.
        if (memory_id == (*map)->memoryId)
        {
            // Check that the length fits in this entry's address range.
            if (((uint64_t)address + length) <= ((uint64_t)(*map)->basicUnitCount * (*map)->basicUnitSize))
            {
                status = kStatus_Success;
            }
            break;
        }
        ++(*map);
    }

    return status;
}

status_t find_external_map_index(uint32_t memoryId, uint32_t *index)
{
    status_t status = kStatus_InvalidArgument;

    const external_memory_map_entry_t *map;
    uint32_t searchingIndex = 0;

    if (index == NULL)
    {
        return status;
    }

    map = &g_bootloaderContext.externalMemoryMap[0];
    // Scan memory map array looking for a match.
    while (map && (map->memoryId != 0) && (map->memoryInterface != NULL))
    {
        if (memoryId == map->memoryId)
        {
            *index = searchingIndex;
            // Find the correct index.
            status = kStatus_Success;
            break;
        }
        searchingIndex++;
        map++;
    };

    return status;
}
#endif // BL_FEATURE_EXPAND_MEMORY

// See memory.h for documentation on this function.
bool mem_is_block_reserved(uint32_t address, uint32_t length)
{
    uint32_t end = address + length - 1;
    uint32_t start = 0;
    for (uint32_t i = 0; i < kProperty_ReservedRegionsCount; ++i)
    {
        reserved_region_t *region = &g_bootloaderContext.propertyInterface->store->reservedRegions[i];

        start = (&g_bootloaderContext.memoryMap[i])->startAddress;
        if ((region->startAddress == start) && (region->endAddress == start))
        {
            // Special case, empty region
            continue;
        }

        if ((address <= region->endAddress) && (end >= region->startAddress))
        {
            return true;
        }
    }
    return false;
}

// See memory.h for documentation on this function.
status_t mem_init(void)
{
    status_t status = kStatus_Success;

#if !defined(BOOTLOADER_HOST)

    const memory_map_entry_t *map = &g_bootloaderContext.memoryMap[0];

    while (map->memoryInterface)
    {
        if (map->memoryInterface->init)
        {
            map->memoryInterface->init();
        }
        ++map;
    }

#if BL_FEATURE_EXPAND_MEMORY
    external_memory_map_entry_t *extMap = (external_memory_map_entry_t *)&g_bootloaderContext.externalMemoryMap[0];
    while (extMap->memoryInterface)
    {
        if (extMap->memoryInterface->init)
        {
            status = extMap->memoryInterface->init();
        }
        ++extMap;
    }
#endif // BL_FEATURE_EXPAND_MEMORY
#endif // BOOTLOADER_HOST

    return status;
}

// See memory.h for documentation on this function.
status_t mem_config(uint32_t memoryId, void *config)
{
    status_t status = kStatus_InvalidArgument;
    bool isMemoryInterfaceFound = false;
    switch (GROUPID(memoryId))
    {
        case kGroup_Internal:
        {
            const memory_map_entry_t *map = &g_bootloaderContext.memoryMap[0];
            while (map->memoryInterface != NULL)
            {
                if (map->memoryId == memoryId)
                {
                    isMemoryInterfaceFound = true;
                    break;
                }
                ++map;
            }
            if (isMemoryInterfaceFound)
            {
                if (map->memoryInterface->config != NULL)
                {
                    status = map->memoryInterface->config(config);
                }
            }
        }
        break;
#if BL_FEATURE_EXPAND_MEMORY
        case kGroup_External:
        {
            external_memory_map_entry_t *extMap =
                (external_memory_map_entry_t *)&g_bootloaderContext.externalMemoryMap[0];
            while (extMap->memoryInterface != NULL)
            {
                if (extMap->memoryId == memoryId)
                {
                    isMemoryInterfaceFound = true;
                    break;
                }
                ++extMap;
            }

            if (isMemoryInterfaceFound)
            {
                status = extMap->memoryInterface->config(config);
            }
        }
        break;
#endif // #if BL_FEATURE_EXPAND_MEMORY
    }

    return status;
}

// See memory.h for documentation on this function.
bool mem_is_erased(uint32_t address, uint32_t length)
{
    const uint8_t *start = (const uint8_t *)address;
    bool isMemoryErased = true;

    while (length)
    {
        if (*start != 0xFF)
        {
            isMemoryErased = false;
            break;
        }
        else
        {
            length--;
            start++;
        }
    }

    return isMemoryErased;
}

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
