/*
 * Copyright 2017 -2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "memory/memory.h"
#include "flash_c040hd_memory.h"
#include "normal_memory.h"
#include "fsl_iap.h"
#include "bootloader/bl_context.h"
#include "bootloader/bootloader.h"
#include "fsl_device_registers.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/fsl_assert.h"
#include <string.h>
#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
#include "authentication/bus_crypto_engine_hal.h"
#endif

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

//! @brief Flash page program memory context
//!
//! An instance is maintained in this file, will is used to keep key information for write and flush
//! operatations.
typedef struct _flash_page_program_info
{
    //!< This address is used to record the address which is used to write the whole buffer into flash memory
    uint32_t startAddress;
    //!< A variable which is used to indicate if the buffer is full.
    uint32_t storedBytes;
#if BL_FETUARE_USE_SHARED_MEMORY_INTERFACE_BUFFER
    uint8_t *buffer;
#else
    //!< A buffer which is used to buffer a full package of data
    uint8_t buffer[FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES];
#endif // BL_FETUARE_USE_SHARED_MEMORY_INTERFACE_BUFFER
} flash_page_program_info_t;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
//! @brief flash memory array.
flash_config_t g_flashState[] = { { 0 } };

//! @brief Interface to flash memory operations.
const memory_region_interface_t g_flashMemoryInterface = {.init = &flash_mem_init,
                                                          .read = &flash_mem_read,
                                                          .write = &flash_mem_write,
#if !BL_FEATURE_MIN_PROFILE || BL_FEATURE_FILL_MEMORY
                                                          .fill = &flash_mem_fill,
#endif // !BL_FEATURE_MIN_PROFILE
                                                          .flush = &flash_mem_flush,
                                                          .erase = flash_mem_erase,
#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
                                                          .config = &flash_config,
#endif // BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
                                                          .erase_all = flash_mem_erase_all };

#if BL_TARGET_FLASH
static uint32_t s_regPrimask = 0U;
#endif

static flash_page_program_info_t s_flash_page_program_info;

#if BL_FETUARE_USE_SHARED_MEMORY_INTERFACE_BUFFER
static const uint32_t k_programBufferSize = FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES;
#else
static const uint32_t k_programBufferSize = sizeof(s_flash_page_program_info.buffer);
#endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

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

// See flash_c040hd_memory.h for documentation on this function.
status_t flash_mem_init(void)
{
    // Update address range of flash
    memory_map_entry_t *map;
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexFlashArray];
    g_bootloaderContext.flashDriverInterface->flash_get_property(
        &g_bootloaderContext.allFlashState[0], kFLASH_PropertyPflashBlockBaseAddr, &map->startAddress);
    uint32_t tmp;
    g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[0],
                                                                 kFLASH_PropertyPflashTotalSize, &tmp);
    // Niobe4 has phantom with smaller flash size, if it is samller phantom, we don't need to
    //  minus FFR size here, as FFR base address is always fixed.
    if (tmp > 0u)
    {
        map->endAddress = map->startAddress + tmp - 1u;
    }
    else
    {
        map->endAddress = map->startAddress;
    }

#if BL_FEATURE_HAS_ALIAS_FLASH_ADDR
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexFlashArrayAlias];
    if (tmp > 0u)
    {
        map->endAddress = map->startAddress + tmp - 1u;
    }
    else
    {
        map->endAddress = map->startAddress;
    }
#endif

    s_flash_page_program_info.storedBytes = 0u;

#if BL_FETUARE_USE_SHARED_MEMORY_INTERFACE_BUFFER
    s_flash_page_program_info.buffer = g_memoryBuffer;
#endif // BL_FETUARE_USE_SHARED_MEMORY_INTERFACE_BUFFER

    return kStatus_Success;
}

// See flash_c040hd_memory.h for documentation on this function.
status_t flash_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
    uint32_t pageSize = g_bootloaderContext.allFlashState[0].PFlashPageSize;
    uint32_t flashSize = g_bootloaderContext.allFlashState[0].PFlashTotalSize;
    if (flashSize < 1)
    {
        return kStatus_CommandUnsupported;
    }

    uint32_t alignedAddress = ALIGN_DOWN(address, pageSize);
    uint32_t alignedLength = ALIGN_UP(length, pageSize);
    while (alignedLength)
    {
        uint32_t physicalAddr = GET_PHYSICAL_ADDR(alignedAddress);
        status_t status = g_bootloaderContext.flashDriverInterface->flash_verify_erase(
            &g_bootloaderContext.allFlashState[0], physicalAddr, pageSize);
        if (status == kStatus_Success)
        {
            return kStatusMemoryBlankPageReadDisallowed;
        }
#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
        /* If the area falls within PRINCE encrypted region return error. */
        if (kSECURE_FALSE == skboot_hal_bus_crypto_engine_is_read_allowed(physicalAddr, pageSize))
        {
            return kStatusMemoryProtectedPageReadDisallowed;
        }
#endif // BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
        alignedLength -= pageSize;
        alignedAddress += pageSize;
    }
    uint32_t physicalAddr = GET_PHYSICAL_ADDR(address);
    return FLASH_Read(&g_bootloaderContext.allFlashState[0], physicalAddr, buffer, length);
}

// See flash_c040hd_memory.h for documentation on this function.
status_t flash_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_write().
    assert(length);
    assert(buffer);

    status_t status = kStatus_Success;

    uint32_t flashSize = g_bootloaderContext.allFlashState[0].PFlashTotalSize;
    if (flashSize < 1)
    {
        return kStatus_CommandUnsupported;
    }

    while (length)
    {
        // Set start address when storing first byte into section program buffer
        if ((!s_flash_page_program_info.storedBytes) && (!s_flash_page_program_info.startAddress))
        {
            // Check address alignment
            if (address & (FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES - 1))
            {
                return kStatus_FLASH_AlignmentError;
            }
            s_flash_page_program_info.startAddress = address;
        }
        else
        {
            // Start page programming operation when meet discontinuous address
            if ((s_flash_page_program_info.startAddress + s_flash_page_program_info.storedBytes) != address)
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
        // Check to see if page program buffer will be filled with current data packet
        if ((s_flash_page_program_info.storedBytes + length) <= k_programBufferSize)
        {
            storeBytes = length;
        }
        else
        {
            storeBytes = k_programBufferSize - s_flash_page_program_info.storedBytes;
        }

        // Copy data to section program buffer
        if (buffer != &s_flash_page_program_info.buffer[s_flash_page_program_info.storedBytes])
        {
            memcpy(&s_flash_page_program_info.buffer[s_flash_page_program_info.storedBytes], buffer, storeBytes);
        }

        s_flash_page_program_info.storedBytes += storeBytes;
        buffer += storeBytes;
        address += storeBytes;
        length -= storeBytes;

        // Start page programming operation when section program buffer is full
        if (s_flash_page_program_info.storedBytes == k_programBufferSize)
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

// See flash_c040hd_memory.h for documentation on this function.
status_t flash_mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_fill().
    assert(length);

    status_t status;

    uint32_t flashSize = g_bootloaderContext.allFlashState[0].PFlashTotalSize;
    if (flashSize < 1)
    {
        return kStatus_CommandUnsupported;
    }

    // Pre-fill section program buffer with pattern
    uint32_t *buffer = (uint32_t *)s_flash_page_program_info.buffer;
    uint32_t maxPatterns = k_programBufferSize >> 2;
    for (uint32_t i = 0; i < maxPatterns; i++)
    {
        *buffer++ = pattern;
    }

    while (length)
    {
        uint32_t bytes;

        s_flash_page_program_info.storedBytes = 0;

        // Check to see if remaining address range can hold whole page program buffer
        if (length < k_programBufferSize)
        {
            bytes = length;
        }
        else
        {
            bytes = k_programBufferSize;
        }

        // flush cached data into target memory,
        status = flash_mem_write(address, bytes, s_flash_page_program_info.buffer);
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

// See flash_c040hd_memory.h for documentation on this function.
status_t flash_mem_flush(void)
{
    status_t status = kStatus_Success;

    if (s_flash_page_program_info.storedBytes)
    {
        uint32_t address = s_flash_page_program_info.startAddress;
        uint32_t length = s_flash_page_program_info.storedBytes;

        // Clear related states no matter following operations are executed successfully or not.
        s_flash_page_program_info.startAddress = 0;
        s_flash_page_program_info.storedBytes = 0;

        // Align length to page program unit
        uint32_t alignedLength = ALIGN_UP(length, (uint32_t)FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES);

        // Fill unused region with ox00s.
        assert(length <= k_programBufferSize);
        if (length < alignedLength)
        {
            memset(&s_flash_page_program_info.buffer[length], 0x00, alignedLength - length);
        }

        uint32_t physicalAddr = GET_PHYSICAL_ADDR(address);
#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
        status = g_bootloaderContext.flashDriverInterface->flash_verify_erase(&g_bootloaderContext.allFlashState[0],
                                                                              physicalAddr, alignedLength);
        if (status != kStatus_Success)
        {
            return kStatusMemoryCumulativeWrite;
        }
#endif
        flash_lock_acquire();
        // Write data of aligned length to flash
        status = g_bootloaderContext.flashDriverInterface->flash_program(
            &g_bootloaderContext.allFlashState[0], physicalAddr, (uint8_t *)s_flash_page_program_info.buffer,
            alignedLength);
        flash_lock_release();

        if (status != kStatus_Success)
        {
            return status;
        }

// Verify wether the data has been programmed to flash successfully.
#if !BL_FEATURE_FLASH_VERIFY_DISABLE
        bool verifyWrites = g_bootloaderContext.propertyInterface->store->verifyWrites;
#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
        /* Do not perform the verification when the encryption is enabled, othervise the flash driver error is returned.
         */
        if (verifyWrites && (kSECURE_FALSE == skboot_hal_bus_crypto_engine_is_encryption_enabled()))
#else
        if (verifyWrites)
#endif // BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
        {
            uint32_t failedAddress;
            uint32_t failedData;

            flash_lock_acquire();
            status = g_bootloaderContext.flashDriverInterface->flash_verify_program(
                &g_bootloaderContext.allFlashState[0], physicalAddr, alignedLength,
                (uint8_t *)&s_flash_page_program_info.buffer[0], &failedAddress, &failedData);
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

// See memory.h for documentation on this function.
status_t flash_mem_erase(uint32_t address, uint32_t length)
{
    status_t status;

    uint32_t flashSize = g_bootloaderContext.allFlashState[0].PFlashTotalSize;
    if (flashSize < 1)
    {
        return kStatus_CommandUnsupported;
    }

    uint32_t physicalAddr = GET_PHYSICAL_ADDR(address);
    flash_lock_acquire();
    status = g_bootloaderContext.flashDriverInterface->flash_erase(&g_bootloaderContext.allFlashState[0], physicalAddr,
                                                                   length, kFLASH_ApiEraseKey);
    flash_lock_release();

#if !BL_FEATURE_FLASH_VERIFY_DISABLE
    if ((status == kStatus_Success) && (g_bootloaderContext.propertyInterface->store->verifyWrites))
    {
        flash_lock_acquire();
        status = g_bootloaderContext.flashDriverInterface->flash_verify_erase(&g_bootloaderContext.allFlashState[0],
                                                                              physicalAddr, length);
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
status_t flash_mem_erase_all(void)
{
    status_t status = kStatus_Success;

    uint32_t flashSize = g_bootloaderContext.allFlashState[0].PFlashTotalSize;
    if (flashSize < 1)
    {
        return kStatus_CommandUnsupported;
    }

    // Decompose the the flash erase all into two region erases.
    reserved_region_t *reservedRegion =
        &g_bootloaderContext.propertyInterface->store->reservedRegions[kProperty_FlashReservedRegionIndex];
    const uint32_t eraseSize = g_bootloaderContext.propertyInterface->store->flashSectorSize[kFlashIndex_Main];
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
        }
        else
        {
            status = flash_mem_erase(map->startAddress, (map->endAddress + 1) - map->startAddress);
        }
    }

    return status;
}

#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
// See memory.h for documentation on this function.
status_t flash_config(uint32_t *config)
{
    uint32_t flashSize = g_bootloaderContext.allFlashState[0].PFlashTotalSize;
    if (flashSize < 1)
    {
        return kStatus_CommandUnsupported;
    }

    uint32_t startAddr = (uint32_t)config;
    uint32_t endAddr = startAddr + sizeof(prince_prot_region_arg_t) - 1;
    // Should check the config is in valid internal space.
    if ((!is_valid_application_location(startAddr)) || (!is_valid_application_location(endAddr)))
    {
        return kStatus_InvalidArgument;
    }

    const prince_prot_region_arg_t *princeConfig = (const prince_prot_region_arg_t *)config;
    // Check mem config tag.
    if ((config == NULL) || (princeConfig->option.tag != kPrinceProtRegionArg_Option_Tag))
    {
        return kStatus_InvalidArgument;
    }
    // Confirmed with Design team, only physical address is supported by PRINCE, so here the memory interface forces to
    // use the physical address
    uint32_t physicalAddr = GET_PHYSICAL_ADDR(princeConfig->start);
    if (kStatus_Success == skboot_hal_bus_crypto_engine_set_encrypt_for_address_range(
                                      princeConfig->option.target_prince_region, physicalAddr, princeConfig->length,
                                      &g_bootloaderContext.allFlashState[0]))
    {
        return kStatus_Success;
    }
    else
    {
        return kStatus_Fail;
    }
}
#endif // BL_FEATURE_HAS_BUS_CRYPTO_ENGINE

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
