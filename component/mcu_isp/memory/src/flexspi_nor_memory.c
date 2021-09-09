/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "fsl_device_registers.h"
#include "bootloader_common.h"
#include "bootloader/bootloader.h"
#include "memory/memory.h"
#include "normal_memory.h"
#include "flexspi_nor_memory.h"
#include "bootloader/bl_context.h"
#include "microseconds/microseconds.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/fsl_assert.h"
#include <string.h>

#if BL_FEATURE_FLEXSPI_NOR_MODULE

#ifndef FLEXSPI_NOR_INSTANCE
#define FLEXSPI_NOR_INSTANCE BL_FEATURE_FLEXSPI_NOR_MODULE_PERIPHERAL_INSTANCE
#endif

#ifndef FLEXSPI_NOR_ERASE_VERIFY
#define FLEXSPI_NOR_ERASE_VERIFY BL_FEATURE_FLEXSPI_NOR_MODULE_ERASE_VERIFY
#endif

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum
{
    kFlashDefaultPattern = 0xFF,
};

//! @brief FLEXSPI NOR memory feature information
//!
//! An instance is maintained in this file, will is used to keep key information for write and flush
//! operations.
typedef struct _flexspi_nor_mem_context
{
    bool isConfigured; //!< The state which indicates whether FlexSPI block is successfully configured.
#if BL_FEATURE_MEM_WRITE_ENABLE
    bool isAddingToBuffer;            //!< State used of determine whether it is the first write at
                                      //!< Start address of one page
    uint32_t writeAddress;            //!< This address is used to record the address which is used
                                      //!< to write the whole page into FlexSPI memory
    uint32_t offset;                  //!< A variable which is used to indicate if the buffer is
                                      //!< full.
    uint32_t nextStartAddress;        //!< A variable is used to indicate if recent two writes
                                      //!< are continuous
    uint8_t buffer[kFlexSpiNorMemory_MaxPageSize]; //!< A buffer which is used to buffer a full
                                                   //!< page of data
#endif // BL_FEATURE_MEM_WRITE_ENABLE
#if BL_FEATURE_FLEXSPI_NOR_MODULE
    uint32_t instance;
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE_PERIPHERAL_INSTANCE
} flexspi_nor_mem_context_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
//! @brief Convert flexspi amba address to physical address
static uint32_t flexspi_get_phy_address(uint32_t mapAddr);
//! @brief Erase function for flexspi nor module driver
static status_t flexspi_nor_memory_erase(uint32_t address, uint32_t length);

//! @brief verify if serial nor memory is erased.
static bool is_flexspi_nor_mem_erased(uint32_t start, uint32_t length);
////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Context of Flexspi operation.
static flexspi_nor_mem_context_t s_flexspiNorContext = {
    .isConfigured = false,
#if BL_FEATURE_MEM_WRITE_ENABLE
    .isAddingToBuffer = false,
#endif // BL_FEATURE_MEM_WRITE_ENABLE
#if BL_FEATURE_FLEXSPI_NOR_MODULE
    .instance = BL_FEATURE_FLEXSPI_NOR_MODULE_PERIPHERAL_INSTANCE,
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE
};

//! @brief Interface to flexspi memory operations
const memory_region_interface_t g_flexspiMemoryInterface = {
    .init = flexspi_nor_mem_init,
    .read = flexspi_nor_mem_read,
#if BL_FEATURE_MEM_WRITE_ENABLE
    .write = flexspi_nor_mem_write,
    .erase = flexspi_nor_mem_erase,
    .flush = flexspi_nor_mem_flush,
#endif // BL_FEATURE_MEM_WRITE_ENABLE
};

#if BL_FEATURE_FLEXSPI_NOR_MODULE
//static const uint32_t s_flexspiInstances[] = FLEXSPI_AMBA_BASE_ADDS;
static const uint32_t s_flexspiMmapIndexes[] = { kIndexFlexSpiNor, kIndexFlexSpiNor, kIndexFlexSpiNor2 };
#endif

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Get the status of flexspi configuration
bool is_flexspi_nor_configured()
{
    return s_flexspiNorContext.isConfigured;
}

void flexspi_nor_mem_enable(void)
{
    s_flexspiNorContext.isConfigured = true;
}

//! @brief Convert flexspi amba address to physical address
static uint32_t flexspi_get_phy_address(uint32_t mapAddr)
{
    return mapAddr - g_memoryMap[s_flexspiMmapIndexes[s_flexspiNorContext.instance]].startAddress;
}

//! @brief Get Property from flexspi driver
status_t flexspi_nor_get_property(uint32_t whichProperty, uint32_t *value)
{
    if (value == NULL)
    {
        return kStatus_InvalidArgument;
    }

    switch (whichProperty)
    {
        case kExternalMemoryPropertyTag_InitStatus:
            *value = is_flexspi_nor_configured() ? kStatus_Success : kStatusMemoryNotConfigured;
            break;

        case kExternalMemoryPropertyTag_StartAddress:
            *value = g_memoryMap[s_flexspiMmapIndexes[s_flexspiNorContext.instance]].startAddress;
            break;

        case kExternalMemoryPropertyTag_MemorySizeInKbytes:
            {
                *value = FLASH_SIZE_KB;
            }
            break;

        case kExternalMemoryPropertyTag_PageSize:
            *value = FLASH_PAGE_SIZE;
            break;

        case kExternalMemoryPropertyTag_SectorSize:
            *value = SECTOR_SIZE;
            break;

        default: // catch inputs that are not recognized
            return kStatus_InvalidArgument;
    }

    return kStatus_Success;
}

// See flexspi_nor_memory.h for documentation on this function.
status_t flexspi_nor_mem_init(void)
{
    /*
     * NOTE: The SPI NOR instruction set may be switched during Boot
     *       So, here bootloader doesn't try to read the configuration
     *       block based on Fuse settings.
     */
    return kStatus_Fail;
}

// See flexspi_nor_memory.h for documentation on this function.
status_t flexspi_nor_mem_read(uint32_t address, uint32_t length, uint8_t *buffer)
{
    if (!is_flexspi_nor_configured())
    {
        return kStatusMemoryNotConfigured;
    }
    return normal_mem_read(address, length, buffer);
}

#if BL_FEATURE_MEM_WRITE_ENABLE
// See flexspi_nor_memory.h for documentation on this function.
status_t flexspi_nor_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_write().
    assert(length);
    assert(buffer);

    // Check if flexspi is configured
    if (!is_flexspi_nor_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    status_t status;
    uint32_t writeLength;

    while (length)
    {
        // If buffer is invalid, it means it is a new write operation.
        if (!s_flexspiNorContext.isAddingToBuffer)
        {
            // If address is page aligned, it means it is a valid start address, else return an error status
            if (address & (FLASH_PAGE_SIZE - 1))
            {
                return kStatus_Fail;
            }

            // Start buffering data
            s_flexspiNorContext.isAddingToBuffer = true;
            s_flexspiNorContext.offset = 0;
            s_flexspiNorContext.writeAddress = address;
        }
        else
        {
            // In this case, it means recent two writes are not continuous, should flush last cached data into memory,
            // then switch to processsing of this write operation.
            if ((s_flexspiNorContext.offset + s_flexspiNorContext.writeAddress) != address)
            {
                // flush cached data into target memory,
                status = flexspi_nor_mem_flush();
                if (status != kStatus_Success)
                {
                    return status;
                }
                // Start processing this write
                continue;
            }
            // Otherwise, it means recent two writes are continuous, continue to buffer data until whole page gets
            // buffered.
        }

        if (s_flexspiNorContext.offset + length < FLASH_PAGE_SIZE)
        {
            writeLength = length;
        }
        else
        {
            writeLength = FLASH_PAGE_SIZE - s_flexspiNorContext.offset;
        }

        // Copy data to internal buffer
        memcpy(&s_flexspiNorContext.buffer[s_flexspiNorContext.offset], buffer, writeLength);
        s_flexspiNorContext.offset += writeLength;
        address += writeLength;
        buffer += writeLength;
        length -= writeLength;

        assert(s_flexspiNorContext.offset <= FLASH_PAGE_SIZE);
        // If the buffer is full, it is time to flush cached data to target memory.
        if (s_flexspiNorContext.offset == FLASH_PAGE_SIZE)
        {
            status = flexspi_nor_mem_flush();
            if (status != kStatus_Success)
            {
                return status;
            }
        }
    }

    return kStatus_Success;
}

bool is_flexspi_nor_mem_erased(uint32_t start, uint32_t length)
{
    bool is_erased = true;
    {
        is_erased = mem_is_erased(start, length);
    }

    return is_erased;
}

bool flexspi_nor_memory_check(uint32_t start, uint8_t *data_to_check, uint32_t length)
{
    bool data_match = true;
    {
        data_match = (memcmp((void *)start, data_to_check, length) == 0) ? true : false;
    }

    return data_match;
}

// See flexspi_nor_memory.h for documentation on this function.
status_t flexspi_nor_mem_flush(void)
{
    status_t status = kStatus_Success;

    if (s_flexspiNorContext.isAddingToBuffer)
    {
        s_flexspiNorContext.isAddingToBuffer = false;
        // Fill unused region with 0xFFs.
        if (s_flexspiNorContext.offset != FLASH_PAGE_SIZE)
        {
            memset(&s_flexspiNorContext.buffer[s_flexspiNorContext.offset], 0xFF,
                   FLASH_PAGE_SIZE - s_flexspiNorContext.offset);
        }

#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
        if (!is_flexspi_nor_mem_erased(s_flexspiNorContext.writeAddress, FLASH_PAGE_SIZE))
        {
            return kStatusMemoryCumulativeWrite;
        }
#endif // BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
        // Write whole page to spi flash
        uint32_t regPrimask = DisableGlobalIRQ();
        status = sbl_flash_write(flexspi_get_phy_address(s_flexspiNorContext.writeAddress), (uint32_t *)s_flexspiNorContext.buffer, FLASH_PAGE_SIZE);
        EnableGlobalIRQ(regPrimask);

        // Clear related states.
        s_flexspiNorContext.isAddingToBuffer = false;
        s_flexspiNorContext.offset = 0;
        if (status != kStatus_Success)
        {
            return status;
        }
#if __CORTEX_M == 0x07
        if (SCB->CCR & SCB_CCR_DC_Msk)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)s_flexspiNorContext.writeAddress,
                                         FLASH_PAGE_SIZE);
        }
#endif
        // Verify whether the data has been programmed to Serial NOR flash successfully.
        if (!flexspi_nor_memory_check(s_flexspiNorContext.writeAddress, s_flexspiNorContext.buffer,
                                      FLASH_PAGE_SIZE))
        {
            return kStatus_Fail;
        }
    }
    return status;
}

// See flexspi_nor_memory.h for documentation on this function.
status_t flexspi_nor_mem_erase(uint32_t address, uint32_t length)
{
    assert(length);

    // Check if QuadSPI is configured
    if (!is_flexspi_nor_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    uint32_t sectorSize = SECTOR_SIZE;
    uint32_t alignedAddress = ALIGN_DOWN(address, sectorSize);
    uint32_t alignedLength = ALIGN_UP(address + length, sectorSize) - alignedAddress;

    status_t status = flexspi_nor_memory_erase(alignedAddress, alignedLength);
    if (status != kStatus_Success)
    {
        return status;
    }

#if __CORTEX_M == 0x07
    if (SCB->CCR & SCB_CCR_DC_Msk)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)alignedAddress, sectorSize);
    }
#endif
#if FLEXSPI_NOR_ERASE_VERIFY
    if (!is_flexspi_nor_mem_erased(alignedAddress, alignedLength))
    {
        return kStatus_Fail;
    }
#endif // #if FLEXSPI_NOR_ERASE_VERIFY

    return kStatus_Success;
}

// See flexspi_nor_memory.h for documentation on this function.
status_t flexspi_nor_mem_erase_all(void)
{
    return kStatus_Fail;
}

#endif //

static status_t flexspi_nor_memory_erase(uint32_t address, uint32_t length)
{
    lock_acquire();
    status_t status = sbl_flash_erase(flexspi_get_phy_address(address), length);
    lock_release();

    return status;
}

#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
