/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "memory/memory.h"
#include "qspi_memory.h"
#include "normal_memory.h"
#include "qspi/qspi.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/fsl_assert.h"
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum
{
    //! @brief Max page size value used to create buffer
    kQspiMemoryBuffer_MaxPageSize = 1024U,
    //! @brief Default top address of qspi memory
    kQspiMemoryMaxSize = 0x08000000UL,
    //! @brief Default top address of QuadSPI alias memory
    kQspiAliasAreaMaxSize = 0x04000000UL,
};

//! @brief QSPI memory feature inforamation
//!
//! An instance is maintained in this file and is used to keep properties of qspi memory, which is
//! used in other functions.
typedef struct _qspi_mem_features
{
    uint32_t pageSize;         //!< Page size of the spi memory
    uint32_t sectorSize;       //!< Sectore size of spi memory
    uint32_t qspiStartAddress; //!< Start address of qspi memory
    uint32_t qspiEndAddress;   //!< End address of qspi memory
} qspi_mem_feature_t;

//! @brief QSPI memory context
//!
//! An instance is maintained in this file, will is used to keep key information for write and flush
//! operatations.
typedef struct _qspi_mem_context
{
    bool isAddingToBuffer;                         //!< State used of determine whether it is the first write at
                                                   //!< Start address of one page
    uint32_t writeAddress;                         //!< This address is used to record the address which is used
                                                   //!< to write the whole page into qspi memory
    uint32_t offset;                               //!< A variable which is used to indicate if the buffer is
                                                   //!< full.
    uint32_t nextStartAddress;                     //!< A variable is used to indicate if recent two writes
                                                   //!< are continuous
    uint8_t buffer[kQspiMemoryBuffer_MaxPageSize]; //!< A buffer which is used to buffer a full
                                                   //!< page of data
} qspi_mem_context_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

//! @brief verify whether the data programmed are equal to given data
static status_t qspi_mem_verify_program(uint32_t start, uint32_t lengthInBytes, const uint8_t *expectedData);
//! @brief verify whether given qspi memory range is erased.
static status_t qspi_mem_verify_erase(uint32_t start, uint32_t lengthInBytes);
// !@brief verify whether the qspi memory is fully erased.
static status_t qspi_mem_verify_erase_all(void);

//! @brief Intialize QuadSPI memory interface
static status_t qspi_memory_init(void);
//! @brief Write function for QuadSPI memory interface
static status_t qspi_memory_write(uint32_t address, uint32_t length, const uint8_t *buffer);
//! @brief Fill QuadSPI memory interface
static status_t qspi_memory_fill(uint32_t address, uint32_t length, uint32_t pattern);
//! @brief Erase function QuadSPI memory interface
static status_t qspi_memory_erase(uint32_t address, uint32_t length);

#if BL_FEATURE_QSPI_ALIAS_AREA
//! @brief Intialize QuadSPI Alias Area Interface
static status_t qspi_alias_area_init(void);
//! @brief Write function for QuadSPI Alias Area interface
static status_t qspi_alias_area_write(uint32_t address, uint32_t length, const uint8_t *buffer);
//! @brief Fill function for QuadSPI Alias Area interface
static status_t qspi_alias_area_fill(uint32_t address, uint32_t length, uint32_t pattern);
//! @brief Erase function for QuadSPI Alias Area interface
static status_t qspi_alias_area_erase(uint32_t address, uint32_t length);
// Get identical address for QuadSPI Alias address
static uint32_t qspi_get_map_address(uint32_t srcAliasAddress);
#endif // #if BL_FEATURE_QSPI_ALIAS_AREA

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Interface to qspi memory operations.
const memory_region_interface_t g_qspiMemoryInterface = {
    .init = &qspi_memory_init,
    .read = &qspi_mem_read,
    .write = &qspi_memory_write,
#if !BL_MIN_PROFILE
    .fill = &qspi_memory_fill,
#endif // !BL_MIN_PROFILE
    .flush = qspi_mem_flush,
    .erase = qspi_memory_erase,
};

#if BL_FEATURE_QSPI_ALIAS_AREA
//! @brief Interface to qspi alias area operations
const memory_region_interface_t g_qspiAliasAreaInterface = {
    .init = &qspi_alias_area_init,
    .read = &qspi_mem_read,
    .write = &qspi_alias_area_write,
#if !BL_MIN_PROFILE
    .fill = &qspi_alias_area_fill,
#endif // !BL_MIN_PROFILE
    .flush = qspi_mem_flush,
    .erase = qspi_alias_area_erase,
};
#endif

qspi_mem_feature_t s_qspi_mem_feature = { 0 };
qspi_mem_context_t s_qspi_mem_context = { 0 };

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See qspi_memory.h for documentation on this function.
status_t qspi_mem_init(uint32_t memoryId)
{
    uint32_t propertyValue;

    memory_map_entry_t *map;
#if BL_FEATURE_QSPI_ALIAS_AREA
    if (memoryId == kQspiMemoryId_AliasArea)
    {
        map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexQspiAliasArea];
    }
    else
#endif
    {
        map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexQspiMemory];
    }

    if (!is_quadspi_configured())
    {
        map->memoryProperty = kMemoryNotExecutable | kMemoryType_FLASH;
        return kStatus_QspiNotConfigured;
    }

    // Get total size of QSPI flash.
    status_t status = quadspi_get_property(kQspiFlashProperty_TotalFlashSize, &propertyValue);
    if (status != kStatus_Success)
    {
        return status;
    }

    if (propertyValue == 0)
    {
        // It means there are no spi flash connected to qspi interface
        // The qspi memory interface should be disabled to prevent bootloader from accidentally accessing
        // qspi memory region.

        // Update address range of flash
        map->endAddress = map->startAddress;

        return kStatus_QspiFlashSizeError;
    }
    else
    {
#if BL_FEATURE_QSPI_ALIAS_AREA
        if (memoryId == kQspiMemoryId_AliasArea)
        {
            if (propertyValue > kQspiAliasAreaMaxSize)
            {
                propertyValue = kQspiAliasAreaMaxSize;
            }
        }
        else
#endif // #if BL_FEATURE_QSPI_ALIAS_AREA
        {
            if (propertyValue > kQspiMemoryMaxSize)
            {
                propertyValue = kQspiMemoryMaxSize;
            }
        }
        map->endAddress = map->startAddress + propertyValue - 1;
        map->memoryProperty = kMemoryIsExecutable | kMemoryType_FLASH;

        // Only use the QuadSPI memory to initialize the s_qspi_mem_feature.
        if (memoryId == kQspiMemoryId_QuadSpiMemory)
        {
            s_qspi_mem_feature.qspiStartAddress = map->startAddress;
            s_qspi_mem_feature.qspiEndAddress = map->endAddress;

            // Get page size of QSPI flash
            status = quadspi_get_property(kQspiFlashProperty_PageSize, &propertyValue);
            assert(status == kStatus_Success);
            s_qspi_mem_feature.pageSize = propertyValue;

            // Get sector size, which is used in qspi_mem_erase
            status = quadspi_get_property(kQspiFlashProperty_SectorSize, &propertyValue);
            assert(status == kStatus_Success);
            s_qspi_mem_feature.sectorSize = propertyValue;
        }

        return kStatus_Success;
    }
}

// See qspi_memory.h for documentation on this function.
status_t qspi_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
    // Check if QuadSPI is configured
    if (!is_quadspi_configured())
    {
        return kStatus_QspiNotConfigured;
    }

    return normal_mem_read(address, length, buffer);
}

// See qspi_memory.h for documentation on this function.
// Description: For qspi_mem_write, the minimum programming unit is one page for various spi serial flash chips.
//              While bootloader command might not be able to send the whole page with only one data packet. To overcome
//              this
//              defeat, qspi memory interface maintains an internal buffer which is used to buffer one page data passed
//              from
//              host and flush buffered data into spi flash when the buffer is full.
status_t qspi_mem_write(uint32_t memoryId, uint32_t address, uint32_t length, const uint8_t *buffer)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_write().
    assert(length);
    assert(buffer);

#if BL_FEATURE_QSPI_ALIAS_AREA
    if (memoryId == kQspiMemoryId_AliasArea)
    {
        uint32_t mapAddress = qspi_get_map_address(address);
        address = mapAddress;
    }
#endif // #if BL_FEATURE_QSPI_ALIAS_AREA

    // Check if QuadSPI is configured
    if (!is_quadspi_configured())
    {
        return kStatus_QspiNotConfigured;
    }

    status_t status = kStatus_Success;

    while (length)
    {
        // If no data in buffer, it means it is a new write operation.
        if (!s_qspi_mem_context.isAddingToBuffer)
        {
            // If address is page aligned, it means it is a valid start address, else return an error status
            if (address & (s_qspi_mem_feature.pageSize - 1))
            {
                return kStatus_QspiFlashAlignmentError;
            }

            // Start buffering data
            s_qspi_mem_context.isAddingToBuffer = true;
            s_qspi_mem_context.offset = 0;
            s_qspi_mem_context.writeAddress = address;
        }
        else
        {
            // In this case, it means recent two writes are not continuous, should flush last cached data into memory,
            // then switch to processsing of this write operation.
            if (s_qspi_mem_context.nextStartAddress != address)
            {
                // flush cached data into target memory,
                status = qspi_mem_flush();
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

        uint32_t writeLength;

        if (s_qspi_mem_context.offset + length < s_qspi_mem_feature.pageSize)
        {
            writeLength = length;
            s_qspi_mem_context.nextStartAddress = address + length;
        }
        else
        {
            writeLength = s_qspi_mem_feature.pageSize - s_qspi_mem_context.offset;
        }

        // Copy data to internal buffer
        memcpy(&s_qspi_mem_context.buffer[s_qspi_mem_context.offset], buffer, writeLength);
        s_qspi_mem_context.offset += writeLength;
        address += writeLength;
        buffer += writeLength;
        length -= writeLength;

        assert(s_qspi_mem_context.offset <= s_qspi_mem_feature.pageSize);
        // If the buffer is full, it is time to flush cached data to target memory.
        if (s_qspi_mem_context.offset == s_qspi_mem_feature.pageSize)
        {
            status = qspi_mem_flush();

            if (status != kStatus_Success)
            {
                return status;
            }
        }
    }

    return kStatus_Success;
}

// See qspi_memory.h for documentation on this function.
status_t qspi_mem_fill(uint32_t memoryId, uint32_t address, uint32_t length, uint32_t pattern)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_fill().
    assert(length);

    // Check if QuadSPI is configured
    if (!is_quadspi_configured())
    {
        return kStatus_QspiNotConfigured;
    }

#if BL_FEATURE_QSPI_ALIAS_AREA
    if (memoryId == kQspiMemoryId_AliasArea)
    {
        uint32_t mapAddress = qspi_get_map_address(address);
        address = mapAddress;
    }
#endif // #if BL_FEATURE_QSPI_ALIAS_AREA

    uint32_t alignedLength = ALIGN_DOWN(length, 4U);
    uint32_t leftLengthInBytes = length & 3;

    status_t status;

    while (alignedLength)
    {
        status = qspi_memory_write(address, 4, (const uint8_t *)&pattern);
        if (status != kStatus_Success)
        {
            return status;
        }

        address += 4;
        alignedLength -= 4;
    }

    if (leftLengthInBytes)
    {
        status = qspi_memory_write(address, leftLengthInBytes, (const uint8_t *)&pattern);
        if (status != kStatus_Success)
        {
            return status;
        }
    }

    status = qspi_mem_flush();
    if (status != kStatus_Success)
    {
        return kStatus_QspiFlashCommandFailure;
    }

    return status;
}

// See qspi_memory.h for documentation on this function.
status_t qspi_mem_flush(void)
{
    status_t status = kStatus_Success;

    if (s_qspi_mem_context.isAddingToBuffer)
    {
        // Fill unused region with oxFFs.
        assert(s_qspi_mem_context.offset <= s_qspi_mem_feature.pageSize);
        if (s_qspi_mem_context.offset != s_qspi_mem_feature.pageSize)
        {
            memset(&s_qspi_mem_context.buffer[s_qspi_mem_context.offset], 0xFF,
                   s_qspi_mem_feature.pageSize - s_qspi_mem_context.offset);
        }

#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
        if (!mem_is_erased(s_qspi_mem_context.writeAddress, s_qspi_mem_feature.pageSize))
        {
            return kStatusMemoryCumulativeWrite;
        }
#endif // BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE

        // Write whole page to spi flash
        status = quadspi_page_program(s_qspi_mem_context.writeAddress, (uint32_t *)s_qspi_mem_context.buffer,
                                      s_qspi_mem_feature.pageSize);
        // Clear related states.
        s_qspi_mem_context.isAddingToBuffer = false;
        s_qspi_mem_context.offset = 0;

        if (status != kStatus_Success)
        {
            return status;
        }

        // Verify wether the data has been programmed to spi flash successfully.
        status = qspi_mem_verify_program(s_qspi_mem_context.writeAddress, s_qspi_mem_feature.pageSize,
                                         s_qspi_mem_context.buffer);
        if (status != kStatus_Success)
        {
            return kStatus_QspiFlashCommandFailure;
        }
    }
    return status;
}

// See qspi_memory.h for documentation on this function.
status_t qspi_mem_erase(uint32_t memoryId, uint32_t address, uint32_t length)
{
    assert(length);

    // Check if QuadSPI is configured
    if (!is_quadspi_configured())
    {
        return kStatus_QspiNotConfigured;
    }

#if BL_FEATURE_QSPI_ALIAS_AREA
    if (memoryId == kQspiMemoryId_AliasArea)
    {
        uint32_t mapAddress = qspi_get_map_address(address);
        address = mapAddress;
    }
#endif // BL_FEATURE_QSPI_ALIAS_AREA

    uint32_t sectorSize = s_qspi_mem_feature.sectorSize;
    uint32_t alignedAddress = ALIGN_DOWN(address, sectorSize);
    uint32_t alignedLength = ALIGN_UP(length, sectorSize);

    while (alignedLength)
    {
        status_t status = quadspi_erase_sector(alignedAddress);
        if (status != kStatus_Success)
        {
            return status;
        }
        status = qspi_mem_verify_erase(alignedAddress, sectorSize);
        if (status != kStatus_Success)
        {
            return kStatus_QspiFlashCommandFailure;
        }

        alignedLength -= sectorSize;
        alignedAddress += sectorSize;
    }

    return kStatus_Success;
}

// See memory.h for documentation on this function.
status_t qspi_mem_erase_all(void)
{
    // Check if QuadSPI is configured
    if (!is_quadspi_configured())
    {
        return kStatus_QspiNotConfigured;
    }

    status_t status = quadspi_erase_all();
    if (status != kStatus_Success)
    {
        return status;
    }
    // verify erase all command
    status = qspi_mem_verify_erase_all();
    if (status != kStatus_Success)
    {
        return kStatus_QspiFlashCommandFailure;
    }

    return kStatus_Success;
}

status_t qspi_mem_verify_program(uint32_t start, uint32_t lengthInBytes, const uint8_t *expectedData)
{
    assert(lengthInBytes);
    assert(expectedData);

    if (!memcmp((void *)start, expectedData, lengthInBytes))
    {
        return kStatus_Success;
    }

    return kStatus_Fail;
}

status_t qspi_mem_verify_erase(uint32_t start, uint32_t lengthInBytes)
{
    assert(lengthInBytes);

    const uint32_t expectedData = 0xFFFFFFFFUL;

    uint32_t alignedStart = ALIGN_DOWN(start, 4U);
    uint32_t alignedLength = ALIGN_UP(lengthInBytes, 4U);
    uint32_t *pLongwordStart = (uint32_t *)alignedStart;

    while (alignedLength)
    {
        if (*pLongwordStart != expectedData)
        {
            return kStatus_Fail;
        }

        ++pLongwordStart;
        alignedLength -= 4;
    }

    return kStatus_Success;
}

status_t qspi_mem_verify_erase_all(void)
{
    uint32_t startAddress = s_qspi_mem_feature.qspiStartAddress;
    uint32_t lengthInBytes = s_qspi_mem_feature.qspiEndAddress - startAddress + 1;

    status_t status = qspi_mem_verify_erase(startAddress, lengthInBytes);

    return status;
}

static status_t qspi_memory_init(void)
{
    return qspi_mem_init(kQspiMemoryId_QuadSpiMemory);
}

static status_t qspi_memory_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    return qspi_mem_write(kQspiMemoryId_QuadSpiMemory, address, length, buffer);
}

static status_t qspi_memory_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    return qspi_mem_fill(kQspiMemoryId_QuadSpiMemory, address, length, pattern);
}

static status_t qspi_memory_erase(uint32_t address, uint32_t length)
{
    return qspi_mem_erase(kQspiMemoryId_QuadSpiMemory, address, length);
}

#if BL_FEATURE_QSPI_ALIAS_AREA
static status_t qspi_alias_area_init(void)
{
    return qspi_mem_init(kQspiMemoryId_AliasArea);
}
static status_t qspi_alias_area_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    return qspi_mem_write(kQspiMemoryId_AliasArea, address, length, buffer);
}
static status_t qspi_alias_area_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    return qspi_mem_fill(kQspiMemoryId_AliasArea, address, length, pattern);
}
static status_t qspi_alias_area_erase(uint32_t address, uint32_t length)
{
    return qspi_mem_erase(kQspiMemoryId_AliasArea, address, length);
}

uint32_t qspi_get_map_address(uint32_t srcAliasAddress)
{
    memory_map_entry_t *map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexQspiAliasArea];
    uint32_t offset = srcAliasAddress - map->startAddress;
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexQspiMemory];

    uint32_t mapAddress = map->startAddress + offset;

    return mapAddress;
}
#endif // BL_FEATURE_QSPI_ALIAS_AREA

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
