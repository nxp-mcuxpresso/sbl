/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "memory/memory.h"
#include "flash_c040hd_memory.h"
#include "flash_ffr_memory.h"
#include "normal_memory.h"
#include "fsl_iap.h"
#include "fsl_iap_ffr.h"
#include "bootloader/bl_context.h"
#include "bootloader/bootloader.h"
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
#define ffr_lock_release() (void)sizeof(0)
#define ffr_lock_acquire() (void)sizeof(0)
#endif // BL_TARGET_FLASH

//! @brief FFR Memory constants.
enum _ffr_memory_constants
{
    kFfrMemory_ErasedValue = ~0
};

//! @brief FFR buffer program memory context
//!
//! An instance is maintained in this file, will is used to keep key information for write and flush
//! operatations.
typedef struct _ffr_spec_buffer_program_info
{
    uint32_t startAddress; //!< This address is used to record the address which is used
                           //!< to write the whole buffer into FFR memory
    uint32_t storedBytes;  //!< A variable which is used to indicate if the buffer is full.
#if BL_FETUARE_USE_SHARED_MEMORY_INTERFACE_BUFFER
    uint8_t *buffer;
#else
    uint8_t buffer[FLASH_FFR_MAX_PAGE_SIZE * 3]; //!< A buffer which is used to buffer a full package of data
#endif
} ffr_spec_buffer_program_info_t;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Interface to flash FFR memory operations.
const memory_region_interface_t g_ffrMemoryInterface = {.init = &ffr_mem_init,
                                                        .read = &ffr_mem_read,
                                                        //.write = &ffr_mem_write,
                                                        .fill = NULL,
                                                        //.flush = &ffr_mem_flush,
                                                        //.erase = &ffr_mem_erase,
                                                        .config = NULL,
                                                        .erase_all = NULL };

#if BL_TARGET_FLASH
static uint32_t s_regPrimask = 0U;
#endif

//static ffr_spec_buffer_program_info_t s_ffr_spec_buf_program_info = {0};

#if BL_FETUARE_USE_SHARED_MEMORY_INTERFACE_BUFFER
static const uint32_t k_programBufferSize = FLASH_FFR_MAX_PAGE_SIZE * 3;
#else
//static const uint32_t k_programBufferSize = sizeof(s_ffr_spec_buf_program_info.buffer);
#endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if BL_TARGET_FLASH
static void ffr_lock_acquire(void)
{
    // Ensure that the program operation cannots be interrupted.
    s_regPrimask = __get_PRIMASK();
    __disable_irq();
}

static void ffr_lock_release(void)
{
    // Release lock after the write operation completes.
    __set_PRIMASK(s_regPrimask);
}
#endif

// See flash_ffr_memory.h for documentation on this function.
status_t ffr_mem_init(void)
{
    // Update address range of flash FFR
    memory_map_entry_t *map;
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexFFR];

    flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[0];

    map->startAddress = flashConfig->ffrConfig.ffrBlockBase;
    // Note: NMPA will be programmed by Factory, customer doesn't need to program it.
    // uint32_t writableFfrSize = flashConfig->ffrConfig.ffrTotalSize - kFfrPageNum_NMPA *
    // flashConfig->ffrConfig.ffrPageSize;
    map->endAddress = map->startAddress + flashConfig->ffrConfig.ffrTotalSize - 1;

    //s_ffr_spec_buf_program_info.storedBytes = 0;

#if BL_FETUARE_USE_SHARED_MEMORY_INTERFACE_BUFFER
    //s_ffr_spec_buf_program_info.buffer = g_memoryBuffer;
#endif

    return kStatus_Success;
}

// See flash_ffr_memory.h for documentation on this function.
status_t ffr_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
    status_t status = kStatus_Fail;

    flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[0];
    uint32_t ffrRegionStart = flashConfig->ffrConfig.ffrBlockBase;
    uint32_t specRegionEnd = ffrRegionStart + kFfrPageNum_SpecArea * flashConfig->PFlashPageSize - 1;
    uint32_t cfpaStart = ffrRegionStart + kFfrPageOffset_CFPA * flashConfig->ffrConfig.ffrPageSize;
    uint32_t cfpaCfgStart = ffrRegionStart + kFfrPageOffset_CFPA_Cfg * flashConfig->ffrConfig.ffrPageSize;
    uint32_t cmpaCfgStart = ffrRegionStart + kFfrPageOffset_CMPA_Cfg * flashConfig->ffrConfig.ffrPageSize;

    if ((address > specRegionEnd) || (address + length > specRegionEnd + 1))
    {
        return kStatus_FLASH_NmpaAccessNotAllowed;
    }
    else if ((address >= cfpaCfgStart) && (address + length <= cmpaCfgStart) &&
             (length <= flashConfig->ffrConfig.ffrPageSize))
    {
        status = FFR_GetCustomerInfieldData(flashConfig, buffer, 0, length);
    }
    else if ((address >= cmpaCfgStart) && (address + length <= cmpaCfgStart + flashConfig->ffrConfig.ffrPageSize))
    {
        status = FFR_GetCustomerData(flashConfig, buffer, address - cmpaCfgStart, length);
    }
    else if (((address >= cfpaStart) && (address + length <= cfpaCfgStart) &&
              (length <= flashConfig->ffrConfig.ffrPageSize)) ||
             (address >= cmpaCfgStart + flashConfig->ffrConfig.ffrPageSize))
    {
        status = flash_mem_read(address, length, buffer);
    }
    else
    {
        status = kStatus_FLASH_AlignmentError;
    }

    return status;
}

/*
// See flash_ffr_memory.h for documentation on this function.
status_t ffr_mem_erase(uint32_t address, uint32_t length)
{
    flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[0];
    uint32_t ffrRegionStart = flashConfig->ffrConfig.ffrBlockBase;
    uint32_t specRegionEnd = ffrRegionStart + kFfrPageNum_SpecArea * flashConfig->PFlashPageSize - 1;
    uint32_t cmpaCfgStart = ffrRegionStart + kFfrPageOffset_CMPA_Cfg * flashConfig->ffrConfig.ffrPageSize;
    if ((address > specRegionEnd) || (address + length > specRegionEnd + 1))
    {
        return kStatus_FLASH_NmpaAccessNotAllowed;
    }
    else if ((address >= cmpaCfgStart) || (address + length >= cmpaCfgStart + 1))
    {
        return kStatus_FLASH_CmpaCfgDirectEraseNotAllowed;
    }

    return flash_mem_erase(address, length);
}

// See flash_ffr_memory.h for documentation on this function.
status_t ffr_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_write().
    assert(length);
    assert(buffer);

    status_t status = kStatus_Success;
    flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[0];
    uint32_t ffrRegionStart = flashConfig->ffrConfig.ffrBlockBase;
    uint32_t specRegionEnd = ffrRegionStart + kFfrPageNum_SpecArea * flashConfig->PFlashPageSize - 1;
    if (address > specRegionEnd)
    {
        return kStatus_FLASH_NmpaAccessNotAllowed;
        // return flash_mem_write(address, length, buffer);
    }

    while (length)
    {
        // Set start address when storing first byte into spec program buffer
        if ((!s_ffr_spec_buf_program_info.storedBytes) && (!s_ffr_spec_buf_program_info.startAddress))
        {
            s_ffr_spec_buf_program_info.startAddress = address;
        }
        else
        {
            // Report error when meet discontinuous address
            if ((s_ffr_spec_buf_program_info.startAddress + s_ffr_spec_buf_program_info.storedBytes) != address)
            {
                // flush cached data into target memory,
                status = ffr_mem_flush();
                if (status != kStatus_Success)
                {
                    return status;
                }
                continue;
            }
        }

        uint32_t storeBytes;
        // Check to see if section program buffer will be filled with current data packet
        if ((s_ffr_spec_buf_program_info.storedBytes + length) <= k_programBufferSize)
        {
            storeBytes = length;
        }
        else
        {
            return kStatus_FLASH_FfrRegionWriteBroken;
        }

        // Copy data to spec program buffer
        memcpy(&s_ffr_spec_buf_program_info.buffer[s_ffr_spec_buf_program_info.storedBytes], buffer, storeBytes);
        s_ffr_spec_buf_program_info.storedBytes += storeBytes;
        buffer += storeBytes;
        address += storeBytes;
        length -= storeBytes;
    }

    return kStatus_Success;
}

// See flash_ffr_memory.h for documentation on this function.
status_t ffr_mem_flush(void)
{
    status_t status = kStatus_Success;

    if (s_ffr_spec_buf_program_info.storedBytes)
    {
        flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[0];
        uint32_t ffrRegionStart = flashConfig->ffrConfig.ffrBlockBase;
        uint32_t cfpaStart = ffrRegionStart;
        uint32_t cmpaStart = cfpaStart + kFfrPageNum_CFPA * flashConfig->PFlashPageSize;
        uint32_t kaStart = cmpaStart + kFfrPageNum_CMPA_Cfg * flashConfig->PFlashPageSize;

        uint32_t address = s_ffr_spec_buf_program_info.startAddress;
        uint32_t length = s_ffr_spec_buf_program_info.storedBytes;

        // Clear related states no matter following operations are executed successfully or not.
        s_ffr_spec_buf_program_info.startAddress = 0;
        s_ffr_spec_buf_program_info.storedBytes = 0;

        ffr_lock_acquire();
        // Write cached data to FFR spec page
        // Program Customer Filed Programmable area
        if ((address + length <= cmpaStart) && (length <= flashConfig->PFlashPageSize))
        {
            status = FFR_InfieldPageWrite(flashConfig, s_ffr_spec_buf_program_info.buffer, length);
        }
        // Program Customer Filed Configuration area
        else if ((address == cmpaStart) && (length == flashConfig->PFlashPageSize))
        {
            status = FFR_ProcessCmpaCfgPageUpdate(flashConfig, kFfrCmpaProgProcess_Pre);
            if (status != kStatus_FLASH_Success)
            {
                return status;
            }
            status = FFR_CustFactoryPageWrite(flashConfig, s_ffr_spec_buf_program_info.buffer, false);
        }
        // Program Keys area
        else if ((address >= kaStart) && (length >= kFfrBlockSize_ActivationCode))
        {
            property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;
            if (propertyStore->ffrKeystoreUpdateOpt == kFfrKeystoreUpdateOpt_WriteMemory)
            {
                status = FFR_KeystoreWrite(flashConfig, (ffr_key_store_t *)s_ffr_spec_buf_program_info.buffer);
            }
            else
            {
                status = kStatus_FLASH_InvalidArgument;
            }
        }
        else
        {
            status = kStatus_FLASH_InvalidArgument;
        }
        ffr_lock_release();
    }

    return status;
}
*/

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
