/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__CONTEXT_H__)
#define __CONTEXT_H__

#include "bootloader_common.h"
#include "bootloader/bl_peripheral.h"
#include "memory/memory.h"
#include "packet/command_packet.h"
#include "bootloader/bl_command.h"
#include "property/property.h"

#if !defined(BOOTLOADER_HOST)
#if BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_FLASH_TYPE_KINETIS_C90TFS_FLASH
#include "flash_c90tfs/api_flash.h"
#elif BL_FLASH_TYPE_LPC_C040HD_FLASH
#include "flash_c040hd/api_flash.h"
#elif BL_FLASH_TYPE_LPC_IP2113_FLASH
#include "flash_ip2113/api_flash.h"
#endif
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#endif // #if !defined(BOOTLOADER_HOST)

//! @addtogroup context
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#if defined(BOOTLOADER_HOST)
// Provide stub definitions for flash driver types for the host.
#define FLASH_CONFIG_T uint32_t
#define FLASH_INTERFACE_T uint32_t
#else // !defined(BOOTLOADER_HOST)
#if !defined(FLASH_CONFIG_T)
#define FLASH_CONFIG_T void
#endif
#if !defined(FLASH_INTERFACE_T)
#define FLASH_INTERFACE_T void
#endif
#endif // #if defined(BOOTLOADER_HOST)

//! @brief Structure of bootloader global context.
typedef struct _bootloaderContext
{
    //! @name API tree
    //@{
    const memory_interface_t *memoryInterface; //!< Abstract interface to memory operations.
    const memory_map_entry_t *memoryMap;       //!< Memory map used by abstract memory interface.
#if BL_FEATURE_EXPAND_MEMORY
    const external_memory_map_entry_t *externalMemoryMap; //!< Memory map used by external memory devices.
#endif                                                    // BL_FEATURE_EXPAND_MEMORY
    const property_interface_t *propertyInterface;        //!< Interface to property store.
    const command_interface_t *commandInterface;          //!< Interface to command processor operations.
    const FLASH_INTERFACE_T *flashDriverInterface;        //!< Internal Flash driver interface.
    const peripheral_descriptor_t *allPeripherals;        //!< Array of all peripherals.
    //@}

    //! @name Runtime state
    //@{
    const peripheral_descriptor_t *activePeripheral; //!< The currently active peripheral.
    FLASH_CONFIG_T *allFlashState;                   //!< Internal Flash driver instance.
    //@}
} bootloader_context_t;

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

extern bootloader_context_t g_bootloaderContext;

//! @}

#endif // __CONTEXT_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
