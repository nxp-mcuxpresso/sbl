/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "memory/memory.h"

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Memory map for RT500.
//!
//! This map is not const because it is updated at runtime with the actual sizes of
//! flash and RAM for the chip we're running on.
//! @note See memory.h for index requirements.
memory_map_entry_t g_memoryMap[] = {
    // SRAM0 Secure Alias address (size: 5MB)
    { 0x10000000u, 0x104fffffu, kMemoryIsExecutable | kMemoryType_RAM | kMemoryAliasAddr, kMemoryInternal,
      &g_normalMemoryInterface },

    // SRAM0 (size: 5MB)
    { 0x00000000u, 0x004fffffu, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },

    // System RAM (size: 5MB)
    { 0x20000000u, 0x204fffffu, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },

    // System RAM Secure Alias address (size: 5MB)
    { 0x30000000u, 0x304fffffu, kMemoryIsExecutable | kMemoryType_RAM | kMemoryAliasAddr, kMemoryInternal,
      &g_normalMemoryInterface },

#if BL_FEATURE_FLEXSPI_NOR_MODULE
    // FLEXSPI memory (Size will be updated later during interface initialization)
    { 0x08000000u, 0x0fffffffu, kMemoryNotExecutable | kMemoryType_FLASH | kMemorySkipInitError, kMemoryFlexSpiNor,
      &g_flexspiMemoryInterface },

    // FLEXSPI Secure Alias memory (Size will be updated later during interface initialization)
    { 0x18000000u, 0x1fffffffu, kMemoryNotExecutable | kMemoryType_FLASH | kMemoryAliasAddr | kMemorySkipInitError,
      kMemoryFlexSpiNor, &g_flexspiMemoryInterface },
#endif
      
    // Terminator
    { 0 }
};

external_memory_map_entry_t g_externalMemoryMap[] = {
    { 0 } // Terminator
};

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
