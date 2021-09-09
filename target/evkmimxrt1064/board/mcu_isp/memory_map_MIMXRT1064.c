/*
 * Copyright 2016-2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "memory/memory.h"

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Memory map for RT105x.
//!
//! This map is not const because it is updated at runtime with the actual sizes of
//! flash and RAM for the chip we're running on.
//! @note Do not change the index of Flash, SRAM, or QSPI (see memory.h).
memory_map_entry_t g_memoryMap[] = {
    // ITCM SRAM(128KB)
    { 0x00000000, 0x0001ffff, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },
    // DTCM SRAM(128KB)
    { 0x20000000, 0x2001ffff, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },
    // OCRAM (256KB)
    { 0x20200000, 0x2023ffff, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },
#if BL_FEATURE_FLEXSPI_NOR_MODULE
    // FlexSPI AMBA memory
    { 0x70000000, 0x7fffffff, kMemoryNotExecutable | kMemoryType_FLASH, kMemoryFlexSpiNor, &g_flexspiMemoryInterface },
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE
    // Terminator
    { 0 }
};

external_memory_map_entry_t g_externalMemoryMap[] = {
    { 0 } // Terminator
};

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
