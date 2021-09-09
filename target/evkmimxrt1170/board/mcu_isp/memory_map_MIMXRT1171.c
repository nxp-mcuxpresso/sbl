/*
 * Copyright 2018-2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "memory/memory.h"
#include "memory_config.h"
#include "mpu/bl_mpu.h"

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
// Dummy interface, do nothing, this definition is defined here to satisfy the memory interface check
const memory_region_interface_t k_dummyInterface;
//! @brief Memory map for RT105x.
//!
//! This map is not const because it is updated at runtime with the actual sizes of
//! flash and RAM for the chip we're running on.
//! @note Do not change the index of Flash, SRAM, or QSPI (see memory.h).
memory_map_entry_t g_memoryMap[] = {
    // ITCM SRAM(512KB)
    {.startAddress = M7_ITCM_SRAM_START_ADDRESS,
     .endAddress = 0, /* Flexible size which will be filled during ROM initialization. */
     .memoryProperty = kMemoryIsExecutable | kMemoryType_RAM,
     .memoryId = kMemoryInternal,
     .memoryInterface = &g_normalMemoryInterface },

    // DTCM SRAM(512KB)
    {.startAddress = M7_DTCM_SRAM_START_ADDRESS,
     .endAddress = 0, /* Flexible size which will be filled during ROM initialization. */
     .memoryProperty = kMemoryIsExecutable | kMemoryType_RAM,
     .memoryId = kMemoryInternal,
     .memoryInterface = &g_normalMemoryInterface },

    // OCRAM (2048KB)
    {.startAddress = OCRAM_START_ADDRESS,
     .endAddress = 0, /* Flexible size which will be filled during ROM initialization. */
     .memoryProperty = kMemoryIsExecutable | kMemoryType_RAM,
     .memoryId = kMemoryInternal,
     .memoryInterface = &g_normalMemoryInterface },

#if BL_FEATURE_FLEXSPI_NOR_MODULE
    // FlexSPI1 AMBA memory
    {.startAddress = FLEXSPI1_AMBA_START_ADDRESS,
     .endAddress = FLEXSPI1_AMBA_END_ADDRESS,
     .memoryProperty = kMemoryNotExecutable | kMemoryType_FLASH,
     .memoryId = kMemoryFlexSpiNor,
     .memoryInterface = &g_flexspiMemoryInterface },

    // FlexSPI2 AMBA memory
    {.startAddress = FLEXSPI2_AMBA_START_ADDRESS,
     .endAddress = FLEXSPI2_AMBA_END_ADDRESS,
     .memoryProperty = kMemoryNotExecutable | kMemoryType_FLASH,
     .memoryId = kMemoryFlexSpiNor,
     .memoryInterface = &g_flexspiMemoryInterface },
#endif

    // Terminator
    { 0 }
};

#if BL_FEATURE_EXPAND_MEMORY
external_memory_map_entry_t g_externalMemoryMap[] = {
    { 0 } // Terminator
};
#endif // #if BL_FEATURE_EXPAND_MEMORY

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
