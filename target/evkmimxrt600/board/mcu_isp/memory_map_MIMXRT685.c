/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "memory/memory.h"
#if BL_FEATURE_TRUSTZONE
#include "trustzone/tzm_mpu.h"
#include "trustzone/tzm_sau.h"
#endif

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Memory map for LPC6824.
//!
//! This map is not const because it is updated at runtime with the actual sizes of
//! flash and RAM for the chip we're running on.
//! @note See memory.h for index requirements.
memory_map_entry_t g_memoryMap[] = {
    // SRAM0 Secure Alias address (size: 4.5MB)
    { 0x10000000u, 0x10477fffu, kMemoryIsExecutable | kMemoryType_RAM | kMemoryAliasAddr, kMemoryInternal,
      &g_normalMemoryInterface },

    // SRAM0 (size: 4.5MB)
    { 0x00000000u, 0x00477fffu, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },

    // System RAM (size: 4.5MB)
    { 0x20000000u, 0x20477fffu, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },

    // System RAM Secure Alias address (size: 4.5MB)
    { 0x30000000u, 0x30477fffu, kMemoryIsExecutable | kMemoryType_RAM | kMemoryAliasAddr, kMemoryInternal,
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

#if BL_FEATURE_MPU_ENABLE
const mpu_region_config_t g_mpuMemoryMap[] = {
    /* regionNum, baseAddress, size, Region Index Attribute, Access Attribute */
    // Non-secure RAM bootloader stack and variables
    { 0u, 0x0001c000u, 0x00464000u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },

    // Secure RAM bootloader stack and variables
    { 1u, 0x10010000u, 0x0000bc00u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },

    // Secure RAM for bootloader ROM patches, need to change attribute to kMPU_ROMAccessAttribute if romcp init passed.
    { 2u, 0x1001bc00u, 0x00000400u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },

    // Secure ROM 256kB
    { 3u, 0x13000000u, 0x00040000u, kMPU_InternalMemoryAttrIndex, kMPU_ROMAccessAttribute },

    // Quad/Octal SPI memory mapped space
    { 4u, 0x08000000u, 0x08000000u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },

    // APB0, APB1 Peripherals
    { 5u, 0x50000000u, 0x00040000u, kMPU_DeviceMemory_nGnRnE, kMPU_RAMAccessAttributeWithoutCode },

    // AHB Peripherals
    { 6u, 0x50100000u, 0x00070000u, kMPU_DeviceMemory_nGnRnE, kMPU_RAMAccessAttributeWithoutCode },

    // Non-secure alias RAM for user application
    { 7u, 0x2001c000u, 0x00464000u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },
};
const uint32_t g_mpuRegionCount = sizeof(g_mpuMemoryMap) / sizeof(g_mpuMemoryMap[0]);

const sau_region_config_t g_sauMemoryMapS[] = {
    /* regionNum, baseAddress, size, Security Attribute */
    { 0u, 0x00000000u, 0x00480000u, kSAU_NonSecureAttribute }, // Whole RAM is non-secure during boot execution
};
const uint32_t g_sauRegionCountS = sizeof(g_sauMemoryMapS) / sizeof(g_sauMemoryMapS[0]);

const sau_region_config_t g_sauMemoryMapNS[] = {
    /* regionNum, baseAddress, size, Security Attribute */
    { 0u, 0x00000000u, 0x13000000u, kSAU_NonSecureAttribute }, // Memory  space before permanently secure ROM
    { 1u, 0x13006000u, 0xccfff000u, kSAU_NonSecureAttribute }, // Memory  space after permanently secure ROM
};
const uint32_t g_sauRegionCountNS = sizeof(g_sauMemoryMapNS) / sizeof(g_sauMemoryMapNS[0]);
#endif

external_memory_map_entry_t g_externalMemoryMap[] = {
    { 0 } // Terminator
};

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
