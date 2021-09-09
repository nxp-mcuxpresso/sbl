/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "memory/memory.h"
//#include "trustzone/tzm_mpu.h"
//#include "trustzone/tzm_sau.h"

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Memory map for LPC5588.
//!
//! This map is not const because it is updated at runtime with the actual sizes of
//! flash and RAM for the chip we're running on.
//! @note See memory.h for index requirements.
memory_map_entry_t g_memoryMap[] = {
    // Flash array (632KB)
    { 0x00000000, 0x0009dfff, kMemoryIsExecutable | kMemoryType_FLASH, kMemoryInternal, &g_flashMemoryInterface },
    // Flash Alias array (632KB)
    { 0x10000000, 0x0009dfff, kMemoryIsExecutable | kMemoryType_FLASH | kMemoryAliasAddr, kMemoryInternal,
      &g_flashMemoryInterface },
    // Flash FFR region (8KB)
    { 0x0009e000, 0x0009ffff, kMemoryNotExecutable | kMemoryType_FLASH, kMemoryFFR, &g_ffrMemoryInterface },
    // Note: Some IP has shared RAM region with RAMX and RAM4, ROM doesn't support access RAMX and RAM4 to avoid
    // potential security holes, in similar cases, ROM also doesn't support device memory access to avoid potential
    // security risks
    // SRAM (256KB)
    { 0x20000000, 0x2003ffff, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },
    // SRAM Alias (256KB)
    { 0x30000000, 0x2003ffff, kMemoryIsExecutable | kMemoryType_RAM | kMemoryAliasAddr, kMemoryInternal,
      &g_normalMemoryInterface },
    // RAMX (32KB)
    { 0x04000000, 0x04007fff, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },
    // RAMX Alias (32KB)
    { 0x14000000, 0x14007fff, kMemoryIsExecutable | kMemoryType_RAM | kMemoryAliasAddr, kMemoryInternal,
      &g_normalMemoryInterface },

    { 0 } // Terminator
};

#if BL_FEATURE_EXPAND_MEMORY
external_memory_map_entry_t g_externalMemoryMap[] = {
#if BL_FEATURE_SPI_NOR_EEPROM_MODULE
    // Serial NOR/EEPROM memory
    { kMemorySpiNorEeprom, 0, 0x10000, 256, &g_spiNorEepromMemoryInterface },
#endif // BL_FEATURE_SPI_NOR_EEPROM_MODULE

    { 0 } // Terminator
};
#endif

#if BL_FEATURE_MPU_ENABLE
const mpu_region_config_t g_mpuMemoryMap[] = {

    /* regionNum, baseAddress, size, Region Index Attribute, Access Attribute */
    // Flash 640kB
    { 0u, 0x00000000u, 0x000a0000u, kMPU_InternalMemoryAttrIndex, kMPU_FlashAccessAttribute },
    // RAM for CASPER, ROM variables, STACK
    { 1u, 0x14000000u, 0x00005a00u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },
    // RAM for ROMPATCH
    { 2u, 0x14005a00u, 0x00000600u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithCode },
    { 3u, 0x14006000u, 0x00002000u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },
#ifndef BL_TARGET_RAM
    // secure ROM 128kB
    { 4u, 0x13000000u, 0x00020000u, kMPU_InternalMemoryAttrIndex, kMPU_ROMAccessAttribute },
    // RAM0-4
    { 5u, 0x30000000u, 0x00044000u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },
    // non-secure RAM for user application, RAM 4 is included
    { 6u, 0x20006000u, 0x0003e000u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },
#else
    // secure ROM 128kB
    { 4u, 0x30020000u, 0x00020000u, kMPU_InternalMemoryAttrIndex, kMPU_ROMAccessAttribute },
    // RAM0-4
    { 5u, 0x30000000u, 0x00020000u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },
    // non-secure RAM for user application, RAM 4 is excluded
    { 6u, 0x20006000u, 0x0000a000u, kMPU_InternalMemoryAttrIndex, kMPU_RAMAccessAttributeWithoutCode },
#endif
    // APB0, APB1 and AHB peripherlas
    { 7u, 0x50000000u, 0x0010c000u, kMPU_PeripheralsAttrIndex, kMPU_RAMAccessAttributeWithoutCode },
};
const uint32_t g_mpuRegionCount = sizeof(g_mpuMemoryMap) / sizeof(g_mpuMemoryMap[0]);

const sau_region_config_t g_sauMemoryMapS[] = {
    /* regionNum, baseAddress, size, Security Attribute */
    { 0u, 0x00000000u, 0x000a0000u, kSAU_NonSecureAttribute }, // Whole FLASH is non-secure during boot execution
// Whole RAM, which not used by bootloader is non-secure during boot execution
#ifndef BL_TARGET_RAM
    { 1u, 0x20006000u, 0x0003a000u, kSAU_NonSecureAttribute },
#else
    { 1u, 0x20006000u, 0x0001a000u, kSAU_NonSecureAttribute },
#endif
};
const uint32_t g_sauRegionCountS = sizeof(g_sauMemoryMapS) / sizeof(g_sauMemoryMapS[0]);

const sau_region_config_t g_sauMemoryMapNS[] = {
    /* regionNum, baseAddress, size, Security Attribute */
    { 0u, 0x00000000u, 0x13000000u, kSAU_NonSecureAttribute }, // Memory  space before permanently secure ROM
    { 1u, 0x13001000u, 0xccfff000u, kSAU_NonSecureAttribute }, // Memory  space after permanently secure ROM
};
const uint32_t g_sauRegionCountNS = sizeof(g_sauMemoryMapNS) / sizeof(g_sauMemoryMapNS[0]);
#endif
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
