/*
 * The Clear BSD License
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
    // ITCM SRAM(256KB)
    { 0x00000000, 0x0003ffff, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },
    // DTCM SRAM(256KB)
    { 0x20000000, 0x2003ffff, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },
    // OCRAM (256KB)
    { 0x20200000, 0x2023ffff, kMemoryIsExecutable | kMemoryType_RAM, kMemoryInternal, &g_normalMemoryInterface },
#if BL_FEATURE_FLEXSPI_NOR_MODULE
    // FlexSPI1 AMBA memory
    { 0x60000000, 0x6fffffff, kMemoryNotExecutable | kMemoryType_FLASH, kMemoryFlexSpiNor, &g_flexspiMemoryInterface },
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE
          // AIPS0 peripherals
          //    { 0x40000000, 0x4007ffff, kMemoryNotExecutable | kMemoryType_Device, &g_deviceMemoryInterface },
          // AIPS1 peripherals
          //    { 0x40080000, 0x400fffff, kMemoryNotExecutable | kMemoryType_Device, &g_deviceMemoryInterface },
    { 0 } // Terminator
};

external_memory_map_entry_t g_externalMemoryMap[] = {
    { 0 } // Terminator
};

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
