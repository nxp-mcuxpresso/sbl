/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "memory/memory.h"
#if !defined(BOOTLOADER_HOST)
#include "fsl_device_registers.h"
#endif // BOOTLOADER_HOST
#include "utilities/fsl_assert.h"
#include "sram_init.h"

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Initialize address ranges of SRAM for chips belongs to cm0plus family
status_t sram_init(void)
{
#if defined(__CORE_CM0PLUS_H_GENERIC)

    uint32_t ram_size = 0;

#if FSL_FEATURE_SIM_OPT_HAS_RAMSIZE
    uint32_t tmp = (SIM->SOPT1 & SIM_SOPT1_RAMSIZE_MASK) >> SIM_SOPT1_RAMSIZE_SHIFT;
    switch (tmp)
    {
        case 1:
            ram_size = 8 * 1024;
            break;
        case 3:
            ram_size = 16 * 1024;
            break;
        case 4:
            ram_size = 24 * 1024;
            break;
        case 5:
            ram_size = 32 * 1024;
            break;
        case 6:
            ram_size = 48 * 1024;
            break;
        case 7:
            ram_size = 64 * 1024;
            break;
        case 8:
            ram_size = 96 * 1024;
            break;
        case 9:
            ram_size = 128 * 1024;
            break;
        case 11:
            ram_size = 256 * 1024;
            break;
        default:
            break;
    }
#else
    uint32_t tmp = (SIM->SDID & SIM_SDID_SRAMSIZE_MASK) >> SIM_SDID_SRAMSIZE_SHIFT;

    if (tmp <= kMaxRamIndex)
    {
        ram_size = kMinKlRamSize << tmp;
    }
#endif

    assert(ram_size > 0);

    if (ram_size > 0)
    {
        // Update address range of SRAM
        memory_map_entry_t *map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexSRAM];
        tmp = ram_size / (kSram_LowerPart + kSram_UpperPart);
        map->startAddress = kSRAMSeparatrix - tmp * kSram_LowerPart;   // start of SRAM
        map->endAddress = kSRAMSeparatrix + tmp * kSram_UpperPart - 1; // end of SRAM
    }
#else
#error "This function only applies to cm0plus family"
#endif // __CORE_CM0PLUS_H_GENERIC

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
