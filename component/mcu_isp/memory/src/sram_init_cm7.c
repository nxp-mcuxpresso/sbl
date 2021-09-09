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
#include "sram_init.h"
#include "utilities/fsl_assert.h"

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
/*
//! @brief Determine if a number is power of 2.
static inline bool is_power_of_2(uint32_t number)
{
    return !(number && (number & (number -1)));
}
*/

//! @brief Initialize address ranges of SRAM for chips belongs to cm4 family
status_t sram_init(void)
{
    memory_map_entry_t *map = NULL;
#if defined(__CORE_CM7_H_GENERIC)

    //    uint32_t ram_size = 0;
    uint32_t tmp;
#if !defined(K32H844P_SERIES)
#if FSL_FEATURE_SIM_OPT_HAS_RAMSIZE
    tmp = (SIM->SOPT1 & SIM_SOPT1_RAMSIZE_MASK) >> SIM_SOPT1_RAMSIZE_SHIFT;
#elif FSL_FEATURE_SIM_SDID_HAS_SRAMSIZE
    tmp = (SIM->SDID & SIM_SDID_RAMSIZE_MASK) >> SIM_SDID_RAMSIZE_SHIFT;
#else
#error "No valid RAMSIZE defined!"
#endif
#else
#warning "RAM SIZE TBD"
    tmp = kRamSize128kIndex;
#endif // #if !defined(K32H844P_SERIES)
    switch (tmp)
    {
        case kRamSize16kIndex:
            break;
        case kRamSize24kIndex:
            break;
        case kRamSize32kIndex:
            break;
        case kRamSize48kIndex:
            break;
        case kRamSize64kIndex:
            break;
        case kRamSize96kIndex:
            break;
        case kRamSize128kIndex:
            // Update DTCM memory range  - 64 KB
            map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexDTCM];
            map->endAddress = map->startAddress + 1024UL * 64;

            // Update OCRAM memory range - 0 KB
            map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexOCRAM];
            map->endAddress = map->startAddress;
            break;
        case kRamSize256kIndex:
            // No need to update, defined by default.
            break;
        default:
            break;
    }

#else
#error "This function only applies to cm7 family"
#endif // __CORE_CM7_H_GENERIC

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
