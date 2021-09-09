/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Bootloader global context data.
//!
//! @ingroup context
bootloader_context_t g_bootloaderContext = {
    .memoryInterface = &g_memoryInterface,
    .memoryMap = g_memoryMap,
#if BL_FEATURE_EXPAND_MEMORY
    .externalMemoryMap = g_externalMemoryMap,
#endif // BL_FEATURE_EXPAND_MEMORY
    .allPeripherals = g_peripherals,
    .activePeripheral = NULL, // Filled in at run time.
    .propertyInterface = &g_propertyInterface,
#if BL_FEATURE_ISP_BOOT
    .commandInterface = &g_commandInterface,
#else
    .commandInterface = NULL,
#endif
#if BL_FEATURE_HAS_INTERNAL_FLASH
    .flashDriverInterface = &g_flashDriverInterface,
    .allFlashState = g_flashState,
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#if AES_SECURITY_SUPPORTED
    .aesInterface = &g_aesInterface,
#endif

};
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
