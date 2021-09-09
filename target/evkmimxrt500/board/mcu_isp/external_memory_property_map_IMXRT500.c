/*
 * Copyright 2017-2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "property/property.h"
#include "bootloader_common.h"
#include "memory/memory.h"

const external_memory_property_interface_t g_externalMemPropertyInterfaceMap[] = {
#if BL_FEATURE_FLEXSPI_NOR_MODULE
    { kMemoryFlexSpiNor, flexspi_nor_get_property },
#endif
    { 0 } // Terminator
};

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
