/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __BL_VERSION_H__
#define __BL_VERSION_H__

#include "bootloader_common.h"
#include "bootloader/bl_peripheral.h"
#include "memory/memory.h"
#include "packet/command_packet.h"
#include "bootloader/bl_command.h"
#include "property/property.h"

//! @addtogroup context
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Version constants for the bootloader.
enum _bootloader_version_constants
{
    kBootloader_Version_Name = 'K',
    kBootloader_Version_Major = 3,
    kBootloader_Version_Minor = 0,
    kBootloader_Version_Bugfix = 0
};

//! @}

#endif // __BL_VERSION_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
