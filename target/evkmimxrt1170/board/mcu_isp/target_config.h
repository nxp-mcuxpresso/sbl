/*
 * Copyright 2018 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__TARGET_CONFIG_H__)
#define __TARGET_CONFIG_H__

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Constants for FlexSPI features.
enum
{
    kFlexSpi_AhbMemoryMaxSizeMB = (256u * 1024u * 1024u),
};

//! @brief Version constants for the target.
enum _target_version_constants
{
    kTarget_Version_Name = 'T',
    kTarget_Version_Major = 1,
    kTarget_Version_Minor = 0,
    kTarget_Version_Bugfix = 0
};

//!@brief FlexSPI related definitions
enum
{
    kFlexSpi1_AMBA_Base = 0x30000000u,
    kFlexSpi1_ALIAS_Base = 0x08000000u,
    kFlexSpi2_AMBA_Base = 0x60000000u,
    kFlexSpi_ConfigBlockOffset = 0x400,
    kFlexSpi_Key_Store_Offset = 0x800,
};

//!@brief Memory index definitions
enum
{
    kIndexSRAM = 0,

    kIndexITCM = 0,
    kIndexDTCM = 1,
    kIndexOCRAM = 2,

    kIndexFlexSpiNor = 3,
    kIndexFlexSpiNorAlias = 4,
    kIndexFlexSpiNor2 = 5,
    kIndexSemc = 6,
};

#define FLEXSPI_AMBA_BASE_ADDS                                    \
    {                                                             \
        FlexSPI1_AMBA_BASE, FlexSPI1_AMBA_BASE, FlexSPI2_AMBA_BASE \
    }

//!@brief PIT backward compatible defintion
#define kCLOCK_Pit kCLOCK_Pit1

//!@brief FLEXSPI instnaces
enum
{
    kFlexspiInstance_1 = 1,
    kFlexspiInstance_2 = 2,
};

//!@brief FLEXSPI Boot Clock Source
enum
{
    kFlexSpiBootClkcSrc = 4,
};

#define USB_CLOCK_NAME kCLOCK_Usb

enum
{
    kProduct_USB_PID = 0x0073,
    kProduct_USB_PID_high = 0x00,
    kProduct_USB_PID_low  = 0x73,
    
    kProduct_USB_VID = 0x15a2,
    kProduct_USB_VID_high = 0x15,
    kProduct_USB_VID_low  = 0xa2,
};

//! @brief Boot device Option
typedef enum _bootdevice_option
{
    kBootDevice_FlexSpiNOR = 0x01,
    kBootDevice_FlexSpiNAND = 0x02,
    kBootDevice_SPIEEPROM = 0x04,
    kBootDevice_SD = 0x08,
    kBootDevice_SemcNOR = 0x10,
    kBootDevice_SemcNAND = 0x20,
    kBootDevice_MMC = 0x40,
} bootdevice_option_t;

//#define XSPI_FLASH_DUMMY_CYCLE_PROBE_OFFSET (0x400)

#endif // __TARGET_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
