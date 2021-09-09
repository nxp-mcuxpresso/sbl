/*
 * Copyright 2017-2021 NXP
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
    kFlexSpi_AhbMemoryMaxSizeMB = (504u * 1024u * 1024u),
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
    kFlexSpi_ConfigBlockOffset = 0x0,
};

//!@brief Memory index definitions
enum
{
    kIndexSRAM = 0,

    kIndexITCM = 0,
    kIndexDTCM = 1,
    kIndexOCRAM = 2,

    kIndexFlexSpiNor = 3,
    kIndexFlexSpiNor2 = 4,
    kIndexFlexSpiNorAlias = 5,
};

#define FLEXSPI_AMBA_BASE_ADDS                                   \
    {                                                            \
        FlexSPI_AMBA_BASE, FlexSPI_AMBA_BASE, FlexSPI2_AMBA_BASE \
    }

#define USB_CLOCK_NAME kCLOCK_UsbOh3

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
    kBootDevice_MMC_SD = 0x08,
    kBootDevice_SemcNOR = 0x10,
    kBootDevice_SemcNAND = 0x20,
} bootdevice_option_t;

extern uint32_t get_flexspinor_instance(void);
extern uint32_t get_flexspinor_amba_base(void);

#endif // __TARGET_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
