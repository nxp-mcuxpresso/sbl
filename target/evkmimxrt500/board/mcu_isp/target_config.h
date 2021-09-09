/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#if !defined(__TARGET_CONFIG_H__)
#define __TARGET_CONFIG_H__

#include "fusemap.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
// Workaround to make the __USED work fine with IAR toolchain
#if defined(__ICCARM__)
#undef __USED
#define __USED __root
#endif

extern void flexspinor_flash_auto_init_config();
enum
{
    kIndexFlexSpiNor = 4,
    kIndexFlexSpiNor2 = 5,
    kIndexFlexSpiNorAlias = 6,
};

//!@brief Reserved RAM size during Bootloader execution
enum
{
    kBootloader_ReservedRAM_Size = 0x1c000
};

//!@brief ISP mode definitions
enum
{
    kBootIspPeripList_All = 0,
    kBootIspPeripList_USBHID = 1,
    kBootIspPeripList_UART = 2,
    kBootIspPeripList_SPI = 3,
    kBootIspPeripList_I2C = 4,
    kBootIspPeripList_Disable = 7,
};

//! @brief Version constants for the target.
enum _target_version_constants
{
    kTarget_Version_Name = 'T',
    kTarget_Version_Major = 2,
    kTarget_Version_Minor = 0,
    kTarget_Version_Bugfix = 0
};

enum _flexspi_constants
{
    kOspiMem_BaseAddr = 0x08000000,
    kOspiMem_ConfigBlockOffset = 0x400,
    kOspiMem_BootImageVersionOffset = 0x600,
    kOspiMem_ImageStartOffset = 0x1000,
    kOspiMem_MaxSizeInBytes = 128ul * 1024ul * 1024ul,
    // Formula: For DDR command, delay cell = 1/4 cycle time at max support frequency / interval per cell
    //        : For SDR command, delay cell = 1/2 cycle time at max support frequency / interval per cell
    kOspiDelayCell_FailsafeValueDdr = 25,
    kOspiDelayCell_FailsafeValueSdr = 50,
};

//! @brief Memory Map index constants
enum _special_memorymap_constants
{
    // kIndexFlashArray = 0,
    kIndexSRAM = 1,
    kIndexSRAMX = 2,
    kIndexSRAM1 = 3,
    kIndexSRAM2 = 4,
    kIndexSRAM3 = 5,

    kRAMSections = 5,

    // kSRAMSeparatrix = (uint32_t)0x20000000 //!< This value is the start address of SRAM_U
};

//! @brief SPI clock frequency
enum __spi_clock_freq
{
    kSpiClockFreq = 48000000u,
    kSpiFlashClockFreq = 24000000u,
};

//! @brief SPI Flash constants
enum
{
    kSpiFlash_ImageInitialStart = 0x1c000,
    kSpiFlash_ImageStartOffset = 0x1000,
};

//! @brief Fatal error related definitions
enum
{
    kFatalError_MaxRetryCnt = 0x04,
};

enum
{
    kFlexSpi1_AMBA_Base = 0x08000000u,
    kFlexSpi1_ALIAS_Base = 0x18000000u,
    kFlexSpi2_AMBA_Base = 0x28000000u
};

enum
{
    kReserved_RamSize = 0x8000u,
};

enum
{
    kIndexITCM = 0,
    kIndexDTCM = 1,
    kIndexOCRAM = 2,
    kIndexFlexSpi1 = 3,
    kIndexFlexSpi1Alias = 4,
    kIndexFlexSpi2 = 5,
    kIndexSemc = 6,
};

#define FLEXSPI_AMBA_BASE_ADDS                    \
    {                                             \
        FlexSPI0_AMBA_BASE, FlexSPI0_AMBA_BASE, FlexSPI1_AMBA_BASE      \
    }


#define kCLOCK_Pit kCLOCK_Pit1


enum
{
    kFlexSpiSerialClk_32MHz = 1,
    kFlexSpiSerialClk_48MHz = 2,
    kFlexSpiSerialClk_64MHz = 3,
    kFlexSpiSerialClk_96MHz = 4,
    kFlexSpiSerialClk_192MHz = 5,
};

enum
{
    kFlexspiInstance_1 = 1,
    kFlexspiInstance_2 = 2,
};

enum
{
    kProduct_USB_PID = 0x0073,
    kProduct_USB_PID_high = 0x00,
    kProduct_USB_PID_low  = 0x73,
    
    kProduct_USB_VID = 0x15a2,
    kProduct_USB_VID_high = 0x15,
    kProduct_USB_VID_low  = 0xa2,
};

#define USB_IRQ_NUMBER USB0_IRQn

#endif // __TARGET_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
