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

#define ROM_HIDING_SIZE_IN_WORDS (0x5000u / sizeof(uint32_t))
/* Clear flexspi remap setting if boot fail */
extern void clear_persistent_status(void);
#define PREPARE_ISP_ENV() clear_persistent_status()

enum
{
    kIndexFlexSpiNor = 4,
    kIndexFlexSpiNorAlias = 5,
    kIndexFlexSpiNor2 = 6,
};

#define FLEXSPI_AMBA_BASE_ADDS                    \
    {                                             \
        FlexSPI_AMBA_BASE, FlexSPI_AMBA_BASE      \
    }

//!@brief Primary boot source definitions
enum
{
    kBootFuseSrc_IspPin = 0,
    kBootFuseSrc_Ospi = 1,
    kBootFuseSrc_uSDHC0 = 2,
    kBootFuseSrc_uSDHC1 = 3,
    kBootFuseSrc_SPI = 4,
    kBootFuseSrc_OspiChannelB = 5,
    kBootFuseSrc_UART = 6,
    kBootFuseSrc_SpiFlash = 7,
    kBootFuseSrc_USBHID = 8,
    kBootFuseSrc_Isp = 9,
    kBootFuseSrc_HTOL_Test = 10,
    kBootFuseSrc_Ospi_FallbackSpiNor_PortB = 11,
    kBootFuseSrc_Ospi_FallbackSpiNor = 12,
    kBootFuseSrc_uSDHC0_FallbackSpiNor = 13,
    kBootFuseSrc_uSDHC1_FallbackSpiNor = 14,
};

//!@brief Reserved RAM size during Bootloader execution
enum
{
    kBootloader_ReservedRAM_Size = 0x1c000
};

//!@brief Boot pin source definitions
enum
{
    kBootSrc_ISP_AUTO = 0, // SPI I2C UART USB-HID
    kBootPinSrc_SD_uSDHC0 = 1,
    kBootSrc_OspiChannelB = 2,
    kBootSrc_Ospi = 3,
    kBootPinSrc_eMMC_uSDHC0 = 4,
    kBootSrc_DFU = 5,
    kBootSrc_ISP_All = 6,
    kBootSrc_Serial = 7,
};

enum
{
    kOspiPortA1 = 0,
    kOspiPortB1 = 1,
    kOspiPortA2 = 2,
    kOspiPortB2 = 3,
};

//!@brief uSDHC boot device type definitions
enum
{
    kBootFuseDeviceType_MMC = 0,
    kBootFuseDeviceType_SD = 1,

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

enum _ospi_constants
{
    kOspiMem_BaseAddr = 0x08000000,
    kOspiMem_ConfigBlockOffset = 0x400,
    kOspiMem_BootImageVersionOffset = 0x600,
    kOspiMem_ImageStartOffset = 0x1000,
    kOspiMem_MaxSizeInBytes = 128ul * 1024ul * 1024ul,
    // Formula: For DDR command, delay cell = 1/4 cycle time at max support frequency / interval per cell
    //        : For SDR command, delay cell = 1/2 cycle time at max support frequency / interval per cell
    kOspiDelayCell_FailsafeValueDdr = 30,
    kOspiDelayCell_FailsafeValueSdr = 50,
};

//!@brief FlexSPI related definitions
enum
{
    kFlexSpi_ConfigBlockOffset = 0x400,
};

enum _sdmmc_constants
{
    kSdmmcMem_ImageStartOffset = 0x1000, // Must be aligned to FSL_SDMMC_DEFAULT_BLOCK_SIZE(512);
    kSdmmcMem_InitialImageSize = 0x200,  // Must be aligned to FSL_SDMMC_DEFAULT_BLOCK_SIZE(512);
};

enum _lpcnext0_log
{
    kInitStatus_DedError = 0x10,
    kInitStatus_NxpFactoryState = 0x11,
    kInitStatus_FaultAnalysis = 0x12,
    kInitStatus_LifeCycleDev = 0x13,
    kInitStatus_LifeCycleProd = 0x14,

    kSubState_PartConfig = 0xe0,
    kSubState_ClockConfig = 0xe1,
    kSubState_TrngConfig = 0xe2,
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
    kFlexSpiSerialClk_32MHz = 1,
    kFlexSpiSerialClk_48MHz = 2,
    kFlexSpiSerialClk_64MHz = 3,
    kFlexSpiSerialClk_96MHz = 4,
    kFlexSpiSerialClk_192MHz = 5,
};

enum
{
    kFlexspiInstance_0 = 0,
    kFlexspiInstance_1 = 1,

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

#define USB_IRQ_NUMBER USB_IRQn

//!@brief Add Security VA suggested part defintions
#define OPT_CONFIG_WAFER_TEST_STATE 0x00000000u
#define OPT_CONFIG_NXP_ROM_DEV_STATE 0x9B99BC98u
#define OPT_CONFIG_NXP_RELEASE_STATE 0x43666764u
#define OPT_CONFIG_NXP_ROM_VISIBLE_DEV_STATE 0x64664367u
#define OTP_CONFIG_NXP_ROM_VISIBLE_RELEASE_STATE 0xBC99989Bu

#define GET_PHYSICAL_ADDR(addr) skboot_addr_to_nonsecure_tzm_addr(addr)

#define STACK_PROTECTION_INIT()                                                   \
    \
{                                                                          \
        extern uint32_t __stack_chk_guard;                                        \
        extern uint32_t skboot_refresh_stack_canary(void);                        \
        uint32_t new_stack_canary = skboot_refresh_stack_canary();                \
        if (__stack_chk_guard != new_stack_canary)                                \
        {                                                                         \
            go_fatal_mode();                                                      \
        }                                                                         \
        debug_printf("Stack canary gets updated, value=%x\n", __stack_chk_guard); \
    \
}

#define OTP_PROGRAM_POST_HANDLER(idx, word)              \
    \
{                                                 \
        if (idx == OTP_OTFAD_KEY_SCRAMBLE_SEED_IDX)      \
        {                                                \
            OCOTP->OTP_SHADOW[(idx)] = (uint32_t)(word); \
        }                                                \
    \
}

#define HS_PAD_COMP_TMO (100)

#endif // __TARGET_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
