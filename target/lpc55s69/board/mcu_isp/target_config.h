/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__TARGET_CONFIG_H__)
#define __TARGET_CONFIG_H__

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// In Niobe4, ROM doesn't occupy RAM0 region, Only the arena occupies the first 16KB of RAM0
#define ARENA_START 0x30000000u

#define ARENA_SIZE 0x4000u

#define BL_FEATURE_PROPERTY_RESERVED_REGION_COUNT 5

#define BL_FEATURE_RESERVED_REGION_AUTO_PROBE 0

#define GET_PHYSICAL_ADDR(addr) ((addr) & ~(1u << 28))

/*< NXP Area Mode parameters */
#define FLASH_NXP_AREA_START (0x9EC00U)
#define FLASH_NXP_AREA_SIZE (4544U)
#define FLASH_NXP_AREA_SHA256_DIGEST (0x9FDC0U)

#define FLASH_IFR_FLASH_OPMODE_PARAMS_ADDR (0x9FC08U)

//! @brief Constants for sram partition
enum _sram_partition
{
    kSram_LowerPart = 1,
    kSram_UpperPart = 3,
};

//! @brief Version constants for the target.
enum _target_version_constants
{
    kTarget_Version_Name = 'T',
    kTarget_Version_Major = 1,
    kTarget_Version_Minor = 1,
    kTarget_Version_Bugfix = 0
};

//! ISP Peripheral definitions
enum __isp_peripheral_constants
{
    kIspPeripheral_Auto = 0,
    kIspPeripheral_UsbHid = 1,
    kIspPeripheral_Uart = 2,
    kIspPeripheral_SpiSlave = 3,
    kIspPeripheral_I2cSlave = 4,

};

//! @brief Memory Map index constants
enum _memorymap_constants
{
    kIndexFlashArray = 0,
    kIndexFlashArrayAlias = 1,

    kIndexFFR = 2,
    kIndexSRAM = 3,
};

//! @brief FFR constants
enum
{
    kNxpFactoryConfigAddr_FLASH_CTRL_OPMODE = 0x9fc08u,

    kPartConfigAddr_PERIPHENCFG = 0x9fca0u,
    kPartConfigAddr_RAMSIZECFG = 0x9fca4u,
    kPartConfigAddr_FLASHSIZECFG = 0x9fca8u,

    kFaultAnalysisEnableAddr_Primary = 0x9e028u,
    kFaultAnalysisEnableAddr_Backup = 0x9e228u,

    kTrimConfigAddr_FRO1MHz = 0x9fcd0u,
    kTrimConfigAddr_DCDC_0 = 0x9fce0u,
    kTrimConfigAddr_DCDC_1 = 0x9fce4u,
    kTrimConfigAddr_BOD = 0x9fcf0u,

    kKeyStoreAddr_Start = 0x9e600u,
    kKeyStoreAddr_End = 0x9ec00u,

    kCmfaAddr_Start = 0x9e400u,
    kCmfaAddr_DigestStart = 0x9e5e0u,

    kNmpaAddr_Part1Start = 0x9ec00u,
    kNmpaAddr_Part1End = 0x9f000u,
    kNmpaAddr_Part2Start = 0x9fc00u,
    kNmpaAddr_Part2End = 0x9fdc0u,
    kNmpaAddr_DigestStart = 0x9fdc0u,
};

enum
{
    kReserved_SRAM_Size = 0x6000,
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

enum
{
#ifndef BL_TARGET_FPGA
    kRecoverySpiBaudrate = 24000000u,
    kRecoverySpiClockRoot = 48000000u,
#else
    kRecoverySpiBaudrate = 6000000u,
    kRecoverySpiClockRoot = 12000000u,
#endif
    kRecoverySpiInstance = 3,
    kRecoveryImageBuffer = 0x20006000u,
};

// !@brief Target specific Secure counter definitions

#define SC_VALUE_INIT_HARDWARE_INNER_STEPS (20)

#define ROMPATCH_RAM_START ((uint32_t)__section_begin(".rompatch_section"))
#define ROMPATCH_RAM_SIZE (0x600u)

//!@brief Device Type Pin definitions
#define DEVICE_TYPE_PIN_SHIFT (24u)
#define DEVICE_TYPE_PIN_MASK (0xFFu << DEVICE_TYPE_PIN_SHIFT)
#define DEVICE_TYPE_PIN_36_WLCSP (0x8u)

//!@brief Determine whether specified SPI instance is available or not
extern bool is_spi_instance_available(uint32_t instance);
#define BL_SPI_INSTANCE_PRESENT(instance) is_spi_instance_available(instance)

//! CRC check pinmux configurations
// Note: This default muxing slot of selected crc check failure pin must be ALT0
#define CRC_CHECK_FAILURE_PIN_NUMBER 11
#define CRC_CHECK_FAILURE_PIN_PORT PORTA
#define CRC_CHECK_FAILURE_PIN_GPIO GPIOA
#define CRC_CHECK_FAILURE_PIN_DEFAULT_MODE 0
#define CRC_CHECK_FAILURE_PIN_GPIO_MODE 1

#endif // __TARGET_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
