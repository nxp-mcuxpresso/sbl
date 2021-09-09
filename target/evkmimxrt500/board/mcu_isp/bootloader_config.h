/*
* Copyright 2014-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2021 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*
*/

#ifndef __BOOTLOADER_CONFIG_H__
#define __BOOTLOADER_CONFIG_H__

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//
// Bootloader configuration options
//

//==============================================================================
//! @name Device type configuration macros
//@{
#if !defined(BL_DEVICE_IS_LPC_SERIES)
#define BL_DEVICE_IS_LPC_SERIES (1)
#endif
//@}

//==============================================================================
//! @name Target-resident configuration macros
//@{
#if !defined(BL_TARGET_FLASH)
#define BL_TARGET_FLASH (0)
#endif
//@}

//==============================================================================
//! @name Boot configuration macros
//@{
// Determines whether to support isp boot via peripherals.
#define BL_FEATURE_ISP_BOOT (1)

// Determines whether to support master boot
#define BL_FEATURE_MASTER_BOOT (0)

// Determines whether to support master boot via SD card.
#define BL_FEATURE_ENABLE_MKBOOT_DEVICE_SD (0)

// Determines whether to support master boot via MMC card.
#define BL_FEATURE_ENABLE_MKBOOT_DEVICE_MMC (0)

#ifndef BL_TARGET_FPGA
#define BL_FEATURE_ENABLE_MKBOOT_DEVICE_USBDFU (0)
#else
#define BL_FEATURE_ENABLE_MKBOOT_DEVICE_USBDFU (0)
#endif

// Deterine whether to support recovery boot via SPI FLASH
#define BL_FEATURE_ENABLE_MKBOOT_DEVICE_REC_SPI_FLASH (0)

// Deterine whether to support dual image boot via FlexSPI FLASH
#define BL_FEATURE_ENABLE_MKBOOT_DUAL_IMAGE_BOOT (0)

// Determines whether to support booting from either core (cm4/cm0+)
#define BL_FEATURE_ENABLE_BOOT_FROM_EITHER_CORE (0)
// Determines whether to relocate bootloader vector table to SRAM
#define BL_FEATURE_RELOCATE_VECTOR_TABLE (0)

// Defined application vector table address in FLASH
// The bootloader will check this address for the application vector table upon startup.
#if !defined(BL_APP_VECTOR_TABLE_ADDRESS)
#define BL_APP_VECTOR_TABLE_ADDRESS 0x8000
#endif

// Defines bootloader peripheral detection default timeout in milliseconds
// After coming out of reset the bootloader will spin in a peripheral detection
// loop for this amount of time. A zero value means no time out.
#if DEBUG
#define BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT 0
#else
#define BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT 5000
#endif // DEBUG

// Determines whether to support low power during peripheral detection
#define BL_FEATURE_POWERDOWN (0)
// Defines bootloader powerdown timeout in milliseconds
// Note: The bootloader enters a low power mode after waiting for this amount of time.
//   A zero value means no time out.
#if DEBUG
#define BL_DEFAULT_POWERDOWN_TIMEOUT 10000
#else
#define BL_DEFAULT_POWERDOWN_TIMEOUT 600000
#endif // DEBUG
//@}

//! @name Peripheral configuration macros
//@{
// Defines uart peripheral instance
#if !defined(BL_CONFIG_FLEXCOMM_USART_0)
#define BL_CONFIG_FLEXCOMM_USART_0 (1)
#endif
#define BL_CONFIG_FLEXCOMM_USART BL_CONFIG_FLEXCOMM_USART_0

// Defines i2c peripheral instance
#if !defined(BL_CONFIG_FLEXCOMM_I2C_2)
#define BL_CONFIG_FLEXCOMM_I2C_2 (0)
#endif
#define BL_CONFIG_FLEXCOMM_I2C BL_CONFIG_FLEXCOMM_I2C_2

// Defines spi peripheral instance
#if !defined(BL_CONFIG_FLEXCOMM_SPI_8)
#define BL_CONFIG_FLEXCOMM_SPI_8 (0)
#endif
#define BL_CONFIG_FLEXCOMM_SPI BL_CONFIG_FLEXCOMM_SPI_8

// Defines usb peripheral instance
#if !defined(BL_CONFIG_USB_HID)
#define BL_CONFIG_USB_HID (0)
#endif
#if !defined(BL_CONFIG_USB_MSC)
#if defined BL_TARGET_RTL
#define BL_CONFIG_USB_MSC (0)
#else
#define BL_CONFIG_USB_MSC (0)
#endif // BL_TARGET_RTL
#endif
#if !defined(BL_CONFIG_USB_DFU)
#define BL_CONFIG_USB_DFU (0)
#endif
#if !defined(BL_CONFIG_HS_USB_HID)
#ifndef BL_TARGET_FPGA
#define BL_CONFIG_HS_USB_HID (1)
#else
#define BL_CONFIG_HS_USB_HID (0)
#endif // !defined(BL_CONFIG_HS_USB_HID)
#endif

#if !defined(BL_CONFIG_HS_USB_DFU)
#ifndef BL_TARGET_FPGA
#define BL_CONFIG_HS_USB_DFU (0)
#else
#define BL_CONFIG_HS_USB_DFU (0)
#endif // !defined(BL_CONFIG_HS_USB_DFU)
#endif

// Determines optimization for peripheral driver
#define BL_I2C_SIZE_OPTIMIZE 1
#define BL_I2C_USED_INSTANCE 0

// Determines the method of UART autobaud (polling/irq)
#define BL_FEATURE_UART_AUTOBAUD_IRQ (1)

// Determines whether to support peripheral pin sharing
#define BL_FEATURE_6PINS_PERIPHERAL (0)
//@}

// Determines the method of SPI (irq/dma)
#define BL_CONFIG_SPI_ENABLE_DMA (0)

// Determines whether supports nIRQ pin.
#define BL_FEATURE_IRQ_NOTIFIER_PIN (0)

// Determines nIRQ pin active state, 0: active low, 1: active high
#define BL_FEATURE_IRQ_NOTIFIER_PIN_ACTIVE_POLARITY (0)

// Determines if BCA is put in Fuse
#define BL_FEATURE_PUT_BCA_IN_FUSE (0)
//@}

//==============================================================================
//! @name Protocal configuration macros
//@{
// Determines whether use big packet size instead of default packet size(32 Bytes)
#define BL_FEATURE_EXPAND_PACKET_SIZE (1)
// Determines the packet size for serial peripheral(Uart, I2C, SPI)
// Note: Make sure that BL_EXPANDED_FRAMING_PACKET_SIZE <= 512(Maxium 1018 when no dummy read from host.), if
// BL_CONFIG_SPI_ENABLE_DMA = 1
#define BL_EXPANDED_FRAMING_PACKET_SIZE (512)
// Determines the packet size for USB HID.
// Note: Make sure that BL_EXPANDED_USB_HID_PACKET_SIZE < 1018
#define BL_EXPANDED_USB_HID_PACKET_SIZE (1012)
//@}

//==============================================================================
//! @name Commmand configuration macros
//@{
// Determines the supported command set (normal/minimal)
#if !defined(BL_FEATURE_MIN_PROFILE)
#define BL_FEATURE_MIN_PROFILE (1)
#endif

#define BL_FEATURE_READ_MEMORY (1)
#define BL_FEATURE_FILL_MEMORY (1)

#define BL_FEATURE_HAS_NO_RECEIVE_SB_FILE (1)

// Determines whether to support flash-read-resource command
#define BL_FEATURE_HAS_NO_READ_SOURCE (1)
// Determines whether to support flash-erase-all-unsecure command
#define BL_FEATURE_ERASEALL_UNSECURE (0)
// Determines whether to support flash-security-disable command
#define BL_FEATURE_FLASH_SECURITY (0)
// Determines whetehr to support key-provisioning command.
#define BL_FEATURE_KEY_PROVISIONING (0)
//@}

//==============================================================================
//! @name Internal flash configuration macros
//@{
// Determines whether to disable HW verification after every flash program/erase operation
#define BL_FEATURE_FLASH_VERIFY_DISABLE (0)
// (Kinetis only) Determines the method of flash programming (word/section)
#define BL_FEATURE_ENABLE_FLASH_PROGRAM_SECTION (0)
// Determines whether to support FAC related flash API
#define BL_FEATURE_FAC_ERASE (0)
// Determines whether to support internal flash
#define BL_FEATURE_HAS_INTERNAL_FLASH (0)
// Determines whether to support secondary internal flash
#define BL_HAS_SECONDARY_INTERNAL_FLASH (0)
// Determines whether to support cumulative check on flash
#define BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE (1)
//@}

#define FSL_FEATURE_SIM_HAS_NO_UID (1)
#define FSL_FEATURE_SIM_HAS_NO_SDID (1)

#define BL_FEATURE_OTP_MODULE (0)
#define BL_FEATURE_ROMPATCH_MODULE (0)

//==============================================================================
//! @name Reliability configuration macros
//@{
// Determines whether to support application integrity check
#define BL_FEATURE_CRC_CHECK (0)
// Determines whether the CRC info block is in new imager header(LPC style) or BCA(Kinetis style)
#define BL_FEATURE_CRC_INFO_IN_IMG_HDR (1)

#if !defined(BL_TARGET_RAM)
// Determines whether to support reliable-update function and command
#define BL_FEATURE_RELIABLE_UPDATE (0)
// Determines the method of reliable-update (hardware/software)
#define BL_FEATURE_HARDWARE_SWAP_UPDATE (0)
#endif // BL_TARGET_RAM
//@}

//==============================================================================
//! @name Security configuration macros
//@{
// Determines whether to support authentication function
#define BL_FEATURE_AUTHENTICATION (0)
// Determines whether to take use of CAU3 implementation for authentication.
#define BL_FEATURE_AUTHENTICATION_CAU3 (0)
// Determines whether to support boot time authentication
// Note: BL_FEATURE_AUTHENTICATION is required.
#define BL_FEATURE_SECURE_BOOT (0)
#define BL_FEATURE_TRUSTZONE (0)

// Determines whether to support AES encryption in receive-sb-file function
#define BL_FEATURE_ENCRYPTION (0)

// Determines whether to take use of CAU3 implementation for AES encryption
#define BL_FEATURE_ENCRYPTION_CAU3 (0)
// Determines the method of CAU3 implementation (CAU3 module only/CAU3&mbedtls lib)
#define BL_FEATURE_MBEDTLS (0)
// Determines whether to support arena
#define BL_FEATURE_ARENA (0)

// Determines whether to take use of LTC module for AES encryption
#define BL_FEATURE_ENCRYPTION_LTC (0)

// Determines whether to take use of MMCAU implementation for AES encryption
#define BL_FEATURE_ENCRYPTION_MMCAU (0)
// Determines the method of mmcau implementation (MMCAU module/mmcau lib)
#define BL_FEATURE_LOAD_MMCAU_LIB (0)

// Defines IFR index of AES key for SB key unwrapping.
// Note: there are two kinds of key address: IFR address, User defined address
#if BL_FEATURE_ENCRYPTION
#ifndef BL_FEATURE_ENCRYPTION_KEY_ADDRESS
#define BL_FEATURE_IFR_ENCYPTION_KEYS (0x90)
#endif
#endif // #if BL_FEATURE_ENCRYPTION

#define BL_FEATURE_IFR_KEK (0xB1)

// Determines the version of SB loader implementation (sb 1.0/sb 2.0)
#define BL_FEATURE_SB_V2 (0)

#define BL_FEATURE_KB_API (0)

#define BL_ROM_API_TEST_KB_OPTION_ADDRESS (0x9000000)
#define BL_ROM_API_TEST_KB_BUFFER_ADDRESS (0x9001000)
#define BL_ROM_API_TEST_KB_BUFFER_LENGTH (0x2000u)
#define BL_ROM_API_TEST_KB_SESSION_ADDRESS (0x9004000)
#define BL_ROM_API_TEST_KB_IMAGE_ADDRESS (0x0)
#define BL_ROM_API_TEST_KB_IMAGE_LENGTH (0x10000)

#define BL_FEATURE_MPU_ENABLE (0)

#define BL_FEATURE_ZEROIZE_SRAM (0)

#define BL_FEATURE_DEBUG_MAILBOX (0)

#define BL_FEATURE_COMPUTE_UUID (1)

#define BL_FEATURE_SECURE_COUNTER (0)

#define BL_FEATURE_ROM_HIDING (0)

#define BL_FEATURE_HAS_BUS_CRYPTO_ENGINE (0)
#define BL_FEATURE_ERASE_PROGRAM_FOR_BUS_CRYPTO_ENGINE (0)

//@}

//==============================================================================
//! @name External memory configuration macros
//@{
#define BL_FEATURE_HAS_NO_INTERNAL_FLASH (1)

// Determines whether to support qspi flash
#define BL_FEATURE_FLEXSPI_NOR_MODULE (1)

#if BL_FEATURE_FLEXSPI_NOR_MODULE
#define BL_FEATURE_FLEXSPI_NOR_MODULE_PERIPHERAL_INSTANCE (0)
#define BL_FEATURE_FLEXSPI_NOR_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_FLEXSPI_ENCRYPT_PROGRAMMING (0)
#define BL_PROT_REGION_BLOCK_OFFSET(i) (0x400u * (i + 1))
#define BL_FLEXSPI_AMBA_BASE (0x08000000u)
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE
#define FLEXSPI_NOR_FEATURE_ENABLE_PROGRAMMING (BL_FEATURE_MEM_WRITE_ENABLE)

// Determines whether to support alias area access to qspi flash
#define BL_FEATURE_FLEXSPI_ALIAS_AREA (1)

#define BL_FEATURE_ENABLE_MKBOOT_DEVICE_FLEXSPI (1)

#define BL_FEATURE_ENABLE_MKBOOT_DEVICE_SPIFLASH (0)
#define BL_FEATURE_SPI_NOR_EEPROM_MODULE (0)

// Memory expansion features
#define BL_FEATURE_EXPAND_MEMORY (1)

// Determine whether to support MMC/eMMC
#define BL_FEATURE_MMC_MODULE (0)
#if BL_FEATURE_ENABLE_MKBOOT_DEVICE_MMC && !BL_FEATURE_MMC_MODULE
#error "'BL_FEATURE_ENABLE_MKBOOT_DEVICE_MMC' requires 'BL_FEATURE_MMC_MODULE'"
#endif
#if BL_FEATURE_MMC_MODULE
#define BL_FEATURE_MMC_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_MMC_MODULE_ENABLE_PERMANENT_CONFIG (0)
#endif // BL_FEATURE_MMC_MODULE

// Determine whether to support eSD/SD/SDHC/SDXC
#define BL_FEATURE_SD_MODULE (0)
#if BL_FEATURE_ENABLE_MKBOOT_DEVICE_SD && !BL_FEATURE_SD_MODULE
#error "'BL_FEATURE_ENABLE_MKBOOT_DEVICE_SD' requires 'BL_FEATURE_SD_MODULE'"
#endif
#if BL_FEATURE_SD_MODULE
#define BL_FEATURE_SD_MODULE_ERASE_VERIFY (1)
#endif // BL_FEATURE_SD_MODULE

// Determines whether to store external memory info in property
#define BL_FEATURE_EXTERNAL_MEMORY_PROPERTY (1)

// Determines whether to use OTFAD module for image decryption on qspi flash
#define BL_HAS_OTFAD_MODULE (0)
//@}

//==============================================================================
//! @name USB DFU configuration macros
//@{
// Determines image download position (FLASH or RAM)
#define BL_FEATURE_USB_DFU_DOWNLOAD_TO_FLASH (0)
// Determines image address and size
#if BL_FEATURE_USB_DFU_DOWNLOAD_TO_FLASH
#define BL_USB_DFU_APP_ADDRESS (0x0)
#else
#define BL_USB_DFU_APP_ADDRESS (0x400000u)
#endif
#define BL_USB_DFU_APP_SIZE (0x8000U)
// Determines whether to support check CRC of full image after downloading
#define BL_FEATURE_USB_DFU_CRC_CHECK (0)
// Determines whether to boot image directly after downloading
#define BL_FEATURE_USB_DFU_DIRECT_BOOT (0)
//@}

//==============================================================================
//! @name Misc configuration macros
//@{
#define BL_FEATURE_MULTICORE (0)
#define BL_FEATURE_MULTI_SRAM_SECTIONS (0)

#if defined(BL_FEATURE_MULTI_SRAM_SECTIONS) && defined(BL_FEATURE_MULTICORE)
#define BL_FEATURE_CORE0_ITCM (1)
#define BL_FEATURE_CORE0_DTCM (1)
#define BL_FEATURE_CORE1_FLASH (1)
#define BL_FEATURE_CORE1_TCM (1)
#endif

#define BL_FEATURE_PROPERTY_RAM_REGION_COUNT (4)
#define BL_FEATURE_PROPERTY_RESERVED_REGION_COUNT (4)

//@}

//==============================================================================
//! @name ROM Utilites
//@{
#define BL_FETAURE_LOG_ENABLE (0)

//@}


//==============================================================================
//! @name Enable watch dog
//@{
#define BL_FEATURE_ENABLE_WATCHDOG (0)

//@}

#endif // __BOOTLOADER_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
