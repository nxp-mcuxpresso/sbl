/*
 * Copyright 2019-2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BOOTLOADER_CONFIG_H__
#define __BOOTLOADER_CONFIG_H__

////////////////////////////////////////////////////////////////////////////////
// Bootloader Configuration Definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Boot Configurations
////////////////////////////////////////////////////////////////////////////////
// Determines whether to support isp boot via peripherals.
#define BL_FEATURE_ISP_BOOT (1)

////////////////////////////////////////////////////////////////////////////////
// Peripheral Configurations
////////////////////////////////////////////////////////////////////////////////
#define BL_FEATURE_ROM_UART_PORT (1)

// UART ports
#if !defined(BL_CONFIG_LPUART_1)
#define BL_CONFIG_LPUART_1 (BL_FEATURE_ROM_UART_PORT)
#define BL_FEATURE_UART_RX_PULLUP (1)
#endif

#define BL_CONFIG_LPUART (BL_CONFIG_LPUART_1)

#define BL_FEATURE_UART_RX_PULLUP (1)
#define BL_FEATURE_UART_AUTOBAUD_IRQ (1)

// SAI/I2S ports
#define BL_FEATURE_ROM_SAI_PORT (0)

#if !defined(BL_CONFIG_SAI_1)
#define BL_CONFIG_SAI_1 (BL_FEATURE_ROM_SAI_PORT)
#endif

#define BL_CONFIG_SAI (BL_CONFIG_SAI_1)

// USB HS port
#if !defined(BL_CONFIG_HS_USB_HID) && !defined(BL_TARGET_FPGA)
#define BL_CONFIG_HS_USB_HID (1) // i.MX RT Series only supoort HS USB
#else
#define BL_CONFIG_HS_USB_HID (0)
#endif

// SDIO SLAVE port
#if !defined(BL_CONFIG_SDIO_SLAVE_0)
#define BL_CONFIG_SDIO_SLAVE_0 (0)
#endif

#define BL_CONFIG_SDIO_SLAVE (BL_CONFIG_SDIO_SLAVE_0)

////////////////////////////////////////////////////////////////////////////////
// Bootloader Feature Configurations
////////////////////////////////////////////////////////////////////////////////
#if !defined(BL_TARGET_FLASH) && !defined(BL_TARGET_RAM)
#define BL_TARGET_FLASH (0)
#endif

#define BL_FEATURE_MIN_PROFILE (1)
#define BL_FEATURE_READ_MEMORY (1)
#define BL_FEATURE_FILL_MEMORY (1)

#if !defined(BL_TARGET_RAM)
#define BL_FEATURE_CRC_CHECK (0)
#endif

// Bootloader peripheral detection default timeout in milliseconds
// After coming out of reset the bootloader will spin in a peripheral detection
// loop for this amount of time. A zero value means no time out.
#if DEBUG
#define BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT 0
#else
#define BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT 5000
#endif // DEBUG

// Determines whetehr to support generate-key-blob command.
#define BL_FEATURE_GEN_KEYBLOB (0)
#define BL_FEATURE_KEYBLOB_USE_SECURE_MEMORY (1)
#define BL_FEATURE_SECURE_MEMORY_BASE (0x00280000u)
// DID for CAAM job ring to generate key blob.
// Note: must have the same value with HAB. otherwise the blob key encryption key will be changed.
#define BL_FEATURE_JOBRING_DID (0)
// Blob Key size in bytes
// (for CAAM, always use AES-CCM-256bit to encrypted the Data Key.)
// (for DCP, implementation defined.)
#define BL_FEATURE_KEYBLOB_BLOB_KEY_MAX_SIZE (32)
// Data Key size in bytes
// for CAAM, data key size can be 16(AES-128bit), 24(AES-192bit), 32(AES-256bit)
// for DCP, data key size can only be 16(AES-128bit)
#define BL_FEATURE_KEYBLOB_DATA_KEY_MAX_SIZE (32)
// Key Blob Header: 8 bytes;
// Encrypted Blob Key: BL_FEATURE_KEYBLOB_BLOB_KEY_SIZE;
//                     (always equals to Blob Key plaintext)
// Encrypted Data Key: BL_FEATURE_KEYBLOB_DATA_KEY_MAX_SIZE_BYTE bytes;
//                     (always equals to Data Key plaintext)
// Message Authentication Code(MAC): 16 bytes.
//                                   (For CAAM, always use MAC16)
#define BL_FEATURE_KEYBLOB_BLOB_MAX_SIZE \
    (8 + BL_FEATURE_KEYBLOB_BLOB_KEY_MAX_SIZE + BL_FEATURE_KEYBLOB_DATA_KEY_MAX_SIZE + 16)

// Determine the hardware engine PUF is used for key store generation.
#define BL_FEATURE_KEY_STORE_PUF (0)
// Determine whether to support key store generation.
#define BL_FEATURE_KEY_STORE (BL_FEATURE_KEY_STORE_PUF)
// Determine the maximum key size in bytes.
#define BL_FEATURE_KEY_STORE_MAX_KEY_SIZE (16)

// Determines whether to support key-provisioning command.
// Note: key-provisioning command must be enabled if key store is supported.
#define BL_FEATURE_KEY_PROVISIONING (BL_FEATURE_KEY_STORE)

////////////////////////////////////////////////////////////////////////////////
// Internal Memory Module Configurations
////////////////////////////////////////////////////////////////////////////////
#define BL_FEATURE_HAS_INTERNAL_FLASH (0)

#if BL_FEATURE_HAS_INTERNAL_FLASH
#if defined(BL_TARGET_RAM)
#define BL_FEATURE_FLASH_SECURITY (0)
#else
#define BL_FEATURE_FLASH_SECURITY (1)
#endif
#define BL_FEATURE_ERASEALL_UNSECURE (0)
#define BL_FEATURE_FLASH_VERIFY_DISABLE (0)
#endif // BL_FEATURE_HAS_INTERNAL_FLASH

#define BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE (1)

////////////////////////////////////////////////////////////////////////////////
// XIP External Memory Module Configurations
////////////////////////////////////////////////////////////////////////////////
#define BL_FEATURE_FLEXSPI_NOR_MODULE (1)

#if BL_FEATURE_FLEXSPI_NOR_MODULE
#define BL_FEATURE_FLEXSPI_NOR_MODULE_PERIPHERAL_INSTANCE (1)
//#define BL_FEATURE_FLEXSPI_NOR_MODULE_PERIPHERAL_INSTANCE (2)
#define BL_FEATURE_FLEXSPI_NOR_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_FLEXSPI_ENCRYPT_PROGRAMMING (0)
#define BL_PROT_REGION_BLOCK_OFFSET(i) (0x400u * (i + 1))
#define BL_FLEXSPI_AMBA_BASE (0x30000000u)
//#define BL_FLEXSPI_AMBA_BASE (0x60000000u)
#define BL_FEATURE_FLEXSPI_ALIAS_AREA (1)
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE
#define FLEXSPI_NOR_FEATURE_ENABLE_PROGRAMMING (BL_FEATURE_MEM_WRITE_ENABLE)

#define BL_FEATURE_SEMC_NOR_MODULE (0)

////////////////////////////////////////////////////////////////////////////////
// Non-XIP External Memory Module Configurations
////////////////////////////////////////////////////////////////////////////////
#define BL_FEATURE_EXPAND_MEMORY (1)

#define BL_FEATURE_SPINAND_MODULE (0)

#if BL_FEATURE_SPINAND_MODULE
#if (!BL_FEATURE_EXPAND_MEMORY) && (BL_FEATURE_SPINAND_MODULE)
#error "BL_FEATURE_EXPAND_MEMORY" must be enabled to enable the SPI NAND feature.
#endif
#define BL_FEATURE_SPINAND_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_SPINAND_MODULE_PERIPHERAL_FLEXSPI (1)
#define BL_FEATURE_SPINAND_MODULE_PERIPHERAL_INSTANCE (1)
#endif // BL_FEATURE_SPINAND_MODULE

#define BL_FEATURE_MMC_MODULE (0)

#if BL_FEATURE_MMC_MODULE
#define BL_FEATURE_MMC_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_MMC_MODULE_ENABLE_PERMANENT_CONFIG (0)
#endif // BL_FEATURE_MMC_MODULE

#define BL_FEATURE_SD_MODULE (0)

#if BL_FEATURE_SD_MODULE
#define BL_FEATURE_SD_MODULE_ERASE_VERIFY (1)
#endif // BL_FEATURE_SD_MODULE

#define BL_FEATURE_SEMC_NAND_MODULE (0)

#if BL_FEATURE_EXPAND_MEMORY
#define BL_FEATURE_EXTERNAL_MEMORY_PROPERTY (1)
#endif // BL_FEATURE_EXPAND_MEMORY

#define BL_FEATURE_SPI_NOR_EEPROM_MODULE (0)
#ifndef BL_TARGET_FPGA
#define BL_FEATURE_SPI_PERIPHERAL_INSTANCE (1)
#else
#define BL_FEATURE_SPI_PERIPHERAL_INSTANCE (3)
#endif
#if BL_FEATURE_SPI_NOR_EEPROM_MODULE
#define BL_FEATURE_SPI_NOR_EEPROM_MODULE_ERASE_VERIFY (1)
#endif // BL_FEATURE_SPI_NOR_EEPROM_MODULE

#define BL_FEATURE_OCOTP_MODULE (0)

////////////////////////////////////////////////////////////////////////////////
// Protocal Configurations
////////////////////////////////////////////////////////////////////////////////
#define BL_FEATURE_EXPAND_PACKET_SIZE (1)

#define BL_EXPANDED_FRAMING_PACKET_SIZE (512)

// Make sure that BL_EXPANDED_USB_HID_PACKET_SIZE < 1020
#define BL_EXPANDED_USB_HID_PACKET_SIZE (1012)

// Make sure that BL_EXPAND_SDIO_SLAVE_PACKET_SIZE + sizeof(framing_data_packet_t)(equals to 6) is power of 2.
#define BL_EXPAND_SDIO_SLAVE_PACKET_SIZE (1018)

////////////////////////////////////////////////////////////////////////////////
// Un-Categoried Configurations
////////////////////////////////////////////////////////////////////////////////
#define BL_FETAURE_USE_STD_EXCEPTION_HANDLER (0)

#endif // __BOOTLOADER_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
