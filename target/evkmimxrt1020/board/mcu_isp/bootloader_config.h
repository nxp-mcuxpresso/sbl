/*
 * The Clear BSD License
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

// Determines whether to support isp boot via peripherals.
#define BL_FEATURE_ISP_BOOT (1)

//! @name Peripheral configuration macros
//@{

#define BL_FEATURE_ROM_UART_PORT (1)

// UART ports
#if !defined(BL_CONFIG_LPUART_1)
#define BL_CONFIG_LPUART_1 (BL_FEATURE_ROM_UART_PORT)
#define BL_FEATURE_UART_RX_PULLUP (1)
#endif

#define BL_CONFIG_LPUART (BL_CONFIG_LPUART_1)

#define BL_FEATURE_UID_IN_FUSE (1)

#define BL_FEATURE_GEN_KEYBLOB (0)
#define BL_FEATURE_KEYBLOB_BLOB_KEY_MAX_SIZE (16)

// USB HS port
#if !defined(BL_CONFIG_HS_USB_HID)
#define BL_CONFIG_HS_USB_HID (1) // i.MX RT Series only supoort HS USB
#endif
//@}

#if !defined(BL_TARGET_FLASH) && !defined(BL_TARGET_RAM)
#define BL_TARGET_FLASH (0)
#endif

// Internal Flash features
#define BL_FEATURE_HAS_NO_INTERNAL_FLASH \
    !(FSL_FEATURE_SOC_FTFA_COUNT || FSL_FEATURE_SOC_FTFE_COUNT || FSL_FEATURE_SOC_FTFL_COUNT)
#if BL_FEATURE_HAS_NO_INTERNAL_FLASH && BL_TARGET_FLASH
#error "No Flash available for Flash bootloader"
#endif

#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#if defined(BL_TARGET_RAM)
#define BL_FEATURE_FLASH_SECURITY (0)
#else
#define BL_FEATURE_FLASH_SECURITY (1)
#endif
#define BL_FEATURE_ERASEALL_UNSECURE (0)
#define BL_FEATURE_FLASH_VERIFY_DISABLE (0)
#endif // !BL_FEATURE_HAS_NO_INTERNAL_FLASH

#define BL_FEATURE_MIN_PROFILE (1)
#define BL_FEATURE_READ_MEMORY (1)
#define BL_FEATURE_FILL_MEMORY (1)

#if !defined(BL_TARGET_RAM)
#define BL_FEATURE_CRC_CHECK (0)
#endif

#define BL_FEATURE_UART_AUTOBAUD_IRQ (1)

#define BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE (1)

#define BL_FEATURE_FLEXSPI_NOR_MODULE (1)

#if BL_FEATURE_FLEXSPI_NOR_MODULE
#define BL_FEATURE_FLEXSPI_NOR_MODULE_PERIPHERAL_INSTANCE (0)
#define BL_FEATURE_FLEXSPI_NOR_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_FLEXSPI_ENCRYPT_PROGRAMMING (0)
#define BL_PROT_REGION_BLOCK_OFFSET(i) (0x400u * (i + 1))
#define BL_FLEXSPI_AMBA_BASE (0x60000000u)
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE

#define BL_FEATURE_SEMC_NOR_MODULE (0)

// Memory expansion features
#define BL_FEATURE_EXPAND_MEMORY (1)

#define BL_FEATURE_SPINAND_MODULE (0)

#if BL_FEATURE_SPINAND_MODULE
#if (!BL_FEATURE_EXPAND_MEMORY) && (BL_FEATURE_SPINAND_MODULE)
#error "BL_FEATURE_EXPAND_MEMORY" must be enabled to enable the SPI NAND feature.
#endif
#define BL_FEATURE_SPINAND_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_SPINAND_MODULE_PERIPHERAL_FLEXSPI (1)
#define BL_FEATURE_SPINAND_MODULE_PERIPHERAL_INSTANCE (0)
#endif // BL_FEATURE_SPINAND_MODULE

#define BL_FEATURE_MMC_MODULE (0)
#if BL_FEATURE_MMC_MODULE
#define BL_FEATURE_MMC_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_MMC_MODULE_PERIPHERAL_INSTANCE (2)
#define BL_FEATURE_MMC_MODULE_ENABLE_PERMANENT_CONFIG (0)
#endif // BL_FEATURE_MMC_MODULE

#define BL_FEATURE_SD_MODULE (0)
#if BL_FEATURE_SD_MODULE
#define BL_FEATURE_SD_MODULE_ERASE_VERIFY (1)
#define BL_FEATURE_SD_MODULE_PERIPHERAL_INSTANCE (1)
#endif // BL_FEATURE_SD_MODULE

#define BL_FEATURE_SEMC_NAND_MODULE (0)

#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE || BL_FEATURE_EMMC_MODULE || BL_FEATURE_SD_MODULE || \
    BL_FEATURE_SEMC_NAND_MODULE
#if !BL_FEATURE_EXPAND_MEMORY
#if defined(BL_FEATURE_EXPAND_MEMORY)
#undef BL_FEATURE_EXPAND_MEMORY
#endif
#define BL_FEATURE_EXPAND_MEMORY (1)
#warning "BL_FEATURE_EXPAND_MEMORY" is enabled automatically to support none-XIP memory devices.
#endif // #if !BL_FEATURE_EXPAND_MEMORY
#define BL_FEATURE_EXTERNAL_MEMORY_PROPERTY (1)
#endif

#define BL_FEATURE_SPI_NOR_EEPROM_MODULE (0)
#if BL_FEATURE_SPI_NOR_EEPROM_MODULE
#define BL_FEATURE_SPI_NOR_EEPROM_MODULE_ERASE_VERIFY (1)
#endif // BL_FEATURE_SPI_NOR_EEPROM_MODULE

//#define BL_FEATURE_OCOTP_MODULE (FSL_FEATURE_SOC_OCOTP_COUNT)
#define BL_FEATURE_OCOTP_MODULE (0)

#define BL_FEATURE_EXPAND_PACKET_SIZE (1)
#define BL_EXPANDED_FRAMING_PACKET_SIZE (512)
// Make sure that BL_EXPANDED_USB_HID_PACKET_SIZE < 1018
#define BL_EXPANDED_USB_HID_PACKET_SIZE (1017)

// Bootloader peripheral detection default timeout in milliseconds
// After coming out of reset the bootloader will spin in a peripheral detection
// loop for this amount of time. A zero value means no time out.
#if DEBUG
#define BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT 0
#else
#define BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT 5000
#endif // DEBUG

// The bootloader will check this address for the application vector table upon startup.
#if !defined(BL_APP_VECTOR_TABLE_ADDRESS)
#define BL_APP_VECTOR_TABLE_ADDRESS 0xa000
#endif

#endif // __BOOTLOADER_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
