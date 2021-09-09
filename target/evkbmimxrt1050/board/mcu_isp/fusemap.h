/*
 * The Clear BSD License
 * Copyright 2017-2021 NXP
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __FUSEMAP_H__
#define __FUSEMAP_H__

#include "fsl_device_registers.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define FUSE_BANK0_OFFSET 0x400
#define HW_FUSE_REG_ADDR(n) (OCOTP_BASE + FUSE_BANK0_OFFSET + ((n)*0x10))
#define HW_OCOTP_REG_RD(n) (*(volatile uint32_t *)HW_FUSE_REG_ADDR(n))

/* ======================== Infinite loop ================================= */
#define ROM_OCOTP_INFINITE_LOOP_SHIFT 11
#define ROM_OCOTP_INFINITE_LOOP_MASK (1 << ROM_OCOTP_INFINITE_LOOP_SHIFT)
#define ROM_OCOTP_INFINITE_LOOP_VALUE() ((SRC->SBMR1 & ROM_OCOTP_INFINITE_LOOP_MASK) >> ROM_OCOTP_INFINITE_LOOP_SHIFT)

/* ======================== Secure Code Protection ================================= */
#define ROM_OCOTP_SEC_CODE_PROTECTION_SHIFT 7
#define ROM_OCOTP_SEC_CODE_PROTECTION_MASK (1 << ROM_OCOTP_SEC_CODE_PROTECTION_SHIFT)
#define ROM_OCOTP_SEC_CODE_PROTECTION_VALUE() \
    ((HW_OCOTP_REG_RD(0x06) & ROM_OCOTP_SEC_CODE_PROTECTION_MASK) >> ROM_OCOTP_SEC_CODE_PROTECTION_SHIFT)

/* ======================== FlexSPI NOR Boot ================================= */
/* Hold_Time */
#define ROM_OCOTP_HOLD_TIME_MASK 0x0000000C
#define ROM_OCOTP_HOLD_TIME_SHIFT ((uint8_t)2)
#define ROM_OCOTP_HOLD_TIME_VALUE() ((SRC->SBMR1 & ROM_OCOTP_HOLD_TIME_MASK) >> ROM_OCOTP_HOLD_TIME_SHIFT)

/* Encrypted XIP */
#define ROM_OCOTP_ENCRYPT_XIP_MASK 0x00000002
#define ROM_OCOTP_ENCRYPT_XIP_SHIFT 0x01
#define ROM_OCOTP_ENCRYPT_XIP_VALAUE() ((SRC->SBMR1 & ROM_OCOTP_ENCRYPT_XIP_MASK) >> ROM_OCOTP_ENCRYPT_XIP_SHIFT)

/* Flash Type */
#define ROM_OCOTP_FLASH_TYPE_MASK 0x00000700
#define ROM_OCOTP_FLASH_TYPE_SHIFT ((uint8_t)8)
#define ROM_OCOTP_FLASH_TYPE_VALUE() ((SRC->SBMR1 & ROM_OCOTP_FLASH_TYPE_MASK) >> ROM_OCOTP_FLASH_TYPE_SHIFT)

/* Delay-Cell_Num */
#define ROM_OCOTP_DELAY_CELL_NUM_MASK 0x7F000000
#define ROM_OCOTP_DELAY_CELL_NUM_SHIFT ((uint8_t)24)
#define ROM_OCOTP_DELAY_CELL_NUM_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_DELAY_CELL_NUM_MASK) >> ROM_OCOTP_DELAY_CELL_NUM_SHIFT)

/* QSPI 2ND pinmux */
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT 20U
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_MASK (1U << ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT)
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_VALUE() \
    ((HW_OCOTP_REG_RD(4) & ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_MASK) >> ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT)

/* ====================== FlexSPI NAND Boot ================================= */
/* Safe Frequency */
#define ROM_OCOTP_SAFE_FREQ_MASK 0x00000020
#define ROM_OCOTP_SAFE_FREQ_SHIFT ((uint8_t)5)
#define ROM_OCOTP_SAFE_FREQ_VALUE() ((SRC->SBMR1 & ROM_OCOTP_SAFE_FREQ_MASK) >> ROM_OCOTP_SAFE_FREQ_SHIFT)

/* COL_Address_width */
#define ROM_OCOTP_COL_ADDRESS_WIDTH_MASK 0x00000010
#define ROM_OCOTP_COL_ADDRESS_WIDTH_SHIFT ((uint8_t)4)
#define ROM_OCOTP_COL_ADDRESS_WIDTH_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_COL_ADDRESS_WIDTH_MASK) >> ROM_OCOTP_COL_ADDRESS_WIDTH_SHIFT)

/* Boot_Search_Stride */
#define ROM_OCOTP_SPI_NAND_HOLD_TIME_MASK 0x0000000c
#define ROM_OCOTP_SPI_NAND_HOLD_TIME_SHIFT ((uint8_t)2)
#define ROM_OCOTP_SPI_NAND_HOLD_TIME_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SPI_NAND_HOLD_TIME_MASK) >> ROM_OCOTP_SPI_NAND_HOLD_TIME_SHIFT)

/* Boot_Search_Stride */
#define ROM_OCOTP_BOOT_SEARCH_STRIDE_MASK 0x00000003
#define ROM_OCOTP_BOOT_SEARCH_STRIDE_SHIFT ((uint8_t)0)
#define ROM_OCOTP_BOOT_SEARCH_STRIDE_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_BOOT_SEARCH_STRIDE_MASK) >> ROM_OCOTP_BOOT_SEARCH_STRIDE_SHIFT)

/* Boot_Search_Count */
#define ROM_OCOTP_BOOT_SEARCH_COUNT_MASK 0x00000100
#define ROM_OCOTP_BOOT_SEARCH_COUNT_SHIFT ((uint8_t)8)
#define ROM_OCOTP_BOOT_SEARCH_COUNT_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_BOOT_SEARCH_COUNT_MASK) >> ROM_OCOTP_BOOT_SEARCH_COUNT_SHIFT)

/* CS_Interval */
#define ROM_OCOTP_CS_INTERVAL_MASK 0x00000600
#define ROM_OCOTP_CS_INTERVAL_SHIFT ((uint8_t)9)
#define ROM_OCOTP_CS_INTERVAL_VALUE() ((SRC->SBMR1 & ROM_OCOTP_CS_INTERVAL_MASK) >> ROM_OCOTP_CS_INTERVAL_SHIFT)

/* SPI NAND Boot - override Busy Offset */
#define ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_OVERRIDE_MASK 0x00000080
#define ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_OVERRIDE_SHIFT ((uint8_t)7)
#define ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_OVERRIDE_VALUE()                        \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_OVERRIDE_MASK) >> \
     ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_OVERRIDE_SHIFT)

/* SPI NAND Boot - Busy Bit Offset */
#define ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_MASK 0x00003F00
#define ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_SHIFT ((uint8_t)8)
#define ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_VALUE()                        \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_MASK) >> \
     ROM_OCOTP_SPI_NAND_BOOT_BUSY_BIT_OFFSET_SHIFT)

/* Bypass_ECC_Read */
#define ROM_OCOTP_BYPASS_ECC_READ_MASK 0x00004000
#define ROM_OCOTP_BYPASS_ECC_READ_SHIFT ((uint8_t)14)
#define ROM_OCOTP_BYPASS_ECC_READ_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_BYPASS_ECC_READ_MASK) >> ROM_OCOTP_BYPASS_ECC_READ_SHIFT)

/* Bypass_Read_Status */
#define ROM_OCOTP_BYPASS_READ_STATUS_MASK 0x00008000
#define ROM_OCOTP_BYPASS_READ_STATUS_SHIFT ((uint8_t)15)
#define ROM_OCOTP_BYPASS_READ_STATUS_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_BYPASS_READ_STATUS_MASK) >> ROM_OCOTP_BYPASS_READ_STATUS_SHIFT)

/* SPI NAND BOOT - Page read time */
#define ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_TIME_MASK 0x00003F00
#define ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_TIME_SHIFT ((uint8_t)8)
#define ROM_OCOTP_SPI_NAND_BOOT_PAGE_FRD_TIME_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_TIME_MASK) >> ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_TIME_SHIFT)

/* SPI NAND BOOT - page read cmd */
#define ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_CMD_MASK 0x00FF0000
#define ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_CMD_SHIFT ((uint8_t)16)
#define ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_CMD_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_CMD_MASK) >> ROM_OCOTP_SPI_NAND_BOOT_PAGE_RD_CMD_SHIFT)

/* SPI NAND BOOT - cache read cmd */
#define ROM_OCOTP_SPI_NAND_BOOT_CACHE_RD_CMD_MASK 0xFF000000
#define ROM_OCOTP_SPI_NAND_BOOT_CACHE_RD_CMD_SHIFT ((uint8_t)24)
#define ROM_OCOTP_SPI_NAND_BOOT_CACHE_RD_CMD_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SPI_NAND_BOOT_CACHE_RD_CMD_MASK) >> ROM_OCOTP_SPI_NAND_BOOT_CACHE_RD_CMD_SHIFT)

/* ======================== SEMC NOR Boot ================================= */
/* Clock Frequency */
#define ROM_OCOTP_SEMC_NOR_CLK_FREQ_MASK 0x00000007
#define ROM_OCOTP_SEMC_NOR_CLK_FREQ_SHIFT ((uint8_t)0)
#define ROM_OCOTP_SEMC_NOR_CLK_FREQ_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NOR_CLK_FREQ_MASK) >> ROM_OCOTP_SEMC_NOR_CLK_FREQ_SHIFT)

/* AC default timing speed */
#define ROM_OCOTP_SEMC_NOR_AC_TIMING_SPEED_MASK 0x00000008
#define ROM_OCOTP_SEMC_NOR_AC_TIMING_SPEED_SHIFT ((uint8_t)3)
#define ROM_OCOTP_SEMC_NOR_AC_TIMING_SPEED_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NOR_AC_TIMING_SPEED_MASK) >> ROM_OCOTP_SEMC_NOR_AC_TIMING_SPEED_SHIFT)

/* AC timing parameter */
#define ROM_OCOTP_SEMC_NOR_AC_TIMING_MASK 0x00000100
#define ROM_OCOTP_SEMC_NOR_AC_TIMING_SHIFT ((uint8_t)8)
#define ROM_OCOTP_SEMC_NOR_AC_TIMING_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NOR_AC_TIMING_MASK) >> ROM_OCOTP_SEMC_NOR_AC_TIMING_SHIFT)

/* Data Port Size */
#define ROM_OCOTP_SEMC_NOR_DATA_PORT_SIZE_MASK 0x00000200
#define ROM_OCOTP_SEMC_NOR_DATA_PORT_SIZE_SHIFT ((uint8_t)9)
#define ROM_OCOTP_SEMC_NOR_DATA_PORT_SIZE_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NOR_DATA_PORT_SIZE_MASK) >> ROM_OCOTP_SEMC_NOR_DATA_PORT_SIZE_SHIFT)

/* DQS pad mode */
#define ROM_OCOTP_SEMC_NOR_DQS_PAD_MODE_MASK 0x00000400
#define ROM_OCOTP_SEMC_NOR_DQS_PAD_MODE_SHIFT ((uint8_t)10)
#define ROM_OCOTP_SEMC_NOR_DQS_PAD_MODE_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NOR_DQS_PAD_MODE_MASK) >> ROM_OCOTP_SEMC_NOR_DQS_PAD_MODE_SHIFT)

/* PCS Selection */
#define ROM_OCOTP_SEMC_NOR_PCS_SELECTION_MASK 0x00000007
#define ROM_OCOTP_SEMC_NOR_PCS_SELECTION_SHIFT ((uint8_t)0)
#define ROM_OCOTP_SEMC_NOR_PCS_SELECTION_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_PCS_SELECTION_MASK) >> ROM_OCOTP_SEMC_NOR_PCS_SELECTION_SHIFT)

/* Address Port Size */
#define ROM_OCOTP_SEMC_NOR_ADDRESS_PORT_SIZE_MASK 0x00000038
#define ROM_OCOTP_SEMC_NOR_ADDRESS_PORT_SIZE_SHIFT ((uint8_t)3)
#define ROM_OCOTP_SEMC_NOR_ADDRESS_PORT_SIZE_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_ADDRESS_PORT_SIZE_MASK) >> ROM_OCOTP_SEMC_NOR_ADDRESS_PORT_SIZE_SHIFT)

/* ADV# Polarity */
#define ROM_OCOTP_SEMC_NOR_ADV_POLARITY_MASK 0x00000040
#define ROM_OCOTP_SEMC_NOR_ADV_POLARITY_SHIFT ((uint8_t)6)
#define ROM_OCOTP_SEMC_NOR_ADV_POLARITY_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_ADV_POLARITY_MASK) >> ROM_OCOTP_SEMC_NOR_ADV_POLARITY_SHIFT)

/* RDY Polarity */
#define ROM_OCOTP_SEMC_NOR_RDY_POLARITY_MASK 0x00000080
#define ROM_OCOTP_SEMC_NOR_RDY_POLARITY_SHIFT ((uint8_t)7)
#define ROM_OCOTP_SEMC_NOR_RDY_POLARITY_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_RDY_POLARITY_MASK) >> ROM_OCOTP_SEMC_NOR_RDY_POLARITY_SHIFT)

/* AC Timing - CES */
#define ROM_OCOTP_SEMC_NOR_AC_CES_MASK 0x00000300
#define ROM_OCOTP_SEMC_NOR_AC_CES_SHIFT ((uint8_t)8)
#define ROM_OCOTP_SEMC_NOR_AC_CES_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_AC_CES_MASK) >> ROM_OCOTP_SEMC_NOR_AC_CES_SHIFT)

/* AC Timing - CEH */
#define ROM_OCOTP_SEMC_NOR_AC_CEH_MASK 0x00000C00
#define ROM_OCOTP_SEMC_NOR_AC_CEH_SHIFT ((uint8_t)10)
#define ROM_OCOTP_SEMC_NOR_AC_CEH_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_AC_CEH_MASK) >> ROM_OCOTP_SEMC_NOR_AC_CEH_SHIFT)

/* AC Timing - CEITV */
#define ROM_OCOTP_SEMC_NOR_AC_CEITV_MASK 0x00003000
#define ROM_OCOTP_SEMC_NOR_AC_CEITV_SHIFT ((uint8_t)12)
#define ROM_OCOTP_SEMC_NOR_AC_CEITV_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_AC_CEITV_MASK) >> ROM_OCOTP_SEMC_NOR_AC_CEITV_SHIFT)

/* AC Timing - TA */
#define ROM_OCOTP_SEMC_NOR_AC_TA_MASK 0x0000C000
#define ROM_OCOTP_SEMC_NOR_AC_TA_SHIFT ((uint8_t)14)
#define ROM_OCOTP_SEMC_NOR_AC_TA_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_AC_TA_MASK) >> ROM_OCOTP_SEMC_NOR_AC_TA_SHIFT)

/* AC Timing - AS */
#define ROM_OCOTP_SEMC_NOR_AC_AS_MASK 0x000F0000
#define ROM_OCOTP_SEMC_NOR_AC_AS_SHIFT ((uint8_t)16)
#define ROM_OCOTP_SEMC_NOR_AC_AS_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_AC_AS_MASK) >> ROM_OCOTP_SEMC_NOR_AC_AS_SHIFT)

/* AC Timing - AH */
#define ROM_OCOTP_SEMC_NOR_AC_AH_MASK 0x00F00000
#define ROM_OCOTP_SEMC_NOR_AC_AH_SHIFT ((uint8_t)20)
#define ROM_OCOTP_SEMC_NOR_AC_AH_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_AC_AH_MASK) >> ROM_OCOTP_SEMC_NOR_AC_AH_SHIFT)

/* AC Timing - REL */
#define ROM_OCOTP_SEMC_NOR_AC_REL_MASK 0x0F000000
#define ROM_OCOTP_SEMC_NOR_AC_REL_SHIFT ((uint8_t)24)
#define ROM_OCOTP_SEMC_NOR_AC_REL_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_AC_REL_MASK) >> ROM_OCOTP_SEMC_NOR_AC_REL_SHIFT)

/* AC Timing - REH */
#define ROM_OCOTP_SEMC_NOR_AC_REH_MASK 0xF0000000
#define ROM_OCOTP_SEMC_NOR_AC_REH_SHIFT ((uint8_t)28)
#define ROM_OCOTP_SEMC_NOR_AC_REH_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NOR_AC_REH_MASK) >> ROM_OCOTP_SEMC_NOR_AC_REH_SHIFT)

/* ======================== SEMC NAND Boot ================================= */
/* Boot_Search_Count */
#define ROM_OCOTP_SEMC_NAND_SEARCH_COUNT_MASK 0x00000001
#define ROM_OCOTP_SEMC_NAND_SEARCH_COUNT_SHIFT ((uint8_t)0)
#define ROM_OCOTP_SEMC_NAND_SEARCH_COUNT_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NAND_SEARCH_COUNT_MASK) >> ROM_OCOTP_SEMC_NAND_SEARCH_COUNT_SHIFT)

/* Boot_Search_Stride  */
#define ROM_OCOTP_SEMC_NAND_SEARCH_STRIDE_MASK 0x0000001E
#define ROM_OCOTP_SEMC_NAND_SEARCH_STRIDE_SHIFT ((uint8_t)1)
#define ROM_OCOTP_SEMC_NAND_SEARCH_STRIDE_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NAND_SEARCH_STRIDE_MASK) >> ROM_OCOTP_SEMC_NAND_SEARCH_STRIDE_SHIFT)

/* ONFI compliant */
#define ROM_OCOTP_SEMC_NAND_ONFI_COMPLIANT_MASK 0x00000100
#define ROM_OCOTP_SEMC_NAND_ONFI_COMPLIANT_SHIFT ((uint8_t)8)
#define ROM_OCOTP_SEMC_NAND_ONFI_COMPLIANT_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NAND_ONFI_COMPLIANT_MASK) >> ROM_OCOTP_SEMC_NAND_ONFI_COMPLIANT_SHIFT)

/* ECC Type */
#define ROM_OCOTP_SEMC_NAND_ECC_TYPE_MASK 0x00000200
#define ROM_OCOTP_SEMC_NAND_ECC_TYPE_SHIFT ((uint8_t)9)
#define ROM_OCOTP_SEMC_NAND_ECC_TYPE_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NAND_ECC_TYPE_MASK) >> ROM_OCOTP_SEMC_NAND_ECC_TYPE_SHIFT)

/* DQS pad mode */
#define ROM_OCOTP_SEMC_NAND_DQS_PAD_MODE_MASK 0x00000400
#define ROM_OCOTP_SEMC_NAND_DQS_PAD_MODE_SHIFT ((uint8_t)10)
#define ROM_OCOTP_SEMC_NAND_DQS_PAD_MODE_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SEMC_NAND_DQS_PAD_MODE_MASK) >> ROM_OCOTP_SEMC_NAND_DQS_PAD_MODE_SHIFT)

/* PCS Selection */
#define ROM_OCOTP_SEMC_NAND_PCS_SELECTION_MASK 0x00000007
#define ROM_OCOTP_SEMC_NAND_PCS_SELECTION_SHIFT ((uint8_t)0)
#define ROM_OCOTP_SEMC_NAND_PCS_SELECTION_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_PCS_SELECTION_MASK) >> ROM_OCOTP_SEMC_NAND_PCS_SELECTION_SHIFT)

/* I/O Port_width */
#define ROM_OCOTP_SEMC_NAND_IO_PORT_WIDTH_MASK 0x00000008
#define ROM_OCOTP_SEMC_NAND_IO_PORT_WIDTH_SHIFT ((uint8_t)3)
#define ROM_OCOTP_SEMC_NAND_IO_PORT_WIDTH_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_IO_PORT_WIDTH_MASK) >> ROM_OCOTP_SEMC_NAND_IO_PORT_WIDTH_SHIFT)

/* EDO mode */
#define ROM_OCOTP_SEMC_NAND_EDO_MODE_MASK 0x00000010
#define ROM_OCOTP_SEMC_NAND_EDO_MODE_SHIFT ((uint8_t)4)
#define ROM_OCOTP_SEMC_NAND_EDO_MODE_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_EDO_MODE_MASK) >> ROM_OCOTP_SEMC_NAND_EDO_MODE_SHIFT)

/* RDY Polarity */
#define ROM_OCOTP_SEMC_NAND_RDY_POLARITY_MASK 0x00000020
#define ROM_OCOTP_SEMC_NAND_RDY_POLARITY_SHIFT ((uint8_t)5)
#define ROM_OCOTP_SEMC_NAND_RDY_POLARITY_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_RDY_POLARITY_MASK) >> ROM_OCOTP_SEMC_NAND_RDY_POLARITY_SHIFT)

/* Ready Check type */
#define ROM_OCOTP_SEMC_NAND_READY_CHECK_TYPE_MASK 0x00000040
#define ROM_OCOTP_SEMC_NAND_READY_CHECK_TYPE_SHIFT ((uint8_t)6)
#define ROM_OCOTP_SEMC_NAND_READY_CHECK_TYPE_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_READY_CHECK_TYPE_MASK) >> ROM_OCOTP_SEMC_NAND_READY_CHECK_TYPE_SHIFT)

/* Clock Frequency */
#define ROM_OCOTP_SEMC_NAND_CLK_FREQ_MASK 0x00000080
#define ROM_OCOTP_SEMC_NAND_CLK_FREQ_SHIFT ((uint8_t)7)
#define ROM_OCOTP_SEMC_NAND_CLK_FREQ_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_CLK_FREQ_MASK) >> ROM_OCOTP_SEMC_NAND_CLK_FREQ_SHIFT)

/* Row Column address mode */
#define ROM_OCOTP_SEMC_NAND_ROW_COL_ADDR_MODE_MASK 0x00000700
#define ROM_OCOTP_SEMC_NAND_ROW_COL_ADDR_MODE_SHIFT ((uint8_t)8)
#define ROM_OCOTP_SEMC_NAND_ROW_COL_ADDR_MODE_VALUE()                        \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_ROW_COL_ADDR_MODE_MASK) >> \
     ROM_OCOTP_SEMC_NAND_ROW_COL_ADDR_MODE_SHIFT)

/* COL_Address_width */
#define ROM_OCOTP_SEMC_NAND_COL_ADDRESS_WIDTH_MASK 0x00003800
#define ROM_OCOTP_SEMC_NAND_COL_ADDRESS_WIDTH_SHIFT ((uint8_t)11)
#define ROM_OCOTP_SEMC_NAND_COL_ADDRESS_WIDTH_VALUE()                        \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_COL_ADDRESS_WIDTH_MASK) >> \
     ROM_OCOTP_SEMC_NAND_COL_ADDRESS_WIDTH_SHIFT)

/* Status Command Type */
#define ROM_OCOTP_SEMC_NAND_STATUS_CMD_TYPE_MASK 0x00004000
#define ROM_OCOTP_SEMC_NAND_STATUS_CMD_TYPE_SHIFT ((uint8_t)14)
#define ROM_OCOTP_SEMC_NAND_STATUS_CMD_TYPE_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_STATUS_CMD_TYPE_MASK) >> ROM_OCOTP_SEMC_NAND_STATUS_CMD_TYPE_SHIFT)

/* Memory Access Command */
#define ROM_OCOTP_SEMC_NAND_ACCESS_COMMAND_MASK 0x00008000
#define ROM_OCOTP_SEMC_NAND_ACCESS_COMMAND_SHIFT ((uint8_t)15)
#define ROM_OCOTP_SEMC_NAND_ACCESS_COMMAND_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_ACCESS_COMMAND_MASK) >> ROM_OCOTP_SEMC_NAND_ACCESS_COMMAND_SHIFT)

/* Pages in block */
#define ROM_OCOTP_SEMC_NAND_PAGES_IN_BLOCK_MASK 0x00070000
#define ROM_OCOTP_SEMC_NAND_PAGES_IN_BLOCK_SHIFT ((uint8_t)16)
#define ROM_OCOTP_SEMC_NAND_PAGES_IN_BLOCK_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_PAGES_IN_BLOCK_MASK) >> ROM_OCOTP_SEMC_NAND_PAGES_IN_BLOCK_SHIFT)

/* Device ECC initial status */
#define ROM_OCOTP_SEMC_NAND_DEVICE_ECC_STATUS_MASK 0x01000000
#define ROM_OCOTP_SEMC_NAND_DEVICE_ECC_STATUS_SHIFT ((uint8_t)24)
#define ROM_OCOTP_SEMC_NAND_DEVICE_ECC_STATUS_VALUE()                        \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_DEVICE_ECC_STATUS_MASK) >> \
     ROM_OCOTP_SEMC_NAND_DEVICE_ECC_STATUS_SHIFT)

/* ONFI Timing mode */
#define ROM_OCOTP_SEMC_NAND_TIMING_MODE_MASK 0x0E000000
#define ROM_OCOTP_SEMC_NAND_TIMING_MODE_SHIFT ((uint8_t)25)
#define ROM_OCOTP_SEMC_NAND_TIMING_MODE_VALUE() \
    ((HW_OCOTP_REG_RD(0x2E) & ROM_OCOTP_SEMC_NAND_TIMING_MODE_MASK) >> ROM_OCOTP_SEMC_NAND_TIMING_MODE_SHIFT)

/* ========================== Recovery boot ======================================= */
#define ROM_OCOTP_RECOVERY_SPI_EEPROM_BOOT_ENABLE_MASK 0x01000000
#define ROM_OCOTP_RECOVERY_SPI_EEPROM_BOOT_ENABLE_SHIFT 24
#define ROM_OCOTP_RECOVERY_SPI_EEPROM_BOOT_ENABLE_VALUE()                        \
    ((HW_OCOTP_REG_RD(0x2D) & ROM_OCOTP_RECOVERY_SPI_EEPROM_BOOT_ENABLE_MASK) >> \
     ROM_OCOTP_RECOVERY_SPI_EEPROM_BOOT_ENABLE_SHIFT)

/* SPI_INSTANCE */
#define ROM_OCOTP_SPI_INSTANCE_MASK 0x06000000
#define ROM_OCOTP_SPI_INSTANCE_SHIFT ((uint8_t)25)
#define ROM_OCOTP_SPI_INSTANCE_VALUE() \
    ((HW_OCOTP_REG_RD(0x2D) & ROM_OCOTP_SPI_INSTANCE_MASK) >> ROM_OCOTP_SPI_INSTANCE_SHIFT)

/* SPI_Memory_Speed */
#define ROM_OCOTP_SPI_MEMORY_SPEED_MASK 0x30000000
#define ROM_OCOTP_SPI_MEMORY_SPEED_SHIFT ((uint8_t)28)
#define ROM_OCOTP_SPI_MEMORY_SPEED_VALUE() \
    ((HW_OCOTP_REG_RD(0x2D) & ROM_OCOTP_SPI_MEMORY_SPEED_MASK) >> ROM_OCOTP_SPI_MEMORY_SPEED_SHIFT)

/* SPI_Memory_Address */
#define ROM_OCOTP_SPI_MEMORY_ADDRESS_MASK 0x08000000
#define ROM_OCOTP_SPI_MEMORY_ADDRESS_SHIFT ((uint8_t)27)
#define ROM_OCOTP_SPI_MEMORY_ADDRESS_VALUE() \
    ((HW_OCOTP_REG_RD(0x2D) & ROM_OCOTP_SPI_MEMORY_ADDRESS_MASK) >> ROM_OCOTP_SPI_MEMORY_ADDRESS_SHIFT)

/* ========================== LP boot ======================================= */
#define ROM_OCOTP_LPB_BOOT_MASK 0x00600000
#define ROM_OCOTP_LPB_BOOT_SHIFT 21
#define ROM_OCOTP_LPB_BOOT_VALUE() ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_LPB_BOOT_MASK) >> ROM_OCOTP_LPB_BOOT_SHIFT)
#define ROM_OCOTP_BOOT_FREQ_MASK 0x04
#define ROM_OCOTP_BOOT_FREQ_SHIFT 0x02
#define ROM_OCOTP_BOOT_FREQ_VALUE() ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_BOOT_FREQ_MASK) >> ROM_OCOTP_BOOT_FREQ_SHIFT)

/* ========================== Boot Failure Indicator Pin ====================== */
/* Enable_Boot_Failure_Indicator pin */
#define BOOT_FAIL_INDICATOR_ENABLE_MASK (0x00000080)
#define BOOT_FAIL_INDICATOR_ENABLE_SHIFT ((uint8_t)7)
#define BOOT_FAIL_INDICATOR_ENABLE_VALUE() \
    ((HW_OCOTP_REG_RD(45) & BOOT_FAIL_INDICATOR_ENABLE_MASK) >> BOOT_FAIL_INDICATOR_ENABLE_SHIFT)

#define BOOT_FAIL_INDICATOR_PIN_BIT4_MASK (0x00000400)
#define BOOT_FAIL_INDICATOR_PIN_BIT4_SHIFT ((uint8_t)10)
#define BOOT_FAIL_INDICATOR_PIN_BIT4_VAL \
    ((HW_OCOTP_REG_RD(7) & BOOT_FAIL_INDICATOR_PIN_BIT4_MASK) >> (BOOT_FAIL_INDICATOR_PIN_BIT4_SHIFT - 4))

#define BOOT_FAIL_INDICATOR_PIN_BIT3_0_MASK (0x000F0000)
#define BOOT_FAIL_INDICATOR_PIN_BIT3_0_SHIFT ((uint8_t)16)
#define BOOT_FAIL_INDICATOR_PIN_BIT3_0_VAL \
    ((HW_OCOTP_REG_RD(7) & BOOT_FAIL_INDICATOR_PIN_BIT3_0_MASK) >> BOOT_FAIL_INDICATOR_PIN_BIT3_0_SHIFT)

#define BOOT_FAIL_INDICATOR_PIN_VALUE() (BOOT_FAIL_INDICATOR_PIN_BIT4_VAL | BOOT_FAIL_INDICATOR_PIN_BIT3_0_VAL)

/* ========================== Secure/Encrypt boot ======================================= */
#define ROM_OCOTP_DCP_DISBALE_SHIFT (29)
#define ROM_OCOTP_DCP_DISABLE_MASK (1UL << ROM_OCOTP_DCP_DISBALE_SHIFT)
#define ROM_OCOTP_DCP_DISABLE_VALUE() ((HW_OCOTP_REG_RD(3) & ROM_OCOTP_DCP_DISABLE_MASK) >> ROM_OCOTP_DCP_DISBALE_SHIFT)
#define ROM_OCOTP_DCP_CRYPTO_DISBALE_SHIFT (31)
#define ROM_OCOTP_DCP_CRYPTO_DISABLE_MASK (1UL << ROM_OCOTP_DCP_CRYPTO_DISBALE_SHIFT)
#define ROM_OCOTP_DCP_CRYPTO_DISABLE_VALUE() \
    ((HW_OCOTP_REG_RD(3) & ROM_OCOTP_DCP_CRYPTO_DISABLE_MASK) >> ROM_OCOTP_DCP_CRYPTO_DISBALE_SHIFT)

#define ROM_OCOTP_BEE_DISABLE_SHIFT (27)
#define ROM_OCOTP_BEE_DISABLE_MASK (1UL << ROM_OCOTP_BEE_DISABLE_SHIFT)
#define ROM_OCOTP_BEE_DISABLE_VALUE() ((HW_OCOTP_REG_RD(3) & ROM_OCOTP_BEE_DISABLE_MASK) >> ROM_OCOTP_BEE_DISABLE_SHIFT)

#define ROM_OCOTP_BEE_KEY0_SEL_SHIFT (12)
#define ROM_OCOTP_BEE_KEY0_SEL_MASK (3UL << ROM_OCOTP_BEE_KEY0_SEL_SHIFT)
#define ROM_OCOTP_BEE_KEY0_SEL_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_BEE_KEY0_SEL_MASK) >> ROM_OCOTP_BEE_KEY0_SEL_SHIFT)
#define ROM_OCOTP_BEE_KEY1_SEL_SHIFT (14)
#define ROM_OCOTP_BEE_KEY1_SEL_MASK (3UL << ROM_OCOTP_BEE_KEY1_SEL_SHIFT)
#define ROM_OCOTP_BEE_KEY1_SEL_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_BEE_KEY1_SEL_MASK) >> ROM_OCOTP_BEE_KEY1_SEL_SHIFT)

/* SEC config0 fuse */
#define ROM_OCOTP_SEC_CONFIG0_MASK (0x00000002)
#define ROM_OCOTP_SEC_CONFIG0_SHIFT ((uint8_t)1)
#define ROM_OCOTP_SEC_CONFIG0_VALUE() ((HW_OCOTP_REG_RD(4) & ROM_OCOTP_SEC_CONFIG0_MASK) >> ROM_OCOTP_SEC_CONFIG0_SHIFT)

/* SEC_Config[1] */
#define ROM_OCOTP_SEC_CONFIG1_MASK 0x00000002
#define ROM_OCOTP_SEC_CONFIG1_SHIFT ((uint8_t)0)
#define ROM_OCOTP_SEC_CONFIG1_VALUE() ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_SEC_CONFIG1_MASK) >> ROM_OCOTP_SEC_CONFIG1_SHIFT)

#define ROM_OCOTP_SEC_CONFIG_VALUE() (ROM_OCOTP_SEC_CONFIG0_VALUE() | ROM_OCOTP_SEC_CONFIG1_VALUE())

/* ========================== MMC/SD boot(Common Part)======================================= */
/* Card Type selection, eMMC or SD. */
#define ROM_OCOTP_SDMMC_TYPE_SEL_SHIFT ((uint8_t)6)
#define ROM_OCOTP_SDMMC_TYPE_SEL_MASK ((uint32_t)(0x3 << ROM_OCOTP_SDMMC_TYPE_SEL_SHIFT))
#define ROM_OCOTP_SDMMC_TYPE_SEL_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SDMMC_TYPE_SEL_MASK) >> ROM_OCOTP_SDMMC_TYPE_SEL_SHIFT)

/* SD1 VOLTAGE SELECTION */
#define ROM_OCOTP_SD1_VOLTAGE_SELECTION_SHIFT ((uint8_t)8)
#define ROM_OCOTP_SD1_VOLTAGE_SELECTION_MASK ((uint32_t)(0x1 << ROM_OCOTP_SD1_VOLTAGE_SELECTION_SHIFT))
#define ROM_OCOTP_SD1_VOLTAGE_SELECTION_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SD1_VOLTAGE_SELECTION_MASK) >> ROM_OCOTP_SD1_VOLTAGE_SELECTION_SHIFT)

/* SD2 VOLTAGE SELECTION */
#define ROM_OCOTP_SD2_VOLTAGE_SELECTION_SHIFT ((uint8_t)5)
#define ROM_OCOTP_SD2_VOLTAGE_SELECTION_MASK ((uint32_t)(0x1 << ROM_OCOTP_SD2_VOLTAGE_SELECTION_SHIFT))
#define ROM_OCOTP_SD2_VOLTAGE_SELECTION_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_SD2_VOLTAGE_SELECTION_MASK) >> ROM_OCOTP_SD2_VOLTAGE_SELECTION_SHIFT)

/* SD Loopback_Clock_Source */
#define ROM_OCOTP_SDMMC_LOOPBACK_CLK_SOURCE_SHIFT ((uint8_t)2)
#define ROM_OCOTP_SDMMC_LOOPBACK_CLK_SOURCE_MASK ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_LOOPBACK_CLK_SOURCE_SHIFT))
#define ROM_OCOTP_SDMMC_LOOPBACK_CLK_SOURCE_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SDMMC_LOOPBACK_CLK_SOURCE_MASK) >> ROM_OCOTP_SDMMC_LOOPBACK_CLK_SOURCE_SHIFT)

/* SD/MMC/eMMC Fast_Boot */
#define ROM_OCOTP_SDMMC_FAST_BOOT_SHIFT ((uint8_t)0)
#define ROM_OCOTP_SDMMC_FAST_BOOT_MASK ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_FAST_BOOT_SHIFT))
#define ROM_OCOTP_SDMMC_FAST_BOOT_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SDMMC_FAST_BOOT_MASK) >> ROM_OCOTP_SDMMC_FAST_BOOT_SHIFT)

/* Power cycle enable */
#define ROM_OCOTP_SDMMC_POWER_CYCLE_ENABLE_SHIFT ((uint8_t)3)
#define ROM_OCOTP_SDMMC_POWER_CYCLE_ENABLE_MASK ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_POWER_CYCLE_ENABLE_SHIFT))
#define ROM_OCOTP_SDMMC_POWER_CYCLE_ENABLE_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SDMMC_POWER_CYCLE_ENABLE_MASK) >> ROM_OCOTP_SDMMC_POWER_CYCLE_ENABLE_SHIFT)

/* Power cycle selection */
#define ROM_OCOTP_SDMMC_PWR_CYCLE_SEL_SHIFT ((uint8_t)30)
#define ROM_OCOTP_SDMMC_PWR_CYCLE_SEL_MASK ((uint32_t)(0x3 << ROM_OCOTP_SDMMC_PWR_CYCLE_SEL_SHIFT))
#define ROM_OCOTP_SDMMC_PWR_CYCLE_SEL_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_SDMMC_PWR_CYCLE_SEL_MASK) >> ROM_OCOTP_SDMMC_PWR_CYCLE_SEL_SHIFT)

/* Power stable cycle selection */
#define ROM_OCOTP_SDMMC_PWR_STABLE_CYCLE_SEL_SHIFT ((uint8_t)29)
#define ROM_OCOTP_SDMMC_PWR_STABLE_CYCLE_SEL_MASK ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_PWR_STABLE_CYCLE_SEL_SHIFT))
#define ROM_OCOTP_SDMMC_PWR_STABLE_CYCLE_SEL_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_SDMMC_PWR_STABLE_CYCLE_SEL_MASK) >> ROM_OCOTP_SDMMC_PWR_STABLE_CYCLE_SEL_SHIFT)

/* DLL enable fuse */
#define ROM_OCOTP_SDMMC_DLL_ENABLE_SHIFT ((uint8_t)24)
#define ROM_OCOTP_SDMMC_DLL_ENABLE_MASK ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_DLL_ENABLE_SHIFT))
#define ROM_OCOTP_SDMMC_DLL_ENABLE_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_SDMMC_DLL_ENABLE_MASK) >> ROM_OCOTP_SDMMC_DLL_ENABLE_SHIFT)

/* DLL override enable for SD/eMMC */
#define ROM_OCOTP_SDMMC_DLL_OVERRIDE_ENABLE_SHIFT ((uint8_t)7)
#define ROM_OCOTP_SDMMC_DLL_OVERRIDE_ENABLE_MASK ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_DLL_OVERRIDE_ENABLE_SHIFT))
#define ROM_OCOTP_SDMMC_DLL_OVERRIDE_ENABLE_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_SDMMC_DLL_OVERRIDE_ENABLE_MASK) >> ROM_OCOTP_SDMMC_DLL_OVERRIDE_ENABLE_SHIFT)

/* DLL delay*/
#define ROM_OCOTP_SDMMC_DLL_DLY_SHIFT ((uint8_t)24)
#define ROM_OCOTP_SDMMC_DLL_DLY_MASK ((uint32_t)(0x7F << ROM_OCOTP_SDMMC_DLL_DLY_SHIFT))
#define ROM_OCOTP_SDMMC_DLL_DLY_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_SDMMC_DLL_DLY_MASK) >> ROM_OCOTP_SDMMC_DLL_DLY_SHIFT)

/* SD/MMC Index */
#define ROM_OCOTP_SDMMC_PORT_SEL_SHIFT ((uint8_t)1)
#define ROM_OCOTP_SDMMC_PORT_SEL_MASK ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_PORT_SEL_SHIFT))
#define ROM_OCOTP_SDMMC_PORT_SEL_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_SDMMC_PORT_SEL_MASK) >> ROM_OCOTP_SDMMC_PORT_SEL_SHIFT)

/* SD/MMC instance 1 reset polarity */
#define ROM_OCOTP_SD1_RST_ACTIVE_POLARITY_SHIFT ((uint8_t)6)
#define ROM_OCOTP_SD1_RST_ACTIVE_POLARITY_MASK ((uint32_t)(0x1 << ROM_OCOTP_SD1_RST_ACTIVE_POLARITY_SHIFT))
#define ROM_OCOTP_SD1_RST_ACTIVE_POLARITY_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_SD1_RST_ACTIVE_POLARITY_MASK) >> ROM_OCOTP_SD1_RST_ACTIVE_POLARITY_SHIFT)

/* SD/MMC instance 2 reset polarity */
#define ROM_OCOTP_SD2_RST_ACTIVE_POLARITY_SHIFT ((uint8_t)15)
#define ROM_OCOTP_SD2_RST_ACTIVE_POLARITY_MASK ((uint32_t)(0x1 << ROM_OCOTP_SD2_RST_ACTIVE_POLARITY_SHIFT))
#define ROM_OCOTP_SD2_RST_ACTIVE_POLARITY_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_SD2_RST_ACTIVE_POLARITY_MASK) >> ROM_OCOTP_SD2_RST_ACTIVE_POLARITY_SHIFT)

/* Override pad setting enable */
#define ROM_OCOTP_SDMMC_OVERRIDE_PAD_SETTINGS_ENABLE_SHIFT ((uint8_t)0)
#define ROM_OCOTP_SDMMC_OVERRIDE_PAD_SETTINGS_ENABLE_MASK \
    ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_OVERRIDE_PAD_SETTINGS_ENABLE_SHIFT))
#define ROM_OCOTP_SDMMC_OVERRIDE_PAD_SETTINGS_ENABLE_VALUE()                     \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_SDMMC_OVERRIDE_PAD_SETTINGS_ENABLE_MASK) >> \
     ROM_OCOTP_SDMMC_OVERRIDE_PAD_SETTINGS_ENABLE_SHIFT)

/* USDHC Pad setttings */
#define ROM_OCOTP_SDMMC_PAD_SETTINGS_SHIFT ((uint8_t)0)
#define ROM_OCOTP_SDMMC_PAD_SETTINGS_MASK ((uint32_t)(0x3F << ROM_OCOTP_SDMMC_PAD_SETTINGS_SHIFT))
#define ROM_OCOTP_SDMMC_PAD_SETTINGS_VALUE() \
    ((HW_OCOTP_REG_RD(45) & ROM_OCOTP_SDMMC_PAD_SETTINGS_MASK) >> ROM_OCOTP_SDMMC_PAD_SETTINGS_SHIFT)

/* USDHC IOMUX SRE Enable */
#define ROM_OCOTP_USDHC_IOMUX_SRE_ENABLE_SHIFT ((uint8_t)8)
#define ROM_OCOTP_USDHC_IOMUX_SRE_ENABLE_MASK ((uint32_t)(0x1 << ROM_OCOTP_USDHC_IOMUX_SRE_ENABLE_SHIFT))
#define ROM_OCOTP_USDHC_IOMUX_SRE_ENABLE_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_USDHC_IOMUX_SRE_ENABLE_MASK) >> ROM_OCOTP_USDHC_IOMUX_SRE_ENABLE_SHIFT)

/* Enable EMMC 22K Pullup */
#define ROM_OCOTP_USDHC_22KOHM_PULLUP_SHIFT ((uint8_t)11)
#define ROM_OCOTP_USDHC_22KOHM_PULLUP_MASK ((uint32_t)(0x1 << ROM_OCOTP_USDHC_22KOHM_PULLUP_SHIFT))
#define ROM_OCOTP_USDHC_22KOHM_PULLUP_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_USDHC_22KOHM_PULLUP_MASK) >> ROM_OCOTP_USDHC_22KOHM_PULLUP_SHIFT)

/* Pad HYS override fuse */
#define ROM_OCOTP_USDHC_PAD_OVERRIDE_HYS_SHIFT ((uint8_t)13)
#define ROM_OCOTP_USDHC_PAD_OVERRIDE_HYS_MASK ((uint32_t)(0x1 << ROM_OCOTP_USDHC_PAD_OVERRIDE_HYS_SHIFT))
#define ROM_OCOTP_USDHC_PAD_OVERRIDE_HYS() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_USDHC_PAD_OVERRIDE_HYS_MASK) >> ROM_OCOTP_USDHC_PAD_OVERRIDE_HYS_SHIFT)

/* USDHC IOMUX SION Enable */
#define ROM_OCOTP_USDHC_IOMUX_SION_ENABLE_SHIFT ((uint8_t)9)
#define ROM_OCOTP_USDHC_IOMUX_SION_ENABLE_MASK ((uint32_t)(0x1 << ROM_OCOTP_USDHC_IOMUX_SION_ENABLE_SHIFT))
#define ROM_OCOTP_USDHC_IOMUX_SION_ENABLE_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_USDHC_IOMUX_SION_ENABLE_MASK) >> ROM_OCOTP_USDHC_IOMUX_SION_ENABLE_SHIFT)

/* USDHC Pad Pulldown Enable */
#define ROM_OCOTP_USDHC_PAD_PULLDOWN_SHIFT ((uint8_t)12)
#define ROM_OCOTP_USDHC_PAD_PULLDOWN_MASK ((uint32_t)(0x1 << ROM_OCOTP_USDHC_PAD_PULLDOWN_SHIFT))
#define ROM_OCOTP_USDHC_PAD_PULLDOWN_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_USDHC_PAD_PULLDOWN_MASK) >> ROM_OCOTP_USDHC_PAD_PULLDOWN_SHIFT)

/* Disable SDMMC Manufacture Mode */
#define ROM_OCOTP_SDMMC_MFG_DISABLE_SHIFT ((uint8_t)3)
#define ROM_OCOTP_SDMMC_MFG_DISABLE_MASK ((uint32_t)(0x1 << ROM_OCOTP_SDMMC_MFG_DISABLE_SHIFT))
#define ROM_OCOTP_SDMMC_MFG_DISABLE_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_SDMMC_MFG_DISABLE_MASK) >> ROM_OCOTP_SDMMC_MFG_DISABLE_SHIFT)

/* SD CMD_OE */
#define USDHC_CMD_OE_PRE_EN_SHIFT ((uint8_t)23)
#define USDHC_CMD_OE_PRE_EN_MASK ((uint32_t)(0x1 << USDHC_CMD_OE_PRE_EN_SHIFT))
#define USDHC_CMD_OE_PRE_EN_VALUE() ((HW_OCOTP_REG_RD(7) & USDHC_CMD_OE_PRE_EN_MASK) >> USDHC_CMD_OE_PRE_EN_SHIFT)

/* ========================== MMC/SD boot(MMC Part)======================================= */
/* MMC Speed */
#define ROM_OCOTP_MMC_SPEED_SHIFT ((uint8_t)5)
#define ROM_OCOTP_MMC_SPEED_MASK ((uint32_t)(0x1 << ROM_OCOTP_MMC_SPEED_SHIFT))
#define ROM_OCOTP_MMC_SPEED_VALUE() ((SRC->SBMR1 & ROM_OCOTP_MMC_SPEED_MASK) >> ROM_OCOTP_MMC_SPEED_SHIFT)

/* MMC/eMMC Fast_Boot_ACK_Disable */
#define ROM_OCOTP_MMC_FAST_BOOT_ACK_SHIFT ((uint8_t)4)
#define ROM_OCOTP_MMC_FAST_BOOT_ACK_MASK ((uint32_t)(0x1 << ROM_OCOTP_MMC_FAST_BOOT_ACK_SHIFT))
#define ROM_OCOTP_MMC_FAST_BOOT_ACK_VALUE() \
    ((SRC->SBMR1 & ROM_OCOTP_MMC_FAST_BOOT_ACK_MASK) >> ROM_OCOTP_MMC_FAST_BOOT_ACK_SHIFT)

/* MMC Bus_Width */
#define ROM_OCOTP_MMC_BUS_WIDTH_SHIFT ((uint8_t)9)
#define ROM_OCOTP_MMC_BUS_WIDTH_MASK ((uint32_t)(0x3 << ROM_OCOTP_MMC_BUS_WIDTH_SHIFT))
#define ROM_OCOTP_MMC_BUS_WIDTH_VALUE() ((SRC->SBMR1 & ROM_OCOTP_MMC_BUS_WIDTH_MASK) >> ROM_OCOTP_MMC_BUS_WIDTH_SHIFT)

/* eMMC fast boot pre-idle reset mode fuse*/
#define ROM_OCOTP_EMMC_RESET_PREIDLE_STATE_SHIFT ((uint8_t)14)
#define ROM_OCOTP_EMMC_RESET_PREIDLE_STATE_MASK ((uint32_t)(0x1 << ROM_OCOTP_EMMC_RESET_PREIDLE_STATE_SHIFT))
#define ROM_OCOTP_EMMC_RESET_PREIDLE_STATE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_EMMC_RESET_PREIDLE_STATE_MASK) >> ROM_OCOTP_EMMC_RESET_PREIDLE_STATE_SHIFT)

/* ========================== MMC/SD boot(SD Part)======================================= */
/* SD Speed */
#define ROM_OCOTP_SD_SPEED_SHIFT ((uint8_t)4)
#define ROM_OCOTP_SD_SPEED_MASK ((uint32_t)(0x3 << ROM_OCOTP_SD_SPEED_SHIFT))
#define ROM_OCOTP_SD_SPEED_VALUE() ((SRC->SBMR1 & ROM_OCOTP_SD_SPEED_MASK) >> ROM_OCOTP_SD_SPEED_SHIFT)

/* SD_Calibration Step  */
#define ROM_OCOTP_SD_CAL_STEP_SHIFT ((uint8_t)30)
#define ROM_OCOTP_SD_CAL_STEP_MASK ((uint32_t)(0x3 << ROM_OCOTP_SD_CAL_STEP_SHIFT))
#define ROM_OCOTP_SD_CAL_STEP_VALUE() \
    ((HW_OCOTP_REG_RD(45) & ROM_OCOTP_SD_CAL_STEP_MASK) >> ROM_OCOTP_SD_CAL_STEP_SHIFT)

/* SD BUS_Width */
#define ROM_OCOTP_SD_BUS_WIDTH_SHIFT ((uint8_t)9)
#define ROM_OCOTP_SD_BUS_WIDTH_MASK ((uint32_t)(0x1 << ROM_OCOTP_SD_BUS_WIDTH_SHIFT))
#define ROM_OCOTP_SD_BUS_WIDTH_VALUE() ((SRC->SBMR1 & ROM_OCOTP_SD_BUS_WIDTH_MASK) >> ROM_OCOTP_SD_BUS_WIDTH_SHIFT)

/* ------------------------ Boot_CFG[23:16] END-------------------------------*/

/* ----------------------- Boot_CFG[31:24] Start ------------------------------*/

/* UART INTERRUPT DISABLE */
#define ROM_OCOTP_UART_INT_DIS_SHIFT (6)
#define ROM_OCOTP_UART_INT_DIS_MASK (1 << ROM_OCOTP_UART_INT_DIS_SHIFT)
#define ROM_OCOTP_UART_INT_DIS_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_UART_INT_DIS_MASK) >> ROM_OCOTP_UART_INT_DIS_SHIFT)

/* ------------------------ Boot_CFG[31:24] END--------------------------------*/

/* =================== OCOTP CFG5 REGISTER ENTRIES END =======================*/

/* =================== OCOTP CFG6 REGISTER ENTRIES ===========================*/

/* DIR_BT_DIS */
#define ROM_OCOTP_DIR_BT_DIS_MASK 0x00000008
#define ROM_OCOTP_DIR_BT_DIS_SHIFT ((uint8_t)3)
#define ROM_OCOTP_DIR_BT_DIS_VALUE() ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_DIR_BT_DIS_MASK) >> ROM_OCOTP_DIR_BT_DIS_SHIFT)

/* BT_FUSE_SEL */
#define ROM_OCOTP_BT_FUSE_SEL_MASK 0x00000010
#define ROM_OCOTP_BT_FUSE_SEL_SHIFT ((uint8_t)4)
#define ROM_OCOTP_BT_FUSE_SEL_VALUE() ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_BT_FUSE_SEL_MASK) >> ROM_OCOTP_BT_FUSE_SEL_SHIFT)

/* Force_Boot_from_fuse */
#define ROM_OCOTP_FORCE_COLD_BOOT_MASK 0x00000020
#define ROM_OCOTP_FORCE_COLD_BOOT_SHIFT ((uint8_t)5)
#define ROM_OCOTP_FORCE_COLD_BOOT_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_FORCE_COLD_BOOT_MASK) >> ROM_OCOTP_FORCE_COLD_BOOT_SHIFT)

/* L2_HW_Invalidate_disable */
#define ROM_OCOTP_L2_HW_INVALIDATE_DISABLE_MASK 0x00000080
#define ROM_OCOTP_L2_HW_INVALIDATE_DISABLE_SHIFT ((uint8_t)7)
#define ROM_OCOTP_L2_HW_INVALIDATE_DISABLE_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_L2_HW_INVALIDATE_DISABLE_MASK) >> ROM_OCOTP_L2_HW_INVALIDATE_DISABLE_SHIFT)

/* Force Boot from Fuse fuse */
#define ROM_OCOTP_FORCE_INTERNAL_BOOT_MASK 0x00010000
#define ROM_OCOTP_FORCE_INTERNAL_BOOT_SHIFT ((uint8_t)16)
#define ROM_OCOTP_FORCE_INTERNAL_BOOT_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_FORCE_INTERNAL_BOOT_MASK) >> ROM_OCOTP_FORCE_INTERNAL_BOOT_SHIFT)

/* SDP_ENABLE fuse */
#define ROM_OCOTP_SDP_DISABLE_MASK 0x00020000
#define ROM_OCOTP_SDP_DISABLE_SHIFT ((uint8_t)17)
#define ROM_OCOTP_SDP_DISABLE_VALUE() ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_SDP_DISABLE_MASK) >> ROM_OCOTP_SDP_DISABLE_SHIFT)

/* SDP_READ_DISABLE Fuse */
#define ROM_OCOTP_SDP_READ_DISABLE_MASK 0x00040000
#define ROM_OCOTP_SDP_READ_DISABLE_SHIFT ((uint8_t)18)
#define ROM_OCOTP_SDP_READ_DISABLE_VALUE() \
    ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_SDP_READ_DISABLE_MASK) >> ROM_OCOTP_SDP_READ_DISABLE_SHIFT)

/* WDOG enable fuse */
#define ROM_OCOTP_WDOG_ENABLE_MASK (0x00200000)
#define ROM_OCOTP_WDOG_ENABLE_SHIFT ((uint8_t)21)
#define ROM_OCOTP_WDOG_ENABLE_VALUE() ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_WDOG_ENABLE_MASK) >> ROM_OCOTP_WDOG_ENABLE_SHIFT)

/* XRDC_ENABLE fuse */
#define ROM_OCOTP_XRDC_ENABLE_MASK (0x10000000)
#define ROM_OCOTP_XRDC_ENABLE_SHIFT ((uint8_t)28)
#define ROM_OCOTP_XRDC_ENABLE_VALUE() ((HW_OCOTP_REG_RD(6) & ROM_OCOTP_SRDC_ENABLE_MASK) >> ROM_OCOTP_SRDC_ENABLE_SHIFT)

/* =================== OCOTP CFG6 REGISTER ENTRIES END==========================*/

/* =================== OCOTP CFG7 REGISTER ENTRIES ==========================*/
/* L1 I-Cache DISABLE */
#define ROM_OCOTP_ICACHE_DISABLE_MASK (0x00000004)
#define ROM_OCOTP_ICACHE_DISABLE_SHIFT ((uint8_t)2)
#define ROM_OCOTP_ICACHE_DISABLE_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_ICACHE_DISABLE_MASK) >> ROM_OCOTP_ICACHE_DISABLE_SHIFT)

/* L1 D-Cache DISABLE */
#define ROM_OCOTP_DCACHE_DISABLE_MASK (0x00000002)
#define ROM_OCOTP_DCACHE_DISABLE_SHIFT ((uint8_t)1)
#define ROM_OCOTP_DCACHE_DISABLE_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_DCACHE_DISABLE_MASK) >> ROM_OCOTP_DCACHE_DISABLE_SHIFT)

/* UART Serial Download DISABLE */
#define ROM_OCOTP_UART_SERIAL_DOWNLOAD_DISABLE_MASK (0x00000010)
#define ROM_OCOTP_UART_SERIAL_DOWNLOAD_DISABLE_SHIFT ((uint8_t)4)
#define ROM_OCOTP_UART_SERIAL_DOWNLOAD_DISABLE_VALUE() \
    ((HW_OCOTP_REG_RD(7) & ROM_OCOTP_UART_SERIAL_DOWNLOAD_DISABLE_MASK) >> ROM_OCOTP_UART_SERIAL_DOWNLOAD_DISABLE_SHIFT)
/* =================== OCOTP CFG7 REGISTER ENTRIES END==========================*/

/* ===============OCOTP CFG45 REGISTER ENTRIES - 0x6D0 =======================*/

/* Pad setttings */
#define ROM_OCOTP_PAD_SETTINGS_MASK (0x0000003F)
#define ROM_OCOTP_PAD_SETTINGS_SHIFT ((uint8_t)0)
#define ROM_OCOTP_PAD_SETTINGS_VALUE() \
    ((HW_OCOTP_REG_RD(45) & ROM_OCOTP_PAD_SETTINGS_MASK) >> ROM_OCOTP_PAD_SETTINGS_SHIFT)

/* USB_Vbus_event_Handler_en */
#define ROM_OCOTP_USB_VBUS_FIX_ENABLE_MASK (0x00000040)
#define ROM_OCOTP_USB_VBUS_FIX_ENABLE_SHIFT ((uint8_t)6)
#define ROM_OCOTP_USB_VBUS_FIX_ENABLE() \
    ((HW_OCOTP_REG_RD(45) & ROM_OCOTP_USB_VBUS_FIX_ENABLE_MASK) >> ROM_OCOTP_USB_VBUS_FIX_ENABLE_SHIFT)

/* NAND read retry sequence select */
#define ROM_OCOTP_READ_RETRY_SEQ_ID_MASK (0x00000F00)
#define ROM_OCOTP_READ_RETRY_SEQ_ID_SHIFT ((uint8_t)8)
#define ROM_OCOTP_READ_RETRY_SEQ_ID() \
    ((HW_OCOTP_REG_RD(45) & ROM_OCOTP_READ_RETRY_SEQ_ID_MASK) >> ROM_OCOTP_READ_RETRY_SEQ_ID_SHIFT)

/* WDOG_Timeout_Select */
#define ROM_OCOTP_WDOG_TIMEOUT_SEL_MASK (0x0000E000)
#define ROM_OCOTP_WDOG_TIMEOUT_SEL_SHIFT ((uint8_t)13)
#define ROM_OCOTP_WDOG_TIMEOUT_SEL_VALUE() \
    ((HW_OCOTP_REG_RD(45) & ROM_OCOTP_WDOG_TIMEOUT_SEL_MASK) >> ROM_OCOTP_WDOG_TIMEOUT_SEL_SHIFT)

/* ============== OCOTP CFG45 REGISTER ENTRIES END - 0x6D0 ===================*/

#endif /* __FUSEMAP_H__*/
