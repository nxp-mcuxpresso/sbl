/*
* Copyright 2021 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef SBL_CONFIG_H__
#define SBL_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* MCU-SBL LPC55S69 Configuration */

#define SOC_LPC55S69_SERIES
#define ARCH_ARM_CORTEX_M33
#define ARCH_ARM_CORTEX_FPU
#define SOC_LPC_SERIES
#define COMPONENT_MCU_ISP

/* MCU SBL core */

#define CONFIG_MCUBOOT_MAX_IMG_SECTORS 400

/* MCU SBL Flash Map */

#define BOOT_FLASH_BASE 0x00000000
#define BOOT_FLASH_HEADER 0x00018000
#define BOOT_FLASH_ACT_APP 0x00020000
#define BOOT_FLASH_CAND_APP 0x00050000
#define BOOT_FLASH_CUSTOMER 0x00080000

/* MCU SBL metadata header */

#define BOOT_METADATA

/* MCU SBL Component */

/* Flash IAP */

#define COMPONENT_FLASHIAP
#define COMPONENT_FLASHIAP_ROM

/* Flash device parameters */

#define COMPONENT_FLASHIAP_SIZE 524288

/* secure */

#define COMPONENT_MCUBOOT_SECURE
#define CONFIG_BOOT_SIGNATURE
#define CONFIG_BOOT_SIGNATURE_TYPE_RSA
#define CONFIG_BOOT_SIGNATURE_TYPE_RSA_LEN 2048
#define COMPONENT_MBEDTLS
#define SBL_MBEDTLS_CONFIG_FILE "ksdk_mbedtls_config.h"

/* Serial Manager */

#define COMPONENT_SERIAL_MANAGER
#define COMPONENT_SERIAL_MANAGER_LPUART
#define SERIAL_PORT_TYPE_UART 1

/* mcu isp support */

#define ISP_TIMEOUT 5

/* Platform Drivers Config */

#define BOARD_FLASH_SUPPORT
#define SOC_CPU_LPC55S69JBD100_cm33_core0

/* On-chip Peripheral Drivers */

#define SOC_GPIO
#define SOC_LPUART
#define SOC_LPUART_2
#define SOC_INPUTMUX
#define SOC_PINT
#define SOC_GINT

/* Board extended module Drivers */


#endif
