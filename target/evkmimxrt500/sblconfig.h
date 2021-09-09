/*
* Copyright 2021 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef SBL_CONFIG_H__
#define SBL_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* MCU-SBL RT500 Configuration */

#define SOC_IMXRT500_SERIES
#define ARCH_ARM_CORTEX_M33
#define ARCH_ARM_CORTEX_FPU
#define SOC_IMXRTXXX_SERIES
#define SOC_REMAP_ENABLE
#define COMPONENT_MCU_ISP

/* MCU SBL core */

#define CONFIG_MCUBOOT_MAX_IMG_SECTORS 400

/* MCU SBL Flash Map */

#define BOOT_FLASH_BASE 0x08000000
#define BOOT_FLASH_HEADER 0x08010000
#define BOOT_FLASH_ACT_APP 0x08100000
#define BOOT_FLASH_CAND_APP 0x08200000
#define BOOT_FLASH_CUSTOMER 0x083f0000

/* MCU SBL metadata header */

#define BOOT_METADATA

/* MCU SBL Component */

/* Flash IAP */

#define COMPONENT_FLASHIAP
#define COMPONENT_FLASHIAP_ROM

/* Flash device parameters */

#define COMPONENT_FLASHIAP_SIZE 67108864

/* secure */

#define COMPONENT_MCUBOOT_SECURE
#define CONFIG_BOOT_SIGNATURE
#define CONFIG_BOOT_SIGNATURE_TYPE_RSA
#define CONFIG_BOOT_SIGNATURE_TYPE_RSA_LEN 2048
#define COMPONENT_MBEDTLS
#define SBL_MBEDTLS_CONFIG_FILE "mcuboot-mbedtls-cfg.h"

/* Serial Manager */

#define COMPONENT_SERIAL_MANAGER
#define COMPONENT_SERIAL_MANAGER_LPUART
#define SERIAL_PORT_TYPE_UART 1

/* mcu isp support */

#define ISP_TIMEOUT 5

/* Platform Drivers Config */

#define BOARD_FLASH_SUPPORT
#define Macronix_MX25UM51345G
#define SOC_MIMXRT595S_M33

/* On-chip Peripheral Drivers */

#define SOC_GPIO
#define SOC_UART
#define SOC_FLEXSPI
#define SOC_FLEXSPI_0
#define SOC_INPUTMUX
#define SOC_PINT

/* Onboard Peripheral Drivers */

/* Board extended module Drivers */


#endif
