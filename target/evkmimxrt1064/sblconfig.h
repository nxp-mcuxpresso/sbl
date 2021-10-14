/*
* Copyright 2021 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef SBL_CONFIG_H__
#define SBL_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* MCU-SBL RT1064 Configuration */

#define SOC_IMXRT1064_SERIES
#define ARCH_ARM_CORTEX_M7
#define ARCH_ARM_CORTEX_FPU
#define SOC_IMXRTYYYY_SERIES
#define SOC_REMAP_ENABLE
#define COMPONENT_MCU_ISP

/* MCU SBL core */

#define CONFIG_MCUBOOT_MAX_IMG_SECTORS 400

/* MCU SBL Flash Map */

#define BOOT_FLASH_BASE 0x70000000
#define BOOT_FLASH_HEADER 0x70010000
#define BOOT_FLASH_ACT_APP 0x70100000
#define BOOT_FLASH_CAND_APP 0x70200000
#define BOOT_FLASH_CUSTOMER 0x703f0000

/* MCU SBL Component */

/* Flash IAP */

#define COMPONENT_FLASHIAP
#define COMPONENT_FLASHIAP_ROM

/* Flash device parameters */

#define COMPONENT_FLASHIAP_SIZE 4194304

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
#define WINBOND_W25QxxxJV
#define SOC_MIMXRT1064DVL6A

/* On-chip Peripheral Drivers */

#define SOC_GPIO
#define SOC_LPUART
#define SOC_LPUART_1
#define SOC_FLEXSPI
#define SOC_FLEXSPI_2

/* Onboard Peripheral Drivers */


/* Board extended module Drivers */


#endif
