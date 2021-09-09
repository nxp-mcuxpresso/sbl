/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef _BUS_CRTPTO_ENGINE_HAL_H_
#define _BUS_CRTPTO_ENGINE_HAL_H_

#if defined __ICCARM__ || defined __GNUC__ || __CC_ARM
#include <stddef.h>
#include "fsl_device_registers.h"
#include "fsl_prince.h"
     
secure_bool_t skboot_hal_bus_crypto_engine_is_encryption_enabled(void);

status_t skboot_hal_bus_crypto_engine_set_encrypt_for_address_range(prince_region_t prince_region, uint32_t startAddr, uint32_t length,
                                                           flash_config_t *flash_context);

status_t skboot_hal_bus_crypto_engine_is_write_allowed(uint32_t start, uint32_t lengthInBytes, flash_config_t *config);

status_t skboot_hal_bus_crypto_engine_is_erase_allowed(uint32_t start, uint32_t lengthInBytes, flash_config_t *config);

status_t skboot_hal_bus_crypto_engine_is_read_allowed(uint32_t physicalAddr, uint32_t pageSize);

#endif /* __ICCARM__ || defined __GNUC__ || __CC_ARM */
#endif /* _BUS_CRTPTO_ENGINE_HAL_H_ */
