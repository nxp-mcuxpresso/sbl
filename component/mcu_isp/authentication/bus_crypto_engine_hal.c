/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "authentication/bus_crypto_engine_hal.h"


secure_bool_t skboot_hal_bus_crypto_engine_is_encryption_enabled(void)
{
    return (((PRINCE_Type *)PRINCE)->ENC_ENABLE == 1) ? kSECURE_TRUE : kSECURE_FALSE;
}

status_t skboot_hal_bus_crypto_engine_set_encrypt_for_address_range(prince_region_t prince_region, uint32_t startAddr, uint32_t length,
                                                           flash_config_t *flash_context)
{
    if (prince_region > kPRINCE_Region2)
    {
        return kStatus_Fail;
    }

    // make sure that PUF key store setup is complete
    if (skboot_hal_bus_crypto_engine_is_encryption_enabled() != kSECURE_TRUE)
    {
        return kStatus_Fail;
    }

    if (kStatus_Success == PRINCE_SetEncryptForAddressRange(prince_region, startAddr, length,
                                                flash_context, true))
    {
        return kStatus_Success;
    }
    else
    {
        return kStatus_Fail;
    }
}

status_t skboot_hal_bus_crypto_engine_is_write_allowed(uint32_t start, uint32_t lengthInBytes, flash_config_t *config)
{
    return kSECURE_TRUE;
}

status_t skboot_hal_bus_crypto_engine_is_erase_allowed(uint32_t start, uint32_t lengthInBytes, flash_config_t *config)
{
    return kSECURE_TRUE;
}

status_t skboot_hal_bus_crypto_engine_is_read_allowed(uint32_t physicalAddr, uint32_t pageSize)
{
    return kSECURE_TRUE;
}
