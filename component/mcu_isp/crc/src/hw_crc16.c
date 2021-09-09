/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "crc/crc16.h"
#include "drivers/crc/fsl_crc_driver.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/fsl_assert.h"

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
void crc16_init(crc16_data_t *crc16Config)
{
    assert(crc16Config);

    crc16Config->currentCrc = 0x0000U;
}

void crc16_update(crc16_data_t *crc16Config, const uint8_t *src, uint32_t lengthInBytes)
{
    assert(crc16Config);
    assert(src);

    crc_user_config_t crcUserConfigPtr;

    crcUserConfigPtr.crcWidth = kCrc16Bits;
    crcUserConfigPtr.seed = crc16Config->currentCrc;
    crcUserConfigPtr.polynomial = 0x1021U;
    crcUserConfigPtr.writeTranspose = kCrcNoTranspose;
    crcUserConfigPtr.readTranspose = kCrcNoTranspose;
    crcUserConfigPtr.complementRead = false;

    // Init CRC module and then run it
    //! Note: We must init CRC module here, As we may seperate one crc calculation into several times
    //! Note: It is better to use lock to ensure the integrity of current updating operation of crc calculation
    //        in case crc module is shared by multiple crc updating requests at the same time
    if (lengthInBytes)
    {
        lock_acquire();
        CRC_DRV_Init(CRC_INSTANCE, &crcUserConfigPtr);
        crcUserConfigPtr.seed = CRC_DRV_GetCrcBlock(CRC_INSTANCE, (uint8_t *)src, lengthInBytes);
        lock_release();
    }

    crc16Config->currentCrc = crcUserConfigPtr.seed;
}

void crc16_finalize(crc16_data_t *crc16Config, uint16_t *hash)
{
    assert(crc16Config);
    assert(hash);

    *hash = crc16Config->currentCrc;

    // De-init CRC module when we complete a full crc calculation
    CRC_DRV_Deinit(CRC_INSTANCE);
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
