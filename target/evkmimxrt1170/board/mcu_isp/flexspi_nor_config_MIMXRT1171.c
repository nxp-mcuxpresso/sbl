/*
 * Copyright 2017-2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "flexspi_nor_flash.h"
#include "fusemap.h"
#include "utilities/fsl_assert.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum
{
    kSerialNOR_3ByteAddressRead = 0U, // Device supports 0x03 Read with 24bit address
    kSerialNOR_4ByteAddressRead = 1U, // Device supports 0x13 Read with 32bit address
    kSerialNOR_HyperFlash1V8 = 2U,    // HyperFlash 1V8 part
    kSerialNOR_HyperFlash3V3 = 3U,    // HyperFlash 3V3 part
    kSerialNOR_MxicOctDDR = 4U,       // MXIC Octal Flash with OPI DDR read enabled by default
    kSerialNOR_MicronOctDDR = 5U,     // Micron octal Flash with OPI DDR read enabled by default
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
// Dedicated 3Byte Address Read(0x03), 24bit address
static const uint32_t s_dedicated3bRead[4] = {
    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x03, RADDR_SDR, FLEXSPI_1PAD, 0x18),
    FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0), 0, 0
};
// Dedicated 4Byte Address Read(0x13), 32 bit address
static const uint32_t s_dedicated4bRead[4] = {
    FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x13, RADDR_SDR, FLEXSPI_1PAD, 0x20),
    FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0), 0, 0
};
// HyperFlash Read
static const uint32_t s_hyperflashRead[4] = {
    FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0, RADDR_DDR, FLEXSPI_8PAD, 0x18),
    FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10, READ_DDR, FLEXSPI_8PAD, 0x04), 0, 0
};

// MXIC Octal DDR read
static const uint32_t s_mxicOctDdrRead[4] = {
    FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xEE, CMD_DDR, FLEXSPI_8PAD, 0x11),
    FLEXSPI_LUT_SEQ(RADDR_DDR, FLEXSPI_8PAD, 0x20, READ_DDR, FLEXSPI_8PAD, 0x04), 0, 0
};

////////////////////////////////////////////////////////////////////////////////
// Codes
////////////////////////////////////////////////////////////////////////////////
status_t flexspi_nor_get_default_cfg_blk(flexspi_nor_config_t *config)
{
    flexspi_mem_config_t *memCfg = &config->memConfig;
    // Read fuse to get device type
    uint32_t flashType = FUSE_FLASH_TYPE_VALUE;

    if (get_primary_boot_device() != kBootDevice_FlexSpiNOR)
    {
        return kStatus_InvalidArgument;
    }

    memset(config, 0, sizeof(flexspi_nor_config_t));

    memCfg->tag = FLEXSPI_CFG_BLK_TAG;
    memCfg->version = FLEXSPI_CFG_BLK_VERSION;
    memCfg->deviceType = kFlexSpiDeviceType_SerialNOR;
    memCfg->sflashA1Size = 128UL * 1024 * 1024; // 128M

    memCfg->csHoldTime = 3;
    memCfg->csSetupTime = 3;
    memCfg->sflashPadType = kSerialFlash_1Pad;

    // For most devices which capacity is lower than 16MB, this command is supported.
    if (flashType == kSerialNOR_3ByteAddressRead)
    {
        memcpy(memCfg->lookupTable, &s_dedicated3bRead, sizeof(s_dedicated3bRead));
    }
    // For most devices which capacity is larger than 16MB, this command is supported.
    else if (flashType == kSerialNOR_4ByteAddressRead)
    {
        memcpy(memCfg->lookupTable, &s_dedicated4bRead, sizeof(s_dedicated4bRead));
    }
    // Devices belonging HyperBus family
    else if ((flashType == kSerialNOR_HyperFlash1V8) || (flashType == kSerialNOR_HyperFlash3V3))
    {
        memCfg->sflashPadType = kSerialFlash_8Pads;
        memCfg->columnAddressWidth = 3;
        memCfg->readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;
        memCfg->controllerMiscOption = 1 << kFlexSpiMiscOffset_WordAddressableEnable;
        if (!FUSE_DELAY_CELL_NUM_VALUE)
        {
            memCfg->dataValidTime[0].time_100ps = 15; // 1.5ns
        }
        else
        {
            memCfg->dataValidTime[0].time_100ps = FUSE_DELAY_CELL_NUM_VALUE;
        }

        memcpy(memCfg->lookupTable, &s_hyperflashRead, sizeof(s_hyperflashRead));

        if (flashType == kSerialNOR_HyperFlash1V8)
        {
            memCfg->controllerMiscOption |= (1 << kFlexSpiMiscOffset_DiffClkEnable);
        }
    }
    // MXIC dedicated Octal command
    else if (flashType == kSerialNOR_MxicOctDDR)
    {
        memCfg->sflashPadType = kSerialFlash_8Pads;
        memCfg->readSampleClkSrc = kFlexSPIReadSampleClk_ExternalInputFromDqsPad;
        if (!FUSE_DELAY_CELL_NUM_VALUE)
        {
            memCfg->dataValidTime[0].time_100ps = 15; // 1.5ns
        }
        else
        {
            memCfg->dataValidTime[0].time_100ps = FUSE_DELAY_CELL_NUM_VALUE;
        }

        memcpy(memCfg->lookupTable, &s_mxicOctDdrRead, sizeof(s_mxicOctDdrRead));
    }

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
