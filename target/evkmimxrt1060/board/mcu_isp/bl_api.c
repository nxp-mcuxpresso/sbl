/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "bl_api.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bootloader_api_entry_t *g_bootloaderTree;

/*******************************************************************************
 * Codes
 ******************************************************************************/

void bl_api_init(void)
{
    g_bootloaderTree = (bootloader_api_entry_t *)*(uint32_t *)0x0020001c;
}

/*******************************************************************************
 * FlexSPI NOR driver
 ******************************************************************************/
#if BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI
status_t flexspi_nor_flash_init(uint32_t instance, flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexSpiNorDriver->init(instance, config);
}

status_t flexspi_nor_flash_section_program(uint32_t instance,
                                           flexspi_nor_config_t *config,
                                           uint32_t dstAddr,
                                           const uint32_t *src)
{
    return g_bootloaderTree->flexSpiNorDriver->program(instance, config, dstAddr, src);
}

status_t flexspi_nor_flash_erase_all(uint32_t instance, flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexSpiNorDriver->erase_all(instance, config);
}

status_t flexspi_nor_get_config(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option)
{
    status_t status = g_bootloaderTree->flexSpiNorDriver->get_config(instance, config, option);

    if ((status == kStatus_Success) && option->option0.B.option_size)
    {
        // A workaround to support drive strength configuration using Flash APIs
        if (option->option1.B.drive_strength)
        {
            flexspi_update_padsetting(&config->memConfig, option->option1.B.drive_strength);
        }

        // A workaround to support parallel mode using Flash APIs
        if (option->option1.B.flash_connection == kSerialNorConnection_Parallel)
        {
            config->memConfig.controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_ParallelEnable);
            config->pageSize *= 2;
            config->sectorSize *= 2;
            config->blockSize *= 2;
            config->memConfig.sflashB1Size = config->memConfig.sflashA1Size;
        }
    }

    return status;
}

status_t flexspi_nor_flash_erase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length)
{
    return g_bootloaderTree->flexSpiNorDriver->erase(instance, config, start, length);
}

status_t flexspi_nor_flash_read(
    uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
    return g_bootloaderTree->flexSpiNorDriver->read(instance, config, dst, start, bytes);
}
#endif // BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI
