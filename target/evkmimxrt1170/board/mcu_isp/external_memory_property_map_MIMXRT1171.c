/*
 * Copyright 2017-2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "property/property.h"
#include "bootloader_common.h"
#include "memory/memory.h"

const external_memory_property_interface_t g_externalMemPropertyInterfaceMap[] = {
#if BL_FEATURE_FLEXSPI_NOR_MODULE
    { kMemoryFlexSpiNor, flexspi_nor_get_property },
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE
#if BL_FEATURE_SPINAND_MODULE
    { kMemorySpiNand, spinand_get_property },
#endif // #if BL_FEATURE_SPINAND_MODULE
#if BL_FEATURE_SEMC_NAND_MODULE
    { kMemorySemcNand, semc_nand_get_property },
#endif // #if BL_FEATURE_SEMC_NAND_MODULE
#if BL_FEATURE_SPI_NOR_EEPROM_MODULE
    { kMemorySpiNorEeprom, spi_nor_eeprom_get_property },
#endif
#if BL_FEATURE_SD_MODULE
    { kMemorySDCard, sd_get_property }, // SD card memory
#endif
#if BL_FEATURE_MMC_MODULE
    { kMemoryMMCCard, mmc_get_property }, // MMC/eMMC card memory
#endif
    { 0 } // Terminator
};

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
