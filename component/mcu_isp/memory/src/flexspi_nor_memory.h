/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__FLEXSPINOR_MEMORY_H__)
#define __FLEXSPINOR_MEMORY_H__

#include "memory/memory.h"
//#include "flexspi_nor_flash.h"

#if BL_FEATURE_FLEXSPI_NOR_MODULE
#include "flexspi_flash.h"
#include "flexspi_flash_config.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum
{
    //! @brief Max page size used to create a page buffer
    kFlexSpiNorMemory_MaxPageSize = 1024U,
    //! @brief Max total size of NOR FLASH supported by FlexSPI
    kFlexSpiNorMemory_MaxSize = 256U * 1024 * 1024,
    //! @brief Max Size of SPI NOR alias memory
    kFlexSpiNorAliasAreaMaxSize = 0x04000000U, // 64MB

    kFlexSpiNorProgramConfigBlock_Tag = 0x0F,
};

#define MAGIC_NUMBER_FLEXSPI_NOR_PROGRAM_CFG \
    (((uint32_t)kFlexSpiNorProgramConfigBlock_Tag << 28) | kFlexSpiNorProgramConfigBlock_Tag)

#define MAGIC_NUMBER_FLEXSPI_NOR_PRECFG (0xcf9 | (kSerialNorCfgOption_Tag << 8) & (kSerialNorCfgOption_Tag << 8))

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

//! @name FlexSPI NOR memory
//@{
  
void flexspi_nor_mem_enable(void);

//! @brief Initialize SPI NOR memory
status_t flexspi_nor_mem_init(void);

//! @brief Read SPI NOR memory.
status_t flexspi_nor_mem_read(uint32_t address, uint32_t length, uint8_t *buffer);

//! @brief Write SPI NOR memory.
status_t flexspi_nor_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Fill SPI NOR memory with a word pattern.
status_t flexspi_nor_mem_fill(uint32_t address, uint32_t length, uint32_t pattern);

//! @brief Erase SPI NOR memory
status_t flexspi_nor_mem_erase(uint32_t address, uint32_t length);

//! @brief flush cached data to SPI NOR memory
status_t flexspi_nor_mem_flush(void);

#if BL_FEATURE_FLEXSPI_ALIAS_AREA
//! @brief Initialize SPI NOR alias
status_t flexspi_nor_alias_init(void);

//! @brief Write SPI NOR alias.
status_t flexspi_nor_alias_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Fill SPI NOR alias with a word pattern.
status_t flexspi_nor_alias_fill(uint32_t address, uint32_t length, uint32_t pattern);

//! @brief  Erase SPI NOR alias
status_t flexspi_nor_alias_erase(uint32_t address, uint32_t length);
#endif // #if BL_FEATURE_FLEXSPI_ALIAS_AREA

//@}

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // BL_FEATURE_FLEXSPI_NOR_MODULE
#endif // __FLEXSPINOR_MEMORY_INTERFACE_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
