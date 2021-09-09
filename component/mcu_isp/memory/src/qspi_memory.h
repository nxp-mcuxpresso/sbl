/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__QSPI_MEMORY_INTERFACE_H__)
#define __QSPI_MEMORY_INTERFACE_H__

#include "memory/memory.h"

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

enum
{
    kQspiMemoryId_QuadSpiMemory = 0,
    kQspiMemoryId_AliasArea = 1,
};

//! @name QSPI memory
//! @note QSPI read is done through the normal memory interface.
//@{

//! @brief Initialize QSPI memory
status_t qspi_mem_init(uint32_t memoryId);

//! @brief Read QSPI memory.
status_t qspi_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer);

//! @brief Write QSPI Flash memory.
status_t qspi_mem_write(uint32_t memoryId, uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Fill QSPI memory with a word pattern.
status_t qspi_mem_fill(uint32_t memoryId, uint32_t address, uint32_t length, uint32_t pattern);

//! @brief  Erase QSPI memory
status_t qspi_mem_erase(uint32_t memoryId, uint32_t address, uint32_t length);

//! @brief Erase all QSPI memory
status_t qspi_mem_erase_all(void);

//! @brief Flush cached data into QSPI memory
status_t qspi_mem_flush(void);

//@}

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // __QSPI_MEMORY_INTERFACE_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
