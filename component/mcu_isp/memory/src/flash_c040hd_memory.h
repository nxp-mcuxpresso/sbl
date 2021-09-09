/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__FLASH_C040HD_MEMORY_INTERFACE_H__)
#define __FLASH_C040HD_MEMORY_INTERFACE_H__

#include "memory/memory.h"
#include "fsl_iap.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

//! @brief flash memory array.
extern flash_config_t g_flashState[];

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct
{
    uint32_t target_prince_region : 2; // 0/1/2
    uint32_t reserved : 22;
    uint32_t tag : 8; // Fixed to 0x50 ('P')
} prince_prot_region_option_t;

typedef struct
{
    prince_prot_region_option_t option;
    uint32_t start;
    uint32_t length;
} prince_prot_region_arg_t;

enum
{
    kPrinceProtRegionArg_Option_Tag = 0x50,
};

//! @name Flash c040hd memory
//! @note Flash read is done through the normal memory interface.
//@{

//! @brief Init selected Flash memory
status_t flash_mem_init(void);

//! @brief Read selected Flash memory.
status_t flash_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer);

//! @brief Write selected Flash memory.
status_t flash_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Fill selected Flash memory with a word pattern.
status_t flash_mem_fill(uint32_t address, uint32_t length, uint32_t pattern);

//! @brief Flush final buffer or cached data into selected FLASH memory.
status_t flash_mem_flush(void);

//! @brief Erase selected Flash memory.
status_t flash_mem_erase(uint32_t address, uint32_t length);

//! @brief Config the Flash memory.
status_t flash_config(uint32_t *config);
//@}

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // __FLASH_C040HD_MEMORY_INTERFACE_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
