/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#if !defined(__FLASH_FFR_MEMORY_H__)
#define __FLASH_FFR_MEMORY_H__

#include "memory/memory.h"
#include "fsl_iap_ffr.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum _ffr_keystore_update_opt
{
    kFfrKeystoreUpdateOpt_KeyProvisioning = 0x0u,
    kFfrKeystoreUpdateOpt_WriteMemory = 0x1u,
    kFfrKeystoreUpdateOpt_Invalid = 0xFFFFFFFFu,
};

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

//! @name Flash c040hd FFR memory
//! @note Flash read is done through the normal memory interface.
//@{

//! @brief Init selected Flash FFR memory
status_t ffr_mem_init(void);

//! @brief Read selected Flash FFR memory.
status_t ffr_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer);

//! @brief Erase selected Flash FFR memory.
status_t ffr_mem_erase(uint32_t address, uint32_t length);

//! @brief Write selected Flash FFR memory.
status_t ffr_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Flush cached data to selected Flash FFR memory.
status_t ffr_mem_flush(void);
//@}

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // __FLASH_FFR_MEMORY_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
