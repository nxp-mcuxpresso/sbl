/*
 * Copyright 2017 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BL_NOR_ENCRYPT_H__
#define __BL_NOR_ENCRYPT_H__

#include "fsl_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    union
    {
        struct
        {
            uint32_t reserved : 28;
            uint32_t tag : 4; //!< Tag, must be 0x0e
        } B;
        uint32_t U;
    } option0;
    uint32_t reserved[10];
} nor_encrypt_option_t;

enum
{
    kNorEncyptOption_Tag = 0x0e, //!< Tag
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

//! @brief Determine whether there is a valid encrypt region info
bool bl_nor_encrypt_region_info_valid(void *arg);

//! @brief Return the Encrypted Region Info presence status
bool bl_nor_encrypt_has_encrypted_region(void);

//! @brief Initialize Encrypt Region based on specified argument
status_t bl_nor_encrypt_init(void *arg);

//! @brief Refresh Encrypted region info
void bl_nor_encrypt_region_refresh(uint32_t start, uint32_t bytes);

//! @brief Check if a specified region is in encrypted region
bool bl_nor_in_encrypted_region(uint32_t start, uint32_t bytes);

//! @brief Get Configuration block
status_t bl_nor_encrypt_get_config_block(uint32_t index, uint32_t *start, uint32_t *bytes);

//! @brief Encrypted data in specified region
status_t bl_nor_encrypt_data(uint32_t addr, uint32_t size, uint32_t *data_start);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // __BL_NOR_ENCRYPT_H__
