/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __BL_CACHE_H__
#define __BL_CACHE_H__

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
enum
{
    kCache_EnableAllowed = 0x5af0c3a5,
};

typedef struct
{
    uint32_t allowEnableICache;
    uint32_t allowEnableDCache;
} cache_context_t;

//!@brief cache initialization in Bootloader
void bl_cache_init(cache_context_t ctx);
//!@brief Enable I-Cache
void bl_icache_enable(void);
//!@brief Disable I-Cache
void bl_icache_disable(void);
//!@brief Invalidate I-Cache
void bl_icache_invalidate(void);

//!@brief Enable D-Cache
void bl_dcache_enable(void);
//!@brief Disable D-Cache
void bl_dcache_disable(void);
//!@brief Clean D-Cache
void bl_dcache_clean(void);
//!@brief Invalidate D-Cache
void bl_dcache_invalidate(void);
//!@brief Clean and Invalidate D-Cache
void bl_dcache_clean_invalidate(void);

#endif // __BL_CACHE_H__
