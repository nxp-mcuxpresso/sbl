/*
 * Copyright 2021 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "mcuboot_config.h"

#ifndef _UPDATE_KEY_CONTEXT_H_
#define _UPDATE_KEY_CONTEXT_H_

// The offset of key block in MCUboot application image
#define KEY_CONTEXT_OFFSET_IN_APP   (0x100u)

/*!
 * @brief Update application key context information.
 *
 * @param key_info_address application key context address.
 */
status_t update_key_context(uint32_t key_context_address);

#endif // _UPDATE_KEY_CONTEXT_H_

