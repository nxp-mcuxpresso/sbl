/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* image header, size 1KB */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
    __attribute__((section(".image_header")))
#elif defined(__ICCARM__)
#pragma location=".image_header"
#endif
const char __image_header[1024] = {0};
