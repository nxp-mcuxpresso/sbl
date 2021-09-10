/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef IMAGE_RTXXX_H_
#define IMAGE_RTXXX_H_

#include "fsl_iap.h"
#include "fsl_iap_kbp.h"
#include "fsl_iap_skboot_authenticate.h"

/*!
 * @brief Verify authenticity of an image.
 *
 * Note: Parameter image_size is no used here. Jeep it for compatible with i.MXRT.
 */
int rom_verify_sig(uint32_t image_start, uint32_t image_size, uint32_t ivt_offset);
#endif
