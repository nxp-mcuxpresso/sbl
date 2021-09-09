/*
 * Copyright 2021 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>

#include "mcuboot_config.h"
#include "hab.h"

#ifdef MCUBOOT_SIGN_ROM

int
rom_verify_sig(uint32_t image_start, uint32_t image_size,
			       uint32_t ivt_offset)
{
  return imx_hab_authenticate_image(image_start, image_size, ivt_offset);
}


#endif