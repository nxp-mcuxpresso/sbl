/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
 
#include "fsl_iap.h"
#include "fsl_iap_ffr.h"
#include "fsl_iap_kbp.h"
#include "fsl_iap_skboot_authenticate.h"

/*!
 * @brief Interface for image authentication API
 */
void HASHCRYPT_IRQHandler(void)
{
    HASH_IRQHandler();
}

/*!
 * @brief Verify authenticity of an image.
 *
 * Note: Parameter image_size is no used here. Jeep it for compatible with i.MXRT.
 */
int rom_verify_sig(uint32_t image_start, uint32_t image_size, uint32_t ivt_offset)
{
    skboot_status_t status;
    secure_bool_t isSignVerified;

    status = skboot_authenticate((uint8_t *)(image_start + ivt_offset), &isSignVerified);
    
    if (status == kStatus_SKBOOT_Success && 
        isSignVerified == kSECURE_TRACKER_VERIFIED)
    {
        return 0;
    }
    
    return -1;
}
