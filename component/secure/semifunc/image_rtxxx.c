/*
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "image_rtxxx.h"

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
 * Note: Parameter image_size is no used here. Keep it for compatible with i.MXRTxxxx.
 */
int rom_verify_sig(uint32_t image_start, uint32_t image_size, uint32_t ivt_offset)
{
    static uint32_t user_buf[1024];
    kb_session_ref_t *context = NULL;
    const kb_options_t options = {.version = kRomApiVersion,
                                  .buffer = (uint8_t *)user_buf,
                                  .bufferLength = sizeof(user_buf),
                                  .op = kRomAuthenticateImage,
                                  {
                                      .authenticate =
                                      {
                                           .profile = 0,
                                           .minBuildNumber = 1,
                                           .maxImageLength = 0xFFFFFFFF,
                                           .userRHK = NULL,
                                      },
                                  },
                                };

    // executes kb_init() with proper options and skboot_authenticate() to do the final image authentication
    status_t initStatus = kb_init((&context), &options);
    if (initStatus != kStatus_Success)
    {
        return -2;
    }

    skboot_status_t status;
    secure_bool_t isSignVerified;
    
    // skboot_authenticate can be used only for XIP images
    status = skboot_authenticate((uint8_t *)(image_start + ivt_offset), &isSignVerified);

    kb_deinit(context);
    if (status == kStatus_SKBOOT_Success && 
        isSignVerified == kSECURE_TRACKER_VERIFIED)
    {
        return 0;
    }

    return -1;
}
