/*
 * Copyright 2017-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _API_FLASH_H_
#define _API_FLASH_H_

#include "bootloader_common.h"
#include "fsl_iap.h"
#include "fsl_iap_ffr.h"
#include "memory/src/flash_c040hd_memory.h"
#include "memory/src/flash_ffr_memory.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define FLASH_CONFIG_T flash_config_t
#define FLASH_INTERFACE_T flash_driver_interface_t
#define FLASH_STATE g_flashState

//! @brief Interface for the flash driver.
typedef struct FlashDriverInterface
{
    standard_version_t version; //!< flash driver API version number.
    // Flash driver
    status_t (*flash_init)(flash_config_t *config);
    status_t (*flash_erase)(flash_config_t *config, uint32_t start, uint32_t lengthInBytes, uint32_t key);
    status_t (*flash_program)(flash_config_t *config, uint32_t start, uint8_t *src, uint32_t lengthInBytes);
    status_t (*flash_verify_erase)(flash_config_t *config, uint32_t start, uint32_t lengthInBytes);
    status_t (*flash_verify_program)(flash_config_t *config,
                                     uint32_t start,
                                     uint32_t lengthInBytes,
                                     const uint8_t *expectedData,
                                     uint32_t *failedAddress,
                                     uint32_t *failedData);
    status_t (*flash_get_property)(flash_config_t *config, flash_property_tag_t whichProperty, uint32_t *value);

    status_t (*flash_erase_with_checker)(flash_config_t *config, uint32_t start, uint32_t lengthInBytes, uint32_t key);
    status_t (*flash_program_with_checker)(flash_config_t *config, uint32_t start, uint8_t *src, uint32_t lengthInBytes);
    status_t (*flash_verify_program_with_checker)(flash_config_t *config,
                                                  uint32_t start,
                                                  uint32_t lengthInBytes,
                                                  const uint8_t *expectedData,
                                                  uint32_t *failedAddress,
                                                  uint32_t *failedData);
    // Flash FFR driver
    status_t (*ffr_init)(flash_config_t *config);
    status_t (*ffr_deinit)(flash_config_t *config);
    status_t (*ffr_cust_factory_page_write)(flash_config_t *config, uint8_t* page_data, bool seal_part); 
    status_t (*ffr_get_uuid)(flash_config_t *config, uint8_t* uuid); 
    status_t (*ffr_get_customer_data)(flash_config_t *config, uint8_t* pData, uint32_t offset, uint32_t len); 
    status_t (*ffr_keystore_write)(flash_config_t *config, ffr_key_store_t* pKeyStore); 
    status_t (*ffr_keystore_get_ac)(flash_config_t *config, uint8_t* pActivationCode); 
    status_t (*ffr_keystore_get_kc)(flash_config_t *config, uint8_t* pKeyCode, ffr_key_type_t keyIndex); 
    status_t (*ffr_infield_page_write)(flash_config_t *config, uint8_t* page_data, uint32_t valid_len); 
    status_t (*ffr_get_customer_infield_data)(flash_config_t *config, uint8_t* pData, uint32_t offset, uint32_t len);
} flash_driver_interface_t;

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

extern const flash_driver_interface_t g_flashDriverInterface;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
status_t FLASH_ErasePrologue(flash_config_t *config, uint32_t start, uint32_t lengthInBytes, uint32_t key);
status_t FLASH_ProgramPrologue(flash_config_t *config, uint32_t start, uint8_t *src, uint32_t lengthInBytes);
status_t FLASH_VerifyProgramPrologue(flash_config_t *config,
                                     uint32_t start,
                                     uint32_t lengthInBytes,
                                     const uint8_t *expectedData,
                                     uint32_t *failedAddress,
                                     uint32_t *failedData);
#endif //BL_FEATURE_HAS_BUS_CRYPTO_ENGINE


#if defined(__cplusplus)
extern "C" {
#endif

#if defined(__cplusplus)
}
#endif

/*!
 *@}
*/

#endif /* _API_FLASH_H_ */
