/*
 * Copyright 2017 NXP
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

#include "flash_c040hd/api_flash.h"
#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
#include "authentication/bus_crypto_engine_hal.h"
#endif
#include "bootloader/bl_context.h"

/*******************************************************************************
* Variables
 ******************************************************************************/

/* Bootloader API interface */
const flash_driver_interface_t g_flashDriverInterface = {
    .version = {.name = kFLASH_DriverVersionName,
                .major = kFLASH_DriverVersionMajor,
                .minor = kFLASH_DriverVersionMinor,
                .bugfix = kFLASH_DriverVersionBugfix },
    .flash_init = FLASH_Init,
    .flash_verify_erase = FLASH_VerifyErase,
    .flash_get_property = FLASH_GetProperty,
    .flash_erase = FLASH_Erase,
    .flash_program = FLASH_Program,
    .flash_verify_program = FLASH_VerifyProgram,

#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
    .flash_erase_with_checker = FLASH_ErasePrologue,
    .flash_program_with_checker = FLASH_ProgramPrologue,
    .flash_verify_program_with_checker = FLASH_VerifyProgramPrologue,
#else
    .flash_erase_with_checker = NULL,
    .flash_program_with_checker = NULL,
    .flash_verify_program_with_checker = NULL,
#endif // BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
/*
    .ffr_init = FFR_Init,
    .ffr_deinit = FFR_Deinit,
    .ffr_cust_factory_page_write = FFR_CustFactoryPageWrite,
    .ffr_get_uuid = FFR_GetUUID,
    .ffr_get_customer_data = FFR_GetCustomerData,
    .ffr_keystore_write = FFR_KeystoreWrite,
    .ffr_keystore_get_ac = FFR_KeystoreGetAC,
    .ffr_keystore_get_kc = FFR_KeystoreGetKC,
    .ffr_infield_page_write = FFR_InfieldPageWrite,
    .ffr_get_customer_infield_data = FFR_GetCustomerInfieldData,
*/
};

#if defined(BL_FEATURE_HAS_BUS_CRYPTO_ENGINE) && BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
status_t FLASH_ErasePrologue(flash_config_t *config, uint32_t start, uint32_t lengthInBytes, uint32_t key)
{
    /* Check that the whole encrypted region is erased at once.
       This is not necessary if called from the ISP commands handler that already performs this check,
       see handle_flash_erase_region() in bl_command.c and ldr_DoEraseCmd_v2() in sbloader_v2.c */
    if (kSECURE_TRUE != skboot_hal_bus_crypto_engine_is_erase_allowed(start, lengthInBytes, config))
    {
        return kStatus_Fail;
    }
    return FLASH_Erase(config, start, lengthInBytes, key);
}

status_t FLASH_ProgramPrologue(flash_config_t *config, uint32_t start, uint8_t *src, uint32_t lengthInBytes)
{
    /* Check that the whole encrypted subregions will be writen at once.
       This is not necessary if called from the ISP commands handler that already performs this check,
       see handle_write_memory() in bl_command.c and ldr_DoLoadCmd_v2() in sbloader_v2.c */
    if (kSECURE_TRUE != skboot_hal_bus_crypto_engine_is_write_allowed(start, lengthInBytes, config))
    {
        return kStatus_Fail;
    }
    return FLASH_Program(config, start, src, lengthInBytes);
}
status_t FLASH_VerifyProgramPrologue(flash_config_t *config,
                                     uint32_t start,
                                     uint32_t lengthInBytes,
                                     const uint8_t *expectedData,
                                     uint32_t *failedAddress,
                                     uint32_t *failedData)
{
    /* Do not perform the verification when the encryption is enabled, othervise the flash driver error is returned. */
    if (kSECURE_TRUE == skboot_hal_bus_crypto_engine_is_encryption_enabled())
    {
        return kStatus_Fail;
    }
    return FLASH_VerifyProgram(config, start, lengthInBytes, expectedData, failedAddress, failedData);
}
#endif // BL_FEATURE_HAS_BUS_CRYPTO_ENGINE
