/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bootloader.h"
#if BL_FEATURE_HAS_INTERNAL_FLASH
#include "flash/fsl_flash.h"
#include "memory/src/flash_memory.h"
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_FEATURE_ENCRYPTION
#include "security/aes_security.h"
#endif // #if BL_FEATURE_ENCRYPTION
#include "bootloader/bl_user_entry.h"

//! @addtogroup context
//! @{

//! @brief Root of the bootloader API tree.
//!
//! An instance of this struct resides in read-only memory in the bootloader. It
//! provides a user application access to APIs exported by the bootloader.
//!
//! @note The order of existing fields must not be changed.
//!
//! @ingroup context
#if 1 // Moved into each SOC based header file in future !!!!!!!!!!!!!
typedef struct BootloaderTree
{
    void (*runBootloader)(void *arg);            //!< Function to start the bootloader executing.
    standard_version_t version;                  //!< Bootloader version number.
    const char *copyright;                       //!< Copyright string.
    const bootloader_context_t *runtimeContext;  //!< Pointer to the bootloader's runtime context.
    const flash_driver_interface_t *flashDriver; //!< Flash driver API.
    const aes_driver_interface_t *aesDriver;     //!< AES driver API.
} bootloader_tree_t;
#endif

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Function table for flash driver.
#if BL_FEATURE_HAS_INTERNAL_FLASH
const flash_driver_interface_t g_flashDriverInterface = {
    .version = {.name = kFLASH_DriverVersionName,
                .major = kFLASH_DriverVersionMajor,
                .minor = kFLASH_DriverVersionMinor,
                .bugfix = kFLASH_DriverVersionBugfix },
    .flash_init = FLASH_Init,
    .flash_erase_all = FLASH_EraseAll,
#if BL_FEATURE_ERASEALL_UNSECURE
    .flash_erase_all_unsecure = FLASH_EraseAllUnsecure,
#else
    .flash_erase_all_unsecure = NULL,
#endif
    .flash_erase = FLASH_Erase,
    .flash_program = FLASH_Program,
    .flash_get_security_state = FLASH_GetSecurityState,
    .flash_security_bypass = FLASH_SecurityBypass,
    .flash_verify_erase_all = FLASH_VerifyEraseAll,
    .flash_verify_erase = FLASH_VerifyErase,
    .flash_verify_program = FLASH_VerifyProgram,
    .flash_get_property = FLASH_GetProperty,
#if BL_FEATURE_BYPASS_WATCHDOG
    .flash_register_callback = FLASH_SetCallback,
#else
    .flash_register_callback = NULL,
#endif
#if !BL_FEATURE_MIN_PROFILE
    .flash_program_once = FLASH_ProgramOnce,
    .flash_read_once = FLASH_ReadOnce,
#if defined(FSL_FEATURE_FLASH_HAS_READ_RESOURCE_CMD) && FSL_FEATURE_FLASH_HAS_READ_RESOURCE_CMD
    .flash_read_resource = FLASH_ReadResource,
#else
    .flash_read_resource = NULL,
#endif
#else
    .flash_program_once = NULL,
    .flash_read_once = NULL,
    .flash_read_resource = NULL,
#endif
#if BL_TARGET_FLASH
    .flash_prepare_execute_in_ram_functions = FLASH_PrepareExecuteInRamFunctions,
#else
    .flash_prepare_execute_in_ram_functions = NULL,
#endif
    .flash_is_execute_only = FLASH_IsExecuteOnly,
#if BL_FEATURE_FAC_ERASE
    .flash_erase_all_execute_only_segments = FLASH_EraseAllExecuteOnlySegments,
    .flash_verify_erase_all_execute_only_segments = FLASH_VerifyEraseAllExecuteOnlySegments,
#else
    .flash_erase_all_execute_only_segments = NULL,
    .flash_verify_erase_all_execute_only_segments = NULL,
#endif
#if BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED
#if FSL_FEATURE_FLASH_HAS_SET_FLEXRAM_FUNCTION_CMD
    .flash_set_flexram_function = FLASH_SetFlexramFunction,
#else
    .flash_set_flexram_function = NULL,
#endif
    .flash_program_section = FLASH_ProgramSection,
#else
    .flash_set_flexram_function = NULL,
    .flash_program_section = NULL,
#endif
};
#endif // BL_FEATURE_HAS_INTERNAL_FLASH

const aes_driver_interface_t g_aesInterface = {
#if AES_SECURITY_SUPPORTED
    .aes_init = aes_init, .aes_encrypt = aes_encrypt, .aes_decrypt = aes_decrypt
};
#else
    0
};
#endif

//! @brief Copyright string for the bootloader.
const char bootloaderCopyright[] = "Copyright (c) 2013-2016 Freescale Semiconductor, Inc. All rights reserved.";

//! @brief Static API tree.
const bootloader_tree_t g_bootloaderTree = {.runBootloader = bootloader_user_entry,
                                            .version = {.name = kBootloader_Version_Name,
                                                        .major = kBootloader_Version_Major,
                                                        .minor = kBootloader_Version_Minor,
                                                        .bugfix = kBootloader_Version_Bugfix },
                                            .copyright = bootloaderCopyright,
                                            .runtimeContext = &g_bootloaderContext,
#if BL_FEATURE_HAS_INTERNAL_FLASH
                                            .flashDriver = &g_flashDriverInterface,
#else
                                            .flashDriver = NULL,
#endif // BL_FEATURE_HAS_INTERNAL_FLASH
                                            .aesDriver = &g_aesInterface };

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
