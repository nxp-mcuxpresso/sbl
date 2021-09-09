/*
 * Copyright (c) 2014-2016, Freescale Semiconductor, Inc.
 * Copyright 2018 NXP.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "bootloader_common.h"
#include "bootloader/bootloader.h"
#include "memory/memory.h"
#if !BL_FEATURE_MIN_PROFILE
#include "sbloader/sbloader.h"
#endif
#include "property/property.h"
#include "utilities/fsl_assert.h"
#if BL_FEATURE_QSPI_MODULE
#include "qspi/qspi.h"
#endif
#if BL_FEATURE_OTFAD_MODULE
#include "otfad/fsl_otfad_driver.h"
#endif
#if BL_FEATURE_CRC_CHECK
#include "bootloader/bl_app_crc_check.h"
#endif
#if BL_FEATURE_RELIABLE_UPDATE
#include "bootloader/bl_reliable_update.h"
#endif
#include <string.h>
#include <stdint.h>
#include "fsl_device_registers.h"
#if BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_FLASH_TYPE_KINETIS_C90TFS_FLASH || BL_FLASH_TYPE_LPC_C040HD_FLASH
#include "fsl_iap.h"
#elif BL_FLASH_TYPE_LPC_IP2113_FLASH
#include "flashiap_wrapper/fsl_flashiap_wrapper.h"
#endif
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#define FTFx_FOPT_BOOTSRCSEL_MASK 0xC0U  //!< bit mask for FOPT[BOOTSRC_SEL]
#define FTFx_FOPT_BOOTSRCSEL_SHIFT 0x06U //!< shift bit for FOPT[BOOTSRC_SEL]

//! @brief Get FOPT value from FTFx Module
#if defined(FTFA)
#define FTFx_FOPT FTFA->FOPT
#elif defined(FTFL)
#define FTFx_FOPT FTFL->FOPT
#elif defined(FTFE)
#define FTFx_FOPT FTFE->FOPT
#endif

enum
{
    //! @brief Boot Source indicating that code is running from rom, needn't to configure QSPI module
    kBootSource_ROM = 0x03,
    //! @brief Boot source indicating that code is running from rom, need to configure QSPI module
    kBootSource_QSPI = 0x02,

    //! @brief OTFAD module instance number
    kOtfadInstance = 0,

    //! @brief Address of key blob array on internal flash
    kKeyBlobAddress = 0x410
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
#if BL_FEATURE_QSPI_MODULE
status_t s_qspi_otfad_init_status = kStatus_QspiNotConfigured;
#endif // #if BL_FEATURE_QSPI_MODULE

#if BL_FEATURE_OTFAD_MODULE
static bool s_isOtfadEnabled = false;
#endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
#if !BL_FEATURE_TIMEOUT
extern void get_user_application_entry(uint32_t *appEntry, uint32_t *appStack);
extern void jump_to_application(uint32_t applicationAddress, uint32_t stackPointer);
extern bool is_direct_boot(void);
extern bool is_application_ready_for_executing(uint32_t applicationAddress);
#endif // !BL_FEATURE_TIMEOUT

#if BL_FEATURE_OTFAD_MODULE
static status_t get_otfad_key(otfad_kek_t *kek);
#endif // BL_FEATURE_OTFAD_MODULE

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See bootloader_common.h for documentation on this function
bool qspi_need_configure(void)
{
#if BL_FEATURE_QSPI_MODULE
    return true;
#else
    return false;
#endif
}

// See bootloader_common.h for documentation on this function
#if BL_FEATURE_OTFAD_MODULE
status_t otfad_init_as_needed(void)
{
    uint32_t keyBlobAddress;

    // Initialize OTFAD module, if present.
    if (is_otfad_present())
    {
        otfad_kek_t otfadKek;
        status_t status = get_otfad_key(&otfadKek);
        if (status == kStatus_OtfadInvalidKey)
        {
            // KEK was not programmed, so must assume code is not
            // encrypted on QSPI. Bypass OTFAD.
            return kStatus_Success;
        }

        // Reload the BCA in case the keyBlobPointer in BCA has been updated.
        g_bootloaderContext.propertyInterface->load_user_config();
        // If we have a valid key blob address in the BCA use that
        if (g_bootloaderContext.propertyInterface->store->configurationData.keyBlobPointer != ~0)
        {
            keyBlobAddress = g_bootloaderContext.propertyInterface->store->configurationData.keyBlobPointer;
        }
        // Otherwise use the default
        else
        {
            keyBlobAddress = kKeyBlobAddress;
        }

        status = otfad_init(kOtfadInstance, (uint8_t *)keyBlobAddress, &otfadKek);

        // Clear otfadKek memory on stack.
        memset(&otfadKek, 0, sizeof(otfadKek));

        if (status == kStatus_Success)
        {
            s_isOtfadEnabled = true;
        }

        return status;
    }
    else
    {
        return kStatus_Success;
    }
}

// See bootloader_common.h for documentation on this function
status_t otfad_bypass_as_needed(void)
{
    if (s_isOtfadEnabled)
    {
        otfad_bypass(kOtfadInstance);
    }

    return kStatus_Success;
}

// See bootloader_common.h for documentation on this function
status_t oftfad_resume_as_needed(void)
{
    if (s_isOtfadEnabled)
    {
        otfad_resume(kOtfadInstance);
    }

    return kStatus_Success;
}

#endif // BL_FEATURE_OTFAD_MODULE

// See bootloader_common.h for documentation on this function
bool is_qspi_present(void)
{
#if BL_FEATURE_QSPI_MODULE
    return is_quadspi_configured();
#else
    return false;
#endif // BL_FEATURE_QSPI_MODULE
}

// See bootloader_common.h for documentation on this function
bool is_otfad_present(void)
{
#if defined(K80F25615_SERIES) || defined(K81F25615_SERIES) || defined(K82F25615_SERIES)
    uint8_t subfamily_id = (SIM->SDID & SIM_SDID_SUBFAMID_MASK) >> SIM_SDID_SUBFAMID_SHIFT;

    // OTFAD is only available on K81 & K82 parts.
    return (subfamily_id > 0);
#else
    return false;
#endif // defined(CPU_MK80FN256VLQR15)
}

// See bootloader_common.h for documentation on this function
bool is_ltc_present(void)
{
#ifdef BL_FEATURE_ENCRYPTION_MMCAU
    return false;
#elif defined(K80F25615_SERIES) || defined(K81F25615_SERIES) || defined(K82F25615_SERIES)
    uint8_t subfamily_id = (SIM->SDID & SIM_SDID_SUBFAMID_MASK) >> SIM_SDID_SUBFAMID_SHIFT;

    // LTC is only available on K81/2 and KL81/2 devices.
    return ((subfamily_id == 1) || (subfamily_id == 2));
#elif FSL_FEATURE_SIM_HAS_MISCCTRL_LTCEN
    return (SIM->MISCCTRL & SIM_MISCCTRL_LTCEN_MASK);
#else
    return false;
#endif // BL_FEATURE_ENCRYPTION_MMCAU
}

// See bootloader_common.h for documentation on this function
bool is_flexspi_nor_present(void)
{
#if BL_FEATURE_FLEXSPI_NOR_MODULE
    return is_flexspi_nor_configured();
#else
    return false;
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE
}

// See bootloader_common.h for documentation on this function
bool is_semc_nor_present(void)
{
#if BL_FEATURE_SEMC_NOR_MODULE
    return is_semc_nor_configured();
#else
    return false;
#endif // BL_FEATURE_SEMC_NOR_MODULE
}

//! @brief Read OTFAD KEK from SIM module.
//!
//! OTFAD key blob decription key (KEK) was read from IFR by hardware and placed
//! in SIM_SECKEY. It is only readable by ROM. All 0xF's means invalid because not programmed.
//! All 0x0's means invalid because security violation detected.
//!
//! @param kek Pointer to returned KEK
//! @retval kStatus_Success Key is valid
//! @retval kStatus_OtfadSecurityViolation Key is all 0
//! @retval kStatus_Fail Key is all 0xF (not programmed)
#if BL_FEATURE_OTFAD_MODULE
status_t get_otfad_key(otfad_kek_t *kek)
{
#if BL_TARGET_FLASH
    assert(kek);

    const uint32_t *kekTable = (const uint32_t *)BL_FEATURE_OTFAD_KEK_ADDRESS;

    kek->keyWord0 = kekTable[0];
    kek->keyWord1 = kekTable[1];
    kek->keyWord2 = kekTable[2];
    kek->keyWord3 = kekTable[3];

    if (!kek->keyWord0 && !kek->keyWord1 && !kek->keyWord2 && !kek->keyWord3)
    {
        // Keys are all 0.
        return kStatus_OtfadSecurityViolation;
    }
    else if (!~kek->keyWord0 && !~kek->keyWord1 && !~kek->keyWord2 && !~kek->keyWord3)
    {
        // Keys are all 0xF.
        return kStatus_OtfadInvalidKey;
    }
    else
    {
        return kStatus_Success;
    }
#else
    return kStatus_Success;
#endif // BL_TARGET_FLASH
}
#endif // BL_HAS_OTFAD_MODULE

#if BL_FEATURE_QSPI_MODULE
//! @brief Return status for intializing qspi and otfad modules
status_t get_qspi_otfad_init_status(void)
{
    return s_qspi_otfad_init_status;
}

//!@bief Update status for intializing qspi and otfad modules
void update_qspi_otfad_init_status(status_t initStatus)
{
    s_qspi_otfad_init_status = initStatus;
    g_bootloaderContext.propertyInterface->store->qspiInitStatus = s_qspi_otfad_init_status;
}
#endif // #if BL_FEATURE_QSPI_MODULE

bool is_in_execute_only_region(uint32_t start, uint32_t lengthInBytes)
{
#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    flash_execute_only_access_state_t state = kFLASH_AccessStateUnLimited;
    g_bootloaderContext.flashDriverInterface->flash_is_execute_only(
        &g_bootloaderContext.allFlashState[kFlashIndex_Main], start, lengthInBytes, &state);
    if (state == kFLASH_AccessStateUnLimited)
    {
#if BL_HAS_SECONDARY_INTERNAL_FLASH
        g_bootloaderContext.flashDriverInterface->flash_is_execute_only(
            &g_bootloaderContext.allFlashState[kFlashIndex_Secondary], start, lengthInBytes, &state);
        if (state != kFLASH_AccessStateUnLimited)
        {
            return true;
        }
        else
#endif // BL_HAS_SECONDARY_INTERNAL_FLASH
        {
            return false;
        }
    }
    else
    {
        return true;
    }
#else
    return false;
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
}

#if !BL_FEATURE_TIMEOUT
//! @brief Returns the user application address and stack pointer.
//!
//! For flash-resident and rom-resident target, gets the user application address
//! and stack pointer from the APP_VECTOR_TABLE.
//! Ram-resident version does not support jumping to application address.
void get_user_application_entry(uint32_t *appEntry, uint32_t *appStack)
{
    assert(appEntry);
    assert(appStack);

#if BL_TARGET_RAM
    *appEntry = 0;
    *appStack = 0;
#else
#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    // Check if address of SP and PC is in an execute-only region.
    if (!is_in_execute_only_region(kDefaultVectorTableAddress, 8))
    {
        *appEntry = APP_VECTOR_TABLE[kInitialPC];
        *appStack = APP_VECTOR_TABLE[kInitialSP];
    }
    else
    {
        // Set to invalid value when vector table is in execute-only region,
        // as ROM doesn't support jumping to an application in such region so far.
        // The main purpose of below operation is to prevent ROM from inifinit loop
        // between NVIC_SystemReset() and fetching SP and PC frome execute-only region.
        *appEntry = 0;
        *appStack = 0;
    }
#else
    *appEntry = APP_VECTOR_TABLE[kInitialPC];
    *appStack = APP_VECTOR_TABLE[kInitialSP];
#endif //  FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
#endif // BL_TARGET_RAM
}
#endif // BL_FEATURE_TIMEOUT

#if !BL_FEATURE_TIMEOUT
bool is_direct_boot(void)
{
    bootloader_configuration_data_t *configurationData =
        &g_bootloaderContext.propertyInterface->store->configurationData;

    return (~configurationData->bootFlags) & kBootFlag_DirectBoot;
}
#endif // !BL_FEATURE_TIMEOUT

#if !BL_FEATURE_TIMEOUT
//! @brief Exits bootloader and jumps to the user application.
void jump_to_application(uint32_t applicationAddress, uint32_t stackPointer)
{
#if BL_FEATURE_OTFAD_MODULE
    quadspi_cache_clear();
    oftfad_resume_as_needed();
#endif

    shutdown_cleanup(kShutdownType_Shutdown);

    // Create the function call to the user application.
    // Static variables are needed since changed the stack pointer out from under the compiler
    // we need to ensure the values we are using are not stored on the previous stack
    static uint32_t s_stackPointer = 0;
    s_stackPointer = stackPointer;
    static void (*farewellBootloader)(void) = 0;
    farewellBootloader = (void (*)(void))applicationAddress;

    // Set the VTOR to the application vector table address.
    SCB->VTOR = (uint32_t)APP_VECTOR_TABLE;

    // Set stack pointers to the application stack pointer.
    __set_MSP(s_stackPointer);
    __set_PSP(s_stackPointer);

    // Jump to the application.
    farewellBootloader();

    // Code should never go here
    __NOP();
}
#endif // !BL_FEATURE_TIMEOUT

//! A given jump address is considered valid if:
//! - Not 0x00000000
//! - Not 0xffffffff
//! - Not the reset handler entry point for the bootloader
//! - Is in flash or is in RAM or QuadSPI (if available)
//! @note this interface is also used by the configure_quadspi command
bool is_valid_application_location(uint32_t applicationAddress)
{
    const memory_map_entry_t *map;
    // Verify that the jumpLocation is non zero and then either within flash or RAM, both calculations are:
    // (jumpLocation >= startAddress) && (jumpLocation < (startAddress + size))
    if ((!applicationAddress) ||              // address is not null AND
        (applicationAddress == 0xffffffff) || // address is not blank Flash (0xff) AND
        (applicationAddress == (uint32_t)&Reset_Handler))
    {
        return false;
    }

    bool isValid = false;
    const uint32_t minThumb2InstructionSize = 2; // smallest thumb2 instruction size is 16-bit.
    // Check if the application address is in valid executable memory range
    status_t status = find_map_entry(applicationAddress, minThumb2InstructionSize, &map);
    if ((status == kStatus_Success) && (map->memoryProperty & kMemoryIsExecutable))
    {
        isValid = true;
    }

    return isValid;
}

bool is_valid_stackpointer_location(uint32_t stackpointerAddress)
{
    const memory_map_entry_t *map;
    bool isValid = false;

    map = &g_bootloaderContext.memoryMap[0];

    while (map->memoryInterface != NULL)
    {
        if ((map->memoryProperty & kMemoryIsExecutable) && (map->memoryProperty & kMemoryType_RAM))
        {
            if ((stackpointerAddress > map->startAddress) && (stackpointerAddress <= (map->endAddress + 1)))
            {
                isValid = true;
                break;
            }
        }

        ++map;
    }

    return isValid;
}

#if !BL_FEATURE_TIMEOUT
//! @brief Jump application is considered ready for executing if the location is valid and crc check is passed
bool is_application_ready_for_executing(uint32_t applicationAddress)
{
    bool result = is_valid_application_location(applicationAddress);

#if BL_FEATURE_OTFAD_MODULE
    if (result && is_qspi_present())
    {
        quadspi_cache_clear();
        status_t status = otfad_init_as_needed();
        if (status != kStatus_Success)
        {
            result = false;
        }
        update_qspi_otfad_init_status(status);
    }
#endif

#if BL_FEATURE_CRC_CHECK
    // Validate application crc only if its location is valid
    if (result)
    {
        result = is_application_crc_check_pass();
    }

#if BL_FEATURE_OTFAD_MODULE
    otfad_bypass_as_needed();
#endif // BL_FEATURE_OTFAD_MODULE

#endif

    return result;
}
#endif // !BL_FEATURE_TIMEOUT

#if BL_FEATURE_QSPI_MODULE
void configure_quadspi_as_needed(void)
{
    // Start the lifetime counter
    microseconds_init();
    if (qspi_need_configure())
    {
        status_t qspiOtfadInitStatus = kStatus_QspiNotConfigured;
        // Try to configure QuadSPI module based on on qspi_config_block_pointer in BCA first,
        // If bootloader cannot get qspi config block from internal flash, try to configure QSPI
        // based on default place (start address of QuadSPI memory).
        uint32_t qspi_config_block_base =
            g_bootloaderContext.propertyInterface->store->configurationData.qspi_config_block_pointer;

        // Get the start address and flash size
        // Note: Basically BCA is always stored in Main flash memory
        uint32_t flashStart;
        g_bootloaderContext.flashDriverInterface->flash_get_property(
            &g_bootloaderContext.allFlashState[kFlashIndex_Main], kFLASH_PropertyPflashBlockBaseAddr, &flashStart);
        uint32_t flashSize;
        g_bootloaderContext.flashDriverInterface->flash_get_property(
            &g_bootloaderContext.allFlashState[kFlashIndex_Main], kFLASH_PropertyPflashTotalSize, &flashSize);

        // Check if the pointer of qspi config block is valid.
        if ((qspi_config_block_base != 0xFFFFFFFF) && (qspi_config_block_base > flashStart) &&
            (qspi_config_block_base <= (flashStart + flashSize - sizeof(qspi_config_t))))
        {
#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
            if (!is_in_execute_only_region(qspi_config_block_base, sizeof(qspi_config_t)))
            {
                qspiOtfadInitStatus = quadspi_init((void *)qspi_config_block_base);
            }
#else
            qspiOtfadInitStatus = quadspi_init((void *)qspi_config_block_base);
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
        }

        if (qspiOtfadInitStatus == kStatus_QspiNotConfigured)
        {
            qspiOtfadInitStatus = quadspi_init(NULL);
        }
        update_qspi_otfad_init_status(qspiOtfadInitStatus);
    }
    // Shutdown the lifetime counter before configuring clock.
    lock_acquire();
    microseconds_shutdown();
    lock_release();
}
#endif

#if BL_FEATURE_HAS_INTERNAL_FLASH
#if !BL_DEVICE_IS_LPC_SERIES
void bootloader_flash_init(void)
{
#if BL_TARGET_FLASH
    //! @brief A static buffer used to hold flash_run_command()
    static uint32_t s_flashRunCommand[kFLASH_ExecuteInRamFunctionMaxSizeInWords];
    //! @brief A static buffer used to hold flash_common_bit_operation()
    static uint32_t s_flashCommonBitOperation[kFLASH_ExecuteInRamFunctionMaxSizeInWords];

    static flash_execute_in_ram_function_config_t s_flashExecuteInRamFunctionInfo = {
        .activeFunctionCount = 0,
        .flashRunCommand = s_flashRunCommand,
        .flashCommonBitOperation = s_flashCommonBitOperation,
    };
#endif

    uint8_t flashIndex = kFlashIndex_Main;
#if BL_HAS_SECONDARY_INTERNAL_FLASH
    for (; flashIndex <= kFlashIndex_Secondary; flashIndex++)
#endif
    {
        g_bootloaderContext.flashDriverInterface->flash_init(&g_bootloaderContext.allFlashState[flashIndex]);

#if BL_TARGET_FLASH
        g_bootloaderContext.allFlashState[flashIndex].flashExecuteInRamFunctionInfo =
            &s_flashExecuteInRamFunctionInfo.activeFunctionCount;
        g_bootloaderContext.flashDriverInterface->flash_prepare_execute_in_ram_functions(
            &g_bootloaderContext.allFlashState[flashIndex]);
#endif
    }
}
#endif // !BL_DEVICE_IS_LPC_SERIES
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH

#if defined(__CC_ARM)
#define ITM_Port8(n) (*((volatile unsigned char *)(0xE0000000 + 4 * n)))
#define ITM_Port16(n) (*((volatile unsigned short *)(0xE0000000 + 4 * n)))
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4 * n)))

#define DEMCR (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA 0x01000000

struct __FILE
{
    int handle; /* Add whatever needed */
};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
    if (DEMCR & TRCENA)
    {
        while (ITM_Port32(0) == 0)
            ;
        ITM_Port8(0) = ch;
    }
    return (ch);
}
#endif

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
