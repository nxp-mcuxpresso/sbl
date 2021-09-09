/*
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP.
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "property/property.h"
#include "memory/memory.h"
#include "packet/command_packet.h"
#include "packet/serial_packet.h"
#include "bootloader/bl_peripheral.h"
#include "bootloader/bl_context.h"
#include "bootloader/bl_version.h"
#include "utilities/fsl_assert.h"
#include <string.h>
#include "flash/fsl_flash.h"
#include "fsl_device_registers.h"
#if BL_FEATURE_QSPI_MODULE
#include "qspi/qspi.h"
#endif // BL_FEATURE_QSPI_MODULE
#if BL_FEATURE_CRC_CHECK
#include "bootloader/bl_app_crc_check.h"
#endif // BL_FEATURE_CRC_CHECK

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

#if (defined(__ICCARM__)) // IAR
#pragma section = ".intvec"
#pragma section = "ApplicationFlash"
#pragma section = "ApplicationRam"
#if defined(BL_TARGET_RAM)
#define __RAM_START ((uint32_t)__section_begin(".intvec"))
#else
#define __RAM_START ((uint32_t)__section_begin("ApplicationRam"))
#endif // #if defined(BL_TARGET_RAM)
#define __RAM_END ((uint32_t)__section_end("ApplicationRam") - 1)
#define __ROM_START ((uint32_t)__section_begin(".intvec"))
#define __ROM_END ((uint32_t)__section_end("ApplicationFlash"))
#elif(defined(__CC_ARM)) // MDK
extern uint32_t Image$$VECTOR_ROM$$Base[];
extern uint32_t Image$$ER_m_text$$Limit[];
extern char Image$$VECTOR_RAM$$Base[];
extern uint32_t Image$$ARM_LIB_STACK$$ZI$$Limit[];
#define __RAM_START ((uint32_t)Image$$VECTOR_RAM$$Base)
#define __RAM_END ((uint32_t)Image$$ARM_LIB_STACK$$ZI$$Limit - 1)
#define __ROM_START ((uint32_t)Image$$VECTOR_ROM$$Base)
#define __ROM_END ((uint32_t)Image$$ER_m_text$$Limit)
#elif(defined(__GNUC__)) // GCC
extern uint32_t __VECTOR_RAM[];
extern uint32_t __VECTOR_TABLE[];
extern char __DATA_END[];
extern uint32_t __STACK_TOP[];
#define __RAM_START ((uint32_t)__VECTOR_RAM)
#define __RAM_END ((uint32_t)__STACK_TOP - 1)
#define __ROM_START ((uint32_t)__VECTOR_TABLE)
#define __ROM_END ((uint32_t)__DATA_END)
#else
#error Unknown toolchain!
#endif // __ICCARM__

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
typedef struct KinetisUniqueDeviceId
{
    uint32_t uid[kUid_MaxSizeInbytes / sizeof(uint32_t)];
    uint32_t uidl;
#if defined(SIM_UIDM_UID)
    uint32_t uidm;
#else
    uint32_t uidml;
    uint32_t uidmh;
#endif // defined(SIM_UIDM)
#if defined(BOOTLOADER_HOST) | defined(SIM_UIDH) | defined(SIM_UIDH_UID)
    uint32_t uidh;
#endif
} kinetis_unique_device_id_t;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Storage for property values.
property_store_t g_propertyStore;
//! @brief Map for external memory property interface.
extern const external_memory_property_interface_t g_externalMemPropertyInterfaceMap[];

// See property.h for documentation on this data structure.
const property_interface_t g_propertyInterface = { bootloader_property_load_user_config, bootloader_property_init,
                                                   bootloader_property_get, bootloader_property_set_uint32,
                                                   &g_propertyStore };

//! @brief Storage for property values computed every time they are read.
static uint32_t s_propertyReturnValue;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
// !@brief Get external memoery proporties
status_t bootloader_get_external_memory_properties(uint32_t memoryId, external_memory_property_store_t *store);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See property.h for documentation on this function.
status_t bootloader_property_load_user_config(void)
{
    bootloader_configuration_data_t *config = &g_bootloaderContext.propertyInterface->store->configurationData;

#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    if (is_in_execute_only_region(kBootloaderConfigAreaAddress, sizeof(bootloader_configuration_data_t)))
    {
        memset(config, 0xff, sizeof(bootloader_configuration_data_t));
    }
    else
#endif // FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    {
        // Copy bootloader configuration data from the flash into the property store.
        memcpy(config, (const void *)kBootloaderConfigAreaAddress, sizeof(bootloader_configuration_data_t));

        // Verify tag. If it is invalid, wipe the config data to all 0xff.
        if (kPropertyStoreTag != config->tag)
        {
            memset(config, 0xff, sizeof(bootloader_configuration_data_t));
        }
    }

    // Update available peripherals based on specific chips
    update_available_peripherals();

    return kStatus_Success;
}

// See property.h for documentation on this function.
status_t bootloader_property_init(void)
{
    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;

    // Fill in default values.
    propertyStore->bootloaderVersion.name = kBootloader_Version_Name;
    propertyStore->bootloaderVersion.major = kBootloader_Version_Major;
    propertyStore->bootloaderVersion.minor = kBootloader_Version_Minor;
    propertyStore->bootloaderVersion.bugfix = kBootloader_Version_Bugfix;

    propertyStore->serialProtocolVersion.name = kSerialProtocol_Version_Name;
    propertyStore->serialProtocolVersion.major = kSerialProtocol_Version_Major;
    propertyStore->serialProtocolVersion.minor = kSerialProtocol_Version_Minor;
    propertyStore->serialProtocolVersion.bugfix = kSerialProtocol_Version_Bugfix;

    propertyStore->targetVersion.name = kTarget_Version_Name;
    propertyStore->targetVersion.major = kTarget_Version_Major;
    propertyStore->targetVersion.minor = kTarget_Version_Minor;
    propertyStore->targetVersion.bugfix = kTarget_Version_Bugfix;

    propertyStore->verifyWrites = true;

    propertyStore->availableCommands = kAvailableCommands;

    /// Initialize flash properties.
    uint8_t flashIndex = kFlashIndex_Main;
#if BL_HAS_SECONDARY_INTERNAL_FLASH
    for (; flashIndex <= kFlashIndex_Secondary; flashIndex++)
#endif
    {
        g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[flashIndex],
                                                                     kFLASH_PropertyPflashBlockBaseAddr,
                                                                     &propertyStore->flashStartAddress[flashIndex]);
        g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[flashIndex],
                                                                     kFLASH_PropertyPflashTotalSize,
                                                                     &propertyStore->flashSizeInBytes[flashIndex]);
        g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[flashIndex],
                                                                     kFLASH_PropertyPflashSectorSize,
                                                                     &propertyStore->flashSectorSize[flashIndex]);
        g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[flashIndex],
                                                                     kFLASH_PropertyPflashBlockSize,
                                                                     &propertyStore->flashBlockSize[flashIndex]);
        g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[flashIndex],
                                                                     kFLASH_PropertyPflashBlockCount,
                                                                     &propertyStore->flashBlockCount[flashIndex]);
        g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[flashIndex],
                                                                     kFLASH_PropertyPflashFacSupport,
                                                                     &propertyStore->flashFacSupport[flashIndex]);
        g_bootloaderContext.flashDriverInterface->flash_get_property(
            &g_bootloaderContext.allFlashState[flashIndex], kFLASH_PropertyPflashAccessSegmentSize,
            &propertyStore->flashAccessSegmentSize[flashIndex]);
        g_bootloaderContext.flashDriverInterface->flash_get_property(
            &g_bootloaderContext.allFlashState[flashIndex], kFLASH_PropertyPflashAccessSegmentCount,
            &propertyStore->flashAccessSegmentCount[flashIndex]);
    }

    // Fill in reserved regions.
    //! @todo Support other tool chain

    uint32_t flashStart = 0;
    uint32_t flashEnd = 0;
    uint32_t ramStart = 0;
    uint32_t ramEnd = 0;

#if !BL_TARGET_FLASH
    flashStart = (&g_bootloaderContext.memoryMap[kIndexFlashArray])->startAddress;
    flashEnd = (&g_bootloaderContext.memoryMap[kIndexFlashArray])->startAddress;
#else
    flashStart = __ROM_START;
    flashEnd = __ROM_END;
    assert(flashEnd);

    // Force flash erase size alignment.
    // Note: Assume that flash-resident bootloader is always in Main flash.
    flashStart = ALIGN_DOWN(flashStart, propertyStore->flashSectorSize[kFlashIndex_Main]);
    flashEnd = ALIGN_UP(flashEnd, propertyStore->flashSectorSize[kFlashIndex_Main]) - 1;
#endif
    ramStart = __RAM_START;
    ramEnd = __RAM_END;
    assert(ramEnd);

    propertyStore->reservedRegions[kProperty_FlashReservedRegionIndex].startAddress = flashStart;
    propertyStore->reservedRegions[kProperty_FlashReservedRegionIndex].endAddress = flashEnd;
    propertyStore->reservedRegions[kProperty_RamReservedRegionIndex].startAddress = ramStart;
    propertyStore->reservedRegions[kProperty_RamReservedRegionIndex].endAddress = ramEnd;

    // Fill in available peripherals array.
    const peripheral_descriptor_t *peripherals = g_bootloaderContext.allPeripherals;
    propertyStore->availablePeripherals = 0;
    for (uint32_t i = 0; peripherals[i].typeMask != 0; ++i)
    {
        // Check that the peripheral is enabled in the user configuration data.
        if (propertyStore->configurationData.enabledPeripherals & peripherals[i].typeMask)
        {
            propertyStore->availablePeripherals |= peripherals[i].typeMask;
        }
    }

    // Fill in unique device id value.
    // Different series have different length of UID (K series=128 bits, KL series=80 bits)
    kinetis_unique_device_id_t *kinetis_uid = (kinetis_unique_device_id_t *)&propertyStore->UniqueDeviceId;
#if (!FSL_FEATURE_SIM_HAS_NO_UID) && defined(SIM)
#if defined(SIM_UIDH) | defined(SIM_UIDH_UID)
    kinetis_uid->uidh = SIM->UIDH;
#endif
#if defined(SIM_UIDM_UID)
    kinetis_uid->uidm = SIM->UIDM;
#else
    kinetis_uid->uidmh = SIM->UIDMH;
    kinetis_uid->uidml = SIM->UIDML;
#endif // defined(SIM_UIDM)
    kinetis_uid->uidl = SIM->UIDL;
#endif // #if !FSL_FEATURE_SIM_HAS_NO_UID

    // Set address range of RAM in property interface
    const memory_map_entry_t *map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexSRAM];
    propertyStore->ramStartAddress[kPropertyIndex_SRAM] = map->startAddress;
    propertyStore->ramSizeInBytes[kPropertyIndex_SRAM] = map->endAddress - map->startAddress + 1;
#if CPU_IS_ARM_CORTEX_M7
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexDTCM];
    propertyStore->ramStartAddress[kPropertyIndex_DTCM] = map->startAddress;
    propertyStore->ramSizeInBytes[kPropertyIndex_DTCM] = map->endAddress - map->startAddress + 1;
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexOCRAM];
    propertyStore->ramStartAddress[kPropertyIndex_OCRAM] = map->startAddress;
    propertyStore->ramSizeInBytes[kPropertyIndex_OCRAM] = map->endAddress - map->startAddress + 1;

#endif

#if BL_FEATURE_CRC_CHECK
    // Initialize crc check status property based on BCA related fields.
    init_crc_check_status(propertyStore);
#endif

    // Fill in default margin level.
    propertyStore->flashReadMargin = (uint32_t)kFLASH_MarginValueUser;

#if BL_FEATURE_QSPI_MODULE
    propertyStore->qspiInitStatus = get_qspi_otfad_init_status();
#endif // BL_FEATURE_QSPI_MODULE

    return kStatus_Success;
}

// See property.h for documentation on this function.
status_t bootloader_property_get(uint8_t tag, uint32_t id, const void **value, uint32_t *valueSize)
{
    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;

    uint32_t flashIndex = kFlashIndex_Main;
#if BL_HAS_SECONDARY_INTERNAL_FLASH
    if (id < kFLASHCount)
    {
        flashIndex = id;
    }
#endif // BL_HAS_SECONDARY_INTERNAL_FLASH

    // Set default value size, may be modified below.
    uint32_t returnSize = sizeof(uint32_t);
    const void *returnValue;
    switch (tag)
    {
        case kPropertyTag_BootloaderVersion:
            returnValue = &propertyStore->bootloaderVersion.version;
            break;

        case kPropertyTag_AvailablePeripherals:
            returnValue = &propertyStore->availablePeripherals;
            break;

        case kPropertyTag_FlashStartAddress:
            returnValue = &propertyStore->flashStartAddress[flashIndex];
            break;

        case kPropertyTag_FlashSizeInBytes:
            returnValue = &propertyStore->flashSizeInBytes[flashIndex];
            break;

        case kPropertyTag_FlashSectorSize:
            returnValue = &propertyStore->flashSectorSize[flashIndex];
            break;

        case kPropertyTag_FlashBlockCount:
            returnValue = &propertyStore->flashBlockCount[flashIndex];
            break;

        case kPropertyTag_RAMStartAddress:
#if CPU_IS_ARM_CORTEX_M7
            if (id >= kRAMCount)
            {
                returnValue = &propertyStore->ramStartAddress[0];
            }
            else
            {
                returnValue = &propertyStore->ramStartAddress[id];
            }
#else
            returnValue = &propertyStore->ramStartAddress[0];
#endif // CPU_IS_ARM_CORTEX_M7
            break;

        case kPropertyTag_RAMSizeInBytes:
#if CPU_IS_ARM_CORTEX_M7
            if (id >= kRAMCount)
            {
                returnValue = &propertyStore->ramSizeInBytes[0];
            }
            else
            {
                returnValue = &propertyStore->ramSizeInBytes[id];
            }
#else
            returnValue = &propertyStore->ramSizeInBytes[0];
#endif // CPU_IS_ARM_CORTEX_M7
            break;

        case kPropertyTag_AvailableCommands:
            returnValue = &propertyStore->availableCommands;
            break;

#if BL_FEATURE_CRC_CHECK
        case kPropertyTag_CrcCheckStatus:
            returnValue = &propertyStore->crcCheckStatus;
            break;
#endif // else falls through to unknown

        case kPropertyTag_VerifyWrites:
            returnValue = &propertyStore->verifyWrites;
            break;

        case kPropertyTag_MaxPacketSize:
            // Read the max packet size from the active peripheral.
            s_propertyReturnValue = g_bootloaderContext.activePeripheral->packetInterface->getMaxPacketSize(
                g_bootloaderContext.activePeripheral);
            returnValue = &s_propertyReturnValue;
            break;

        case kPropertyTag_ReservedRegions:
            returnSize = sizeof(propertyStore->reservedRegions);
            returnValue = propertyStore->reservedRegions;
            break;

#if !FSL_FEATURE_SIM_HAS_NO_SDID && defined(SIM)
        case kPropertyTag_SystemDeviceId:
            s_propertyReturnValue = SIM->SDID;
            returnValue = &s_propertyReturnValue;
            break;
#endif // #if !FSL_FEATURE_SIM_HAS_NO_SDID

        case kPropertyTag_FlashSecurityState:
        {
            // Note: Both Main and Secondary flash share the same security state
            //  So it doesn't matter what index of allFlashState[] we use for this FLASH API.
            flash_security_state_t securityState;
            g_bootloaderContext.flashDriverInterface->flash_get_security_state(
                &g_bootloaderContext.allFlashState[kFlashIndex_Main], &securityState);
            s_propertyReturnValue = (securityState != kFLASH_SecurityStateNotSecure);

            returnValue = &s_propertyReturnValue;
            break;
        }

        case kPropertyTag_UniqueDeviceId:
            returnSize = sizeof(kinetis_unique_device_id_t);
            returnValue = &propertyStore->UniqueDeviceId;
            break;

        case kPropertyTag_FacSupport:
            returnSize = sizeof(propertyStore->flashFacSupport[flashIndex]);
            returnValue = &propertyStore->flashFacSupport[flashIndex];
            break;

        case kPropertyTag_FlashAccessSegmentSize:
            returnSize = sizeof(propertyStore->flashAccessSegmentSize[flashIndex]);
            returnValue = &propertyStore->flashAccessSegmentSize[flashIndex];
            break;

        case kPropertyTag_FlashAccessSegmentCount:
            returnSize = sizeof(propertyStore->flashAccessSegmentCount[flashIndex]);
            returnValue = &propertyStore->flashAccessSegmentCount[flashIndex];
            break;

        case kPropertyTag_FlashReadMargin:
            // Note: Currently both Main and Secondary flash share the same flash read margin
            returnSize = sizeof(propertyStore->flashReadMargin);
            returnValue = &propertyStore->flashReadMargin;
            break;

#if BL_FEATURE_QSPI_MODULE
        case kPropertyTag_QspiInitStatus:
            returnValue = &propertyStore->qspiInitStatus;
            break;
#endif // else falls through to unknown

        case kPropertyTag_TargetVersion:
            returnValue = &propertyStore->targetVersion.version;
            break;
#if BL_FEATURE_EXTERNAL_MEMORY_PROPERTY
        case kPropertyTag_ExternalMemoryAttributes:
        {
            status_t status =
                bootloader_get_external_memory_properties(id, &propertyStore->externalMemoryPropertyStore);
            if (status != kStatus_Success)
            {
                return status;
            }
        }
            returnSize = sizeof(propertyStore->externalMemoryPropertyStore);
            returnValue = &propertyStore->externalMemoryPropertyStore;
            break;
#endif // BL_FEATURE_EXTERNAL_MEMORY_PROPERTY

#if BL_FEATURE_RELIABLE_UPDATE
        case kPropertyTag_ReliableUpdateStatus:
            returnValue = &propertyStore->reliableUpdateStatus;
            break;
#endif // BL_FEATURE_RELIABLE_UPDATE

        default:
            return kStatus_UnknownProperty;
    }

    // Set the return size.
    if (valueSize)
    {
        *valueSize = returnSize;
    }

    // Set the return value
    if (value)
    {
        *value = returnValue;
    }

    return kStatus_Success;
}

// See property.h for documentation on this function.
status_t bootloader_property_set_uint32(uint8_t tag, uint32_t value)
{
    status_t status = kStatus_UnknownProperty;
    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;

    switch (tag)
    {
        case kPropertyTag_VerifyWrites:
            if (value != 0 && value != 1)
            {
                status = kStatus_InvalidPropertyValue;
            }
            else
            {
                propertyStore->verifyWrites = value;
                status = kStatus_Success;
            }
            break;

        case kPropertyTag_FlashReadMargin:
            if (value >= kFLASH_MarginValueInvalid)
            {
                status = kStatus_InvalidPropertyValue;
            }
            else
            {
                propertyStore->flashReadMargin = value;
                status = kStatus_Success;
            }
            break;

        case kPropertyTag_BootloaderVersion:
        case kPropertyTag_AvailablePeripherals:
        case kPropertyTag_FlashStartAddress:
        case kPropertyTag_FlashSizeInBytes:
        case kPropertyTag_FlashSectorSize:
        case kPropertyTag_FlashBlockCount:
        case kPropertyTag_RAMStartAddress:
        case kPropertyTag_RAMSizeInBytes:
        case kPropertyTag_AvailableCommands:
#if BL_FEATURE_CRC_CHECK
        case kPropertyTag_CrcCheckStatus:
#endif
#if BL_FEATURE_QSPI_MODULE
        case kPropertyTag_QspiInitStatus:
#endif
#if BL_FEATURE_EXTERNAL_MEMORY_PROPERTY
        case kPropertyTag_ExternalMemoryAttributes:
#endif // BL_FEATURE_EXTERNAL_MEMORY_PROPERTY
#if BL_FEATURE_RELIABLE_UPDATE
        case kPropertyTag_ReliableUpdateStatus:
#endif // BL_FEATURE_RELIABLE_UPDATE
        case kPropertyTag_MaxPacketSize:
        case kPropertyTag_ReservedRegions:
        case kPropertyTag_SystemDeviceId:
        case kPropertyTag_FlashSecurityState:
        case kPropertyTag_UniqueDeviceId:
        case kPropertyTag_FacSupport:
        case kPropertyTag_FlashAccessSegmentSize:
        case kPropertyTag_FlashAccessSegmentCount:
        case kPropertyTag_TargetVersion:
            status = kStatus_ReadOnlyProperty;
            break;
        default:
            status = kStatus_UnknownProperty;
            break;
    }

    return status;
}

#if BL_FEATURE_EXTERNAL_MEMORY_PROPERTY
status_t bootloader_get_external_memory_properties(uint32_t memoryId, external_memory_property_store_t *store)
{
    extern const external_memory_property_interface_t g_externalMemPropertyInterfaceMap[];

    // Find external memory property interface map.
    const external_memory_property_interface_t *map = &g_externalMemPropertyInterfaceMap[0];
    while (map && map->get)
    {
        if (map->memoryId == memoryId)
        {
            break;
        }
        map++;
    }

    if (map->get == NULL)
    {
        return kStatus_InvalidArgument;
    }

    external_memory_property_store_t propertyStore;
    uint32_t memoryInitStatus;
    map->get(kExternalMemoryPropertyTag_InitStatus, &memoryInitStatus);
    if (memoryInitStatus != kStatus_Success)
    {
        return memoryInitStatus;
    }

    uint32_t *property = (uint32_t *)&propertyStore.startAddress;
    propertyStore.availableAttributesFlag = 0;
    for (uint32_t tag = kExternalMemoryPropertyTag_Start; tag <= kExternalMemoryPropertyTag_End; tag++)
    {
        uint32_t tmp = 0;
        status_t status = map->get(tag, &tmp);
        if (status == kStatus_Success)
        {
            *property = tmp;
            propertyStore.availableAttributesFlag |= 1 << (tag - 1);
        }
        else
        {
            *property = 0;
        }

        property++;
    }

    memcpy(store, &propertyStore, sizeof(propertyStore));

    return kStatus_Success;
}
#endif // BL_FEATURE_EXTERNAL_MEMORY_PROPERTY

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
