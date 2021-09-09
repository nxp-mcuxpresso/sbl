/*
 * Copyright 2017-2018 NXP.
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include "bootloader/bl_context.h"
#include "bootloader/bl_peripheral.h"
#include "bootloader/bl_version.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "memory/memory.h"
#include "packet/command_packet.h"
#include "packet/serial_packet.h"
#include "property/property.h"
#include "utilities/fsl_assert.h"

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

#if (defined(__ICCARM__)) // IAR
#pragma section = ".intvec"
#pragma section = "ApplicationFlash"
#pragma section = "ApplicationRam"
#define __RAM_START ((uint32_t)__section_begin("ApplicationRam"))
#define __RAM_END ((uint32_t)__section_end("ApplicationRam") - 1)
#define __ROM_START ((uint32_t)__section_begin(".intvec"))
#define __ROM_END ((uint32_t)__section_end("ApplicationFlash"))
#elif (defined(__CC_ARM)) // MDK
extern uint32_t Image$$VECTOR_ROM$$Base[];
extern uint32_t Image$$ER_m_text$$Limit[];
extern char Image$$RW_m_data$$Base[];
extern uint32_t Image$$ARM_LIB_STACK$$ZI$$Limit[];
#define __RAM_START ((uint32_t)Image$$RW_m_data$$Base)
#define __RAM_END ((uint32_t)Image$$ARM_LIB_STACK$$ZI$$Limit - 1)
#define __ROM_START ((uint32_t)Image$$VECTOR_ROM$$Base)
#define __ROM_END ((uint32_t)Image$$ER_m_text$$Limit)
#elif (defined(__GNUC__)) // GCC
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
enum
{
    kUniqueId_SizeInBytes = 8,
};

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

    memset(config, 0xff, sizeof(bootloader_configuration_data_t));

    // Update available peripherals based on specific chips
    update_available_peripherals();

    return kStatus_Success;
}

// See property.h for documentation on this function.
status_t bootloader_property_init(void)
{
    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;

    // Fill in default values.
    propertyStore->bootloaderVersion.name = (char)kBootloader_Version_Name;
    propertyStore->bootloaderVersion.major = kBootloader_Version_Major;
    propertyStore->bootloaderVersion.minor = kBootloader_Version_Minor;
    propertyStore->bootloaderVersion.bugfix = kBootloader_Version_Bugfix;

    propertyStore->serialProtocolVersion.name = (char)kSerialProtocol_Version_Name;
    propertyStore->serialProtocolVersion.major = kSerialProtocol_Version_Major;
    propertyStore->serialProtocolVersion.minor = kSerialProtocol_Version_Minor;
    propertyStore->serialProtocolVersion.bugfix = kSerialProtocol_Version_Bugfix;

    propertyStore->targetVersion.name = (char)kTarget_Version_Name;
    propertyStore->targetVersion.major = kTarget_Version_Major;
    propertyStore->targetVersion.minor = kTarget_Version_Minor;
    propertyStore->targetVersion.bugfix = kTarget_Version_Bugfix;

    propertyStore->verifyWrites = true;

    propertyStore->availableCommands = kAvailableCommands;

    // Fill in reserved regions.
    //! @todo Support other tool chain

    uint32_t codeStart = 0;
    uint32_t codeEnd = 0;
    uint32_t ramStart = 0;
    uint32_t ramEnd = 0;

//#warning sbl-tip1: Need to aligned with linker file to enable reserved region
    /*
    codeStart = __ROM_START;
    codeEnd = __ROM_END;

    ramStart = __RAM_START;
    ramEnd = __RAM_END;
    */

    propertyStore->reservedRegions[kProperty_FlashReservedRegionIndex].startAddress = codeStart;
    propertyStore->reservedRegions[kProperty_FlashReservedRegionIndex].endAddress = codeEnd;
    propertyStore->reservedRegions[kProperty_RamReservedRegionIndex].startAddress = ramStart;
    propertyStore->reservedRegions[kProperty_RamReservedRegionIndex].endAddress = ramEnd;

    // Fill in available peripherals array.
    const peripheral_descriptor_t *peripherals = g_bootloaderContext.allPeripherals;
    propertyStore->availablePeripherals = 0;
    uint32_t enabledPeripherals = (propertyStore->configurationData.enabledPeripherals_extern
                                   << (sizeof(propertyStore->configurationData.enabledPeripherals) * 8)) |
                                  propertyStore->configurationData.enabledPeripherals;
    for (uint32_t i = 0; peripherals[i].typeMask != 0; ++i)
    {
        // Check that the peripheral is enabled in the user configuration data.
        if (enabledPeripherals & peripherals[i].typeMask)
        {
            propertyStore->availablePeripherals |= peripherals[i].typeMask;
        }
    }

// Fill in unique device id value.
#if defined(K32H844P_SERIES)
    propertyStore->UniqueDeviceId.uid[0] = OCOTP->CFG[0].CFG;
    propertyStore->UniqueDeviceId.uid[1] = OCOTP->CFG[1].CFG;
#else
//    propertyStore->UniqueDeviceId.uid[0] = OCOTP->CFG0;
//    propertyStore->UniqueDeviceId.uid[1] = OCOTP->CFG1;
#endif // K32H844P_SERIES

    // Set address range of RAM in property interface
    const memory_map_entry_t *map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexITCM];
    propertyStore->ramStartAddress[kIndexITCM] = map->startAddress;
    propertyStore->ramSizeInBytes[kIndexITCM] = map->endAddress - map->startAddress + 1;
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexDTCM];
    propertyStore->ramStartAddress[kIndexDTCM] = map->startAddress;
    propertyStore->ramSizeInBytes[kIndexDTCM] = map->endAddress - map->startAddress + 1;
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexOCRAM];
    propertyStore->ramStartAddress[kIndexOCRAM] = map->startAddress;
    propertyStore->ramSizeInBytes[kIndexOCRAM] = map->endAddress - map->startAddress + 1;

    update_specific_properties();

    return kStatus_Success;
}

// See property.h for documentation on this function.
status_t bootloader_property_get(uint8_t tag, uint32_t id, const void **value, uint32_t *valueSize)
{
    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;

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

        case kPropertyTag_RAMStartAddress:
#if (__CORTEX_M == 7)
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
#endif
            break;

        case kPropertyTag_RAMSizeInBytes:
#if (__CORTEX_M == 7)
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
#endif
            break;

        case kPropertyTag_AvailableCommands:
            returnValue = &propertyStore->availableCommands;
            break;

        case kPropertyTag_CheckStatus:
            switch (id)
            {
#if BL_FEATURE_CRC_CHECK
                case kCheckStatus_Crc:
                    returnValue = &propertyStore->crcCheckStatus;
                    break;
#endif // else falls through to unknown
#if BL_FETAURE_LOG_ENABLE
                case kCheckStatus_LastError:
                    bl_log_get_last_error(&returnValue, &returnSize);
                    if (returnSize == 0u)
                    {
                        static status_t statusNoError = kStatus_Success;
                        returnSize = 4;
                        returnValue = &statusNoError;
                    }
                    break;
#endif // BL_FETAURE_LOG_ENABLE
                default:
                    return kStatus_UnknownProperty;
                    break;
            }
            break;

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

        case kPropertyTag_SecurityState:
        {
            s_propertyReturnValue = (get_hab_status() == kHabStatus_Close) ? true : false;
            returnValue = &s_propertyReturnValue;
            break;
        }

        case kPropertyTag_UniqueDeviceId:
            returnSize = kUniqueId_SizeInBytes;
            returnValue = &propertyStore->UniqueDeviceId;
            break;

        case kPropertyTag_TargetVersion:
            returnValue = &propertyStore->targetVersion.version;
            break;
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
    switch (tag)
    {
        case kPropertyTag_BootloaderVersion:
        case kPropertyTag_AvailablePeripherals:
        case kPropertyTag_RAMStartAddress:
        case kPropertyTag_RAMSizeInBytes:
        case kPropertyTag_AvailableCommands:
        case kPropertyTag_ExternalMemoryAttributes:
        case kPropertyTag_MaxPacketSize:
        case kPropertyTag_ReservedRegions:
        case kPropertyTag_SystemDeviceId:
        case kPropertyTag_UniqueDeviceId:
        case kPropertyTag_TargetVersion:
            return kStatus_ReadOnlyProperty;
        default:
            return kStatus_UnknownProperty;
    }
}

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

    external_memory_property_store_t propertyStore = { 0 };
    uint32_t memoryInitStatus;
    status_t status = map->get(kExternalMemoryPropertyTag_InitStatus, &memoryInitStatus);
    if (status != kStatus_Success)
    {
        return status;
    }
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

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
