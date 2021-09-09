/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <sbl.h>
#include <stdint.h>
#include <string.h>
#include "bootloader/bootloader.h"
#include "bootloader_common.h"
#include "memory/memory.h"
#include "property/property.h"
#if !BL_FEATURE_MIN_PROFILE
#include "sbloader/sbloader.h"
#endif
#include "utilities/fsl_assert.h"
#include "utilities/fsl_rtos_abstraction.h"
#ifdef CONFIG_BOOT_ENCRYPTED_XIP
#include "update_key_context.h"
#endif

#if !defined(BOOTLOADER_HOST)
#if BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_FLASH_TYPE_KINETIS_C90TFS_FLASH || BL_FLASH_TYPE_LPC_C040HD_FLASH
#include "fsl_iap.h"
#elif BL_FLASH_TYPE_LPC_IP2113_FLASH
#include "flashiap_wrapper/fsl_flashiap_wrapper.h"
#endif
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#include "fsl_device_registers.h"
#if BL_FEATURE_QSPI_MODULE
#include "qspi/qspi.h"
#endif // #if BL_FEATURE_QSPI_MODULE
#if BL_FEATURE_OTFAD_MODULE
#include "otfad/fsl_otfad_driver.h"
#endif // #if BL_FEATURE_OTFAD_MODULE
#if BL_FEATURE_OCOTP_MODULE
#include "ocotp/fsl_ocotp.h"
#endif // #if BL_FEATURE_OCOTP_MODULE
#if BL_FEATURE_RELIABLE_UPDATE
#include "bootloader/bl_reliable_update.h"
#endif
#if BL_FEATURE_GEN_KEYBLOB
#include "keyblob.h"
#endif
#if BL_FEATURE_KEY_PROVISIONING
#include "keystore_puf.h"
#endif
#endif // #if !defined(BOOTLOADER_HOST)
#if BL_FEATURE_FLEXSPI_NOR_MODULE
#include "flexspi_flash.h"
#include "flexspi_flash_config.h"
#include "memory/src/flexspi_nor_memory.h"
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE

//! @addtogroup command
//! @{

//! @name State machine
//@{
static status_t handle_command(uint8_t *packet, uint32_t packetLength);
static status_t handle_data(bool *hasMoreData);
//@}

//! @name Command handlers
//@{
void handle_reset(uint8_t *packet, uint32_t packetLength);
void handle_flash_erase_all(uint8_t *packet, uint32_t packetLength);
void handle_flash_erase_all_unsecure(uint8_t *packet, uint32_t packetLength);
void handle_flash_erase_region(uint8_t *packet, uint32_t packetLength);
void handle_receive_sb_file(uint8_t *packet, uint32_t packetLength);
void handle_read_memory(uint8_t *packet, uint32_t packetLength);
void handle_fill_memory(uint8_t *packet, uint32_t packetLength);
void handle_set_property(uint8_t *packet, uint32_t packetLength);
void handle_get_property(uint8_t *packet, uint32_t packetLength);
void handle_write_memory(uint8_t *packet, uint32_t packetLength);
void handle_execute(uint8_t *packet, uint32_t packetLength);
void handle_call(uint8_t *packet, uint32_t packetLength);
void handle_flash_security_disable(uint8_t *packet, uint32_t packetLength);
void handle_flash_program_once(uint8_t *packet, uint32_t length);
void handle_flash_read_once(uint8_t *packet, uint32_t length);
void handle_flash_read_resource(uint8_t *packet, uint32_t length);
void handle_configure_memory(uint8_t *packet, uint32_t packetLength);
void handle_reliable_update(uint8_t *packet, uint32_t packetLength);
void handle_generate_key_blob(uint8_t *packet, uint32_t packetLength);
void handle_key_provisioning(uint8_t *packet, uint32_t packetLength);
//@}

//! @name Command responses
//@{
void send_read_memory_response(uint32_t commandStatus, uint32_t length);
void send_generate_key_blob_response(uint32_t commandStatus, uint32_t length);
void send_generic_response(uint32_t commandStatus, uint32_t commandTag);
void send_get_property_response(uint32_t commandStatus, uint32_t *value, uint32_t numValues);
void send_flash_read_once_response(uint32_t commandStatus, uint32_t *value, uint32_t byteCount);
void send_flash_read_resource_response(uint32_t commandStatus, uint32_t length);
void send_key_provisioning_response(uint32_t commandStatus, uint32_t length);
//@}

//! @name Data phase
//@{
static void reset_data_phase(void);
void finalize_data_phase(status_t status);
status_t handle_data_bidirection(bool *hasMoreData);
status_t handle_data_producer(bool *hasMoreData);
status_t handle_data_consumer(bool *hasMoreData);
//@}

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//#define TEST_SENDER_ABORT
//#define TEST_RECEIVER_ABORT

enum _secure_commands
{
    //! @brief Bitmask of commands allowed when flash security is enabled.
    //!
    //! This bitmask uses the same format as the AvailableCommands property. This is,
    //! the bit number for a given command is the command's tag value minus one.
    kCommandsAllowedWhenSecure = (HAS_CMD(kCommandTag_FlashSecurityDisable) | HAS_CMD(kCommandTag_GetProperty) |
                                  HAS_CMD(kCommandTag_Reset) | HAS_CMD(kCommandTag_SetProperty) |
                                  HAS_CMD(kCommandTag_FlashEraseAllUnsecure) | HAS_CMD(kCommandTag_ReceiveSbFile))
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#if defined(SOC_IMXRTYYYY_SERIES) && defined(CONFIG_BOOT_ENCRYPTED_XIP)
static uint32_t g_startAddress = 0;
#endif

//! @brief Command handler table.
const command_handler_entry_t g_commandHandlerTable[] = {
// cmd handler              // data handler or NULL
#if !BL_FEATURE_MIN_PROFILE
    { handle_flash_erase_all, NULL },              // kCommandTag_FlashEraseAll = 0x01
    { handle_flash_erase_region, NULL },           // kCommandTag_FlashEraseRegion = 0x02
    { handle_read_memory, handle_data_producer },  // kCommandTag_ReadMemory = 0x03
    { handle_write_memory, handle_data_consumer }, // kCommandTag_WriteMemory = 0x04
    { handle_fill_memory, NULL },                  // kCommandTag_FillMemory = 0x05
#if BL_FEATURE_FLASH_SECURITY
    { handle_flash_security_disable, NULL }, // kCommandTag_FlashSecurityDisable = 0x06
#else
    { 0 },
#endif                                                // BL_FEATURE_FLASH_SECURITY
    { handle_get_property, NULL },                    // kCommandTag_GetProperty = 0x07
    { handle_receive_sb_file, handle_data_consumer }, // kCommandTag_ReceiveSbFile = 0x08
    { handle_execute, NULL },                         // kCommandTag_Execute = 0x09
    { handle_call, NULL },                            // kCommandTag_Call = 0x0a
    { handle_reset, NULL },                           // kCommandTag_Reset = 0x0b
    { handle_set_property, NULL },                    // kCommandTag_SetProperty = 0x0c
#if BL_FEATURE_ERASEALL_UNSECURE
    { handle_flash_erase_all_unsecure, NULL }, // kCommandTag_FlashEraseAllUnsecure = 0x0d
#else                                          // BL_FEATURE_ERASEALL_UNSECURE
    { 0 }, // kCommandTag_FlashEraseAllUnsecure = 0x0d
#endif                                         // BL_FEATURE_ERASEALL_UNSECURE
#if BL_FEATURE_HAS_INTERNAL_FLASH || BL_FEATURE_OCOTP_MODULE || BL_FEATURE_OTP_MODULE
    { handle_flash_program_once, NULL }, // kCommandTag_ProgramOnce = 0x0e
    { handle_flash_read_once, NULL },    // kCommandTag_ReadOnce = 0x0f
#if !BL_FEATURE_HAS_NO_READ_SOURCE
    { handle_flash_read_resource, handle_data_producer }, // kCommandTag_ReadResource = 0x10
#else
    { 0 },
#endif // !BL_FEATURE_HAS_NO_READ_SOURCE
#else
    { 0 },
    { 0 },
    { 0 },
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH || BL_FEATURE_OCOTP_MODULE
#if BL_FEATURE_QSPI_MODULE || BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_EXPAND_MEMORY || \
    BL_FEATURE_SPI_NOR_EEPROM_MODULE || BL_FEATURE_HAS_INTERNAL_FLASH
    { handle_configure_memory, NULL }, // kCommandTag_ConfigureMemory = 0x11
#else
    { 0 }, // kCommandTag_ConfigureMemory = 0x11
#endif // BL_FEATURE_QSPI_MODULE || BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_EXPAND_MEMORY ||
       // BL_FEATURE_SPI_NOR_EEPROM_MODULE
#if BL_FEATURE_RELIABLE_UPDATE
    { handle_reliable_update, NULL }, // kCommandTag_ReliableUpdate = 0x12
#else
    { 0 }, // kCommandTag_ReliableUpdate = 0x12
#endif // BL_FEATURE_RELIABLE_UPDATE
#if BL_FEATURE_GEN_KEYBLOB
    { handle_generate_key_blob, handle_data_bidirection }, // kCommandTag_GenerateKeyBlob = 0x13
#else
    { 0 }, // kCommandTag_GenerateKeyBlob = 0x13
#endif     // BL_FEATURE_GEN_KEYBLOB
    { 0 }, // kCommandTag_Reserved0 = 0x14
#if BL_FEATURE_KEY_PROVISIONING
    { handle_key_provisioning, handle_data_bidirection }, // kCommandTag_KeyProvisioning = 0x15
#else
    { 0 }, // kCommandTag_KeyProvisioning = 0x15
#endif // BL_FEATURE_KEY_PROVISIONING
#else  // BL_FEATURE_MIN_PROFILE
    /////////////////////////////////////////////////////////////////////////////////////////////////
    { handle_flash_erase_all, NULL },              // kCommandTag_FlashEraseAll = 0x01
    { handle_flash_erase_region, NULL },           // kCommandTag_FlashEraseRegion = 0x02
#if BL_FEATURE_READ_MEMORY
    { handle_read_memory, handle_data_producer },  // kCommandTag_ReadMemory = 0x03
#else // BL_FEATURE_READ_MEMORY
    { 0 }, // kCommandTag_ReadMemory = 0x03
#endif
    { handle_write_memory, handle_data_consumer }, // kCommandTag_WriteMemory = 0x04
#if BL_FEATURE_FILL_MEMORY
    { handle_fill_memory, NULL },                  // kCommandTag_FillMemory = 0x05
#else
    { 0 },
#endif // BL_FEATURE_FILL_MEMORY
#if BL_FEATURE_FLASH_SECURITY
    { handle_flash_security_disable, NULL },       // kCommandTag_FlashSecurityDisable = 0x06
#else
    { 0 },
#endif // BL_FEATURE_FLASH_SECURITY
    { handle_get_property, NULL },                 // kCommandTag_GetProperty = 0x07
    { 0 },                                         // kCommandTag_ReceiveSbFile = 0x08
    { handle_execute, NULL },                      // kCommandTag_Execute = 0x09
    { 0 },                                         // kCommandTag_Call = 0x0a
    { handle_reset, NULL },                        // kCommandTag_Reset = 0x0b
    { handle_set_property, NULL },                 // kCommandTag_SetProperty = 0x0c
#if BL_FEATURE_ERASEALL_UNSECURE
    { handle_flash_erase_all_unsecure, NULL },     // kCommandTag_FlashEraseAllUnsecure = 0x0d
#else  // BL_FEATURE_ERASEALL_UNSECURE
    { 0 }, // kCommandTag_FlashEraseAllUnsecure = 0x0d
#endif // BL_FEATURE_ERASEALL_UNSECURE
#if BL_FEATURE_HAS_INTERNAL_FLASH || BL_FEATURE_OCOTP_MODULE
    { handle_flash_program_once, NULL }, // kCommandTag_ProgramOnce = 0x0e
    { handle_flash_read_once, NULL },    // kCommandTag_ReadOnce = 0x0f
#else
    { 0 },
    { 0 },
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH || BL_FEATURE_OCOTP_MODULE
    { 0 },
#if BL_FEATURE_QSPI_MODULE || BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_EXPAND_MEMORY || \
    BL_FEATURE_SPI_NOR_EEPROM_MODULE || BL_FEATURE_HAS_INTERNAL_FLASH
    { handle_configure_memory, NULL }, // kCommandTag_ConfigureMemory = 0x11
#else
    { 0 }, // kCommandTag_ConfigureMemory = 0x11
#endif // BL_FEATURE_QSPI_MODULE || BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_EXPAND_MEMORY ||
       // BL_FEATURE_SPI_NOR_EEPROM_MODULE
    { 0 },                                         // kCommandTag_ReliableUpdate = 0x12
    { 0 },                                         // kCommandTag_GenerateKeyBlob = 0x13
    { 0 },                                         // kCommandTag_Reserved0 = 0x14
    { 0 },                                         // kCommandTag_KeyProvisioning = 0x15
#endif // BL_FEATURE_MIN_PROFILE
};

//! @brief Command processor state data.
command_processor_data_t g_commandData;

// See bl_command.h for documentation on this interface.
command_interface_t g_commandInterface = { bootloader_command_init, bootloader_command_pump,
                                           (command_handler_entry_t *)&g_commandHandlerTable, &g_commandData };

#if BL_FEATURE_EXPAND_PACKET_SIZE
static uint8_t s_dataProducerPacket[kMaxBootloaderPacketSize];
#endif // BL_FEATURE_EXPAND_PACKET_SIZE

#if BL_FEATURE_KEY_PROVISIONING
BL_ALIGN(4) uint8_t s_userKeyBuffer[BL_FEATURE_KEY_STORE_MAX_KEY_SIZE] = { 0 };
BL_ALIGN(4) uint8_t s_userKeyStoreBuffer[sizeof(key_store_t)] = { 0 };
#endif

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See bl_command.h for documentation on this function.
status_t bootloader_command_init()
{
    command_processor_data_t *data = g_bootloaderContext.commandInterface->stateData;

    data->state = kCommandState_CommandPhase;
    return kStatus_Success;
}

// See bl_command.h for documentation on this function.
status_t bootloader_command_pump()
{
    status_t status = kStatus_Success;
    bool hasMoreData = false;

    if (g_bootloaderContext.activePeripheral->packetInterface)
    {
        switch (g_bootloaderContext.commandInterface->stateData->state)
        {
            default:
            case kCommandState_CommandPhase:
                status = g_bootloaderContext.activePeripheral->packetInterface->readPacket(
                    g_bootloaderContext.activePeripheral, &g_bootloaderContext.commandInterface->stateData->packet,
                    &g_bootloaderContext.commandInterface->stateData->packetLength, kPacketType_Command);
                if ((status != kStatus_Success) && (status != kStatus_AbortDataPhase) && (status != kStatus_Ping))
                {
                    debug_printf("Error: readPacket returned status 0x%x\r\n", status);
                    break;
                }
                if (g_bootloaderContext.commandInterface->stateData->packetLength == 0)
                {
                    // No command packet is available. Return success.
                    break;
                }
                status = handle_command(g_bootloaderContext.commandInterface->stateData->packet,
                                        g_bootloaderContext.commandInterface->stateData->packetLength);
                if (status != kStatus_Success)
                {
                    debug_printf("Error: handle_command returned status 0x%x\r\n", status);
                    break;
                }
                g_bootloaderContext.commandInterface->stateData->state = kCommandState_DataPhase;
                break;

            case kCommandState_DataPhase:
                status = handle_data(&hasMoreData);
                if (status != kStatus_Success)
                {
                    debug_printf("Error: handle_data returned status 0x%x\r\n", status);
                    g_bootloaderContext.commandInterface->stateData->state = kCommandState_CommandPhase;
                    break;
                }
                g_bootloaderContext.commandInterface->stateData->state =
                    hasMoreData ? kCommandState_DataPhase : kCommandState_CommandPhase;
                break;
        }
    }

    return status;
}

//! @brief Find command handler entry.
//!
//! @retval NULL if no entry found.
static const command_handler_entry_t *find_entry(uint8_t tag)
{
    if (tag < kFirstCommandTag || tag > kLastCommandTag)
    {
        return 0; // invalid command
    }
    const command_handler_entry_t *entry =
        &g_bootloaderContext.commandInterface->handlerTable[(tag - kFirstCommandTag)];

    return entry;
}

//! @brief Handle a command transaction.
static status_t handle_command(uint8_t *packet, uint32_t packetLength)
{
    command_packet_t *commandPacket = (command_packet_t *)packet;
    uint8_t commandTag = commandPacket->commandTag;
    status_t status = kStatus_Success;

    // Look up the handler entry and save it for the data phaase.
    g_bootloaderContext.commandInterface->stateData->handlerEntry = find_entry(commandTag);

    if (g_bootloaderContext.commandInterface->stateData->handlerEntry &&
        g_bootloaderContext.commandInterface->stateData->handlerEntry->handleCommand)
    {
#if !BOOTLOADER_HOST && BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_DEVICE_IS_LPC_SERIES
#elif BL_DEVICE_IS_KINETIS_SERIES
        // Get flash security state.
        // Note: Both Main and Secondary flash share the same security state
        //  So it doesn't matter what index of allFlashState[] we use for this FLASH API.
        flash_security_state_t securityState;
        status = g_bootloaderContext.flashDriverInterface->flash_get_security_state(
            &g_bootloaderContext.allFlashState[kFlashIndex_Main], &securityState);
        if (status == kStatus_Success)
        {
            // If flash security is enabled, make sure the command is one that is allowed. If
            // it's not, then we return an error response.
            if ((securityState != kFLASH_SecurityStateNotSecure) &&
                !IS_CMD_AVAILABLE(kCommandsAllowedWhenSecure, commandTag))
            {
                // Security is enabled and the command is not one of the few that can be
                // run, so return a security violation error.
                debug_printf("Error: command 0x%x not available due to flash security\r\n", commandPacket->commandTag);
                status = kStatus_SecurityViolation;
            }
            else
            {
#elif BL_DEVICE_IS_IMX_SERIES
// Get SoC security state.
// Note: The SoC State is blown in the Fuse
#else
#error Unsupport family!
#endif
#endif // #if !BOOTLOADER_HOST && BL_FEATURE_HAS_INTERNAL_FLASH
       // Process the command normally.
                g_bootloaderContext.commandInterface->stateData->handlerEntry->handleCommand(packet, packetLength);
                return kStatus_Success;
#if !BOOTLOADER_HOST && BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_DEVICE_IS_LPC_SERIES
#elif BL_DEVICE_IS_KINETIS_SERIES
            }
        }
#elif BL_DEVICE_IS_IMX_SERIES
#else
#error Unsupport family!
#endif
#endif // #if !BOOTLOADER_HOST && BL_FEATURE_HAS_INTERNAL_FLASH
    }
    else
    {
        // We don't recognize this command, so return an error response.
        debug_printf("unknown command 0x%x\r\n", commandPacket->commandTag);
        status = kStatus_UnknownCommand;
    }

    // Should only get to this point if an error occurred before running the command handler.
    send_generic_response(status, commandTag);
    return status;
}

//! @brief Handle a data transaction.
static status_t handle_data(bool *hasMoreData)
{
    if (g_bootloaderContext.commandInterface->stateData->handlerEntry)
    {
        // Run data phase if present, otherwise just return success.
        *hasMoreData = 0;
        return g_bootloaderContext.commandInterface->stateData->handlerEntry->handleData ?
                   g_bootloaderContext.commandInterface->stateData->handlerEntry->handleData(hasMoreData) :
                   kStatus_Success;
    }

    debug_printf("Error: no handler entry for data phase\r\n");
    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// Command Handlers
////////////////////////////////////////////////////////////////////////////////

//! @brief Reset command handler.
void handle_reset(uint8_t *packet, uint32_t packetLength)
{
    command_packet_t *commandPacket = (command_packet_t *)packet;
    send_generic_response(kStatus_Success, commandPacket->commandTag);

#if !defined(BOOTLOADER_HOST)
    // Wait for the ack from the host to the generic response
    g_bootloaderContext.activePeripheral->packetInterface->finalize(g_bootloaderContext.activePeripheral);

    // Prepare for shutdown.
    shutdown_cleanup(kShutdownType_Reset);
#if defined(BL_FEATURE_6PINS_PERIPHERAL) && BL_FEATURE_6PINS_PERIPHERAL
    shutdown_cleanup(kShutdownType_Cleanup);
#endif // BL_FEATURE_6PINS_PERIPHERAL

    NVIC_SystemReset();
    // Does not get here.
    assert(0);
#endif // BOOTLOADER_HOST
}

#if BL_FEATURE_RELIABLE_UPDATE
//! @brief Reliable Update command handler.
void handle_reliable_update(uint8_t *packet, uint32_t packetLength)
{
    command_packet_t *commandPacket = (command_packet_t *)packet;
    status_t status = kStatus_Success;

#if !defined(BOOTLOADER_HOST)
    reliable_update_packet_t *command = (reliable_update_packet_t *)packet;
    uint32_t address = command->address;

    // Call reliable update implementation..
    bootloader_reliable_update_as_requested(kReliableUpdateOption_Swap, address);

    status = g_bootloaderContext.propertyInterface->store->reliableUpdateStatus;
    send_generic_response(status, commandPacket->commandTag);

#if BL_IS_HARDWARE_SWAP_ENABLED
    // A system reset is needed, As swap command will only take effect after reset
    if (status == kStatus_ReliableUpdateSuccess)
    {
        // Wait for the ack from the host to the generic response
        g_bootloaderContext.activePeripheral->packetInterface->finalize(g_bootloaderContext.activePeripheral);

        // Prepare for shutdown.
        shutdown_cleanup(kShutdownType_Reset);

        NVIC_SystemReset();
        // Does not get here.
        assert(0);
    }
#endif // BL_IS_HARDWARE_SWAP_ENABLED
#else
    send_generic_response(status, commandPacket->commandTag);
#endif // BOOTLOADER_HOST
}
#endif // BL_FEATURE_RELIABLE_UPDATE

//! @brief Reset data phase variables.
static void reset_data_phase()
{
    memset(&g_bootloaderContext.commandInterface->stateData->dataPhase, 0,
           sizeof(g_bootloaderContext.commandInterface->stateData->dataPhase));
}

//! @brief Flash Erase All command handler.
void handle_flash_erase_all(uint8_t *packet, uint32_t packetLength)
{
    flash_erase_all_packet_t *commandPacket = (flash_erase_all_packet_t *)packet;
    status_t status = kStatus_Success;

// Call flash erase all implementation.
#ifdef BOOTLOADER_HOST
    host_flash_erase_all();
#else
                                                   // For target without QSPI module, ignore the memory identifier
#if ((!BL_FEATURE_QSPI_MODULE) && (!BL_FEATURE_FAC_ERASE) && (!BL_FEATURE_EXPAND_MEMORY) && \
     (BL_FEATURE_HAS_INTERNAL_FLASH))
    status = flash_mem_erase_all();
#else
    switch (commandPacket->memoryId)
    {
#if BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_FEATURE_FAC_ERASE
        case kMemoryInternal:
            status = flash_mem_erase_all(kFlashEraseAllOption_Blocks);
            break;
        case kMemoryFlashExecuteOnly:
            status = flash_mem_erase_all(kFlashEraseAllOption_ExecuteOnlySegments);
            break;
#else
        case kMemoryInternal:
            status = flash_mem_erase_all();
            break;
#endif // BL_FEATURE_FAC_ERASE
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_FEATURE_FLEXSPI_NOR_MODULE
        case kMemoryFlexSpiNor:
            status = flexspi_nor_mem_erase_all();
            break;
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE
#if BL_FEATURE_QSPI_MODULE
        case kMemoryQuadSpi0:
            status = qspi_mem_erase_all();
            break;
#endif
#if BL_FEATURE_SPINAND_MODULE
        case kMemorySpiNand:
            status = spinand_mem_erase_all();
            break;
#endif
#if BL_FEATURE_SEMC_NOR_MODULE
        case kMemorySemcNor:
            status = semc_nor_mem_erase_all();
            break;
#endif
#if BL_FEATURE_SEMC_NAND_MODULE
        case kMemorySemcNand:
            status = semc_nand_mem_erase_all();
            break;
#endif
#if BL_FEATURE_SPI_NOR_EEPROM_MODULE
        case kMemorySpiNorEeprom:
            status = spi_nor_eeprom_mem_erase_all();
            break;
#endif
#if BL_FEATURE_MMC_MODULE
        case kMemoryMMCCard:
            status = mmc_mem_erase_all();
            break;
#endif
#if BL_FEATURE_SD_MODULE
        case kMemorySDCard:
            status = sd_mem_erase_all();
            break;
#endif
        default:
            status = kStatus_InvalidArgument;
            break;
    }
#endif // #if ((!BL_FEATURE_QSPI_MODULE) && (!BL_FEATURE_FAC_ERASE) && (!BL_FEATURE_EXPAND_MEMORY) &&
// (BL_FEATURE_HAS_INTERNAL_FLASH))
#endif // #ifdef BOOTLOADER_HOST

    send_generic_response(status, commandPacket->commandPacket.commandTag);
}

//! @brief Flash Erase All Unsecure command handler.
void handle_flash_erase_all_unsecure(uint8_t *packet, uint32_t packetLength)
{
    command_packet_t *commandPacket = (command_packet_t *)packet;
    status_t status = kStatus_Success;

// Call flash erase all unsecure implementation.
#ifdef BOOTLOADER_HOST
    host_flash_erase_all_unsecure();
#elif BL_FEATURE_ERASEALL_UNSECURE
    status = flash_mem_erase_all_unsecure();
#endif

    send_generic_response(status, commandPacket->commandTag);
}

//! @brief Flash Erase Region command handler.
void handle_flash_erase_region(uint8_t *packet, uint32_t packetLength)
{
    flash_erase_region_packet_t *command = (flash_erase_region_packet_t *)packet;
    status_t status = kStatus_Success;

// Call flash erase region implementation.
#ifdef BOOTLOADER_HOST
    host_flash_erase_region(command->startAddress, command->byteCount);
#else
    status = g_bootloaderContext.memoryInterface->erase(command->startAddress, command->byteCount, command->memoryId);
#endif

    send_generic_response(status, command->commandPacket.commandTag);
}

#if !BL_FEATURE_MIN_PROFILE
//! @brief Receive SB File command handler.
void handle_receive_sb_file(uint8_t *packet, uint32_t packetLength)
{
    receive_sb_file_packet_t *command = (receive_sb_file_packet_t *)packet;

    // Start the data phase.
    reset_data_phase();
    g_bootloaderContext.commandInterface->stateData->dataPhase.count = command->byteCount;
    g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag = kCommandTag_ReceiveSbFile;
    send_generic_response(kStatus_Success, command->commandPacket.commandTag);

    // Initialize the SB file loader state machine
    sbloader_init();
}
#endif

//! @brief Get Property command handler.
void handle_get_property(uint8_t *packet, uint32_t packetLength)
{
    get_property_packet_t *command = (get_property_packet_t *)packet;

    uint32_t *value = NULL;
    uint32_t valueSize = 0;
    status_t status = g_bootloaderContext.propertyInterface->get(command->propertyTag, command->memoryId,
                                                                 (const void **)&value, &valueSize);

    // Make sure the property's size is no more than the size of the max number of return parameters.
    assert(valueSize <= (kMaxPropertyReturnValues * sizeof(uint32_t)));

    // Currently there are no property responses that contain a data phase.
    g_bootloaderContext.commandInterface->stateData->dataPhase.count = 0;
    send_get_property_response(status, value, (valueSize / sizeof(uint32_t)));

#if BL_FEATURE_FLEXSPI_NOR_MODULE
    // Init flash after first time get-property 
    static bool isFirstTimeGetProperty = true;
    if (isFirstTimeGetProperty)
    {
        sbl_flash_init();
        isFirstTimeGetProperty = false;
        flexspi_nor_mem_enable();
    }
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE
}

//! @brief Set Property command handler.
void handle_set_property(uint8_t *packet, uint32_t packetLength)
{
    set_property_packet_t *command = (set_property_packet_t *)packet;

    status_t status = g_bootloaderContext.propertyInterface->set_uint32(command->propertyTag, command->propertyValue);

    send_generic_response(status, command->commandPacket.commandTag);
}

#if BL_FEATURE_QSPI_MODULE || BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SEMC_NOR_MODULE || \
    BL_FEATURE_EXPAND_MEMORY || BL_FEATURE_SPI_NOR_EEPROM_MODULE  || BL_FEATURE_HAS_INTERNAL_FLASH
//! @brief Configure memory command handler.
void handle_configure_memory(uint8_t *packet, uint32_t packetLength)
{
    configure_memory_packet_t *command = (configure_memory_packet_t *)packet;
    status_t status = kStatus_InvalidArgument;

#if BL_FEATURE_QSPI_MODULE
    if (command->flashMemId == kMemoryQuadSpi0)
    {
        status = configure_qspi(command->configBlockAddress);
    }
    else
#endif // BL_FEATURE_QSPI_MODULE
#if BL_FEATURE_FLEXSPI_NOR_MODULE
        if (command->flashMemId == kMemoryFlexSpiNor)
    {
        //status = flexspi_nor_mem_config((uint32_t *)command->configBlockAddress);
        status = kStatus_Success;
    }
    else
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE
#if BL_FEATURE_SEMC_NOR_MODULE
        if (command->flashMemId == kMemorySemcNor)
    {
        status = semc_nor_mem_config((uint32_t *)command->configBlockAddress);
    }
    else
#endif // #if BL_FEATURE_SEMC_NOR_MODULE
#if BL_FEATURE_EXPAND_MEMORY
        if (GROUPID(command->flashMemId) == kGroup_External)
    {
        uint32_t index;
        status = find_external_map_index(command->flashMemId, &index);
        if (status == kStatus_Success)
        {
            external_memory_map_entry_t *map =
                (external_memory_map_entry_t *)&g_bootloaderContext.externalMemoryMap[index];
            status = map->memoryInterface->config((uint32_t *)command->configBlockAddress);
        }
    }
#endif // #if BL_FEATURE_EXPAND_MEMORY
#if BL_FEATURE_HAS_INTERNAL_FLASH
    if (GROUPID(command->flashMemId) == kGroup_Internal)
    {
        const memory_map_entry_t *mapEntry;
        status = find_map_entry(*(uint32_t *)(command->configBlockAddress + 4), 4, &mapEntry);
        if (status == kStatus_Success)
        {
            status = mapEntry->memoryInterface->config((uint32_t *)command->configBlockAddress);
        }
    }
#endif

    send_generic_response(status, command->commandPacket.commandTag);
}
#endif // BL_FEATURE_QSPI_MODULE || BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_EXPAND_MEMORY ||
       // BL_FEATURE_SPI_NOR_EEPROM_MODULE

#if BL_FEATURE_QSPI_MODULE
status_t configure_qspi(const uint32_t address)
{
    status_t status = kStatus_Success;

    uint32_t startAddr = address;
    uint32_t endAddr = startAddr + sizeof(qspi_config_t) - 1;

    // Validate parameters.
    if ((!is_valid_application_location(startAddr)) || (!is_valid_application_location(endAddr)))
    {
        status = kStatus_InvalidArgument;
    }

    // Call configure quadspi implementation.
    if (status == kStatus_Success)
    {
#ifdef BOOTLOADER_HOST
#else
        status = quadspi_init((void *)startAddr);
        if (status == kStatus_Success)
        {
            // Re-init memory interface to intialize qspi memory interface
            g_bootloaderContext.memoryInterface->init();
        }
        g_bootloaderContext.propertyInterface->store->qspiInitStatus = status;
#endif
    }

    return status;
}
#endif // BL_FEATURE_QSPI_MODULE

//! @brief Write Memory command handler.
void handle_write_memory(uint8_t *packet, uint32_t packetLength)
{
    write_memory_packet_t *command = (write_memory_packet_t *)packet;

    // Start the data phase.
    reset_data_phase();
    g_bootloaderContext.commandInterface->stateData->dataPhase.memoryId = command->memoryId;
    g_bootloaderContext.commandInterface->stateData->dataPhase.count = command->byteCount;
    g_bootloaderContext.commandInterface->stateData->dataPhase.address = command->startAddress;
#if defined(SOC_IMXRTYYYY_SERIES) && defined(CONFIG_BOOT_ENCRYPTED_XIP)
	g_startAddress = command->startAddress;
#endif
    g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag = kCommandTag_WriteMemory;
    send_generic_response(kStatus_Success, command->commandPacket.commandTag);
}

#if BL_FEATURE_GEN_KEYBLOB
#if defined(BL_FEATURE_KEYBLOB_DATA_KEY_MAX_SIZE)
static uint8_t s_keyData[BL_FEATURE_KEYBLOB_DATA_KEY_MAX_SIZE];
#else
// AES-128bit.
static uint8_t s_keyData[16];
#endif
#if defined(BL_FEATURE_KEYBLOB_BLOB_MAX_SIZE)
static uint8_t s_keyBlob[BL_FEATURE_KEYBLOB_BLOB_MAX_SIZE];
#else
// Key Blob Header: 8 bytes;
// Encrypted Blob Key: 32 bytes;
// Encrypted Data Key: equals to the size of s_keyData;
// Message Authentication Code(MAC): 16 bytes.
static uint8_t s_keyBlob[56 + sizeof(s_keyData)];
#endif
static uint32_t s_keySize;
static uint32_t s_keySel;

//! @brief Generate Key Blob command handler.
void handle_generate_key_blob(uint8_t *packet, uint32_t packetLength)
{
    status_t status = kStatus_Fail;

    generate_key_blob_packet_t *command = (generate_key_blob_packet_t *)packet;

    // Start the data phase.
    reset_data_phase();
    g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag = kCommandTag_GenerateKeyBlob;
    g_bootloaderContext.commandInterface->stateData->dataPhase.option = kDataPhase_Option_Skip;

    if (command->operationPhase == kGenKeyBlob_Phase_SendKey)
    {
        if (command->keyLength <= sizeof(s_keyData))
        {
            // receiving data key.
            s_keySize = command->keyLength;
            s_keySel = command->keySel;
            g_bootloaderContext.commandInterface->stateData->dataPhase.count = command->keyLength;
            g_bootloaderContext.commandInterface->stateData->dataPhase.address = (uint32_t)&s_keyData[0];
            g_bootloaderContext.commandInterface->stateData->dataPhase.option = kDataPhase_Option_Consumer;

            status = kStatus_Success;
        }
        else
        {
            status = kStatus_InvalidArgument;
        }
        send_generic_response(status, command->commandPacket.commandTag);
    }
    else if (command->operationPhase == kGenKeyBlob_Phase_ReceiveKeyBlob)
    {
        // generate key blob
        uint32_t keyBlobSize = 0;
        status = generate_key_blob(s_keyData, s_keySize, s_keySel, s_keyBlob, &keyBlobSize);

        assert(keyBlobSize <= sizeof(s_keyBlob));

        if (status == kStatus_Success)
        {
            g_bootloaderContext.commandInterface->stateData->dataPhase.count = keyBlobSize;
            g_bootloaderContext.commandInterface->stateData->dataPhase.address = (uint32_t)&s_keyBlob[0];
            g_bootloaderContext.commandInterface->stateData->dataPhase.option = kDataPhase_Option_Producer;
        }
        send_generate_key_blob_response(status, keyBlobSize);
    }
    else
    {
        send_generate_key_blob_response(kStatus_InvalidArgument, 0);
    }
}
#endif // #if BL_FEATURE_GEN_KEYBLOB

//! @brief Read Memory command handler.
void handle_read_memory(uint8_t *packet, uint32_t packetLength)
{
    read_memory_packet_t *command = (read_memory_packet_t *)packet;

    // Start the data phase.
    reset_data_phase();
    g_bootloaderContext.commandInterface->stateData->dataPhase.memoryId = command->memoryId;
    g_bootloaderContext.commandInterface->stateData->dataPhase.count = command->byteCount;
    g_bootloaderContext.commandInterface->stateData->dataPhase.address = command->startAddress;
    g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag = kCommandTag_ReadMemory;
    send_read_memory_response(kStatus_Success, command->byteCount);
}

//! @brief Complete the data phase, optionally send a response.
void finalize_data_phase(status_t status)
{
    g_bootloaderContext.commandInterface->stateData->dataPhase.address = 0;
    g_bootloaderContext.commandInterface->stateData->dataPhase.count = 0;

    // Force to write cached data to target memory
    if (g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag == kCommandTag_WriteMemory)
    {
        assert(g_bootloaderContext.memoryInterface->flush);
        status_t flushStatus = g_bootloaderContext.memoryInterface->flush();

        // Update status only if the last operation result is successfull in order to reflect
        // real result of the write operation.
        if (status == kStatus_Success)
        {
            status = flushStatus;

#if defined(SOC_IMXRTYYYY_SERIES) && defined(CONFIG_BOOT_ENCRYPTED_XIP)
            if ((g_startAddress == BOOT_FLASH_ACT_APP) ||
                (g_startAddress == BOOT_FLASH_CAND_APP))
            {
                update_key_context(g_startAddress + KEY_CONTEXT_OFFSET_IN_APP);
                g_startAddress = 0;
            }
#endif
        }
    }
#if BL_FEATURE_KEY_PROVISIONING
    if ((status == kStatus_Success) &&
        (g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag == kCommandTag_KeyProvisioning))
    {
        if (g_bootloaderContext.commandInterface->stateData->dataPhase.argument0 ==
            kKeyProvisioning_Operation_SetUserKey)
        {
            status =
                key_store_set_key(s_userKeyBuffer, g_bootloaderContext.commandInterface->stateData->dataPhase.argument2,
                                  g_bootloaderContext.commandInterface->stateData->dataPhase.argument1);
        }
        else if (g_bootloaderContext.commandInterface->stateData->dataPhase.argument0 ==
                 kKeyProvisioning_Operation_WriteKeyStore)
        {
            status = key_store_set(s_userKeyStoreBuffer, sizeof(s_userKeyStoreBuffer));
        }
    }
#endif // #if BL_FEATURE_KEY_PROVISIONING
#if BL_FEATURE_EXPAND_MEMORY
    // Reset the state machine of memory interface.
    assert(g_bootloaderContext.memoryInterface->finalize);
    status_t finalizeStatus = g_bootloaderContext.memoryInterface->finalize();
    if (status == kStatus_Success)
    {
        status = finalizeStatus;
    }
#endif // BL_FEATURE_EXPAND_MEMORY

    // Send final response packet.
    send_generic_response(status, g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag);

#if !BL_FEATURE_MIN_PROFILE
    if ((status == kStatus_AbortDataPhase) &&
        g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag == kCommandTag_ReceiveSbFile)
    {
        // Aborting due to sb loader jump or reset command.
        // If jump or reset successful, this will not return.
        // In the current architecture there is no way to handle an error return from sbloader_finalize()
        // because we already sent the "abort" status above to indicate that a jump command was encountered.
        sbloader_finalize();
    }
#endif // !BL_FEATURE_MIN_PROFILE
}

//! @brief Handle data phase, direction is defined in dataPhase.option.
status_t handle_data_bidirection(bool *hasMoreData)
{
    if (g_bootloaderContext.commandInterface->stateData->dataPhase.option == kDataPhase_Option_Consumer)
    {
        return handle_data_consumer(hasMoreData);
    }
    else if (g_bootloaderContext.commandInterface->stateData->dataPhase.option == kDataPhase_Option_Producer)
    {
        return handle_data_producer(hasMoreData);
    }
    // Skip the data phase for other options, including invalid options.
    else // if (g_bootloaderContext.commandInterface->stateData->dataPhase.option == kDataPhase_Option_Skip)
    {
        *hasMoreData = false;
        return kStatus_Success;
    }
}

//! @brief Handle data phase with data consumer (read from host).
status_t handle_data_consumer(bool *hasMoreData)
{
    if (g_bootloaderContext.commandInterface->stateData->dataPhase.count == 0)
    {
        // No data phase.
        *hasMoreData = false;
        finalize_data_phase(kStatus_Success);
        return kStatus_Success;
    }

    *hasMoreData = true;
    uint32_t memoryId = g_bootloaderContext.commandInterface->stateData->dataPhase.memoryId;
    uint32_t remaining = g_bootloaderContext.commandInterface->stateData->dataPhase.count;
    uint32_t dataAddress = g_bootloaderContext.commandInterface->stateData->dataPhase.address;
    uint8_t *packet;
    uint32_t packetLength = 0;
    status_t status;

    // Read the data packet.
    status = g_bootloaderContext.activePeripheral->packetInterface->readPacket(
        g_bootloaderContext.activePeripheral, &packet, &packetLength, kPacketType_Data);
    if (status != kStatus_Success)
    {
        // Abort data phase due to error.
        debug_printf("consumer abort data phase due to status 0x%x\r\n", status);
        g_bootloaderContext.activePeripheral->packetInterface->abortDataPhase(g_bootloaderContext.activePeripheral);
        finalize_data_phase(status);
        *hasMoreData = false;
        return kStatus_Success;
    }
    if (packetLength == 0)
    {
        // Sender requested data phase abort.
        debug_printf("Data phase aborted by sender\r\n");
        finalize_data_phase(kStatus_AbortDataPhase);
        *hasMoreData = false;
        return kStatus_Success;
    }

    //
    // Write the data to the destination address.
    //

    packetLength = MIN(packetLength, remaining);

#if !BL_FEATURE_MIN_PROFILE
    if (g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag == kCommandTag_ReceiveSbFile)
    {
        // Consumer is sb loader state machine
        g_bootloaderContext.commandInterface->stateData->dataPhase.data = packet;
        g_bootloaderContext.commandInterface->stateData->dataPhase.dataBytesAvailable = packetLength;

        status = sbloader_pump(packet, packetLength);

        // kStatusRomLdrDataUnderrun means need more data
        // kStatusRomLdrSectionOverrun means we reached the end of the sb file processing
        // either of these are OK
        if ((status == kStatusRomLdrDataUnderrun) || (status == kStatusRomLdrSectionOverrun))
        {
            status = kStatus_Success;
        }
    }
#if BL_FEATURE_GEN_KEYBLOB
    else if (g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag == kCommandTag_GenerateKeyBlob)
    {
        memcpy((void *)dataAddress, packet, packetLength);
        dataAddress += packetLength;
    }
    else
#endif
#if BL_FEATURE_KEY_PROVISIONING
        if (g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag == kCommandTag_KeyProvisioning)
    {
        memcpy((uint8_t *)dataAddress, packet, packetLength);
        dataAddress += packetLength;
    }
    else
#endif
#endif // !BL_FEATURE_MIN_PROFILE
    {
        // Consumer is memory interface.
        status = g_bootloaderContext.memoryInterface->write(dataAddress, packetLength, packet, memoryId);
        dataAddress += packetLength;
    }

    remaining -= packetLength;

#ifdef TEST_RECEIVER_ABORT
    status = kStatus_Fail;
#endif

    if (remaining == 0)
    {
        finalize_data_phase(status);
        *hasMoreData = false;
    }
    else if (status != kStatus_Success)
    {
        // Abort data phase due to error.
        debug_printf("Data phase error 0x%x, aborting\r\n", status);
        g_bootloaderContext.activePeripheral->packetInterface->abortDataPhase(g_bootloaderContext.activePeripheral);
        finalize_data_phase(status);
        *hasMoreData = false;
    }
    else
    {
        g_bootloaderContext.commandInterface->stateData->dataPhase.count = remaining;
        g_bootloaderContext.commandInterface->stateData->dataPhase.address = dataAddress;
    }

    return kStatus_Success;
}

//! @brief Handle data phase with data producer (send to host).
status_t handle_data_producer(bool *hasMoreData)
{
    if (g_bootloaderContext.commandInterface->stateData->dataPhase.count == 0)
    {
        // No data phase.
        *hasMoreData = false;
        finalize_data_phase(kStatus_Success);
        return kStatus_Success;
    }

    *hasMoreData = true;
    uint32_t memoryId = g_bootloaderContext.commandInterface->stateData->dataPhase.memoryId;
    uint32_t remaining = g_bootloaderContext.commandInterface->stateData->dataPhase.count;
    uint32_t dataAddress = g_bootloaderContext.commandInterface->stateData->dataPhase.address;
    uint8_t *data = g_bootloaderContext.commandInterface->stateData->dataPhase.data;
    uint8_t commandTag = g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag;
    status_t status = kStatus_Success;

    // Initialize the data packet to send.
    uint32_t packetSize;
#if BL_FEATURE_EXPAND_PACKET_SIZE
    uint8_t *packet = s_dataProducerPacket;
    uint32_t packetBufferSize =
        g_bootloaderContext.activePeripheral->packetInterface->getMaxPacketSize(g_bootloaderContext.activePeripheral);
    packetSize = MIN(packetBufferSize, remaining);
#else
    uint8_t packet[kMinPacketBufferSize];
    packetSize = MIN(kMinPacketBufferSize, remaining);
#endif // BL_FEATURE_EXPAND_PACKET_SIZE

    // Copy the data into the data packet.
    if (data)
    {
        // Copy data using compiler-generated memcpy.
        memcpy(packet, data, packetSize);
        data += packetSize;
        status = kStatus_Success;
    }
    else
    {
        if (commandTag == kCommandTag_ReadMemory)
        {
            // Copy data using memory interface.
            status = g_bootloaderContext.memoryInterface->read(dataAddress, packetSize, packet, memoryId);
        }
#if !BL_FEATURE_MIN_PROFILE
#if BL_FEATURE_HAS_INTERNAL_FLASH
        else if (commandTag == kCommandTag_FlashReadResource)
        {
// Read data from special-purpose flash memory
#if !defined(BOOTLOADER_HOST)
            flash_read_resource_option_t option =
                (flash_read_resource_option_t)g_bootloaderContext.commandInterface->stateData->dataPhase.option;
            lock_acquire();
            // Note: Both Main and Secondary flash share the same IFR Memory
            //  So it doesn't matter what index of allFlashState[] we use for this FLASH API.
            status = g_bootloaderContext.flashDriverInterface->flash_read_resource(
                &g_bootloaderContext.allFlashState[kFlashIndex_Main], dataAddress, (uint32_t *)packet, packetSize,
                option);
            lock_release();
#endif // BOOTLOADER_HOST
        }
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
#if BL_FEATURE_GEN_KEYBLOB
        else if (commandTag == kCommandTag_GenerateKeyBlob)
        {
            memcpy(packet, (void *)dataAddress, packetSize);
        }
#endif
#if BL_FEATURE_KEY_PROVISIONING
        else if (g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag == kCommandTag_KeyProvisioning)
        {
            memcpy(packet, (uint8_t *)dataAddress, packetSize);
        }
#endif
#endif // #if !BL_FEATURE_MIN_PROFILE
        dataAddress += packetSize;
    }

    if (status != kStatus_Success)
    {
        debug_printf("Error: %s returned status 0x%x, abort data phase\r\n",
                     (commandTag == kCommandTag_ReadMemory) ? "read memory" : "flash read resource", status);
        // Send zero length packet to tell host we are aborting data phase
        g_bootloaderContext.activePeripheral->packetInterface->writePacket(
            g_bootloaderContext.activePeripheral, (const uint8_t *)packet, 0, kPacketType_Data);
        finalize_data_phase(status);
        *hasMoreData = false;
        return kStatus_Success;
    }
    remaining -= packetSize;

#ifdef TEST_SENDER_ABORT
#ifndef WIN32
// Disble IAR "statement is unreachable" error
#pragma diag_suppress = Pe111
#endif // WIN32
    // Send zero length packet to abort data phase.
    g_bootloaderContext.activePeripheral->packetInterface->writePacket(g_bootloaderContext.activePeripheral,
                                                                       (const uint8_t *)packet, 0, kPacketType_Data);
    finalize_data_phase(kStatus_AbortDataPhase);
    *hasMoreData = false;
    return kStatus_Success;
#endif // TEST_SENDER_ABORT;

    status = g_bootloaderContext.activePeripheral->packetInterface->writePacket(
        g_bootloaderContext.activePeripheral, (const uint8_t *)packet, packetSize, kPacketType_Data);

    if (remaining == 0)
    {
        finalize_data_phase(status);
        *hasMoreData = false;
    }
    else if (status != kStatus_Success)
    {
        debug_printf("writePacket aborted due to status 0x%x\r\n", status);
        finalize_data_phase(status);
        *hasMoreData = false;
    }
    else
    {
        g_bootloaderContext.commandInterface->stateData->dataPhase.count = remaining;
        g_bootloaderContext.commandInterface->stateData->dataPhase.address = dataAddress;
    }

    return kStatus_Success;
}

//! @brief Fill Memory command handler.
void handle_fill_memory(uint8_t *packet, uint32_t packetLength)
{
    fill_memory_packet_t *command = (fill_memory_packet_t *)packet;

    status_t status =
        g_bootloaderContext.memoryInterface->fill(command->startAddress, command->byteCount, command->patternWord);

    send_generic_response(status, command->commandPacket.commandTag);
}

//! @brief Execute command handler.
void handle_execute(uint8_t *packet, uint32_t packetLength)
{
    execute_call_packet_t *command = (execute_call_packet_t *)packet;

#if !defined(BOOTLOADER_HOST)
    static uint32_t s_addr = 0;
    uint32_t call_address = command->callAddress;
    uint32_t argument_word = command->argumentWord;
    s_addr = command->stackpointer;
    status_t responseStatus = kStatus_Success;

    // Validate stack pointer address. It must either be 0 or within the RAM range.
    if (!((s_addr == 0) || is_valid_stackpointer_location(s_addr)))
    {
        // Invalid stack pointer value, respond with kStatus_InvalidArgument.
        responseStatus = kStatus_InvalidArgument;
    }

    // Validate call address.
    if (!is_valid_application_location(call_address))
    {
        // Invalid address, respond with kStatus_InvalidArgument.
        responseStatus = kStatus_InvalidArgument;
    }

#if BL_FEATURE_OTFAD_MODULE
    if (is_qspi_present())
    {
        quadspi_cache_clear();
        status_t status = otfad_init_as_needed();
        if (status != kStatus_Success)
        {
            responseStatus = kStatus_OtfadInvalidKeyBlob;
        }
        update_qspi_otfad_init_status(status);
    }
#endif

    // Send response immediately since call may not return
    send_generic_response(responseStatus, command->commandPacket.commandTag);

    if (responseStatus == kStatus_Success)
    {
        static call_function_t s_callFunction = 0;
        s_callFunction = (call_function_t)call_address;

        // Prepare for shutdown.
        shutdown_cleanup(kShutdownType_Shutdown);

        // Static variables are needed since we are changing the stack pointer out from under the compiler
        // we need to ensure the values we are using are not stored on the previous stack
        static uint32_t s_argument = 0;
        s_argument = argument_word;

        if (s_addr)
        {
            // Set main stack pointer and process stack pointer
            __set_MSP(s_addr);
            __set_PSP(s_addr);
        }

        s_callFunction(s_argument);
        // Dummy fcuntion call, should never go to this fcuntion call
        shutdown_cleanup(kShutdownType_Shutdown);
    }
#else
    // Just send a successful response.
    send_generic_response(kStatus_Success, command->commandPacket.commandTag);
#endif // BOOTLOADER_HOST
}

//! @brief Call command handler.
void handle_call(uint8_t *packet, uint32_t packetLength)
{
    execute_call_packet_t *command = (execute_call_packet_t *)packet;
    status_t responseStatus = kStatus_Success;

#if !defined(BOOTLOADER_HOST)
    // Validate call address.
    if (!is_valid_application_location(command->callAddress))
    {
        // Invalid address, respond with kStatus_InvalidArgument.
        responseStatus = kStatus_InvalidArgument;
    }
    else
    {
        call_function_t callFunction = (call_function_t)command->callAddress;
        shutdown_cleanup(kShutdownType_Cleanup);
        responseStatus = callFunction(command->argumentWord);
    }
#endif // BOOTLOADER_HOST

    send_generic_response(responseStatus, command->commandPacket.commandTag);
}

#if BL_FEATURE_FLASH_SECURITY
//! @brief Flash Security Disable command handler.
void handle_flash_security_disable(uint8_t *packet, uint32_t packetLength)
{
    flash_security_disable_packet_t *command = (flash_security_disable_packet_t *)packet;

    status_t status = kStatus_Success;
#if !defined(BOOTLOADER_HOST)
    // Flash interface wants little endian, so just send two uint32s.
    // Note: Both Main and Secondary flash share the same security control
    //  So it doesn't matter what index of allFlashState[] we use for this FLASH API.
    status = g_bootloaderContext.flashDriverInterface->flash_security_bypass(
        &g_bootloaderContext.allFlashState[kFlashIndex_Main], (uint8_t *)&command->keyLow);
#endif // BOOTLOADER_HOST

    send_generic_response(status, command->commandPacket.commandTag);
}
#endif // BL_FEATURE_FLASH_SECURITY

//! @brief  Flash Program Once command handler
void handle_flash_program_once(uint8_t *packet, uint32_t length)
{
    flash_program_once_packet_t *command = (flash_program_once_packet_t *)packet;

    status_t status = kStatus_Success;

#if !defined(BOOTLOADER_HOST)
    lock_acquire();
#if BL_FEATURE_HAS_INTERNAL_FLASH
#if !BL_DEVICE_IS_LPC_SERIES
    // Note: Both Main and Secondary flash share the same IFR Memory
    //  So it doesn't matter what index of allFlashState[] we use for this FLASH API.
    status = g_bootloaderContext.flashDriverInterface->flash_program_once(
        &g_bootloaderContext.allFlashState[kFlashIndex_Main], command->index, &command->data[0], command->byteCount);
#endif // !BL_DEVICE_IS_LPC_SERIES
#elif BL_FEATURE_OCOTP_MODULE
    status = ocotp_program_once(OCOTP, command->index, &command->data[0], command->byteCount);
    if (status == kStatus_Success)
    {
        uint32_t programData = command->data[0];
        uint32_t readData = 0;

        status = ocotp_read_once(OCOTP, command->index, &readData, command->byteCount);
        if (status == kStatus_Success)
        {
            if (programData != (readData & programData))
            {
                status = kStatus_OCOTP_ProgramFailure;
            }
        }
    }
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
    lock_release();
#endif // BOOTLOADER_HOST

    send_generic_response(status, command->commandPacket.commandTag);
}

//! @brief  Flash Read Once command handler
void handle_flash_read_once(uint8_t *packet, uint32_t length)
{
    flash_read_once_packet_t *command = (flash_read_once_packet_t *)packet;

    uint32_t readOnceItemData[2] = { 0 };

    status_t status = kStatus_Success;

#if !defined(BOOTLOADER_HOST)
    lock_acquire();
#if BL_FEATURE_HAS_INTERNAL_FLASH
#if !BL_DEVICE_IS_LPC_SERIES
    // Note: Both Main and Secondary flash share the same IFR Memory
    //  So it doesn't matter what index of allFlashState[] we use for this FLASH API.
    status = g_bootloaderContext.flashDriverInterface->flash_read_once(
        &g_bootloaderContext.allFlashState[kFlashIndex_Main], command->index, &readOnceItemData[0], command->byteCount);
#endif // !BL_DEVICE_IS_LPC_SERIES
#elif BL_FEATURE_OCOTP_MODULE
    status = ocotp_read_once(OCOTP, command->index, &readOnceItemData[0], command->byteCount);
#endif // #if BL_FEATURE_HAS_INTERNAL_FLASH
    lock_release();
#endif // BOOTLOADER_HOST

    send_flash_read_once_response(status, readOnceItemData, command->byteCount);
}

//! @brief  Flash Read Resource command handler
void handle_flash_read_resource(uint8_t *packet, uint32_t length)
{
    flash_read_resource_packet_t *command = (flash_read_resource_packet_t *)packet;

    // Start the data phase.
    reset_data_phase();
    g_bootloaderContext.commandInterface->stateData->dataPhase.count = command->byteCount;
    g_bootloaderContext.commandInterface->stateData->dataPhase.address = command->startAddress;
    g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag = kCommandTag_FlashReadResource;
    g_bootloaderContext.commandInterface->stateData->dataPhase.option = (uint8_t)command->option;
    send_flash_read_resource_response(kStatus_Success, command->byteCount);
}

//! @brief Send a generic response packet.
void send_generic_response(uint32_t commandStatus, uint32_t commandTag)
{
    generic_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = kCommandTag_GenericResponse;
    responsePacket.commandPacket.flags = 0;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2;
    responsePacket.status = commandStatus;
    responsePacket.commandTag = commandTag;

    status_t status = g_bootloaderContext.activePeripheral->packetInterface->writePacket(
        g_bootloaderContext.activePeripheral, (const uint8_t *)&responsePacket, sizeof(responsePacket),
        kPacketType_Command);
    if (status != kStatus_Success)
    {
        debug_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}

//! @brief Send a get property response packet.
void send_get_property_response(uint32_t commandStatus, uint32_t *value, uint32_t numValues)
{
    get_property_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = kCommandTag_GetPropertyResponse;
    responsePacket.commandPacket.flags = 0;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 1 + numValues; // status + value words
    responsePacket.status = commandStatus;

    for (uint32_t i = 0; i < numValues; ++i)
    {
        responsePacket.propertyValue[i] = value[i];
    }

    uint32_t packetSize =
        sizeof(responsePacket.commandPacket) + (responsePacket.commandPacket.parameterCount * sizeof(uint32_t));

    status_t status = g_bootloaderContext.activePeripheral->packetInterface->writePacket(
        g_bootloaderContext.activePeripheral, (const uint8_t *)&responsePacket, packetSize, kPacketType_Command);
    if (status != kStatus_Success)
    {
        debug_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}

#if BL_FEATURE_GEN_KEYBLOB
//! @brief Send a read memory response packet.
void send_generate_key_blob_response(uint32_t commandStatus, uint32_t length)
{
    generate_key_blob_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = kCommandTag_GenerateKeyBlobResponse;
    responsePacket.commandPacket.flags = kCommandFlag_HasDataPhase;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2;
    responsePacket.status = commandStatus;
    responsePacket.dataByteCount = length;

    status_t status = g_bootloaderContext.activePeripheral->packetInterface->writePacket(
        g_bootloaderContext.activePeripheral, (const uint8_t *)&responsePacket, sizeof(responsePacket),
        kPacketType_Command);
    if (status != kStatus_Success)
    {
        debug_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}
#endif

//! @brief Send a read memory response packet.
void send_read_memory_response(uint32_t commandStatus, uint32_t length)
{
    read_memory_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = kCommandTag_ReadMemoryResponse;
    responsePacket.commandPacket.flags = kCommandFlag_HasDataPhase;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2;
    responsePacket.status = commandStatus;
    responsePacket.dataByteCount = length;

    status_t status = g_bootloaderContext.activePeripheral->packetInterface->writePacket(
        g_bootloaderContext.activePeripheral, (const uint8_t *)&responsePacket, sizeof(responsePacket),
        kPacketType_Command);
    if (status != kStatus_Success)
    {
        debug_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}

//! @brief Send a flash read once resposne packet.
void send_flash_read_once_response(uint32_t commandStatus, uint32_t *value, uint32_t byteCount)
{
    flash_read_once_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = kCommandTag_FlashReadOnceResponse;
    responsePacket.commandPacket.flags = 0;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2; // always includes two parameters: status and byte count
    responsePacket.status = commandStatus;

    if (commandStatus == kStatus_Success)
    {
        responsePacket.commandPacket.parameterCount += byteCount / sizeof(uint32_t); // add parameter: data
        responsePacket.byteCount = byteCount;
        memcpy(responsePacket.data, value, byteCount);
    }
    else
    {
        responsePacket.byteCount = 0;
    }

    uint32_t packetSize =
        sizeof(responsePacket.commandPacket) + (responsePacket.commandPacket.parameterCount * sizeof(uint32_t));

    status_t status = g_bootloaderContext.activePeripheral->packetInterface->writePacket(
        g_bootloaderContext.activePeripheral, (const uint8_t *)&responsePacket, packetSize, kPacketType_Command);
    if (status != kStatus_Success)
    {
        debug_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}

//! @brief Send a flash read resource memory response packet.
void send_flash_read_resource_response(uint32_t commandStatus, uint32_t length)
{
    flash_read_resource_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = kCommandTag_FlashReadResourceResponse;
    responsePacket.commandPacket.flags = kCommandFlag_HasDataPhase;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2;
    responsePacket.status = commandStatus;
    responsePacket.dataByteCount = length;

    status_t status = g_bootloaderContext.activePeripheral->packetInterface->writePacket(
        g_bootloaderContext.activePeripheral, (const uint8_t *)&responsePacket, sizeof(responsePacket),
        kPacketType_Command);
    if (status != kStatus_Success)
    {
        debug_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}

#if BL_FEATURE_KEY_PROVISIONING
//! @brief Key Provisioning command handler.
void handle_key_provisioning(uint8_t *packet, uint32_t packetLength)
{
    status_t status = kStatus_Success;

    key_provisioning_packet_t *command = (key_provisioning_packet_t *)packet;
    uint32_t operation = command->operation;
    bool isDataPhaseRequired = false;
    uint32_t memoryId = kMemoryInternal;
    uint8_t *keyStoreAddress = NULL;
    uint32_t keyStoreSize = 0;

    g_bootloaderContext.commandInterface->stateData->dataPhase.option = kDataPhase_Option_Skip;
    switch (operation)
    {
        case kKeyProvisioning_Operation_Enroll:
            status = key_store_enroll();
            break;
        case kKeyProvisioning_Operation_SetUserKey:
            isDataPhaseRequired = true;
            if (command->size > BL_FEATURE_KEY_STORE_MAX_KEY_SIZE)
            {
                status = kStatus_InvalidArgument;
                break;
            }
            // Start the data phase.
            reset_data_phase();
            g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag = kCommandTag_KeyProvisioning;
            g_bootloaderContext.commandInterface->stateData->dataPhase.count = command->size;
            g_bootloaderContext.commandInterface->stateData->dataPhase.address = (uint32_t)&s_userKeyBuffer;
            g_bootloaderContext.commandInterface->stateData->dataPhase.option = kDataPhase_Option_Consumer;
            g_bootloaderContext.commandInterface->stateData->dataPhase.argument0 = command->operation;
            g_bootloaderContext.commandInterface->stateData->dataPhase.argument1 = command->type;
            g_bootloaderContext.commandInterface->stateData->dataPhase.argument2 = command->size;
            break;
        case kKeyProvisioning_Operation_SetIntrinsicKey:
            status = key_store_set_key(NULL, command->size, command->type);
            break;
        case kKeyProvisioning_Operation_WriteNonVolatile:
            memoryId = command->type;
            status = key_store_write_nonvolatile(memoryId);
            break;
        case kKeyProvisioning_Operation_ReadNonVolatile:
            memoryId = command->type;
            status = key_store_read_nonvolatile(memoryId);
            break;
        case kKeyProvisioning_Operation_WriteKeyStore:
            isDataPhaseRequired = true;
            // Start the data phase.
            reset_data_phase();
            g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag = kCommandTag_KeyProvisioning;
            g_bootloaderContext.commandInterface->stateData->dataPhase.count = sizeof(s_userKeyStoreBuffer);
            g_bootloaderContext.commandInterface->stateData->dataPhase.address = (uint32_t)s_userKeyStoreBuffer;
            g_bootloaderContext.commandInterface->stateData->dataPhase.option = kDataPhase_Option_Consumer;
            g_bootloaderContext.commandInterface->stateData->dataPhase.argument0 = command->operation;
            break;
        case kKeyProvisioning_Operation_ReadKeyStore:
            isDataPhaseRequired = true;
            reset_data_phase();
            status = key_store_get(&keyStoreAddress, &keyStoreSize);
            if (status == kStatus_Success)
            {
                // Start the data phase only when key store is available.
                g_bootloaderContext.commandInterface->stateData->dataPhase.commandTag = kCommandTag_KeyProvisioning;
                g_bootloaderContext.commandInterface->stateData->dataPhase.count = keyStoreSize;
                g_bootloaderContext.commandInterface->stateData->dataPhase.address = (uint32_t)keyStoreAddress;
                g_bootloaderContext.commandInterface->stateData->dataPhase.option = kDataPhase_Option_Producer;
            }
            break;
        default:
            status = kStatus_InvalidArgument;
    }

    if (isDataPhaseRequired)
    {
        send_key_provisioning_response(status, g_bootloaderContext.commandInterface->stateData->dataPhase.count);
    }
    else
    {
        send_generic_response(status, command->commandPacket.commandTag);
    }
}

//! @brief Send a key provisioning response packet.
void send_key_provisioning_response(uint32_t commandStatus, uint32_t length)
{
    key_provisioning_response_packet_t responsePacket;
    responsePacket.commandPacket.commandTag = kCommandTag_KeyProvisioningResponse;
    responsePacket.commandPacket.flags = kCommandFlag_HasDataPhase;
    responsePacket.commandPacket.reserved = 0;
    responsePacket.commandPacket.parameterCount = 2;
    responsePacket.status = commandStatus;
    responsePacket.dataByteCount = length;

    status_t status = g_bootloaderContext.activePeripheral->packetInterface->writePacket(
        g_bootloaderContext.activePeripheral, (const uint8_t *)&responsePacket, sizeof(responsePacket),
        kPacketType_Command);
    if (status != kStatus_Success)
    {
        debug_printf("Error: writePacket returned status 0x%x\r\n", status);
    }
}
#endif // BL_FEATURE_KEY_PROVISIONING

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
