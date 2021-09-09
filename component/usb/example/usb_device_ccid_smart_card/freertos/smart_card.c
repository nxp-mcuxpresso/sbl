/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_ccid.h"

#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "emvl1_interface.h"
#include "smart_card.h"

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

#include <stdio.h>
#include <stdlib.h>
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
#include "usb_phy.h"
#endif

#if (USB_DEVICE_CONFIG_USE_TASK < 1U)
#error USB_DEVICE_CONFIG_USE_TASK need to > 0U, Please change the MARCO USB_DEVICE_CONFIG_USE_TASK in file "usb_device_config.h".
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle);
#endif

static usb_status_t USB_DeviceCcidSmartCardCommandParse(usb_device_ccid_command_struct_t *command);
static usb_status_t USB_DeviceCcidCommandHandle(class_handle_t classHandle,
                                                usb_device_ccid_command_struct_t *usbDeviceCcidCommand);
static usb_status_t USB_DeviceCcidCallback(class_handle_t classHandle, uint32_t event, void *callbackParam);
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);
static void EMVL1_CcidCallback(uint8_t event, void *buffer, uint32_t size);
static void USB_DeviceApplicationInit(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern usb_device_class_struct_t g_UsbDeviceCcidClass;

usb_ccid_smart_card_struct_t g_UsbDeviceCcidSmartCard;

/* CCID config */
usb_device_class_config_struct_t g_UsbDeviceCcidConfig[1] = {
    {
        USB_DeviceCcidCallback, (class_handle_t)NULL, &g_UsbDeviceCcidClass,
    },
};

/* CCID Config list */
usb_device_class_config_list_struct_t g_UsbDeviceCcidConfigList = {
    g_UsbDeviceCcidConfig, USB_DeviceCallback, 1U,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Parse the ccid command */
static usb_status_t USB_DeviceCcidSmartCardCommandParse(usb_device_ccid_command_struct_t *command)
{
    usb_device_ccid_transfer_block_command_t *usbDeviceCcidCommand =
        (usb_device_ccid_transfer_block_command_t *)(command->commandBuffer);
    usb_device_ccid_data_block_response_t *ccidResponse = NULL;
    uint32_t receiveLength = 0U;
    uint32_t temp32;
    uint8_t error = kStatus_CCID_EMV_Error;

    ccidResponse = (usb_device_ccid_data_block_response_t *)&g_UsbDeviceCcidSmartCard
                       .slotsResponseBuffer[usbDeviceCcidCommand->bSlot][0];

    ccidResponse->bError = 0U;
    ccidResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_DATABLOCK;
    ccidResponse->bSeq = usbDeviceCcidCommand->bSeq;
    ccidResponse->bSlot = usbDeviceCcidCommand->bSlot;
    ccidResponse->bStatus = 0U;
    ccidResponse->bChainParameter = 0x00U;
    USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, ccidResponse->dwLength);

    temp32 = USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidCommand->dwLength);

    if (temp32 > (command->commandLength - USB_DEVICE_CCID_COMMAND_HEADER_LENGTH))
    {
        ccidResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_BAD_LENGTH;
        ccidResponse->bStatus =
            (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) | (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
    }
    else
    {
        receiveLength = USB_DEVICE_CCID_RESPONSE_BUFFER_LENGTH - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH;

        error = EMVL1_SendApduCommand(usbDeviceCcidCommand->abData, temp32, ccidResponse->abData, &receiveLength);

        USB_LONG_TO_LITTLE_ENDIAN_DATA(receiveLength, ccidResponse->dwLength);

        if (error != kStatus_CCID_EMV_Success)
        {
            ccidResponse->bStatus =
                (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) | (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
            ccidResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_HW_ERROR;

            EMVL1_SmartCardPowerOff();

            usb_device_ccid_data_block_response_t *ccid_datablock =
                (usb_device_ccid_data_block_response_t *)&g_UsbDeviceCcidSmartCard
                    .slotsAtrBuffer[usbDeviceCcidCommand->bSlot][0];
            receiveLength = USB_DEVICE_CCID_ATR_BUFFER_LENGTH - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH;
            EMVL1_SmartCardPowerOn(ccid_datablock->abData, &receiveLength);
        }
    }
    command->responseBuffer = (uint8_t *)ccidResponse;
    command->responseLength =
        USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH + USB_LONG_FROM_LITTLE_ENDIAN_DATA(ccidResponse->dwLength);
    return kStatus_USB_Success;
}

/* Handle USB device ccid command */
static usb_status_t USB_DeviceCcidCommandHandle(class_handle_t classHandle,
                                                usb_device_ccid_command_struct_t *usbDeviceCcidCommand)
{
    usb_device_ccid_parameters_response_common_t *usbDeviceCcidParamResponse;
    usb_device_ccid_common_command_t *commonRequest;
    usb_device_ccid_slot_status_response_t *usbDeviceCcidSlotStatusResponse;
    usb_device_ccid_data_block_response_t *usbDeviceCcidDatablockResponse;
    emv_card_data_information_struct_t usbDeviceCcidCardInformation;
    uint32_t length;
    usb_status_t usbError = kStatus_USB_InvalidRequest;
    uint8_t error = kStatus_CCID_EMV_Success;

    if ((((class_handle_t)NULL == classHandle) || (NULL == usbDeviceCcidCommand)) ||
        (NULL == usbDeviceCcidCommand->commandBuffer) ||
        (USB_UNINITIALIZED_VAL_32 == usbDeviceCcidCommand->commandLength))
    {
        return kStatus_USB_Error;
    }

    commonRequest = (usb_device_ccid_common_command_t *)usbDeviceCcidCommand->commandBuffer;

    for (int i = 0U; i < usbDeviceCcidCommand->commandLength; i++)
    {
        g_UsbDeviceCcidSmartCard.slotsCommandBuffer[commonRequest->bSlot][i] = usbDeviceCcidCommand->commandBuffer[i];
    }

    switch (commonRequest->bMessageType)
    {
        case USB_DEVICE_CCID_PC_TO_RDR_ICCPOWERON:
            /* ICC power on command */
            usbDeviceCcidDatablockResponse = (usb_device_ccid_data_block_response_t *)&g_UsbDeviceCcidSmartCard
                                                 .slotsAtrBuffer[commonRequest->bSlot][0];
            length = USB_DEVICE_CCID_ATR_BUFFER_LENGTH - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH;
            error = EMVL1_SmartCardPowerOn(usbDeviceCcidDatablockResponse->abData, &length);
            g_UsbDeviceCcidSmartCard.clockStatus = USB_DEVICE_CCID_CLCOK_STATUS_CLOCK_RUNNING;
            if (kStatus_CCID_EMV_Success == error)
            {
                g_UsbDeviceCcidSmartCard.powerOn = 1U;
                usbDeviceCcidDatablockResponse->bError = 0U;
                usbDeviceCcidDatablockResponse->bStatus =
                    (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_PROCESSED_NO_ERROR) |
                    (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
            }
            else
            {
                length = 0U;
                if (kStatus_CCID_EMV_CardRemoved == error)
                {
                    usbDeviceCcidDatablockResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_ICC_MUTE;
                    usbDeviceCcidDatablockResponse->bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) |
                                                              (USB_DEVICE_CCID_SLOT_STATUS_ICC_NOT_PRESENT);
                }
                else
                {
                    usbDeviceCcidDatablockResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_HW_ERROR;
                    usbDeviceCcidDatablockResponse->bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) |
                                                              (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_INACTIVE);
                }
            }
            if (EMVL1_SmartCardGetProtocol() & EMV_SMART_CARD_PROTOCOL_T1)
            {
                /* T = 1U */
                g_UsbDeviceCcidSmartCard.protocol = USB_DEVICE_CCID_PROTOCOL_NUMBER_T1;
            }
            else if (EMVL1_SmartCardGetProtocol() & EMV_SMART_CARD_PROTOCOL_T0)
            {
                /* T = 0U */
                g_UsbDeviceCcidSmartCard.protocol = USB_DEVICE_CCID_PROTOCOL_NUMBER_T0;
            }
            else
            {
                usb_echo("This card is not a EMV card.\r\n");
                usbDeviceCcidDatablockResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_ICC_PROTOCOL_NOT_SUPPORTED;
                usbDeviceCcidDatablockResponse->bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) |
                                                          (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_INACTIVE);
            }

            /* set default card paramter */
            if (kStatus_CCID_EMV_Success == EMVL1_SmartCardGetInformation(&usbDeviceCcidCardInformation))
            {
                usbDeviceCcidParamResponse = (usb_device_ccid_parameters_response_common_t *)&g_UsbDeviceCcidSmartCard
                                                 .slotsCurrentParameter[0][0];
                if (USB_DEVICE_CCID_PROTOCOL_NUMBER_T1 == g_UsbDeviceCcidSmartCard.protocol)
                {
                    USB_LONG_TO_LITTLE_ENDIAN_DATA(
                        sizeof(usb_device_ccid_parameters_T1_response_t) - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH,
                        usbDeviceCcidParamResponse->t1.dwLength);
                    usbDeviceCcidParamResponse->t1.bmFindexDindex =
                        ((usbDeviceCcidCardInformation.FI & 0x0FU) << 0x04U) |
                        (usbDeviceCcidCardInformation.DI & 0x0FU);
                    usbDeviceCcidParamResponse->t1.bmTCCKST1 =
                        0x10U | (kEmvSmartCardConventionInverse == usbDeviceCcidCardInformation.convention) |
                        (kEmvSmartCardChecksumTypeCRC == usbDeviceCcidCardInformation.paramUnion.t1.checksumType);
                    usbDeviceCcidParamResponse->t1.bGuardTimeT1 = usbDeviceCcidCardInformation.GTN;
                    usbDeviceCcidParamResponse->t1.bmWaitingIntegersT1 =
                        ((usbDeviceCcidCardInformation.paramUnion.t1.BWI & 0x0FU) << 0x04U) |
                        (usbDeviceCcidCardInformation.paramUnion.t1.CWI & 0x0FU);
                    usbDeviceCcidParamResponse->t1.bClockStop = 0x00U;
                    usbDeviceCcidParamResponse->t1.bIFSC = usbDeviceCcidCardInformation.paramUnion.t1.IFSC;
                    usbDeviceCcidParamResponse->t1.bNadValue = g_UsbDeviceCcidSmartCard.nodeAddress;
                }
                else
                {
                    USB_LONG_TO_LITTLE_ENDIAN_DATA(
                        sizeof(usb_device_ccid_parameters_T0_response_t) - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH,
                        usbDeviceCcidParamResponse->t0.dwLength);
                    usbDeviceCcidParamResponse->t0.bmFindexDindex =
                        ((usbDeviceCcidCardInformation.FI & 0x0FU) << 0x04U) |
                        (usbDeviceCcidCardInformation.DI & 0x0FU);
                    usbDeviceCcidParamResponse->t0.bmTCCKST0 =
                        0x00U | (kEmvSmartCardConventionInverse == usbDeviceCcidCardInformation.convention);
                    usbDeviceCcidParamResponse->t0.bGuardTimeT0 = usbDeviceCcidCardInformation.GTN;
                    usbDeviceCcidParamResponse->t0.bWaitingIntegerT0 = usbDeviceCcidCardInformation.paramUnion.t0.WI;
                    usbDeviceCcidParamResponse->t0.bClockStop = 0x00U;
                }
            }

            usbDeviceCcidDatablockResponse->bChainParameter = 0x00U;
            usbDeviceCcidDatablockResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_DATABLOCK;
            usbDeviceCcidDatablockResponse->bSeq = commonRequest->bSeq;
            usbDeviceCcidDatablockResponse->bSlot = commonRequest->bSlot;
            USB_LONG_TO_LITTLE_ENDIAN_DATA(length, usbDeviceCcidDatablockResponse->dwLength);
            usbError = kStatus_USB_Success;
            usbDeviceCcidCommand->responseBuffer = (uint8_t *)usbDeviceCcidDatablockResponse;
            usbDeviceCcidCommand->responseLength =
                USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH +
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidDatablockResponse->dwLength);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_ICCPOWEROFF:
            /* ICC pwer off command */
            usbDeviceCcidSlotStatusResponse = (usb_device_ccid_slot_status_response_t *)&g_UsbDeviceCcidSmartCard
                                                  .slotsResponseBuffer[commonRequest->bSlot][0];
            error = EMVL1_SmartCardPowerOff();
            g_UsbDeviceCcidSmartCard.powerOn = 0U;
            g_UsbDeviceCcidSmartCard.clockStatus = USB_DEVICE_CCID_CLCOK_STATUS_CLOCK_STOPPED_UNKNOWN;
            if (kStatus_CCID_EMV_Success == error)
            {
                usbDeviceCcidSlotStatusResponse->bError = 0U;
                usbDeviceCcidSlotStatusResponse->bStatus =
                    (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_PROCESSED_NO_ERROR) |
                    (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_INACTIVE);
            }
            else
            {
                usbDeviceCcidSlotStatusResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_HW_ERROR;
                usbDeviceCcidSlotStatusResponse->bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) |
                                                           (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
            }
            usbDeviceCcidSlotStatusResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_SLOTSTATUS;
            usbDeviceCcidSlotStatusResponse->bSeq = commonRequest->bSeq;
            usbDeviceCcidSlotStatusResponse->bSlot = commonRequest->bSlot;
            usbDeviceCcidSlotStatusResponse->bClockStatus = g_UsbDeviceCcidSmartCard.clockStatus;
            USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, usbDeviceCcidSlotStatusResponse->dwLength);
            usbError = kStatus_USB_Success;
            usbDeviceCcidCommand->responseBuffer = (uint8_t *)usbDeviceCcidSlotStatusResponse;
            usbDeviceCcidCommand->responseLength =
                USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH +
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidSlotStatusResponse->dwLength);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_GETSLOTSTATUS:
            /* Get slot status command */
            usbDeviceCcidSlotStatusResponse = (usb_device_ccid_slot_status_response_t *)&g_UsbDeviceCcidSmartCard
                                                  .slotsResponseBuffer[commonRequest->bSlot][0];
            error = EMVL1_SmartCardPresence();
            if (kStatus_CCID_EMV_Success == error)
            {
                usbDeviceCcidSlotStatusResponse->bStatus =
                    (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_PROCESSED_NO_ERROR) |
                    (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
            }
            else
            {
                usbDeviceCcidSlotStatusResponse->bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) |
                                                           (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_INACTIVE);
            }
            usbDeviceCcidSlotStatusResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_SLOTSTATUS;
            usbDeviceCcidSlotStatusResponse->bSeq = commonRequest->bSeq;
            usbDeviceCcidSlotStatusResponse->bSlot = commonRequest->bSlot;
            usbDeviceCcidSlotStatusResponse->bClockStatus = g_UsbDeviceCcidSmartCard.clockStatus;
            usbDeviceCcidSlotStatusResponse->bError = 0U;
            USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, usbDeviceCcidSlotStatusResponse->dwLength);
            usbError = kStatus_USB_Success;
            usbDeviceCcidCommand->responseBuffer = (uint8_t *)usbDeviceCcidSlotStatusResponse;
            usbDeviceCcidCommand->responseLength =
                USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH +
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidSlotStatusResponse->dwLength);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_XFRBLOCK:
            /* APDU message received */
            usb_echo("\r\nNew command received!\r\n");
            usbError = USB_DeviceCcidSmartCardCommandParse(usbDeviceCcidCommand);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_GETPARAMETERS:
            /* Get paramters command */
            usbDeviceCcidParamResponse = (usb_device_ccid_parameters_response_common_t *)&g_UsbDeviceCcidSmartCard
                                             .slotsCurrentParameter[commonRequest->bSlot][0];

            usbDeviceCcidParamResponse->common.bMessageType = USB_DEVICE_CCID_RDR_TO_PC_PARAMETERS;
            usbDeviceCcidParamResponse->common.bSeq = commonRequest->bSeq;
            usbDeviceCcidParamResponse->common.bSlot = commonRequest->bSlot;
            usbDeviceCcidParamResponse->common.bProtocolNum = g_UsbDeviceCcidSmartCard.protocol;

            if (USB_DEVICE_CCID_PROTOCOL_NUMBER_T1 == g_UsbDeviceCcidSmartCard.protocol)
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(
                    sizeof(usb_device_ccid_parameters_T1_response_t) - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH,
                    usbDeviceCcidParamResponse->t1.dwLength);
            }
            else
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(
                    sizeof(usb_device_ccid_parameters_T0_response_t) - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH,
                    usbDeviceCcidParamResponse->t0.dwLength);
            }
            usbError = kStatus_USB_Success;
            usbDeviceCcidCommand->responseBuffer = (uint8_t *)usbDeviceCcidParamResponse;
            usbDeviceCcidCommand->responseLength =
                USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH +
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidParamResponse->common.dwLength);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_RESETPARAMETERS:
            /* Reset paramters command */
            usbDeviceCcidParamResponse = (usb_device_ccid_parameters_response_common_t *)&g_UsbDeviceCcidSmartCard
                                             .slotsCurrentParameter[commonRequest->bSlot][0];

            usbDeviceCcidParamResponse->common.bMessageType = USB_DEVICE_CCID_RDR_TO_PC_PARAMETERS;
            usbDeviceCcidParamResponse->common.bSeq = commonRequest->bSeq;
            usbDeviceCcidParamResponse->common.bSlot = commonRequest->bSlot;
            usbDeviceCcidParamResponse->common.bProtocolNum = g_UsbDeviceCcidSmartCard.protocol;
            if (kStatus_CCID_EMV_Success == EMVL1_SmartCardGetInformation(&usbDeviceCcidCardInformation))
            {
                if (USB_DEVICE_CCID_PROTOCOL_NUMBER_T1 == g_UsbDeviceCcidSmartCard.protocol)
                {
                    USB_LONG_TO_LITTLE_ENDIAN_DATA(
                        sizeof(usb_device_ccid_parameters_T1_response_t) - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH,
                        usbDeviceCcidParamResponse->t1.dwLength);
                    usbDeviceCcidParamResponse->t1.bmFindexDindex =
                        ((usbDeviceCcidCardInformation.FI & 0x0FU) << 0x04U) |
                        (usbDeviceCcidCardInformation.DI & 0x0FU);
                    usbDeviceCcidParamResponse->t1.bmTCCKST1 =
                        0x10U | (kEmvSmartCardConventionInverse == usbDeviceCcidCardInformation.convention) |
                        (kEmvSmartCardChecksumTypeCRC == usbDeviceCcidCardInformation.paramUnion.t1.checksumType);
                    usbDeviceCcidParamResponse->t1.bGuardTimeT1 = usbDeviceCcidCardInformation.GTN;
                    usbDeviceCcidParamResponse->t1.bmWaitingIntegersT1 =
                        ((usbDeviceCcidCardInformation.paramUnion.t1.BWI & 0x0FU) << 0x04U) |
                        (usbDeviceCcidCardInformation.paramUnion.t1.CWI & 0x0FU);
                    usbDeviceCcidParamResponse->t1.bClockStop = 0x00U;
                    usbDeviceCcidParamResponse->t1.bIFSC = usbDeviceCcidCardInformation.paramUnion.t1.IFSC;
                    usbDeviceCcidParamResponse->t1.bNadValue = g_UsbDeviceCcidSmartCard.nodeAddress;
                }
                else
                {
                    USB_LONG_TO_LITTLE_ENDIAN_DATA(
                        sizeof(usb_device_ccid_parameters_T0_response_t) - USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH,
                        usbDeviceCcidParamResponse->t0.dwLength);
                    usbDeviceCcidParamResponse->t0.bmFindexDindex =
                        ((usbDeviceCcidCardInformation.FI & 0x0FU) << 0x04U) |
                        (usbDeviceCcidCardInformation.DI & 0x0FU);
                    usbDeviceCcidParamResponse->t0.bmTCCKST0 =
                        0x00U | (kEmvSmartCardConventionInverse == usbDeviceCcidCardInformation.convention);
                    usbDeviceCcidParamResponse->t0.bGuardTimeT0 = usbDeviceCcidCardInformation.GTN;
                    usbDeviceCcidParamResponse->t0.bWaitingIntegerT0 = usbDeviceCcidCardInformation.paramUnion.t0.WI;
                    usbDeviceCcidParamResponse->t0.bClockStop = 0x00U;
                }
                usbDeviceCcidParamResponse->common.bStatus =
                    (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_PROCESSED_NO_ERROR) |
                    (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
                usbDeviceCcidParamResponse->common.bError = 0U;
            }
            else
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, usbDeviceCcidParamResponse->common.dwLength);
                usbDeviceCcidParamResponse->common.bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) |
                                                             (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
                usbDeviceCcidParamResponse->common.bError = USB_DEVICE_CCID_SLOT_ERROR_PROTOCOL_INVALID;
            }
            usbError = kStatus_USB_Success;
            usbDeviceCcidCommand->responseBuffer = (uint8_t *)usbDeviceCcidParamResponse;
            usbDeviceCcidCommand->responseLength =
                USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH +
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidParamResponse->common.dwLength);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_SETPARAMETERS:
            /* Set parameters command */
            usbDeviceCcidSlotStatusResponse = (usb_device_ccid_slot_status_response_t *)&g_UsbDeviceCcidSmartCard
                                                  .slotsResponseBuffer[commonRequest->bSlot][0];

            usbDeviceCcidSlotStatusResponse->bError = 0U;
            usbDeviceCcidSlotStatusResponse->bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_PROCESSED_NO_ERROR) |
                                                       (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
            usbDeviceCcidSlotStatusResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_SLOTSTATUS;
            usbDeviceCcidSlotStatusResponse->bSeq = commonRequest->bSeq;
            usbDeviceCcidSlotStatusResponse->bSlot = commonRequest->bSlot;
            usbDeviceCcidSlotStatusResponse->bClockStatus = g_UsbDeviceCcidSmartCard.clockStatus;
            USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, usbDeviceCcidSlotStatusResponse->dwLength);
            usbError = kStatus_USB_Success;
            usbDeviceCcidCommand->responseBuffer = (uint8_t *)usbDeviceCcidSlotStatusResponse;
            usbDeviceCcidCommand->responseLength =
                USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH +
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidSlotStatusResponse->dwLength);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_ESCAPE:
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_ICCCLOCK:
            /* Get ICC clock status command */
            usbDeviceCcidSlotStatusResponse = (usb_device_ccid_slot_status_response_t *)&g_UsbDeviceCcidSmartCard
                                                  .slotsResponseBuffer[commonRequest->bSlot][0];
            usbDeviceCcidSlotStatusResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_SLOTSTATUS;
            usbDeviceCcidSlotStatusResponse->bSeq = commonRequest->bSeq;
            usbDeviceCcidSlotStatusResponse->bSlot = commonRequest->bSlot;
            usbDeviceCcidSlotStatusResponse->bClockStatus = g_UsbDeviceCcidSmartCard.clockStatus;
            usbDeviceCcidSlotStatusResponse->bError = 0U;
            usbDeviceCcidSlotStatusResponse->bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_PROCESSED_NO_ERROR) |
                                                       (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
            USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, usbDeviceCcidSlotStatusResponse->dwLength);
            usbError = kStatus_USB_Success;
            usbDeviceCcidCommand->responseBuffer = (uint8_t *)usbDeviceCcidSlotStatusResponse;
            usbDeviceCcidCommand->responseLength =
                USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH +
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidSlotStatusResponse->dwLength);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_T0APDU:
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_SECURE:
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_MECHANICAL:
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_ABORT:
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_SETDATARATEANDCLOCKFREQUENCY:
            break;
        default:
            break;
    }
    return usbError;
}

/* USB device ccid class callback */
static usb_status_t USB_DeviceCcidCallback(class_handle_t classHandle, uint32_t event, void *callbackParam)
{
    uint8_t *temp8;
    usb_device_ccid_control_request_struct_t *controlRequest;
    usb_status_t error = kStatus_USB_Error;

    switch (event)
    {
        case kUSB_DeviceCcidEventCommandReceived:
            /* Command received */
            error = USB_DeviceCcidCommandHandle(classHandle, callbackParam);
            break;
        case kUSB_DeviceCcidEventResponseSent:
            break;
        case kUSB_DeviceCcidEventGetSlotCount:
            if (callbackParam)
            {
                /* Get the slot count */
                temp8 = (uint8_t *)callbackParam;
                *temp8 = USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS;
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceCcidEventGetSlotStatus:
            if (callbackParam)
            {
                /* Get the slot status */
                usb_device_ccid_slot_status_struct_t *slotStatus;
                slotStatus = (usb_device_ccid_slot_status_struct_t *)callbackParam;
                if (USB_DEVICE_CCID_SMART_CARD_SLOT_INDEX == slotStatus->slot)
                {
                    slotStatus->clockStatus = g_UsbDeviceCcidSmartCard.clockStatus;
                    if (!g_UsbDeviceCcidSmartCard.present)
                    {
                        slotStatus->present = USB_DEVICE_CCID_SLOT_STATUS_ICC_NOT_PRESENT;
                    }
                    else
                    {
                        if (g_UsbDeviceCcidSmartCard.powerOn)
                        {
                            slotStatus->present = USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE;
                        }
                        else
                        {
                            slotStatus->present = USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_INACTIVE;
                        }
                    }
                    error = kStatus_USB_Success;
                }
            }
            break;
        case kUSB_DeviceCcidEventCommandAbort:
            break;
        case kUSB_DeviceCcidEventGetClockFrequencies:
            if (callbackParam)
            {
                controlRequest = (usb_device_ccid_control_request_struct_t *)callbackParam;
                controlRequest->buffer = (uint8_t *)&g_UsbDeviceCcidSmartCard.clockFrequency;
                controlRequest->length = sizeof(g_UsbDeviceCcidSmartCard.clockFrequency);
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceCcidEventGetDataRate:
            if (callbackParam)
            {
                controlRequest = (usb_device_ccid_control_request_struct_t *)callbackParam;
                controlRequest->buffer = (uint8_t *)&g_UsbDeviceCcidSmartCard.dataRate;
                controlRequest->length = sizeof(g_UsbDeviceCcidSmartCard.dataRate);
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceCcidEventSlotChangeSent:
            if (!g_UsbDeviceCcidSmartCard.present)
            {
                EMVL1_SmartCardPowerOff();
                g_UsbDeviceCcidSmartCard.powerOn = 0U;
            }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCcidEventHardwareErrorSent:
            break;
        default:
            break;
    }

    return error;
}

/* Device callback */
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Success;
    uint8_t *temp8 = (uint8_t *)param;
    uint16_t *temp16 = (uint16_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            /* USB device bus reset signal detected */
            g_UsbDeviceCcidSmartCard.attach = 0U;
            g_UsbDeviceCcidSmartCard.remoteWakeup = 0U;
            g_UsbDeviceCcidSmartCard.suspend = 0U;
            g_UsbDeviceCcidSmartCard.powerOn = 0U;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            if (kStatus_USB_Success == USB_DeviceGetStatus(g_UsbDeviceCcidSmartCard.deviceHandle,
                                                           kUSB_DeviceStatusSpeed, &g_UsbDeviceCcidSmartCard.speed))
            {
                USB_DeviceSetSpeed(g_UsbDeviceCcidSmartCard.speed);
            }
#endif
            error = kStatus_USB_Success;
        }
        break;

        case kUSB_DeviceEventSuspend:
        {
            /* USB device bus suspend signal detected */
            if (g_UsbDeviceCcidSmartCard.attach)
            {
                g_UsbDeviceCcidSmartCard.suspend = 1U;
                usb_echo("USB ccid device suspend\r\n");
                error = kStatus_USB_Success;
            }
        }
        break;
        case kUSB_DeviceEventResume:
        {
            /* USB device bus resume signal detected */
            if (g_UsbDeviceCcidSmartCard.attach)
            {
                g_UsbDeviceCcidSmartCard.suspend = 0U;
                usb_echo("USB ccid device resume\r\n");
                error = kStatus_USB_Success;
            }
        }
        break;
        case kUSB_DeviceEventSetRemoteWakeup:
            if (param)
            {
                if (g_UsbDeviceCcidSmartCard.attach)
                {
                    g_UsbDeviceCcidSmartCard.remoteWakeup = *temp8;
                    error = kStatus_USB_Success;
                }
            }
            break;
        case kUSB_DeviceEventSetConfiguration:
            if (USB_DEVICE_CCID_SMART_CARD_CONFIGURE_INDEX == (*temp8))
            {
                g_UsbDeviceCcidSmartCard.attach = 1U;
                g_UsbDeviceCcidSmartCard.currentConfiguration = *temp8;
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceEventSetInterface:
            if (g_UsbDeviceCcidSmartCard.attach)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if (interface < USB_DEVICE_CCID_SMART_CARD_INTERFACE_COUNT)
                {
                    g_UsbDeviceCcidSmartCard.currentInterfaceAlternateSetting[interface] = alternateSetting;
                    error = kStatus_USB_Success;
                }
                else
                {
                    error = kStatus_USB_InvalidRequest;
                }
            }
            break;
        case kUSB_DeviceEventGetConfiguration:
            if (param)
            {
                *temp8 = g_UsbDeviceCcidSmartCard.currentConfiguration;
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceEventGetInterface:
            if (param)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                if (interface < USB_DEVICE_CCID_SMART_CARD_INTERFACE_COUNT)
                {
                    *temp16 =
                        (*temp16 & 0xFF00U) | g_UsbDeviceCcidSmartCard.currentInterfaceAlternateSetting[interface];
                    error = kStatus_USB_Success;
                }
                else
                {
                    error = kStatus_USB_InvalidRequest;
                }
            }
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param)
            {
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param)
            {
                error = USB_DeviceGetConfigurationDescriptor(handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetStringDescriptor:
            if (param)
            {
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
        default:
            break;
    }

    return error;
}

/* the slot changed callback */
static void EMVL1_CcidCallback(uint8_t event, void *buffer, uint32_t size)
{
    switch (event)
    {
        case kEmvSmartCardEventCardInserted:
            g_UsbDeviceCcidSmartCard.present = 1U;
            if (g_UsbDeviceCcidSmartCard.attach)
            {
                if ((g_UsbDeviceCcidSmartCard.suspend) && (g_UsbDeviceCcidSmartCard.remoteWakeup))
                {
                    USB_DeviceSetStatus(g_UsbDeviceCcidSmartCard.deviceHandle, kUSB_DeviceStatusBus, NULL);
                }
            }
            USB_DeviceCcidNotifySlotChange(g_UsbDeviceCcidSmartCard.classHandle, USB_DEVICE_CCID_SMART_CARD_SLOT_INDEX,
                                           kUSB_DeviceCcidSlotStatePresent);
            usb_echo("CCID card is inserted!\r\n");
            break;
        case kEmvSmartCardEventCardRemoved:
            g_UsbDeviceCcidSmartCard.present = 0U;
            g_UsbDeviceCcidSmartCard.clockStatus = USB_DEVICE_CCID_CLCOK_STATUS_CLOCK_STOPPED_UNKNOWN;
            if (g_UsbDeviceCcidSmartCard.attach)
            {
                if ((g_UsbDeviceCcidSmartCard.suspend) && (g_UsbDeviceCcidSmartCard.remoteWakeup))
                {
                    USB_DeviceSetStatus(g_UsbDeviceCcidSmartCard.deviceHandle, kUSB_DeviceStatusBus, NULL);
                }
            }
            USB_DeviceCcidNotifySlotChange(g_UsbDeviceCcidSmartCard.classHandle, USB_DEVICE_CCID_SMART_CARD_SLOT_INDEX,
                                           kUSB_DeviceCcidSlotStateNoPresent);
            EMVL1_SmartCardPowerOff();
            usb_echo("CCID card is removed!\r\n");
            break;
        default:
            break;
    }
}

static void USB_DeviceApplicationInit(void)
{
    USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    g_UsbDeviceCcidSmartCard.speed = USB_SPEED_FULL;
    g_UsbDeviceCcidSmartCard.attach = 0U;
    g_UsbDeviceCcidSmartCard.deviceHandle = NULL;

    if (kStatus_USB_Success !=
        USB_DeviceClassInit(CONTROLLER_ID, &g_UsbDeviceCcidConfigList, &g_UsbDeviceCcidSmartCard.deviceHandle))
    {
        usb_echo("USB device CCID smart card failed\r\n");
        return;
    }
    else
    {
        usb_echo("USB device CCID smart card demo\r\n");
        g_UsbDeviceCcidSmartCard.classHandle = g_UsbDeviceCcidConfigList.config[0].classHandle;
    }

    USB_DeviceIsrEnable();

    g_UsbDeviceCcidSmartCard.clockFrequency = USB_DEVICE_CCID_SMART_CARD_DEFAULT_CLOCK;
    g_UsbDeviceCcidSmartCard.dataRate = USB_DEVICE_CCID_SMART_CARD_DATA_RATE;
    g_UsbDeviceCcidSmartCard.clockStatus = USB_DEVICE_CCID_CLCOK_STATUS_CLOCK_STOPPED_UNKNOWN;
    g_UsbDeviceCcidSmartCard.protocol = USB_DEVICE_CCID_PROTOCOL_NUMBER_T0;

    g_UsbDeviceCcidSmartCard.present = 0U;
    g_UsbDeviceCcidSmartCard.powerOn = 0U;

    EMVL1_SmartCardInit(EMVL1_CcidCallback);

    USB_DeviceRun(g_UsbDeviceCcidSmartCard.deviceHandle);
}

#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTask(void *handle)
{
    while (1U)
    {
        USB_DeviceTaskFn(handle);
    }
}
#endif

void APP_task(void *handle)
{
    USB_DeviceApplicationInit();

#if USB_DEVICE_CONFIG_USE_TASK
    if (g_UsbDeviceCcidSmartCard.deviceHandle)
    {
        if (xTaskCreate(USB_DeviceTask,                            /* pointer to the task */
                        "usb device task",                         /* task name for kernel awareness debugging */
                        5000L / sizeof(portSTACK_TYPE),            /* task stack size */
                        g_UsbDeviceCcidSmartCard.deviceHandle,     /* optional task startup argument */
                        5U,                                        /* initial priority */
                        &g_UsbDeviceCcidSmartCard.deviceTaskHandle /* optional task handle to create */
                        ) != pdPASS)
        {
            usb_echo("usb device task create failed!\r\n");
            return;
        }
    }
#endif

    if (xTaskCreate(EMVL1_StartCardTask,                          /* pointer to the task */
                    "smart card task",                            /* task name for kernel awareness debugging */
                    2000L / sizeof(portSTACK_TYPE),               /* task stack size */
                    NULL,                                         /* optional task startup argument */
                    6U,                                           /* initial priority */
                    &g_UsbDeviceCcidSmartCard.smartCardTaskHandle /* optional task handle to create */
                    ) != pdPASS)
    {
        usb_echo("emvl1 smart card task create failed!\r\n");
        return;
    }

    while (1U)
    {
        vTaskDelay(MSEC_TO_TICK(1U));
    }
}

#if defined(__CC_ARM) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
    BOARD_InitHardware();

    if (xTaskCreate(APP_task,                                       /* pointer to the task */
                    "app task",                                     /* task name for kernel awareness debugging */
                    5000L / sizeof(portSTACK_TYPE),                 /* task stack size */
                    &g_UsbDeviceCcidSmartCard,                      /* optional task startup argument */
                    4U,                                             /* initial priority */
                    &g_UsbDeviceCcidSmartCard.applicationTaskHandle /* optional task handle to create */
                    ) != pdPASS)
    {
        usb_echo("app task create failed!\r\n");
#if (defined(__CC_ARM) || defined(__GNUC__))
        return 1U;
#else
        return;
#endif
    }

    vTaskStartScheduler();

#if (defined(__CC_ARM) || defined(__GNUC__))
    return 1U;
#endif
}
