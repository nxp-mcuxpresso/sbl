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

static usb_status_t USB_DeviceCcidInterruptIn(usb_device_handle deviceHandle,
                                              usb_device_endpoint_callback_message_struct_t *event,
                                              void *callbackParam);
static usb_status_t USB_DeviceCcidBulkIn(usb_device_handle deviceHandle,
                                         usb_device_endpoint_callback_message_struct_t *event,
                                         void *callbackParam);
static usb_status_t USB_DeviceCcidSmartCardCommandParse(uint8_t *buffer, uint32_t length);
static usb_status_t USB_DeviceCcidBulkOut(usb_device_handle deviceHandle,
                                          usb_device_endpoint_callback_message_struct_t *event,
                                          void *callbackParam);
static void EMVL1_CcidCallback(uint8_t event, void *buffer, uint32_t size);
static void USB_DeviceApplicationInit(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

usb_ccid_smart_card_struct_t g_UsbDeviceCcidSmartCard;

extern uint8_t g_UsbDeviceCurrentConfigure;
extern uint8_t g_UsbDeviceInterface[USB_DEVICE_CCID_SMART_CARD_INTERFACE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

/* USB device CCID interrupt IN pipe callback */
static usb_status_t USB_DeviceCcidInterruptIn(usb_device_handle deviceHandle,
                                              usb_device_endpoint_callback_message_struct_t *event,
                                              void *callbackParam)
{
    usb_device_ccid_notify_slot_chnage_notification_t *ccidNotify;

    if (((NULL == deviceHandle) || (NULL == event)) || (NULL == event->buffer) ||
        (USB_UNINITIALIZED_VAL_32 == event->length))
    {
        return kStatus_USB_Error;
    }

    ccidNotify = (usb_device_ccid_notify_slot_chnage_notification_t *)event->buffer;

    if (USB_DEVICE_CCID_RDR_TO_PC_NOTIFYSLOTCHANGE == ccidNotify->bMessageType)
    {
        ccidNotify->bmSlotICCState[0] &= ~0x02U;
        if (!(ccidNotify->bmSlotICCState[0] & (0x01U << (USB_DEVICE_CCID_SMART_CARD_SLOT_INDEX << 1U))))
        {
            EMVL1_SmartCardPowerOff();
        }
    }
    return kStatus_USB_Success;
}

/* USB device ciid bulk IN pipe callback */
static usb_status_t USB_DeviceCcidBulkIn(usb_device_handle deviceHandle,
                                         usb_device_endpoint_callback_message_struct_t *event,
                                         void *callbackParam)
{
    return kStatus_USB_Error;
}

/* Parse the ccid command */
static usb_status_t USB_DeviceCcidSmartCardCommandParse(uint8_t *buffer, uint32_t length)
{
    usb_device_ccid_transfer_block_command_t *usbDeviceCcidCommand =
        (usb_device_ccid_transfer_block_command_t *)(buffer);
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

    if (temp32 > (length - USB_DEVICE_CCID_COMMAND_HEADER_LENGTH))
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
    return USB_DeviceSendRequest(
        g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN, (uint8_t *)ccidResponse,
        USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH + USB_LONG_FROM_LITTLE_ENDIAN_DATA(ccidResponse->dwLength));
}

/* USB device ccid bulk OUT pipe callback */
static usb_status_t USB_DeviceCcidBulkOut(usb_device_handle deviceHandle,
                                          usb_device_endpoint_callback_message_struct_t *event,
                                          void *callbackParam)
{
    usb_device_ccid_set_parameters_command_common_t *usbDeviceCcidSetParamCommand;
    usb_device_ccid_parameters_response_common_t *usbDeviceCcidParamResponse;
    usb_device_ccid_common_command_t *commonRequest;
    usb_device_ccid_slot_status_response_t *usbDeviceCcidSlotStatusResponse;
    usb_device_ccid_data_block_response_t *usbDeviceCcidDatablockResponse;
    emv_card_data_information_struct_t usbDeviceCcidCardInformation;
    uint32_t length;
    usb_status_t usbError = kStatus_USB_InvalidRequest;
    uint8_t error = kStatus_CCID_EMV_Success;

    if (((NULL == deviceHandle) || (NULL == event)) || (NULL == event->buffer) ||
        (USB_UNINITIALIZED_VAL_32 == event->length))
    {
        return kStatus_USB_Error;
    }

    commonRequest = (usb_device_ccid_common_command_t *)event->buffer;

    /* Check the slot is valid or not */
    if (USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS > commonRequest->bSlot)
    {
        for (int i = 0U; i < event->length; i++)
        {
            g_UsbDeviceCcidSmartCard.slotsCommandBuffer[commonRequest->bSlot][i] = event->buffer[i];
        }
        commonRequest =
            (usb_device_ccid_common_command_t *)&g_UsbDeviceCcidSmartCard.slotsCommandBuffer[commonRequest->bSlot][0];
    }

    /* If the slot is invalid, send the slot invalid response to the host */
    if (USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS <= commonRequest->bSlot)
    {
        usbDeviceCcidSlotStatusResponse = (usb_device_ccid_slot_status_response_t *)&g_UsbDeviceCcidSmartCard
                                              .slotsResponseBuffer[USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS][0];
        usbDeviceCcidSlotStatusResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_SLOTSTATUS;
        USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, usbDeviceCcidSlotStatusResponse->dwLength);
        usbDeviceCcidSlotStatusResponse->bSlot = commonRequest->bSlot;
        usbDeviceCcidSlotStatusResponse->bSeq = commonRequest->bSeq;
        usbDeviceCcidSlotStatusResponse->bStatus =
            USB_DEVICE_CCID_SLOT_STATUS_ICC_NOT_PRESENT | USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED;
        usbDeviceCcidSlotStatusResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_SLOT_NOT_EXIST;
        usbDeviceCcidSlotStatusResponse->bClockStatus = g_UsbDeviceCcidSmartCard.clockStatus;
        USB_DeviceRecvRequest(g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_OUT,
                              g_UsbDeviceCcidSmartCard.slotsCommandBuffer[USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS],
                              USB_DEVICE_CCID_COMMAND_BUFFER_LENGTH);
        return USB_DeviceSendRequest(g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                                     (uint8_t *)usbDeviceCcidSlotStatusResponse,
                                     USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH);
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
            usbError = USB_DeviceSendRequest(
                g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                (uint8_t *)usbDeviceCcidDatablockResponse,
                USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH +
                    USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidDatablockResponse->dwLength));
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_ICCPOWEROFF:
            /* ICC pwer off command */
            usbDeviceCcidSlotStatusResponse = (usb_device_ccid_slot_status_response_t *)&g_UsbDeviceCcidSmartCard
                                                  .slotsResponseBuffer[commonRequest->bSlot][0];
            error = EMVL1_SmartCardPowerOff();
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
            usbError = USB_DeviceSendRequest(
                g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                (uint8_t *)usbDeviceCcidSlotStatusResponse, USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH);
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
            usbError = USB_DeviceSendRequest(
                g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                (uint8_t *)usbDeviceCcidSlotStatusResponse, USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_XFRBLOCK:
            /* APDU message received */
            usb_echo("\r\nNew command received!\r\n");
            usbError = USB_DeviceCcidSmartCardCommandParse(event->buffer, event->length);
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
            usbError = USB_DeviceSendRequest(
                g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                (uint8_t *)usbDeviceCcidParamResponse,
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidParamResponse->common.dwLength) +
                    USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH);
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
            usbError = USB_DeviceSendRequest(
                g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                (uint8_t *)usbDeviceCcidParamResponse,
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidParamResponse->common.dwLength) +
                    USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH);
            break;
        case USB_DEVICE_CCID_PC_TO_RDR_SETPARAMETERS:
            /* Set parameters command */
            usbDeviceCcidSetParamCommand = (usb_device_ccid_set_parameters_command_common_t *)event->buffer;
            usbDeviceCcidSlotStatusResponse = (usb_device_ccid_slot_status_response_t *)&g_UsbDeviceCcidSmartCard
                                                  .slotsResponseBuffer[commonRequest->bSlot][0];

            if (USB_LONG_FROM_LITTLE_ENDIAN_DATA(usbDeviceCcidSetParamCommand->common.dwLength) >
                (event->length - USB_DEVICE_CCID_COMMAND_HEADER_LENGTH))
            {
                usbDeviceCcidSlotStatusResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_BAD_LENGTH;
                usbDeviceCcidSlotStatusResponse->bStatus = (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) |
                                                           (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
            }
            else
            {
                usbDeviceCcidSlotStatusResponse->bError = 0U;
                usbDeviceCcidSlotStatusResponse->bStatus =
                    (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_PROCESSED_NO_ERROR) |
                    (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
            }
            usbDeviceCcidSlotStatusResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_SLOTSTATUS;
            usbDeviceCcidSlotStatusResponse->bSeq = commonRequest->bSeq;
            usbDeviceCcidSlotStatusResponse->bSlot = commonRequest->bSlot;
            usbDeviceCcidSlotStatusResponse->bClockStatus = g_UsbDeviceCcidSmartCard.clockStatus;
            USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, usbDeviceCcidSlotStatusResponse->dwLength);

            usbError = USB_DeviceSendRequest(
                g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                (uint8_t *)usbDeviceCcidSlotStatusResponse, USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH);
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
            usbError = USB_DeviceSendRequest(
                g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                (uint8_t *)usbDeviceCcidSlotStatusResponse, USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH);
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

    if (kStatus_USB_InvalidRequest == usbError)
    {
        usbDeviceCcidSlotStatusResponse = (usb_device_ccid_slot_status_response_t *)&g_UsbDeviceCcidSmartCard
                                              .slotsResponseBuffer[commonRequest->bSlot][0];
        usbDeviceCcidSlotStatusResponse->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_SLOTSTATUS;
        usbDeviceCcidSlotStatusResponse->bSeq = commonRequest->bSeq;
        usbDeviceCcidSlotStatusResponse->bSlot = commonRequest->bSlot;
        usbDeviceCcidSlotStatusResponse->bClockStatus = g_UsbDeviceCcidSmartCard.clockStatus;
        usbDeviceCcidSlotStatusResponse->bError = USB_DEVICE_CCID_SLOT_ERROR_COMMAND_NOT_SUPPORTED;
        usbDeviceCcidSlotStatusResponse->bStatus =
            (USB_DEVICE_CCID_SLOT_STATUS_COMMAND_STATUS_FAILED) | (USB_DEVICE_CCID_SLOT_STATUS_ICC_PRESENT_ACTIVE);
        USB_LONG_TO_LITTLE_ENDIAN_DATA(0U, usbDeviceCcidSlotStatusResponse->dwLength);
        usbError =
            USB_DeviceSendRequest(g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN,
                                  (uint8_t *)usbDeviceCcidSlotStatusResponse, USB_DEVICE_CCID_RESPONSE_HEADER_LENGTH);
    }

    USB_DeviceRecvRequest(g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_OUT,
                          g_UsbDeviceCcidSmartCard.slotsCommandBuffer[USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS],
                          USB_DEVICE_CCID_COMMAND_BUFFER_LENGTH);
    return usbError;
}

/* Device callback */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Success;
    uint8_t *temp8 = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            /* USB device bus reset signal detected */
            g_UsbDeviceCcidSmartCard.attach = 0U;
            g_UsbDeviceCcidSmartCard.remoteWakeup = 0U;
            g_UsbDeviceCcidSmartCard.suspend = 0U;
            USB_DeviceControlPipeInit(g_UsbDeviceCcidSmartCard.deviceHandle);
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
        case kUSB_DeviceEventSetConfiguration:
            if (USB_DEVICE_CCID_SMART_CARD_CONFIGURE_INDEX == (*temp8))
            {
                /* Set the configuration */
                usb_device_endpoint_init_struct_t epInitStruct;
                usb_device_endpoint_callback_struct_t epCallback;
                uint8_t usbCurrentSr;

                epCallback.callbackFn = USB_DeviceCcidInterruptIn;
                epCallback.callbackParam = handle;

                epInitStruct.zlt = 0U;
                epInitStruct.transferType = USB_ENDPOINT_INTERRUPT;
                epInitStruct.endpointAddress = USB_DEVICE_CCID_SMART_CARD_ENDPOINT_INTERRUPT_IN |
                                               (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                if (USB_SPEED_HIGH == g_UsbDeviceCcidSmartCard.speed)
                {
                    epInitStruct.maxPacketSize = HS_INTERRUPT_IN_PACKET_SIZE;
                }
                else
                {
                    epInitStruct.maxPacketSize = FS_INTERRUPT_IN_PACKET_SIZE;
                }

                USB_DeviceInitEndpoint(g_UsbDeviceCcidSmartCard.deviceHandle, &epInitStruct, &epCallback);

                epCallback.callbackFn = USB_DeviceCcidBulkIn;
                epCallback.callbackParam = handle;

                epInitStruct.zlt = 1U;
                epInitStruct.transferType = USB_ENDPOINT_BULK;
                epInitStruct.endpointAddress = USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN |
                                               (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                if (USB_SPEED_HIGH == g_UsbDeviceCcidSmartCard.speed)
                {
                    epInitStruct.maxPacketSize = HS_BULK_IN_PACKET_SIZE;
                }
                else
                {
                    epInitStruct.maxPacketSize = FS_BULK_IN_PACKET_SIZE;
                }

                USB_DeviceInitEndpoint(g_UsbDeviceCcidSmartCard.deviceHandle, &epInitStruct, &epCallback);

                epCallback.callbackFn = USB_DeviceCcidBulkOut;
                epCallback.callbackParam = handle;

                epInitStruct.zlt = 0U;
                epInitStruct.transferType = USB_ENDPOINT_BULK;
                epInitStruct.endpointAddress = USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_OUT |
                                               (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                if (USB_SPEED_HIGH == g_UsbDeviceCcidSmartCard.speed)
                {
                    epInitStruct.maxPacketSize = HS_BULK_OUT_PACKET_SIZE;
                }
                else
                {
                    epInitStruct.maxPacketSize = FS_BULK_OUT_PACKET_SIZE;
                }

                USB_DeviceInitEndpoint(g_UsbDeviceCcidSmartCard.deviceHandle, &epInitStruct, &epCallback);

                USB_DeviceRecvRequest(g_UsbDeviceCcidSmartCard.deviceHandle,
                                      USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_OUT,
                                      g_UsbDeviceCcidSmartCard.slotsCommandBuffer[USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS],
                                      USB_DEVICE_CCID_COMMAND_BUFFER_LENGTH);
                usbCurrentSr = DisableGlobalIRQ();
                __ASM("CPSID I");
                g_UsbDeviceCcidSmartCard.attach = 1U;
                /* If the slot status changed and the slot is present, send the slot changed status to the host */
                if (g_UsbDeviceCcidSmartCard.slotsChanged)
                {
                    if ((g_UsbDeviceCcidSmartCard.suspend) && (g_UsbDeviceCcidSmartCard.remoteWakeup))
                    {
                        USB_DeviceSetStatus(g_UsbDeviceCcidSmartCard.deviceHandle, kUSB_DeviceStatusBus, NULL);
                    }
                    usb_device_ccid_notify_slot_chnage_notification_t *ccidNotify =
                        (usb_device_ccid_notify_slot_chnage_notification_t *)&g_UsbDeviceCcidSmartCard
                            .slotsChangeBuffer[0];
                    ccidNotify->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_NOTIFYSLOTCHANGE;
                    ccidNotify->bmSlotICCState[0] = 0x03U;
                    USB_DeviceSendRequest(
                        g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_INTERRUPT_IN,
                        g_UsbDeviceCcidSmartCard.slotsChangeBuffer, sizeof(g_UsbDeviceCcidSmartCard.slotsChangeBuffer));
                }
                EnableGlobalIRQ(usbCurrentSr);
            }
            break;
        default:
            break;
    }

    return error;
}

/* Get the setup buffer */
usb_status_t USB_DeviceGetSetupBuffer(usb_device_handle handle, usb_setup_struct_t **setupBuffer)
{
    static uint32_t setup[2];
    if (NULL == setupBuffer)
    {
        return kStatus_USB_InvalidParameter;
    }
    *setupBuffer = (usb_setup_struct_t *)&setup;
    return kStatus_USB_Success;
}

/* Configure the device remote wakeup(Enable or disable) */
usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable)
{
    g_UsbDeviceCcidSmartCard.remoteWakeup = enable;
    return kStatus_USB_Success;
}

/* Configure the endpoint status(Idle or stall) */
usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status)
{
    if (status)
    {
        return USB_DeviceStallEndpoint(handle, ep);
    }
    else
    {
        return USB_DeviceUnstallEndpoint(handle, ep);
    }
}

/* Get the class-specific request buffer */
usb_status_t USB_DeviceGetClassReceiveBuffer(usb_device_handle handle,
                                             usb_setup_struct_t *setup,
                                             uint32_t *length,
                                             uint8_t **buffer)
{
    return kStatus_USB_InvalidRequest;
}

/* Process the class-specific request */
usb_status_t USB_DeviceProcessClassRequest(usb_device_handle handle,
                                           usb_setup_struct_t *setup,
                                           uint32_t *length,
                                           uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    switch (setup->bRequest)
    {
        case USB_DEVICE_CCID_ABORT:
            break;
        case USB_DEVICE_CCID_GET_CLOCK_FREQUENCIES:
            *buffer = (uint8_t *)&g_UsbDeviceCcidSmartCard.clockFrequency;
            *length = sizeof(g_UsbDeviceCcidSmartCard.clockFrequency);
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_CCID_GET_DATA_RATES:
            *buffer = (uint8_t *)&g_UsbDeviceCcidSmartCard.dataRate;
            *length = sizeof(g_UsbDeviceCcidSmartCard.dataRate);
            error = kStatus_USB_Success;
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
            if (g_UsbDeviceCcidSmartCard.attach)
            {
                if ((g_UsbDeviceCcidSmartCard.suspend) && (g_UsbDeviceCcidSmartCard.remoteWakeup))
                {
                    USB_DeviceSetStatus(g_UsbDeviceCcidSmartCard.deviceHandle, kUSB_DeviceStatusBus, NULL);
                }
                usb_device_ccid_notify_slot_chnage_notification_t *ccidNotify =
                    (usb_device_ccid_notify_slot_chnage_notification_t *)&g_UsbDeviceCcidSmartCard.slotsChangeBuffer[0];
                ccidNotify->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_NOTIFYSLOTCHANGE;
                ccidNotify->bmSlotICCState[0] = 0x03U;
                USB_DeviceSendRequest(
                    g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_INTERRUPT_IN,
                    g_UsbDeviceCcidSmartCard.slotsChangeBuffer, sizeof(g_UsbDeviceCcidSmartCard.slotsChangeBuffer));
            }
            g_UsbDeviceCcidSmartCard.slotsChanged = 1U;
            usb_echo("CCID card is inserted!\r\n");
            break;
        case kEmvSmartCardEventCardRemoved:
            g_UsbDeviceCcidSmartCard.clockStatus = USB_DEVICE_CCID_CLCOK_STATUS_CLOCK_STOPPED_UNKNOWN;
            if (g_UsbDeviceCcidSmartCard.attach)
            {
                if ((g_UsbDeviceCcidSmartCard.suspend) && (g_UsbDeviceCcidSmartCard.remoteWakeup))
                {
                    USB_DeviceSetStatus(g_UsbDeviceCcidSmartCard.deviceHandle, kUSB_DeviceStatusBus, NULL);
                }
                usb_device_ccid_notify_slot_chnage_notification_t *ccidNotify =
                    (usb_device_ccid_notify_slot_chnage_notification_t *)&g_UsbDeviceCcidSmartCard.slotsChangeBuffer[0];
                ccidNotify->bMessageType = USB_DEVICE_CCID_RDR_TO_PC_NOTIFYSLOTCHANGE;
                ccidNotify->bmSlotICCState[0] = 0x02U;
                USB_DeviceSendRequest(
                    g_UsbDeviceCcidSmartCard.deviceHandle, USB_DEVICE_CCID_SMART_CARD_ENDPOINT_INTERRUPT_IN,
                    g_UsbDeviceCcidSmartCard.slotsChangeBuffer, sizeof(g_UsbDeviceCcidSmartCard.slotsChangeBuffer));
            }
            g_UsbDeviceCcidSmartCard.slotsChanged = 0U;
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
        USB_DeviceInit(CONTROLLER_ID, USB_DeviceCallback, &g_UsbDeviceCcidSmartCard.deviceHandle))
    {
        usb_echo("USB device CCID smart card failed\r\n");
        return;
    }
    else
    {
        usb_echo("USB device CCID smart card demo\r\n");
    }

    USB_DeviceIsrEnable();

    g_UsbDeviceCcidSmartCard.clockFrequency = USB_DEVICE_CCID_SMART_CARD_DEFAULT_CLOCK;
    g_UsbDeviceCcidSmartCard.dataRate = USB_DEVICE_CCID_SMART_CARD_DATA_RATE;
    g_UsbDeviceCcidSmartCard.clockStatus = USB_DEVICE_CCID_CLCOK_STATUS_CLOCK_STOPPED_UNKNOWN;
    g_UsbDeviceCcidSmartCard.protocol = USB_DEVICE_CCID_PROTOCOL_NUMBER_T0;

    for (int i = 0U; i < sizeof(g_UsbDeviceCcidSmartCard.slotsChangeBuffer); i++)
    {
        g_UsbDeviceCcidSmartCard.slotsChangeBuffer[i] = 0x00U;
    }
    g_UsbDeviceCcidSmartCard.slotsChanged = 0U;

    EMVL1_SmartCardInit(EMVL1_CcidCallback);

    USB_DeviceRun(g_UsbDeviceCcidSmartCard.deviceHandle);
}

#if defined(__CC_ARM) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
    BOARD_InitHardware();

    USB_DeviceApplicationInit();

    while (1U)
    {
#if USB_DEVICE_CONFIG_USE_TASK
        USB_DeviceTaskFn(g_UsbDeviceCcidSmartCard.deviceHandle);
#endif
    }
}
