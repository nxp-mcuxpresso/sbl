/*
 * The Clear BSD License
 * Copyright 2015 - 2017 NXP
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

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_timer.h"
#include "usb_pd_interface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PD_MSG_HEADER_MASK_FOR_SOP (0xFFFFFFFFu)
#define PD_MSG_HEADER_MASK_FOR_SOPP_CABLE (0xFFDFFFFFu)
#define PD_MSG_HEADER_MASK_FOR_SOPP_NON_CABLE (0xFEDFFFFFu)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

uint8_t PD_ConnectCheck(pd_instance_t *pdInstance);
void PD_PortTaskEventProcess(pd_instance_t *pdInstance, uint32_t eventSet);
pd_status_t PD_PhyControl(pd_instance_t *pdInstance, uint32_t control, void *param);
void PD_MsgReceive(pd_instance_t *pdInstance);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void PD_MsgInit(pd_instance_t *pdInstance)
{
    pdInstance->receiveState = 0;
    pdInstance->receivedData = NULL;
    pdInstance->hardResetReceived = 0;
    pdInstance->sendingMsgHeader.msgHeaderVal = 0;

    PD_PhyControl(pdInstance, PD_PHY_RESET_MSG_FUNCTION, NULL);
}

void PD_MsgReset(pd_instance_t *pdInstance)
{
    PD_PhyControl(pdInstance, PD_PHY_CANCEL_MSG_TX, NULL);
    pdInstance->sendingState = 0;
    PD_PhyControl(pdInstance, PD_PHY_CANCEL_MSG_RX, NULL);
    pdInstance->receiveState = 0;

    PD_MsgInit(pdInstance);
}

void PD_MsgDisable(pd_instance_t *pdInstance)
{
    PD_PhyControl(pdInstance, PD_PHY_DISABLE_MSG_TX, NULL);
    PD_PhyControl(pdInstance, PD_PHY_DISABLE_MSG_RX, NULL);
}

void PD_MsgSetPortRole(pd_instance_t *pdInstance, uint8_t powerRole, uint8_t dataRole)
{
    pd_phy_msg_header_info_t headerInfo;
    pdInstance->sendingMsgHeader.bitFields.portPowerRoleOrCablePlug = powerRole;
    pdInstance->sendingMsgHeader.bitFields.portDataRole = dataRole;
    pdInstance->sendingMsgHeader.bitFields.specRevision = pdInstance->revision;

    headerInfo.dataRole = dataRole;
    headerInfo.powerRole = powerRole;
    headerInfo.cablePlug = 0;
    headerInfo.revision = pdInstance->revision;
    PD_PhyControl(pdInstance, PD_PHY_SET_MSG_HEADER_INFO, &headerInfo);
}

void PD_MsgSendDone(pd_instance_t *pdInstance, pd_status_t result)
{
    if (pdInstance->sendingState == 1)
    {
        pdInstance->sendingState = 2;
        pdInstance->sendingResult = result;
        USB_OsaEventSet(pdInstance->taskEventHandle, PD_TASK_EVENT_SEND_DONE);
    }
}

uint8_t PD_MsgWaitSendResult(pd_instance_t *pdInstance)
{
    uint32_t eventSet;
    do
    {
        eventSet = 0u;
        USB_OsaEventWait(pdInstance->taskEventHandle, 0xffffu, 0, 5, &eventSet);
        PD_PortTaskEventProcess(pdInstance, eventSet);
        if (eventSet & PD_TASK_EVENT_SEND_DONE)
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_SEND_DONE);
            break;
        }

        /* Disconnected */
        if (PD_ConnectCheck(pdInstance) == (uint8_t)kConnectState_Disconnected)
        {
            PD_PhyControl(pdInstance, PD_PHY_CANCEL_MSG_TX, NULL);
            pdInstance->sendingState = 0;
            return kStatus_PD_Error;
        }
    } while (1);

    return ((pdInstance->sendingResult == kStatus_PD_Success) ? 1 : 0);
}

pd_status_t PD_MsgSend(
    pd_instance_t *pdInstance, start_of_packet_t sop, message_type_t msgType, uint32_t dataLength, uint8_t *dataBuffer)
{
    uint8_t index;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if ((pdInstance->hardResetReceived) || (pdInstance->sendingState == 1))
    {
        USB_OSA_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }

    pdInstance->sendingState = 1;
    USB_OSA_EXIT_CRITICAL();

    /* set msg header */
    pdInstance->sendingData[0] = (pdInstance->sendingMsgHeader.msgHeaderVal | (msgType & PD_MSG_TYPE_VALUE_MASK) |
                                  (((dataLength - 2) >> 2) << PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_POS))
                                 << 16;
    if ((sop != kPD_MsgSOP) && (pdInstance->pdConfig->deviceType != kDeviceType_Cable))
    {
        pdInstance->sendingData[0] &= PD_MSG_HEADER_MASK_FOR_SOPP_NON_CABLE;
    }
    /* copy msg data */
    if (msgType != kPD_MsgSourceCapabilities)
    {
        for (index = 0; index < ((dataLength - 2 + 3) >> 2); ++index)
        {
            pdInstance->sendingData[index + 1] = USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS((dataBuffer + index * 4));
        }
    }

    PD_MsgReceive(pdInstance);
    if (pdInstance->phyInterface->pdPhySend(pdInstance->pdPhyHandle, sop,
                                            ((uint8_t *)&(pdInstance->sendingData[0])) + 2,
                                            (dataLength)) == kStatus_PD_Success)
    {
        return kStatus_PD_Success;
    }
    else
    {
        pdInstance->sendingState = 0;
    }

    return kStatus_PD_Error;
}

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
pd_status_t PD_MsgSendExtendedMsgPart(pd_instance_t *pdInstance,
                                      start_of_packet_t sop,
                                      message_type_t extMsgType,
                                      uint32_t dataLength,
                                      uint8_t *dataBuffer)
{
    uint16_t index;
    uint8_t *uint8Buffer;
    uint8_t objCount = 0;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if ((pdInstance->hardResetReceived) || (pdInstance->receiveState == 2) || (pdInstance->sendingState == 1))
    {
        USB_OSA_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }
    pdInstance->sendingState = 1;
    USB_OSA_EXIT_CRITICAL();

    if (((pd_extended_msg_header_t *)(&(pdInstance->sendingData[1])))->bitFields.chunked)
    {
        objCount = ((dataLength + 2 + 3) >> 2);
    }
    /* set msg header */
    pdInstance->sendingData[0] =
        (pdInstance->sendingMsgHeader.msgHeaderVal | (PD_MSG_HEADER_EXTENDED_MASK) |
         (extMsgType & PD_MSG_TYPE_VALUE_MASK) | (objCount << PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_POS))
        << 16;
    if ((sop != kPD_MsgSOP) && (pdInstance->pdConfig->deviceType != kDeviceType_Cable))
    {
        pdInstance->sendingData[0] &= PD_MSG_HEADER_MASK_FOR_SOPP_NON_CABLE;
    }
    /* copy msg data (there is 2-bytes ext header) */
    uint8Buffer = (uint8_t *)&(pdInstance->sendingData[1]);
    uint8Buffer += 2;
    for (index = 0; index < dataLength; ++index)
    {
        uint8Buffer[index] = dataBuffer[index];
    }

    PD_MsgReceive(pdInstance);
    if (pdInstance->phyInterface->pdPhySend(pdInstance->pdPhyHandle, sop,
                                            ((uint8_t *)&(pdInstance->sendingData[0])) + 2,
                                            (dataLength + 4)) == kStatus_PD_Success)
    {
        return kStatus_PD_Success;
    }
    else
    {
        pdInstance->receiveState = 0;
    }

    return kStatus_PD_Error;
}

pd_status_t PD_MsgSendExtendedMsg(pd_instance_t *pdInstance,
                                  start_of_packet_t sop,
                                  message_type_t extMsgType,
                                  uint32_t dataLength,
                                  uint8_t *dataBuffer)
{
    pd_extended_msg_header_t *extMsgHeader;

    /* set extended msg header */
    extMsgHeader = (pd_extended_msg_header_t *)(&(pdInstance->sendingData[1]));
    extMsgHeader->bitFields.chunked = (pdInstance->unchunkedFeature ? 0 : 1);
    if (pdInstance->unchunkedFeature)
    {
        extMsgHeader->bitFields.dataSize = dataLength;
        extMsgHeader->bitFields.chunkNumber = 0;
        extMsgHeader->bitFields.requestChunk = 0;
    }
    else
    {
        if (dataLength > 26)
        {
            dataLength = 26;
        }
        extMsgHeader->bitFields.dataSize = dataLength;
        extMsgHeader->bitFields.chunkNumber = 0;
        extMsgHeader->bitFields.requestChunk = 0;
    }

    return PD_MsgSendExtendedMsgPart(pdInstance, sop, extMsgType, dataLength, dataBuffer);
}

pd_status_t PD_MsgSendChunkedExtendedMsg(pd_instance_t *pdInstance,
                                         start_of_packet_t sop,
                                         message_type_t extMsgType,
                                         pd_extended_msg_header_t extHeader,
                                         uint32_t dataLength,
                                         uint8_t *dataBuffer)
{
    pd_extended_msg_header_t *extMsgHeader;

    /* set extended msg header */
    extMsgHeader = (pd_extended_msg_header_t *)(&(pdInstance->sendingData[1]));
    extMsgHeader->extendedMsgHeaderVal = extHeader.extendedMsgHeaderVal;

    return PD_MsgSendExtendedMsgPart(pdInstance, sop, extMsgType, dataLength, dataBuffer);
}

pd_status_t PD_MsgSendUnchunkedExtendedMsg(pd_instance_t *pdInstance,
                                           start_of_packet_t sop,
                                           message_type_t extMsgType,
                                           uint32_t dataLength,
                                           uint8_t *dataBuffer)
{
    pd_extended_msg_header_t *extMsgHeader;

    /* set extended msg header */
    extMsgHeader = (pd_extended_msg_header_t *)(&(pdInstance->sendingData[1]));
    extMsgHeader->bitFields.chunked = (pdInstance->unchunkedFeature ? 0 : 1);
    extMsgHeader->bitFields.dataSize = dataLength;
    extMsgHeader->bitFields.chunkNumber = 0;
    extMsgHeader->bitFields.requestChunk = 0;

    return PD_MsgSendExtendedMsgPart(pdInstance, sop, extMsgType, dataLength, dataBuffer);
}

pd_status_t PD_MsgSendRequestChunkMsg(pd_instance_t *pdInstance,
                                      start_of_packet_t sop,
                                      message_type_t extMsgType,
                                      pd_extended_msg_header_t extHeader)
{
    pd_extended_msg_header_t *extMsgHeader;

    /* set extended msg header */
    extMsgHeader = (pd_extended_msg_header_t *)(&(pdInstance->sendingData[1]));
    extMsgHeader->extendedMsgHeaderVal = extHeader.extendedMsgHeaderVal;

    return PD_MsgSendExtendedMsgPart(pdInstance, sop, extMsgType, 0, NULL);
}
#endif

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
pd_status_t PD_MsgSendStructuredVDMAndWait(pd_instance_t *pdInstance,
                                           start_of_packet_t sop,
                                           pd_structured_vdm_header_t reponseVdmHeader,
                                           uint8_t count,
                                           uint32_t *vdos)
{
    uint8_t index;
    uint8_t *ptr;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if ((pdInstance->hardResetReceived) || (pdInstance->sendingState == 1))
    {
        USB_OSA_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }

    pdInstance->sendingState = 1;
    USB_OSA_EXIT_CRITICAL();

    /* set msg header */
    *(((uint16_t *)&(pdInstance->sendingData[0])) + 1) =
        pdInstance->sendingMsgHeader.msgHeaderVal | (kPD_MsgVendorDefined & PD_MSG_TYPE_VALUE_MASK) |
        (uint16_t)((uint16_t)(count + 1) << PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_POS);
    if ((sop != kPD_MsgSOP) && (pdInstance->pdConfig->deviceType != kDeviceType_Cable))
    {
        pdInstance->sendingData[0] &= PD_MSG_HEADER_MASK_FOR_SOPP_NON_CABLE;
    }
    /* set structured vdm header */
    ptr = (uint8_t *)&(pdInstance->sendingData[1]);
    *ptr++ = reponseVdmHeader.structuredVdmHeaderVal;
    *ptr++ = (reponseVdmHeader.structuredVdmHeaderVal >> 8);
    *ptr++ = (reponseVdmHeader.structuredVdmHeaderVal >> 16);
    *ptr++ = (reponseVdmHeader.structuredVdmHeaderVal >> 24);
    /* copy msg data */
    for (index = 0; index < count; ++index)
    {
        pdInstance->sendingData[index + 2] = vdos[index];
    }

    PD_MsgReceive(pdInstance);
    if (pdInstance->phyInterface->pdPhySend(pdInstance->pdPhyHandle, sop,
                                            ((uint8_t *)&(pdInstance->sendingData[0])) + 2,
                                            ((count + 1) * 4 + 2)) != kStatus_PD_Success)
    {
        pdInstance->sendingState = 0;
        return kStatus_PD_Error;
    }
    if (!PD_MsgWaitSendResult(pdInstance))
    {
        pdInstance->sendingState = 0;
        return kStatus_PD_Error;
    }

    return kStatus_PD_Success;
}

pd_status_t PD_MsgSendUnstructuredVDM(pd_instance_t *pdInstance,
                                      start_of_packet_t sop,
                                      uint8_t *dataBuffer,
                                      uint32_t dataLength)
{
    if (PD_MsgSend(pdInstance, sop, kPD_MsgVendorDefined, dataLength, dataBuffer) == kStatus_PD_Success)
    {
        if (PD_MsgWaitSendResult(pdInstance))
        {
            return kStatus_PD_Success;
        }
    }
    pdInstance->sendingState = 0;
    return kStatus_PD_Error;
}
#endif

pd_status_t PD_MsgSendHardReset(pd_instance_t *pdInstance)
{
    return PD_PhyControl(pdInstance, PD_PHY_SEND_HARD_RESET, NULL);
}

void PD_MsgReceived(pd_instance_t *pdInstance, uint32_t msgLength, pd_status_t result)
{
    if (pdInstance->receiveState == 1)
    {
        pdInstance->receiveState = 2;
        pdInstance->receivedLength = msgLength;
        pdInstance->receiveResult = result;
        USB_OsaEventSet(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG);
    }
}

void PD_MsgStopReceive(pd_instance_t *pdInstance)
{
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    pdInstance->enableReceive = 0;
    if (pdInstance->receiveState == 1)
    {
        pdInstance->receiveState = 0;
        USB_OSA_EXIT_CRITICAL();
        PD_PhyControl(pdInstance, PD_PHY_CANCEL_MSG_RX, NULL);
    }
    else
    {
        USB_OSA_EXIT_CRITICAL();
    }
}

void PD_MsgStartReceive(pd_instance_t *pdInstance)
{
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    pdInstance->enableReceive = 1;
    USB_OSA_EXIT_CRITICAL();
    PD_MsgReceive(pdInstance);
}

void PD_MsgReceive(pd_instance_t *pdInstance)
{
    uint32_t receiveLength;

    if (!(pdInstance->enableReceive))
    {
        return;
    }

    if (pdInstance->receiveState == 0)
    {
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        receiveLength = 260;
#else
        receiveLength = 32;
#endif
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        if ((pdInstance->receivingDataBuffer == NULL) ||
            ((pdInstance->unchunkedFeature) &&
             (pdInstance->receivingDataBuffer = (uint8_t *)pdInstance->receivingChunkedData)))
#else
        if (pdInstance->receivingDataBuffer == NULL)
#endif
        {
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            if (pdInstance->unchunkedFeature)
            {
                pdInstance->receivingDataBuffer = (uint8_t *)pdInstance->receivingData;
            }
            else
            {
                pdInstance->receivingDataBuffer = (uint8_t *)pdInstance->receivingChunkedData;
            }
#else
            pdInstance->receivingDataBuffer = (uint8_t *)&(pdInstance->receivingData[0]);
#endif
        }
        pdInstance->receiveState = 1;
        if (pdInstance->phyInterface->pdPhyReceive(pdInstance->pdPhyHandle, pdInstance->pendingSOP,
                                                   pdInstance->receivingDataBuffer + 2,
                                                   receiveLength) != kStatus_PD_Success)
        {
            pdInstance->receiveState = 0;
            USB_OsaEventSet(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG);
        }
    }
}

uint8_t PD_MsgGetReceiveResult(pd_instance_t *pdInstance)
{
    uint8_t retVal = 0;
    uint32_t receiveLength;
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint32_t index;
    uint8_t *destBuffer;
    uint8_t *sourceBuffer;
    uint8_t needCopy = 0;
    pd_msg_header_t msgHeader;
    pd_extended_msg_header_t extHeader;
#endif

    if (pdInstance->receiveState == 2)
    {
        pdInstance->receiveState = 0;
        receiveLength = pdInstance->receivedLength;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        sourceBuffer = (uint8_t *)pdInstance->receivingDataBuffer;
        sourceBuffer += 2;
        msgHeader.msgHeaderVal = (uint16_t)(USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(sourceBuffer));
        if (msgHeader.bitFields.extended)
        {
            sourceBuffer += 2;
            extHeader.extendedMsgHeaderVal = (uint32_t)(USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(sourceBuffer));

            /* chunked ext msg need copy, because it may be larger than 28 bytes and are chunked */
            if ((void *)pdInstance->receivingDataBuffer == (void *)pdInstance->receivingChunkedData)
            {
                needCopy = 1;

                destBuffer = (uint8_t *)pdInstance->receivingData;
                destBuffer += 4;
                sourceBuffer = (uint8_t *)pdInstance->receivingDataBuffer;
                sourceBuffer += 4;

                destBuffer[0] = sourceBuffer[0]; /* ext header */
                destBuffer[1] = sourceBuffer[1];
                destBuffer += 2;
                sourceBuffer += 2; /* ext header */
            }
            else
            {
                needCopy = 0;
            }

            if ((!extHeader.bitFields.chunked))
            {
                /* unchunked ext msg */
                if (receiveLength > 260)
                {
                    receiveLength = 260;
                }
            }
            else
            {
                /* chunked ext msg */
                if (receiveLength > 32)
                {
                    receiveLength = 32;
                }

                if (needCopy)
                {
                    destBuffer += (extHeader.bitFields.chunkNumber * 26);
                }
            }
        }
        else
#endif
        {
            if (receiveLength > 32)
            {
                receiveLength = 32;
            }
        }

        if (pdInstance->receiveResult == kStatus_PD_Success)
        {
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            if (needCopy)
            {
                pdInstance->receivingData[0] = ((uint32_t *)pdInstance->receivingDataBuffer)[0]; /* msg header */
                receiveLength -= 4; /* reduce the normal header and ext header */
                for (index = 0; index < (receiveLength); ++index)
                {
                    destBuffer[index] = sourceBuffer[index];
                }
                pdInstance->receivedData = (uint32_t *)&(pdInstance->receivingData[0]);
            }
            else
#endif
            {
                pdInstance->receivedData = (uint32_t *)pdInstance->receivingDataBuffer;
            }
        }

        retVal = (pdInstance->receiveResult == kStatus_PD_Success ? 1 : 0);
    }

    return retVal;
}

uint8_t PD_MsgRecvPending(pd_instance_t *pdInstance)
{
    return (pdInstance->receiveState == 2);
}

void PD_MsgSrcStartCommand(pd_instance_t *pdInstance)
{
#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    if (pdInstance->commandSrcOwner)
    {
        if (!PD_TimerCheckInvalidOrTimeOut(pdInstance, tSinkTxTimer))
        {
            while (!PD_TimerCheckInvalidOrTimeOut(pdInstance, tSinkTxTimer))
            {
            }
        }
    }
    else
    {
        uint8_t rpVal = kCurrent_1A5;
        pdInstance->commandSrcOwner = 1;
        PD_PhyControl(pdInstance, PD_PHY_SRC_SET_TYPEC_CURRENT_CAP, &rpVal);
        PD_TimerStart(pdInstance, tSinkTxTimer, T_SINK_TX);
        while (!PD_TimerCheckInvalidOrTimeOut(pdInstance, tSinkTxTimer))
        {
        }
    }
#endif
}

void PD_MsgSrcEndCommand(pd_instance_t *pdInstance)
{
#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    uint8_t rpVal = kCurrent_3A;
    pdInstance->commandSrcOwner = 0;
    PD_PhyControl(pdInstance, PD_PHY_SRC_SET_TYPEC_CURRENT_CAP, &rpVal);
#endif
}

uint8_t PD_MsgSnkCheckStartCommand(pd_instance_t *pdInstance)
{
#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    uint8_t typecCurrent;

    PD_PhyControl(pdInstance, PD_PHY_SNK_GET_TYPEC_CURRENT_CAP, &typecCurrent);
    return (typecCurrent == kCurrent_3A);
#else
    return 1;
#endif
}
