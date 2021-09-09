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

#include <string.h>
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_timer.h"
#include "usb_pd_interface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define N_CAPS_COUNT (50)
#define N_HARD_RESET_COUNT (2)
#define PD_EXTENDED_SRC_CAP_DATA_LENGTH (23)
#define PD_EXTENDED_STATUS_MSG_DATA_LENGTH (3)
#define PD_EXTENDED_BATTERY_CAP_MSG_DATA_LENGTH (9)

#define MSG_DATA_BUFFER ((uint32_t *)(&(pdInstance->receivedData[1])))
#define MSG_DATA_HEADER (pdInstance->receivedData[0] >> 16)

#define PD_NOT_SUPPORT_REPLY_MSG ((pdInstance->revision >= PD_SPEC_REVISION_30) ? kPD_MsgNotSupported : kPD_MsgReject)

#define VDM_ID_HEADER_VDO_UFP_CABLE_PLUG_TYPE_MASK (0x38000000u)
#define VDM_ID_HEADER_VDO_PASSIVE_CABLE_VAL (0x18000000u)
#define VDM_ID_HEADER_VDO_ACTIVE_CABLE_VAL (0x20000000u)
#define VDM_CABLE_VDO_CURRENT_CAPABILITY (0x00000060u)

typedef enum _trigger_event
{
    PSM_TRIGGER_NONE,
    PSM_TRIGGER_NON_START,
    PSM_TRIGGER_DPM_MSG,
    PSM_TRIGGER_PD_MSG,
    PSM_TRIGGER_RECEIVE_HARD_RESET,
} trigger_event_t;

typedef struct _psm_trigger_info
{
    uint8_t triggerEvent;
    uint8_t pdMsgSop;
    uint8_t pdMsgType;
    uint8_t vdmMsgType;
    uint8_t *pdMsgDataBuffer;
    uint32_t pdMsgDataLength;
    uint32_t pdExtMsgLength;
    pd_structured_vdm_header_t vdmHeader;
    pd_msg_header_t msgHeader;

    uint8_t dpmMsg;
} psm_trigger_info_t;

typedef enum _pd_state_machine_state
{
    kSM_None = 0,
    kSM_Continue,
    kSM_WaitEvent,
    kSM_ErrorRecovery,
    kSM_Detach,
} pd_state_machine_state_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static uint8_t PD_PsmStartCommand(pd_instance_t *pdInstance, uint8_t command, uint8_t isInitiator);

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
static uint8_t Pd_PsmSecondaryStateHandler(pd_instance_t *pdInstance,
                                           uint8_t statIndex,
                                           uint8_t sop,
                                           psm_trigger_info_t *triggerInfo);
static void PD_PsmSecondaryStateHandlerTerminate(pd_instance_t *pdInstance, uint8_t sop);
#endif

void PD_MsgReset(pd_instance_t *pdInstance);
void PD_MsgDisable(pd_instance_t *pdInstance);
void PD_MsgSetPortRole(pd_instance_t *pdInstance, uint8_t powerRole, uint8_t dataRole);

uint8_t PD_MsgWaitSendResult(pd_instance_t *pdInstance);

pd_status_t PD_MsgSendStructuredVDMAndWait(pd_instance_t *pdInstance,
                                           start_of_packet_t sop,
                                           pd_structured_vdm_header_t reponseVdmHeader,
                                           uint8_t count,
                                           uint32_t *vdos);

pd_status_t PD_MsgSendHardReset(pd_instance_t *pdInstance);

pd_status_t PD_MsgSend(
    pd_instance_t *pdInstance, start_of_packet_t sop, message_type_t msgType, uint32_t dataLength, uint8_t *dataBuffer);

void PD_MsgReceive(pd_instance_t *pdInstance);

void PD_MsgStopReceive(pd_instance_t *pdInstance);

void PD_MsgStartReceive(pd_instance_t *pdInstance);

uint8_t PD_MsgGetReceiveResult(pd_instance_t *pdInstance);

uint8_t PD_MsgRecvPending(pd_instance_t *pdInstance);

void PD_DpmSrcTransitionToNewpwr(pd_instance_t *pdInstance, pd_rdo_t rdo, uint8_t gotoMin);

/* internal function */
uint8_t PD_DpmCheckVbus(pd_instance_t *pdInstance);

uint8_t PD_DpmCheckLessOrEqualVsafe5v(pd_instance_t *pdInstance);

uint8_t PD_DpmCheckVsafe0V(pd_instance_t *pdInstance);

uint8_t PD_DpmGetMsg(pd_instance_t *pdInstance);

void PD_DpmClearMsg(pd_instance_t *pdInstance, pd_command_t id);

void PD_DpmSendMsg(pd_handle pdHandle, uint8_t id);

void PD_ConnectSetPowerProgress(pd_instance_t *pdInstance, uint8_t state);

void PD_DpmDisconnect(pd_instance_t *pdInstance);

pd_status_t PD_MsgSendRequestChunkMsg(pd_instance_t *pdInstance,
                                      start_of_packet_t sop,
                                      message_type_t extMsgType,
                                      pd_extended_msg_header_t extHeader);

pd_status_t PD_MsgSendExtendedMsg(pd_instance_t *pdInstance,
                                  start_of_packet_t sop,
                                  message_type_t extMsgType,
                                  uint32_t dataLength,
                                  uint8_t *dataBuffer);

pd_status_t PD_MsgSendChunkedExtendedMsg(pd_instance_t *pdInstance,
                                         start_of_packet_t sop,
                                         message_type_t extMsgType,
                                         pd_extended_msg_header_t extHeader,
                                         uint32_t dataLength,
                                         uint8_t *dataBuffer);

pd_status_t PD_MsgSendUnchunkedExtendedMsg(pd_instance_t *pdInstance,
                                           start_of_packet_t sop,
                                           message_type_t extMsgType,
                                           uint32_t dataLength,
                                           uint8_t *dataBuffer);

pd_status_t PD_MsgSendUnstructuredVDM(pd_instance_t *pdInstance,
                                      start_of_packet_t sop,
                                      uint8_t *dataBuffer,
                                      uint32_t dataLength);

void PD_DpmDischargeVbus(pd_instance_t *pdInstance, uint8_t enable);
void PD_ConnectInitRole(pd_instance_t *pdInstance, uint8_t errorRecovery);
uint8_t PD_ConnectCheck(pd_instance_t *pdInstance);
TypeCState_t PD_ConnectGetStateMachine(pd_instance_t *pdInstance);
void PD_ConnectAltModeEnterFail(pd_instance_t *pdInstance, uint8_t pdConnected);
void PD_DpmSetVconn(pd_instance_t *pdInstance, uint8_t enable);
void PD_ConnectSetPRSwapRole(pd_instance_t *pdInstance, uint8_t powerRole);
void PD_MsgSrcEndCommand(pd_instance_t *pdInstance);

pd_status_t PD_DpmAppCallback(pd_instance_t *pdInstance, uint32_t event, void *param, uint8_t done);
pd_status_t PD_PhyControl(pd_instance_t *pdInstance, uint32_t control, void *param);

void PD_DpmDischargeVconn(pd_instance_t *pdInstance, uint8_t enable);

TypeCState_t PD_ConnectGetInitRoleState(pd_instance_t *pdInstance);

static void PD_PsmCommandFail(pd_instance_t *pdInstance, uint8_t command);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*! ***************************************************************************
   \brief Function to simplify the error handling of message transmit
   \note Only called from CLP task context
******************************************************************************/
static void PD_PsmTransitionOnMsgSendError(pd_instance_t *pdInstance,
                                           uint8_t interruptedState,
                                           pd_psm_state_t errorState)
{
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
    if ((PD_MsgRecvPending(pdInstance)) && (pdInstance->receivedSop == kPD_MsgSOP) &&
        (((MSG_DATA_HEADER & PD_MSG_HEADER_MESSAGE_TYPE_MASK) >> PD_MSG_HEADER_MESSAGE_TYPE_POS) ==
         (kPD_MsgVendorDefined & PD_MSG_TYPE_VALUE_MASK)))
    {
        /* re-excute the psmCurState */
        if (pdInstance->psmCurState != PSM_INTERRUPTED_REQUEST)
        {
            pdInstance->psmInterruptedState = pdInstance->psmCurState;
            pdInstance->psmCurState = PSM_INTERRUPTED_REQUEST;
        }
        pdInstance->psmNewState = PSM_INTERRUPTED_REQUEST;
    }
#endif
    if (PD_MsgRecvPending(pdInstance))
    {
        pdInstance->psmNewState = (pd_psm_state_t)interruptedState;
    }
    pdInstance->psmNewState = errorState;
    return;
}

static uint8_t PD_MsgSendControlTransition(pd_instance_t *pdInstance,
                                           uint8_t msgType,
                                           pd_psm_state_t successNextState,
                                           uint8_t interruptedState,
                                           pd_psm_state_t errorState)
{
    if ((PD_MsgSend(pdInstance, kPD_MsgSOP, (message_type_t)msgType, 2, NULL) == kStatus_PD_Success) &&
        (PD_MsgWaitSendResult(pdInstance)))
    {
        if (successNextState != PE_PSM_STATE_NO_CHANGE)
        {
            pdInstance->psmNewState = successNextState;
        }
        return 1; /* success */
    }

    PD_PsmTransitionOnMsgSendError(pdInstance, interruptedState, errorState);

    return 0; /* fail */
}

static inline void PD_PsmTransitionOnMsgSendControl(pd_instance_t *pdInstance,
                                                    uint8_t msgType,
                                                    pd_psm_state_t successNextState)
{
    PD_MsgSendControlTransition(pdInstance, msgType, successNextState, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
}

static uint8_t PD_MsgSendDataTransition(pd_instance_t *pdInstance,
                                        uint8_t msgType,
                                        uint32_t doCount,
                                        uint32_t *dos,
                                        pd_psm_state_t successNextState,
                                        uint8_t interruptedState,
                                        pd_psm_state_t errorState)
{
    if ((PD_MsgSend(pdInstance, kPD_MsgSOP, (message_type_t)msgType, 2 + (doCount * 4), (uint8_t *)dos) ==
         kStatus_PD_Success) &&
        (PD_MsgWaitSendResult(pdInstance)))
    {
        if (successNextState != PE_PSM_STATE_NO_CHANGE)
        {
            pdInstance->psmNewState = successNextState;
        }
        return 1; /* success */
    }

    PD_PsmTransitionOnMsgSendError(pdInstance, interruptedState, errorState);

    return 0; /* fail */
}

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)

static uint8_t PD_MsgSendExtTransition(pd_instance_t *pdInstance,
                                       start_of_packet_t sop,
                                       uint8_t msgType,
                                       uint32_t dataLength,
                                       uint8_t *dataBuffer,
                                       pd_psm_state_t successNextState,
                                       uint8_t interruptedState,
                                       pd_psm_state_t errorState)
{
    if (PD_MsgSendExtendedMsg(pdInstance, sop, (message_type_t)msgType, dataLength, dataBuffer) &&
        PD_MsgWaitSendResult(pdInstance))
    {
        if (successNextState != PE_PSM_STATE_NO_CHANGE)
        {
            pdInstance->psmNewState = successNextState;
        }
        return 1; /* success */
    }

    PD_PsmTransitionOnMsgSendError(pdInstance, interruptedState, errorState);

    return 0; /* fail */
}
#endif

static void PD_PsmSetNormalPower(pd_instance_t *pdInstance, uint8_t psmState)
{
    switch (psmState)
    {
        /* the follow state is stable state (may stay long time) */
        case PSM_PE_SRC_READY:
        case PSM_PE_SNK_READY:
        case PSM_PE_SNK_WAIT_FOR_CAPABILITIES:
        case PSM_PE_SRC_SEND_CAPABILITIES:
        case PSM_PE_SRC_DISABLED:
        case PSM_EXIT_TO_ERROR_RECOVERY:
        case PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY:
            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
            break;

        default:
            break;
    }
}

static void PD_PsmReset(pd_instance_t *pdInstance)
{
    uint8_t uint8Tmp;

    pdInstance->psmGotoMinTx = 0;
    pdInstance->psmGotoMinRx = 0;
    pdInstance->psmHardResetNeedsVSafe0V = 0;
    pdInstance->psmPresentlyPdConnected = 0;
    pdInstance->psmCablePlugResetNeeded = 0;
    pdInstance->psmCableDiscoveried = 0;
    pdInstance->commandEvaluateResult = kStatus_PD_Error;

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
    for (uint8Tmp = 0; uint8Tmp < PSM_SECONDARY_STATE_COUNT; uint8Tmp++)
    {
        pdInstance->psmSecondaryState[uint8Tmp] = PSM_IDLE;
        pdInstance->psmNewSecondaryState[uint8Tmp] = PSM_UNKNOWN;
    }
#endif

    uint8Tmp = 0;
    PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &uint8Tmp);
    PD_TimerCancelAllTimers(pdInstance, tSenderResponseTimer, _tMaxPSMTimer);
}

/* check messages that don't must require snkRdy or srcRdy */
static uint8_t PD_PsmPrimaryStateProcessDpmMsg(pd_instance_t *pdInstance,
                                               uint8_t *returnNewState,
                                               psm_trigger_info_t *triggerInfo)
{
    uint8_t newStateTmp = PSM_UNKNOWN;
    uint8_t didNothing = 1;

    /* assume the message is processed */
    triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
    switch (triggerInfo->dpmMsg)
    {
        case PD_DPM_CONTROL_HARD_RESET:
            newStateTmp = PSM_HARD_RESET;
            didNothing = 0;
            break;

        case PD_DPM_CONTROL_SOFT_RESET:
            if (PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_SOFT_RESET, 1))
            {
                newStateTmp = PSM_SEND_SOFT_RESET;
                didNothing = 0;
            }
            else
            {
                PD_PsmCommandFail(pdInstance, PD_DPM_CONTROL_SOFT_RESET);
            }
            break;
#if 0
        case PD_DPM_CONTROL_EXIT_MODE:
            pdInstance->vdmExitReceived[pdInstance->structuredVdmCommandParameter.vdmSop] = 1;
            break;
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        case PD_DPM_FAST_ROLE_SWAP:
            if (PD_PsmStartCommand(pdInstance, PD_DPM_FAST_ROLE_SWAP, 1))
            {
                /* need enter the PE_FRS_SRC_SNK_CC_Signal to wait the fr_swap msg */
                if (pdInstance->frSignaledWaitFrSwap)
                {
                    pdInstance->psmNewState = PE_FRS_SRC_SNK_CC_Signal;
                }
            }
            break;
#endif

        default:
            /* the event is not processed */
            triggerInfo->triggerEvent = PSM_TRIGGER_DPM_MSG;
            break;
    }

    if (newStateTmp != PSM_UNKNOWN)
    {
        *returnNewState = newStateTmp;
    }

    return didNothing;
}

static uint8_t PD_PsmPrimaryStateProcessPdMsg(pd_instance_t *pdInstance,
                                              uint8_t *returnNewState,
                                              psm_trigger_info_t *triggerInfo)
{
    uint8_t newStateTmp = PSM_UNKNOWN;
    uint8_t didNothing = 1;

    if (triggerInfo->triggerEvent == PSM_TRIGGER_RECEIVE_HARD_RESET)
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;

        if (pdInstance->psmCurState == PSM_PE_BIST_TEST_DATA_MODE)
        {
            /* From the IAS: There is no exit from this test mode except some chip level reset.
               Reset the protocol layer to exit carrier mode */
            PD_PhyControl(pdInstance, PD_PHY_EXIT_BIST, NULL);
        }
        /* Exit any active alternate mode */
        /* All timers and secondary states are reset during this call */
        PD_PsmReset(pdInstance);

        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
        {
            /* source receive hard_reset, 1. start tPSHardReset */
            PD_TimerStart(pdInstance, tPSHardResetTimer, T_PS_HARD_RESET);
            newStateTmp = PSM_PE_SRC_HARD_RESET_RECEIVED;
        }
        else if (pdInstance->curPowerRole == kPD_PowerRoleSink)
        {
            pdInstance->callbackFns->PD_SnkStopDrawVbus(pdInstance->callbackParam, kVbusPower_InHardReset);
            pdInstance->asmHardResetSnkProcessing = 1;
            newStateTmp = PSM_PE_SNK_TRANSITION_TO_DEFAULT;
        }
        else
        {
        }
        didNothing = 0;
    }
    else if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgType == kPD_MsgBIST))
    {
#if (defined PD_CONFIG_COMPLIANCE_TEST_ENABLE) && (PD_CONFIG_COMPLIANCE_TEST_ENABLE)
        pd_bist_object_t bistObj;
        bistObj.objVal = USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(triggerInfo->pdMsgDataBuffer);
        if (bistObj.bitFields.testMode == kBIST_TestData)
        {
            /* Do nothing, the HW does not have a mode for kBIST_TestData */
            newStateTmp = kBIST_TestData;
            PD_PhyControl(pdInstance, PD_PHY_ENTER_BIST, &newStateTmp);
            newStateTmp = PSM_PE_BIST_TEST_DATA_MODE;
        }
        else
        {
            PD_TimerClear(pdInstance, tNoResponseTimer);
            newStateTmp = PSM_PE_BIST_CARRIER_MODE_2;
        }
#endif
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
    }
    else if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgType == kPD_MsgSoftReset))
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        /* Soft reset is ignored when in BIST carrier mode */
        /* Soft reset is ignored during fast role swap signalling */
        if ((pdInstance->psmCurState != PSM_PE_BIST_CARRIER_MODE_2) &&
            (pdInstance->psmCurState != PE_FRS_SRC_SNK_CC_Signal))
        {
            PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_SOFT_RESET, 0);
            newStateTmp = PSM_SOFT_RESET;
            didNothing = 0;
        }
    }
    /* The received message's data role is not right */
    else if ((pdInstance->curDataRole == triggerInfo->msgHeader.bitFields.portDataRole) &&
             (triggerInfo->pdMsgType != kPD_MsgGoodCRC))
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NON_START;
        newStateTmp = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
        didNothing = 0;
    }
    /* received source_capabilities */
    else if ((pdInstance->psmCurState != PSM_PE_SNK_DISCOVERY) && (triggerInfo->pdMsgType == kPD_MsgSourceCapabilities))
    {
        uint32_t u32Tmp = 0;

        u32Tmp = u32Tmp;
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceOnly)
        {
            PD_PsmTransitionOnMsgSendControl(pdInstance, PD_NOT_SUPPORT_REPLY_MSG, pdInstance->psmCurState);
        }
        else
        {
            switch (pdInstance->psmCurState)
            {
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
                case PSM_PE_SRC_DISCOVERY:
                case PSM_PE_SRC_SEND_CAPABILITIES:
                case PSM_PE_SRC_TRANSITION_SUPPLY:
                case PSM_PE_SRC_READY:
                case PSM_PE_SRC_GET_SINK_CAP:
#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                    if ((pdInstance->curDataRole == triggerInfo->msgHeader.bitFields.portDataRole) &&
                        (triggerInfo->pdMsgType != kPD_MsgGoodCRC))
                    {
                        /* Conflicting data roles (with delay for good crc tx) */
                        newStateTmp = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
                    }
                    else
#endif
                    {
                        newStateTmp = PSM_SEND_SOFT_RESET;
                    }
                    break;

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF:
                case PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON:
                case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP:
                case PSM_PE_PRS_SNK_SRC_SOURCE_ON:
#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                    if ((pdInstance->curDataRole == triggerInfo->msgHeader.bitFields.portDataRole) &&
                        (triggerInfo->pdMsgType != kPD_MsgGoodCRC))
                    {
                        /* Conflicting data roles (with delay for good crc tx) */
                        newStateTmp = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
                    }
                    else
#endif
                    {
                        /* A protocol error during power role swap triggers a Hard Reset */
                        newStateTmp = PSM_HARD_RESET;
                    }
                    break;
#endif

#endif

/* Source state, message expected */
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                case PSM_PE_DR_SRC_GET_SOURCE_CAP:
                case PSM_PE_SNK_GET_SOURCE_CAP:
                {
                    pd_capabilities_t sourceCapa;
                    PD_TimerClear(pdInstance, tSenderResponseTimer);
                    /* Clear any pending message now that we have the latest */
                    (void)PD_DpmClearMsg(pdInstance, PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES);
                    sourceCapa.capabilities = MSG_DATA_BUFFER;
                    sourceCapa.capabilitiesCount = (triggerInfo->pdMsgDataLength);
                    if (pdInstance->commandProcessing == PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES)
                    {
                        pdInstance->commandProcessing = 0;
                    }
                    PD_DpmAppCallback(pdInstance, PD_DPM_GET_PARTNER_SRC_CAP_SUCCESS, &sourceCapa, 1);
                    if (pdInstance->psmCurState == PSM_PE_SNK_GET_SOURCE_CAP)
                    {
                        newStateTmp = PSM_PE_SNK_READY;
                    }
                    else
                    {
                        newStateTmp = PSM_PE_SRC_READY;
                    }
                    break;
                }
#endif

/* Sink states, message accepted: */
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
                case PSM_PE_SNK_WAIT_FOR_CAPABILITIES:
                case PSM_PE_SNK_SELECT_CAPABILITY:
                case PSM_PE_SNK_TRANSITION_SINK:
                case PSM_PE_SNK_READY:
                case PSM_PE_SNK_GIVE_SINK_CAP:
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF:
                case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP:
                case PSM_PE_DR_SNK_GET_SINK_CAP:
#endif
#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                    if ((pdInstance->curDataRole == triggerInfo->msgHeader.bitFields.portDataRole) &&
                        (triggerInfo->pdMsgType != kPD_MsgGoodCRC))
                    {
                        /* Conflicting data roles (with delay for good crc tx) */
                        newStateTmp = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
                    }
                    else
#endif
                    {
                        pd_capabilities_t sourceCapa;
                        if (pdInstance->revision > triggerInfo->msgHeader.bitFields.specRevision)
                        {
                            pdInstance->revision = triggerInfo->msgHeader.bitFields.specRevision;
                            PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                        }
                        PD_TimerClear(pdInstance, tSinkWaitCapTimer);
                        pdInstance->psmPresentlyPdConnected = 1;
                        pdInstance->psmPreviouslyPdConnected = 1;

                        /* Update the PDOs for the EC and send interrupt */
                        sourceCapa.capabilities = MSG_DATA_BUFFER;
                        sourceCapa.capabilitiesCount = (triggerInfo->pdMsgDataLength);
                        pdInstance->selfOrPartnerFirstSourcePDO.PDOValue = sourceCapa.capabilities[0];
                        if (pdInstance->commandProcessing == PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES)
                        {
                            pdInstance->commandProcessing = 0;
                        }
                        PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RECEIVE_PARTNER_SRC_CAP, &sourceCapa, 0);
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                        /* Ensure secondary state machine does not transition primary state machine */
                        PD_PsmSecondaryStateHandlerTerminate(pdInstance, 0xffu);
#endif
                        newStateTmp = PSM_PE_SNK_EVALUATE_CAPABILITY;
                    }
                    break;
#endif

                default:
                    break;
            }
        }

        didNothing = 0;
    }
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
    else if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgType == kPD_MsgVendorDefined) &&
             (triggerInfo->vdmHeader.bitFields.command == kVDM_ExitMode) &&
             (triggerInfo->vdmHeader.bitFields.commandType == kVDM_Initiator))
    {
        pdInstance->vdmExitReceived[triggerInfo->pdMsgSop] = triggerInfo->vdmHeader.structuredVdmHeaderVal;
        pdInstance->vdmExitReceivedSOP[triggerInfo->pdMsgSop] = triggerInfo->pdMsgSop;
    }
#endif
    else
    {
    }

    if (newStateTmp != PSM_UNKNOWN)
    {
        *returnNewState = newStateTmp;
    }

    return didNothing;
}

void PD_PortTaskEventProcess(pd_instance_t *pdInstance, uint32_t eventSet)
{
    if (eventSet & PD_TASK_EVENT_PHY_STATE_CHAGNE)
    {
        PD_PhyControl(pdInstance, PD_PHY_UPDATE_STATE, NULL);
    }

    if (eventSet & PD_TASK_EVENT_OTHER)
    {
        USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_OTHER);
    }

    if (pdInstance->isConnected)
    {
        eventSet &= (~(uint32_t)(PD_TASK_EVENT_PHY_STATE_CHAGNE | PD_TASK_EVENT_OTHER | PD_TASK_EVENT_FR_SWAP_SINGAL |
                                 PD_TASK_EVENT_DPM_MSG));
    }
    else
    {
        eventSet &= (~(uint32_t)(PD_TASK_EVENT_PHY_STATE_CHAGNE | PD_TASK_EVENT_OTHER | PD_TASK_EVENT_FR_SWAP_SINGAL));
    }

    /* clear the events that aren't processed in this condition */
    if (eventSet)
    {
        if (!((pdInstance->dpmStateMachine == 1) && (pdInstance->isConnected)))
        {
            USB_OsaEventClear(pdInstance->taskEventHandle, eventSet);
        }
    }
}

static uint8_t PD_PsmStartCommand(pd_instance_t *pdInstance, uint8_t command, uint8_t isInitiator)
{
#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    uint8_t start = 1;

    if (isInitiator)
    {
        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
        {
            PD_MsgSrcStartCommand(pdInstance);
        }
        else
        {
            start = PD_MsgSnkCheckStartCommand(pdInstance);
        }
    }
    else
    {
        start = 1;
    }

    if (start)
#endif
    {
        pdInstance->commandProcessing = command;
        pdInstance->commandIsInitiator = isInitiator;
    }

#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    return start;
#else
    return 1;
#endif
}

static void PD_PsmCommandFail(pd_instance_t *pdInstance, uint8_t command)
{
    uint32_t commandResultCallback = kCommandResult_Error;
    uint32_t event = 0xFFFFFFFFu;

    switch (command)
    {
        case PD_DPM_CONTROL_POWER_NEGOTIATION:
        {
            if (pdInstance->commandIsInitiator)
            {
                event = PD_DPM_SRC_RDO_FAIL;
            }
            else
            {
                event = PD_DPM_SNK_RDO_FAIL;
            }
            break;
        }
        case PD_DPM_CONTROL_REQUEST:
        {
            if (pdInstance->curPowerRole == kPD_PowerRoleSink)
            {
                event = PD_DPM_SNK_RDO_FAIL;
            }
            else
            {
                event = PD_DPM_SRC_RDO_FAIL;
            }
            break;
        }
        case PD_DPM_CONTROL_GOTO_MIN:
        {
            if (pdInstance->commandIsInitiator)
            {
                event = PD_DPM_SRC_GOTOMIN_FAIL;
            }
            else
            {
                event = PD_DPM_SNK_GOTOMIN_FAIL;
            }
            break;
        }
        case PD_DPM_CONTROL_SOFT_RESET:
        {
            event = PD_DPM_SOFT_RESET_FAIL;
            break;
        }
        case PD_DPM_CONTROL_HARD_RESET:
        {
            /* hard reset will always success */
            break;
        }
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
        case PD_DPM_CONTROL_PR_SWAP:
        {
            event = PD_DPM_PR_SWAP_FAIL;
            break;
        }
#endif
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
        case PD_DPM_CONTROL_DR_SWAP:
        {
            event = PD_DPM_DR_SWAP_FAIL;
            break;
        }
#endif
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
        case PD_DPM_CONTROL_VCONN_SWAP:
        {
            event = PD_DPM_VCONN_SWAP_FAIL;
            break;
        }
#endif
        case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
        {
            event = PD_DPM_GET_PARTNER_SRC_CAP_FAIL;
            break;
        }
        case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
        {
            event = PD_DPM_GET_PARTNER_SNK_CAP_FAIL;
            break;
        }
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
        case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
        case PD_DPM_CONTROL_DISCOVERY_SVIDS:
        case PD_DPM_CONTROL_DISCOVERY_MODES:
        case PD_DPM_CONTROL_ENTER_MODE:
        case PD_DPM_CONTROL_EXIT_MODE:
        case PD_DPM_CONTROL_SEND_ATTENTION:
        case PD_DPM_SEND_VENDOR_STRUCTURED_VDM:
        {
            event = PD_DPM_STRUCTURED_VDM_FAIL;
            break;
        }
        case PD_DPM_SEND_UNSTRUCTURED_VDM:
        {
            event = PD_DPM_SEND_UNSTRUCTURED_VDM_FAIL;
            break;
        }
#endif
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
        case PD_DPM_CONTROL_CABLE_RESET:
        {
            break;
        }
#endif
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        case PD_DPM_GET_SRC_EXT_CAP:
        {
            event = PD_DPM_GET_SRC_EXT_CAP_FAIL;
            break;
        }
        case PD_DPM_GET_STATUS:
        {
            event = PD_DPM_GET_STATUS_FAIL;
            break;
        }
        case PD_DPM_GET_BATTERY_CAP:
        {
            event = PD_DPM_GET_BATTERY_CAP_FAIL;
            break;
        }
        case PD_DPM_GET_BATTERY_STATUS:
        {
            event = PD_DPM_GET_BATTERY_STATUS_FAIL;
            break;
        }
        case PD_DPM_GET_MANUFACTURER_INFO:
        {
            event = PD_DPM_GET_MANUFACTURER_INFO_FAIL;
            break;
        }
#if 0
        case PD_DPM_SECURITY_REQUEST:
        {
            event = PD_DPM_SECURITY_REQUEST_FAIL;
            break;
        }
        case PD_DPM_FIRMWARE_UPDATE_REQUEST:
        {
            break;
        }
#endif
        case PD_DPM_ALERT:
        {
            event = PD_DPM_SEND_ALERT_FAIL;
            break;
        }
#endif
#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        case PD_DPM_FAST_ROLE_SWAP:
        {
            event = PD_DPM_FR_SWAP_FAIL;
            break;
        }
#endif

        default:
            break;
    }

    if (event != 0xFFFFFFFFu)
    {
        if (event != PD_DPM_STRUCTURED_VDM_FAIL)
        {
            PD_DpmAppCallback(pdInstance, event, &commandResultCallback, 1);
        }
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
        else
        {
            pd_svdm_command_result_t commandVdmResult;
            commandVdmResult.vdmCommandResult = kCommandResult_Error;
            commandVdmResult.vdmCommand = pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command;
            commandVdmResult.vdoData = NULL;
            commandVdmResult.vdoCount = 0;

            PD_DpmAppCallback(pdInstance, event, &commandVdmResult, 1);
        }
#endif
    }
}

#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)) || \
    (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE))
static void PD_PsmEndVdmCommand(pd_instance_t *pdInstance, uint8_t secondState)
{
    if (secondState != PSM_IDLE)
    {
        return;
    }

    if (pdInstance->commandProcessing == 0)
    {
        return;
    }
    else
    {
        PD_PsmCommandFail(pdInstance, pdInstance->commandProcessing);
    }
}
#endif

static void PD_PsmEndCommand(pd_instance_t *pdInstance, uint8_t psmState)
{
    if (pdInstance->commandProcessing == 0)
    {
        return;
    }
    else
    {
        uint8_t commandFail = 0;

        switch (pdInstance->commandProcessing)
        {
            case PD_DPM_CONTROL_HARD_RESET:
            case PD_DPM_CONTROL_SOFT_RESET:
            case PD_DPM_CONTROL_CABLE_RESET:
            {
                /* process in the state machine */
                break;
            }

#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PD_DPM_CONTROL_PR_SWAP:
            {
                switch (psmState)
                {
                    case PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP:
                    case PSM_PE_PRS_SRC_SNK_ACCEPT_PR_SWAP:
                    case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF:
                    case PSM_PE_PRS_SRC_SNK_ASSERT_RD:
                    case PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON:
                    case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP:
                    case PSM_PE_PRS_SRC_SNK_REJECT_PR_SWAP:
                    case PSM_PE_PRS_SNK_SRC_EVALUATE_PR_SWAP:
                    case PSM_PE_PRS_SNK_SRC_ACCEPT_PR_SWAP:
                    case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF:
                    case PSM_PE_PRS_SNK_SRC_ASSERT_RP:
                    case PSM_PE_PRS_SNK_SRC_SOURCE_ON:
                    case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP:
                    case PSM_PE_PRS_SNK_SRC_REJECT_PR_SWAP:
                    case PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT:
                        /* these states means the pr_swap ams is processing */
                        break;

                    default:
                        /* PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, NULL, 1); */
                        commandFail = 1;
                        break;
                }
                break;
            }
#endif

            case PD_DPM_CONTROL_REQUEST:
                switch (psmState)
                {
                    case PSM_PE_SRC_NEGOTIATE_CAPABILITY:
                    case PSM_PE_SRC_TRANSITION_SUPPLY:
                    case PSM_PE_SNK_EVALUATE_CAPABILITY:
                    case PSM_PE_SNK_TRANSITION_SINK:
                    case PSM_PE_SNK_SELECT_CAPABILITY:
                    case PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT:
                        break;

                    case PSM_PE_SNK_READY:
                        if (pdInstance->psmSnkReceiveRdoWaitRetry)
                        {
                            /* even in snk_rdy bu it is wait reply */
                        }
                        else
                        {
                            /* PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RDO_FAIL, NULL, 1); */
                            commandFail = 1;
                        }
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_CONTROL_POWER_NEGOTIATION:
                switch (psmState)
                {
                    case PSM_PE_SRC_DISCOVERY:
                    case PSM_PE_SRC_NEGOTIATE_CAPABILITY:
                    case PSM_PE_SRC_TRANSITION_SUPPLY:
                    case PSM_PE_SNK_TRANSITION_SINK:
                    case PSM_PE_SNK_SELECT_CAPABILITY:
                    case PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT:
                    case PSM_PE_SRC_SEND_CAPABILITIES:
                    case PSM_PE_SNK_WAIT_FOR_CAPABILITIES:
                    case PSM_PE_SNK_GIVE_SINK_CAP:
                    case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF:
                    case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP:
                    case PSM_PE_DR_SNK_GET_SINK_CAP:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
                switch (psmState)
                {
                    case PSM_PE_DR_SRC_GET_SOURCE_CAP:
                    case PSM_PE_SNK_GET_SOURCE_CAP:
                    case PSM_PE_SNK_READY:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
                switch (psmState)
                {
                    case PSM_PE_DR_SNK_GET_SINK_CAP:
                    case PSM_PE_SRC_GET_SINK_CAP:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_CONTROL_GOTO_MIN:
                switch (psmState)
                {
                    case PSM_PE_SNK_TRANSITION_SINK:
                    case PSM_PE_SRC_TRANSITION_SUPPLY:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
            case PD_DPM_CONTROL_DR_SWAP:
                switch (psmState)
                {
                    case PSM_PE_DRS_SEND_DR_SWAP:
                    case PSM_PE_DRS_EVALUATE_DR_SWAP:
                    case PSM_PE_DRS_ACCEPT_DR_SWAP:
                    case PSM_PE_DRS_REJECT_DR_SWAP:
                    case PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            case PD_DPM_CONTROL_VCONN_SWAP:
                switch (psmState)
                {
                    case PSM_PE_VCS_SEND_SWAP:
                    case PSM_PE_VCS_WAIT_FOR_VCONN:
                    case PSM_PE_VCS_TURN_OFF_VCONN:
                    case PSM_PE_VCS_TURN_ON_VCONN:
                    case PSM_PE_VCS_SEND_PS_RDY:
                    case PSM_PE_VCS_EVALUATE_SWAP:
                    case PSM_PE_VCS_ACCEPT_SWAP:
                    case PSM_PE_VCS_REJECT_SWAP:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

            case PD_DPM_GET_SRC_EXT_CAP:
                switch (psmState)
                {
                    case PE_SNK_GET_SOURCE_CAP_EXT:
                    case PE_DR_SRC_GET_SOURCE_CAP_EXT:
                    case PE_SRC_GIVE_SOURCE_CAP_EXT:
                    case PE_DR_SNK_GIVE_SOURCE_CAP_EXT:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PD_DPM_GET_STATUS:
                switch (psmState)
                {
                    case PE_SNK_Get_Source_Status:
                    case PE_SRC_Give_Source_Status:
                    case PE_SRC_Get_Sink_Status:
                    case PE_SNK_Give_Sink_Status:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_GET_BATTERY_CAP:
                switch (psmState)
                {
                    case PE_Give_Battery_Cap:
                    case PE_Get_Battery_Cap:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_GET_BATTERY_STATUS:
                switch (psmState)
                {
                    case PE_Get_Battery_Status:
                    case PE_Give_Battery_Status:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

            case PD_DPM_GET_MANUFACTURER_INFO:
                switch (psmState)
                {
                    case PE_Get_Manfacturer_Info:
                    case PE_Give_Manufacturer_Info:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;

#if 0
            case PD_DPM_SECURITY_REQUEST:
                switch (psmState)
                {
                    case PE_Send_Security_Request:
                    case PE_Send_Security_Response:
                    case PE_Security_Response_Received:
                    case PSM_PE_SNK_READY:
                    case PSM_PE_SRC_READY:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

            case PD_DPM_ALERT:
                switch (psmState)
                {
                    case PE_SRC_Send_Source_Alert:
                    case PE_SNK_Source_Alert_Received:
                    case PE_SNK_Send_Sink_Alert:
                    case PE_SRC_Sink_Alert_Received:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PD_DPM_FAST_ROLE_SWAP:
                switch (psmState)
                {
                    case PE_FRS_SRC_SNK_CC_Signal:
                    case PE_FRS_SRC_SNK_Evaluate_Swap:
                    case PE_FRS_SRC_SNK_Accept_Swap:
                    case PE_FRS_SRC_SNK_Transition_to_off:
                    case PE_FRS_SRC_SNK_Assert_Rd:
                    case PE_FRS_SRC_SNK_Wait_Source_on:
                    case PE_FRS_SNK_SRC_Send_Swap:
                    case PE_FRS_SNK_SRC_Transition_to_off:
                    case PE_FRS_SNK_SRC_Vbus_Applied:
                    case PE_FRS_SNK_SRC_Assert_Rp:
                    case PE_FRS_SNK_SRC_Source_on:
                        break;

                    default:
                        commandFail = 1;
                        break;
                }
                break;
#endif

            default:
            {
                break;
            }
        }

        if (commandFail)
        {
            PD_PsmCommandFail(pdInstance, pdInstance->commandProcessing);
        }
    }
}

static void PD_PsmSinkAndSourceRdyProcessDpmMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    if (triggerInfo->dpmMsg < PD_DPM_CONTROL_DISCOVERY_IDENTITY)
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        if (!PD_PsmStartCommand(pdInstance, triggerInfo->dpmMsg, 1))
        {
            PD_PsmCommandFail(pdInstance, triggerInfo->dpmMsg);
            return;
        }
        switch (triggerInfo->dpmMsg)
        {
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
            case PD_DPM_CONTROL_DR_SWAP:
            {
                /* TODO: check alternate mode */
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tDrSwapWaitTimer))
                {
                    pdInstance->psmNewState = PSM_PE_DRS_SEND_DR_SWAP;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_TIMER_TIME_OUT;
                }
                break;
            }
#endif
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            case PD_DPM_CONTROL_VCONN_SWAP:
            {
                pdInstance->psmNewState = PSM_PE_VCS_SEND_SWAP;
                break;
            }
#endif
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PD_DPM_GET_BATTERY_CAP:
            {
                pdInstance->psmNewState = PE_Get_Battery_Cap;
                break;
            }
            case PD_DPM_GET_BATTERY_STATUS:
            {
                pdInstance->psmNewState = PE_Get_Battery_Status;
                break;
            }
            case PD_DPM_GET_MANUFACTURER_INFO:
            {
                pdInstance->psmNewState = PE_Get_Manfacturer_Info;
                break;
            }
#if 0
            case PD_DPM_SECURITY_REQUEST:
            {
                pdInstance->extAllDataSize = pdInstance->extRemainSize = pdInstance->commandExtParam.dataLength;
                pdInstance->extChunkNumber = 0;
                pdInstance->extDataBuffer = pdInstance->commandExtParam.dataBuffer;
                pdInstance->extRequestChunkHeader.bitFields.chunked = (pdInstance->unchunkedFeature ? 0 : 1);
                pdInstance->extRequestChunkHeader.bitFields.chunkNumber = 0;
                pdInstance->extRequestChunkHeader.bitFields.dataSize = pdInstance->extAllDataSize;
                pdInstance->extRequestChunkHeader.bitFields.requestChunk = 0;
                pdInstance->psmNewState = PE_Send_Security_Request;
                break;
            }
#endif
#endif

            default:
                break;
        }
    }
}

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
static void PD_PsmSinkRdyProcessDpmMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    if (triggerInfo->dpmMsg < PD_DPM_CONTROL_DISCOVERY_IDENTITY)
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        if (!PD_PsmStartCommand(pdInstance, triggerInfo->dpmMsg, 1))
        {
            PD_PsmCommandFail(pdInstance, triggerInfo->dpmMsg);
            return;
        }
        switch (triggerInfo->dpmMsg)
        {
            case PD_DPM_CONTROL_REQUEST:
            {
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSinkRequestTimer))
                {
                    pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT;
                }
                break;
            }
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PD_DPM_CONTROL_PR_SWAP:
            {
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPrSwapWaitTimer))
                {
                    pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT;
                }
                break;
            }
#endif
            case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
            {
                pdInstance->psmNewState = PSM_PE_SNK_GET_SOURCE_CAP;
                break;
            }
            case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
            {
                pdInstance->psmNewState = PSM_PE_DR_SNK_GET_SINK_CAP;
                break;
            }
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PD_DPM_GET_SRC_EXT_CAP:
            {
                pdInstance->psmNewState = PE_SNK_GET_SOURCE_CAP_EXT;
                break;
            }
            case PD_DPM_GET_STATUS:
            {
                pdInstance->psmNewState = PE_SNK_Get_Source_Status;
                break;
            }
            case PD_DPM_ALERT:
            {
                pdInstance->psmNewState = PE_SNK_Send_Sink_Alert;
                break;
            }
#endif

            default:
                break;
        }

        PD_PsmSinkAndSourceRdyProcessDpmMessage(pdInstance, triggerInfo);
    }
}
#endif

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static void PD_PsmSourceRdyProcessDpmMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    if (triggerInfo->dpmMsg < PD_DPM_CONTROL_DISCOVERY_IDENTITY)
    {
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        if (!PD_PsmStartCommand(pdInstance, triggerInfo->dpmMsg, 1))
        {
            PD_PsmCommandFail(pdInstance, triggerInfo->dpmMsg);
            return;
        }
        switch (triggerInfo->dpmMsg)
        {
            case PD_DPM_CONTROL_POWER_NEGOTIATION:
            {
                pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
                break;
            }
            case PD_DPM_CONTROL_GOTO_MIN:
            {
                pdInstance->psmGotoMinTx = 1;
                pdInstance->psmNewState = PSM_PE_SRC_TRANSITION_SUPPLY;
                break;
            }
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PD_DPM_CONTROL_PR_SWAP:
            {
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPrSwapWaitTimer))
                {
                    pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT;
                }
                break;
            }
#endif
            case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
            {
                pdInstance->psmNewState = PSM_PE_DR_SRC_GET_SOURCE_CAP;
                break;
            }
            case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
            {
                pdInstance->psmNewState = PSM_PE_SRC_GET_SINK_CAP;
                break;
            }
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PD_DPM_GET_SRC_EXT_CAP:
            {
                pdInstance->psmNewState = PE_DR_SRC_GET_SOURCE_CAP_EXT;
                break;
            }
            case PD_DPM_GET_STATUS:
            {
                pdInstance->psmNewState = PE_SRC_Get_Sink_Status;
                break;
            }
            case PD_DPM_ALERT:
            {
                pdInstance->psmNewState = PE_SRC_Send_Source_Alert;
                break;
            }
#endif

            default:
                break;
        }

        PD_PsmSinkAndSourceRdyProcessDpmMessage(pdInstance, triggerInfo);
    }
}
#endif

#if 0
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
static void PD_PsmSendChunkedMsgRequest(pd_instance_t *pdInstance,
                                        start_of_packet_t sop,
                                        message_type_t extMsgType,
                                        pd_extended_msg_header_t extHeader)
{
    pd_status_t sendStatus;
    pd_psm_state_t prevState;

    if (pdInstance->curPowerRole == kPD_PowerRoleSink)
    {
        prevState = PSM_PE_SNK_READY;
    }
    else
    {
        prevState = PSM_PE_SRC_READY;
    }

    extHeader.bitFields.requestChunk = 1;
    extHeader.bitFields.chunkNumber++;
    sendStatus = PD_MsgSendRequestChunkMsg(pdInstance, sop, extMsgType, extHeader);
    if ((sendStatus == kStatus_PD_Success) && PD_MsgWaitSendResult(pdInstance))
    {
        pdInstance->psmNewState = prevState;
    }
    else
    {
        PD_PsmTransitionOnMsgSendError(pdInstance, prevState, PSM_SEND_SOFT_RESET);
    }
}
#endif
#endif

static void PD_PsmSinkAndSourceRdyProcessPdMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    uint32_t commandResultCallback;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    pd_extended_msg_header_t extHeader;
    extHeader.extendedMsgHeaderVal = (uint32_t)(USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS((triggerInfo->pdMsgDataBuffer)));
#endif

    if ((triggerInfo->pdMsgSop != kPD_MsgSOP) || (triggerInfo->pdMsgType == kPD_MsgVendorDefined))
    {
        return;
    }

    triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
    /* note: vdm message will be processed before this */
    switch (triggerInfo->pdMsgType)
    {
        case kPD_MsgAlert:
        {
            pd_command_data_param_t commandExtParam;
            if (pdInstance->curPowerRole == kPD_PowerRoleSink)
            {
                pdInstance->psmNewState = PE_SNK_Source_Alert_Received;
            }
            else
            {
                pdInstance->psmNewState = PE_SRC_Sink_Alert_Received;
            }
            commandExtParam.dataBuffer = &triggerInfo->pdMsgDataBuffer[0];
            commandExtParam.dataLength = 4;
            commandExtParam.sop = kPD_MsgSOP;
            PD_DpmAppCallback(pdInstance, PD_DPM_ALERT_RECEIVED, &commandExtParam, 0);
            if (commandExtParam.resultStatus == kCommandResult_NotSupported)
            {
                PD_PsmTransitionOnMsgSendControl(
                    pdInstance, kPD_MsgNotSupported,
                    (pdInstance->curPowerRole == kPD_PowerRoleSource) ? PSM_PE_SRC_READY : PSM_PE_SNK_READY);
            }
            break;
        }

        case kPD_MsgNotSupported:
            commandResultCallback = kCommandResult_NotSupported;
#if 0
            if (pdInstance->commandProcessing == PD_DPM_SECURITY_REQUEST)
            {
                PD_DpmAppCallback(pdInstance, PD_DPM_SECURITY_REQUEST_FAIL, &commandResultCallback, 1);
            }
            else
#endif
            if (pdInstance->alertWaitReply)
            {
                pdInstance->alertWaitReply = 0;
                PD_DpmAppCallback(pdInstance, PD_DPM_SEND_ALERT_FAIL, &commandResultCallback, 0);
            }
            else
            {
            }
            break;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        case kPD_MsgGetBatteryCap:
            pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
            pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
            pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
            pdInstance->commandExtParamCallback.sop = triggerInfo->pdMsgSop;
            pdInstance->psmNewState = PE_Give_Battery_Cap;
            break;

        case kPD_MsgGetBatteryStatus:
            pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
            pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
            pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
            pdInstance->commandExtParamCallback.sop = triggerInfo->pdMsgSop;
            pdInstance->psmNewState = PE_Give_Battery_Status;
            break;

        case kPD_MsgGetManufacturerInfo:
            pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
            pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
            pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
            pdInstance->commandExtParamCallback.sop = triggerInfo->pdMsgSop;
            pdInstance->psmNewState = PE_Give_Manufacturer_Info;
            break;

#if 0
        case kPD_MsgSecurityRequest:
        {
            /* (responser) chunked request's response, or the first msg */
            if (!(extHeader.bitFields.requestChunk))
            {
                if (extHeader.bitFields.chunked)
                {
                    if (((extHeader.bitFields.chunkNumber + 1) * 26) < extHeader.bitFields.dataSize)
                    {
                        /* request next chunk */
                        PD_PsmSendChunkedMsgRequest(pdInstance, (start_of_packet_t)triggerInfo->pdMsgSop,
                                                    kPD_MsgSecurityRequest, extHeader);
                    }
                    else
                    {
                        /* receive all the data */
                        pdInstance->psmNewState = PE_Send_Security_Response;
                        pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
                        pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
                        pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
                        pdInstance->commandExtParamCallback.sop = triggerInfo->pdMsgSop;
                        pdInstance->extRequestChunkHeader.bitFields.requestChunk = 0;
                    }
                }
                else
                {
                    /* receive all the data */
                    pdInstance->psmNewState = PE_Send_Security_Response;
                    pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
                    pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
                    pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
                    pdInstance->commandExtParamCallback.sop = triggerInfo->pdMsgSop;
                    pdInstance->extRequestChunkHeader.bitFields.requestChunk = 0;
                }
            }
            else
            {
                /* (initiator) chunked request received */
                pdInstance->extRequestChunkHeader = extHeader;
                pdInstance->psmNewState = PE_Send_Security_Request;
            }
            break;
        }

        case kPD_MsgSecurityResponse:
        {
            /* (responser) chunked request's response, or the first msg */
            if (!(extHeader.bitFields.requestChunk))
            {
                if (extHeader.bitFields.chunked)
                {
                    if (((extHeader.bitFields.chunkNumber + 1) * 26) < extHeader.bitFields.dataSize)
                    {
                        /* request next chunk */
                        PD_PsmSendChunkedMsgRequest(pdInstance, (start_of_packet_t)triggerInfo->pdMsgSop,
                                                    kPD_MsgSecurityResponse, extHeader);
                    }
                    else
                    {
                        /* receive all the data */
                        pdInstance->psmNewState = PE_Security_Response_Received;
                        pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
                        pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
                        pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
                        pdInstance->commandExtParamCallback.sop = triggerInfo->pdMsgSop;
                        pdInstance->extRequestChunkHeader.bitFields.requestChunk = 0;
                    }
                }
                else
                {
                    /* receive all the data */
                    pdInstance->psmNewState = PE_Security_Response_Received;
                    pdInstance->commandExtParamCallback.dataBuffer = (uint8_t *)&(pdInstance->receivedData[1]);
                    pdInstance->commandExtParamCallback.dataBuffer += 2; /* data pos */
                    pdInstance->commandExtParamCallback.dataLength = extHeader.bitFields.dataSize;
                    pdInstance->commandExtParamCallback.sop = triggerInfo->pdMsgSop;
                    pdInstance->extRequestChunkHeader.bitFields.requestChunk = 0;
                }
            }
            else
            {
                /* (initiator) chunked request received */
                pdInstance->extRequestChunkHeader = extHeader;
                pdInstance->psmNewState = PE_Send_Security_Response;
            }
            break;
        }
#endif
#endif

        default:
            break;
    }
}

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
static void PD_PsmSinkRdyProcessPdMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    uint8_t replyNotSupport = 0;

    if ((triggerInfo->pdMsgSop != kPD_MsgSOP) || (triggerInfo->pdMsgType == kPD_MsgVendorDefined))
    {
        return;
    }

    triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
    /* note: vdm message will be processed before this */
    switch (triggerInfo->pdMsgType)
    {
        case kPD_MsgRequest:
        {
            replyNotSupport = PSM_PE_SNK_READY;
            break;
        }

        case kPD_MsgGotoMin:
        {
#if 0
            if (pdInstance->rdoSuccessRequest.bitFields.giveBack)
#endif
            {
                PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_GOTO_MIN, 0);
                pdInstance->psmGotoMinRx = 1;
                pdInstance->psmNewState = PSM_PE_SNK_TRANSITION_SINK;
            }
#if 0
            else
            {
                pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
            }
#endif
            break;
        }

        case kPD_MsgGetSinkCap:
        {
            pdInstance->psmNewState = PSM_PE_SNK_GIVE_SINK_CAP;
            break;
        }

        case kPD_MsgGetSourceCap:
        {
            if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkOnly)
            {
                replyNotSupport = PSM_PE_SNK_READY;
            }
            else
            {
                pdInstance->psmNewState = PSM_PE_DR_SNK_GIVE_SOURCE_CAP;
            }
            break;
        }

        case kPD_MsgPrSwap:
        {
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceDefault) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSourcingDevice) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSinkingHost))
            {
                PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_PR_SWAP, 0);
                pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_EVALUATE_PR_SWAP;
            }
            else
#endif
            {
                replyNotSupport = PSM_PE_SNK_READY;
            }
            break;
        }

        case kPD_MsgDrSwap:
        {
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
            if (0
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                || (pdInstance->psmVdmActiveModeValidMask)
#endif
                    )
            {
                pdInstance->psmNewState = PSM_HARD_RESET;
            }
            else
            {
                if (pdInstance->pdPowerPortConfig->dataFunction == kDataConfig_DRD)
                {
                    PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_DR_SWAP, 0);
                    pdInstance->psmNewState = PSM_PE_DRS_EVALUATE_DR_SWAP;
                }
                else
                {
                    replyNotSupport = PSM_PE_SNK_READY;
                }
            }
#else
            replyNotSupport = PSM_PE_SNK_READY;
#endif
            break;
        }

        case kPD_MsgVconnSwap:
        {
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            if (pdInstance->pdPowerPortConfig->vconnSupported)
            {
                PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_VCONN_SWAP, 0);
                pdInstance->psmNewState = PSM_PE_VCS_EVALUATE_SWAP;
            }
            else
#endif
            {
                replyNotSupport = PSM_PE_SNK_READY;
            }
            break;
        }

        case kPD_MsgGetSourceCapExtended:
        {
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkOnly) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceOnly))
            {
                replyNotSupport = PSM_PE_SNK_READY;
            }
            else
            {
                pdInstance->psmNewState = PE_DR_SNK_GIVE_SOURCE_CAP_EXT;
            }
            break;
        }

        case kPD_MsgGetStatus:
        {
            pdInstance->alertWaitReply = 0;
            pdInstance->psmNewState = PE_SNK_Give_Sink_Status;
            break;
        }

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        case kPD_MsgFrSwap:
        {
            /* not_supported, only source can receive this msg */
            replyNotSupport = PSM_PE_SNK_READY;
            break;
        }
#endif

        default:
            triggerInfo->triggerEvent = PSM_TRIGGER_PD_MSG;
            break;
    }

    if (replyNotSupport)
    {
        PD_PsmTransitionOnMsgSendControl(pdInstance, PD_NOT_SUPPORT_REPLY_MSG, (pd_psm_state_t)replyNotSupport);
    }

    PD_PsmSinkAndSourceRdyProcessPdMessage(pdInstance, triggerInfo);
    if (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG)
    {
        pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
    }
}
#endif

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static void PD_PsmSourceRdyProcessPdMessage(pd_instance_t *pdInstance, psm_trigger_info_t *triggerInfo)
{
    uint8_t replyNotSupport = 0;

    if ((triggerInfo->pdMsgSop != kPD_MsgSOP) || (triggerInfo->pdMsgType == kPD_MsgVendorDefined))
    {
        return;
    }

    triggerInfo->triggerEvent = PSM_TRIGGER_NONE;

    /* note: vdm message will be processed before this */
    switch (triggerInfo->pdMsgType)
    {
        case kPD_MsgGotoMin:
        {
            /* not_supported */
            replyNotSupport = PSM_PE_SRC_READY;
            break;
        }

        case kPD_MsgGetSourceCap:
        {
            pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
            break;
        }

        case kPD_MsgGetSinkCap:
        {
            if (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceOnly)
            {
                replyNotSupport = PSM_PE_SRC_READY;
            }
            else
            {
                pdInstance->psmNewState = PSM_PE_DR_SRC_GIVE_SINK_CAP;
            }
            break;
        }

        case kPD_MsgRequest:
        {
            pdInstance->psmNewState = PSM_PE_SRC_NEGOTIATE_CAPABILITY;
            PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_REQUEST, 0);
            pdInstance->partnerRdoRequest.rdoVal = *((uint32_t *)(&(triggerInfo->pdMsgDataBuffer[0])));
            break;
        }

        case kPD_MsgPrSwap:
        {
#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceDefault) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSourcingDevice) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSinkingHost))
            {
                PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_PR_SWAP, 0);
                pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP;
            }
            else
#endif
            {
                replyNotSupport = PSM_PE_SRC_READY;
            }
            break;
        }

        case kPD_MsgDrSwap:
        {
#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
            if (0
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                || (pdInstance->psmVdmActiveModeValidMask)
#endif
                    )
            {
                pdInstance->psmNewState = PSM_HARD_RESET;
            }
            else
            {
                if (pdInstance->pdPowerPortConfig->dataFunction == kDataConfig_DRD)
                {
                    PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_DR_SWAP, 0);
                    pdInstance->psmNewState = PSM_PE_DRS_EVALUATE_DR_SWAP;
                }
                else
                {
                    replyNotSupport = PSM_PE_SRC_READY;
                }
            }
#else
            replyNotSupport = PSM_PE_SRC_READY;
#endif
            break;
        }

        case kPD_MsgVconnSwap:
        {
#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            if (pdInstance->pdPowerPortConfig->vconnSupported)
            {
                PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_VCONN_SWAP, 0);
                pdInstance->psmNewState = PSM_PE_VCS_EVALUATE_SWAP;
            }
            else
#endif
            {
                replyNotSupport = PSM_PE_SRC_READY;
            }
            break;
        }

        case kPD_MsgGetSourceCapExtended:
        {
            pdInstance->psmNewState = PE_SRC_GIVE_SOURCE_CAP_EXT;
            break;
        }

        case kPD_MsgGetStatus:
        {
            pdInstance->alertWaitReply = 0;
            pdInstance->psmNewState = PE_SRC_Give_Source_Status;
            break;
        }

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        case kPD_MsgFrSwap:
        {
            if ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceDefault) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSourcingDevice) ||
                (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSinkingHost))
            {
                /* no need to enter PE_FRS_SRC_SNK_CC_Signal */
                pdInstance->frSignaledWaitFrSwap = 0u;
                pdInstance->psmNewState = PE_FRS_SRC_SNK_Evaluate_Swap;
            }
            else
            {
                /* not_supported */
                replyNotSupport = PSM_PE_SRC_READY;
            }
            break;
        }
#endif

        default:
        {
            /* receive unexpected message */
            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
            break;
        }
    }

    if (replyNotSupport)
    {
        PD_PsmTransitionOnMsgSendControl(pdInstance, PD_NOT_SUPPORT_REPLY_MSG, (pd_psm_state_t)replyNotSupport);
    }

    PD_PsmSinkAndSourceRdyProcessPdMessage(pdInstance, triggerInfo);
}
#endif

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
static uint8_t PD_PsmVdmIdleProcessDpmMessage(pd_instance_t *pdInstance,
                                              uint8_t statIndex,
                                              psm_trigger_info_t *triggerInfo)
{
    pd_psm_state_t secondNewState = PSM_UNKNOWN;

    if (triggerInfo->dpmMsg >= PD_DPM_CONTROL_DISCOVERY_IDENTITY)
    {
        /* special process */
        if (pdInstance->structuredVdmCommandParameter.vdmSop != statIndex)
        {
            return PSM_UNKNOWN;
        }
        switch (triggerInfo->dpmMsg)
        {
            case PD_DPM_CONTROL_CABLE_RESET:
                if (statIndex == 0)
                {
                    return PSM_UNKNOWN;
                }
                break;

            default:
                break;
        }

        /* common process */
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        if (!PD_PsmStartCommand(pdInstance, triggerInfo->dpmMsg, 1))
        {
            PD_PsmCommandFail(pdInstance, triggerInfo->dpmMsg);
            return PSM_UNKNOWN;
        }
        switch (triggerInfo->dpmMsg)
        {
            case PD_DPM_CONTROL_CABLE_RESET:
            {
                secondNewState = PSM_PE_DFP_CBL_SEND_CABLE_RESET;
                break;
            }
            case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
            {
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVDMBusyTimer))
                {
                    secondNewState = PSM_PE_DFP_UFP_VDM_IDENTITY_REQUEST;
                }
                else
                {
                    pdInstance->psmVDMBusyWaitDpmMsg = PSM_PE_DFP_UFP_VDM_IDENTITY_REQUEST;
                    secondNewState = PSM_PE_DFP_UFP_VDM_VDM_BUSY_WAIT;
                }
                break;
            }
            case PD_DPM_CONTROL_DISCOVERY_SVIDS:
            {
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVDMBusyTimer))
                {
                    secondNewState = PSM_PE_DFP_VDM_SVIDS_REQUEST;
                }
                else
                {
                    pdInstance->psmVDMBusyWaitDpmMsg = PSM_PE_DFP_VDM_SVIDS_REQUEST;
                    secondNewState = PSM_PE_DFP_UFP_VDM_VDM_BUSY_WAIT;
                }
                break;
            }
            case PD_DPM_CONTROL_DISCOVERY_MODES:
            {
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVDMBusyTimer))
                {
                    secondNewState = PSM_PE_DFP_VDM_MODES_REQUEST;
                }
                else
                {
                    pdInstance->psmVDMBusyWaitDpmMsg = PSM_PE_DFP_VDM_MODES_REQUEST;
                    secondNewState = PSM_PE_DFP_UFP_VDM_VDM_BUSY_WAIT;
                }
                break;
            }
            case PD_DPM_CONTROL_ENTER_MODE:
            {
                secondNewState = PSM_PE_DFP_VDM_MODE_ENTRY_REQUEST;
                break;
            }
            case PD_DPM_CONTROL_EXIT_MODE:
            {
                secondNewState = PSM_PE_DFP_VDM_MODE_EXIT_REQUEST;
                break;
            }
            case PD_DPM_CONTROL_SEND_ATTENTION:
            {
                secondNewState = PSM_PE_DFP_VDM_ATTENTION_REQUEST;
                break;
            }
            case PD_DPM_SEND_VENDOR_STRUCTURED_VDM:
            {
                secondNewState = PSM_PE_VENDOR_STRUCTURED_VDM_REQUEST;
                break;
            }
            case PD_DPM_SEND_UNSTRUCTURED_VDM:
            {
                secondNewState = PSM_PD_SEND_UNSTRUCTURED_VDM;
                break;
            }

            default:
                break;
        }
    }

    return secondNewState;
}

static void PD_PsmVdmIdleProcessPdMessage(pd_instance_t *pdInstance, uint8_t statIndex, psm_trigger_info_t *triggerInfo)
{
    /* note: vdm message will be processed before this */
    if (triggerInfo->pdMsgType == kPD_MsgVendorDefined)
    {
        if (triggerInfo->pdMsgSop != statIndex)
        {
            return;
        }

        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;

        /* structured vdm */
        if (triggerInfo->vdmHeader.bitFields.vdmType)
        {
            if (triggerInfo->vdmHeader.bitFields.commandType == kVDM_Initiator)
            {
                pd_structured_vdm_header_t reponseVdmHeader;
                pd_svdm_command_request_t commandVdmRequest;
                uint8_t needReply = 0;
                commandVdmRequest.vdoSop = triggerInfo->pdMsgSop;
                commandVdmRequest.vdmHeader.structuredVdmHeaderVal = triggerInfo->vdmHeader.structuredVdmHeaderVal;
                reponseVdmHeader.structuredVdmHeaderVal = triggerInfo->vdmHeader.structuredVdmHeaderVal;

                /* special process */
                switch (triggerInfo->vdmHeader.bitFields.command)
                {
                    case kVDM_DiscoverIdentity:
                        if (triggerInfo->vdmHeader.bitFields.SVID == PD_STANDARD_ID)
                        {
                            needReply = 1;
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                        }
                        break;

                    case kVDM_DiscoverSVIDs:
                        if (triggerInfo->vdmHeader.bitFields.SVID == PD_STANDARD_ID)
                        {
                            needReply = 1;
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                        }
                        break;

                    case kVDM_DiscoverModes:
                        needReply = 1;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                        break;

                    case kVDM_EnterMode:
                        if (pdInstance->curDataRole == kPD_DataRoleDFP)
                        {
                            /* not supported */
                            reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                            PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader,
                                                           0, NULL);
                        }
                        else
                        {
                            if (triggerInfo->msgHeader.bitFields.NumOfDataObjs > 1)
                            {
                                commandVdmRequest.vdoCount = 1;
                                commandVdmRequest.vdoData = (uint32_t *)&triggerInfo->pdMsgDataBuffer[4];
                            }
                            if (((statIndex == 0) && (pdInstance->curDataRole == kPD_DataRoleUFP)) || (statIndex >= 1))
                            {
                                needReply = 1;
                                PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                            }
                        }
                        break;

                    case kVDM_ExitMode:
                        pdInstance->vdmExitReceived[statIndex] = 0;
                        if (pdInstance->curDataRole == kPD_DataRoleDFP)
                        {
                            /* not supported */
                            reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                            PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader,
                                                           0, NULL);
                        }
                        else
                        {
                            if (((statIndex == 0) && (pdInstance->curDataRole == kPD_DataRoleUFP)) || (statIndex >= 1))
                            {
                                needReply = 1;
                                pdInstance->vdmExitReceived[statIndex] = 0;
                                PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                            }
                        }
                        break;

                    case kVDM_Attention:
                        if (triggerInfo->msgHeader.bitFields.NumOfDataObjs > 1)
                        {
                            commandVdmRequest.vdoCount = 1;
                            commandVdmRequest.vdoData = (uint32_t *)&triggerInfo->pdMsgDataBuffer[4];
                        }
                        needReply = 0;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                        break;

                    default:
                    {
                        /* vendor structured vdm */
                        if (triggerInfo->vdmHeader.bitFields.command >= 16)
                        {
                            needReply = 1;
                            commandVdmRequest.vdoCount = (triggerInfo->pdMsgDataLength - 1);
                            commandVdmRequest.vdmHeader.structuredVdmHeaderVal =
                                USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS((triggerInfo->pdMsgDataBuffer));
                            commandVdmRequest.vdoData = (uint32_t *)(triggerInfo->pdMsgDataBuffer + 4);
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);
                        }
                        break;
                    }
                }

                /* common process */
                switch (triggerInfo->vdmHeader.bitFields.command)
                {
                    case kVDM_DiscoverIdentity:
                    case kVDM_DiscoverSVIDs:
                    case kVDM_DiscoverModes:
                        if (needReply)
                        {
                            if (commandVdmRequest.requestResultStatus == kCommandResult_VDMACK)
                            {
                                reponseVdmHeader.bitFields.commandType = kVDM_ResponderACK;
                                PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex,
                                                               reponseVdmHeader, commandVdmRequest.vdoCount,
                                                               (uint32_t *)commandVdmRequest.vdoData);
                            }
                            else if (commandVdmRequest.requestResultStatus == kCommandResult_VDMNAK)
                            {
                                reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                                PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex,
                                                               reponseVdmHeader, 0, NULL);
                            }
                            else
                            {
                                reponseVdmHeader.bitFields.commandType = kVDM_ResponderBUSY;
                                PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex,
                                                               reponseVdmHeader, 0, NULL);
                            }
                        }
                        break;

                    case kVDM_EnterMode:
                    case kVDM_ExitMode:
                        if (triggerInfo->vdmHeader.bitFields.command == kVDM_ExitMode)
                        {
                            pdInstance->psmVdmActiveModeValidMask &=
                                ~(uint32_t)(0x01u << triggerInfo->vdmHeader.bitFields.objPos);
                        }
                        if (needReply)
                        {
                            if (commandVdmRequest.requestResultStatus == kCommandResult_VDMACK)
                            {
                                reponseVdmHeader.bitFields.commandType = kVDM_ResponderACK;
                                if (PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex,
                                                                   reponseVdmHeader, commandVdmRequest.vdoCount,
                                                                   (uint32_t *)commandVdmRequest.vdoData))
                                {
                                    if (triggerInfo->vdmHeader.bitFields.command == kVDM_EnterMode)
                                    {
                                        pdInstance->psmVdmActiveModeValidMask |=
                                            (uint32_t)(0x01u << triggerInfo->vdmHeader.bitFields.objPos);
                                    }
                                }
                            }
                            else
                            {
                                reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                                PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex,
                                                               reponseVdmHeader, 0, NULL);
                            }
                        }
                        break;

                    default:
                        /* vendor structured vdm */
                        if (triggerInfo->vdmHeader.bitFields.command >= 16)
                        {
                            if (needReply)
                            {
                                if (commandVdmRequest.requestResultStatus == kCommandResult_VDMACK)
                                {
                                    reponseVdmHeader.bitFields.commandType = kVDM_ResponderACK;
                                    PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex,
                                                                   reponseVdmHeader, commandVdmRequest.vdoCount,
                                                                   (uint32_t *)commandVdmRequest.vdoData);
                                }
                                else if (commandVdmRequest.requestResultStatus == kCommandResult_VDMNAK)
                                {
                                    reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                                    PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex,
                                                                   reponseVdmHeader, 0, NULL);
                                }
                                else
                                {
                                    reponseVdmHeader.bitFields.commandType = kVDM_ResponderBUSY;
                                    PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex,
                                                                   reponseVdmHeader, 0, NULL);
                                }
                            }
                        }
                        break;
                }
            }
            else
            {
                /* should not reach here */
            }
        }
        else
        {
            /* unstructured vdm */
            pd_unstructured_vdm_command_param_t unstructuredVDMParam;
            unstructuredVDMParam.vdmSop = triggerInfo->pdMsgSop;
            unstructuredVDMParam.vdmHeaderAndVDOsData = (uint32_t *)triggerInfo->pdMsgDataBuffer;
            unstructuredVDMParam.vdmHeaderAndVDOsCount = (triggerInfo->pdMsgDataLength);
            PD_DpmAppCallback(pdInstance, PD_DPM_UNSTRUCTURED_VDM_RECEIVED, &unstructuredVDMParam, 1);
        }
    }
}
#endif

static uint8_t PD_PsmCanPendingReceive(pd_instance_t *pdInstance)
{
    if ((pdInstance->psmCurState == PSM_PE_SNK_STARTUP) || (pdInstance->psmCurState == PSM_PE_SNK_DISCOVERY) ||
        (pdInstance->psmCurState == PSM_PE_SNK_TRANSITION_TO_DEFAULT) || (pdInstance->psmCurState == PSM_HARD_RESET))
    {
        return 0;
    }

    if ((pdInstance->psmNewState == PSM_PE_SNK_STARTUP) || (pdInstance->psmNewState == PSM_PE_SNK_DISCOVERY) ||
        (pdInstance->psmNewState == PSM_PE_SNK_TRANSITION_TO_DEFAULT) || (pdInstance->psmNewState == PSM_HARD_RESET) ||
        (pdInstance->psmNewState == PSM_PE_SNK_WAIT_FOR_CAPABILITIES))
    {
        return 0;
    }

    return 1;
}

static uint8_t PD_PsmProcessState(pd_instance_t *pdInstance)
{
    uint32_t taskEventSet;
    uint32_t commandResultCallback = kCommandResult_Reject;
    uint32_t inCase32Tmp = 0;
    psm_trigger_info_t triggerInfo;
    uint8_t didNothingStepA = 1;
    uint8_t didNothingStepB = 1;
    uint8_t msgReceived = 0;

    commandResultCallback = commandResultCallback;
    inCase32Tmp = inCase32Tmp;

    if (pdInstance->psmCurState != PSM_UNKNOWN)
    {
        triggerInfo.triggerEvent = PSM_TRIGGER_NONE;
        triggerInfo.pdMsgType = kPD_MsgInvalid;
        triggerInfo.dpmMsg = 0;
        triggerInfo.pdMsgDataBuffer = NULL;

        /* get the newest events */
        if (USB_OsaEventCheck(pdInstance->taskEventHandle, 0xffu, &taskEventSet) == kStatus_USB_OSA_Success)
        {
            if (taskEventSet & PD_TASK_EVENT_TIME_OUT)
            {
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_TIME_OUT);
            }
            else if (taskEventSet & PD_TASK_EVENT_RECEIVED_HARD_RESET)
            {
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG);
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_RECEIVED_HARD_RESET);
                if (pdInstance->psmCurState != PSM_PE_BIST_CARRIER_MODE_2)
                {
                    /* Hard reset is ignored when in BIST carrier mode */
                    pdInstance->hardResetReceived = 0;
                    triggerInfo.triggerEvent = PSM_TRIGGER_RECEIVE_HARD_RESET;
                }
            }
            else if (taskEventSet & PD_TASK_EVENT_FR_SWAP_SINGAL)
            {
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_FR_SWAP_SINGAL);
                if ((pdInstance->curPowerRole == kPD_PowerRoleSink) && (pdInstance->psmExplicitContractExisted))
                {
                    pdInstance->psmNewState = PE_FRS_SNK_SRC_Send_Swap;
                }
            }
            else if (taskEventSet & PD_TASK_EVENT_PD_MSG)
            {
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG);
                msgReceived = 1;

                if (pdInstance->psmCurState != PSM_PE_BIST_TEST_DATA_MODE)
                {
                    inCase32Tmp = 0;
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    if ((pdInstance->receivedSop == kPD_MsgSOPp) || (pdInstance->receivedSop == kPD_MsgSOPpp))
                    {
                        PD_MsgGetReceiveResult(pdInstance);
                        PD_MsgReceive(pdInstance);
                        msgReceived = 0;
                        inCase32Tmp = 1;
                    }
                    else
#endif
                        if ((pdInstance->receivedSop == kPD_MsgSOP) && (PD_MsgGetReceiveResult(pdInstance)))
                    {
                        inCase32Tmp = 1;
                    }
                    else
                    {
                    }

                    if (inCase32Tmp)
                    {
                        triggerInfo.msgHeader.msgHeaderVal = (uint16_t)(MSG_DATA_HEADER);
                        triggerInfo.pdMsgSop = pdInstance->receivedSop;

                        if (!(triggerInfo.msgHeader.bitFields.extended))
                        {
                            if (triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0)
                            {
                                triggerInfo.pdMsgType = triggerInfo.msgHeader.bitFields.messageType;
                                /* There are three places: two message will be sent continuously by one port.
                                 * 1. fast role swap.
                                 * 2. sink beginning power role swap.
                                 * 3. rdo request in source.
                                 * 4. after soft reset, the partner will send source_caps.
                                 */
                                if (triggerInfo.pdMsgType == kPD_MsgAccept)
                                {
                                    if (pdInstance->psmCurState != PSM_SEND_SOFT_RESET)
                                    {
                                        PD_MsgReceive(pdInstance);
                                    }
                                }
                                if (triggerInfo.pdMsgType == kPD_MsgFrSwap)
                                {
                                    pdInstance->frSwapReceived = 1;
                                }
                            }
                            else
                            {
                                triggerInfo.pdMsgType =
                                    (triggerInfo.msgHeader.bitFields.messageType | PD_MSG_DATA_TYPE_MASK);
                                if ((triggerInfo.pdMsgType == kPD_MsgRequest) &&
                                    (pdInstance->revision > triggerInfo.msgHeader.bitFields.specRevision))
                                {
                                    pdInstance->revision = triggerInfo.msgHeader.bitFields.specRevision;
                                    PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                                }
                            }
                            triggerInfo.pdMsgDataBuffer = (uint8_t *)MSG_DATA_BUFFER;
                            triggerInfo.pdMsgDataLength =
                                ((MSG_DATA_HEADER & PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_MASK) >>
                                 PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_POS);
                        }
                        else
                        {
                            triggerInfo.pdMsgType =
                                (triggerInfo.msgHeader.bitFields.messageType | PD_MSG_EXT_TYPE_MASK);
                            triggerInfo.pdMsgDataBuffer = (uint8_t *)MSG_DATA_BUFFER;
                            triggerInfo.pdMsgDataLength =
                                ((MSG_DATA_HEADER & PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_MASK) >>
                                 PD_MSG_HEADER_NUMBER_OF_DATA_OBJECTS_POS);
                            if (!(triggerInfo.pdMsgDataBuffer[1] & 0x80u))
                            {
                                triggerInfo.pdExtMsgLength =
                                    (triggerInfo.pdMsgDataBuffer[0] + (triggerInfo.pdMsgDataBuffer[1] & 0x01u) * 256);
                            }
                            else
                            {
                                triggerInfo.pdExtMsgLength =
                                    (triggerInfo.pdMsgDataBuffer[0] + (triggerInfo.pdMsgDataBuffer[1] & 0x01u) * 16);
                            }
                        }
                        triggerInfo.triggerEvent = PSM_TRIGGER_PD_MSG;

                        if (triggerInfo.pdMsgType == kPD_MsgVendorDefined)
                        {
                            triggerInfo.vdmHeader.structuredVdmHeaderVal = (uint32_t)MSG_DATA_BUFFER[0];
                            /* it is interrupted */
                            PD_MsgReceive(pdInstance);
                            if (USB_OsaEventCheck(pdInstance->taskEventHandle, PD_TASK_EVENT_PD_MSG, &taskEventSet) ==
                                kStatus_USB_OSA_Success)
                            {
                                return kSM_Continue; /* process next message */
                            }
                        }
                    }
                    else
                    {
                    }
                }
                else
                {
                    /* Ignore everything except Hard Reset and disconnect */
                }
            }
            else if (taskEventSet & PD_TASK_EVENT_DPM_MSG)
            {
                uint8_t command;
                command = PD_DpmGetMsg(pdInstance);
                triggerInfo.dpmMsg = 0;
                if (command)
                {
                    triggerInfo.dpmMsg = command;
                    switch (command)
                    {
                        case PD_DPM_CONTROL_SOFT_RESET:
                        case PD_DPM_CONTROL_HARD_RESET:
                        case PD_DPM_FAST_ROLE_SWAP:
                            break;

                        default:
                            if ((pdInstance->psmCurState != PSM_PE_SNK_READY) &&
                                (pdInstance->psmCurState != PSM_PE_SRC_READY))
                            {
                                triggerInfo.dpmMsg = 0;
                            }
                            break;
                    }
                }
                else
                {
                    USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_DPM_MSG);
                }

                if (triggerInfo.dpmMsg != 0)
                {
                    PD_DpmClearMsg(pdInstance, (pd_command_t)triggerInfo.dpmMsg);
                    USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_DPM_MSG);
                    triggerInfo.triggerEvent = PSM_TRIGGER_DPM_MSG;
                }
            }
            else if (taskEventSet & PD_TASK_EVENT_SEND_DONE)
            {
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_SEND_DONE);
            }
            else
            {
                PD_PortTaskEventProcess(pdInstance, taskEventSet);
                triggerInfo.triggerEvent = PSM_TRIGGER_NONE;
            }
        }

        if (((triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo.pdMsgSop == kPD_MsgSOP)) ||
            (triggerInfo.triggerEvent == PSM_TRIGGER_RECEIVE_HARD_RESET))
        {
            didNothingStepA &=
                PD_PsmPrimaryStateProcessPdMsg(pdInstance, (uint8_t *)&pdInstance->psmNewState, &triggerInfo);
        }
        else if (triggerInfo.triggerEvent == PSM_TRIGGER_DPM_MSG)
        {
            didNothingStepA &=
                PD_PsmPrimaryStateProcessDpmMsg(pdInstance, (uint8_t *)&pdInstance->psmNewState, &triggerInfo);
        }
        else
        {
        }

        if (!didNothingStepA)
        {
            if ((msgReceived) && (PD_PsmCanPendingReceive(pdInstance)))
            {
                PD_MsgReceive(pdInstance); /* msg has been processed, receiving next msg */
            }
            return kSM_Continue;
        }

        /* different timeout need hard_reset */
        if (((PD_TimerCheckValidTimeOut(pdInstance, tSinkWaitCapTimer)) ||
             (PD_TimerCheckValidTimeOut(pdInstance, tPSTransitionTimer)) ||
             (PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer))) &&
            (pdInstance->psmHardResetCount <= N_HARD_RESET_COUNT) && (pdInstance->curPowerRole == kPD_PowerRoleSink))
        {
            if (PD_TimerCheckValidTimeOut(pdInstance, tSinkWaitCapTimer))
            {
                PD_TimerClear(pdInstance, tSinkWaitCapTimer);
            }

            if (PD_TimerCheckValidTimeOut(pdInstance, tPSTransitionTimer))
            {
                PD_TimerClear(pdInstance, tPSTransitionTimer);
            }

            if (PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer))
            {
                PD_TimerClear(pdInstance, tNoResponseTimer);
            }

            pdInstance->psmNewState = PSM_HARD_RESET;
            didNothingStepA = 0;
        }

        /* alternate mode tAMETimeoutTimer */
        else if (PD_TimerCheckValidTimeOut(pdInstance, tAMETimeoutTimer))
        {
            PD_TimerClear(pdInstance, tAMETimeoutTimer);
            PD_ConnectAltModeEnterFail(pdInstance, pdInstance->psmPresentlyPdConnected);

#ifdef USBPD_ENABLE_USB_BILLBOARD
            UsbbSetAltModeConfigResult(AMR_CONFIG_FAILED);
#endif
        }

        /* Check if we are waiting for an event for state transition */
        else if (pdInstance->psmNewState == pdInstance->psmCurState)
        {
            didNothingStepB = 0;
            switch (pdInstance->psmCurState)
            {
                case PSM_EXIT_TO_ERROR_RECOVERY:            /* (B) */
                case PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY: /* (B) */
                    return kSM_ErrorRecovery;

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                case PSM_INTERRUPTED_REQUEST: /* (B) */
                    /* Transitioned back here after VDM handling, re attempt the request */
                    pdInstance->psmNewState = pdInstance->psmInterruptedState;
                    pdInstance->psmInterruptedState = PSM_UNKNOWN;
                    break;
#endif

                /* do har_reset actively do in (C) */
                case PSM_HARD_RESET: /* (B) */
                    if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                    {
                        /* source send hard_reset, 2. after tpstimer. */
                        if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSHardResetTimer))
                        {
                            /* Hard reset is complete */
                            pdInstance->psmNewState = PSM_PE_SRC_TRANSITION_TO_DEFAULT;
                        }
                        else
                        {
                            didNothingStepB = 1;
                        }
                    }
                    else if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                    {
                        /* sink send hard_reset */
                        pdInstance->callbackFns->PD_SnkStopDrawVbus(pdInstance->callbackParam, kVbusPower_InHardReset);
                        pdInstance->asmHardResetSnkProcessing = 1;
                        pdInstance->psmNewState = PSM_PE_SNK_TRANSITION_TO_DEFAULT;
                    }
                    else
                    {
                    }
                    break;

                case PSM_SEND_SOFT_RESET: /* (B) */
                    if ((triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo.pdMsgSop == kPD_MsgSOP) &&
                        (triggerInfo.pdMsgType == kPD_MsgAccept))
                    {
                        PD_TimerClear(pdInstance, tSenderResponseTimer);
                        if (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET)
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_SUCCESS, NULL, 1);
                        }
                        else
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_REQUEST, NULL, 0);
                        }

                        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        if (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET)
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_FAIL, NULL, 1);
                        }
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_SOFT_RESET: /* (B) */
                    /* If we get here, then recover with a hard reset */
                    pdInstance->psmNewState = PSM_HARD_RESET;
                    break;

                case PSM_CHECK_ASYNC_RX: /* (B) */
                    /* The async conditions at the top of the loop have been evaluated */
                    /* continue to soft reset */
                    pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                    break;

                case PSM_BYPASS: /* (B) */
                    /* We do nothing here. */
                    didNothingStepB = 1;
                    break;

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
                case PSM_PE_SRC_STARTUP: /* (B) */
                    /* only for pr_swap, the state will enter this */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else if (PD_TimerCheckValidTimeOut(pdInstance, tSwapSourceStartTimer))
                    {
                        PD_TimerClear(pdInstance, tSwapSourceStartTimer);
                        pdInstance->psmNewState = PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SRC_DISCOVERY: /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSourceCapabilityTimer))
                    {
                        if ((pdInstance->psmCapsCounter > N_CAPS_COUNT))
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_DISABLED;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
                        }
                    }
                    else if ((PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer)) &&
                             (pdInstance->psmHardResetCount > N_HARD_RESET_COUNT))
                    {
                        if (pdInstance->psmPreviouslyPdConnected)
                        {
                            pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_DISABLED;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY: /* (B) */
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    Pd_PsmSecondaryStateHandler(pdInstance, 1, kPD_MsgSOPp, &triggerInfo);
                    if (pdInstance->psmSecondaryState[1] == PSM_IDLE)
                    {
                        pdInstance->pendingSOP = kPD_MsgSOPMask;
                        pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
                    }
#endif
                    break;

                case PSM_PE_SRC_SEND_CAPABILITIES: /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        PD_TimerClear(pdInstance, tSenderResponseTimer);
                        if (triggerInfo.pdMsgType == kPD_MsgRequest)
                        {
                            pdInstance->partnerRdoRequest.rdoVal = *((uint32_t *)(&(triggerInfo.pdMsgDataBuffer[0])));
                            pdInstance->psmNewState = PSM_PE_SRC_NEGOTIATE_CAPABILITY;
                        }
                        else
                        {
                            /* discard vendor defined message at here */
                            if (triggerInfo.pdMsgType == kPD_MsgVendorDefined)
                            {
                                pdInstance->psmNewState = pdInstance->psmCurState;
                            }
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else if ((PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer)) &&
                             (pdInstance->psmHardResetCount > N_HARD_RESET_COUNT))
                    {
                        if (pdInstance->psmPreviouslyPdConnected)
                        {
                            pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_DISABLED;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SRC_TRANSITION_SUPPLY: /* (B) */
                    PD_PsmTransitionOnMsgSendControl(pdInstance, kPD_MsgPsRdy, PSM_PE_SRC_READY);
                    if (pdInstance->psmGotoMinTx)
                    {
                        pdInstance->psmGotoMinTx = 0;
                        PD_DpmAppCallback(pdInstance, PD_DPM_SRC_GOTOMIN_SUCCESS, NULL, 1);
                    }
                    else
                    {
                        if (pdInstance->psmNewState == PSM_PE_SRC_READY)
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_SRC_RDO_SUCCESS, NULL, 1);
                        }
                    }
                    break;

                case PSM_PE_SRC_READY: /* (B) */
                                       /* state machine */
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                    inCase32Tmp = 1;
                    inCase32Tmp &= Pd_PsmSecondaryStateHandler(pdInstance, 0, kPD_MsgSOP, &triggerInfo);
                    inCase32Tmp &= Pd_PsmSecondaryStateHandler(pdInstance, 1, kPD_MsgSOPp, &triggerInfo);
                    inCase32Tmp &= Pd_PsmSecondaryStateHandler(pdInstance, 2, kPD_MsgSOPpp, &triggerInfo);
                    if ((pdInstance->psmSecondaryState[0] != PSM_IDLE) ||
                        (pdInstance->psmSecondaryState[1] != PSM_IDLE) ||
                        (pdInstance->psmSecondaryState[2] != PSM_IDLE))
                    {
                        if (inCase32Tmp)
                        {
                            didNothingStepB = 0;
                        }
                        break;
                    }
#endif

                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        didNothingStepB = 0;
                        PD_PsmSourceRdyProcessPdMessage(pdInstance, &triggerInfo);
                    }
                    else if (triggerInfo.triggerEvent == PSM_TRIGGER_DPM_MSG)
                    {
                        didNothingStepB = 0;
                        PD_PsmSourceRdyProcessDpmMessage(pdInstance, &triggerInfo);
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SRC_DISABLED: /* (B) */
                    if ((pdInstance->psmPreviouslyPdConnected) &&
                        (PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer)) &&
                        (pdInstance->psmHardResetCount > N_HARD_RESET_COUNT))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SRC_WAIT_NEW_CAPABILITIES: /* (B) */
                    /* The exit transition is handled by the global state transitions */
                    didNothingStepB = 1;
                    break;

                case PSM_PE_SRC_HARD_RESET_RECEIVED: /* (B) */
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSHardResetTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_SRC_TRANSITION_TO_DEFAULT;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SRC_TRANSITION_TO_DEFAULT: /* (B) */
                    if (pdInstance->psmHardResetNeedsVSafe0V)
                    {
                        /* source send hard_reset, 4. after tSrcRecover, open vsafe5v. */
                        /* source receive hard_reset, 4. after tSrcRecover, open vsafe5v. */
                        /* 2. wait tSrcRecover */
                        if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSrcRecoverTimer))
                        {
                            pdInstance->psmHardResetNeedsVSafe0V = 0;
                            pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam,
                                                                           kVbusPower_InHardReset);
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
#ifdef USBPD_ENABLE_VCONN_DISCHARGE
                            PD_DpmDischargeVconn(pdInstance, 0);
#endif
                            PD_DpmSetVconn(pdInstance, pdInstance->raPresent);
#endif
                        }
                        else
                        {
                            didNothingStepB = 1;
                        }
                    }
                    /* source send hard_reset, 5. wait vsafe5v. */
                    /* source receive hard_reset, 5. wait vsafe5v. */
                    else if (PD_DpmCheckVbus(pdInstance))
                    {
                        PD_DpmAppCallback(pdInstance, PD_DPM_SRC_HARD_RESET_REQUEST, NULL, 1);
                        PD_TimerStart(pdInstance, tNoResponseTimer, T_NO_RESPONSE);
                        pdInstance->enterSrcFromSwap = 0;
                        pdInstance->psmNewState = PSM_PE_SRC_STARTUP;
                    }
                    else
                    {
                        /* Wait until the power supply transitions to default. */
                        didNothingStepB = 0;
                    }
                    break;

#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
                case PSM_PE_DR_SRC_GET_SOURCE_CAP: /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        PD_TimerClear(pdInstance, tSenderResponseTimer);
                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && ((triggerInfo.pdMsgType == kPD_MsgReject) ||
                                                                     (triggerInfo.pdMsgType == kPD_MsgNotSupported)))
                        {
                            if (pdInstance->psmCurState == PSM_PE_DR_SRC_GET_SOURCE_CAP)
                            {
                                pdInstance->psmNewState = PSM_PE_SRC_READY;
                            }
                            else
                            {
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                            }

                            if (triggerInfo.pdMsgType == kPD_MsgReject)
                            {
                                commandResultCallback = kCommandResult_Reject;
                            }
                            else
                            {
                                commandResultCallback = kCommandResult_NotSupported;
                            }
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_PARTNER_SRC_CAP_FAIL, &commandResultCallback, 1);
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        if (pdInstance->psmCurState == PSM_PE_DR_SRC_GET_SOURCE_CAP)
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF: /* (B) */
                    /* check source is standby */
                    if (PD_DpmCheckVsafe0V(pdInstance))
                    {
                        /* NOTE : DPM will actively discharge VBUS, and use a timer to ensure at least a min discharge
                         * time
                         */
                        /* have enter standby */
                        PD_DpmDischargeVbus(pdInstance, 0);
                        pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_ASSERT_RD;
                    }
                    else if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        pdInstance->psmNewState = PSM_CHECK_ASYNC_RX;
                    }
                    else
                    {
                        didNothingStepB = 0; /* need check vbus constantly */
                    }
                    break;

                case PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON: /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPsRdy))
                        {
                            PD_MsgStopReceive(pdInstance);
                            PD_TimerClear(pdInstance, tPSSourceOnTimer);
                            /* Swap from SOURCE to SINK */
                            /* pr swap end */
                            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                            /* Cable plug will need a soft reset */
                            pdInstance->psmCablePlugResetNeeded = 1;
#endif

                            PD_PhyControl(pdInstance, PD_PHY_SNK_GET_TYPEC_CURRENT_CAP, &inCase32Tmp);
                            pdInstance->callbackFns->PD_SnkDrawTypeCVbus(pdInstance->callbackParam,
                                                                         (uint8_t)inCase32Tmp, kVbusPower_InPRSwap);
                            PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_SUCCESS, NULL, 1);

                            pdInstance->psmNewState = PSM_PE_SNK_STARTUP;
                        }
                        else
                        {
                            /* A protocol error during power role swap triggers a Hard Reset */
                            pdInstance->psmNewState = PSM_HARD_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSSourceOnTimer))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        switch (triggerInfo.pdMsgType)
                        {
                            case kPD_MsgAccept:
                                pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF;
                                break;

                            case kPD_MsgReject:
                                pdInstance->psmNewState = PSM_PE_SRC_READY;
                                commandResultCallback = kCommandResult_Reject;
                                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, &commandResultCallback, 1);
                                break;

                            case kPD_MsgWait:
                                pdInstance->psmNewState = PSM_PE_SRC_READY;
                                PD_TimerStart(pdInstance, tPrSwapWaitTimer, T_PRSWAP_WAIT);
                                commandResultCallback = kCommandResult_Wait;
                                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, &commandResultCallback, 1);
                                break;

                            default:
                                if (triggerInfo.pdMsgType != kPD_MsgInvalid)
                                {
                                    /* SourceCapabilities and VendorDefined are handled in the global section. */
                                    /* soft reset other packets */
                                    pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                                }
                                break;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_SRC_READY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                }
                break;
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
                case PSM_PE_SNK_DISCOVERY: /* (B) */
                    if ((pdInstance->psmPreviouslyPdConnected) &&
                        (PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer)) &&
                        (pdInstance->psmHardResetCount > N_HARD_RESET_COUNT))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else if (pdInstance->psmHardResetNeedsVSafe0V)
                    {
                        if ((PD_DpmCheckVsafe0V(pdInstance)) ||
                            (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSHardResetTimer)))
                        {
                            pdInstance->psmHardResetNeedsVSafe0V = 0;
                        }
                        else
                        {
                            /* No change in state, we should stay in low power mode */
                            didNothingStepB = 1;
                        }
                    }
                    else if ((PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &taskEventSet) ==
                              kStatus_PD_Success) &&
                             (taskEventSet & PD_VBUS_POWER_STATE_VBUS_MASK))
                    {
                        if (pdInstance->asmHardResetSnkProcessing)
                        {
                            pdInstance->asmHardResetSnkProcessing = 0;
                            PD_PhyControl(pdInstance, PD_PHY_SNK_GET_TYPEC_CURRENT_CAP, &inCase32Tmp);
                            pdInstance->callbackFns->PD_SnkDrawTypeCVbus(pdInstance->callbackParam,
                                                                         (uint8_t)inCase32Tmp, kVbusPower_InHardReset);
                            PD_DpmAppCallback(pdInstance, PD_DPM_SNK_HARD_RESET_REQUEST, NULL, 1);
                        }

                        /* Hard Reset is no longer in progress, allow VBus monitoring */
                        /* kVbusPower_InHardReset is end */
                        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
                        pdInstance->psmHardResetNeedsVSafe0V = 0;

                        pdInstance->psmNewState = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
                    }
                    else
                    {
                        /* need check vbus constantly */
                        didNothingStepB = 0;
                    }
                    break;

                case PSM_PE_SNK_WAIT_FOR_CAPABILITIES: /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                    }
                    else if ((pdInstance->psmPreviouslyPdConnected) &&
                             (PD_TimerCheckValidTimeOut(pdInstance, tNoResponseTimer)) &&
                             (pdInstance->psmHardResetCount > N_HARD_RESET_COUNT))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SNK_SELECT_CAPABILITY: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        switch (triggerInfo.pdMsgType)
                        {
                            case kPD_MsgAccept:
                                pdInstance->psmNewState = PSM_PE_SNK_TRANSITION_SINK;
                                break;

                            case kPD_MsgWait:
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                                commandResultCallback = kCommandResult_Wait;
                                PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RDO_FAIL, &commandResultCallback, 1);
                                if (pdInstance->psmExplicitContractExisted)
                                {
                                    pdInstance->psmNewState = PSM_PE_SNK_READY;
                                    pdInstance->psmSnkReceiveRdoWaitRetry = 1;
                                    PD_TimerStart(pdInstance, tSinkRequestTimer, T_SINK_REQUEST);
                                }
                                else
                                {
                                    pdInstance->psmNewState = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
                                }
                                break;

                            case kPD_MsgReject:
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                                commandResultCallback = kCommandResult_Reject;
                                PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RDO_FAIL, &commandResultCallback, 1);
                                if (pdInstance->psmExplicitContractExisted)
                                {
                                    pdInstance->psmNewState = PSM_PE_SNK_READY;
                                }
                                else
                                {
                                    pdInstance->psmNewState = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
                                }
                                break;

                            default:
                                if (triggerInfo.pdMsgType != kPD_MsgInvalid)
                                {
                                    /* SourceCapabilities and VendorDefined are handled in the global section. */
                                    /* soft reset other packets */
                                    pdInstance->psmNewState = PSM_CHECK_SINK_SOURCE_CAP_RX;
                                }
                                break;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                }
                break;

                case PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT: /* (B) */
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSinkRequestTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY;
                    }
                    break;

                case PSM_PE_SNK_TRANSITION_SINK: /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPing))
                        {
                            /* Remain in the same state */
                            pdInstance->psmNewState = PSM_PE_SNK_TRANSITION_SINK;
                        }
                        else
                        {
                            PD_TimerClear(pdInstance, tPSTransitionTimer);
                            if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPsRdy))
                            {
                                PD_MsgReceive(pdInstance);
                                if (pdInstance->psmGotoMinRx)
                                {
                                    pdInstance->psmGotoMinRx = 0;
                                    PD_DpmAppCallback(pdInstance, PD_DPM_SNK_GOTOMIN_SUCCESS, NULL, 1);
                                }
                                else
                                {
                                    pdInstance->rdoSuccessRequest = pdInstance->rdoRequest;
                                    pdInstance->callbackFns->PD_SnkDrawRequestVbus(pdInstance->callbackParam,
                                                                                   pdInstance->rdoRequest);
                                    PD_DpmAppCallback(pdInstance, PD_DPM_SNK_RDO_SUCCESS, NULL, 1);
                                }
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                            }
                            else
                            {
                                pdInstance->psmNewState = PSM_HARD_RESET;
                            }
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SNK_READY: /* (B) */
                                       /* state machine */
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                    inCase32Tmp = 1;
                    inCase32Tmp &= Pd_PsmSecondaryStateHandler(pdInstance, 0, kPD_MsgSOP, &triggerInfo);
                    inCase32Tmp &= Pd_PsmSecondaryStateHandler(pdInstance, 1, kPD_MsgSOPp, &triggerInfo);
                    inCase32Tmp &= Pd_PsmSecondaryStateHandler(pdInstance, 2, kPD_MsgSOPpp, &triggerInfo);
                    if ((pdInstance->psmSecondaryState[0] != PSM_IDLE) ||
                        (pdInstance->psmSecondaryState[1] != PSM_IDLE) ||
                        (pdInstance->psmSecondaryState[2] != PSM_IDLE))
                    {
                        if (inCase32Tmp)
                        {
                            didNothingStepB = 0;
                        }
                        break;
                    }
#endif

                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        didNothingStepB = 0;
                        PD_PsmSinkRdyProcessPdMessage(pdInstance, &triggerInfo);
                    }
                    else if (triggerInfo.triggerEvent == PSM_TRIGGER_DPM_MSG)
                    {
                        didNothingStepB = 0;
                        PD_PsmSinkRdyProcessDpmMessage(pdInstance, &triggerInfo);
                    }
                    else if (PD_TimerCheckValidTimeOut(pdInstance, tSinkRequestTimer))
                    {
                        if (pdInstance->psmSnkReceiveRdoWaitRetry)
                        {
                            pdInstance->psmSnkReceiveRdoWaitRetry = 0;
                            pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_SNK_TRANSITION_TO_DEFAULT: /* (B) */
                    break;
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)

                case PSM_PE_DR_SNK_GET_SINK_CAP: /* (B) */
                case PSM_PE_SRC_GET_SINK_CAP:    /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if (pdInstance->psmCurState == PSM_PE_SRC_GET_SINK_CAP)
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }

                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        PD_TimerClear(pdInstance, tSenderResponseTimer);
                        if (triggerInfo.pdMsgType == kPD_MsgSinkCapabilities)
                        {
                            pd_capabilities_t sinkCapa;
                            /* Clear any pending message now that we have the latest */
                            (void)PD_DpmClearMsg(pdInstance, PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES);
                            /* Update the PDOs for the EC and send interrupt */
                            sinkCapa.capabilities = MSG_DATA_BUFFER;
                            sinkCapa.capabilitiesCount = triggerInfo.pdMsgDataLength;
                            if (pdInstance->commandProcessing == PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES)
                            {
                                pdInstance->commandProcessing = 0;
                            }
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_PARTNER_SNK_CAP_SUCCESS, &sinkCapa, 1);
                        }
                        else if ((triggerInfo.pdMsgType == kPD_MsgReject) ||
                                 (triggerInfo.pdMsgType == kPD_MsgNotSupported))
                        {
                            if (triggerInfo.pdMsgType == kPD_MsgReject)
                            {
                                commandResultCallback = kCommandResult_Reject;
                            }
                            else
                            {
                                commandResultCallback = kCommandResult_NotSupported;
                            }
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_PARTNER_SNK_CAP_FAIL, &commandResultCallback, 1);
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        if (pdInstance->psmCurState == PSM_PE_SRC_GET_SINK_CAP)
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF: /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPsRdy))
                        {
                            PD_TimerClear(pdInstance, tPSSourceOffTimer);
                            pdInstance->curPowerRole = kPD_PowerRoleSource;
                            PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                            pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_ASSERT_RP;
                        }
                        else if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPing))
                        {
                            /* Remain in the same state */
                            pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF;
                        }
                        else
                        {
                            /* A protocol error during power role swap triggers a Hard Reset */
                            pdInstance->psmNewState = PSM_HARD_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSSourceOffTimer))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_PRS_SNK_SRC_SOURCE_ON: /* (B) */
                    if (PD_DpmCheckVbus(pdInstance))
                    {
                        if (PD_MsgSendControlTransition(pdInstance, kPD_MsgPsRdy, PSM_PE_SRC_STARTUP,
                                                        PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY,
                                                        PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY))
                        {
                            /* Swap from SINK to SOURCE */
                            /* pr swap end */
                            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                            /* Cable plug will need a soft reset */
                            pdInstance->psmCablePlugResetNeeded = 1;
#endif
                            pdInstance->enterSrcFromSwap = 1;
                            PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_SUCCESS, NULL, 1);
                        }
                    }
                    else if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        pdInstance->psmNewState = PSM_CHECK_ASYNC_RX;
                    }
                    else
                    {
                        /* need check vbus constantly */
                        didNothingStepB = 0;
                    }
                    break;

                case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        switch (triggerInfo.pdMsgType)
                        {
                            case kPD_MsgAccept:
                                pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF;
                                break;

                            case kPD_MsgReject:
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                                commandResultCallback = kCommandResult_Reject;
                                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, &commandResultCallback, 1);
                                break;

                            case kPD_MsgWait:
                                pdInstance->psmNewState = PSM_PE_SNK_READY;
                                PD_TimerStart(pdInstance, tPrSwapWaitTimer, T_PRSWAP_WAIT);
                                commandResultCallback = kCommandResult_Wait;
                                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, &commandResultCallback, 1);
                                break;

                            default:
                                if (triggerInfo.pdMsgType != kPD_MsgInvalid)
                                {
                                    /* SourceCapabilities and VendorDefined are handled in the global section. */
                                    /* soft reset other packets */
                                    pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                                }
                                break;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_SNK_READY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                }
                break;

                case PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT: /* (B) */
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPrSwapWaitTimer))
                    {
                        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                        {
                            pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP;
                        }
                    }
                    break;
#endif

#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
                case PSM_PE_DRS_SEND_DR_SWAP: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        switch (triggerInfo.pdMsgType)
                        {
                            case kPD_MsgAccept:
                                pdInstance->curDataRole =
                                    (pdInstance->curDataRole == kPD_DataRoleUFP) ? kPD_DataRoleDFP : kPD_DataRoleUFP;
                                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                                pdInstance->psmNewState = PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP;
                                break;

                            case kPD_MsgReject:
                                pdInstance->psmNewState = (pd_psm_state_t)pdInstance->psmDrSwapPrevState;
                                commandResultCallback = kCommandResult_Reject;
                                PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_FAIL, &commandResultCallback, 1);
                                break;

                            case kPD_MsgWait:
                                pdInstance->psmNewState = (pd_psm_state_t)pdInstance->psmDrSwapPrevState;
                                PD_TimerStart(pdInstance, tDrSwapWaitTimer, T_DRSWAP_WAIT);
                                commandResultCallback = kCommandResult_Wait;
                                PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_FAIL, &commandResultCallback, 1);
                                break;

                            default:
                                if (triggerInfo.pdMsgType != kPD_MsgInvalid)
                                {
                                    /* SourceCapabilities and VendorDefined are handled in the global section. */
                                    /* soft reset other packets */
                                    pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                                }
                                break;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = (pd_psm_state_t)pdInstance->psmDrSwapPrevState;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                }
                break;

                case PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_TIMER_TIME_OUT: /* (B) */
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tDrSwapWaitTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_DRS_SEND_DR_SWAP;
                    }
                    break;
#endif

#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
                case PSM_PE_VCS_SEND_SWAP: /* (B) */
                {
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        switch (triggerInfo.pdMsgType)
                        {
                            case kPD_MsgAccept:
                                pdInstance->psmNewState = (pdInstance->psmPresentlyVconnSource == kPD_IsVconnSource ?
                                                               PSM_PE_VCS_WAIT_FOR_VCONN :
                                                               PSM_PE_VCS_TURN_ON_VCONN);
                                break;

                            case kPD_MsgReject:
                                pdInstance->psmNewState = (pd_psm_state_t)pdInstance->psmVconnSwapPrevState;
                                commandResultCallback = kCommandResult_Reject;
                                PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_FAIL, &commandResultCallback, 1);
                                break;

                            case kPD_MsgWait:
                                pdInstance->psmNewState = (pd_psm_state_t)pdInstance->psmVconnSwapPrevState;
                                commandResultCallback = kCommandResult_Wait;
                                PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_FAIL, &commandResultCallback, 1);
                                break;

                            default:
                                if (triggerInfo.pdMsgType != kPD_MsgInvalid)
                                {
                                    /* SourceCapabilities and VendorDefined are handled in the global section. */
                                    /* soft reset other packets */
                                    pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                                }
                                break;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = (pd_psm_state_t)pdInstance->psmVconnSwapPrevState;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                }
                break;

                case PSM_PE_VCS_WAIT_FOR_VCONN: /* (B) */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        /* SourceCapabilities and VendorDefined are handled in the global section. */
                        PD_TimerClear(pdInstance, tVconnOnTimer);
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) && (triggerInfo.pdMsgType == kPD_MsgPsRdy))
                        {
                            pdInstance->psmNewState = PSM_PE_VCS_TURN_OFF_VCONN;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVconnOnTimer))
                    {
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PSM_PE_VCS_TURN_ON_VCONN: /* (B) */
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVconnOnTimer))
                    {
                        pdInstance->psmNewState = PSM_PE_VCS_SEND_PS_RDY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
#endif

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
                case PE_DR_SRC_GET_SOURCE_CAP_EXT: /* B */
                case PE_SNK_GET_SOURCE_CAP_EXT:    /* B */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if (pdInstance->psmCurState == PE_SNK_GET_SOURCE_CAP_EXT)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }

                        if (triggerInfo.pdMsgType == kPD_MsgSourceCapExtended)
                        {
                            pd_command_data_param_t extMsgParam;
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            /* the fixed 23 bytes length */
                            extMsgParam.dataBuffer = (triggerInfo.pdMsgDataBuffer + 2);
                            if ((triggerInfo.pdExtMsgLength) >= PD_EXTENDED_SRC_CAP_DATA_LENGTH)
                            {
                                extMsgParam.dataLength = PD_EXTENDED_SRC_CAP_DATA_LENGTH;
                            }
                            else
                            {
                                extMsgParam.dataLength = triggerInfo.pdExtMsgLength;
                            }
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_SRC_EXT_CAP_SUCCESS, &extMsgParam, 1);
                        }
                        else if ((triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                                 (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgNotSupported))
                        {
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            commandResultCallback = kCommandResult_NotSupported;
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_SRC_EXT_CAP_FAIL, &commandResultCallback, 1);
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        commandResultCallback = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_GET_SRC_EXT_CAP_FAIL, &commandResultCallback, 1);
                        if (pdInstance->psmCurState == PE_SNK_GET_SOURCE_CAP_EXT)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PE_SRC_Get_Sink_Status:   /* B */
                case PE_SNK_Get_Source_Status: /* B */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if (pdInstance->psmCurState == PE_SNK_Get_Source_Status)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }

                        if (triggerInfo.pdMsgType == kPD_MsgStatus)
                        {
                            pd_command_data_param_t extMsgParam;
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            /* the fixed 3 bytes length */
                            extMsgParam.dataBuffer = (triggerInfo.pdMsgDataBuffer + 2);
                            if ((triggerInfo.pdExtMsgLength) >= PD_EXTENDED_STATUS_MSG_DATA_LENGTH)
                            {
                                extMsgParam.dataLength = PD_EXTENDED_STATUS_MSG_DATA_LENGTH;
                            }
                            else
                            {
                                extMsgParam.dataLength = triggerInfo.pdExtMsgLength;
                            }
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_STATUS_SUCCESS, &extMsgParam, 1);
                        }
                        else if ((triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                                 (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgNotSupported))
                        {
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            commandResultCallback = kCommandResult_NotSupported;
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_STATUS_FAIL, &commandResultCallback, 1);
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        commandResultCallback = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_GET_STATUS_FAIL, &commandResultCallback, 1);
                        if (pdInstance->psmCurState == PE_SNK_Get_Source_Status)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PE_Get_Battery_Cap: /* B */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }

                        if (triggerInfo.pdMsgType == kPD_MsgBatteryCapabilities)
                        {
                            pd_command_data_param_t extMsgParam;
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            /* the fixed 3 bytes length */
                            extMsgParam.dataBuffer = (triggerInfo.pdMsgDataBuffer + 2);
                            if ((triggerInfo.pdExtMsgLength) >= PD_EXTENDED_BATTERY_CAP_MSG_DATA_LENGTH)
                            {
                                extMsgParam.dataLength = PD_EXTENDED_BATTERY_CAP_MSG_DATA_LENGTH;
                            }
                            else
                            {
                                extMsgParam.dataLength = triggerInfo.pdExtMsgLength;
                            }
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_BATTERY_CAP_SUCCESS, &extMsgParam, 1);
                        }
                        else if ((triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                                 (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgNotSupported))
                        {
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            commandResultCallback = kCommandResult_NotSupported;
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_BATTERY_CAP_FAIL, &commandResultCallback, 1);
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        commandResultCallback = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_GET_BATTERY_CAP_FAIL, &commandResultCallback, 1);
                        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PE_Get_Battery_Status: /* B */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }

                        if (triggerInfo.pdMsgType == kPD_MsgBatteryStatus)
                        {
                            pd_command_data_param_t extMsgParam;
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            extMsgParam.dataBuffer = (triggerInfo.pdMsgDataBuffer);
                            extMsgParam.dataLength = triggerInfo.pdExtMsgLength;
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_BATTERY_STATUS_SUCCESS, &extMsgParam, 1);
                        }
                        else if ((triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                                 (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgNotSupported))
                        {
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            commandResultCallback = kCommandResult_NotSupported;
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_BATTERY_STATUS_FAIL, &commandResultCallback, 1);
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        commandResultCallback = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_GET_BATTERY_STATUS_FAIL, &commandResultCallback, 1);
                        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PE_Get_Manfacturer_Info: /* B */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }

                        if (triggerInfo.pdMsgType == kPD_MsgManufacturerInfo)
                        {
                            pd_command_data_param_t extMsgParam;
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            /* 4 ~ 26 bytes length */
                            extMsgParam.dataBuffer = (triggerInfo.pdMsgDataBuffer + 2);
                            extMsgParam.dataLength = triggerInfo.pdExtMsgLength;
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_MANUFACTURER_INFO_SUCCESS, &extMsgParam, 1);
                        }
                        else if ((triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                                 (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgNotSupported))
                        {
                            PD_TimerClear(pdInstance, tSenderResponseTimer);

                            commandResultCallback = kCommandResult_NotSupported;
                            PD_DpmAppCallback(pdInstance, PD_DPM_GET_MANUFACTURER_INFO_FAIL, &commandResultCallback, 1);
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_SEND_SOFT_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        commandResultCallback = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_GET_MANUFACTURER_INFO_FAIL, &commandResultCallback, 1);
                        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                case PE_FRS_SRC_SNK_CC_Signal: /* B */
                    didNothingStepB = 1;
                    if (((triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG) &&
                         (triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                         (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgFrSwap)) ||
                        (pdInstance->frSwapReceived))
                    {
                        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InFRSwap);
                        pdInstance->frSwapReceived = 0;
                        pdInstance->psmNewState = PE_FRS_SRC_SNK_Evaluate_Swap;
                        didNothingStepB = 0;
                    }
                    break;

                case PE_FRS_SRC_SNK_Transition_to_off: /* B */
                {
                    /* wait vbus reach <= vSafe5V */
                    if (PD_DpmCheckLessOrEqualVsafe5v(pdInstance))
                    {
                        pdInstance->psmNewState = PE_FRS_SRC_SNK_Assert_Rd;
                    }
                    else
                    {
                        /* need check vbus constantly */
                        didNothingStepB = 0;
                    }
                    break;
                }

                case PE_FRS_SRC_SNK_Wait_Source_on: /* B */
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if ((triggerInfo.pdMsgSop == kPD_MsgSOP) &&
                            (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgPsRdy))
                        {
                            PD_MsgStopReceive(pdInstance);
                            /* pr swap end */
                            PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
                            PD_TimerClear(pdInstance, tPSSourceOnTimer);
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                            /* Cable plug will need a soft reset */
                            pdInstance->psmCablePlugResetNeeded = 1;
#endif

                            PD_PhyControl(pdInstance, PD_PHY_SNK_GET_TYPEC_CURRENT_CAP, &inCase32Tmp);
                            pdInstance->callbackFns->PD_SnkDrawTypeCVbus(pdInstance->callbackParam,
                                                                         (uint8_t)inCase32Tmp, kVbusPower_InFRSwap);
                            PD_DpmAppCallback(pdInstance, PD_DPM_FR_SWAP_SUCCESS, NULL, 1);
                            pdInstance->psmNewState = PSM_PE_SNK_STARTUP;
                        }
                        else
                        {
                            /* A protocol error during power role swap triggers a Hard Reset */
                            pdInstance->psmNewState = PSM_HARD_RESET;
                        }
                    }
                    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tPSSourceOnTimer))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PE_FRS_SNK_SRC_Send_Swap: /* B */
                    if ((pdInstance->fr5VOpened == 0) && PD_DpmCheckLessOrEqualVsafe5v(pdInstance))
                    {
                        pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_InFRSwap);
                        pdInstance->fr5VOpened = 1;
                    }
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if ((triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                            (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgAccept))
                        {
                            pdInstance->psmNewState = PE_FRS_SNK_SRC_Transition_to_off;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
                        }
                    }
                    else if (PD_TimerCheckValidTimeOut(pdInstance, tSenderResponseTimer))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PE_FRS_SNK_SRC_Transition_to_off: /* B */
                    if ((pdInstance->fr5VOpened == 0) && PD_DpmCheckLessOrEqualVsafe5v(pdInstance))
                    {
                        pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_InFRSwap);
                        pdInstance->fr5VOpened = 1;
                    }
                    if (triggerInfo.triggerEvent == PSM_TRIGGER_PD_MSG)
                    {
                        if ((triggerInfo.msgHeader.bitFields.NumOfDataObjs == 0) &&
                            (triggerInfo.msgHeader.bitFields.messageType == kPD_MsgPsRdy))
                        {
                            PD_TimerClear(pdInstance, tPSSourceOffTimer);
                            PD_MsgSetPortRole(pdInstance, kPD_PowerRoleSource, pdInstance->curDataRole);
                            pdInstance->psmNewState = PE_FRS_SNK_SRC_Vbus_Applied;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
                        }
                    }
                    else if (PD_TimerCheckValidTimeOut(pdInstance, tPSSourceOffTimer))
                    {
                        pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY;
                    }
                    else
                    {
                        didNothingStepB = 1;
                    }
                    break;

                case PE_FRS_SNK_SRC_Vbus_Applied: /* B */
                {
                    uint8_t checkVal;
                    if ((pdInstance->fr5VOpened == 0) && PD_DpmCheckLessOrEqualVsafe5v(pdInstance))
                    {
                        pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_InFRSwap);
                        pdInstance->fr5VOpened = 1;
                    }
                    PD_PhyControl(pdInstance, PD_PHY_FR_SWAP_CHECK_VBUS_APPLIED, &checkVal);
                    if ((pdInstance->fr5VOpened) || (checkVal))
                    {
                        pdInstance->fr5VOpened = 1;
                        pdInstance->psmNewState = PE_FRS_SNK_SRC_Assert_Rp;
                    }
                    else
                    {
                        /* need check vbus constantly */
                        didNothingStepB = 0;
                    }
                    break;
                }
#endif
                case PSM_PE_BIST_TEST_DATA_MODE: /* (B) */
                    didNothingStepB = 1;
                    break;

                case PSM_PE_BIST_CARRIER_MODE_2: /* (B) */
#if (defined PD_CONFIG_COMPLIANCE_TEST_ENABLE) && (PD_CONFIG_COMPLIANCE_TEST_ENABLE)
                    if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tBISTContModeTimer))
                    {
                        PD_PhyControl(pdInstance, PD_PHY_RESET_MSG_FUNCTION, NULL);
                        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_TRANSITION_TO_DEFAULT;
                        }
                        if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                        {
                            pdInstance->psmNewState = PSM_PE_SNK_TRANSITION_TO_DEFAULT;
                        }
                        /* PD_PhyControl(pdInstance, PD_PHY_EXIT_BIST, NULL); */
                    }
                    else
                    {
                        /* We do nothing here, it's all done by the hardware until the device is power cycled. */
                        didNothingStepB = 1;
                    }
#endif
                    break;

                default:
                    didNothingStepB = 1;

                    break;
            }
        }
        else
        {
        }

        if (triggerInfo.triggerEvent != PSM_TRIGGER_NONE)
        {
            if (triggerInfo.triggerEvent == PSM_TRIGGER_DPM_MSG)
            {
                PD_PsmCommandFail(pdInstance, triggerInfo.dpmMsg);
            }
            triggerInfo.triggerEvent = PSM_TRIGGER_NONE;
        }
    }

    if ((msgReceived) && (PD_PsmCanPendingReceive(pdInstance)))
    {
        PD_MsgReceive(pdInstance); /* msg has been processed, receiving next msg */
    }

    if ((!didNothingStepB) || (!didNothingStepA))
    {
        return kSM_Continue;
    }

    return kSM_WaitEvent;
}

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
static uint32_t *PD_PsmGetSourcePDOs(pd_instance_t *pdInstance)
{
    uint8_t index;
    pd_source_pdo_t *pdo;
    pd_source_pdo_t *destPdo;

    for (index = 0; index < pdInstance->pdPowerPortConfig->sourceCapCount; ++index)
    {
        pdo = (pd_source_pdo_t *)&(pdInstance->pdPowerPortConfig->sourceCaps[index]);
        destPdo = (pd_source_pdo_t *)&(pdInstance->sendingData[1 + index]);
        destPdo->PDOValue = pdo->PDOValue;

        switch (pdo->commonPDO.pdoType)
        {
            case kPDO_Fixed:
            {
                if (pdo->fixedPDO.maxCurrent > pdInstance->dpmCableMaxCurrent)
                {
                    destPdo->fixedPDO.maxCurrent = pdInstance->dpmCableMaxCurrent;
                }
                break;
            }

            case kPDO_Variable:
            {
                if (pdo->variablePDO.maxCurrent > pdInstance->dpmCableMaxCurrent)
                {
                    destPdo->variablePDO.maxCurrent = pdInstance->dpmCableMaxCurrent;
                }
                break;
            }

            case kPDO_Battery:
            {
                if ((pdo->batteryPDO.maxAllowPower * 500 / pdo->batteryPDO.minVoltage) > pdInstance->dpmCableMaxCurrent)
                {
                    destPdo->batteryPDO.maxAllowPower =
                        pdInstance->dpmCableMaxCurrent * 10 * (pdo->batteryPDO.minVoltage * 50 / 1000) / 250;
                }
                break;
            }

            default:
                break;
        }
    }

    return (uint32_t *)&(pdInstance->sendingData[1]);
}
#endif

static uint8_t PD_PsmEnterState(pd_instance_t *pdInstance)
{
    pd_psm_state_t prevState;
    uint32_t inCase32Tmp = 0;
    pd_status_t sendStatus = kStatus_PD_Error;
    uint8_t didNothingC = 1;
    uint32_t commandResultCallback = kCommandResult_Error;

    commandResultCallback = commandResultCallback;
    sendStatus = sendStatus;
    while (pdInstance->psmNewState != pdInstance->psmCurState)
    {
        didNothingC = 0;
        prevState = pdInstance->psmCurState;
        pdInstance->psmCurState = pdInstance->psmNewState;

        switch (pdInstance->psmCurState)
        {
            case PSM_EXIT_TO_ERROR_RECOVERY:            /* (C) */
            case PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY: /* (C) */
                if (pdInstance->psmNewState == PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY)
                {
                    /* Delay for 1ms to make sure any previous GoodCRC transmission has ended. */
                    PD_TimerStart(pdInstance, tDelayTimer, mSec(1));
                    while (!(PD_TimerCheckInvalidOrTimeOut(pdInstance, tDelayTimer)))
                    {
                        ;
                    }
                }
                PD_DpmDisconnect(pdInstance);
                /* the code will not excute, fix keil warning and iar misra */
                if (didNothingC != 0)
                {
                    break;
                }
                /* Exit to CLP in disconnected state */
                return kSM_ErrorRecovery;

            case PSM_IDLE: /* (C) */
                pdInstance->psmHardResetCount = 0;
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
                pdInstance->psmCapsCounter = 0;
#endif
                /* kVbusPower_InHardReset is end */
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
                PD_MsgReset(pdInstance);
                PD_PsmReset(pdInstance);
                if (pdInstance->pdConfig->deviceType == kDeviceType_AlternateModeProduct)
                {
                    /* If we are an alternate mode adapter, then set tAMETimeout. */
                    PD_TimerStart(pdInstance, tAMETimeoutTimer, T_AME_TIMEOUT);
                }

                if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                {
#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
                    pdInstance->curPowerRole = kPD_PowerRoleSource;
                    pdInstance->enterSrcFromSwap = 0;
                    pdInstance->psmNewState = PSM_PE_SRC_STARTUP;
#else
                    pdInstance->psmNewState = PSM_BYPASS;
#endif
                }
                else if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                {
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
                    pdInstance->curPowerRole = kPD_PowerRoleSink;
                    pdInstance->psmNewState = PSM_PE_SNK_STARTUP;
#else
                    pdInstance->psmNewState = PSM_BYPASS;
#endif
                }
                else
                {
                }
                break;

            case PSM_INTERRUPTED_REQUEST: /* (C) */
                break;

            case PSM_HARD_RESET: /* (C) */
#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
                /* Do this as early as possible, to prevent disconnects
                 * Hard Reset is in progress, ignore VBus going away.
                 * source need set this too, because judge vbus too (dead battery related) for disconnect.
                */
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InHardReset);
#endif
                PD_PsmReset(pdInstance);
                /* Delay for 1ms to make sure any previous GoodCRC tranmission has ended. */
                PD_TimerStart(pdInstance, tDelayTimer, mSec(1));
                while (!(PD_TimerCheckInvalidOrTimeOut(pdInstance, tDelayTimer)))
                {
                    ;
                }
                /* source send hard_reset, 1. send hard_reset msg. */
                PD_MsgSendHardReset(pdInstance);
                if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                {
                    PD_TimerStart(pdInstance, tPSHardResetTimer, T_PS_HARD_RESET);
                }
                pdInstance->psmHardResetCount++;
                break;

            case PSM_SEND_SOFT_RESET: /* (C) */
            {
                /* Interate at least 1 time to cover an RX between our initial discard, and tx completion */
                inCase32Tmp = 1; /* as retry time variable */
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
                if (0 /*pdInstance->psmSoftResetSop != kPD_MsgSOP */)
                {
                    PD_PsmSecondaryStateHandlerTerminate(pdInstance, pdInstance->psmSoftResetSop);
                    /*Pd_PsmSecondaryStateHandler(pdInstance, pdInstance->psmSoftResetSop, pdInstance->psmSoftResetSop,
                     */
                    /*&triggerInfo); */
                }
#endif

                while (1)
                {
                    if ((PD_MsgSend(pdInstance, kPD_MsgSOP, kPD_MsgSoftReset, 2, NULL) == kStatus_PD_Success) &&
                        (PD_MsgWaitSendResult(pdInstance)))
                    {
                        PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                    }
                    else if (((PD_MsgRecvPending(pdInstance)) &&
                              (((MSG_DATA_HEADER & PD_MSG_HEADER_MESSAGE_TYPE_MASK) >>
                                PD_MSG_HEADER_MESSAGE_TYPE_POS) == kPD_MsgSoftReset)) ||
                             (pdInstance->hardResetReceived))
                    {
                        if (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET)
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_SUCCESS, NULL, 1);
                        }
                        else
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_REQUEST, NULL, 0);
                        }
                    }
                    else if (PD_MsgRecvPending(pdInstance))
                    {
                        if (inCase32Tmp > 0)
                        {
                            inCase32Tmp--;
                            continue;
                        }
                        else
                        {
                            if (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET)
                            {
                                PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_FAIL, NULL, 1);
                            }
                            pdInstance->psmNewState = PSM_HARD_RESET;
                            /* TODO: else send cable reset */
                        }
                    }
                    else
                    {
                        if (pdInstance->commandProcessing == PD_DPM_CONTROL_SOFT_RESET)
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_FAIL, NULL, 1);
                        }
                        pdInstance->psmNewState = PSM_HARD_RESET;
                    }
                    break;
                }
                break;
            }

            case PSM_SOFT_RESET: /* (C) */
                /* Alert the DPM so it can reset it's state */
                if (pdInstance->curPowerRole == kPD_PowerRoleSource)
                {
                    inCase32Tmp = PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY;
                }
                else
                {
                    inCase32Tmp = PSM_PE_SNK_WAIT_FOR_CAPABILITIES;
                }

                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgAccept, (pd_psm_state_t)inCase32Tmp, PSM_HARD_RESET,
                                                PSM_HARD_RESET))
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_SOFT_RESET_REQUEST, NULL, 0);
                }
                break;

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
            case PSM_PE_SRC_STARTUP: /* (C) */
            {
                pd_phy_msg_header_info_t *msgHeader = ((pd_phy_msg_header_info_t *)&inCase32Tmp);
                pdInstance->psmPresentlyPdConnected = 0;
                pdInstance->psmCapsCounter = 0;

                /* kVbusPower_InHardReset is end */
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
                /* Delay for 1ms to make sure any previous GoodCRC tranmission has ended. */
                PD_TimerStart(pdInstance, tDelayTimer, mSec(1));
                while (!(PD_TimerCheckInvalidOrTimeOut(pdInstance, tDelayTimer)))
                {
                    ;
                }
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                msgHeader->dataRole = pdInstance->curDataRole;
                msgHeader->powerRole = pdInstance->curPowerRole;
                msgHeader->cablePlug = 0;
#if (defined PD_SPEC_REVISION_ISSUE_FIX) && (PD_SPEC_REVISION_ISSUE_FIX)
                if (pdInstance->revision >= PD_SPEC_REVISION_30)
                {
                    msgHeader->revision = PD_SPEC_REVISION_20;
                }
                else
#endif
                {
                    msgHeader->revision = pdInstance->revision;
                }
                PD_PhyControl(pdInstance, PD_PHY_SET_MSG_HEADER_INFO, &inCase32Tmp);
                pdInstance->pendingSOP = kPD_MsgSOPMask;
                PD_MsgStartReceive(pdInstance);

                /* A large value allows the sink to debounce CC and VBUS after the connection. */
                if (pdInstance->enterSrcFromSwap)
                {
                    PD_TimerStart(pdInstance, tSwapSourceStartTimer, T_SEND_SOURCE_CAP);
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY;
                }
                break;
            }

            case PSM_PE_SRC_DISCOVERY: /* (C) */
                pdInstance->psmPresentlyPdConnected = 0;
                /* Do not clear previouslyPdConnected here */
                PD_TimerStart(pdInstance, tSourceCapabilityTimer, T_SEND_SOURCE_CAP);
                break;

            case PSM_PE_SRC_IMPLICIT_CABLE_DISCOVERY: /* (C) */
#if (defined PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG) && (PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG)
                if ((pdInstance->raPresent) && (pdInstance->psmCableDiscoveried == 0))
                {
#if 0
                    if ((pdInstance->psmCurState != PSM_INTERRUPTED_REQUEST) &&
                        (pdInstance->psmSecondaryState[1] == PSM_IDLE) &&
                        (pdInstance->psmNewSecondaryState[1] == PSM_UNKNOWN))
#endif
                    {
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                        pdInstance->pendingSOP = kPD_MsgSOPMask | kPD_MsgSOPpMask | kPD_MsgSOPppMask;
#endif
                        if (pdInstance->psmCablePlugResetNeeded)
                        {
                            pdInstance->psmNewSecondaryState[1] = PSM_PE_SRC_IMPLICIT_CABLE_SOFT_RESET;
                        }
                        else
                        {
                            pdInstance->psmNewSecondaryState[1] = PSM_PE_SRC_VDM_IDENTITY_REQUEST;
                        }
                    }
                }
                else
#endif
                {
                    pdInstance->psmNewState = PSM_PE_SRC_SEND_CAPABILITIES;
                }
                break;

            case PSM_PE_SRC_SEND_CAPABILITIES: /* (C) */
                pdInstance->selfOrPartnerFirstSourcePDO.PDOValue = pdInstance->pdPowerPortConfig->sourceCaps[0];
                if (pdInstance->commandProcessing == 0)
                {
                    /* case1: the start-up state machine; case2: dpm msg PD_DPM_CONTROL_POWER_NEGOTIATION */
                    if (!PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_POWER_NEGOTIATION, 1))
                    {
                        /* invalid code, for coverity fix. */
                        pdInstance->commandProcessing = 0;
                    }
                }
                pdInstance->psmCapsCounter++;
                if (PD_MsgSendDataTransition(
                        pdInstance, kPD_MsgSourceCapabilities, pdInstance->pdPowerPortConfig->sourceCapCount,
                        PD_PsmGetSourcePDOs(pdInstance), PE_PSM_STATE_NO_CHANGE, PSM_CHECK_ASYNC_RX,
                        pdInstance->psmPresentlyPdConnected ? PSM_SEND_SOFT_RESET : PSM_PE_SRC_DISCOVERY))
                {
                    pdInstance->psmPresentlyPdConnected = 1;
                    pdInstance->psmPreviouslyPdConnected = 1;
                    PD_TimerClear(pdInstance, tNoResponseTimer);
                    pdInstance->psmHardResetCount = 0;
                    pdInstance->psmCapsCounter = 0;
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PSM_PE_SRC_NEGOTIATE_CAPABILITY: /* (C) */
            {
                pd_negotiate_power_request_t negotiateResult;

                if ((pdInstance->partnerRdoRequest.bitFields.unchunkedSupported) &&
                    (pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.unchunkedSupported))
                {
                    pdInstance->unchunkedFeature = 1;
                }
                else
                {
                    pdInstance->unchunkedFeature = 0;
                }

                negotiateResult.rdo = pdInstance->partnerRdoRequest;
                negotiateResult.negotiateResult = kCommandResult_Accept;
                PD_DpmAppCallback(pdInstance, PD_DPM_SRC_RDO_REQUEST, &negotiateResult, 0);
                if (negotiateResult.negotiateResult == kCommandResult_Accept)
                {
                    pdInstance->psmGotoMinTx = 0;
                    pdInstance->psmNewState = PSM_PE_SRC_TRANSITION_SUPPLY;
                }
                else
                {
                    pdInstance->commandEvaluateResult = negotiateResult.negotiateResult;
                    pdInstance->psmNewState = PSM_PE_SRC_CAPABILITY_RESPONSE;
                }
                break;
            }

            case PSM_PE_SRC_TRANSITION_SUPPLY: /* (C) */
                if (pdInstance->psmGotoMinTx)
                {
                    inCase32Tmp = kPD_MsgGotoMin;
                }
                else
                {
                    inCase32Tmp = kPD_MsgAccept;
                }

                if (PD_MsgSendControlTransition(pdInstance, (message_type_t)inCase32Tmp, PE_PSM_STATE_NO_CHANGE,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET))
                {
                    /* transition power */
                    PD_TimerStart(pdInstance, tDelayTimer, T_SRC_TRANSITION);
                    while (!PD_TimerCheckValidTimeOut(pdInstance, tDelayTimer))
                    {
                        ;
                    }
                    if (pdInstance->psmGotoMinTx)
                    {
                        pdInstance->callbackFns->PD_SrcGotoMinReducePower(pdInstance->callbackParam);
                    }
                    else
                    {
                        pdInstance->callbackFns->PD_SrcTurnOnRequestVbus(pdInstance->callbackParam,
                                                                         pdInstance->partnerRdoRequest);
                    }
                }
                else
                {
                    if (pdInstance->psmGotoMinTx)
                    {
                        pdInstance->psmGotoMinTx = 0;
                    }
                }
                break;

            case PSM_PE_SRC_READY: /* (C) */
                pdInstance->psmSecondaryState[0] = PSM_IDLE;
                pdInstance->psmSecondaryState[1] = PSM_IDLE;
                pdInstance->psmSecondaryState[2] = PSM_IDLE;
                pdInstance->psmExplicitContractExisted = 1;
#if 0 /* even do the cable discovery identity, but cannot do SOP'/SOP'' communication after negotiation done */
/* cannot receive SOP' until explicit contract */
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                if (pdInstance->curDataRole == kPD_DataRoleDFP)
                {
                    pdInstance->pendingSOP = kPD_MsgSOPMask | kPD_MsgSOPpMask | kPD_MsgSOPppMask;
                }
                else
#endif
#endif
                pdInstance->pendingSOP = kPD_MsgSOPMask;
                break;

            case PSM_PE_SRC_WAIT_NEW_CAPABILITIES: /* (C) */
                break;

            case PSM_PE_SRC_DISABLED: /* (C) */
                PD_PhyControl(pdInstance, PD_PHY_DISABLE_MSG_RX, NULL);
                PD_DpmAppCallback(pdInstance, PD_FUNCTION_DISABLED, NULL, 0);
                break;

            case PSM_PE_SRC_CAPABILITY_RESPONSE: /* (C) */
                if (pdInstance->commandEvaluateResult == kCommandResult_Reject)
                {
                    inCase32Tmp = kPD_MsgReject;
                    commandResultCallback = kCommandResult_Reject;
                }
                else
                {
                    inCase32Tmp = kPD_MsgWait;
                    commandResultCallback = kCommandResult_Error;
                }

                if (PD_MsgSendControlTransition(pdInstance, (message_type_t)inCase32Tmp, PE_PSM_STATE_NO_CHANGE,
                                                PSM_HARD_RESET, PSM_HARD_RESET))
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_SRC_RDO_FAIL, &commandResultCallback, 1);
                    if (!pdInstance->psmExplicitContractExisted)
                    {
                        pdInstance->psmNewState = PSM_PE_SRC_WAIT_NEW_CAPABILITIES;
                    }
                    else
                    {
                        uint8_t stillValid = 0;
                        PD_DpmAppCallback(pdInstance, PD_DPM_SRC_CONTRACT_STILL_VALID, &stillValid, 0);
                        if ((stillValid) || (inCase32Tmp == kPD_MsgWait))
                        {
                            pdInstance->psmNewState = PSM_PE_SRC_READY;
                        }
                        else
                        {
                            pdInstance->psmNewState = PSM_HARD_RESET;
                        }
                    }
                }
                break;

            case PSM_PE_SRC_HARD_RESET_RECEIVED: /* (C) */
                break;

            case PSM_PE_SRC_TRANSITION_TO_DEFAULT: /* (C) */
                pdInstance->psmPresentlyPdConnected = 0;
                pdInstance->psmExplicitContractExisted = 0u;
                pdInstance->psmHardResetNeedsVSafe0V = 1;

                /* source send hard_reset, 2. change to supply vsafe5v. */
                /* source receive hard_reset, 2. change to supply vsafe5v. */
                PD_TimerClear(pdInstance, tSrcRecoverTimer);
                do
                {
                    if (pdInstance->callbackFns->PD_ControlVconn != NULL)
                    {
                        pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 0);
                    }
                    pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, kVbusPower_InHardReset);
                    PD_DpmDischargeVbus(pdInstance, 1);
                    inCase32Tmp = 10;
                    while ((!PD_DpmCheckVsafe0V(pdInstance)) && (inCase32Tmp != 0))
                    {
                        inCase32Tmp--;
                    }
                } while (inCase32Tmp == 0);

                PD_DpmDischargeVbus(pdInstance, 0);
                /* source send hard_reset, 3. start tSrcRecover timer. */
                /* source receive hard_reset, 3. start tSrcRecover timer. */
                /* 1. vbus is vsafe0v -> start the tSrcRecover timer -> After tSrcRecover the Source applies power to
                 * VBUS */
                PD_TimerStart(pdInstance, tSrcRecoverTimer, T_SRC_RECOVER);

                /* Message reception should not be re-enabled until PE_SNK_Startup */
                PD_MsgReset(pdInstance);
                /* Change our data role to DFP, and turn off vconn */
                pdInstance->curDataRole = kPD_DataRoleDFP;
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
                /* Request DPM to turn off vconn */
                PD_DpmSetVconn(pdInstance, 0);
#ifdef USBPD_ENABLE_VCONN_DISCHARGE
                PD_DpmDischargeVconn(pdInstance, 1);
#endif
#endif
                break;

            case PSM_PE_SRC_GIVE_SOURCE_CAP: /* (C) */
                if (pdInstance->pdPowerPortConfig->sourceCaps != NULL)
                {
                    PD_MsgSendDataTransition(
                        pdInstance, kPD_MsgSourceCapabilities, pdInstance->pdPowerPortConfig->sourceCapCount,
                        PD_PsmGetSourcePDOs(pdInstance), PSM_PE_SRC_READY, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_PsmTransitionOnMsgSendControl(pdInstance, PD_NOT_SUPPORT_REPLY_MSG, PSM_PE_SRC_READY);
                }
                break;
#endif

#if ((defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)) || \
    (defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE))

#if (defined PD_CONFIG_SOURCE_ROLE_ENABLE) && (PD_CONFIG_SOURCE_ROLE_ENABLE)
            case PSM_PE_SRC_GET_SINK_CAP: /* (C) */
#endif
#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PSM_PE_DR_SNK_GET_SINK_CAP: /* (C) */
#endif
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgGetSinkCap, PE_PSM_STATE_NO_CHANGE, prevState,
                                                PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PSM_PE_DR_SRC_GET_SOURCE_CAP: /* (C) */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgGetSourceCap, PE_PSM_STATE_NO_CHANGE, prevState,
                                                PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
            case PSM_PE_SNK_GET_SOURCE_CAP: /* (C) */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgGetSourceCap, PSM_PE_SNK_READY, prevState,
                                                PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PSM_PE_DR_SRC_GIVE_SINK_CAP: /* (C) */
                if ((pdInstance->pdPowerPortConfig->typecRole != kPowerConfig_SourceOnly) &&
                    (pdInstance->pdPowerPortConfig->sinkCaps != NULL))
                {
                    PD_MsgSendDataTransition(pdInstance, kPD_MsgSinkCapabilities,
                                             pdInstance->pdPowerPortConfig->sinkCapCount,
                                             pdInstance->pdPowerPortConfig->sinkCaps, PSM_PE_SRC_READY,
                                             PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_MsgSendControlTransition(pdInstance, PD_NOT_SUPPORT_REPLY_MSG, PSM_PE_SRC_READY,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;

            case PSM_PE_PRS_SRC_SNK_EVALUATE_PR_SWAP: /* (C) */
                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_REQUEST, &pdInstance->commandEvaluateResult, 0);
                if (pdInstance->commandEvaluateResult == kCommandResult_Accept)
                {
                    pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_ACCEPT_PR_SWAP;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_REJECT_PR_SWAP;
                }
                break;

            case PSM_PE_PRS_SRC_SNK_ACCEPT_PR_SWAP: /* (C) */
                PD_PsmTransitionOnMsgSendControl(pdInstance, kPD_MsgAccept, PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF);
                break;

            case PSM_PE_PRS_SRC_SNK_TRANSITION_TO_OFF: /* (C) */
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InPRSwap);

                /* pr swap transition to standby */
                /* 1. tSrcTransition */
                PD_TimerStart(pdInstance, tDelayTimer, T_SRC_TRANSITION);
                while (!PD_TimerCheckValidTimeOut(pdInstance, tDelayTimer))
                {
                    ;
                }
                /* 2. start enter to standby tSrcSwapStdby */
                pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, kVbusPower_InPRSwap);
                PD_DpmDischargeVbus(pdInstance, 1);
                break;

            case PSM_PE_PRS_SRC_SNK_ASSERT_RD: /* (C) */
                pdInstance->curPowerRole = kPD_PowerRoleSink;
                PD_ConnectSetPRSwapRole(pdInstance, pdInstance->curPowerRole);
                pdInstance->psmNewState = PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON;
                break;

            case PSM_PE_PRS_SRC_SNK_WAIT_SOURCE_ON: /* (C) */
                pdInstance->curPowerRole = kPD_PowerRoleSink;
                /* 0 Role stays as standby until we receive the PS_RDY message */
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgPsRdy, PE_PSM_STATE_NO_CHANGE,
                                                PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY,
                                                PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY))
                {
                    PD_TimerStart(pdInstance, tPSSourceOnTimer, T_PS_SOURCE_ON);
                }
                break;

            case PSM_PE_PRS_SRC_SNK_SEND_PR_SWAP: /* (C) */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgPrSwap, PE_PSM_STATE_NO_CHANGE, prevState,
                                                PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PSM_PE_PRS_SRC_SNK_REJECT_PR_SWAP: /* (C) */
                if (pdInstance->commandEvaluateResult == kCommandResult_Reject)
                {
                    inCase32Tmp = kPD_MsgReject;
                }
                else
                {
                    inCase32Tmp = kPD_MsgWait;
                }
                PD_PsmTransitionOnMsgSendControl(pdInstance, inCase32Tmp, PSM_PE_SRC_READY);
                commandResultCallback = kCommandResult_Reject;
                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, &commandResultCallback, 1);
                break;
#endif

#if (defined PD_CONFIG_SINK_ROLE_ENABLE) && (PD_CONFIG_SINK_ROLE_ENABLE)
            case PSM_PE_SNK_STARTUP: /* (C) */
                /* Delay for 1ms to make sure any previous GoodCRC tranmission has ended. */
                PD_TimerStart(pdInstance, tDelayTimer, mSec(1));
                while (!(PD_TimerCheckInvalidOrTimeOut(pdInstance, tDelayTimer)))
                {
                    ;
                }
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);

                pdInstance->psmPresentlyPdConnected = 0;
                /* Do not clear previouslyPdConnected here */
                pdInstance->psmNewState = PSM_PE_SNK_DISCOVERY;
                break;

            case PSM_PE_SNK_DISCOVERY: /* (C) */
                break;

            case PSM_PE_SNK_WAIT_FOR_CAPABILITIES: /* (C) */
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                pdInstance->pendingSOP = kPD_MsgSOPMask;
                PD_MsgStartReceive(pdInstance);
                PD_TimerStart(pdInstance, tSinkWaitCapTimer, T_SINK_WAIT_CAP);
                break;

            case PSM_PE_SNK_EVALUATE_CAPABILITY: /* (C) */
                pdInstance->psmHardResetCount = 0;
                PD_TimerClear(pdInstance, tNoResponseTimer);
                PD_DpmAppCallback(pdInstance, PD_DPM_SNK_GET_RDO, &pdInstance->rdoRequest, 0);
                if (pdInstance->commandProcessing == 0)
                {
                    PD_PsmStartCommand(pdInstance, PD_DPM_CONTROL_REQUEST, 0);
                }
                if ((pdInstance->rdoRequest.bitFields.unchunkedSupported) &&
                    (pdInstance->selfOrPartnerFirstSourcePDO.fixedPDO.unchunkedSupported))
                {
                    pdInstance->unchunkedFeature = 1;
                }
                else
                {
                    pdInstance->unchunkedFeature = 0;
                }
                pdInstance->psmNewState = PSM_PE_SNK_SELECT_CAPABILITY;
                break;

            case PSM_PE_SNK_SELECT_CAPABILITY: /* (C) */
                /* TX interrupted by RX */
                if (prevState == PSM_PE_SNK_READY)
                {
                    inCase32Tmp = prevState;
                }
                else
                {
                    /* receive packet */
                    inCase32Tmp = PSM_CHECK_ASYNC_RX;
                }

                if (PD_MsgSendDataTransition(pdInstance, kPD_MsgRequest, 1, (uint32_t *)(&pdInstance->rdoRequest),
                                             PE_PSM_STATE_NO_CHANGE, inCase32Tmp, PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PSM_PE_SNK_SELECT_CAPABILITY_WAIT_TIMER_TIME_OUT: /* (C) */
                break;

            case PSM_PE_SNK_TRANSITION_SINK: /* (C) */
                PD_TimerStart(pdInstance, tPSTransitionTimer, T_PS_TRANSITION);
                if (pdInstance->psmGotoMinRx)
                {
                    pdInstance->callbackFns->PD_SnkGotoMinReducePower(pdInstance->callbackParam);
                }
                else
                {
                    /* Start ignoring droops on VBus */
                    PD_ConnectSetPowerProgress(pdInstance, kVbusPower_ChangeInProgress);
                }
                break;

            case PSM_PE_SNK_READY: /* (C) */
                inCase32Tmp = 1;
                pdInstance->psmSecondaryState[0] = PSM_IDLE;
                pdInstance->psmSecondaryState[1] = PSM_IDLE;
                pdInstance->psmSecondaryState[2] = PSM_IDLE;
                pdInstance->psmExplicitContractExisted = 1;
#if 0 /* even do the cable discovery identity, but cannot do SOP'/SOP'' communication after negotiation done */
/* cannot receive SOP' until explicit contract */
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                if (pdInstance->curDataRole == kPD_DataRoleDFP)
                {
                    pdInstance->pendingSOP = kPD_MsgSOPMask | kPD_MsgSOPpMask | kPD_MsgSOPppMask;
                }
                else
#endif
#endif
                pdInstance->pendingSOP = kPD_MsgSOPMask;
                PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &inCase32Tmp);
                /* Stop ignoring droops on VBus */
                /* kVbusPower_ChangeInProgress is done */
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
                break;

            case PSM_PE_SNK_TRANSITION_TO_DEFAULT: /* (C) */
                /* Message reception should not be re-enabled until PE_SNK_Startup */
                PD_MsgReset(pdInstance);
                pdInstance->psmExplicitContractExisted = 0u;
                pdInstance->psmPresentlyPdConnected = 0;
                pdInstance->curDataRole = kPD_DataRoleUFP;
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
                /* Request DPM to turn off vconn */
                PD_DpmSetVconn(pdInstance, 0);
#endif

                PD_TimerStart(pdInstance, tNoResponseTimer, T_NO_RESPONSE);
                PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &inCase32Tmp);
                if (!(inCase32Tmp & PD_VBUS_POWER_STATE_VSAFE0V_MASK))
                {
                    pdInstance->psmHardResetNeedsVSafe0V = 1;
                    PD_TimerStart(pdInstance, tPSHardResetTimer, T_PS_HARD_RESET + 10);
                }
                pdInstance->psmNewState = PSM_PE_SNK_STARTUP;
                break;

            case PSM_PE_SNK_GIVE_SINK_CAP: /* (C) */
                if (pdInstance->pdPowerPortConfig->sinkCaps != NULL)
                {
                    PD_MsgSendDataTransition(pdInstance, kPD_MsgSinkCapabilities,
                                             pdInstance->pdPowerPortConfig->sinkCapCount,
                                             pdInstance->pdPowerPortConfig->sinkCaps, PSM_PE_SNK_READY,
                                             PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_MsgSendControlTransition(pdInstance, PD_NOT_SUPPORT_REPLY_MSG, PSM_PE_SNK_READY,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;
#endif

#if defined(PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
            case PSM_PE_DR_SNK_GIVE_SOURCE_CAP: /* (C) */
                if (((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceOnly) ||
                     (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault) ||
                     (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling)) &&
                    (pdInstance->pdPowerPortConfig->sourceCaps != NULL))
                {
                    PD_MsgSendDataTransition(
                        pdInstance, kPD_MsgSourceCapabilities, pdInstance->pdPowerPortConfig->sourceCapCount,
                        PD_PsmGetSourcePDOs(pdInstance), PSM_PE_SNK_READY, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_MsgSendControlTransition(pdInstance, PD_NOT_SUPPORT_REPLY_MSG, PSM_PE_SNK_READY,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;

            case PSM_PE_PRS_SNK_SRC_EVALUATE_PR_SWAP: /* (C) */
                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_REQUEST, &pdInstance->commandEvaluateResult, 0);
                if (pdInstance->commandEvaluateResult == kCommandResult_Accept)
                {
                    pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_ACCEPT_PR_SWAP;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_REJECT_PR_SWAP;
                }
                break;

            case PSM_PE_PRS_SNK_SRC_ACCEPT_PR_SWAP: /* (C) */
                PD_PsmTransitionOnMsgSendControl(pdInstance, kPD_MsgAccept, PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF);
                break;

            case PSM_PE_PRS_SNK_SRC_TRANSITION_TO_OFF: /* (C) */
                pdInstance->psmExplicitContractExisted = 0;
                inCase32Tmp = 0u;
                PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &inCase32Tmp);
                PD_TimerStart(pdInstance, tPSSourceOffTimer, T_PS_SOURCE_OFF);
                PD_ConnectSetPowerProgress(pdInstance, kVbusPower_InPRSwap);
                /* sink transition to standby. */
                pdInstance->callbackFns->PD_SnkStopDrawVbus(pdInstance->callbackParam, kVbusPower_InPRSwap);
                break;

            case PSM_PE_PRS_SNK_SRC_ASSERT_RP: /* (C) */
                pdInstance->curPowerRole = kPD_PowerRoleSource;
                /* We must assume a new default role */
                pdInstance->curPowerRole = kPD_PowerRoleSource;
                /* For reliability, do not start driving VBus before tSwapSourceStart */
                pdInstance->psmNewState = PSM_PE_PRS_SNK_SRC_SOURCE_ON;
                PD_ConnectSetPRSwapRole(pdInstance, pdInstance->curPowerRole);
                break;

            case PSM_PE_PRS_SNK_SRC_SOURCE_ON: /* (C) */
                pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_InPRSwap);
                break;

            case PSM_PE_PRS_SNK_SRC_SEND_PR_SWAP: /* (C) */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgPrSwap, PE_PSM_STATE_NO_CHANGE, prevState,
                                                PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PSM_PE_PRS_SNK_SRC_REJECT_PR_SWAP: /* (C) */
                if (pdInstance->commandEvaluateResult == kCommandResult_Reject)
                {
                    inCase32Tmp = kPD_MsgReject;
                }
                else
                {
                    inCase32Tmp = kPD_MsgWait;
                }
                PD_PsmTransitionOnMsgSendControl(pdInstance, inCase32Tmp, PSM_PE_SNK_READY);
                commandResultCallback = kCommandResult_Reject;
                PD_DpmAppCallback(pdInstance, PD_DPM_PR_SWAP_FAIL, &commandResultCallback, 1);
                break;
#endif

            case PSM_PE_BIST_TEST_DATA_MODE: /* (C) */
                break;

            case PSM_PE_BIST_CARRIER_MODE_2: /* (C) */
#if (defined PD_CONFIG_COMPLIANCE_TEST_ENABLE) && (PD_CONFIG_COMPLIANCE_TEST_ENABLE)
                PD_TimerStart(pdInstance, tBISTContModeTimer, T_BIST_CONT_MODE);
                /* Delay for 1ms to make sure any previous GoodCRC tranmission has ended. */
                PD_TimerStart(pdInstance, tDelayTimer, mSec(1));
                while (!PD_TimerCheckValidTimeOut(pdInstance, tDelayTimer))
                {
                }
                inCase32Tmp = kBIST_CarrierMode2;
                PD_PhyControl(pdInstance, PD_PHY_ENTER_BIST, &inCase32Tmp);
#endif
                break;

            case PSM_CHECK_ASYNC_RX: /* (C) */
                break;

            case PSM_BYPASS: /* (C) */
                break;

#if defined(PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
            case PSM_PE_DRS_EVALUATE_DR_SWAP: /* (C) */
                pdInstance->psmDrSwapPrevState = prevState;
                PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_REQUEST, &pdInstance->commandEvaluateResult, 0);
                if (pdInstance->commandEvaluateResult == kCommandResult_Accept)
                {
                    pdInstance->psmNewState = PSM_PE_DRS_ACCEPT_DR_SWAP;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_DRS_REJECT_DR_SWAP;
                }
                break;

            case PSM_PE_DRS_EVALUATE_DR_SWAP_WAIT_TIMER_TIME_OUT: /* (C) */
            case PSM_PE_PRS_EVALUATE_PR_SWAP_WAIT_TIMER_TIME_OUT: /* (C) */
                break;

            case PSM_PE_DRS_REJECT_DR_SWAP: /* (C) */
                PD_PsmTransitionOnMsgSendControl(
                    pdInstance,
                    ((pdInstance->commandEvaluateResult == kCommandResult_Reject) ? kPD_MsgReject : kPD_MsgWait),
                    (pd_psm_state_t)pdInstance->psmDrSwapPrevState);
                if (pdInstance->commandEvaluateResult == kCommandResult_Reject)
                {
                    inCase32Tmp = kCommandResult_Reject;
                    PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_FAIL, &inCase32Tmp, 0);
                }
                else
                {
                    inCase32Tmp = kCommandResult_Wait;
                    PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_FAIL, &inCase32Tmp, 0);
                }
                break;

            case PSM_PE_DRS_SEND_DR_SWAP:                   /* (C) */
                pdInstance->psmDrSwapPrevState = prevState; /* snk_rdy or src_rdy */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgDrSwap, PE_PSM_STATE_NO_CHANGE, prevState,
                                                PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PSM_PE_DRS_ACCEPT_DR_SWAP: /* (C) */
            {
                pd_phy_msg_header_info_t *msgHeader = ((pd_phy_msg_header_info_t *)&inCase32Tmp);
                msgHeader->dataRole =
                    ((pdInstance->curDataRole == kPD_DataRoleUFP) ? kPD_DataRoleDFP : kPD_DataRoleUFP);
                msgHeader->powerRole = pdInstance->curPowerRole;
                msgHeader->cablePlug = 0;
                msgHeader->revision = pdInstance->revision;
                PD_PhyControl(pdInstance, PD_PHY_SET_MSG_HEADER_INFO, &inCase32Tmp);
                PD_PsmTransitionOnMsgSendControl(pdInstance, kPD_MsgAccept, PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP);
                if (pdInstance->psmNewState == PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP)
                {
                    pdInstance->curDataRole =
                        (pdInstance->curDataRole == kPD_DataRoleUFP) ? kPD_DataRoleDFP : kPD_DataRoleUFP;
                    pdInstance->sendingMsgHeader.bitFields.portDataRole = pdInstance->curDataRole;
                }
                break;
            }

            case PSM_PE_DRS_CHANGE_TO_DFP_OR_UFP: /* (C) */
                /* Exit any active alternate mode */

                pdInstance->pendingSOP = kPD_MsgSOPMask;
                if (pdInstance->curDataRole == kPD_DataRoleDFP)
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_SUCCESS, NULL, 1);
#if 0 /* even do the cable discovery identity, but cannot do SOP'/SOP'' communication after negotiation done */
#if (defined PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    pdInstance->pendingSOP = kPD_MsgSOPMask | kPD_MsgSOPpMask | kPD_MsgSOPppMask;
#endif
#endif
                }
                else
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_DR_SWAP_SUCCESS, NULL, 1);
                }

                /* Raise the interrupt */
                if ((pdInstance->curDataRole == kPD_DataRoleDFP) || (pdInstance->curPowerRole == kPD_PowerRoleSource))
                {
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    /* Cable plug will need a soft reset */
                    pdInstance->psmCablePlugResetNeeded = 1;
#endif
                }
                /* return to previous state */
                pdInstance->psmNewState = pdInstance->psmDrSwapPrevState;
                break;
#endif

#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
            case PSM_PE_VCS_SEND_SWAP: /* (C) */
                pdInstance->psmVconnSwapPrevState = prevState;
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgVconnSwap, PE_PSM_STATE_NO_CHANGE,
                                                pdInstance->psmVconnSwapPrevState, PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PSM_PE_VCS_WAIT_FOR_VCONN: /* (C) */
                PD_TimerStart(pdInstance, tVconnOnTimer, T_VCONN_SOURCE_ON);
                break;

            case PSM_PE_VCS_TURN_OFF_VCONN: /* (C) */
                pdInstance->psmPresentlyVconnSource = kPD_NotVconnSource;
                /* Inform the DPM */
                pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 0);
                PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_SUCCESS, NULL, 1);
                /* return to previous state */
                pdInstance->psmNewState = pdInstance->psmVconnSwapPrevState;
                break;

            case PSM_PE_VCS_TURN_ON_VCONN: /* (C) */
                pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
                /* Inform the DPM */
                pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 1);
                PD_TimerStart(pdInstance, tVconnOnTimer, T_VCONN_SOURCE_ON / 2);
                break;

            case PSM_PE_VCS_SEND_PS_RDY: /* (C) */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgPsRdy, pdInstance->psmVconnSwapPrevState,
                                                PSM_SEND_SOFT_RESET, PSM_SEND_SOFT_RESET))
                {
                    PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_SUCCESS, NULL, 1);
                }
                break;

            case PSM_PE_VCS_EVALUATE_SWAP: /* (C) */
                pdInstance->psmVconnSwapPrevState = prevState;
                PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_REQUEST, &pdInstance->commandEvaluateResult, 0);
                if (pdInstance->commandEvaluateResult == kCommandResult_Accept)
                {
                    pdInstance->psmNewState = PSM_PE_VCS_ACCEPT_SWAP;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_VCS_REJECT_SWAP;
                }
                break;

            case PSM_PE_VCS_ACCEPT_SWAP: /* (C) */
                PD_PsmTransitionOnMsgSendControl(pdInstance, kPD_MsgAccept,
                                                 pdInstance->psmPresentlyVconnSource == kPD_IsVconnSource ?
                                                     PSM_PE_VCS_WAIT_FOR_VCONN :
                                                     PSM_PE_VCS_TURN_ON_VCONN);

                break;

            case PSM_PE_VCS_REJECT_SWAP: /* (C) */
                PD_PsmTransitionOnMsgSendControl(
                    pdInstance,
                    (pdInstance->commandEvaluateResult == kCommandResult_Reject) ? kPD_MsgReject : kPD_MsgWait,
                    (pd_psm_state_t)pdInstance->psmVconnSwapPrevState);
                if (pdInstance->commandEvaluateResult == kCommandResult_Reject)
                {
                    inCase32Tmp = kCommandResult_Reject;
                    PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_FAIL, &inCase32Tmp, 1);
                }
                else
                {
                    inCase32Tmp = kCommandResult_Wait;
                    PD_DpmAppCallback(pdInstance, PD_DPM_VCONN_SWAP_FAIL, &inCase32Tmp, 1);
                }
                break;
#endif

#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
            case PE_SNK_GET_SOURCE_CAP_EXT:    /* C */
            case PE_DR_SRC_GET_SOURCE_CAP_EXT: /* C */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgGetSourceCapExtended, PE_PSM_STATE_NO_CHANGE,
                                                prevState, PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PE_SRC_GIVE_SOURCE_CAP_EXT:    /* C */
            case PE_DR_SNK_GIVE_SOURCE_CAP_EXT: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_SRC_EXT_CAP, &pdInstance->commandExtParamCallback, 0);
                if (pdInstance->psmCurState == PE_DR_SNK_GIVE_SOURCE_CAP_EXT)
                {
                    inCase32Tmp = PSM_PE_SNK_READY;
                }
                else
                {
                    inCase32Tmp = PSM_PE_SRC_READY;
                }

                if (pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported)
                {
                    PD_MsgSendExtTransition(pdInstance, kPD_MsgSOP, kPD_MsgSourceCapExtended,
                                            pdInstance->commandExtParamCallback.dataLength,
                                            pdInstance->commandExtParamCallback.dataBuffer, (pd_psm_state_t)inCase32Tmp,
                                            PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_MsgSendControlTransition(pdInstance, kPD_MsgNotSupported, (pd_psm_state_t)inCase32Tmp,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;

            case PE_SRC_Get_Sink_Status:   /* C */
            case PE_SNK_Get_Source_Status: /* C */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgGetStatus, PE_PSM_STATE_NO_CHANGE, prevState,
                                                PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PE_SRC_Give_Source_Status: /* C */
            case PE_SNK_Give_Sink_Status:   /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_STATUS, &pdInstance->commandExtParamCallback, 0);
                if (pdInstance->psmCurState == PE_SNK_Give_Sink_Status)
                {
                    inCase32Tmp = PSM_PE_SNK_READY;
                }
                else
                {
                    inCase32Tmp = PSM_PE_SRC_READY;
                }

                if (pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported)
                {
                    PD_MsgSendExtTransition(pdInstance, kPD_MsgSOP, kPD_MsgStatus,
                                            pdInstance->commandExtParamCallback.dataLength,
                                            pdInstance->commandExtParamCallback.dataBuffer, (pd_psm_state_t)inCase32Tmp,
                                            PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_MsgSendControlTransition(pdInstance, kPD_MsgNotSupported, (pd_psm_state_t)inCase32Tmp,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;

            case PE_Get_Battery_Cap: /* C */
            {
                if (PD_MsgSendExtTransition(pdInstance, kPD_MsgSOP, kPD_MsgGetBatteryCap, 1,
                                            &pdInstance->getBatteryCapDataBlock, PE_PSM_STATE_NO_CHANGE, prevState,
                                            PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;
            }

            case PE_Give_Battery_Cap: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_BATTERY_CAP, &pdInstance->commandExtParamCallback, 0);
                if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                {
                    inCase32Tmp = PSM_PE_SNK_READY;
                }
                else
                {
                    inCase32Tmp = PSM_PE_SRC_READY;
                }
                if (pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported)
                {
                    PD_MsgSendExtTransition(pdInstance, kPD_MsgSOP, kPD_MsgBatteryCapabilities,
                                            pdInstance->commandExtParamCallback.dataLength,
                                            pdInstance->commandExtParamCallback.dataBuffer, (pd_psm_state_t)inCase32Tmp,
                                            PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_MsgSendControlTransition(pdInstance, kPD_MsgNotSupported, (pd_psm_state_t)inCase32Tmp,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;

            case PE_Get_Battery_Status: /* C */
                if (PD_MsgSendExtTransition(pdInstance, kPD_MsgSOP, kPD_MsgGetBatteryStatus, 1,
                                            &pdInstance->getBatteryCapDataBlock, PE_PSM_STATE_NO_CHANGE, prevState,
                                            PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PE_Give_Battery_Status: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_BATTERY_STATUS, &pdInstance->commandExtParamCallback, 0);

                if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                {
                    inCase32Tmp = PSM_PE_SNK_READY;
                }
                else
                {
                    inCase32Tmp = PSM_PE_SRC_READY;
                }
                if (pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported)
                {
                    PD_MsgSendDataTransition(pdInstance, kPD_MsgBatteryStatus,
                                             pdInstance->commandExtParamCallback.dataLength >> 2,
                                             (uint32_t *)pdInstance->commandExtParamCallback.dataBuffer,
                                             (pd_psm_state_t)inCase32Tmp, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_MsgSendControlTransition(pdInstance, PD_NOT_SUPPORT_REPLY_MSG, (pd_psm_state_t)inCase32Tmp,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;

            case PE_Get_Manfacturer_Info: /* C */
                if (PD_MsgSendExtTransition(pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop,
                                            kPD_MsgGetManufacturerInfo, 2, pdInstance->commandExtParam.dataBuffer,
                                            PE_PSM_STATE_NO_CHANGE, prevState, PSM_SEND_SOFT_RESET))
                {
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                break;

            case PE_Give_Manufacturer_Info: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_GIVE_MANUFACTURER_INFO, &pdInstance->commandExtParamCallback, 0);
                if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                {
                    inCase32Tmp = PSM_PE_SNK_READY;
                }
                else
                {
                    inCase32Tmp = PSM_PE_SRC_READY;
                }
                if (pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported)
                {
                    PD_MsgSendExtTransition(pdInstance, kPD_MsgSOP, kPD_MsgManufacturerInfo,
                                            pdInstance->commandExtParamCallback.dataLength,
                                            pdInstance->commandExtParamCallback.dataBuffer, (pd_psm_state_t)inCase32Tmp,
                                            PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                else
                {
                    PD_MsgSendControlTransition(pdInstance, kPD_MsgNotSupported, (pd_psm_state_t)inCase32Tmp,
                                                PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;

#if 0
            case PE_Send_Security_Request: /* C */
            {
                uint32_t sendLength;

                pd_extended_msg_header_t extHeader = pdInstance->extRequestChunkHeader;

                if (!(extHeader.bitFields.requestChunk))
                {
                    if (!pdInstance->unchunkedFeature)
                    {
                        sendLength = pdInstance->extAllDataSize;
                        if (sendLength > 26)
                        {
                            sendLength = 26;
                        }
                        extHeader.bitFields.chunked = 1;
                        extHeader.bitFields.chunkNumber = 0;
                        extHeader.bitFields.requestChunk = 0;
                        extHeader.bitFields.dataSize = pdInstance->extAllDataSize;
                        sendStatus = PD_MsgSendChunkedExtendedMsg(
                            pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop, kPD_MsgSecurityRequest,
                            extHeader, sendLength, pdInstance->extDataBuffer);
                    }
                    else
                    {
                        sendStatus = PD_MsgSendUnchunkedExtendedMsg(
                            pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop, kPD_MsgSecurityRequest,
                            pdInstance->extAllDataSize, pdInstance->extDataBuffer);
                    }
                }
                else
                {
                    extHeader.bitFields.requestChunk = 0;
                    sendLength = (pdInstance->extAllDataSize - (extHeader.bitFields.chunkNumber * 26));
                    if (sendLength > 26)
                    {
                        sendLength = 26;
                    }
                    sendStatus = PD_MsgSendChunkedExtendedMsg(
                        pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop, kPD_MsgSecurityRequest, extHeader,
                        sendLength, (pdInstance->extDataBuffer + extHeader.bitFields.chunkNumber * 26));
                }

                if ((sendStatus == kStatus_PD_Success) && PD_MsgWaitSendResult(pdInstance))
                {
                    if (extHeader.bitFields.chunkNumber * 26 >= extHeader.bitFields.dataSize)
                    {
                        PD_DpmAppCallback(pdInstance, PD_DPM_SECURITY_REQUEST_SUCCESS, NULL, 1);
                    }

                    if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                    {
                        pdInstance->psmNewState = PSM_PE_SNK_READY;
                    }
                    else
                    {
                        pdInstance->psmNewState = PSM_PE_SRC_READY;
                    }
                }
                else
                {
                    inCase32Tmp = kCommandResult_Error;
                    PD_DpmAppCallback(pdInstance, PD_DPM_SECURITY_REQUEST_FAIL, &inCase32Tmp, 1);
                    PD_PsmTransitionOnMsgSendError(pdInstance, prevState, PSM_SEND_SOFT_RESET);
                }

                break;
            }

            case PE_Send_Security_Response: /* C */
            {
                uint32_t sendLength;

                pd_extended_msg_header_t extHeader = pdInstance->extRequestChunkHeader;

                if (!(extHeader.bitFields.requestChunk))
                {
                    /* first time */
                    PD_DpmAppCallback(pdInstance, PD_DPM_RESPONSE_SECURITY_REQUEST, &pdInstance->commandExtParamCallback,
                                      0);
                    if (pdInstance->commandExtParamCallback.resultStatus != kCommandResult_NotSupported)
                    {
                        pdInstance->extAllDataSize = pdInstance->commandExtParamCallback.dataLength;
                        pdInstance->extDataBuffer = pdInstance->commandExtParamCallback.dataBuffer;

                        if (!pdInstance->unchunkedFeature)
                        {
                            sendLength = pdInstance->extAllDataSize;
                            if (sendLength > 26)
                            {
                                sendLength = 26;
                            }
                            extHeader.bitFields.chunked = 1;
                            extHeader.bitFields.chunkNumber = 0;
                            extHeader.bitFields.requestChunk = 0;
                            extHeader.bitFields.dataSize = pdInstance->extAllDataSize;
                            sendStatus = PD_MsgSendChunkedExtendedMsg(
                                pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop, kPD_MsgSecurityResponse,
                                extHeader, sendLength, pdInstance->extDataBuffer);
                        }
                        else
                        {
                            sendStatus = PD_MsgSendUnchunkedExtendedMsg(
                                pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop, kPD_MsgSecurityResponse,
                                pdInstance->extAllDataSize, pdInstance->extDataBuffer);
                        }
                    }
                    else
                    {
                        sendStatus = PD_MsgSendDefaultControl(pdInstance, kPD_MsgNotSupported);
                    }
                }
                else
                {
                    extHeader.bitFields.requestChunk = 0;
                    sendLength = (pdInstance->extAllDataSize - (extHeader.bitFields.chunkNumber * 26));
                    if (sendLength > 26)
                    {
                        sendLength = 26;
                    }
                    sendStatus = PD_MsgSendChunkedExtendedMsg(
                        pdInstance, (start_of_packet_t)pdInstance->commandExtParam.sop, kPD_MsgSecurityRequest, extHeader,
                        sendLength, (pdInstance->extDataBuffer + extHeader.bitFields.chunkNumber * 26));
                }

                if ((sendStatus == kStatus_PD_Success) && (PD_MsgWaitSendResult(pdInstance)))
                {
                    if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                    {
                        pdInstance->psmNewState = PSM_PE_SNK_READY;
                    }
                    else
                    {
                        pdInstance->psmNewState = PSM_PE_SRC_READY;
                    }
                }
                else
                {
                    PD_PsmTransitionOnMsgSendError(pdInstance, PSM_CHECK_ASYNC_RX, PSM_SEND_SOFT_RESET);
                }
                break;
            }

            case PE_Security_Response_Received: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_SECURITY_RESPONSE_RECEIVED, &pdInstance->commandExtParamCallback, 0);
                if (pdInstance->curPowerRole == kPD_PowerRoleSink)
                {
                    pdInstance->psmNewState = PSM_PE_SNK_READY;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_SRC_READY;
                }
                break;
#endif

            case PE_SRC_Send_Source_Alert: /* C */
            case PE_SNK_Send_Sink_Alert:   /* C */
                if (pdInstance->psmCurState == PE_SNK_Send_Sink_Alert)
                {
                    inCase32Tmp = PSM_PE_SNK_READY;
                }
                else
                {
                    inCase32Tmp = PSM_PE_SRC_READY;
                }

                if (PD_MsgSendDataTransition(pdInstance, kPD_MsgAlert, 1, &pdInstance->alertADO,
                                             (pd_psm_state_t)inCase32Tmp, prevState, PSM_SEND_SOFT_RESET))
                {
                    pdInstance->alertWaitReply = 1u;
                    PD_DpmAppCallback(pdInstance, PD_DPM_SEND_ALERT_SUCCESS, NULL, 1);
                }
                else
                {
                    inCase32Tmp = kCommandResult_Error;
                    PD_DpmAppCallback(pdInstance, PD_DPM_SEND_ALERT_FAIL, &inCase32Tmp, 1);
                }
                break;

            case PE_SNK_Source_Alert_Received: /* C */
            case PE_SRC_Sink_Alert_Received:   /* C */
                if (pdInstance->psmCurState == PE_SNK_Source_Alert_Received)
                {
                    pdInstance->psmNewState = PSM_PE_SNK_READY;
                }
                else
                {
                    pdInstance->psmNewState = PSM_PE_SRC_READY;
                }
                break;
#endif

#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            case PE_FRS_SRC_SNK_CC_Signal: /* C */
            {
                PD_MsgReceive(pdInstance);
                break;
            }

            case PE_FRS_SRC_SNK_Evaluate_Swap: /* C */
                PD_DpmAppCallback(pdInstance, PD_DPM_FR_SWAP_REQUEST, &pdInstance->commandEvaluateResult, 0);
                if (pdInstance->commandEvaluateResult == kCommandResult_Accept)
                {
                    pdInstance->psmNewState = PE_FRS_SRC_SNK_Accept_Swap;
                }
                else
                {
                    pdInstance->psmNewState = PSM_HARD_RESET;
                }
                break;

            case PE_FRS_SRC_SNK_Accept_Swap: /* C */
                PD_MsgSendControlTransition(pdInstance, kPD_MsgAccept, PE_FRS_SRC_SNK_Transition_to_off, PSM_HARD_RESET,
                                            PSM_HARD_RESET);
                break;

            case PE_FRS_SRC_SNK_Transition_to_off: /* C */
                break;

            case PE_FRS_SRC_SNK_Assert_Rd: /* C */
                pdInstance->curPowerRole = kPD_PowerRoleSink;
                PD_ConnectSetPRSwapRole(pdInstance, pdInstance->curPowerRole);
                pdInstance->psmNewState = PE_FRS_SRC_SNK_Wait_Source_on;
                break;

            case PE_FRS_SRC_SNK_Wait_Source_on: /* C */
                PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgPsRdy, PE_PSM_STATE_NO_CHANGE,
                                                PSM_EXIT_TO_ERROR_RECOVERY, PSM_EXIT_TO_ERROR_RECOVERY))
                {
                    PD_TimerStart(pdInstance, tPSSourceOnTimer, T_PS_SOURCE_ON);
                }
                break;

            case PE_FRS_SNK_SRC_Send_Swap: /* C */
                inCase32Tmp = 0;
                pdInstance->fr5VOpened = 0;
                PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &inCase32Tmp);
                if ((pdInstance->fr5VOpened == 0) && PD_DpmCheckLessOrEqualVsafe5v(pdInstance))
                {
                    pdInstance->callbackFns->PD_SrcTurnOnTypeCVbus(pdInstance->callbackParam, kVbusPower_InFRSwap);
                    pdInstance->fr5VOpened = 1;
                }
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgFrSwap, PE_PSM_STATE_NO_CHANGE,
                                                PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY,
                                                PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY))
                {
                    PD_PsmStartCommand(pdInstance, PD_DPM_FAST_ROLE_SWAP, 0);
                    PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                }
                else
                {
                    pdInstance->psmNewState = PSM_EXIT_TO_ERROR_RECOVERY_WITH_DELAY;
                }
                break;

            case PE_FRS_SNK_SRC_Transition_to_off: /* C */
                /* uint8_t enable = 0; */
                /* PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &enable); */
                PD_TimerStart(pdInstance, tPSSourceOffTimer, T_PS_SOURCE_OFF);
                break;

            case PE_FRS_SNK_SRC_Vbus_Applied: /* C */
                break;

            case PE_FRS_SNK_SRC_Assert_Rp: /* C */
                pdInstance->curPowerRole = kPD_PowerRoleSource;
                PD_ConnectSetPRSwapRole(pdInstance, pdInstance->curPowerRole);
                pdInstance->psmNewState = PE_FRS_SNK_SRC_Source_on;
                break;

            case PE_FRS_SNK_SRC_Source_on: /* C */
                if (PD_MsgSendControlTransition(pdInstance, kPD_MsgPsRdy, PSM_PE_SRC_STARTUP,
                                                PSM_EXIT_TO_ERROR_RECOVERY, PSM_EXIT_TO_ERROR_RECOVERY))
                {
                    /* pr swap end */
                    PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    /* Cable plug will need a soft reset */
                    pdInstance->psmCablePlugResetNeeded = 1;
#endif
                    pdInstance->enterSrcFromSwap = 1;
                    PD_DpmAppCallback(pdInstance, PD_DPM_FR_SWAP_SUCCESS, NULL, 1);
                }
                break;
#endif

            default:
                break;
        }

        PD_PsmEndCommand(pdInstance, pdInstance->psmCurState);
        PD_PsmSetNormalPower(pdInstance, pdInstance->psmCurState);
    }

    if (didNothingC == 1)
    {
        return kSM_WaitEvent;
    }
    else
    {
        return kSM_Continue;
    }
}

#if ((defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)) || \
    (defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE))

/*! ***************************************************************************
   \brief Common Handler for structured VDM initiator requests
   \param reject_flags will perSopPartner[kPD_MsgSOP].flags on failure
   \retval PSM_IDLE on failure
   \retval org_state on success
   \note Only called from CLP task context
******************************************************************************/
static pd_psm_state_t PD_PsmStructuredVdmInitiatorRequest(pd_instance_t *pdInstance,
                                                          pd_psm_state_t orgState,
                                                          start_of_packet_t sop,
                                                          uint16_t svid,
                                                          uint8_t position,
                                                          pd_vdm_command_t command,
                                                          uint8_t vdo_count,
                                                          uint32_t *vdos)
{
    pd_structured_vdm_header_t vdmHeader;

    vdmHeader.structuredVdmHeaderVal = 0;
    vdmHeader.bitFields.command = command;
    vdmHeader.bitFields.commandType = kVDM_Initiator;
    vdmHeader.bitFields.objPos = position;
    vdmHeader.bitFields.SVID = svid;
    vdmHeader.bitFields.vdmType = 1;
    vdmHeader.bitFields.vdmVersion = PD_CONFIG_STRUCTURED_VDM_VERSION;
    /* Send Discover Modes request */
    if (PD_MsgSendStructuredVDMAndWait(pdInstance, sop, vdmHeader, vdo_count, vdos))
    {
        if (command == kVDM_EnterMode)
        {
            PD_TimerStart(pdInstance, tVDMModeEntryTimer, T_VDM_WAIT_MODE_ENTRY);
        }
        else if (command == kVDM_ExitMode)
        {
            PD_TimerStart(pdInstance, tVDMModeExitTimer, T_VDM_WAIT_MODE_EXIT);
        }
        else
        {
            PD_TimerStart(pdInstance, tVDMResponseTimer, T_VDM_SENDER_RESPONSE);
        }
        /* Event driven -> wait for response packet or timeout */
        return orgState;
    }
    else
    {
        return PSM_IDLE;
    }
}

static uint8_t PD_PsmVdmResponseHandler(pd_instance_t *pdInstance,
                                        uint8_t sop,
                                        uint8_t vdmMsgType,
                                        psm_trigger_info_t *triggerInfo,
                                        tTimer_t timr,
                                        uint8_t ackState,
                                        uint8_t nakState)
{
    uint8_t expected = 0;
    uint8_t newStateTmp = PSM_UNKNOWN;
    if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgType == kPD_MsgVendorDefined) &&
        (triggerInfo->vdmHeader.bitFields.command == vdmMsgType) &&
        (triggerInfo->vdmHeader.bitFields.commandType != kVDM_Initiator) && (triggerInfo->pdMsgSop == sop))
    {
        if ((vdmMsgType == kVDM_DiscoverIdentity) || (vdmMsgType == kVDM_DiscoverSVIDs))
        {
            if (triggerInfo->vdmHeader.bitFields.SVID == PD_STANDARD_ID)
            {
                expected = 1;
            }
        }
        else
        {
            expected = 1;
        }
    }

    if (expected)
    {
        PD_TimerClear(pdInstance, timr);
        triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
        pdInstance->asmVdmReplyMsg = triggerInfo->vdmHeader.bitFields.commandType;
        switch (triggerInfo->vdmHeader.bitFields.commandType)
        {
            case kVDM_ResponderACK:
                newStateTmp = ackState;
                break;

            case kVDM_ResponderBUSY:
                newStateTmp = nakState;
                break;

            case kVDM_ResponderNAK:
                if (vdmMsgType == kVDM_ExitMode)
                {
                    newStateTmp = ackState;
                }
                else
                {
                    newStateTmp = nakState;
                }
                break;

            default:
                newStateTmp = nakState;
                break;
        }
    }
    else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, timr))
    {
        pdInstance->asmVdmReplyMsg = 0xFFu;
        newStateTmp = nakState;
    }
    else
    {
        /* TODO: AMS interrupt process */
    }

    return newStateTmp;
}

/*! ***************************************************************************
   \brief Meet the requirement to terminate the msgSopP and kPD_MsgSOPp communications on
******************************************************************************/
static void PD_PsmSecondaryStateHandlerTerminate(pd_instance_t *pdInstance, uint8_t sop)
{
    int i;

    if (sop == 0xffu)
    {
        for (i = 0; i < 3; i++)
        {
            if (pdInstance->psmSecondaryState[i] != PSM_IDLE)
            {
                pdInstance->psmNewSecondaryState[i] = PSM_IDLE;
            }
        }
    }
    else
    {
        if (pdInstance->psmSecondaryState[sop] != PSM_IDLE)
        {
            pdInstance->psmNewSecondaryState[sop] = PSM_IDLE;
        }
    }
}

/*! ***************************************************************************
   \brief
   \note Only called from CLP task context
******************************************************************************/
static uint8_t Pd_PsmSecondaryStateHandler(pd_instance_t *pdInstance,
                                           uint8_t statIndex,
                                           uint8_t sop,
                                           psm_trigger_info_t *triggerInfo)
{
    uint32_t commandResultCallback;
    uint8_t didNothingSecond = 0;
    pd_psm_state_t secondNewState = PSM_UNKNOWN;
    uint8_t index;

    while (1)
    {
        /* global transition */
        if (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG)
        {
            /* sop has high priority */
            if (sop != kPD_MsgSOP)
            {
                if (
#if defined(PD_CONFIG_CABLE_COMMUNICATION_ENABLE) && (PD_CONFIG_CABLE_COMMUNICATION_ENABLE)
                    (pdInstance->psmNewSecondaryState[statIndex] != PSM_PE_DFP_CBL_SEND_CABLE_RESET) &&
                    (pdInstance->psmSecondaryState[statIndex] != PSM_PE_DFP_CBL_SEND_CABLE_RESET) &&
                    (pdInstance->psmSecondaryState[statIndex] != PSM_PE_DFP_CBL_SEND_SOFT_RESET) &&
#endif
                    ((pdInstance->psmNewSecondaryState[statIndex] != PSM_UNKNOWN) ||
                     (pdInstance->psmSecondaryState[statIndex] != PSM_IDLE)))
                {
                    /* During PD Connection (Explicit Contract): */
                    /* o The DFP can communicate with a Cable Plug, using SOP' Packets or SOP" Packets, at any time it
                     */
                    /* is not engaged in any other SOP Communications. */
                    /* o If SOP Packets are received by the DFP, during SOP' or SOP" Communication, the SOP' or SOP" */
                    /* Communication is immediately terminated (the Cable Plug times out and does not retry) */
                    /* o If the DFP needs to initiate an SOP Communication during an ongoing SOP' or SOP" Communication
                     */
                    /* (e.g.for a Capabilities change) then the SOP' or SOP" Communications will be interrupted. */

                    /* Check for pending message on SOP */
                    /* Primary state machine needs to handle kPD_MsgSOP */
                    if (triggerInfo->pdMsgSop == kPD_MsgSOP)
                    {
                        /* Abort the current action */
                        pdInstance->psmNewSecondaryState[statIndex] = PSM_IDLE;
                        didNothingSecond = 1;
                    }
                }
                /* Allow soft reset to also interrupt with priority over cable resets */
                if (triggerInfo->pdMsgType == kPD_MsgSoftReset)
                {
                    /* Abort the current action */
                    pdInstance->psmNewSecondaryState[statIndex] = PSM_IDLE;
                    didNothingSecond = 1;
                }

                if (didNothingSecond)
                {
                    return didNothingSecond;
                }
            }
        }

        /* global transition for dpm message */
        if ((triggerInfo->triggerEvent == PSM_TRIGGER_DPM_MSG) &&
            (triggerInfo->dpmMsg >= PD_DPM_CONTROL_DISCOVERY_IDENTITY))
        {
            if (((pdInstance->psmNewSecondaryState[statIndex] != PSM_UNKNOWN) ||
                 (pdInstance->psmSecondaryState[statIndex] != PSM_IDLE)) &&
                (pdInstance->structuredVdmCommandParameter.vdmSop == (sop)))
            {
                triggerInfo->triggerEvent = PSM_TRIGGER_NONE;
                PD_PsmCommandFail(pdInstance, triggerInfo->dpmMsg);
            }
        }

        /* State C */
        if (pdInstance->psmNewSecondaryState[statIndex] != PSM_UNKNOWN)
        {
            pdInstance->psmSecondaryState[statIndex] = pdInstance->psmNewSecondaryState[statIndex];
            pdInstance->psmNewSecondaryState[statIndex] = PSM_UNKNOWN;
            secondNewState = pdInstance->psmSecondaryState[statIndex];
            pd_svdm_command_result_t commandVdmResult;
            commandVdmResult.vdmHeader.structuredVdmHeaderVal =
                pdInstance->structuredVdmCommandParameter.vdmHeader.structuredVdmHeaderVal;
            commandVdmResult.vdoSop = triggerInfo->pdMsgSop;

            switch (pdInstance->psmSecondaryState[statIndex])
            {
                case PSM_IDLE: /* C */
                    /* dpm message will callback */
                    break;

                case PSM_PE_SRC_IMPLICIT_CABLE_SOFT_RESET: /* C */
                    if ((PD_MsgSend(pdInstance, kPD_MsgSOPp, kPD_MsgSoftReset, 2, NULL) == kStatus_PD_Success) &&
                        (PD_MsgWaitSendResult(pdInstance)))
                    {
                        PD_TimerStart(pdInstance, tSenderResponseTimer, T_SENDER_RESPONSE);
                    }
                    else
                    {
                        secondNewState = PSM_IDLE;
                    }
                    break;

                case PSM_PE_SRC_VDM_IDENTITY_REQUEST: /* C , only for sopp */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, pdInstance->psmSecondaryState[statIndex], (start_of_packet_t)sop, PD_STANDARD_ID, 0,
                        kVDM_DiscoverIdentity, 0, NULL);
                    break;

                case PSM_PE_SRC_VDM_IDENTITY_ACKED: /* C */
                    pdInstance->psmCableDiscoveried = 1;
                    for (index = 0; index < triggerInfo->pdMsgDataLength; ++index)
                    {
                        pdInstance->psmCableIdentities[index] = triggerInfo->pdMsgDataBuffer[index];
                    }
                    if (((pdInstance->psmCableIdentities[1] & VDM_ID_HEADER_VDO_UFP_CABLE_PLUG_TYPE_MASK) ==
                         VDM_ID_HEADER_VDO_PASSIVE_CABLE_VAL) ||
                        ((pdInstance->psmCableIdentities[1] & VDM_ID_HEADER_VDO_UFP_CABLE_PLUG_TYPE_MASK) ==
                         VDM_ID_HEADER_VDO_ACTIVE_CABLE_VAL))
                    {
                        if (triggerInfo->pdMsgDataLength > 4)
                        {
                            switch (pdInstance->psmCableIdentities[4] & VDM_CABLE_VDO_CURRENT_CAPABILITY)
                            {
                                case 0x01u:
                                    pdInstance->dpmCableMaxCurrent = 3000 / 10; /* 3A */
                                    break;
                                case 0x02u:
                                    pdInstance->dpmCableMaxCurrent = 5000 / 10; /* 5A */
                                    break;
                                default:
                                    pdInstance->dpmCableMaxCurrent = 3000 / 10; /* 3A */
                                    break;
                            }
                        }
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_SRC_VDM_IDENTITY_NAKED: /* C */
                    for (index = 0; index < 7; ++index)
                    {
                        pdInstance->psmCableIdentities[index] = 0;
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_UFP_VDM_IDENTITY_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, pdInstance->psmSecondaryState[statIndex], (start_of_packet_t)sop, PD_STANDARD_ID, 0,
                        kVDM_DiscoverIdentity, 0, NULL);
                    break;

                case PSM_PE_DFP_UFP_VDM_IDENTITY_ACKED: /* C */
                    commandVdmResult.vdmHeader.structuredVdmHeaderVal = ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                    commandVdmResult.vdmCommand = kVDM_DiscoverIdentity;
                    commandVdmResult.vdoData = (uint32_t *)(triggerInfo->pdMsgDataBuffer + 4);
                    commandVdmResult.vdoCount = triggerInfo->pdMsgDataLength - 1;
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_UFP_VDM_IDENTITY_NAKED: /* C */
                    commandVdmResult.vdmCommand = kVDM_DiscoverIdentity;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    if (pdInstance->asmVdmReplyMsg == kVDM_ResponderNAK)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMNAK;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else
                    {
                        PD_TimerStart(pdInstance, tVDMBusyTimer, T_VDM_BUSY);
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMBUSY;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_SVIDS_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, pdInstance->psmSecondaryState[statIndex], (start_of_packet_t)sop, PD_STANDARD_ID, 0,
                        kVDM_DiscoverSVIDs, 0, NULL);
                    break;

                case PSM_PE_DFP_VDM_SVIDS_ACKED: /* C */
                    commandVdmResult.vdmHeader.structuredVdmHeaderVal = ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                    commandVdmResult.vdmCommand = kVDM_DiscoverSVIDs;
                    commandVdmResult.vdoData = (uint32_t *)(triggerInfo->pdMsgDataBuffer + 4);
                    commandVdmResult.vdoCount = triggerInfo->pdMsgDataLength - 1;
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_SVIDS_NAKED: /* C */
                    commandVdmResult.vdmCommand = kVDM_DiscoverSVIDs;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    if (pdInstance->asmVdmReplyMsg == kVDM_ResponderNAK)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMNAK;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else if (pdInstance->asmVdmReplyMsg == kVDM_ResponderBUSY)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMBUSY;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else
                    {
                        /* time out or receive non-right message */
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_MODES_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, pdInstance->psmSecondaryState[statIndex], (start_of_packet_t)sop,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID, 0, kVDM_DiscoverModes, 0,
                        NULL);
                    break;

                case PSM_PE_DFP_VDM_MODES_ACKED: /* C */
                    commandVdmResult.vdmHeader.structuredVdmHeaderVal = ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                    commandVdmResult.vdmCommand = kVDM_DiscoverModes;
                    commandVdmResult.vdoData = (uint32_t *)(triggerInfo->pdMsgDataBuffer + 4);
                    commandVdmResult.vdoCount = triggerInfo->pdMsgDataLength - 1;
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_MODES_NAKED: /* C */
                    commandVdmResult.vdmCommand = kVDM_DiscoverModes;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    if (pdInstance->asmVdmReplyMsg == kVDM_ResponderNAK)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMNAK;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else if (pdInstance->asmVdmReplyMsg == kVDM_ResponderBUSY)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMBUSY;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else
                    {
                        /* time out or receive non-right message */
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_MODE_ENTRY_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, pdInstance->psmSecondaryState[statIndex], (start_of_packet_t)sop,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos, kVDM_EnterMode,
                        (pdInstance->structuredVdmCommandParameter.vdoCount) ? 1 : 0,
                        (pdInstance->structuredVdmCommandParameter.vdoCount) ?
                            (pdInstance->structuredVdmCommandParameter.vdoData) :
                            NULL);
                    break;

                case PSM_PE_DFP_VDM_MODE_ENTRY_ACKED: /* C */
                    commandVdmResult.vdmHeader.structuredVdmHeaderVal = ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                    commandVdmResult.vdmCommand = kVDM_EnterMode;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    pdInstance->psmVdmActiveModeValidMask |=
                        (uint8_t)(0x01u << ((((uint32_t *)(triggerInfo->pdMsgDataBuffer))[0] >> 8) & 0x07u));
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_MODE_ENTRY_NAKED: /* C */
                    commandVdmResult.vdmCommand = kVDM_EnterMode;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    if (pdInstance->asmVdmReplyMsg == kVDM_ResponderNAK)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMNAK;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else if (pdInstance->asmVdmReplyMsg == kVDM_ResponderBUSY)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMBUSY;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else
                    {
                        /* time out or receive non-right message */
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_MODE_EXIT_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, pdInstance->psmSecondaryState[statIndex], (start_of_packet_t)sop,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos, kVDM_ExitMode, 0, NULL);
                    break;

                case PSM_PE_DFP_VDM_MODE_EXIT_ACKED: /* C */
                    commandVdmResult.vdmCommand = kVDM_ExitMode;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    pdInstance->psmVdmActiveModeValidMask &=
                        ~(uint32_t)(0x01 << pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos);
                    if (pdInstance->asmVdmReplyMsg == kVDM_ResponderACK)
                    {
                        commandVdmResult.vdmHeader.structuredVdmHeaderVal =
                            ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    }
                    else
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMNAK;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_DFP_VDM_MODE_EXIT_HARD_RESET: /* C */
                    commandVdmResult.vdmCommand = kVDM_ExitMode;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    if (pdInstance->asmVdmReplyMsg == kVDM_ResponderBUSY)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMBUSY;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else
                    {
                        /* time out or receive non-right message */
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    secondNewState = PSM_IDLE;
                    pdInstance->psmNewState = PSM_HARD_RESET;
                    /* TODO: it may be CABLE. */
                    break;

                case PSM_PE_DFP_VDM_ATTENTION_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, pdInstance->psmSecondaryState[statIndex], (start_of_packet_t)sop,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos, kVDM_Attention,
                        (pdInstance->structuredVdmCommandParameter.vdoCount) ? 1 : 0,
                        (pdInstance->structuredVdmCommandParameter.vdoCount) ?
                            (pdInstance->structuredVdmCommandParameter.vdoData) :
                            NULL);

                    commandVdmResult.vdmCommand = kVDM_Attention;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    if (secondNewState == PSM_IDLE)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else
                    {
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    }
                    break;

                case PSM_PE_DFP_CBL_SEND_CABLE_RESET: /* C */
                    PD_PhyControl(pdInstance, PD_PHY_SEND_CABLE_RESET, NULL);
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PE_VENDOR_STRUCTURED_VDM_REQUEST: /* C */
                    secondNewState = PD_PsmStructuredVdmInitiatorRequest(
                        pdInstance, pdInstance->psmSecondaryState[statIndex], (start_of_packet_t)sop,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID,
                        pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos,
                        (pd_vdm_command_t)pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command,
                        pdInstance->structuredVdmCommandParameter.vdoCount,
                        pdInstance->structuredVdmCommandParameter.vdoData);

                    if (!(pdInstance->structuredVdmCommandParameter.vendorVDMNeedResponse))
                    {
                        commandVdmResult.vdmCommand =
                            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command;
                        commandVdmResult.vdoData = NULL;
                        commandVdmResult.vdoCount = 0;
                        if (secondNewState == pdInstance->psmSecondaryState[statIndex])
                        {
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, NULL, 1);
                        }
                        else
                        {
                            commandVdmResult.vdmCommandResult = kCommandResult_Error;
                            PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                        }
                        secondNewState = PSM_IDLE;
                    }
                    break;

                case PSM_PE_VENDOR_STRUCTURED_VDM_ACKED: /* C */
                {
                    commandVdmResult.vdmHeader.structuredVdmHeaderVal = ((uint32_t *)triggerInfo->pdMsgDataBuffer)[0];
                    commandVdmResult.vdmCommand = commandVdmResult.vdmHeader.bitFields.command;
                    commandVdmResult.vdoData = (uint32_t *)(triggerInfo->pdMsgDataBuffer + 4);
                    commandVdmResult.vdoCount = triggerInfo->pdMsgDataLength - 1;
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_SUCCESS, &commandVdmResult, 1);
                    secondNewState = PSM_IDLE;
                    break;
                }

                case PSM_PE_VENDOR_STRUCTURED_VDM_NAKED: /* C */
                    commandVdmResult.vdmCommand = pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command;
                    commandVdmResult.vdoData = NULL;
                    commandVdmResult.vdoCount = 0;
                    if (pdInstance->asmVdmReplyMsg == kVDM_ResponderNAK)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMNAK;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else if (pdInstance->asmVdmReplyMsg == kVDM_ResponderBUSY)
                    {
                        commandVdmResult.vdmCommandResult = kCommandResult_VDMBUSY;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    else
                    {
                        /* time out or receive non-right message */
                        commandVdmResult.vdmCommandResult = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_FAIL, &commandVdmResult, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;

                case PSM_PD_SEND_UNSTRUCTURED_VDM: /* C */
                {
                    pd_status_t status = PD_MsgSendUnstructuredVDM(
                        pdInstance, (start_of_packet_t)pdInstance->unstructuredVdmCommandParameter.vdmSop,
                        (uint8_t *)(pdInstance->unstructuredVdmCommandParameter.vdmHeaderAndVDOsData),
                        pdInstance->unstructuredVdmCommandParameter.vdmHeaderAndVDOsCount * 4 + 2);
                    if (status == kStatus_PD_Success)
                    {
                        PD_DpmAppCallback(pdInstance, PD_DPM_SEND_UNSTRUCTURED_VDM_SUCCESS, NULL, 1);
                    }
                    else
                    {
                        commandResultCallback = kCommandResult_Error;
                        PD_DpmAppCallback(pdInstance, PD_DPM_SEND_UNSTRUCTURED_VDM_FAIL, &commandResultCallback, 1);
                    }
                    secondNewState = PSM_IDLE;
                    break;
                }

                default:
                    break;
            }

            PD_PsmEndVdmCommand(pdInstance, pdInstance->psmSecondaryState[statIndex]);

            if (secondNewState != pdInstance->psmSecondaryState[statIndex])
            {
                pdInstance->psmNewSecondaryState[statIndex] = secondNewState;
                continue;
            }
        }

        /* state B */
        secondNewState = pdInstance->psmSecondaryState[statIndex];
        switch (pdInstance->psmSecondaryState[statIndex])
        {
            case PSM_IDLE: /* B */
                if (triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG)
                {
                    PD_PsmVdmIdleProcessPdMessage(pdInstance, statIndex, triggerInfo);
                }
                else if (triggerInfo->triggerEvent == PSM_TRIGGER_DPM_MSG)
                {
                    secondNewState = (pd_psm_state_t)PD_PsmVdmIdleProcessDpmMessage(pdInstance, statIndex, triggerInfo);
                }
                else
                {
                }

                if (pdInstance->vdmExitReceived[sop])
                {
                    pd_svdm_command_request_t commandVdmRequest;
                    pd_structured_vdm_header_t reponseVdmHeader;
                    commandVdmRequest.vdmHeader.structuredVdmHeaderVal = pdInstance->vdmExitReceived[sop];
                    pdInstance->vdmExitReceived[sop] = 0;
                    commandVdmRequest.vdoSop = pdInstance->vdmExitReceivedSOP[sop];
                    PD_DpmAppCallback(pdInstance, PD_DPM_STRUCTURED_VDM_REQUEST, &commandVdmRequest, 0);

                    reponseVdmHeader.structuredVdmHeaderVal = commandVdmRequest.vdmHeader.structuredVdmHeaderVal;
                    reponseVdmHeader.bitFields.objPos = commandVdmRequest.vdoSop;
                    if (commandVdmRequest.requestResultStatus == kCommandResult_VDMACK)
                    {
                        reponseVdmHeader.bitFields.commandType = kVDM_ResponderACK;
                        PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader,
                                                       commandVdmRequest.vdoCount,
                                                       (uint32_t *)commandVdmRequest.vdoData);
                    }
                    else
                    {
                        reponseVdmHeader.bitFields.commandType = kVDM_ResponderNAK;
                        PD_MsgSendStructuredVDMAndWait(pdInstance, (start_of_packet_t)statIndex, reponseVdmHeader, 0,
                                                       NULL);
                    }
                }
                break;

            case PSM_PE_DFP_UFP_VDM_VDM_BUSY_WAIT: /* B */
                if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tVDMBusyTimer))
                {
                    secondNewState = (pd_psm_state_t)pdInstance->psmVDMBusyWaitDpmMsg;
                }
                break;

            case PSM_PE_SRC_IMPLICIT_CABLE_SOFT_RESET: /* B */
                if ((triggerInfo->triggerEvent == PSM_TRIGGER_PD_MSG) && (triggerInfo->pdMsgSop == sop) &&
                    (triggerInfo->pdMsgType == kPD_MsgAccept))
                {
                    PD_TimerClear(pdInstance, tSenderResponseTimer);
                    secondNewState = PSM_PE_SRC_VDM_IDENTITY_REQUEST;
                }
                else if (PD_TimerCheckInvalidOrTimeOut(pdInstance, tSenderResponseTimer))
                {
                    secondNewState = PSM_IDLE;
                }
                else
                {
                    didNothingSecond = 1;
                }
                break;

            case PSM_PE_SRC_VDM_IDENTITY_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_DiscoverIdentity, triggerInfo, tVDMResponseTimer,
                    PSM_PE_SRC_VDM_IDENTITY_ACKED, PSM_PE_SRC_VDM_IDENTITY_NAKED);
                break;

            case PSM_PE_DFP_UFP_VDM_IDENTITY_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_DiscoverIdentity, triggerInfo, tVDMResponseTimer,
                    PSM_PE_DFP_UFP_VDM_IDENTITY_ACKED, PSM_PE_DFP_UFP_VDM_IDENTITY_NAKED);
                break;

            case PSM_PE_DFP_VDM_SVIDS_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_DiscoverSVIDs, triggerInfo, tVDMResponseTimer, PSM_PE_DFP_VDM_SVIDS_ACKED,
                    PSM_PE_DFP_VDM_SVIDS_NAKED);
                break;

            case PSM_PE_DFP_VDM_MODES_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_DiscoverModes, triggerInfo, tVDMResponseTimer, PSM_PE_DFP_VDM_MODES_ACKED,
                    PSM_PE_DFP_VDM_MODES_NAKED);
                break;

            case PSM_PE_DFP_VDM_MODE_ENTRY_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_EnterMode, triggerInfo, tVDMModeEntryTimer, PSM_PE_DFP_VDM_MODE_ENTRY_ACKED,
                    PSM_PE_DFP_VDM_MODE_ENTRY_NAKED);
                break;

            /* The Responder shall not return a BUSY acknowledgement and shall
               only return a NAK acknowledgement to a request not containing an Active Mode (i.e. Invalid object
               position). An
               Initiator which fails to receive an ACK within tVDMWaitModeExit or receives a NAK or BUSY response shall
               exit its
               Active Mode. */
            case PSM_PE_DFP_VDM_MODE_EXIT_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, kVDM_ExitMode, triggerInfo, tVDMModeExitTimer, PSM_PE_DFP_VDM_MODE_EXIT_ACKED,
                    PSM_PE_DFP_VDM_MODE_EXIT_HARD_RESET);
                break;

            case PSM_PE_DFP_VDM_ATTENTION_REQUEST: /* B */
                secondNewState = PSM_IDLE;
                break;

            case PSM_PE_VENDOR_STRUCTURED_VDM_REQUEST: /* B */
                secondNewState = (pd_psm_state_t)PD_PsmVdmResponseHandler(
                    pdInstance, sop, pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command, triggerInfo,
                    tVDMResponseTimer, PSM_PE_VENDOR_STRUCTURED_VDM_ACKED, PSM_PE_VENDOR_STRUCTURED_VDM_NAKED);
                break;

            default:
                break;
        }

        if ((secondNewState != PSM_UNKNOWN) && (secondNewState != pdInstance->psmSecondaryState[statIndex]))
        {
            pdInstance->psmNewSecondaryState[statIndex] = secondNewState;
            continue;
        }

        return didNothingSecond;
    }
}

#endif

static void PD_PsmConnect(pd_instance_t *pdInstance)
{
    /* Initialise static flags */
    PD_PsmReset(pdInstance);

    /* Enable Msg hardware at SNK/SRC STARTUP */

    pdInstance->psmCurState = PSM_UNKNOWN;

    PD_MsgSetPortRole(pdInstance, pdInstance->curPowerRole, pdInstance->curDataRole);

    pdInstance->psmPresentlyPdConnected = 0;
    pdInstance->psmPreviouslyPdConnected = 0;

    pdInstance->psmExplicitContractExisted = 0;
    pdInstance->unchunkedFeature = 0;
    pdInstance->commandProcessing = 0;
    pdInstance->dpmMsgBits = 0;
    pdInstance->waitTime = PD_WAIT_EVENT_TIME;
    pdInstance->asmHardResetSnkProcessing = 0;
    pdInstance->psmHardResetNeedsVSafe0V = 0;
    pdInstance->hardResetReceived = 0;
}

static void PD_PsmDisconnect(pd_instance_t *pdInstance)
{
    uint8_t uint8Tmp;

    PD_TimerCancelAllTimers(pdInstance, tSenderResponseTimer, _tMaxPSMTimer);

    uint8Tmp = 0;
    PD_PhyControl(pdInstance, PD_PHY_CONTROL_FR_SWAP, &uint8Tmp);

    pdInstance->psmPresentlyPdConnected = 0;
    pdInstance->psmPreviouslyPdConnected = 0;
#if defined(PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
    pdInstance->psmPresentlyVconnSource = kPD_VconnNone;
#endif
    pdInstance->psmCurState = PSM_UNKNOWN;
    pdInstance->psmNewState = PSM_UNKNOWN;
    pdInstance->unchunkedFeature = 0;
    pdInstance->commandProcessing = 0;
    pdInstance->dpmMsgBits = 0;
    pdInstance->waitTime = PD_WAIT_EVENT_TIME;

    /* disconnect message layer */
    PD_MsgDisable(pdInstance);
}

void PD_DpmDisconnect(pd_instance_t *pdInstance)
{
    PD_TimerCancelAllTimers(pdInstance, _tStartTimer, _tMaxDpmTimer);
    PD_PsmDisconnect(pdInstance);
}

void PD_DpmConnect(pd_instance_t *pdInstance)
{
    PD_TimerCancelAllTimers(pdInstance, _tStartTimer, _tMaxDpmTimer);
    pdInstance->dpmCableMaxCurrent = 3000 / 10; /* 3A */
    /* Source should provide VSafe5V by default */
    /* do in the connect callback */

    pdInstance->dpmStateMachine = 0;
    PD_PsmConnect(pdInstance);
}

uint8_t PD_PsmStateMachine(pd_instance_t *pdInstance)
{
    uint32_t infoVal;
    uint8_t returnVal = kSM_Continue;

    switch (pdInstance->dpmStateMachine)
    {
        case 0:
            /* wait vbus charge */
            /* Neither Source nor Sink should enter PSM before VSafe5V is available */
            PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &infoVal);
            if (infoVal & PD_VBUS_POWER_STATE_VBUS_MASK)
            {
                pdInstance->dpmStateMachine = 1;
            }
            break;

        case 1:
            pdInstance->psmNewState = (pdInstance->psmCurState == PSM_UNKNOWN) ? PSM_IDLE : pdInstance->psmCurState;

            /* Did we do anything in Step A or B or C */
            /* Step A: common process */
            /* Step B: state is not changed */
            /* Step C: change to new state */

            do
            {
                returnVal = PD_PsmEnterState(pdInstance);
                if ((returnVal != kSM_ErrorRecovery) && (returnVal != kSM_Detach))
                {
                    if (PD_ConnectCheck(pdInstance) == (uint8_t)kConnectState_Disconnected)
                    {
                        returnVal = kSM_Detach;
                    }
                }
                if ((returnVal != kSM_ErrorRecovery) && (returnVal != kSM_Detach))
                {
                    returnVal = PD_PsmProcessState(pdInstance);
                    if ((returnVal != kSM_ErrorRecovery) && (returnVal != kSM_Detach))
                    {
                        if (PD_ConnectCheck(pdInstance) == (uint8_t)kConnectState_Disconnected)
                        {
                            returnVal = kSM_Detach;
                        }
                    }
                }
            } while (((pdInstance->psmCurState != pdInstance->psmNewState) || (returnVal == kSM_Continue)) &&
                     (returnVal != kSM_ErrorRecovery) && (returnVal != kSM_Detach));

            if ((returnVal == kSM_ErrorRecovery) || (returnVal == kSM_Detach))
            {
                PD_TimerCancelAllTimers(pdInstance, _tStartTimer, _tMaxDpmTimer);
            }
            break;

        default:
            break;
    }

    return returnVal;
}

void PD_DpmDischargeVbus(pd_instance_t *pdInstance, uint8_t enable)
{
    PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VBUS, &enable);
}

#ifdef USBPD_ENABLE_VCONN_DISCHARGE
void PD_DpmDischargeVconn(pd_instance_t *pdInstance, uint8_t enable)
{
    PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VCONN, &enable);
}
#endif

void PD_DpmSetVconn(pd_instance_t *pdInstance, uint8_t enable)
{
    if (pdInstance->pdPowerPortConfig->vconnSupported)
    {
        PD_PhyControl(pdInstance, PD_PHY_CONTROL_VCONN, &enable);
    }
}

/*! **************************************************************************
    \brief The PSM has accepted a new power contract as source.
    \note  We need to wait tSnkTransition before the DPM can act on this new request
    (USBPD requirement is to wait tSnkTransition before DPM is informed)
    \note Only called from CLP task context
 ******************************************************************************/
void PD_DpmSrcTransitionToNewpwr(pd_instance_t *pdInstance, pd_rdo_t rdo, uint8_t gotoMin)
{
    PD_TimerStart(pdInstance, tDelayTimer, T_SRC_TRANSITION);
    while (!PD_TimerCheckValidTimeOut(pdInstance, tDelayTimer))
    {
        ;
    }

    if (gotoMin)
    {
        pdInstance->callbackFns->PD_SrcGotoMinReducePower(pdInstance->callbackParam);
    }
    else
    {
        pdInstance->callbackFns->PD_SrcTurnOnRequestVbus(pdInstance->callbackParam, rdo);
    }
}

/* internal function */
uint8_t PD_DpmCheckVbus(pd_instance_t *pdInstance)
{
    uint8_t powerState;
    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &powerState);
    if ((powerState & PD_VBUS_POWER_STATE_VSAFE5V_MASK) || (powerState & PD_VBUS_POWER_STATE_VBUS_MASK))
    {
        return 1;
    }
    return 0;
}

uint8_t PD_DpmCheckLessOrEqualVsafe5v(pd_instance_t *pdInstance)
{
    uint8_t powerState;
    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &powerState);
    if (powerState & PD_VBUS_POWER_STATE_VSAFE5V_MASK)
    {
        return 1;
    }
    if (powerState & PD_VBUS_POWER_STATE_VSAFE0V_MASK)
    {
        return 1;
    }
    if ((!(powerState & PD_VBUS_POWER_STATE_VSAFE5V_MASK)) && (!(powerState & PD_VBUS_POWER_STATE_VBUS_MASK)))
    {
        return 1;
    }

    return 0;
}

uint8_t PD_DpmCheckVsafe0V(pd_instance_t *pdInstance)
{
    uint8_t powerState;

    PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &powerState);
    return (powerState & PD_VBUS_POWER_STATE_VSAFE0V_MASK);
}

uint8_t PD_DpmGetMsg(pd_instance_t *pdInstance)
{
    uint8_t reVal = 0;
    uint8_t commandIndex;

    if (pdInstance->dpmMsgBits)
    {
        for (commandIndex = PD_DPM_CONTROL_POWER_NEGOTIATION; commandIndex < PD_DPM_CONTROL_COUNT; ++commandIndex)
        {
            if (pdInstance->dpmMsgBits & (0x01u << commandIndex))
            {
                reVal = commandIndex;
                break;
            }
        }
    }

    return reVal;
}

void PD_DpmClearMsg(pd_instance_t *pdInstance, pd_command_t id)
{
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if (pdInstance->dpmMsgBits & (0x01u << id))
    {
        pdInstance->dpmMsgBits &= (~(0x01u << id));
    }
    USB_OSA_EXIT_CRITICAL();
}

void PD_DpmSendMsg(pd_handle pdHandle, uint8_t id)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    pdInstance->dpmMsgBits |= (0x00000001u << id);
    USB_OSA_EXIT_CRITICAL();
    USB_OsaEventSet(pdInstance->taskEventHandle, PD_TASK_EVENT_DPM_MSG);
}

void PD_StackStateMachine(pd_instance_t *pdInstance)
{
    uint32_t taskEventSet = 0;
    TypeCState_t connectStatus;
    uint8_t smState = 0;
    uint8_t connected;

    if (pdInstance->initializeLabel == 0)
    {
        pdInstance->initializeLabel = 1;
        pdInstance->isConnected = 0;
        /* We want to override any existing MTP-based connection */
        PD_ConnectInitRole(pdInstance, 0);
        pdInstance->connectedResult = PD_ConnectGetStateMachine(pdInstance);
        PD_DpmSetVconn(pdInstance, 0);
    }
    USB_OsaEventWait(pdInstance->taskEventHandle, 0xffffu, 0, pdInstance->waitTime, &taskEventSet);
/* if waiting for ever and no event */
#if defined USB_STACK_BM
    if ((pdInstance->waitTime == 0) && (taskEventSet == 0u))
    {
        return;
    }
#endif
    pdInstance->waitTime = PD_WAIT_EVENT_TIME;
    PD_PortTaskEventProcess(pdInstance, taskEventSet);
    if (taskEventSet & PD_TASK_EVENT_TYPEC_STATE_PROCESS)
    {
        USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_TYPEC_STATE_PROCESS);
    }

    /* Process Type-C state change by PD stack */
    connectStatus = PD_ConnectGetStateMachine(pdInstance);
    if (connectStatus != pdInstance->connectedResult)
    {
        pdInstance->connectedResult = connectStatus;
    }

    /* Process Type-C state change by Type-C state machine */
    connected = PD_ConnectCheck(pdInstance);
    connectStatus = PD_ConnectGetStateMachine(pdInstance);
    if (connected != kConnectState_NotStable)
    {
        if (connected == kConnectState_Connected)
        {
            connected = 1;
        }
        else
        {
            connected = 0;
        }
        /* connect state change */
        if ((connected != pdInstance->isConnected) ||
            ((pdInstance->isConnected) && (connectStatus != pdInstance->connectedResult)))
        {
            /* change from disconnect to connect */
            if ((!(pdInstance->isConnected)) && (connected))
            {
                switch (connectStatus)
                {
                    case TYPEC_ATTACHED_SRC:
                    case TYPEC_ATTACHED_SNK:
                        pdInstance->pendingSOP = kPD_MsgSOPMask;
                        if (connectStatus == TYPEC_ATTACHED_SRC)
                        {
                            pdInstance->curPowerRole = kPD_PowerRoleSource;
                            pdInstance->curDataRole = kPD_DataRoleDFP;
                            pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
                        }
                        else
                        {
                            pdInstance->curPowerRole = kPD_PowerRoleSink;
                            pdInstance->curDataRole = kPD_DataRoleUFP;
                            pdInstance->psmPresentlyVconnSource = kPD_NotVconnSource;
                        }
                        break;

                    case TYPEC_AUDIO_ACCESSORY:
                    case TYPEC_UNORIENTED_DEBUG_ACCESSORY_SRC:
                    case TYPEC_DEBUG_ACCESSORY_SNK:
                    case TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK:
                    case TYPEC_POWERED_ACCESSORY:
                        /* do nothing */
                        /* pdInstance->curPowerRole = kPD_PowerRoleSource; */
                        /* pdInstance->curDataRole = kPD_DataRoleDFP; */
                        break;

#if (defined PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
                    case TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC:
                        if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_NONE)
                        {
                            /* Wait for disconnection */
                        }
                        else
                        {
                            pdInstance->curPowerRole = kPD_PowerRoleSource;
                            pdInstance->curDataRole = kPD_DataRoleDFP;
                            pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
                        }
                        break;
#endif
                    default:
                        break;
                }

                PD_DpmAppCallback(pdInstance, PD_CONNECTED, NULL, 0);
                PD_DpmConnect(pdInstance);
            }
            /* still connect, but role change. For example: Try.SRC */
            else if ((pdInstance->isConnected) && (connected) && (connectStatus != pdInstance->connectedResult))
            {
                switch (connectStatus)
                {
                    case TYPEC_ATTACHED_SRC:
                    case TYPEC_ATTACHED_SNK:
                        pdInstance->pendingSOP = kPD_MsgSOPMask;
                        if (connectStatus == TYPEC_ATTACHED_SRC)
                        {
                            pdInstance->curPowerRole = kPD_PowerRoleSource;
                            pdInstance->curDataRole = kPD_DataRoleDFP;
                            pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
                        }
                        else
                        {
                            pdInstance->curPowerRole = kPD_PowerRoleSink;
                            pdInstance->curDataRole = kPD_DataRoleUFP;
                            pdInstance->psmPresentlyVconnSource = kPD_NotVconnSource;
                        }
                        break;

                    case TYPEC_AUDIO_ACCESSORY:
                    case TYPEC_UNORIENTED_DEBUG_ACCESSORY_SRC:
                    case TYPEC_DEBUG_ACCESSORY_SNK:
                    case TYPEC_ORIENTED_DEBUG_ACCESSORY_SNK:
                    case TYPEC_POWERED_ACCESSORY:
                        break;

#if (defined PD_CONFIG_DEBUG_ACCESSORY_SUPPORT) && (PD_CONFIG_DEBUG_ACCESSORY_SUPPORT)
                    case TYPEC_ORIENTED_DEBUG_ACCESSORY_SRC:
                        if (PD_CONFIG_DEBUG_ACCESSORY_ROLE == CONFIG_DEBUG_ACCESSORY_NONE)
                        {
                            /* Wait for disconnection */
                        }
                        else
                        {
                            pdInstance->curPowerRole = kPD_PowerRoleSource;
                            pdInstance->curDataRole = kPD_DataRoleDFP;
                            pdInstance->psmPresentlyVconnSource = kPD_IsVconnSource;
                        }
                        break;
#endif
                    default:
                        break;
                }
                PD_DpmAppCallback(pdInstance, PD_CONNECT_ROLE_CHANGE, NULL, 0);
            }
            /* change from connect to disconnect */
            else if ((pdInstance->isConnected) && (!(connected)))
            {
                PD_DpmAppCallback(pdInstance, PD_DISCONNECTED, NULL, 0);
                PD_DpmDisconnect(pdInstance);
                pdInstance->initializeLabel = 0;
            }
            else
            {
            }

            pdInstance->isConnected = connected;
        }
        else if (pdInstance->isConnected) /* connect result stable and connected */
        {
            pdInstance->noConnectButVBusExit = 0;
            smState = PD_PsmStateMachine(pdInstance);

            /* Force a partner disconnect if the PSM has requested a error recovery */
            if ((smState == kSM_ErrorRecovery) || (smState == kSM_Detach))
            {
                pdInstance->isConnected = 0;
                PD_DpmAppCallback(pdInstance, PD_DISCONNECTED, NULL, 0);
                PD_DpmDisconnect(pdInstance);
                if (smState == kSM_ErrorRecovery)
                {
                    PD_ConnectInitRole(pdInstance, 1);
                }
                pdInstance->initializeLabel = 0;
                pdInstance->waitTime = PD_WAIT_EVENT_TIME;
            }
            else if (smState == kSM_Continue)
            {
                pdInstance->waitTime = PD_WAIT_EVENT_TIME;
            }
            else
            {
                pdInstance->waitTime = 0; /* wait for ever */
            }
        }
        else
        {
        }

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        /* check FR_Swap signal */
        if (USB_OsaEventCheck(pdInstance->taskEventHandle, PD_TASK_EVENT_FR_SWAP_SINGAL, NULL) ==
            kStatus_USB_OSA_Success)
        {
            if (!(pdInstance->isConnected))
            {
                USB_OsaEventClear(pdInstance->taskEventHandle, PD_TASK_EVENT_FR_SWAP_SINGAL);

                /* when receiving FR_Swap signal, PTN5110 will open source vbus defaultly.  */
                if (pdInstance->pdConfig->phyType == kPD_PhyPTN5110)
                {
                    pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, kVbusPower_Stable);
                }
            }
        }

        /* check unreasonable situation */
        if (!(pdInstance->isConnected))
        {
            PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &taskEventSet);
            if (taskEventSet & (PD_VBUS_POWER_STATE_VSAFE5V_MASK | PD_VBUS_POWER_STATE_VBUS_MASK))
            {
                pdInstance->noConnectButVBusExit++;
                if (pdInstance->callbackFns->PD_ControlVconn != NULL)
                {
                    pdInstance->callbackFns->PD_ControlVconn(pdInstance->callbackParam, 0);
                }
                pdInstance->callbackFns->PD_SrcTurnOffVbus(pdInstance->callbackParam, kVbusPower_Stable);
                if (pdInstance->noConnectButVBusExit >= 100)
                {
                    pd_phy_config_t phyConfig;

                    pdInstance->noConnectButVBusExit = 0;
                    pdInstance->phyInterface->pdPhyDeinit(pdInstance->pdPhyHandle);
                    phyConfig.interface = pdInstance->pdConfig->phyInterface;
                    phyConfig.interfaceParam = pdInstance->pdConfig->interfaceParam;
                    pdInstance->phyInterface->pdPhyInit(pdInstance, &(pdInstance->pdPhyHandle), &phyConfig);
                }
            }
        }
        else
        {
            pdInstance->noConnectButVBusExit = 0;
        }
#endif
    }
    pdInstance->connectedResult = connectStatus;

#if 0
    /* unclear error recovery */
    connectStatus = PD_ConnectGetStateMachine(pdInstance);
    if (connectStatus == PD_ConnectGetInitRoleState(pdInstance))
    {
        pdInstance->typeCStateNeedRecovery++;
    }
    else
    {
        pdInstance->typeCStateNeedRecovery = 0;
    }
    if ((!(pdInstance->isConnected)) && (pdInstance->typeCStateNeedRecovery >= 100))
    {
        pdInstance->typeCStateNeedRecovery = 0;
        PD_ConnectInitRole(pdInstance, 1);
    }
#endif
}

/* change to command and is async */
pd_status_t PD_Command(pd_handle pdHandle, uint32_t command, void *param)
{
    pd_status_t status = kStatus_PD_Success;
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    /* common process */
    switch (command)
    {
#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
        case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
        case PD_DPM_CONTROL_DISCOVERY_SVIDS:
        case PD_DPM_CONTROL_DISCOVERY_MODES:
        {
            pdInstance->structuredVdmCommandParameter = *((pd_svdm_command_param_t *)param);

            pdInstance->structuredVdmCommandParameter.vdoCount = 0;
            pdInstance->structuredVdmCommandParameter.vdoData = NULL;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.commandType = kVDM_Initiator;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.objPos = 0;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.vdmVersion = PD_CONFIG_STRUCTURED_VDM_VERSION;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.vdmType = 1;
            if ((command == PD_DPM_CONTROL_DISCOVERY_IDENTITY) || (command == PD_DPM_CONTROL_DISCOVERY_SVIDS))
            {
                pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.SVID = PD_STANDARD_ID;
            }
            break;
        }

        case PD_DPM_SEND_VENDOR_STRUCTURED_VDM:
        case PD_DPM_CONTROL_ENTER_MODE:
        case PD_DPM_CONTROL_EXIT_MODE:
        case PD_DPM_CONTROL_SEND_ATTENTION:
        {
            pdInstance->structuredVdmCommandParameter = *((pd_svdm_command_param_t *)param);

            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.commandType = kVDM_Initiator;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.vdmVersion = PD_CONFIG_STRUCTURED_VDM_VERSION;
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.vdmType = 1;
            if (command == PD_DPM_CONTROL_EXIT_MODE)
            {
                pdInstance->structuredVdmCommandParameter.vdoCount = 0;
                pdInstance->structuredVdmCommandParameter.vdoData = NULL;
            }
            break;
        }
#endif

        default:
            break;
    }

    /* special process */
    switch (command)
    {
        case PD_DPM_CONTROL_POWER_NEGOTIATION:
        {
            if ((pdInstance->psmCurState == PSM_PE_SRC_READY) && (pdInstance->curPowerRole == kPD_PowerRoleSource))
            {
                status = kStatus_PD_Success;
            }
            else
            {
                status = kStatus_PD_Error;
            }
            break;
        }

        case PD_DPM_CONTROL_REQUEST:
        {
            if ((pdInstance->psmCurState == PSM_PE_SNK_READY) && (pdInstance->curPowerRole == kPD_PowerRoleSink))
            {
                pdInstance->rdoRequest.rdoVal = *((uint32_t *)param);
                status = kStatus_PD_Success;
            }
            else
            {
                status = kStatus_PD_Error;
            }
            break;
        }

        case PD_DPM_CONTROL_GOTO_MIN:
        {
            if (pdInstance->psmCurState == PSM_PE_SRC_READY)
            {
                status = kStatus_PD_Success;
            }
            else
            {
                status = kStatus_PD_Error;
            }
            break;
        }

#if (defined PD_CONFIG_DUAL_POWER_ROLE_ENABLE) && (PD_CONFIG_DUAL_POWER_ROLE_ENABLE)
        case PD_DPM_CONTROL_PR_SWAP:
        {
            if (((pdInstance->psmCurState == PSM_PE_SNK_READY) || (pdInstance->psmCurState == PSM_PE_SRC_READY)) &&
                ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault) ||
                 (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceDefault) ||
                 (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) ||
                 (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSourcingDevice) ||
                 (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSinkingHost)))
            {
                status = kStatus_PD_Success;
            }
            else
            {
                status = kStatus_PD_Error;
            }
            break;
        }
#endif

#if (defined PD_CONFIG_DUAL_DATA_ROLE_ENABLE) && (PD_CONFIG_DUAL_DATA_ROLE_ENABLE)
        case PD_DPM_CONTROL_DR_SWAP:
        {
            if (pdInstance->pdPowerPortConfig->dataFunction == kDataConfig_DRD)
            {
                status = kStatus_PD_Success;
            }
            else
            {
                status = kStatus_PD_Error;
            }
            break;
        }
#endif

#if (defined PD_CONFIG_VCONN_SUPPORT) && (PD_CONFIG_VCONN_SUPPORT)
        case PD_DPM_CONTROL_VCONN_SWAP:
        {
            if (pdInstance->pdPowerPortConfig->vconnSupported)
            {
                status = kStatus_PD_Success;
            }
            else
            {
                status = kStatus_PD_Error;
            }
            break;
        }
#endif

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        case PD_DPM_GET_SRC_EXT_CAP:
#endif
        case PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES:
        {
            status = kStatus_PD_Error;
            if (pdInstance->pdPowerPortConfig->typecRole != kPowerConfig_SourceOnly)
            {
                status = kStatus_PD_Success;
            }
            break;
        }

        case PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES:
        {
            status = kStatus_PD_Error;
            if (pdInstance->pdPowerPortConfig->typecRole != kPowerConfig_SinkOnly)
            {
                status = kStatus_PD_Success;
            }
            break;
        }

        case PD_DPM_CONTROL_SOFT_RESET:
            pdInstance->psmSoftResetSop = *((uint8_t *)param);
            status = kStatus_PD_Success;
            break;

        case PD_DPM_CONTROL_HARD_RESET:
            status = kStatus_PD_Success;
            break;

#if (defined PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE) && (PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE)
        case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_DiscoverIdentity;
            status = kStatus_PD_Success;
            break;
        }

        case PD_DPM_CONTROL_DISCOVERY_SVIDS:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_DiscoverSVIDs;
            status = kStatus_PD_Success;
            break;
        }

        case PD_DPM_CONTROL_DISCOVERY_MODES:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_DiscoverModes;
            status = kStatus_PD_Success;
            break;
        }

        case PD_DPM_CONTROL_SEND_ATTENTION:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_Attention;
            status = kStatus_PD_Success;
            break;
        }

        case PD_DPM_SEND_VENDOR_STRUCTURED_VDM:
        {
            status = kStatus_PD_Success;
            break;
        }

        case PD_DPM_CONTROL_ENTER_MODE:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_EnterMode;

            if (pdInstance->curDataRole == kPD_DataRoleUFP)
            {
                status = kStatus_PD_Error;
            }
            else
            {
                status = kStatus_PD_Success;
            }
            break;
        }

        case PD_DPM_CONTROL_EXIT_MODE:
        {
            pdInstance->structuredVdmCommandParameter.vdmHeader.bitFields.command = kVDM_ExitMode;

            if (pdInstance->curDataRole == kPD_DataRoleUFP)
            {
                status = kStatus_PD_Error;
            }
            else
            {
                status = kStatus_PD_Success;
            }
            break;
        }

        case PD_DPM_SEND_UNSTRUCTURED_VDM:
        {
            pd_unstructured_vdm_header_t *unstructuredVDMHeader;
            pdInstance->unstructuredVdmCommandParameter = *((pd_unstructured_vdm_command_param_t *)param);
            unstructuredVDMHeader =
                (pd_unstructured_vdm_header_t *)&(pdInstance->unstructuredVdmCommandParameter.vdmHeaderAndVDOsData[0]);
            unstructuredVDMHeader->bitFields.vdmType = 0;
            status = kStatus_PD_Success;
            break;
        }
#endif

        case PD_DPM_CONTROL_CABLE_RESET:
            status = kStatus_PD_Success;
            break;

#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        case PD_DPM_GET_STATUS:
            status = kStatus_PD_Success;
            break;

        case PD_DPM_GET_BATTERY_CAP:
        case PD_DPM_GET_BATTERY_STATUS:
            pdInstance->getBatteryCapDataBlock = *((uint8_t *)param);
            status = kStatus_PD_Success;
            break;

        case PD_DPM_GET_MANUFACTURER_INFO:
#if 0
        case PD_DPM_SECURITY_REQUEST:
        case PD_DPM_FIRMWARE_UPDATE_REQUEST:
#endif
            pdInstance->commandExtParam = *((pd_command_data_param_t *)param);
            status = kStatus_PD_Success;
            break;

        case PD_DPM_ALERT:
            pdInstance->alertADO = *((uint32_t *)param);
            status = kStatus_PD_Success;
            break;
#endif

#if (defined PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        case PD_DPM_FAST_ROLE_SWAP:
            if ((pdInstance->psmCurState == PSM_PE_SRC_READY) && (pdInstance->curPowerRole == kPD_PowerRoleSource) &&
                ((pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SinkDefault) ||
                 (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_SourceDefault) ||
                 (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPToggling) ||
                 (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSourcingDevice) ||
                 (pdInstance->pdPowerPortConfig->typecRole == kPowerConfig_DRPSinkingHost)))
            {
                status = kStatus_PD_Success;
            }
            else
            {
                status = kStatus_PD_Error;
            }
            break;
#endif

        default:
            status = kStatus_PD_Error;
            break;
    }

    if (status == kStatus_PD_Success)
    {
/* (1). PTN5110 need signal fr_swap before turn off vbus, otherwise signal will fail
 * (2). "enter the PE_FRS_SRC_SNK_CC_Signal state" after "signal fr_swap" need be atomic.
 *  the psmNewState need change in the same task context, it is the PD task. here
 *  will not change it. So the (2) is not satisfied.
 */
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        if (command == PD_DPM_FAST_ROLE_SWAP)
        {
            pdInstance->frSwapReceived = 0;
            pdInstance->frSignaledWaitFrSwap = 1;
            PD_PhyControl(pdInstance, PD_PHY_SIGNAL_FR_SWAP, NULL);
        }
#endif
        PD_DpmSendMsg(pdHandle, command);
    }

    return status;
}

pd_status_t PD_Control(pd_handle pdHandle, uint32_t controlCode, void *param)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    pd_status_t status = kStatus_PD_Success;
    uint32_t caseValue = 0;

    caseValue = caseValue;
    switch (controlCode)
    {
        case PD_CONTROL_GET_TYPEC_ORIENTATION:
            if (pdInstance->ccUsed == kPD_CC1)
            {
                *((uint8_t *)param) = 0;
            }
            else
            {
                *((uint8_t *)param) = 1;
            }
            break;

        case PD_CONTROL_GET_POWER_ROLE:
            *((uint8_t *)param) = pdInstance->curPowerRole;
            break;

        case PD_CONTROL_GET_DATA_ROLE:
            *((uint8_t *)param) = pdInstance->curDataRole;
            break;

        case PD_CONTROL_GET_TYPEC_CONNECT_STATE:
            *((uint8_t *)param) = pdInstance->connectState;
            break;

        case PD_CONTROL_GET_VCONN_ROLE:
            *((uint8_t *)param) = pdInstance->psmPresentlyVconnSource;
            break;

        case PD_CONTROL_GET_SNK_TYPEC_CURRENT_CAP:
            PD_PhyControl(pdInstance, PD_PHY_SNK_GET_TYPEC_CURRENT_CAP, param);
            break;

        case PD_CONTROL_PHY_POWER_PIN:
            PD_PhyControl(pdInstance, PD_PHY_CONTROL_POWER_PIN, param);
            break;

        case PD_CONTROL_VCONN:
            PD_PhyControl(pdInstance, PD_PHY_CONTROL_VCONN, param);
            break;

        case PD_CONTROL_GET_PHY_LOW_POWER_STATE:
            status = PD_PhyControl(pdInstance, PD_PHY_GET_LOWPOWER_STATE, param);
            break;

        case PD_CONTROL_DISCHARGE_VBUS:
        {
            uint32_t tmp32Val = 1;
            PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VBUS, &tmp32Val);
#if (defined PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE) && (PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE)
            PD_TimerStart(pdInstance, tTypeCVbusMinDischargeTimer, T_MIN_VBUS_DISCHARGE);
#else
            PD_TimerStart(pdInstance, tTypeCVbusMaxDischargeTimer, T_MAX_VBUS_DISCHARGE);
#endif
            while (1)
            {
                PD_PhyControl(pdInstance, PD_PHY_GET_VBUS_POWER_STATE, &tmp32Val);
                if ((tmp32Val & PD_VBUS_POWER_STATE_VSAFE0V_MASK) ||
#if (defined PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE) && (PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE)
                    (PD_TimerCheckInvalidOrTimeOut(pdInstance, tTypeCVbusMinDischargeTimer))
#else
                    PD_TimerCheckInvalidOrTimeOut(pdInstance, tTypeCVbusMaxDischargeTimer)
#endif
                        )
                {
                    break;
                }
            }
            tmp32Val = 0;
            PD_PhyControl(pdInstance, PD_PHY_DISCHARGE_VBUS, &tmp32Val);

            break;
        }

        case PD_CONTROL_INFORM_VBUS_VOLTAGE_RANGE:
            caseValue = *((uint32_t *)param);
            PD_PhyControl(pdInstance, PD_PHY_INFORM_VBUS_VOLTAGE_RANGE, &caseValue);
            break;

        default:
            break;
    }

    return status;
}

pd_status_t PD_DpmAppCallback(pd_instance_t *pdInstance, uint32_t event, void *param, uint8_t done)
{
    if (done)
    {
        if (pdInstance->curPowerRole == kPD_PowerRoleSource)
        {
            PD_MsgSrcEndCommand(pdInstance);
        }

        /* in case the power is not reset to normal operation */
        PD_ConnectSetPowerProgress(pdInstance, kVbusPower_Stable);
        pdInstance->commandProcessing = 0;
    }

    return pdInstance->pdCallback(pdInstance->callbackParam, event, param);
}
