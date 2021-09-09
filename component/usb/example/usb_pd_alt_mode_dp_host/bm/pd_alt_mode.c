/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
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
#include "string.h"
#include "pd_alt_mode.h"
#include "pd_app_misc.h"
#include "pd_board_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static pd_alt_mode_t s_AltModeInstances[PD_ALT_MODE_MAX_PORT];

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PD_AltModeSetEvent(pd_alt_mode_t *altModeInstance, uint32_t events)
{
    /* it is called in isr and bm */
    APP_CRITICAL_ALLOC();
    APP_ENTER_CRITICAL();
    altModeInstance->altModeTaskEvent |= events;
    APP_EXIT_CRITICAL();
}

static uint32_t PD_AltModeGetAndClearEvent(pd_alt_mode_t *altModeInstance)
{
    APP_CRITICAL_ALLOC();
    APP_ENTER_CRITICAL();
    uint32_t events = altModeInstance->altModeTaskEvent;
    altModeInstance->altModeTaskEvent = 0;
    APP_EXIT_CRITICAL();
    return events;
}

static void PD_AltModeDelayRetryCommand(pd_alt_mode_t *altModeInstance, uint8_t command, uint32_t delay)
{
    altModeInstance->delayTime = delay;
    altModeInstance->retryCommand = command;
}

static void PD_AltModeTrigerCommand(pd_alt_mode_t *altModeInstance, uint8_t command)
{
    altModeInstance->dfpCommand = command;
    PD_AltModeSetEvent(altModeInstance, PD_ALT_MODE_EVENT_COMMAND);
    /* PD_AltModeSendCommand(altModeInstance, command); */
}

static pd_status_t PD_AltModeSendCommand(pd_alt_mode_t *altModeInstance, uint8_t command)
{
    pd_svdm_command_param_t structuredVDMCommandParam;
    uint32_t vdmCommand = 0;
    void *param = NULL;

    structuredVDMCommandParam.vdmSop = kPD_MsgSOP;
    structuredVDMCommandParam.vdmHeader.bitFields.SVID = 0xFF00u;
    structuredVDMCommandParam.vdmHeader.bitFields.vdmType = 1;
    structuredVDMCommandParam.vdmHeader.bitFields.objPos = 0;
    structuredVDMCommandParam.vdmHeader.bitFields.commandType = kVDM_Initiator;

    switch (command)
    {
        case PD_DPM_CONTROL_DISCOVERY_IDENTITY:
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_DiscoverIdentity;
            vdmCommand = PD_DPM_CONTROL_DISCOVERY_IDENTITY;
            param = &structuredVDMCommandParam;
            break;

        case PD_DPM_CONTROL_DISCOVERY_SVIDS:
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_DiscoverSVIDs;
            vdmCommand = PD_DPM_CONTROL_DISCOVERY_SVIDS;
            param = &structuredVDMCommandParam;
            break;

        case PD_DPM_CONTROL_DR_SWAP:
        {
            uint8_t dataRole;
            PD_Control(altModeInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);
            if ((altModeInstance->altModeConfig->workAsDFP) && (dataRole == kPD_DataRoleUFP))
            {
                vdmCommand = PD_DPM_CONTROL_DR_SWAP;
            }
            else
            {
                PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_IDENTITY);
            }
            param = NULL;
            break;
        }

        default:
            break;
    }

    if (vdmCommand != 0)
    {
        if (PD_Command(altModeInstance->pdHandle, vdmCommand, param) != kStatus_PD_Success)
        {
            /* wait and retry again */
            PD_AltModeDelayRetryCommand(altModeInstance, vdmCommand, PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
        }
    }

    return kStatus_PD_Success;
}

static void PD_AltModeInstanceReset(pd_alt_mode_t *altModeInstance)
{
    PD_AltModeStartAMETime(altModeInstance);
    altModeInstance->dfpCommand = 0;
    altModeInstance->retryCommand = 0;
}

void PD_AltModeModuleTaskWakeUp(void *altModeHandle, void *moduleHandle)
{
    uint8_t index;
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;

    if ((altModeHandle == NULL) || (moduleHandle == NULL))
    {
        return;
    }

    for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
    {
        if (altModeInstance->altModeModuleInstance[index] == moduleHandle)
        {
            PD_AltModeSetEvent(altModeInstance, (0x00000001u << index));
        }
    }
}

void PD_AltModeTimer1msISR(void *altModeHandle)
{
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;
    uint8_t index;

    if (altModeHandle == NULL)
    {
        return;
    }

    if (altModeInstance->delayTime > 0)
    {
        altModeInstance->delayTime--;
        if (altModeInstance->delayTime == 0)
        {
            PD_AltModeTrigerCommand(altModeInstance, altModeInstance->retryCommand);
        }
    }

    if (altModeInstance->AMETime > 0)
    {
        altModeInstance->AMETime--;
        if (altModeInstance->AMETime == 0)
        {
            PD_AltModeSetEvent(altModeInstance, PD_ALT_MODE_EVENT_AME_TIMEOUT);
        }
    }

    /* modules' 1ms ISR */
    for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
    {
        if ((altModeInstance->altModeModuleInstance[index] != NULL) &&
            (altModeInstance->altModeConfig->modules[index].pd_alt_mode_1ms_isr != NULL))
        {
            altModeInstance->altModeConfig->modules[index].pd_alt_mode_1ms_isr(
                altModeInstance->altModeModuleInstance[index]);
        }
    }
}

pd_status_t PD_AltModeInit(pd_handle pdHandle, const pd_alt_mode_config_t *altModeConfig, void **altModeHandle)
{
    pd_alt_mode_t *altModeInstance = NULL;
    pd_status_t status = kStatus_PD_Success;
    uint8_t index = 0;
    APP_CRITICAL_ALLOC();

    APP_ENTER_CRITICAL();
    for (index = 0; index < sizeof(s_AltModeInstances) / sizeof(pd_alt_mode_t); ++index)
    {
        if (s_AltModeInstances[index].occupied == 0)
        {
            s_AltModeInstances[index].occupied = 1;
            altModeInstance = &s_AltModeInstances[index];
            break;
        }
    }
    if (altModeInstance == NULL)
    {
        APP_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }
    APP_EXIT_CRITICAL();

    altModeInstance->pdHandle = pdHandle;
    altModeInstance->altModeConfig = altModeConfig;
    PD_AltModeInstanceReset(altModeInstance);
    /* according SVID list, initialize every alt mode module. */
    for (index = 0; index < altModeConfig->moduleCount; ++index)
    {
        if ((altModeConfig->modules[index].pd_alt_mode_init == NULL) ||
            (altModeConfig->modules[index].pd_alt_mode_deinit == NULL) ||
            (altModeConfig->modules[index].pd_alt_mode_callback_event == NULL) ||
            (altModeConfig->modules[index].pd_alt_mode_control == NULL) ||
            (altModeConfig->modules[index].pd_alt_mode_task == NULL) ||
            (altModeConfig->modules[index].pd_alt_mode_1ms_isr == NULL))
        {
            status = kStatus_PD_Error;
            break;
        }

        status = altModeConfig->modules[index].pd_alt_mode_init(pdHandle, altModeInstance,
                                                                altModeConfig->modules[index].config,
                                                                &(altModeInstance->altModeModuleInstance[index]));
        if ((altModeInstance->altModeModuleInstance[index] == NULL) || (status != kStatus_PD_Success))
        {
            status = kStatus_PD_Error;
            break;
        }
    }
    if (status != kStatus_PD_Success)
    {
        for (index = 0; index < altModeConfig->moduleCount; ++index)
        {
            if (altModeInstance->altModeModuleInstance[index] != NULL)
            {
                altModeConfig->modules[index].pd_alt_mode_deinit(altModeInstance->altModeModuleInstance[index]);
                altModeInstance->altModeModuleInstance[index] = NULL;
            }
        }
        return status;
    }

    *altModeHandle = altModeInstance;
    return kStatus_PD_Success;
}

pd_status_t PD_AltModeDeinit(void *altModeHandle)
{
    uint8_t index = 0;
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;

    if (altModeHandle == NULL)
    {
        return kStatus_PD_Error;
    }

    /* according SVID list, initialize every alt mode module. */
    for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
    {
        if (altModeInstance->altModeModuleInstance[index] != NULL)
        {
            altModeInstance->altModeConfig->modules[index].pd_alt_mode_deinit(
                altModeInstance->altModeModuleInstance[index]);
            altModeInstance->altModeModuleInstance[index] = NULL;
        }
    }

    altModeInstance->occupied = 0;
    return kStatus_PD_Success;
}

pd_status_t PD_AltModeEnter(void *altModeHandle)
{
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;
    uint8_t dataRole;

    if (altModeHandle == NULL)
    {
        return kStatus_PD_Error;
    }

    if (altModeInstance->altModeConfig->workAsDFP)
    {
        altModeInstance->retryCountCommand = 0;
        PD_Control(altModeInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);
        if (dataRole == kPD_DataRoleDFP)
        {
            /* start discover */
            PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_IDENTITY);
        }
        else
        {
            /* PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DR_SWAP); */
            /* partner may start data role swap */
            /* PD_AltModeDelayRetryCommand(altModeInstance, PD_DPM_CONTROL_DR_SWAP, PD_ALT_MODE_DR_SWAP_DELAY_TIME); */
        }
    }
    else
    {
        /* do nothing */
    }

    return kStatus_PD_Success;
}

pd_status_t PD_AltModeExit(void *altModeHandle, pd_alt_mode_state_t *enteredMode)
{
    uint8_t index = 0;
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;
    pd_status_t status = kStatus_PD_Error;
    pd_status_t statusTmp;

    if ((enteredMode == NULL) || (altModeHandle == NULL))
    {
        return kStatus_PD_Error;
    }

    for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
    {
        if (altModeInstance->altModeModuleInstance[index] != NULL)
        {
            if (enteredMode->SVID == altModeInstance->altModeConfig->modules[index].SVID)
            {
                statusTmp = altModeInstance->altModeConfig->modules[index].pd_alt_mode_control(
                    altModeInstance->altModeModuleInstance[index], kAltMode_TriggerExitMode, enteredMode);
                if (status != kStatus_PD_Success)
                {
                    status = statusTmp;
                }
            }
        }
    }

    return status;
}

pd_status_t PD_AltModeState(void *altModeHandle, pd_alt_mode_state_t *enteredMode)
{
    uint8_t index = 0;
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;
    pd_status_t statusTmp;

    if (altModeHandle == NULL)
    {
        return kStatus_PD_Error;
    }

    for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
    {
        if (altModeInstance->altModeModuleInstance[index] != NULL)
        {
            if (enteredMode->SVID == altModeInstance->altModeConfig->modules[index].SVID)
            {
                statusTmp = altModeInstance->altModeConfig->modules[index].pd_alt_mode_control(
                    altModeInstance->altModeModuleInstance[index], kAltMode_GetModeState, enteredMode);
                if (statusTmp == kStatus_PD_Success)
                {
                    return kStatus_PD_Success;
                }
            }
        }
    }

    return kStatus_PD_Error;
}

pd_status_t PD_AltModeStartAMETime(void *altModeHandle)
{
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;

    if (altModeHandle == NULL)
    {
        return kStatus_PD_Error;
    }
    altModeInstance->AMETime = AME_TIMEOUT_VALUE;
    return kStatus_PD_Success;
}

pd_status_t PD_AltModeStopAMETime(void *altModeHandle)
{
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;

    if (altModeHandle == NULL)
    {
        return kStatus_PD_Error;
    }
    altModeInstance->AMETime = 0;
    return kStatus_PD_Success;
}

void PD_AltModeTask(void *altModeHandle)
{
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)altModeHandle;
    uint32_t u32Value;
    uint8_t index;

    if (altModeHandle == NULL)
    {
        return;
    }

    /* Alt Mode events process */
    u32Value = PD_AltModeGetAndClearEvent(altModeInstance);
    if (u32Value)
    {
        if (u32Value & PD_ALT_MODE_EVENT_COMMAND)
        {
            if (altModeInstance->retryCountCommand != altModeInstance->dfpCommand)
            {
                altModeInstance->retryCount = PD_ALT_MODE_COMMAND_RETRY_COUNT;
                altModeInstance->retryCountCommand = altModeInstance->dfpCommand;
                PD_AltModeSendCommand(altModeInstance, altModeInstance->dfpCommand);
            }
            else
            {
                if (altModeInstance->retryCount > 0)
                {
                    altModeInstance->retryCount--;
                    PD_AltModeSendCommand(altModeInstance, altModeInstance->dfpCommand);
                }
                else
                {
                    /* do hard reset */
                    PD_Command(altModeInstance->pdHandle, PD_DPM_CONTROL_HARD_RESET, NULL);
                }
            }
        }
        else if (u32Value & PD_ALT_MODE_EVENT_AME_TIMEOUT)
        {
            /* TODO: enable USB billboard device */
        }
        else
        {
        }

        if (u32Value & PD_ALT_MODE_EVENT_MODULES)
        {
            for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
            {
                if (u32Value & (0x00000001u << index))
                {
                    altModeInstance->altModeConfig->modules[index].pd_alt_mode_task(
                        altModeInstance->altModeModuleInstance[index]);
                }
            }
        }
    }
}

static pd_status_t PD_AltModeStandardVDMCallbackProcess(pd_alt_mode_t *altModeInstance, uint32_t event, void *param)
{
    pd_svdm_command_request_t *svdmRequest = (pd_svdm_command_request_t *)param;
    uint8_t index;

    if (event == PD_DPM_STRUCTURED_VDM_REQUEST)
    {
        /* ack or nak, no busy */
        /* partner return nak if it is not in the alternate mode */
        switch (svdmRequest->vdmHeader.bitFields.command)
        {
            case kVDM_DiscoverIdentity: /* UFP */
                svdmRequest->vdoData = altModeInstance->altModeConfig->identityData;
                svdmRequest->vdoCount = altModeInstance->altModeConfig->identityObjectCount;
                svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                break;

            case kVDM_DiscoverSVIDs: /* UFP */
#if 0
                for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
                {
                    if ((index & 0x01) == 0)
                    {
                        altModeInstance->altModeMsgBuffer[index >> 2] =
                            (altModeInstance->altModeConfig->modules[index].SVID << 16);
                    }
                    else
                    {
                        altModeInstance->altModeMsgBuffer[index >> 2] |=
                            altModeInstance->altModeConfig->modules[index].SVID;
                    }
                }
                if ((altModeInstance->altModeConfig->moduleCount & 0x01) == 0)
                {
                    altModeInstance->altModeMsgBuffer[altModeInstance->altModeConfig->moduleCount >> 2] = 0u;
                }
                svdmRequest->vdoData = altModeInstance->altModeMsgBuffer;
                svdmRequest->vdoCount = altModeInstance->altModeConfig->moduleCount / 2 + 1;
#endif
                svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                break;

            case kVDM_DiscoverModes: /* UFP */
                /* PD VID NAK */
                svdmRequest->vdoData = NULL;
                svdmRequest->vdoCount = 0;
                svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                break;

            case kVDM_EnterMode: /* UFP */
                /* PD VID NAK */
                svdmRequest->vdoData = NULL;
                svdmRequest->vdoCount = 0;
                svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                break;

            case kVDM_ExitMode: /* UFP */
                /* PD VID NAK */
                svdmRequest->vdoData = NULL;
                svdmRequest->vdoCount = 0;
                svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                break;

            case kVDM_Attention: /* DFP */
                /* PD VID doesn't support */
                svdmRequest->vdoData = NULL;
                svdmRequest->vdoCount = 0;
                svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                break;

            default:
                break;
        }

        return kStatus_PD_Success;
    }
    else if (event == PD_DPM_STRUCTURED_VDM_SUCCESS)
    {
        pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
        switch (svdmResult->vdmCommand)
        {
            case kVDM_DiscoverIdentity: /* ACK msg, DFP */
                /* 1. TODO, receive identity */
                /* 2. get SVIDs */
                PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_SVIDS);
                break;

            case kVDM_DiscoverSVIDs: /* ACK msg, DFP */
            {
                uint16_t svid = 0;
                /* what SVID to enter?  */
                for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
                {
                    uint8_t index1 = 0;
                    for (index1 = 0; index1 < svdmResult->vdoCount * 2; ++index1)
                    {
                        svid = (uint16_t)(svdmResult->vdoData[index1 >> 1] >> ((index1 & 0x01u) ? 0 : 16));
                        if (svid == altModeInstance->altModeConfig->modules[index].SVID)
                        {
                            break;
                        }
                    }
                    if (index1 < svdmResult->vdoCount * 2)
                    {
                        break;
                    }
                }
                if (index < altModeInstance->altModeConfig->moduleCount)
                {
                    altModeInstance->altModeConfig->modules[index].pd_alt_mode_control(
                        altModeInstance->altModeModuleInstance[index], kAltMode_TriggerEnterMode, NULL);
                }
                else
                {
                    /* if SVIDs is not end, get next SVIDs */
                    /* last SVID */
                    svid = (uint16_t)(svdmResult->vdoData[svdmResult->vdoCount - 1] & 0x0000FFFFu);
                    if (svid != 0x0000u)
                    {
                        /* get next SVIDs LIST */
                        PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_SVIDS);
                    }
                }
                break;
            }

            case kVDM_DiscoverModes: /* ACK msg, DFP */
                /* shoudn't go here becase don't do PD VID discover */;
                break;

            case kVDM_EnterMode: /* ACK msg, DFP */
                /* shoudn't go here becase don't do PD VID enter */;
                break;

            case kVDM_ExitMode: /* ACK msg, DFP */
                /* shoudn't go here becase don't do PD VID exit */;
                break;

            default:
                break;
        }
        return kStatus_PD_Success;
    }
    else if (event == PD_DPM_STRUCTURED_VDM_FAIL)
    {
        pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
        if (svdmResult->vdmCommandResult == kCommandResult_VDMNAK)
        {
            /* don't support this command */
            return kStatus_PD_Success;
        }
        if (svdmResult->vdmCommand == kVDM_DiscoverIdentity)
        {
            /* wait and retry again */
            PD_AltModeDelayRetryCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_IDENTITY,
                                        PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
        }
        else if (svdmResult->vdmCommand == kVDM_DiscoverSVIDs)
        {
            /* wait and retry again */
            PD_AltModeDelayRetryCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_SVIDS,
                                        PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
        }
        else
        {
        }
        return kStatus_PD_Success;
    }
    else if (event == PD_DPM_DR_SWAP_REQUEST)
    {
        uint8_t dataRole;
        PD_Control(altModeInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);
        if (((dataRole == kPD_DataRoleDFP) && (altModeInstance->altModeConfig->workAsDFP)) ||
            ((!(altModeInstance->altModeConfig->workAsDFP)) && (dataRole == kPD_DataRoleUFP)))
        {
            *((uint8_t *)param) = kCommandResult_Reject;
        }
        else
        {
            *((uint8_t *)param) = kCommandResult_Accept;
        }
        return kStatus_PD_Success;
    }
    else if (event == PD_DPM_DR_SWAP_SUCCESS)
    {
        if (altModeInstance->dfpCommand == PD_DPM_CONTROL_DR_SWAP)
        {
            PD_AltModeTrigerCommand(altModeInstance, PD_DPM_CONTROL_DISCOVERY_IDENTITY);
        }
    }
    else if (event == PD_DPM_DR_SWAP_FAIL)
    {
        if (altModeInstance->dfpCommand == PD_DPM_CONTROL_DR_SWAP)
        {
            /* wait and retry again */
            PD_AltModeDelayRetryCommand(altModeInstance, PD_DPM_CONTROL_DR_SWAP, PD_ALT_MODE_DR_SWAP_DELAY_TIME);
        }
    }
    else
    {
        /* don't process */
    }

    return kStatus_PD_Error;
}

pd_status_t PD_AltModeCallback(void *callbackParam, uint32_t event, void *param)
{
    /* if this event is processed, return kStatus_PD_Success */
    pd_status_t status = kStatus_PD_Error;
    pd_status_t statusTmp;
    uint32_t index = 0;
    pd_alt_mode_t *altModeInstance = (pd_alt_mode_t *)callbackParam;
    uint32_t controlCode = kAltMode_Invalid;
    void *controlParam = NULL;
    uint16_t msgSVID = 0;

    if (callbackParam == NULL)
    {
        return kStatus_PD_Error;
    }

    /* process attach/detach events */
    switch (event)
    {
        case PD_CONNECTED:
        case PD_CONNECT_ROLE_CHANGE:
            PD_AltModeInstanceReset(altModeInstance);
            controlCode = kAltMode_Attach;
            controlParam = NULL;
            break;

        case PD_DISCONNECTED:
            PD_AltModeInstanceReset(altModeInstance);
            controlCode = kAltMode_Detach;
            controlParam = NULL;
            break;

        case PD_DPM_SNK_HARD_RESET_REQUEST:
            PD_AltModeInstanceReset(altModeInstance);
            controlCode = kAltMode_HardReset;
            controlParam = NULL;
            break;

        case PD_DPM_SRC_HARD_RESET_REQUEST:
            PD_AltModeInstanceReset(altModeInstance);
            controlCode = kAltMode_HardReset;
            controlParam = NULL;
            break;

        default:
            break;
    }

    if (controlCode != kAltMode_Invalid)
    {
        for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
        {
            if (altModeInstance->altModeModuleInstance[index] != NULL)
            {
                statusTmp = altModeInstance->altModeConfig->modules->pd_alt_mode_callback_event(
                    altModeInstance->altModeModuleInstance[index], controlCode, 0u, controlParam);
                if (status != kStatus_PD_Success)
                {
                    status = statusTmp;
                }
            }
        }

        return status;
    }

    /* process AMS events */
    switch (event)
    {
        /* structured vdm */
        case PD_DPM_STRUCTURED_VDM_REQUEST:
            controlCode = kAltMode_StructedVDMMsgReceivedProcess;
            controlParam = param;
            msgSVID = ((pd_svdm_command_request_t *)param)->vdmHeader.bitFields.SVID;
            break;

        case PD_DPM_STRUCTURED_VDM_SUCCESS:
            controlCode = kAltMode_StructedVDMMsgSuccess;
            controlParam = param;
            msgSVID = ((pd_svdm_command_result_t *)param)->vdmHeader.bitFields.SVID;
            break;

        case PD_DPM_STRUCTURED_VDM_FAIL:
            controlCode = kAltMode_StructedVDMMsgFail;
            controlParam = param;
            msgSVID = ((pd_svdm_command_result_t *)param)->vdmHeader.bitFields.SVID;
            break;

        /* dr swap */
        case PD_DPM_DR_SWAP_REQUEST:
        case PD_DPM_DR_SWAP_SUCCESS:
        case PD_DPM_DR_SWAP_FAIL:
            break;

        /* unstructured vdm */
        case PD_DPM_UNSTRUCTURED_VDM_RECEIVED:
            controlCode = kAltMode_UnstructedVDMMsgReceived;
            controlParam = param;
            msgSVID = (((pd_unstructured_vdm_command_param_t *)param)->vdmHeaderAndVDOsData[0] >> 16);
            break;

        case PD_DPM_SEND_UNSTRUCTURED_VDM_SUCCESS:
        case PD_DPM_SEND_UNSTRUCTURED_VDM_FAIL:
            controlCode = kAltMode_UnstructedVDMMsgSentResult;
            controlParam = param;
            msgSVID = 0;
            break;

        default:
            break;
    }

    if (controlCode != kAltMode_Invalid)
    {
        if (msgSVID == 0xFF00)
        {
            /* standard VDM process */
            statusTmp = PD_AltModeStandardVDMCallbackProcess(altModeInstance, event, param);
            if (status != kStatus_PD_Success)
            {
                status = statusTmp;
            }
        }
        else
        {
            for (index = 0; index < altModeInstance->altModeConfig->moduleCount; ++index)
            {
                if (altModeInstance->altModeModuleInstance[index] != NULL)
                {
                    if (((msgSVID != 0) && (msgSVID == altModeInstance->altModeConfig->modules[index].SVID)) ||
                        (msgSVID == 0))
                    {
                        statusTmp = altModeInstance->altModeConfig->modules->pd_alt_mode_callback_event(
                            altModeInstance->altModeModuleInstance[index], controlCode, msgSVID, controlParam);
                        if (status != kStatus_PD_Success)
                        {
                            status = statusTmp;
                        }
                    }
                }
            }
        }
    }
    else
    {
        /* dr swap */
        statusTmp = PD_AltModeStandardVDMCallbackProcess(altModeInstance, event, param);
        if (status != kStatus_PD_Success)
        {
            status = statusTmp;
        }
    }

    return status;
}
