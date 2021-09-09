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
#include "pd_board_config.h"
#include "pd_alt_mode.h"
#include "pd_alt_mode_dp.h"
#include "pd_dp_hpd_driver.h"
#include "pd_typec_crossbar.h"
#include "pd_app_misc.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

pd_alt_mode_dp_t s_AltModeDisplayPortInstance[PD_DP_MAX_PORT];

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PD_DpDelayRetryCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command, uint32_t delay)
{
    dpInstance->delayTime = delay;
    dpInstance->dfpCommand = command;
    /* dpInstance->delayEvents = events; */
}

/* DP DFP */
static pd_status_t PD_DpDFPSendCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command)
{
    pd_svdm_command_param_t structuredVDMCommandParam;
    uint32_t vdmCommand = 0;

    if (dpInstance->dfpCommandDoing)
    {
        return kStatus_PD_Error;
    }
    structuredVDMCommandParam.vdmSop = kPD_MsgSOP;
    structuredVDMCommandParam.vdmHeader.bitFields.SVID = DP_SVID;
    structuredVDMCommandParam.vdmHeader.bitFields.vdmType = 1;
    structuredVDMCommandParam.vdmHeader.bitFields.objPos = 0;
    structuredVDMCommandParam.vdmHeader.bitFields.commandType = kVDM_Initiator;

    switch (command)
    {
        case kVDM_DiscoverModes:
            if (dpInstance->dpState != kDPMode_Exited)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData = NULL;
            vdmCommand = PD_DPM_CONTROL_DISCOVERY_MODES;
            break;

        case kVDM_EnterMode:
            if (dpInstance->dpState != kDPMode_Exited)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_EnterMode;
            vdmCommand = PD_DPM_CONTROL_ENTER_MODE;
            break;

        case kDPVDM_StatusUpdate:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 1;
            structuredVDMCommandParam.vdoData = (uint32_t *)&dpInstance->dpSelfStatus;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kDPVDM_StatusUpdate;
            structuredVDMCommandParam.vendorVDMNeedResponse = 1;
            vdmCommand = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;

        case kDPVDM_Configure:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 1;
            structuredVDMCommandParam.vdoData = (uint32_t *)&dpInstance->dpConfigure;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kDPVDM_Configure;
            structuredVDMCommandParam.vendorVDMNeedResponse = 1;
            vdmCommand = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;

        case kVDM_ExitMode:
            if (dpInstance->dpState < kDPMode_EnterDPDone)
            {
                return kStatus_PD_Error;
            }
            structuredVDMCommandParam.vdoCount = 0;
            structuredVDMCommandParam.vdoData = NULL;
            structuredVDMCommandParam.vdmHeader.bitFields.objPos = dpInstance->selectModeIndex;
            structuredVDMCommandParam.vdmHeader.bitFields.command = kVDM_ExitMode;
            vdmCommand = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
            break;

        default:
            break;
    }

    if (vdmCommand != 0)
    {
        dpInstance->dfpCommandDoing = 1;
        if (PD_Command(dpInstance->pdHandle, vdmCommand, &structuredVDMCommandParam) != kStatus_PD_Success)
        {
            dpInstance->dfpCommandDoing = 0;
            /* wait and retry again */
            PD_DpDelayRetryCommand(dpInstance, command, PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
        }
    }

    return kStatus_PD_Success;
}

/* DP DFP */
static void PD_DpDFPTrigerCommand(pd_alt_mode_dp_t *dpInstance, uint8_t command)
{
    APP_CRITICAL_ALLOC();

    dpInstance->dfpCommand = command;
    APP_ENTER_CRITICAL();
    dpInstance->triggerCommand = 1;
    APP_EXIT_CRITICAL();
    PD_AltModeModuleTaskWakeUp(dpInstance->altModeHandle, dpInstance);
}

/* DP DFP function */
static pd_status_t PD_DpDFPProcessModes(pd_alt_mode_dp_t *dpInstance)
{
    pd_dp_mode_obj_t modeObj;
    uint8_t index;
    uint8_t configurePin = 0;

    if (dpInstance->pdMsgReceivedVDOCount < 1)
    {
        return kStatus_PD_Success;
    }

    dpInstance->selectModeIndex = 0;

    for (index = 0; index < dpInstance->pdMsgReceivedVDOCount; ++index)
    {
        modeObj.modeVal = dpInstance->pdMsgReceivedBuffer[index];
        if ((modeObj.modeVal & 0xFF000000u) || ((modeObj.modeVal & 0x00FFFFFFu) == 0) ||
            ((modeObj.bitFields.portCap & kDPPortCap_UFPD) == 0))
        {
            /* invalid mode */
            continue;
        }

        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        if (configurePin & (dpInstance->dpConfigParam->supportPinAssigns))
        {
            dpInstance->selectModeIndex = index + 1;
            break;
        }
    }

    if (dpInstance->selectModeIndex != 0)
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        PRINTF("device supported pin assignments: ");
        if (configurePin & kPinAssign_A)
        {
            PRINTF("A");
        }
        if (configurePin & kPinAssign_B)
        {
            PRINTF("B");
        }
        if (configurePin & kPinAssign_C)
        {
            PRINTF("C");
        }
        if (configurePin & kPinAssign_D)
        {
            PRINTF("D");
        }
        if (configurePin & kPinAssign_E)
        {
            PRINTF("E");
        }
#endif
        PRINTF("\r\n");
        return kStatus_PD_Success;
    }

    return kStatus_PD_Error;
}

/* DP DFP function */
static pd_status_t PD_DpDFPConstructConfigure(pd_alt_mode_dp_t *dpInstance)
{
    pd_dp_mode_obj_t modeObj;
    uint8_t setSignal = 0;
    uint8_t configurePin = 0;

    if (dpInstance->selectModeIndex == 0)
    {
        return kStatus_PD_Error;
    }

    modeObj.modeVal = dpInstance->pdMsgReceivedBuffer[dpInstance->selectModeIndex - 1];

    /* if prefer multi function, kPinAssign_B and kPinAssign_D has high priority */
    if ((dpInstance->dpPartnerStatus.bitFields.multiFunctionPreferred) ||
        (dpInstance->dpConfigParam->multiFunctionPrefered))
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }
        configurePin &= (kPinAssign_B | kPinAssign_D);
        configurePin &= dpInstance->dpConfigParam->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_D)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_D;
            }
            else if (configurePin & kPinAssign_B)
            {
                setSignal = kDPSignal_USBGEN2;
                configurePin = kPinAssign_B;
            }
            else
            {
            }
        }
    }

    /* multi function is not prefered or don't get kPinAssign_B and kPinAssign_D
     * prefer the 4 lane pin assignment*/
    if (configurePin == 0)
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        configurePin &= (~(kPinAssign_B | kPinAssign_D));
        configurePin &= dpInstance->dpConfigParam->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_C)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_C;
            }
            else if (configurePin & kPinAssign_E)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_E;
            }
            else if (configurePin & kPinAssign_A)
            {
                setSignal = kDPSignal_USBGEN2;
                configurePin = kPinAssign_A;
            }
            else
            {
            }
        }
    }

    /* get the first one */
    if (configurePin == 0)
    {
        if (modeObj.bitFields.receptacleIndication)
        {
            /* receptacle */
            configurePin = modeObj.bitFields.UFPDPinSupport;
        }
        else
        {
            configurePin = modeObj.bitFields.DFPDPinSupport;
        }

        configurePin &= dpInstance->dpConfigParam->supportPinAssigns;
        if (configurePin != 0)
        {
            if (configurePin & kPinAssign_A)
            {
                setSignal = kDPSignal_USBGEN2;
                configurePin = kPinAssign_A;
            }
            else if (configurePin & kPinAssign_B)
            {
                setSignal = kDPSignal_USBGEN2;
                configurePin = kPinAssign_B;
            }
            else if (configurePin & kPinAssign_C)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_C;
            }
            else if (configurePin & kPinAssign_D)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_D;
            }
            else if (configurePin & kPinAssign_E)
            {
                setSignal = kDPSignal_DP;
                configurePin = kPinAssign_E;
            }
            else
            {
            }
        }
    }

    if (configurePin != 0)
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        PRINTF("select pin assignments: ");
        if (configurePin & kPinAssign_A)
        {
            PRINTF("A");
        }
        else if (configurePin & kPinAssign_B)
        {
            PRINTF("B");
        }
        else if (configurePin & kPinAssign_C)
        {
            PRINTF("C");
        }
        else if (configurePin & kPinAssign_D)
        {
            PRINTF("D");
        }
        else if (configurePin & kPinAssign_E)
        {
            PRINTF("E");
        }
        else
        {
        }
        PRINTF("\r\n");
#endif
        dpInstance->dpConfigure.bitFields.setConfig = kDPConfig_UFPD;
        dpInstance->dpConfigure.bitFields.setSignal = setSignal;
        dpInstance->dpConfigure.bitFields.configureUFPUPin = configurePin;
        return kStatus_PD_Success;
    }

    return kStatus_PD_Error;
}

static void PD_DpDFPSetConfigureAsUSB(pd_alt_mode_dp_t *dpInstance)
{
    dpInstance->dpConfigure.bitFields.setConfig = kDPConfig_USB;
    dpInstance->dpConfigure.bitFields.setSignal = kDPSignal_Unspecified;
    dpInstance->dpConfigure.bitFields.configureUFPUPin = kPinAssign_DeSelect;
}

static void PD_DpDFPProcessUFPstatus(pd_alt_mode_dp_t *dpInstance)
{
    uint8_t driverVal = kDPDriver_Low;
    if (dpInstance->dpPartnerStatus.bitFields.HPDInterrupt)
    {
        driverVal = kDPDriver_IRQ;
    }
    else if (dpInstance->dpPartnerStatus.bitFields.HPDState)
    {
        driverVal = kDPDriver_High;
    }
    else
    {
        driverVal = kDPDriver_Low;
    }
    PD_DpHpdDriverControl(&dpInstance->hpdDriverInstance, driverVal);

    /* Figure 5-4 */
    if ((dpInstance->dpPartnerStatus.bitFields.exitDPModeReq) || (dpInstance->dpPartnerStatus.bitFields.USBConfigReq) ||
        (!(dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)))
    {
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
        PRINTF("start exit mode\r\n");
#endif
        PD_DpHpdDriverControl(&dpInstance->hpdDriverInstance, kDPDriver_Low);
        /* The DFP_U shall issue an Exit Mode command only when the port is configured to be in USB configuration.
         */
        /* Receipt of an Exit Mode command while not configured in USB Configuration indicates an error in the
         * DFP_U. */
        PD_DpHpdDriverSetLow(&dpInstance->hpdDriverInstance);
        if (dpInstance->dpState == kDPMode_ConfigureDone)
        {
            PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_SAFE_MODE, 0);
            PD_DpDFPSetConfigureAsUSB(dpInstance);
            PD_DpDFPSendCommand(dpInstance, kDPVDM_Configure);
        }
        else
        {
            PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_USB3_ONLY, 0);
            PD_DpDFPSendCommand(dpInstance, kVDM_ExitMode);
        }
    }
}

/* DP UFP function */
static void PD_DpUFPProcessStatusUpdate(pd_alt_mode_dp_t *dpInstance)
{
    /* check dfp's status and update self status */
}

/* DP UFP function */
static pd_status_t PD_DpUFPProcessConfigure(pd_alt_mode_dp_t *dpInstance)
{
    if ((dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_DFPD) ||
        (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD))
    {
    }
    else
    {
    }

    return kStatus_PD_Success;
}

static void PD_DpInstanceReset(pd_alt_mode_dp_t *dpInstance)
{
    dpInstance->dpState = kDPMode_Exited;
    dpInstance->taskEvent = 0u;
    dpInstance->dfpCommandDoing = 0;
    PD_DpHpdDriverControl(&dpInstance->hpdDriverInstance, kDPDriver_Low);
    PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_USB3_ONLY, 0);
}

void PD_DPModule1msISR(void *moduleInstance)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;

    if (dpInstance->delayTime > 0)
    {
        dpInstance->delayTime--;
        if (dpInstance->delayTime == 0)
        {
            /* PD_DpModuleSetEvent(dpInstance, dpInstance->delayEvents); */
            PD_DpDFPTrigerCommand(dpInstance, dpInstance->dfpCommand);
        }
    }
    PD_DpHpdDriver1msISR(&dpInstance->hpdDriverInstance);
}

/*
 * pdHandle - PD tack handle.
 * altModeHandle - alt mode driver handle.
 * moduleConfig - displayport module configuration parameter.
 * moduleInstance - return the displayport module instance handle
 *
*/
pd_status_t PD_DPInit(pd_handle pdHandle, void *altModeHandle, const void *moduleConfig, void **moduleInstance)
{
    uint32_t index = 0;
    pd_alt_mode_dp_t *dpInstance = NULL;
    APP_CRITICAL_ALLOC();

    APP_ENTER_CRITICAL();
    for (index = 0; index < sizeof(s_AltModeDisplayPortInstance) / sizeof(pd_alt_mode_dp_t); ++index)
    {
        if (s_AltModeDisplayPortInstance[index].occupied == 0)
        {
            s_AltModeDisplayPortInstance[index].occupied = 1;
            dpInstance = &s_AltModeDisplayPortInstance[index];
            break;
        }
    }

    if (dpInstance == NULL)
    {
        APP_EXIT_CRITICAL();
        return kStatus_PD_Error;
    }
    APP_EXIT_CRITICAL();
    dpInstance->pdHandle = pdHandle;
    dpInstance->altModeHandle = altModeHandle;
    dpInstance->dpConfigParam = (pd_alt_mode_dp_config_t *)moduleConfig;
    if (PD_CrossbarInit(&dpInstance->crossbarInstance, dpInstance->pdHandle,
                        dpInstance->dpConfigParam->crossbarConfig) != 0)
    {
        dpInstance->occupied = 0;
        return kStatus_PD_Error;
    }

    if (PD_DpHpdDriverInit(&dpInstance->hpdDriverInstance, dpInstance->altModeHandle, dpInstance,
                           dpInstance->dpConfigParam->hpdConfig) != 0)
    {
        PD_CrossbarDeinit(&dpInstance->crossbarInstance);
        dpInstance->occupied = 0;
        return kStatus_PD_Error;
    }
    PD_DpInstanceReset(dpInstance);

    *moduleInstance = dpInstance;
    return kStatus_PD_Success;
}

pd_status_t PD_DPDeinit(void *moduleInstance)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;

    PD_CrossbarDeinit(&dpInstance->crossbarInstance);
    dpInstance->occupied = 0;
    return kStatus_PD_Success;
}

pd_status_t PD_DPControl(void *moduleInstance, uint32_t controlCode, void *controlParam)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;
    pd_status_t status = kStatus_PD_Success;

    /* dfp */
    switch (controlCode)
    {
        /* DFP start to enter mode sequence */
        case kAltMode_TriggerEnterMode:
            PD_DpDFPTrigerCommand(dpInstance, kVDM_DiscoverModes);
            break;

        case kAltMode_TriggerExitMode:
            PD_DpDFPTrigerCommand(dpInstance, kVDM_ExitMode);
            /* PD_DpModuleSetEvent(dpInstance, DP_TASK_EVENT_DFP_EXIT_MODE_TRIGGER); */
            break;

        case kAltMode_GetModeState:
        {
            if (controlParam == NULL)
            {
                status = kStatus_PD_Error;
            }
            else
            {
                pd_alt_mode_state_t *modeState = (pd_alt_mode_state_t *)controlParam;
                modeState->SVID = 0u;
                modeState->mode = 0;
                if (dpInstance->dpState >= kDPMode_EnterDPDone)
                {
                    modeState->SVID = DP_SVID;
                    modeState->mode = dpInstance->selectModeIndex;
                }
            }
            break;
        }

        default:
            break;
    }

    return status;
}

/* msgSVID: 0 - this msg related event doesn't know SVID. */
pd_status_t PD_DPCallbackEvent(void *moduleInstance, uint32_t processCode, uint16_t msgSVID, void *param)
{
    pd_status_t status = kStatus_PD_Error;
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)moduleInstance;
    uint32_t index = 0;

    if ((msgSVID != 0) && (msgSVID != 0xFF01u))
    {
        return status;
    }

    /* process the msg related events, if not self msg or self shouldn't process this event return error. */
    switch (processCode)
    {
        case kAltMode_Attach:
            PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_USB3_ONLY, 0);
            PD_DpHpdDriverSetLow(&dpInstance->hpdDriverInstance);
            PD_DpInstanceReset(dpInstance);
            break;

        case kAltMode_Detach:
            PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_SHUTDOWN, 0);
            PD_DpHpdDriverSetLow(&dpInstance->hpdDriverInstance);
            PD_DpInstanceReset(dpInstance);
            break;

        case kAltMode_HardReset:
            PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_USB3_ONLY, 0);
            PD_DpHpdDriverSetLow(&dpInstance->hpdDriverInstance);
            PD_DpInstanceReset(dpInstance);
            break;

        case kAltMode_StructedVDMMsgReceivedProcess:
        {
            pd_svdm_command_request_t *svdmRequest = (pd_svdm_command_request_t *)param;
            if (msgSVID == 0xFF01)
            {
                status = kStatus_PD_Success;
                switch (svdmRequest->vdmHeader.bitFields.command)
                {
                    case kVDM_DiscoverModes: /* DP UFP */
                        for (index = 0; index < dpInstance->dpConfigParam->modesCount; ++index)
                        {
                            dpInstance->pdMsgBuffer[index] = dpInstance->dpConfigParam->modesList[index];
                        }
                        svdmRequest->vdoData = (uint32_t *)&dpInstance->pdMsgBuffer[0];
                        svdmRequest->vdoCount = dpInstance->dpConfigParam->modesCount;
                        svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        break;

                    case kVDM_EnterMode: /* DP UFP */
                        dpInstance->dpConfigure.configureVal = svdmRequest->vdoData[0];
                        svdmRequest->vdoData = NULL;
                        svdmRequest->vdoCount = 0;
                        if (svdmRequest->vdmHeader.bitFields.objPos <= dpInstance->dpConfigParam->modesCount)
                        {
                            dpInstance->dpState = kDPMode_EnterDPDone;
                            PD_AltModeStopAMETime(dpInstance->altModeHandle);
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        }
                        break;

                    case kVDM_ExitMode: /* DP UFP */
                        svdmRequest->vdoData = NULL;
                        svdmRequest->vdoCount = 0;
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpState = kDPMode_Exited;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;

                    case kDPVDM_StatusUpdate: /* DP UFP */
                        /* can receive at any time */
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpPartnerStatus.statusVal = svdmRequest->vdoData[0];
                            PD_DpUFPProcessStatusUpdate(dpInstance);
                            svdmRequest->vdoData = (uint32_t *)&dpInstance->dpSelfStatus;
                            svdmRequest->vdoCount = 1;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                            if (dpInstance->dpState == kDPMode_EnterDPDone)
                            {
                                dpInstance->dpState = kDPMode_StatusUpdateDone;
                            }
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;

                    case kDPVDM_Configure: /* DP UFP */
                        /* can receive at any time */
                        if ((dpInstance->dpState == kDPMode_EnterDPDone) ||
                            (dpInstance->dpState == kDPMode_StatusUpdateDone) ||
                            (dpInstance->dpState == kDPMode_ConfigureDone))
                        {
                            dpInstance->dpConfigure.configureVal = svdmRequest->vdoData[0];
                            if ((dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_DFPD) ||
                                (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD))
                            {
                                if (dpInstance->dpState != kDPMode_ConfigureDone)
                                {
                                    dpInstance->dpState = kDPMode_ConfigureDone;
                                }
                            }
                            else
                            {
                                /* back the state */
                                dpInstance->dpState = kDPMode_StatusUpdateDone;
                            }

                            svdmRequest->vdoData = NULL;
                            svdmRequest->vdoCount = 0;
                            svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                            /* after set the displayport signal then ACK */
                            PD_DpUFPProcessConfigure(dpInstance);
                        }
                        else
                        {
                            svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                        }
                        break;

                    case kVDM_Attention: /* DP DFP */
                        if (svdmRequest->vdoCount == 1)
                        {
                            dpInstance->dpPartnerStatus.statusVal = svdmRequest->vdoData[0];
                            /* process DP status */
                            if (dpInstance->dpState == kDPMode_StatusUpdateDone)
                            {
                                if (dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)
                                {
                                    PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_SAFE_MODE, 0);
                                    if (PD_DpDFPConstructConfigure(dpInstance) == kStatus_PD_Success)
                                    {
                                        PD_DpDFPTrigerCommand(dpInstance, kDPVDM_Configure);
                                    }
                                }
                            }
                            else if (dpInstance->dpState == kDPMode_ConfigureDone)
                            {
/* PD_DpModuleSetEvent(dpInstance, DP_TASK_EVENT_DFP_DP_CHECK_UFP_STATUS); */
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
                                PRINTF("receive attention\r\n");
#endif
                                PD_DpDFPProcessUFPstatus(dpInstance);
                            }
                            else
                            {
                                /* don't process */
                            }
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
        }

        case kAltMode_StructedVDMMsgSuccess:
        {
            /* ACK msg, DP DFP */
            /* DFP is doing ASM command */
            if (dpInstance->dfpCommandDoing)
            {
                pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
                dpInstance->dfpCommandDoing = 0;
                status = kStatus_PD_Success;
                switch (svdmResult->vdmCommand)
                {
                    case kVDM_DiscoverModes:
                        dpInstance->dpSelfStatus.bitFields.DFPDUFPDConnected = kDFP_D_Connected;
                        dpInstance->pdMsgReceivedVDOCount = svdmResult->vdoCount;
                        for (index = 0; index < svdmResult->vdoCount; ++index)
                        {
                            dpInstance->pdMsgReceivedBuffer[index] = svdmResult->vdoData[index];
                        }
                        dpInstance->pdVDMMsgReceivedHeader.structuredVdmHeaderVal =
                            svdmResult->vdmHeader.structuredVdmHeaderVal;
                        if (PD_DpDFPProcessModes(dpInstance) == kStatus_PD_Success)
                        {
                            PD_DpDFPTrigerCommand(dpInstance, kVDM_EnterMode);
                        }
                        break;

                    case kVDM_EnterMode:
                        dpInstance->dpState = kDPMode_EnterDPDone;
                        PD_DpDFPTrigerCommand(dpInstance, kDPVDM_StatusUpdate);
                        break;

                    case kDPVDM_StatusUpdate:
                        dpInstance->dpState = kDPMode_StatusUpdateDone;
                        dpInstance->dpPartnerStatus.statusVal = svdmResult->vdoData[0];
                        if (dpInstance->dpPartnerStatus.bitFields.DFPDUFPDConnected & kUFP_D_Connected)
                        {
                            /* if false, wait attention message */
                            PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_SAFE_MODE, 0);
                            if (PD_DpDFPConstructConfigure(dpInstance) == kStatus_PD_Success)
                            {
                                PD_DpDFPTrigerCommand(dpInstance, kDPVDM_Configure);
                            }
                        }
                        break;

                    case kDPVDM_Configure:
                        /* 1. configure as DP; 2. configure as USB (exit DP) */
                        if ((dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_DFPD) ||
                            (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_UFPD))
                        {
                            dpInstance->dpState = kDPMode_ConfigureDone;
                            if ((dpInstance->dpConfigure.bitFields.configureUFPUPin == kPinAssign_C) ||
                                (dpInstance->dpConfigure.bitFields.configureUFPUPin == kPinAssign_E))
                            {
                                PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_DP4_LANE,
                                                  dpInstance->dpConfigure.configureVal);
                            }
                            else
                            {
                                PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_DP2_LANE_USB3,
                                                  dpInstance->dpConfigure.configureVal);
                            }
                            PD_DpHpdDriverReleaseLow(&dpInstance->hpdDriverInstance);
                            /* PD_DpModuleSetEvent(dpInstance, DP_TASK_EVENT_DFP_DP_CHECK_UFP_STATUS); */
                            PD_DpDFPProcessUFPstatus(dpInstance);
#if (defined PD_ALT_MODE_LOG) && (PD_ALT_MODE_LOG)
                            PRINTF("dp configure success\r\n");
#endif
                        }
                        else if (dpInstance->dpConfigure.bitFields.setConfig == kDPConfig_USB)
                        {
                            /* back the state */
                            dpInstance->dpState = kDPMode_StatusUpdateDone;
                            PD_DpHpdDriverSetLow(&dpInstance->hpdDriverInstance);
                            PD_CrossbarSetMux(&dpInstance->crossbarInstance, MUX_USB3_ONLY, 0);
                            PD_DpDFPTrigerCommand(dpInstance, kVDM_ExitMode);
                        }
                        break;

                    case kVDM_ExitMode:
                        PD_DpHpdDriverSetLow(&dpInstance->hpdDriverInstance);
                        dpInstance->dpState = kDPMode_Exited;
                        break;

                    default:
                        break;
                }
            }
            break;
        }

        case kAltMode_StructedVDMMsgFail:
        {
            /* NAK/Not_supported/BUSY/time_out, DP DFP */
            /* DFP is doing ASM command */
            if (dpInstance->dfpCommandDoing)
            {
                pd_svdm_command_result_t *svdmResult = (pd_svdm_command_result_t *)param;
                uint32_t command = 0;
                status = kStatus_PD_Success;
                dpInstance->dfpCommandDoing = 0;
                if (svdmResult->vdmCommandResult == kCommandResult_VDMNAK)
                {
                    /* don't support this command */
                    return status;
                }
                if ((svdmResult->vdmCommand == kVDM_DiscoverModes) || (svdmResult->vdmCommand == kVDM_EnterMode) ||
                    (svdmResult->vdmCommand == kVDM_ExitMode) || (svdmResult->vdmCommand == kDPVDM_StatusUpdate) ||
                    (svdmResult->vdmCommand == kDPVDM_Configure))
                {
                    command = svdmResult->vdmCommand;
                }

                if (command != 0)
                {
                    PD_DpDelayRetryCommand(dpInstance, command, PD_ALT_MODE_ERROR_RETRY_WAIT_TIME);
                }
            }
        }

        case kAltMode_UnstructedVDMMsgReceived:
        case kAltMode_UnstructedVDMMsgSentResult:
            /* DP doesn't have this type message */
            break;

        default:
            break;
    }
    return status;
}

/* 1. send msg from self.
 * wait for send result callback (timer); wait for ACK reply msg callback.
 *
 * 2. ACK received VDM msg.
 * ACK is sent in the callback -> task wait the send result (timer);
 *
 * 3. HPD
 * Dock board detect HPD, Host board driver HPD.
*/
void PD_DPTask(void *taskParam)
{
    pd_alt_mode_dp_t *dpInstance = (pd_alt_mode_dp_t *)taskParam;

    APP_CRITICAL_ALLOC();
    APP_ENTER_CRITICAL();
    if (dpInstance->triggerCommand)
    {
        dpInstance->triggerCommand = 0;
        APP_EXIT_CRITICAL();

        if (dpInstance->retryCountCommand != dpInstance->dfpCommand)
        {
            dpInstance->retryCount = PD_ALT_MODE_COMMAND_RETRY_COUNT;
            dpInstance->retryCountCommand = dpInstance->dfpCommand;
            PD_DpDFPSendCommand(dpInstance, dpInstance->dfpCommand);
        }
        else
        {
            if (dpInstance->retryCount > 0)
            {
                dpInstance->retryCount--;
                PD_DpDFPSendCommand(dpInstance, dpInstance->dfpCommand);
            }
            else
            {
                /* do hard reset */
                PD_Command(dpInstance->pdHandle, PD_DPM_CONTROL_HARD_RESET, NULL);
            }
        }
    }
    else
    {
        APP_EXIT_CRITICAL();
    }
    /* exit critical */
    PD_DpHpdDrvierProcess(&dpInstance->hpdDriverInstance);
}
