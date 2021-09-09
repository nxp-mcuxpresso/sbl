/*
 * The Clear BSD License
 * Copyright 2016 - 2017 NXP
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

#include <stdint.h>
#include <stdbool.h>
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "pd_app.h"
#include "fsl_debug_console.h"
#include "pd_power_interface.h"
#include "pd_command_interface.h"
#include "pd_board_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

pd_status_t PD_DemoFindPDO(
    pd_app_t *pdAppInstance, pd_rdo_t *rdo, uint32_t requestVoltagemV, uint32_t requestCurrentmA, uint32_t *voltage);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

pd_status_t PD_DpmSoftResetCallback(void *callbackParam)
{
    /* reset soft status */
    PRINTF("app soft reset\r\n");
    return kStatus_PD_Success;
}

pd_status_t PD_DpmHardResetCallback(void *callbackParam)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    /* reset state, The Sink shall not draw more than iSafe0mA when VBUS is driven to vSafe0V. */
    pdAppInstance->selfHasEnterAlernateMode = 0;
    PRINTF("hard reset request\r\n");
    return kStatus_PD_Success;
}

pd_status_t PD_DpmPowerRoleSwapRequestCallback(void *callbackParam, uint8_t frSwap, uint8_t *evaluateResult)
{
#if (defined PD_CONFIG_COMPLIANCE_TEST_ENABLE) && (PD_CONFIG_COMPLIANCE_TEST_ENABLE)
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    uint8_t powerRole;
    pd_source_pdo_t pdo;

    if (frSwap)
    {
        *evaluateResult = kCommandResult_Accept;
    }
    else
    {
        PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_POWER_ROLE, &powerRole);

        pdo.PDOValue = ((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sourceCaps[0];
        if ((powerRole == kPD_PowerRoleSource) && (pdo.fixedPDO.externalPowered))
        {
            if ((pdAppInstance->partnerSourceCapNumber == 0) && (pdAppInstance->partnerSinkCapNumber == 0))
            {
                *evaluateResult = kCommandResult_Wait;
                PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES, NULL);
            }
            else
            {
                if (pdAppInstance->partnerSourceCapNumber)
                {
                    if (pdAppInstance->partnerSourceCaps[0].fixedPDO.externalPowered == 0)
                    {
                        *evaluateResult = kCommandResult_Reject;
                    }
                    else
                    {
                        *evaluateResult = kCommandResult_Accept;
                    }
                }
                else
                {
                    if (pdAppInstance->partnerSinkCaps[0].fixedPDO.externalPowered == 0)
                    {
                        *evaluateResult = kCommandResult_Reject;
                    }
                    else
                    {
                        *evaluateResult = kCommandResult_Accept;
                    }
                }
            }
        }
        else
        {
            *evaluateResult = kCommandResult_Accept;
        }
    }
#else
    *evaluateResult = kCommandResult_Accept;
#endif
    return kStatus_PD_Success;
}

pd_status_t PD_DpmPowerRoleSwapResultCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    uint8_t roleInfo;

    if (success)
    {
        PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_POWER_ROLE, &roleInfo);
        if (roleInfo == kPD_PowerRoleSource)
        {
            PRINTF("enter source\r\n");
        }
        else
        {
            PRINTF("enter sink\r\n");
        }
    }
    else
    {
        switch (failResultType)
        {
            case kCommandResult_Reject:
                PRINTF("power role swap result: reject\r\n");
                break;

            case kCommandResult_Wait:
                PRINTF("power role swap result: wait\r\n");
                break;

            case kCommandResult_Error:
                PRINTF("power role swap result: fail\r\n");
                break;

            case kCommandResult_NotSupported:
                break;

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmDataRoleSwapRequestCallback(void *callbackParam, uint8_t *evaluateResult)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    *evaluateResult = ((pdAppInstance->drSwapAccept) ? kCommandResult_Accept : kCommandResult_Reject);
    return kStatus_PD_Success;
}

pd_status_t PD_DpmDataRoleSwapResultCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    uint8_t roleInfo;

    if (success)
    {
        PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &roleInfo);
        if (roleInfo == kPD_DataRoleDFP)
        {
            PRINTF("dr swap result: enter dfp\r\n");
        }
        else
        {
            PRINTF("dr swap result: enter ufp\r\n");
        }
    }
    else
    {
        switch (failResultType)
        {
            case kCommandResult_Error:
                PRINTF("dr swap result: fail\r\n");
                break;

            case kCommandResult_Reject:
                PRINTF("dr swap result: reject\r\n");
                break;

            case kCommandResult_Wait:
                PRINTF("dr swap result: wait\r\n");
                break;

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmVconnSwapRequestCallback(void *callbackParam, uint8_t *evaluateResult)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    *evaluateResult = ((pdAppInstance->vconnSwapAccept) ? kCommandResult_Accept : kCommandResult_Reject);
    return kStatus_PD_Success;
}

pd_status_t PD_DpmVconnSwapResultCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    uint8_t roleInfo;

    if (success)
    {
        PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_VCONN_ROLE, &roleInfo);
        if (roleInfo == kPD_IsVconnSource)
        {
            PRINTF("vconn swap result: turn as source\r\n");
        }
        else
        {
            PRINTF("vconn swap result: not vconn source\r\n");
        }
    }
    else
    {
        switch (failResultType)
        {
            case kCommandResult_Reject:
                PRINTF("vconn swap result: reject\r\n");
                break;

            case kCommandResult_Wait:
                PRINTF("vconn swap result: wait\r\n");
                break;

            case kCommandResult_Error:
                PRINTF("vconn swap result: fail\r\n");
                break;

            case kCommandResult_NotSupported:
                break;

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSrcRDORequestCallback(void *callbackParam, pd_rdo_t rdo, uint8_t *negotiateResult)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    pd_source_pdo_t pdo;
    uint8_t accept = 0;

    /* prepare for power supply, The power supply shall be ready to operate at the new power level within
     * tSrcReady */
    if (rdo.bitFields.objectPosition == 1)
    {
        pdo.PDOValue = ((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sourceCaps[0];
        if (rdo.bitFields.operateValue <= pdo.fixedPDO.maxCurrent)
        {
            accept = 1;
        }
    }
    else if ((rdo.bitFields.objectPosition == 2) &&
             (((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sourceCapCount >= 2))
    {
        pdo.PDOValue = ((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sourceCaps[1];
        if (rdo.bitFields.operateValue <= pdo.fixedPDO.maxCurrent)
        {
            accept = 1;
        }
    }
    else
    {
    }

    if (accept)
    {
        pdAppInstance->sinkRequestRDO.rdoVal = rdo.rdoVal;
        *negotiateResult = kCommandResult_Accept;
    }
    else
    {
        *negotiateResult = kCommandResult_Reject;
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSrcPreContractStillValidCallback(void *callbackParam, uint8_t *isStillValid)
{
    /* if pre contract exist, return true */
    *isStillValid = 1;
    return kStatus_PD_Success;
}

pd_status_t PD_DpmSrcRDOResultCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    if (success)
    {
        PRINTF("partner sink's request %dV success\r\n",
               (pdAppInstance->sinkRequestRDO.bitFields.objectPosition == 1) ? 5 : 9);
    }
    else
    {
        switch (failResultType)
        {
            case kCommandResult_Error:
                PRINTF("source has error in power negotiation\r\n");
                break;

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmReceivePartnerSrcCapsCallback(void *callbackParam, pd_capabilities_t *caps)
{
    uint32_t index;
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    pd_source_pdo_t sourcePDO;
    PRINTF("receive source capabilities:\r\n");
    pdAppInstance->partnerSourceCapNumber = caps->capabilitiesCount;
    for (index = 0; index < pdAppInstance->partnerSourceCapNumber; ++index)
    {
        pdAppInstance->partnerSourceCaps[index].PDOValue = caps->capabilities[index];
        sourcePDO.PDOValue = caps->capabilities[index];
        switch (sourcePDO.commonPDO.pdoType)
        {
            case kPDO_Fixed:
            {
                PRINTF("%d: fixed PDO; ", index + 1);
                PRINTF("vol:%dmV, current:%dmA\r\n", sourcePDO.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT,
                       sourcePDO.fixedPDO.maxCurrent * PD_PDO_CURRENT_UNIT);
                break;
            }

            case kPDO_Variable:
            {
                PRINTF("%d: variable PDO; ", index + 1);
                PRINTF("vol:%dmV ~ %dmV, current:%dmA\r\n", sourcePDO.variablePDO.maxVoltage * PD_PDO_VOLTAGE_UNIT,
                       sourcePDO.variablePDO.minVoltage * PD_PDO_VOLTAGE_UNIT,
                       sourcePDO.variablePDO.maxCurrent * PD_PDO_CURRENT_UNIT);
                break;
            }

            case kPDO_Battery:
            {
                PRINTF("%d: battery PDO; ", index + 1);
                PRINTF("vol:%dmV ~ %dmV, power:%dmW\r\n", sourcePDO.batteryPDO.maxVoltage * PD_PDO_VOLTAGE_UNIT,
                       sourcePDO.batteryPDO.minVoltage * PD_PDO_VOLTAGE_UNIT,
                       sourcePDO.batteryPDO.maxAllowPower * PD_PDO_POWER_UNIT);
                break;
            }

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmGetPartnerSrcCapsFailCallback(void *callbackParam, uint8_t failResultType)
{
    switch (failResultType)
    {
        case kCommandResult_Error:
            PRINTF("get src cap fail");
            break;

        case kCommandResult_NotSupported:
            PRINTF("get src cap replying not supported");
            break;

        default:
            break;
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSnkGetRequestRDOCallback(void *callbackParam, pd_rdo_t *rdo)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    uint32_t voltage;

    if (PD_DemoFindPDO(pdAppInstance, rdo, PD_DEMO_EXPECTED_VOLTAGE, PD_DEMO_EXPECTED_CURRENT, &voltage) !=
        kStatus_PD_Success)
    {
        PRINTF("cap mismatch\r\n");
    }
    pdAppInstance->sinkRequestRDO = *rdo;
    pdAppInstance->sinkRequestVoltage = voltage;

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSnkRDOResultCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    if (success)
    {
        PRINTF("sink request %dV success\r\n", pdAppInstance->sinkRequestVoltage / 1000);
    }
    else
    {
        switch (failResultType)
        {
            case kCommandResult_Error:
                PRINTF("sink request power result: fail\r\n");
                break;

            case kCommandResult_Reject:
                PRINTF("sink request power result: reject\r\n");
                break;

            case kCommandResult_Wait:
                PRINTF("sink request power result: wait\r\n");
                break;

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSrcGotoMinResultCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    if (success)
    {
        PRINTF("success\r\n");
    }
    else
    {
        PRINTF("fail\r\n");
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSnkGotoMinResultCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    if (success)
    {
        PRINTF("success\r\n");
    }
    else
    {
        PRINTF("fail\r\n");
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmReceivePartnerSnkCapsCallback(void *callbackParam, pd_capabilities_t *caps)
{
    uint32_t index;
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    pd_sink_pdo_t sinkPDO;
    PRINTF("receive sink capabilities:\r\n");
    pdAppInstance->partnerSinkCapNumber = caps->capabilitiesCount;
    for (index = 0; index < pdAppInstance->partnerSinkCapNumber; ++index)
    {
        pdAppInstance->partnerSinkCaps[index].PDOValue = caps->capabilities[index];
        sinkPDO.PDOValue = caps->capabilities[index];
        switch (sinkPDO.commonPDO.pdoType)
        {
            case kPDO_Fixed:
            {
                PRINTF("%d: fixed PDO; ", index + 1);
                PRINTF("vol:%dmV, current:%dmA\r\n", sinkPDO.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT,
                       sinkPDO.fixedPDO.operateCurrent * PD_PDO_CURRENT_UNIT);
                break;
            }

            case kPDO_Variable:
            {
                PRINTF("%d: variable PDO; ", index + 1);
                PRINTF("vol:%dmV ~ %dmV, current:%dmA\r\n", sinkPDO.variablePDO.maxVoltage * PD_PDO_VOLTAGE_UNIT,
                       sinkPDO.variablePDO.minVoltage * PD_PDO_VOLTAGE_UNIT,
                       sinkPDO.variablePDO.operateCurrent * PD_PDO_CURRENT_UNIT);
                break;
            }

            case kPDO_Battery:
            {
                PRINTF("%d: battery PDO; ", index + 1);
                PRINTF("vol:%dmV ~ %dmV, power:%dmW\r\n", sinkPDO.batteryPDO.maxVoltage * PD_PDO_VOLTAGE_UNIT,
                       sinkPDO.batteryPDO.minVoltage * PD_PDO_VOLTAGE_UNIT,
                       sinkPDO.batteryPDO.operatePower * PD_PDO_POWER_UNIT);
                break;
            }

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmGetPartnerSnkCapsFailCallback(void *callbackParam, uint8_t failResultType)
{
    switch (failResultType)
    {
        case kCommandResult_Error:
            PRINTF("get snk cap fail");
            break;

        case kCommandResult_NotSupported:
            PRINTF("get snk cap replying not supported");
            break;

        default:
            break;
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSVDMRequestCallback(void *callbackParam, pd_svdm_command_request_t *svdmRequest)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    /* common process */
    switch (svdmRequest->vdmHeader.bitFields.command)
    {
        case kVDM_DiscoverIdentity:
        case kVDM_DiscoverSVIDs:
            break;

        case kVDM_DiscoverModes:
        case kVDM_EnterMode:
        case kVDM_ExitMode:
            if (svdmRequest->vdmHeader.bitFields.SVID != PD_VENDOR_VID)
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                return kStatus_PD_Success;
            }
            break;

        default:
            break;
    }

    /* ack or nak, no busy */
    /* partner return nak if it is not in the alternate mode */
    switch (svdmRequest->vdmHeader.bitFields.command)
    {
        case kVDM_DiscoverIdentity:
            svdmRequest->vdoData = (uint32_t *)&pdAppInstance->selfVdmIdentity;
            svdmRequest->vdoCount = sizeof(pdAppInstance->selfVdmIdentity) / 4;
            if (pdAppInstance->reqestResponse == kCommandResult_Accept)
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMACK;
            }
            else
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMBUSY;
            }
            break;

        case kVDM_DiscoverSVIDs:
            svdmRequest->vdoData = (uint32_t *)&pdAppInstance->selfVdmSVIDs;
            svdmRequest->vdoCount = 1;
            if (pdAppInstance->reqestResponse == kCommandResult_Accept)
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMACK;
            }
            else
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMBUSY;
            }
            break;

        case kVDM_DiscoverModes:
            svdmRequest->vdoData = (uint32_t *)&pdAppInstance->selfVdmModes;
            svdmRequest->vdoCount = 1;
            if (pdAppInstance->reqestResponse == kCommandResult_Accept)
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMACK;
            }
            else
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMBUSY;
            }
            break;

        case kVDM_EnterMode:
            svdmRequest->vdoData = NULL;
            svdmRequest->vdoCount = 0;
            if (pdAppInstance->reqestResponse == kCommandResult_Accept)
            {
                if (svdmRequest->vdmHeader.bitFields.objPos == 1)
                {
                    svdmRequest->requestResultStatus = kCommandResult_VDMACK;
                    pdAppInstance->selfHasEnterAlernateMode = 1;
                }
                else
                {
                    svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
                    pdAppInstance->selfHasEnterAlernateMode = 1;
                }
            }
            else
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMBUSY;
            }
            break;

        case kVDM_ExitMode:
            svdmRequest->vdoData = NULL;
            svdmRequest->vdoCount = 0;
            if ((pdAppInstance->selfHasEnterAlernateMode == 1) && (svdmRequest->vdmHeader.bitFields.objPos == 1))
            {
                pdAppInstance->selfHasEnterAlernateMode = 0;
                svdmRequest->requestResultStatus = kCommandResult_VDMACK;
            }
            else
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMNAK;
            }
            break;

        case kVDM_Attention:
        {
            uint8_t dataRole;

            PRINTF("receive attention\r\n");
            pdAppInstance->structuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
            pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.objPos =
                svdmRequest->vdmHeader.bitFields.objPos;
            pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.SVID = svdmRequest->vdmHeader.bitFields.SVID;

            PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);
            if (dataRole == kPD_DataRoleDFP)
            {
                PRINTF("start exit mode command\r\n");
                if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_EXIT_MODE,
                               &pdAppInstance->structuredVDMCommandParam) != kStatus_PD_Success)
                {
                    PRINTF("command start fail\r\n");
                }
            }
            break;
        }

        default:
            /* vendor defined structured vdm */
            if (pdAppInstance->reqestResponse == kCommandResult_Accept)
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMACK;
            }
            else
            {
                svdmRequest->requestResultStatus = kCommandResult_VDMBUSY;
            }
            break;
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSVDMResultCallback(void *callbackParam, uint8_t success, pd_svdm_command_result_t *svdmResult)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    if (success)
    {
        switch (svdmResult->vdmCommand)
        {
            case kVDM_DiscoverIdentity:
            {
                pd_id_header_vdo_t idHeaderVDO;
                idHeaderVDO.vdoValue = USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(((uint8_t *)(svdmResult->vdoData)));
                PRINTF("vendor:%x, modual support:%d, usb communication capable device:%d",
                       idHeaderVDO.bitFields.usbVendorID, idHeaderVDO.bitFields.modalOperateSupport,
                       idHeaderVDO.bitFields.usbCommunicationCapableAsDevice);
                PRINTF(", usb communication capable host:%d", idHeaderVDO.bitFields.usbCommunicationCapableAsHost);
                PRINTF(", product type: (DFP or Cable)");
                if (idHeaderVDO.bitFields.productTypeDFP != 0)
                {
                    switch (idHeaderVDO.bitFields.productTypeDFP)
                    {
                        case 0:
                            PRINTF("undefined, ");
                            break;

                        case 1:
                            PRINTF("PDUSB Hub, ");
                            break;

                        case 2:
                            PRINTF("PDUSB Host, ");
                            break;

                        case 3:
                            PRINTF("Power Brick, ");
                            break;

                        case 4:
                            PRINTF("Alternate Mode Controller (AMC), ");
                            break;

                        default:
                            PRINTF("cannot recognition, ");
                            break;
                    }
                }

                PRINTF("(UFP)");
                if (idHeaderVDO.bitFields.productTypeUFPOrCablePlug != 0)
                {
                    switch (idHeaderVDO.bitFields.productTypeUFPOrCablePlug)
                    {
                        case 0:
                            PRINTF("undefined\r\n");
                            break;

                        case 1:
                            PRINTF("PDUSB Hub\r\n");
                            break;

                        case 2:
                            PRINTF("PDUSB Peripheral\r\n");
                            break;

                        case 3:
                            PRINTF("Passive Cable\r\n");
                            break;

                        case 4:
                            PRINTF("Active Cable\r\n");
                            break;

                        case 5:
                            PRINTF("Alternate Mode Adapter (AMA)\r\n");
                            break;

                        default:
                            PRINTF("cannot recognition\r\n");
                            break;
                    }
                }

                PRINTF("(2) discovery SVIDs\r\n");
                pdAppInstance->structuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
                if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_DISCOVERY_SVIDS,
                               &pdAppInstance->structuredVDMCommandParam) != kStatus_PD_Success)
                {
                    PRINTF("command fail\r\n");
                }
                break;
            }

            case kVDM_DiscoverSVIDs:
            {
                uint32_t index;
                uint8_t svidIndex = 1;
                uint32_t SVID;
                uint8_t *buffPtr = (uint8_t *)svdmResult->vdoData;
                for (index = 0; index < (svdmResult->vdoCount); ++index)
                {
                    SVID = USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(buffPtr);
                    buffPtr += 4;
                    if (SVID != 0)
                    {
                        pdAppInstance->partnerSVIDs[index * 2] = SVID >> 16;
                        if (pdAppInstance->partnerSVIDs[index * 2] != 0x0000u)
                        {
                            PRINTF("SVID%d: %x\r\n", svidIndex, pdAppInstance->partnerSVIDs[index * 2]);
                            svidIndex++;
                        }

                        pdAppInstance->partnerSVIDs[index * 2 + 1] = (SVID & 0x0000FFFFu);
                        if (pdAppInstance->partnerSVIDs[index * 2 + 1] != 0x0000u)
                        {
                            PRINTF("SVID%d: %x\r\n", svidIndex, pdAppInstance->partnerSVIDs[index * 2 + 1]);
                            svidIndex++;
                        }
                    }

                    pdAppInstance->partnerSVIDs[index * 2 + 1] = USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(buffPtr);
                }

                if (pdAppInstance->partnerSVIDs[0] == 0x0000u)
                {
                    PRINTF("SVIDs: none\r\n");
                }
                else
                {
                    PRINTF("(3) discovery Modes\r\n");
                    pdAppInstance->structuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
                    pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.SVID = pdAppInstance->partnerSVIDs[0];
                    if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_DISCOVERY_MODES,
                                   &pdAppInstance->structuredVDMCommandParam) != kStatus_PD_Success)
                    {
                        PRINTF("command fail\r\n");
                    }
                }
                break;
            }

            case kVDM_DiscoverModes:
            {
                uint32_t index;
                uint8_t *buffPtr = (uint8_t *)svdmResult->vdoData;
                uint8_t dataRole;
                for (index = 0; index < (svdmResult->vdoCount); ++index)
                {
                    pdAppInstance->partnerModes[index * 2] = USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(buffPtr);
                    buffPtr += 2;
                    PRINTF("Mode%d: %x\r\n", 1 + index * 2, pdAppInstance->partnerModes[index * 2]);
                    pdAppInstance->partnerModes[index * 2 + 1] = USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(buffPtr);
                    buffPtr += 2;
                    PRINTF("Mode%d: %x\r\n", 1 + index * 2 + 1, pdAppInstance->partnerModes[index * 2 + 1]);
                }

                PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);

                if (dataRole == kPD_DataRoleDFP)
                {
                    PRINTF("(4) enter mode 1\r\n");
                    pdAppInstance->structuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
                    pdAppInstance->structuredVDMCommandParam.vdoCount = 0;
                    pdAppInstance->structuredVDMCommandParam.vdoData = NULL;
                    pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.objPos = 1;
                    pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.SVID = pdAppInstance->partnerSVIDs[0];
                    if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_ENTER_MODE,
                                   &pdAppInstance->structuredVDMCommandParam) != kStatus_PD_Success)
                    {
                        PRINTF("command fail\r\n");
                    }
                }
                break;
            }

            case kVDM_EnterMode:
                PRINTF("enter mode result: ack\r\n");
                break;

            case kVDM_ExitMode:
                PRINTF("exit mode result: ack\r\n");
                break;

            case kVDM_Attention:
                PRINTF("send attention result: success\r\n");
                break;

            default:
            {
                /* process the buffer data and length */
                /* pd_svdm_command_result_t *vdmResult = (pd_svdm_command_result_t*)param; */
                PRINTF("vendor structured vdm result: success\r\n");
                break;
            }
        }
    }
    else
    {
        switch (svdmResult->vdmCommandResult)
        {
            case kCommandResult_Error:
                PRINTF("structured vdm result: fail\r\n");
                break;

            case kCommandResult_VDMNAK:
                PRINTF("structured vdm result: nak\r\n");
                break;

            case kCommandResult_VDMBUSY:
                PRINTF("structured vdm result: busy\r\n");
                break;

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmUnstructuredVDMReceivedCallback(void *callbackParam,
                                                  pd_unstructured_vdm_command_param_t *unstructuredVDMParam)
{
    uint32_t index;

    /* process the unstructured vdm */
    PRINTF("receive unstructured vdm, sop:%d, vdo count:%d\r\n", unstructuredVDMParam->vdmSop,
           unstructuredVDMParam->vdmHeaderAndVDOsCount);
    PRINTF("VDO Header:%x\r\n", unstructuredVDMParam->vdmHeaderAndVDOsData[0]);
    for (index = 1; index < unstructuredVDMParam->vdmHeaderAndVDOsCount; ++index)
    {
        PRINTF("VDO%d:%d\r\n", index, unstructuredVDMParam->vdmHeaderAndVDOsData[index]);
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmUnstructuredVDMSendResultCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    if (success)
    {
        PRINTF("send unstructured vdm result: success\r\n");
    }
    else
    {
        PRINTF("send unstructured vdm result: fail\r\n");
    }

    return kStatus_PD_Success;
}

#if ((defined PD_CONFIG_REVISION) && (PD_CONFIG_REVISION >= PD_SPEC_REVISION_30))
pd_status_t PD_DpmGetInfoRequestCallback(void *callbackParam, uint8_t type, pd_command_data_param_t *dataParam)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    switch (type)
    {
        case kInfoType_SrcExtCap:
            if (pdAppInstance->reqestResponse != kCommandResult_Accept)
            {
                dataParam->resultStatus = kCommandResult_NotSupported;
            }
            else
            {
                dataParam->resultStatus = kCommandResult_Accept;
            }

            dataParam->dataBuffer = (uint8_t *)&pdAppInstance->selfExtCap;
            dataParam->dataLength = sizeof(pdAppInstance->selfExtCap);
            break;

        case kInfoType_Status:
            if (pdAppInstance->reqestResponse != kCommandResult_Accept)
            {
                dataParam->resultStatus = kCommandResult_NotSupported;
            }
            else
            {
                dataParam->resultStatus = kCommandResult_Accept;
            }

            dataParam->dataBuffer = (uint8_t *)&pdAppInstance->selfStatus;
            dataParam->dataLength = sizeof(pdAppInstance->selfStatus);
            break;

        case kInfoType_BatteryCap:
            if (pdAppInstance->reqestResponse != kCommandResult_Accept)
            {
                dataParam->resultStatus = kCommandResult_NotSupported;
            }
            else
            {
                dataParam->resultStatus = kCommandResult_Accept;
            }
            dataParam->dataBuffer = (uint8_t *)&pdAppInstance->selfBatteryCap;
            dataParam->dataLength = sizeof(pdAppInstance->selfBatteryCap);
            break;

        case kInfoType_BatteryStatus:
            if (pdAppInstance->reqestResponse != kCommandResult_Accept)
            {
                dataParam->resultStatus = kCommandResult_NotSupported;
            }
            else
            {
                dataParam->resultStatus = kCommandResult_Accept;
            }
            dataParam->dataBuffer = (uint8_t *)&pdAppInstance->selfBatteryStatus;
            dataParam->dataLength = sizeof(pdAppInstance->selfBatteryStatus);
            break;

        case kInfoType_ManufacturerInfo:
            if (pdAppInstance->reqestResponse != kCommandResult_Accept)
            {
                dataParam->resultStatus = kCommandResult_NotSupported;
            }
            else
            {
                dataParam->resultStatus = kCommandResult_Accept;
            }
            dataParam->dataBuffer = (uint8_t *)&pdAppInstance->selfManufacInfo;
            dataParam->dataLength = 7;
            break;

        default:
            break;
    }
    return kStatus_PD_Success;
}

pd_status_t PD_DpmGetInfoResultCallback(
    void *callbackParam, uint8_t type, uint8_t success, pd_command_data_param_t *successData, uint8_t failResultType)
{
    if (success)
    {
        switch (type)
        {
            case kInfoType_SrcExtCap:
            {
                pd_source_cap_ext_data_block_t *extDataBlock;
                extDataBlock = (pd_source_cap_ext_data_block_t *)successData->dataBuffer;
                PRINTF("vid:%x, pid:%x, xid:%x, fw version:%d, hw version:%d\r\n", (uint16_t)extDataBlock->vid,
                       (uint16_t)extDataBlock->pid,
                       USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(((uint8_t *)&(extDataBlock->xid))),
                       (uint8_t)extDataBlock->fwVersion, (uint8_t)extDataBlock->hwVersion);
                break;
            }

            case kInfoType_Status:
            {
                pd_status_data_block_t *status = (pd_status_data_block_t *)(&successData->dataBuffer[0]);
                PRINTF("Internal Temp:%d\r\n", status->internalTemp);
                break;
            }

            case kInfoType_BatteryCap:
            {
                pd_battery_cap_data_block_t *partnerBatteryCap =
                    (pd_battery_cap_data_block_t *)(successData->dataBuffer);
                PRINTF("design cap:%dmWH, last full cap:%d\r\n", partnerBatteryCap->batteryDesignCap * 100,
                       partnerBatteryCap->batteryLastFullChargeCap * 100);
                break;
            }

            case kInfoType_BatteryStatus:
            {
                pd_battery_status_data_object_t *partnerBatteryStatus =
                    (pd_battery_status_data_object_t *)(successData->dataBuffer);
                PRINTF("battery charge state:%dmWH, last full cap:%d\r\n", partnerBatteryStatus->batteryPC * 100);
                break;
            }

            case kInfoType_ManufacturerInfo:
            {
                pd_manufac_info_data_block_t *partnerManufacturerInfo =
                    (pd_manufac_info_data_block_t *)(successData->dataBuffer);
                partnerManufacturerInfo->manufacturerString[successData->dataLength - 4] = 0;
                PRINTF("manufacturer string:%s\r\n", partnerManufacturerInfo->manufacturerString);
                break;
            }

            default:
                break;
        }
    }
    else
    {
        switch (failResultType)
        {
            case kCommandResult_Error:
                PRINTF("get info fail\r\n");
                break;

            case kCommandResult_NotSupported:
                PRINTF("get info is not supported by partner\r\n");
                break;

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmReceiveAlertCallback(void *callbackParam, pd_command_data_param_t *dataParam)
{
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;
    pd_alert_data_object_t alertObj;

    dataParam->resultStatus = kCommandResult_Accept;
    alertObj.alertValue = USB_LONG_FROM_LITTLE_ENDIAN_ADDRESS(dataParam->dataBuffer);
    PRINTF("alert change:%x\r\n", alertObj.bitFields.typeOfAlert);

    PRINTF("start get status command\r\n");
    if (PD_Command(pdAppInstance->pdHandle, PD_DPM_GET_STATUS, NULL) != kStatus_PD_Success)
    {
        PRINTF("command start fail\r\n");
    }

    return kStatus_PD_Success;
}

pd_status_t PD_DpmSendAlertCallback(void *callbackParam, uint8_t success, uint8_t failResultType)
{
    if (success)
    {
        PRINTF("send alert result: success\r\n");
    }
    else
    {
        switch (failResultType)
        {
            case kCommandResult_Error:
                PRINTF("send alert result: fail\r\n");
                break;

            case kCommandResult_NotSupported:
                PRINTF("send alert result: not supported\r\n");
                break;

            default:
                break;
        }
    }

    return kStatus_PD_Success;
}
#endif
