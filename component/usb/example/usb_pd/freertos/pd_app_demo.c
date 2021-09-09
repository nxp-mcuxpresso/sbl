/*
 * The Clear BSD License
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

#include <stdarg.h>
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "board.h"
#include "pd_app.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "pd_power_interface.h"
#include "pd_board_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#if (defined BOARD_PD_SW_INPUT_SUPPORT) && (BOARD_PD_SW_INPUT_SUPPORT)
uint8_t HW_GpioReadPowerRequestSW(void);
uint8_t HW_GpioReadPowerChangeSW(void);
#endif
pd_status_t PD_DemoFindPDO(
    pd_app_t *pdAppInstance, pd_rdo_t *rdo, uint32_t requestVoltagemV, uint32_t requestCurrentmA, uint32_t *voltage);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static void USB_PDDemoRequest5V(pd_app_t *pdAppInstance)
{
    pdAppInstance->sinkRequestVoltage = 5000;
    pdAppInstance->sinkRequestRDO.bitFields.objectPosition = 1;
    pdAppInstance->sinkRequestRDO.bitFields.giveBack = 0;
    pdAppInstance->sinkRequestRDO.bitFields.capabilityMismatch = 0;
    pdAppInstance->sinkRequestRDO.bitFields.usbCommunicationsCapable = 0;
    pdAppInstance->sinkRequestRDO.bitFields.noUsbSuspend = 1;
    pdAppInstance->sinkRequestRDO.bitFields.operateValue = 270; /* 2.7A */
    pdAppInstance->sinkRequestRDO.bitFields.maxOrMinOperateValue = 270;
}

static uint8_t USB_PDDemoRequestHighVoltage(pd_app_t *pdAppInstance)
{
    pd_rdo_t rdo;
    uint8_t snkCapIndex;
    uint32_t voltage;
    pd_sink_pdo_t pdo;

    for (snkCapIndex = (((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sinkCapCount - 1);
         snkCapIndex > 0; --snkCapIndex)
    {
        pdo.PDOValue = ((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sinkCaps[snkCapIndex];
        if (PD_DemoFindPDO(pdAppInstance, &rdo, pdo.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT, 0u, &voltage) ==
            kStatus_PD_Success)
        {
            break;
        }
    }

    if (snkCapIndex > 0)
    {
        pdAppInstance->sinkRequestVoltage = voltage;
        pdAppInstance->sinkRequestRDO.rdoVal = rdo.rdoVal;
        return 1;
    }
    else
    {
        return 0;
    }
}

static void USB_PDDemoPrintMenu(pd_app_t *pdAppInstance)
{
    uint8_t powerRole;
    uint8_t dataRole;

    PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_POWER_ROLE, &powerRole);
    PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);

    if (powerRole == kPD_PowerRoleSource)
    {
        /* source role */
        PRINTF("The menu is as follow for source:\r\n");
        PRINTF("0. print menu\r\n");
        PRINTF("a. source power change\r\n");
        PRINTF("b. goto min\r\n");
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
        PRINTF("c. fast role swap\r\n");
#endif
    }
    else
    {
        /* sink role */
        PRINTF("The menu is as follow for sink:\r\n");
        PRINTF("0. print menu\r\n");
        PRINTF("a. get partner source capabilities\r\n");
        PRINTF("b. request 5V\r\n");
        if (((pd_power_port_config_t *)pdAppInstance->pdConfigParam->deviceConfig)->sinkCapCount > 1)
        {
            PRINTF("c. request high voltage\r\n");
        }
    }

    PRINTF("d. hard reset\r\n");
    PRINTF("e. soft reset\r\n");
    PRINTF("f. power role swap\r\n");
    PRINTF("g. data role swap\r\n");
    PRINTF("h. vconn swap\r\n");
    PRINTF("i. get partner sink capabilities\r\n");
    PRINTF("j. standard structured VDM test (only DFP can send enter mode)\r\n");
    if (dataRole == kPD_DataRoleDFP)
    {
        PRINTF("k. exit mode (only DFP)\r\n");
    }
    PRINTF("l. send attention\r\n");
    PRINTF("m. test vendor structured VDM\r\n");
    PRINTF("n. test unstructured VDM\r\n");
#if ((defined PD_CONFIG_REVISION) && (PD_CONFIG_REVISION >= PD_SPEC_REVISION_30))
    PRINTF("o. get source extended capabilities\r\n");
    PRINTF("p. get status\r\n");
    PRINTF("q. alert\r\n");
    PRINTF("r. get battery capabilities\r\n");
    PRINTF("s. get battery status\r\n");
    PRINTF("t. get manufacturer info\r\n");
#endif
    PRINTF("u. cable reset  (not supported yet)\r\n");
}

static void USB_PDDemoProcessMenu(pd_app_t *pdAppInstance, char ch)
{
    uint8_t powerRole;
    uint8_t dataRole;
    uint32_t commandValid;
    void *commandParam;
    pd_unstructured_vdm_command_param_t unstructuredVDMCommandParam;
#if ((defined PD_CONFIG_REVISION) && (PD_CONFIG_REVISION >= PD_SPEC_REVISION_30))
    pd_command_data_param_t extCommandParam;
#endif

    PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_POWER_ROLE, &powerRole);
    /* function codes */
    if ((ch >= '0') && (ch <= '9'))
    {
        commandValid = 0;
        switch (ch)
        {
            case '0':
                USB_PDDemoPrintMenu(pdAppInstance);
                break;

            default:
                break;
        }
    }
    else if ((ch >= 'a') && (ch <= 'z'))
    {
        PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &dataRole);
        commandValid = 0;

        switch (ch)
        {
            case 'a':
                if (powerRole == kPD_PowerRoleSource)
                {
                    PRINTF("a. source power change\r\n");
                    commandValid = PD_DPM_CONTROL_POWER_NEGOTIATION;
                    commandParam = NULL;
                }
                else
                {
                    PRINTF("a. get partner source capabilities\r\n");
                    commandValid = PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES;
                    commandParam = NULL;
                }
                break;

            case 'b':
                if (powerRole == kPD_PowerRoleSource)
                {
                    PRINTF("b. goto min\r\n");
                    if (!(pdAppInstance->sinkRequestRDO.bitFields.giveBack))
                    {
                        PRINTF("warning: the GiveBack flag is not set\r\n");
                    }
                    commandValid = PD_DPM_CONTROL_GOTO_MIN;
                    commandParam = NULL;
                }
                else
                {
                    PRINTF("b. request 5V\r\n");
                    USB_PDDemoRequest5V(pdAppInstance);
                    commandValid = PD_DPM_CONTROL_REQUEST;
                    commandParam = &pdAppInstance->sinkRequestRDO;
                }
                break;

            case 'c':
                if (powerRole == kPD_PowerRoleSource)
                {
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                    PRINTF("c. fast role swap\r\n");
                    commandValid = PD_DPM_FAST_ROLE_SWAP;
                    commandParam = NULL;
#endif
                }
                else
                {
                    PRINTF("c. request high voltage\r\n");
                    if (USB_PDDemoRequestHighVoltage(pdAppInstance))
                    {
                        commandValid = PD_DPM_CONTROL_REQUEST;
                        commandParam = &pdAppInstance->sinkRequestRDO;
                    }
                    else
                    {
                        PRINTF("partner don't support");
                    }
                }
                break;

            case 'd':
                PRINTF("d. hard reset\r\n");
                commandValid = PD_DPM_CONTROL_HARD_RESET;
                commandParam = NULL;
                break;

            case 'e':
                PRINTF("e. soft reset\r\n");
                commandValid = PD_DPM_CONTROL_SOFT_RESET;
                commandParam = &pdAppInstance->msgSop;
                break;

            case 'f':
                PRINTF("f. power role swap\r\n");
                commandValid = PD_DPM_CONTROL_PR_SWAP;
                commandParam = NULL;
                break;

            case 'g':
                PRINTF("g. data role swap\r\n");
                PRINTF("warning: hard reset will occur if in alternate mode (menu 'i')\r\n");
                commandValid = PD_DPM_CONTROL_DR_SWAP;
                commandParam = NULL;
                break;

            case 'h':
                PRINTF("h. vconn swap\r\n");
                commandValid = PD_DPM_CONTROL_VCONN_SWAP;
                commandParam = NULL;
                break;

            case 'i':
                PRINTF("i. get partner sink capabilities\r\n");
                commandValid = PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES;
                commandParam = NULL;
                break;

            case 'j':
                PRINTF("j. standard structured VDM test (only DFP can send enter mode)\r\n");
                PRINTF("(1) discovery identity\r\n");
                commandValid = PD_DPM_CONTROL_DISCOVERY_IDENTITY;
                pdAppInstance->structuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
                commandParam = &pdAppInstance->structuredVDMCommandParam;
                break;

            case 'k':
                if (dataRole == kPD_DataRoleDFP)
                {
                    PRINTF("k. exit mode (only DFP)\r\n");
                    commandValid = PD_DPM_CONTROL_EXIT_MODE;
                    pdAppInstance->structuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
                    pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.objPos = 1;
                    pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.SVID = pdAppInstance->partnerSVIDs[0];
                    commandParam = &pdAppInstance->structuredVDMCommandParam;
                }
                break;

            case 'l':
                PRINTF("l. send attention\r\n");
                commandValid = PD_DPM_CONTROL_SEND_ATTENTION;
                pdAppInstance->structuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
                pdAppInstance->structuredVDMCommandParam.vdoCount = 0;
                pdAppInstance->structuredVDMCommandParam.vdoData = NULL;
                pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.objPos = 1;
                pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.SVID = PD_VENDOR_VID;
                commandParam = &pdAppInstance->structuredVDMCommandParam;
                break;

            case 'm':
                PRINTF("m. test vendor structured VDM\r\n");
                commandValid = PD_DPM_SEND_VENDOR_STRUCTURED_VDM;
                pdAppInstance->structuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
                pdAppInstance->structuredVDMCommandParam.vdoCount = 0;
                pdAppInstance->structuredVDMCommandParam.vdoData = NULL;
                pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.SVID = PD_VENDOR_VID;
                pdAppInstance->structuredVDMCommandParam.vdmHeader.bitFields.command = 16;
                pdAppInstance->structuredVDMCommandParam.vendorVDMNeedResponse = 1;
                commandParam = &pdAppInstance->structuredVDMCommandParam;
                break;

            case 'n':
            {
                PRINTF("n. test unstructured VDM\r\n");
                commandValid = PD_DPM_SEND_UNSTRUCTURED_VDM;
                pdAppInstance->unstructuredVDMCommandHeader.bitFields.SVID = PD_VENDOR_VID;
                pdAppInstance->unstructuredVDMCommandHeader.bitFields.vdmType = 0;
                unstructuredVDMCommandParam.vdmSop = pdAppInstance->msgSop;
                unstructuredVDMCommandParam.vdmHeaderAndVDOsCount = 1;
                unstructuredVDMCommandParam.vdmHeaderAndVDOsData =
                    (uint32_t *)&pdAppInstance->unstructuredVDMCommandHeader;
                commandParam = &unstructuredVDMCommandParam;
                break;
            }

#if ((defined PD_CONFIG_REVISION) && (PD_CONFIG_REVISION >= PD_SPEC_REVISION_30))
            case 'o':
            {
                PRINTF("o. get source extended capabilities\r\n");
                commandValid = PD_DPM_GET_SRC_EXT_CAP;
                commandParam = NULL;
                break;
            }

            case 'p':
            {
                PRINTF("p. get status\r\n");
                commandValid = PD_DPM_GET_STATUS;
                commandParam = NULL;
                break;
            }

            case 'q':
            {
                PRINTF("q. alert\r\n");
                commandValid = PD_DPM_ALERT;
                pdAppInstance->selfAlert.bitFields.typeOfAlert = 0x02u; /* battery status change */
                commandParam = &pdAppInstance->selfAlert;
                break;
            }

            case 'r':
            {
                PRINTF("r. get battery capabilities\r\n");
                commandValid = PD_DPM_GET_BATTERY_CAP;
                pdAppInstance->getBatteryCapDataBlock = 0x01u; /* get battery 1 cap */
                commandParam = &pdAppInstance->getBatteryCapDataBlock;
                break;
            }

            case 's':
            {
                PRINTF("s. get battery status\r\n");
                commandValid = PD_DPM_GET_BATTERY_STATUS;
                pdAppInstance->getBatteryCapDataBlock = 0x01u; /* get battery 1 cap */
                commandParam = &pdAppInstance->getBatteryCapDataBlock;
                break;
            }

            case 't':
            {
                PRINTF("t. get manufacturer info\r\n");
                pdAppInstance->commonData[0] = 1;
                pdAppInstance->commonData[1] = 0; /* battery zero */
                extCommandParam.dataBuffer = &pdAppInstance->commonData[0];
                extCommandParam.dataLength = 2;
                extCommandParam.sop = pdAppInstance->msgSop;
                commandValid = PD_DPM_GET_MANUFACTURER_INFO;
                commandParam = &extCommandParam;
                break;
            }
#endif

#if 0
        case 'u':
            PRINTF("u. cable reset\r\n");
            commandValid = PD_DPM_CONTROL_CABLE_RESET;
            commandParam = NULL;
            break;
#endif

            default:
                break;
        }

        if (commandValid)
        {
            if (PD_Command(pdAppInstance->pdHandle, commandValid, commandParam) != kStatus_PD_Success)
            {
                PRINTF("command fail\r\n");
            }
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            else
            {
                if (commandValid == PD_DPM_FAST_ROLE_SWAP)
                {
                    PD_PowerSrcTurnOffVbus(pdAppInstance, kVbusPower_Stable);
                }
            }
#endif
        }
    }
    else
    {
    }
}

void PD_DemoInit(void)
{
    volatile uint8_t discardValue;

    g_DemoGlobal.powerRequestSWState = kDEMO_SWIdle;
    g_DemoGlobal.powerChangeSWState = kDEMO_SWIdle;
    g_DemoGlobal.processPort = 0x00u;
    g_DemoGlobal.consoleInputChar = 0x00u;

    /* clear debug console input cache */
    DbgConsole_TryGetchar((char *)&discardValue);
    /* fix arm gcc warning */
    discardValue = discardValue;

    /* demo event */
    g_DemoGlobal.demoEvent = xEventGroupCreate();
    if (NULL == g_DemoGlobal.demoEvent)
    {
        PRINTF("create event fail\r\n");
    }
}

#if (defined BOARD_PD_SW_INPUT_SUPPORT) && (BOARD_PD_SW_INPUT_SUPPORT)
void PD_Demo1msIsrProcessSW(pd_app_t *pdAppInstance)
{
    portBASE_TYPE taskToWake = pdFALSE;

    if (g_DemoGlobal.powerRequestSWState == kDEMO_SWIsrTrigger)
    {
        /* 10ms as short press, 700ms as long press */
        if (HW_GpioReadPowerRequestSW() == 0)
        {
            g_DemoGlobal.powerRequestSWTime++;
            if (g_DemoGlobal.powerRequestSWTime > 700)
            {
                g_DemoGlobal.powerRequestSWState = kDEMO_SWLongPress;
                if (pdPASS == xEventGroupSetBitsFromISR(g_DemoGlobal.demoEvent, PD_DEMO_SW_EVENT_BIT, &taskToWake))
                {
                    MISRAC_DISABLE
                    portYIELD_FROM_ISR(taskToWake);
                    MISRAC_ENABLE
                }
            }
        }
        else
        {
            if (g_DemoGlobal.powerRequestSWTime > 10)
            {
                g_DemoGlobal.powerRequestSWState = kDEMO_SWShortPress;
                if (pdPASS == xEventGroupSetBitsFromISR(g_DemoGlobal.demoEvent, PD_DEMO_SW_EVENT_BIT, &taskToWake))
                {
                    MISRAC_DISABLE
                    portYIELD_FROM_ISR(taskToWake);
                    MISRAC_ENABLE
                }
            }
            else
            {
                g_DemoGlobal.powerRequestSWState = kDEMO_SWIdle;
            }
        }
    }
    else if (g_DemoGlobal.powerRequestSWState == kDEMO_SWProcessed)
    {
        if (HW_GpioReadPowerRequestSW() == 1)
        {
            g_DemoGlobal.powerRequestSWState = kDEMO_SWIdle;
        }
    }
    else if (g_DemoGlobal.powerRequestSWState == kDEMO_SWIdle)
    {
        if (HW_GpioReadPowerRequestSW() == 0)
        {
            g_DemoGlobal.powerRequestSWTime = 0u;
            g_DemoGlobal.powerRequestSWState = kDEMO_SWIsrTrigger;
        }
    }
    else
    {
    }

    if (g_DemoGlobal.powerChangeSWState == kDEMO_SWIsrTrigger)
    {
        /* 10ms as short press, 700ms as long press */
        if (HW_GpioReadPowerChangeSW() == 0)
        {
            g_DemoGlobal.powerChangeSWTime++;
            if (g_DemoGlobal.powerChangeSWTime > 700)
            {
                g_DemoGlobal.powerChangeSWState = kDEMO_SWLongPress;
                if (pdPASS == xEventGroupSetBitsFromISR(g_DemoGlobal.demoEvent, PD_DEMO_SW_EVENT_BIT, &taskToWake))
                {
                    MISRAC_DISABLE
                    portYIELD_FROM_ISR(taskToWake);
                    MISRAC_ENABLE
                }
            }
        }
        else
        {
            if (g_DemoGlobal.powerChangeSWTime > 10)
            {
                g_DemoGlobal.powerChangeSWState = kDEMO_SWShortPress;
                if (pdPASS == xEventGroupSetBitsFromISR(g_DemoGlobal.demoEvent, PD_DEMO_SW_EVENT_BIT, &taskToWake))
                {
                    MISRAC_DISABLE
                    portYIELD_FROM_ISR(taskToWake);
                    MISRAC_ENABLE
                }
            }
            else
            {
                g_DemoGlobal.powerChangeSWState = kDEMO_SWIdle;
            }
        }
    }
    else if (g_DemoGlobal.powerChangeSWState == kDEMO_SWProcessed)
    {
        if (HW_GpioReadPowerChangeSW() == 1)
        {
            g_DemoGlobal.powerChangeSWState = kDEMO_SWIdle;
        }
    }
    else if (g_DemoGlobal.powerChangeSWState == kDEMO_SWIdle)
    {
        if (HW_GpioReadPowerChangeSW() == 0)
        {
            g_DemoGlobal.powerChangeSWTime = 0u;
            g_DemoGlobal.powerChangeSWState = kDEMO_SWIsrTrigger;
        }
    }
    else
    {
    }
}

void USB_PDDemoProcessRequstSW(pd_app_t *pdAppInstance, uint8_t SWState)
{
    uint8_t powerRole;

    PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_POWER_ROLE, &powerRole);

    if (SWState == kDEMO_SWShortPress)
    {
        if (powerRole == kPD_PowerRoleSink)
        {
            PRINTF("request 5V\r\n");
            USB_PDDemoRequest5V(pdAppInstance);
            if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_REQUEST, &pdAppInstance->sinkRequestRDO) !=
                kStatus_PD_Success)
            {
                PRINTF("request call fail\r\n");
            }
        }
    }
    else if (SWState == kDEMO_SWLongPress)
    {
        if (powerRole == kPD_PowerRoleSink)
        {
            PRINTF("request high voltage\r\n");
            if (USB_PDDemoRequestHighVoltage(pdAppInstance))
            {
                if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_REQUEST, &pdAppInstance->sinkRequestRDO) !=
                    kStatus_PD_Success)
                {
                    PRINTF("request call fail\r\n");
                }
            }
            else
            {
                PRINTF("partner don't support");
            }
        }
    }
    else
    {
    }
}

void USB_PDDemoProcessPowerChangeSW(pd_app_t *pdAppInstance, uint8_t SWState)
{
    if (SWState == kDEMO_SWShortPress)
    {
        PRINTF("request power role swap\r\n");
        if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_PR_SWAP, NULL) != kStatus_PD_Success)
        {
            PRINTF("request call fail\r\n");
        }
    }
    else if (SWState == kDEMO_SWLongPress)
    {
        PRINTF("hard reset\r\n");
        if (PD_Command(pdAppInstance->pdHandle, PD_DPM_CONTROL_HARD_RESET, NULL) != kStatus_PD_Success)
        {
            PRINTF("request call fail\r\n");
        }
    }
    else
    {
    }
}
#endif

pd_status_t PD_DemoFindPDO(
    pd_app_t *pdAppInstance, pd_rdo_t *rdo, uint32_t requestVoltagemV, uint32_t requestCurrentmA, uint32_t *voltage)
{
    uint32_t index;
    pd_source_pdo_t sourcePDO;
    uint8_t findSourceCap = 0;

    if (pdAppInstance->partnerSourceCapNumber == 0)
    {
        return kStatus_PD_Error;
    }

    /* default rdo as 5V - 0.5A or less */
    *voltage = 5000;
    rdo->bitFields.objectPosition = 1;
    rdo->bitFields.giveBack = 0;
    rdo->bitFields.capabilityMismatch = 0;
    rdo->bitFields.usbCommunicationsCapable = 0;
    rdo->bitFields.noUsbSuspend = 1;
    rdo->bitFields.operateValue = 500 / PD_PDO_CURRENT_UNIT;
#if ((defined PD_CONFIG_REVISION) && (PD_CONFIG_REVISION >= PD_SPEC_REVISION_30))
    rdo->bitFields.unchunkedSupported = 1;
#endif
    if (rdo->bitFields.operateValue > pdAppInstance->partnerSourceCaps[0].fixedPDO.maxCurrent)
    {
        rdo->bitFields.operateValue = pdAppInstance->partnerSourceCaps[0].fixedPDO.maxCurrent;
    }
    rdo->bitFields.maxOrMinOperateValue = rdo->bitFields.operateValue;

    for (index = 0; index < pdAppInstance->partnerSourceCapNumber; ++index)
    {
        sourcePDO.PDOValue = pdAppInstance->partnerSourceCaps[index].PDOValue;
        switch (sourcePDO.commonPDO.pdoType)
        {
            case kPDO_Fixed:
            {
                if ((sourcePDO.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT == requestVoltagemV) &&
                    (sourcePDO.fixedPDO.maxCurrent * PD_PDO_CURRENT_UNIT >= requestCurrentmA))
                {
                    *voltage = sourcePDO.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT;
                    rdo->bitFields.objectPosition = (index + 1);
                    rdo->bitFields.operateValue = requestCurrentmA / PD_PDO_CURRENT_UNIT;
                    rdo->bitFields.maxOrMinOperateValue = rdo->bitFields.operateValue;
                    findSourceCap = 1;
                }
                break;
            }

            case kPDO_Variable:
            {
                if ((sourcePDO.variablePDO.minVoltage * PD_PDO_VOLTAGE_UNIT <= requestVoltagemV) &&
                    (sourcePDO.variablePDO.maxVoltage * PD_PDO_VOLTAGE_UNIT >= requestVoltagemV) &&
                    (sourcePDO.variablePDO.maxCurrent * PD_PDO_CURRENT_UNIT >= requestCurrentmA))
                {
                    *voltage = sourcePDO.variablePDO.minVoltage * PD_PDO_VOLTAGE_UNIT;
                    rdo->bitFields.objectPosition = (index + 1);
                    rdo->bitFields.operateValue = requestCurrentmA / PD_PDO_CURRENT_UNIT;
                    rdo->bitFields.maxOrMinOperateValue = rdo->bitFields.operateValue;
                    findSourceCap = 1;
                }
                break;
            }

            case kPDO_Battery:
            {
                if ((sourcePDO.batteryPDO.minVoltage * PD_PDO_VOLTAGE_UNIT <= requestVoltagemV) &&
                    (sourcePDO.batteryPDO.maxVoltage * PD_PDO_VOLTAGE_UNIT >= requestVoltagemV) &&
                    (sourcePDO.batteryPDO.maxAllowPower * PD_PDO_POWER_UNIT >=
                     (requestVoltagemV * requestCurrentmA / 1000)))
                {
                    *voltage = sourcePDO.batteryPDO.minVoltage * PD_PDO_VOLTAGE_UNIT;
                    rdo->bitFields.objectPosition = (index + 1);
                    rdo->bitFields.operateValue = (requestVoltagemV * requestCurrentmA) / 1000 / PD_PDO_POWER_UNIT;
                    rdo->bitFields.maxOrMinOperateValue = rdo->bitFields.operateValue;
                    findSourceCap = 1;
                }
                break;
            }

            default:
                break;
        }

        if (findSourceCap)
        {
            break;
        }
    }

    if (findSourceCap)
    {
        return kStatus_PD_Success;
    }
    return kStatus_PD_Error;
}

void pd_DemoConsoleReadFun(void *arg)
{
    while (1)
    {
        g_DemoGlobal.consoleInputChar = GETCHAR();
        xEventGroupSetBits(g_DemoGlobal.demoEvent, PD_DEMO_INPUT_EVENT_BIT);
    }
}

void PD_DemoTaskFun(void)
{
    uint32_t eventBits;
    pd_app_t *pdAppInstance = NULL;
    uint8_t connectState = kTYPEC_ConnectNone;
    uint8_t hasEvent = 0;

    eventBits = xEventGroupWaitBits(g_DemoGlobal.demoEvent, 0xFFu, 1, 0, portMAX_DELAY);

#if (defined PD_DEMO_PORTS_COUNT) && (PD_DEMO_PORTS_COUNT > 1)
    if (eventBits & PD_DEMO_INPUT_EVENT_BIT)
    {
        uint8_t testPort = 0;
        if (g_DemoGlobal.consoleInputChar == '1')
        {
            testPort = 1;
        }
        else if (g_DemoGlobal.consoleInputChar == '2')
        {
            testPort = 2;
        }
        else if (g_DemoGlobal.consoleInputChar == '3')
        {
            testPort = 3;
        }
        else if (g_DemoGlobal.consoleInputChar == '4')
        {
            testPort = 4;
        }
        else
        {
        }

        if (testPort != 0)
        {
            eventBits &= ~(PD_DEMO_INPUT_EVENT_BIT);
            PRINTF("test port %d\r\n", testPort);
            g_DemoGlobal.processPort = testPort;
        }
    }
#else
    g_DemoGlobal.processPort = 1;
#endif

    if (eventBits)
    {
#if (defined PD_DEMO_PORTS_COUNT) && (PD_DEMO_PORTS_COUNT > 1)
        if (g_DemoGlobal.processPort == 0x00u)
        {
            hasEvent = 0;
#if (defined BOARD_PD_SW_INPUT_SUPPORT) && (BOARD_PD_SW_INPUT_SUPPORT)
            if (eventBits & PD_DEMO_SW_EVENT_BIT)
            {
                if ((g_DemoGlobal.powerRequestSWState == kDEMO_SWShortPress) ||
                    (g_DemoGlobal.powerRequestSWState == kDEMO_SWLongPress))
                {
                    g_DemoGlobal.powerRequestSWState = kDEMO_SWProcessed;
                }
                else if ((g_DemoGlobal.powerChangeSWState == kDEMO_SWShortPress) ||
                         (g_DemoGlobal.powerChangeSWState == kDEMO_SWLongPress))
                {
                    g_DemoGlobal.powerChangeSWState = kDEMO_SWProcessed;
                }
                else
                {
                }
                hasEvent = 1;
            }
#endif

            if (eventBits & PD_DEMO_INPUT_EVENT_BIT)
            {
                g_DemoGlobal.consoleInputChar = 0x00u;
                hasEvent = 1;
            }

            if (hasEvent)
            {
                PRINTF("please set the tested port by input number 1-4:\r\n");
            }
            return;
        }
#endif

        if ((g_DemoGlobal.processPort) > 0 && (g_DemoGlobal.processPort < 5))
        {
            for (uint8_t index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
            {
                if (g_PDAppInstanceArray[index]->portNumber == g_DemoGlobal.processPort)
                {
                    pdAppInstance = g_PDAppInstanceArray[index];
                    break;
                }
            }
        }
        if ((pdAppInstance == NULL) || (pdAppInstance->pdHandle == NULL))
        {
            return;
        }
        /* get connect state */
        PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_TYPEC_CONNECT_STATE, &connectState);
        if (connectState == kTYPEC_ConnectNone)
        {
            if (hasEvent)
            {
                PRINTF("port don't connect\r\n");
            }
        }

#if (defined BOARD_PD_SW_INPUT_SUPPORT) && (BOARD_PD_SW_INPUT_SUPPORT)
        if (eventBits & PD_DEMO_SW_EVENT_BIT)
        {
            if ((g_DemoGlobal.powerRequestSWState == kDEMO_SWShortPress) ||
                (g_DemoGlobal.powerRequestSWState == kDEMO_SWLongPress))
            {
                USB_PDDemoProcessRequstSW(pdAppInstance, g_DemoGlobal.powerRequestSWState);
                g_DemoGlobal.powerRequestSWState = kDEMO_SWProcessed;
            }
            else if ((g_DemoGlobal.powerChangeSWState == kDEMO_SWShortPress) ||
                     (g_DemoGlobal.powerChangeSWState == kDEMO_SWLongPress))
            {
                USB_PDDemoProcessPowerChangeSW(pdAppInstance, g_DemoGlobal.powerChangeSWState);
                g_DemoGlobal.powerChangeSWState = kDEMO_SWProcessed;
            }
            else
            {
            }
        }
#endif

        if (eventBits & PD_DEMO_INPUT_EVENT_BIT)
        {
            USB_PDDemoProcessMenu(pdAppInstance, g_DemoGlobal.consoleInputChar);
        }
    }
}
