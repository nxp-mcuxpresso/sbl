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

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "pd_app.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "board.h"
#include "pd_power_interface.h"
#include "pd_board_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void PD_DemoInit(void);
void PD_DemoTaskFun(void);
void BOARD_InitHardware(void);
void PD_Demo1msIsrProcess(void);
void HW_GpioInit(void);
void HW_TimerInit(void);
void HW_GpioPDPHYIntEnable(uint8_t port, uint8_t enable);
void HW_I2CInit(void);
void HW_TimerCallback(void);
pd_status_t PD_PowerBoardReset(pd_power_control_instance_t *powerControl);
void PD_DemoReset(pd_app_t *pdAppInstance);
pd_status_t PD_DpmAppCommandCallback(void *callbackParam, uint32_t event, void *param);

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if (defined PD_DEMO_PORT1_ENABLE) && (PD_DEMO_PORT1_ENABLE > 0)
pd_app_t g_PDAppInstancePort1;
#endif
#if (defined PD_DEMO_PORT2_ENABLE) && (PD_DEMO_PORT2_ENABLE > 0)
pd_app_t g_PDAppInstancePort2;
#endif
#if (defined PD_DEMO_PORT3_ENABLE) && (PD_DEMO_PORT3_ENABLE > 0)
pd_app_t g_PDAppInstancePort3;
#endif
#if (defined PD_DEMO_PORT4_ENABLE) && (PD_DEMO_PORT4_ENABLE > 0)
pd_app_t g_PDAppInstancePort4;
#endif
extern pd_instance_config_t *g_PortsConfigArray[];
pd_app_t *g_PDAppInstanceArray[] = {
#if (defined PD_DEMO_PORT1_ENABLE) && (PD_DEMO_PORT1_ENABLE > 0)
    &g_PDAppInstancePort1,
#endif
#if (defined PD_DEMO_PORT2_ENABLE) && (PD_DEMO_PORT2_ENABLE > 0)
    &g_PDAppInstancePort2,
#endif
#if (defined PD_DEMO_PORT3_ENABLE) && (PD_DEMO_PORT3_ENABLE > 0)
    &g_PDAppInstancePort3,
#endif
#if (defined PD_DEMO_PORT3_ENABLE) && (PD_DEMO_PORT4_ENABLE > 0)
    &g_PDAppInstancePort4,
#endif
};

pd_power_handle_callback_t callbackFunctions = {
    NULL,
    NULL,
    NULL,
    NULL,
    PD_PowerSnkDrawTypeCVbus,
    PD_PowerSnkDrawRequestVbus,
    PD_PowerSnkStopDrawVbus,
    PD_PowerSnkGotoMinReducePower,
    PD_PowerControlVconn,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

#if (defined PD_DEMO_PORT1_ENABLE) && (PD_DEMO_PORT1_ENABLE > 0)
void HW_GpioPDPHYPort1IntCallback(void)
{
    PD_PTN5110IsrFunction(g_PDAppInstancePort1.pdHandle);
}
#endif

#if (defined PD_DEMO_PORT2_ENABLE) && (PD_DEMO_PORT2_ENABLE > 0)
void HW_GpioPDPHYPort2IntCallback(void)
{
    PD_PTN5110IsrFunction(g_PDAppInstancePort2.pdHandle);
}
#endif

#if (defined PD_DEMO_PORT3_ENABLE) && (PD_DEMO_PORT3_ENABLE > 0)
void HW_GpioPDPHYPort3IntCallback(void)
{
    PD_PTN5110IsrFunction(g_PDAppInstancePort3.pdHandle);
}
#endif

#if (defined PD_DEMO_PORT4_ENABLE) && (PD_DEMO_PORT4_ENABLE > 0)
void HW_GpioPDPHYPort4IntCallback(void)
{
    PD_PTN5110IsrFunction(g_PDAppInstancePort4.pdHandle);
}
#endif

pd_status_t PD_DpmConnectCallback(void *callbackParam, uint32_t event, void *param)
{
    pd_status_t status = kStatus_PD_Error;
    pd_app_t *pdAppInstance = (pd_app_t *)callbackParam;

    switch (event)
    {
        case PD_DISCONNECTED:
            PD_PowerBoardReset(&pdAppInstance->powerControlInstance);
            PD_DemoReset(pdAppInstance);
            PRINTF("port %d disconnect\r\n", pdAppInstance->portNumber);
            status = kStatus_PD_Success;
            break;

        case PD_CONNECT_ROLE_CHANGE:
        case PD_CONNECTED:
        {
            uint8_t getInfo;
            PD_DemoReset(pdAppInstance);
            pdAppInstance->partnerSourceCapNumber = 0;
            PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_POWER_ROLE, &getInfo);

            PRINTF((event == PD_CONNECTED) ? "port %d connected," : "port %d connect change,",
                   pdAppInstance->portNumber);
            PRINTF(" power role:%s,", (getInfo == kPD_PowerRoleSource) ? "Source" : "Sink");
            PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_DATA_ROLE, &getInfo);
            PRINTF(" data role:%s,", (getInfo == kPD_DataRoleDFP) ? "DFP" : "UFP");
            PD_Control(pdAppInstance->pdHandle, PD_CONTROL_GET_VCONN_ROLE, &getInfo);
            PRINTF(" vconn source:%s\r\n", (getInfo == kPD_IsVconnSource) ? "yes" : "no");
            status = kStatus_PD_Success;
            break;
        }

        default:
            break;
    }

    return status;
}

pd_status_t PD_DpmDemoAppCallback(void *callbackParam, uint32_t event, void *param)
{
    pd_status_t status = kStatus_PD_Error;

    switch (event)
    {
        case PD_FUNCTION_DISABLED:
            /* need hard or software reset */
            status = kStatus_PD_Success;
            break;

        case PD_CONNECTED:
        case PD_CONNECT_ROLE_CHANGE:
        case PD_DISCONNECTED:
            status = PD_DpmConnectCallback(callbackParam, event, param);
            break;

        default:
            status = PD_DpmAppCommandCallback(callbackParam, event, param);
            break;
    }
    return status;
}

void PD_AppInit(void)
{
    uint8_t index;
    pd_app_t *pdAppInstance;
    pd_app_t *pdAppInstanceArray[] =
    {
#if (defined PD_DEMO_PORT1_ENABLE) && (PD_DEMO_PORT1_ENABLE > 0)
        &g_PDAppInstancePort1,
#else
        NULL,
#endif
#if (defined PD_DEMO_PORT2_ENABLE) && (PD_DEMO_PORT2_ENABLE > 0)
        &g_PDAppInstancePort2,
#else
        NULL,
#endif
#if (defined PD_DEMO_PORT3_ENABLE) && (PD_DEMO_PORT3_ENABLE > 0)
        &g_PDAppInstancePort3,
#else
        NULL,
#endif
#if (defined PD_DEMO_PORT3_ENABLE) && (PD_DEMO_PORT4_ENABLE > 0)
        &g_PDAppInstancePort4,
#else
        NULL,
#endif
    };

    for (index = 0; index < 4; ++index)
    {
        if (pdAppInstanceArray[index] != NULL)
        {
            pdAppInstanceArray[index]->portNumber = (index + 1);
        }
    }

    for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
    {
        pdAppInstance = g_PDAppInstanceArray[index];
        pdAppInstance->pdHandle = NULL;
        pdAppInstance->pdConfigParam = g_PortsConfigArray[index];
        if (PD_InstanceInit(&pdAppInstance->pdHandle, PD_DpmDemoAppCallback, &callbackFunctions, pdAppInstance,
                            g_PortsConfigArray[index]) != kStatus_PD_Success)
        {
            PRINTF("pd port %d init fail\r\n", pdAppInstance->portNumber);
        }
        PD_PowerBoardControlInit(&pdAppInstance->powerControlInstance, pdAppInstance->pdHandle);

        pdAppInstance->msgSop = kPD_MsgSOP;
        pdAppInstance->partnerSourceCapNumber = 0;
        /* alternate mode (VDM) */
        pdAppInstance->selfVdmIdentity.idHeaderVDO.vdoValue = 0;
        pdAppInstance->selfVdmIdentity.idHeaderVDO.bitFields.modalOperateSupport = 1;
#if ((defined PD_CONFIG_REVISION) && (PD_CONFIG_REVISION >= PD_SPEC_REVISION_30))
        pdAppInstance->selfVdmIdentity.idHeaderVDO.bitFields.productTypeDFP = 2; /* PDUSB Host */
#endif
        pdAppInstance->selfVdmIdentity.idHeaderVDO.bitFields.productTypeUFPOrCablePlug = 2; /* PDUSB Peripheral */
        pdAppInstance->selfVdmIdentity.idHeaderVDO.bitFields.usbCommunicationCapableAsDevice = 0;
        pdAppInstance->selfVdmIdentity.idHeaderVDO.bitFields.usbCommunicationCapableAsHost = 0;
        pdAppInstance->selfVdmIdentity.idHeaderVDO.bitFields.usbVendorID = PD_VENDOR_VID;
        pdAppInstance->selfVdmIdentity.pid = PD_CONFIG_PID;
        pdAppInstance->selfVdmIdentity.certStatVDO = PD_CONFIG_XID;
        pdAppInstance->selfVdmIdentity.bcdDevice = PD_CONFIG_BCD_DEVICE;
        pdAppInstance->selfVdmSVIDs = ((uint32_t)PD_VENDOR_VID << 16); /* only one SVID (display port) */
    }
}

void HW_TimerCallback(void)
{
    /* Callback into timer service */
    uint8_t index;
    for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
    {
        PD_TimerIsrFunction(g_PDAppInstanceArray[index]->pdHandle);
    }

    PD_Demo1msIsrProcess();
}

void PD_PortTask(void *arg)
{
    pd_app_t *pdAppInstance = (pd_app_t *)arg;
    HW_GpioPDPHYIntEnable(pdAppInstance->portNumber, 1);
    while (1)
    {
        PD_InstanceTask(pdAppInstance->pdHandle);
    }
}

static void pd_demo_task(void *arg)
{
    uint8_t index;

    HW_GpioInit();
    HW_TimerInit();
    HW_I2CInit();
    PD_AppInit();
    PD_DemoInit();
    PRINTF("pd init success\r\n");

    for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
    {
        if (xTaskCreate(PD_PortTask, "port", (1024u + 512u) / sizeof(portSTACK_TYPE), g_PDAppInstanceArray[index], 5,
                        NULL) != pdPASS)
        {
            PRINTF("create task error\r\n");
        }
    }

    while (1)
    {
        PD_DemoTaskFun();
    }
}

int main(void)
{
    BOARD_InitHardware();

    if (xTaskCreate(pd_demo_task, "demo", 1024 / sizeof(portSTACK_TYPE), NULL, 4, NULL) != pdPASS)
    {
        PRINTF("create demo task error\r\n");
    }

    vTaskStartScheduler();

    while (1)
    {
        ;
    }
}
