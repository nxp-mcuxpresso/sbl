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
#include "pd_alt_mode.h"
#include "pd_alt_mode_dp.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if (PD_DEMO_PORTS_COUNT > PD_DP_MAX_PORT) || (PD_DEMO_PORTS_COUNT > PD_ALT_MODE_MAX_PORT)
#error "please increase the instance count"
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void BOARD_InitHardware(void);
void HW_GpioInit(void);
void HW_TimerInit(void);
void HW_GpioPDPHYIntEnable(uint8_t port, uint8_t enable);
void HW_TimerCallback(void);
pd_status_t PD_DpmAppCommandCallback(void *callbackParam, uint32_t event, void *param);
pd_status_t PD_AltModeCallback(void *callbackParam, uint32_t event, void *param);
void HW_I2CInit(void);

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
extern const pd_alt_mode_config_t *g_AltModeConfigsArray[];
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
    PD_PowerSrcTurnOnDefaultVbus,  PD_PowerSrcTurnOnRequestVbus,  PD_PowerSrcTurnOffVbus,
    PD_PowerSrcGotoMinReducePower, PD_PowerSnkDrawTypeCVbus,      PD_PowerSnkDrawRequestVbus,
    PD_PowerSnkStopDrawVbus,       PD_PowerSnkGotoMinReducePower, PD_PowerControlVconn,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

void PD_FreeRTOSEnterCritical(uint32_t *sr)
{
#if defined(__GIC_PRIO_BITS)
    if ((__get_CPSR() & CPSR_M_Msk) == 0x13)
#else
    if (__get_IPSR())
#endif
    {
        *sr = portSET_INTERRUPT_MASK_FROM_ISR();
    }
    else
    {
        portENTER_CRITICAL();
    }
}

void PD_FreeRTOSExitCritical(uint32_t sr)
{
#if defined(__GIC_PRIO_BITS)
    if ((__get_CPSR() & CPSR_M_Msk) == 0x13)
#else
    if (__get_IPSR())
#endif
    {
        portCLEAR_INTERRUPT_MASK_FROM_ISR(sr);
    }
    else
    {
        portEXIT_CRITICAL();
    }
}

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
            PRINTF("port %d disconnect\r\n", pdAppInstance->portNumber);
            status = kStatus_PD_Success;
            break;

        case PD_CONNECT_ROLE_CHANGE:
        case PD_CONNECTED:
        {
            uint8_t getInfo;

            pdAppInstance->partnerSourceCapNumber = 0;
            pdAppInstance->partnerSinkCapNumber = 0;
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

pd_status_t PD_StackDemoAppCallback(void *callbackParam, uint32_t event, void *param)
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

pd_status_t PD_StackCallback(void *callbackParam, uint32_t event, void *param)
{
    pd_status_t status1 = PD_StackDemoAppCallback(callbackParam, event, param);
    pd_status_t status2 = PD_AltModeCallback(((pd_app_t *)callbackParam)->altModeHandle, event, param);

    if ((status1 == kStatus_PD_Success) || (status2 == kStatus_PD_Success))
    {
        return kStatus_PD_Success;
    }
    else
    {
        return kStatus_PD_Error;
    }
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
        pdAppInstance->altModeConfig = (void *)g_AltModeConfigsArray[index];
        if (PD_InstanceInit(&pdAppInstance->pdHandle, PD_StackCallback, &callbackFunctions, pdAppInstance,
                            g_PortsConfigArray[index]) != kStatus_PD_Success)
        {
            PRINTF("pd port %d init fail\r\n", pdAppInstance->portNumber);
        }
        PD_PowerBoardControlInit(&pdAppInstance->powerControlInstance, pdAppInstance->pdHandle);

        if (PD_AltModeInit(pdAppInstance->pdHandle, g_AltModeConfigsArray[index], &(pdAppInstance->altModeHandle)) !=
            kStatus_PD_Success)
        {
            PRINTF("pd port %d alt mode init fail\r\n", pdAppInstance->portNumber);
        }

        pdAppInstance->msgSop = kPD_MsgSOP;
        pdAppInstance->partnerSourceCapNumber = 0;
        pdAppInstance->partnerSinkCapNumber = 0;
        /* evaluate result */
        pdAppInstance->prSwapAccept = 1;
        pdAppInstance->vconnSwapAccept = 1;
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

    for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
    {
        if (g_PDAppInstanceArray[index]->altModeHandle != NULL)
        {
            PD_AltModeTimer1msISR(g_PDAppInstanceArray[index]->altModeHandle);
        }
    }
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
void PD_PortAltModeTask(void *arg)
{
    pd_app_t *pdAppInstance = (pd_app_t *)arg;
    while (1)
    {
        PD_AltModeTask(pdAppInstance->altModeHandle);
    }
}

static void pd_demo_task(void *arg)
{
    uint8_t index;

    HW_GpioInit();
    HW_TimerInit();
    HW_I2CInit();
    PD_AppInit();
    PRINTF("pd init success\r\n");

    for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
    {
        if (xTaskCreate(PD_PortTask, "port", (1024u + 512u) / sizeof(portSTACK_TYPE), g_PDAppInstanceArray[index], 5,
                        NULL) != pdPASS)
        {
            PRINTF("create task error\r\n");
        }
        if (xTaskCreate(PD_PortAltModeTask, "port_alt", (1024u) / sizeof(portSTACK_TYPE), g_PDAppInstanceArray[index],
                        4, NULL) != pdPASS)
        {
            PRINTF("create task error\r\n");
        }
    }
    vTaskDelete(NULL);
}

int main(void)
{
    BOARD_InitHardware();

    if (xTaskCreate(pd_demo_task, "demo", 1024 / sizeof(portSTACK_TYPE), NULL, 6, NULL) != pdPASS)
    {
        PRINTF("create demo task error\r\n");
    }

    vTaskStartScheduler();

    while (1)
    {
        ;
    }
}
