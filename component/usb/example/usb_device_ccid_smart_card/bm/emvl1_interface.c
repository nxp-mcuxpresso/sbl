/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
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
#include "board.h"
#include "fsl_debug_console.h"
#include <stdint.h>
#include "clock_config.h"
#include "emvl1_core.h"
#include "emvl1_adapter_sdk.h"
#include "emvl1_interface.h"
#include "fsl_smartcard.h"
#if defined(FSL_FEATURE_SOC_EMVSIM_COUNT) && FSL_FEATURE_SOC_EMVSIM_COUNT
#include "fsl_smartcard_emvsim.h"
#else
#include "fsl_smartcard_uart.h"
#endif

/*******************************************************************************
* Definitions
******************************************************************************/

/*******************************************************************************
* Prototypes
******************************************************************************/

/*******************************************************************************
* Variables
******************************************************************************/
smartcard_context_t g_SmartCardContext;
smartcard_core_params_t g_SmartCardCoreParams;
emv_callback_t g_ApplicationCallback;

/*******************************************************************************
* Code
******************************************************************************/
void EMVL1_SmartCardInterfaceCallbackFunction(void *base, void *smartCardState)
{
    if (EMVL1_SmartCardPresence())
    {
        if (NULL != g_ApplicationCallback)
        {
            g_ApplicationCallback(kEmvSmartCardEventCardInserted, NULL, 0U);
        }
    }
    else
    {
        if (NULL != g_ApplicationCallback)
        {
            g_ApplicationCallback(kEmvSmartCardEventCardRemoved, NULL, 0U);
        }
    }
}

void EMVL1_SmartCardTimeDelay(uint32_t microseconds)
{
    uint32_t startSysTickValue = SysTick->VAL;
    /* Wait desired time */
    while (microseconds > 0)
    {
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            while (SysTick->VAL > startSysTickValue)
            {
            }
            microseconds--;
        }
    }
}
/* Function initializes time delay for smartcard driver */
void EMVL1_SmartCardInstallTimeDelay(smartcard_context_t *handle)
{
    /* Disable SysTick timer */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    /* Initialize Reload value to 1us */
    SysTick->LOAD = CLOCK_GetFreq(kCLOCK_CoreSysClk) / 1000000;
    /* Set clock source to processor clock */
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    /* Enable SysTick timer */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    /* Store time delay function to smartcard handle */
    handle->timeDelay = EMVL1_SmartCardTimeDelay;
}

void EMVL1_SmartCardInterruptsConfig(void)
{
    /* Set smartcard communication peripheral interrupt priority */
    NVIC_SetPriority(BOARD_SMARTCARD_MODULE_IRQ, 2u);
#if defined(BOARD_SMARTCARD_MODULE_ERRIRQ)
    NVIC_SetPriority(BOARD_SMARTCARD_MODULE_ERRIRQ, 2u);
#endif
/* Set smartcard presence detect gpio pin interrupt priority */
#if defined(USING_PHY_TDA8035)
    NVIC_SetPriority(BOARD_SMARTCARD_IRQ_PIN_IRQ, 1u);
#endif /* USING_TDA8035_INTERFACE */
       /* Set external PIT timer interrupt priority
        * (used for initial TS char detection time-out) */
#if !(defined(FSL_FEATURE_SOC_EMVSIM_COUNT) && (FSL_FEATURE_SOC_EMVSIM_COUNT))
#if !defined(BOARD_SMARTCARD_TS_TIMER_IRQ)
#error "Please specify external PIT timer interrupt !"
#else
    NVIC_SetPriority(BOARD_SMARTCARD_TS_TIMER_IRQ, 1u);
#endif
#endif
}

/*!
 * @brief This function initialize smartcard_user_config_t structure. Enables configuration
 * of most common settings of the UART peripheral, smartcard clock, smartcard interface configuration,
 * interface instance number, slot number, operating voltage ...
 */
void EMVL1_SmartCardInitUserConfig(smartcard_interface_config_t *interfaceConfig)
{
    /* Initialize interface configuration structure to default values */
    SMARTCARD_PHY_GET_DEFAULT_CONFIG(interfaceConfig);
    interfaceConfig->smartCardClock = BOARD_SMARTCARD_CLOCK_VALUE;
    interfaceConfig->vcc = kSMARTCARD_VoltageClassB3_3V;
    interfaceConfig->clockModule = BOARD_SMARTCARD_CLOCK_MODULE;
    interfaceConfig->clockModuleChannel = BOARD_SMARTCARD_CLOCK_MODULE_CHANNEL;
    interfaceConfig->clockModuleSourceClock = BOARD_SMARTCARD_CLOCK_MODULE_SOURCE_CLK;
    interfaceConfig->controlPort = BOARD_SMARTCARD_CONTROL_PORT;
    interfaceConfig->controlPin = BOARD_SMARTCARD_CONTROL_PIN;
    interfaceConfig->irqPort = BOARD_SMARTCARD_IRQ_PORT;
    interfaceConfig->irqPin = BOARD_SMARTCARD_IRQ_PIN;
    interfaceConfig->resetPort = BOARD_SMARTCARD_RST_PORT;
    interfaceConfig->resetPin = BOARD_SMARTCARD_RST_PIN;
    interfaceConfig->vsel0Port = BOARD_SMARTCARD_VSEL0_PORT;
    interfaceConfig->vsel0Pin = BOARD_SMARTCARD_VSEL0_PIN;
    interfaceConfig->vsel1Port = BOARD_SMARTCARD_VSEL1_PORT;
    interfaceConfig->vsel1Pin = BOARD_SMARTCARD_VSEL1_PIN;
#if defined(BOARD_SMARTCARD_TS_TIMER_ID)
    interfaceConfig->tsTimerId = BOARD_SMARTCARD_TS_TIMER_ID;
#endif
}

void EMVL1_SmartCardInit(emv_callback_t callback)
{
    uint8_t *temp;

    g_ApplicationCallback = callback;

    temp = (uint8_t *)&g_SmartCardCoreParams;
    for (int i = 0; i < sizeof(smartcard_core_params_t); i++)
    {
        temp[i] = 0;
    }

    temp = (uint8_t *)&g_SmartCardContext;
    for (int i = 0; i < sizeof(smartcard_context_t); i++)
    {
        temp[i] = 0;
    }

    /* initialize EMV Level 1 core library */
    g_SmartCardCoreParams.x_context = &g_SmartCardContext;
    /* Initialize smartcard interrupts */
    EMVL1_SmartCardInterruptsConfig();
    /* Install time delay defined in application to smartcard driver. */
    EMVL1_SmartCardInstallTimeDelay(&g_SmartCardContext);
    /* Fill-in smartcard config structure with init data */
    EMVL1_SmartCardInitUserConfig(&g_SmartCardContext.interfaceConfig);

    /* Install test/application callback function */
    g_SmartCardContext.interfaceCallback = EMVL1_SmartCardInterfaceCallbackFunction;

    /* Initialize the smartcard module with base address and configuration structure*/
    SMARTCARD_INIT(BOARD_SMARTCARD_MODULE, g_SmartCardCoreParams.x_context,
                   CLOCK_GetFreq(BOARD_SMARTCARD_CLOCK_MODULE_SOURCE_CLK));
    SMARTCARD_PHY_INIT(BOARD_SMARTCARD_MODULE, &g_SmartCardContext.interfaceConfig,
                       CLOCK_GetFreq(BOARD_SMARTCARD_CLOCK_MODULE_SOURCE_CLK));

    if (EMVL1_SmartCardPresence())
    {
        if (NULL != g_ApplicationCallback)
        {
            g_ApplicationCallback(kEmvSmartCardEventCardInserted, NULL, 0U);
        }
    }
}

uint8_t EMVL1_SmartCardPowerOn(uint8_t *atrBuffer, uint32_t *length)
{
    smartcard_interface_control_t interfaceControl = kSMARTCARD_InterfaceReadStatus;
    smartcard_context_t *handle = (smartcard_context_t *)g_SmartCardCoreParams.x_context;

    if ((NULL == length) || (NULL == atrBuffer))
    {
        return kStatus_CCID_EMV_InvalidParameter;
    }

    /* Read card presence status */
    SMARTCARD_PHY_CONTROL(g_SmartCardCoreParams.x_context, interfaceControl, 0);

    /* Check if a card is already inserted */
    if (!handle->cardParams.present)
    {
        return kStatus_CCID_EMV_CardRemoved;
    }

    SMARTCARD_PHY_DEACTIVATE(g_SmartCardCoreParams.x_context);

    /* Reset, Receive and parse ATR */

    /* Invalidate ATR buffer first */
    for (int i = 0; i < *length; i++)
    {
        atrBuffer[i] = 0;
    }

    *length = 0U;

    if (smartcard_reset_handle_atr(&g_SmartCardCoreParams, atrBuffer, kSMARTCARD_ColdReset, kSMARTCARD_NoWarmReset,
                                   length) == kSCCoreFail)
    {
        return kStatus_CCID_EMV_Unsupported;
    }

    return kStatus_CCID_EMV_Success;
}

uint8_t EMVL1_SmartCardPowerOff(void)
{
    SMARTCARD_PHY_DEACTIVATE(g_SmartCardCoreParams.x_context);
    return kStatus_CCID_EMV_Success;
}

uint8_t EMVL1_SmartCardGetProtocol(void)
{
    uint8_t protocol = 0U;
    smartcard_context_t *handle = (smartcard_context_t *)g_SmartCardCoreParams.x_context;

    if (handle->cardParams.t1Indicated)
    {
        protocol |= EMV_SMART_CARD_PROTOCOL_T1;
    }
    if (handle->cardParams.t0Indicated)
    {
        protocol |= EMV_SMART_CARD_PROTOCOL_T0;
    }
    return protocol;
}

uint8_t EMVL1_SmartCardPresence(void)
{
    smartcard_interface_control_t interfaceControl = kSMARTCARD_InterfaceReadStatus;
    smartcard_context_t *handle = (smartcard_context_t *)g_SmartCardCoreParams.x_context;

    for (int i = 0U; i < 10000000U; i++)
    {
        __ASM("nop");
    }
    /* clear present flag */
    handle->cardParams.present = 0;

    /* Read card presence status */
    SMARTCARD_PHY_CONTROL(g_SmartCardCoreParams.x_context, interfaceControl, 0);

    /* Check if a card is already inserted */
    return (uint8_t)(handle->cardParams.present > 0U);
}

uint8_t EMVL1_SmartCardGetInformation(emv_card_data_information_struct_t *information)
{
    smartcard_context_t *handle = (smartcard_context_t *)g_SmartCardCoreParams.x_context;
    if (information)
    {
        information->DI = handle->cardParams.Di;
        information->FI = 1U;

        information->convention = (kSMARTCARD_InverseConvention == handle->cardParams.convention) ?
                                      kEmvSmartCardConventionInverse :
                                      kEmvSmartCardConventionDirect;
        information->GTN = handle->cardParams.GTN;

        if (handle->cardParams.t1Indicated)
        {
            information->paramUnion.t1.BWI = handle->cardParams.BWI;
            information->paramUnion.t1.CWI = handle->cardParams.CWI;
            information->paramUnion.t1.IFSC = handle->cardParams.IFSC;
            information->paramUnion.t1.checksumType = kEmvSmartCardChecksumTypeLRC;
        }
        else if (handle->cardParams.t0Indicated)
        {
            information->paramUnion.t0.WI = handle->cardParams.WI;
        }
        else
        {
            return kStatus_CCID_EMV_Error;
        }
        return kStatus_CCID_EMV_Success;
    }
    return kStatus_CCID_EMV_Error;
}

uint8_t EMVL1_SendApduCommand(uint8_t *commandApdu,
                              uint32_t commandApduLength,
                              uint8_t *ResponseApdu,
                              uint32_t *ResponseApduLength)
{
    smartcard_context_t *handle = (smartcard_context_t *)g_SmartCardCoreParams.x_context;
    smartcard_tal_cmd_t talCmd;
    smartcard_tal_resp_t talResp;
    smartcard_core_error_t iret;

    if ((commandApdu == NULL) || (ResponseApdu == NULL) || (ResponseApduLength == NULL))
    {
        return kStatus_CCID_EMV_InvalidParameter;
    }

    talCmd.apdu = commandApdu;

    talCmd.length = commandApduLength;

    if (commandApduLength > 5U)
    {
        if (commandApduLength == (5U + commandApdu[4]))
        {
            talCmd.apdu[talCmd.length] = 0x00U;
            talCmd.length++;
        }
    }

#if (defined(_USB_DEBUG) && _USB_DEBUG)
    PRINTF("C-APDU Command: ");
    for (int i = 0U; i < (talCmd.length); i++)
    {
        PRINTF("%02X ", commandApdu[i]);
    }
    PRINTF("\r\n");
    PRINTF("Handle command...\t");
#endif
    memset(ResponseApdu, 0x00U, *ResponseApduLength);
    talResp.length = 0U;
    talResp.resp = ResponseApdu;
    iret = smartcard_tal_send_cmd(&g_SmartCardCoreParams, &talCmd, &talResp);

    *ResponseApduLength = talResp.length;
    ResponseApdu[*ResponseApduLength] = handle->statusBytes[0];
    (*ResponseApduLength)++;
    ResponseApdu[*ResponseApduLength] = handle->statusBytes[1];
    (*ResponseApduLength)++;

    if ((iret == kSCCoreTransportTxFail) || (iret == kSCCoreTransportRxFail) || (iret == kSCCoreIncorrectStatusBytes) ||
        (iret == kSCCoreGRIncorrectReceiveLength) || (iret == kSCCoreUnknownProcByte) || (iret == kSCCoreIOError) ||
        (iret == kSCCoreT1TxRetryExceeded) || (iret == kSCCoreT1RxErrorExceeded) ||
        (iret == kSCCoreT1INFLengthInvalid) || (iret == kSCCoreT1AbortRequestReceived))
    {
        PRINTF("Command failed, iret = %d\r\n", iret);
        return kStatus_CCID_EMV_Error;
    }

#if (defined(_USB_DEBUG) && _USB_DEBUG)
    PRINTF("Read Record Successed!\r\nR-APDU Command: ");
    for (int i = 0U; i < talResp.length; i++)
    {
        PRINTF("%02X ", ResponseApdu[i]);
    }
    PRINTF("%02X ", handle->statusBytes[0]);
    PRINTF("%02X ", handle->statusBytes[1]);
    PRINTF("\r\n");
#endif
    return kStatus_CCID_EMV_Success;
}
