/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#include "fsl_device_registers.h"
#include "fsl_lpit.h"
#include "usb_timer.h"

static usb_timer_callback_t timerCallback;

void LPIT0_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    LPIT_ClearStatusFlags(LPIT0, kLPIT_Channel0TimerFlag);
    timerCallback();
   /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping 
     exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void USB_TimerInit(uint8_t instance, uint32_t interval, uint32_t clock, usb_timer_callback_t callback)
{
    LPIT_Type *instanceList[] = LPIT_BASE_PTRS;
    IRQn_Type instanceIrq[][FSL_FEATURE_LPIT_TIMER_COUNT] = LPIT_IRQS;
    /* Structure of initialize PIT */
    lpit_config_t lpitConfig;
    lpit_chnl_params_t lpitChannelConfig;

    /*
     * lpitConfig.enableRunInDebug = false;
     * lpitConfig.enableRunInDoze = false;
     */
    LPIT_GetDefaultConfig(&lpitConfig);

    /* Init lpit module */
    LPIT_Init(instanceList[instance], &lpitConfig);

    lpitChannelConfig.chainChannel = false;
    lpitChannelConfig.enableReloadOnTrigger = false;
    lpitChannelConfig.enableStartOnTrigger = false;
    lpitChannelConfig.enableStopOnTimeout = false;
    lpitChannelConfig.timerMode = kLPIT_PeriodicCounter;
    /* Set default values for the trigger source */
    lpitChannelConfig.triggerSelect = kLPIT_Trigger_TimerChn0;
    lpitChannelConfig.triggerSource = kLPIT_TriggerSource_External;

    /* Init lpit channel 0 */
    LPIT_SetupChannel(instanceList[instance], kLPIT_Chnl_0, &lpitChannelConfig);

    /* Set timer period for channel 0 */
    LPIT_SetTimerPeriod(instanceList[instance], kLPIT_Chnl_0, USEC_TO_COUNT(interval, clock));

    /* Enable timer interrupts for channel 0 */
    LPIT_EnableInterrupts(instanceList[instance], kLPIT_Channel0TimerInterruptEnable);

    timerCallback = callback;

    /* Enable at the NVIC */
    EnableIRQ(instanceIrq[0][instance]);
}

void USB_TimerInt(uint8_t instance, uint8_t enable)
{
    LPIT_Type *instanceList[] = LPIT_BASE_PTRS;
    if (enable)
    {
        /* Start counting */
        LPIT_StartTimer(instanceList[instance], kLPIT_Chnl_0);
    }
    else
    {
        LPIT_StopTimer(instanceList[instance], kLPIT_Chnl_0);
    }
}
