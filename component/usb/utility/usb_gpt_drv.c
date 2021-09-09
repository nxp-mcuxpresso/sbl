/*
 * The Clear BSD License
 * Copyright (c) 2017, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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
#include "fsl_gpt.h"
#include "usb_timer.h"

static usb_timer_callback_t timerCallback;

void GPT1_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare1Flag);
    timerCallback();
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
 exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void USB_TimerInit(uint8_t instance, uint32_t interval, uint32_t clock, usb_timer_callback_t callback)
{
    gpt_config_t gptConfig;
    GPT_Type *instanceList[] = GPT_BASE_PTRS;
    IRQn_Type instanceIrq[] = GPT_IRQS;

    GPT_GetDefaultConfig(&gptConfig);
    /* Initialize GPT module */
    GPT_Init(instanceList[instance], &gptConfig);
    /* Set both GPT modules to 1 second duration */
    GPT_SetOutputCompareValue(instanceList[instance], kGPT_OutputCompare_Channel1, USEC_TO_COUNT(interval, clock));
    /* Enable GPT Output Compare1 interrupt */
    GPT_EnableInterrupts(instanceList[instance], kGPT_OutputCompare1InterruptEnable);

    timerCallback = callback;
    /* Enable at the Interrupt */
    EnableIRQ(instanceIrq[instance]);
}

void USB_TimerInt(uint8_t instance, uint8_t enable)
{
    GPT_Type *instanceList[] = GPT_BASE_PTRS;
    if (enable)
    {
        /* Start Timer */
        GPT_StartTimer(instanceList[instance]);
    }
    else
    {
        GPT_StopTimer(instanceList[instance]);
    }
}
