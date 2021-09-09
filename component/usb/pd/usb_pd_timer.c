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

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_timer.h"
#include "usb_pd_interface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*
  1. timr has started & timr time out: timrsTimeOutState == 1;
  2. timr not start || timr time out: timrsRunningState == 0;
*/

static void PD_TimerCallback(pd_instance_t *pdInstance, uint8_t timrName)
{
    switch (timrName)
    {
        case tMsgHardResetCompleteTimer:
            /* do nothing */
            break;
#if 0
        /* if has USB function */
        case tUSBSuspendTimer:
        case tUSBTimer:
            break;
#endif

        default:
            /* do nothing, code will use _PD_TimerCheckInvalidOrTimeOut to check the timr */
            USB_OsaEventSet(pdInstance->taskEventHandle, PD_TASK_EVENT_TIME_OUT);
            break;
    }
}

pd_status_t PD_TimerClear(pd_handle pdHandle, uint8_t timrName)
{
    uint32_t bitMape;
    USB_OSA_SR_ALLOC();
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if (timrName >= PD_MAX_TIMER_COUNT)
    {
        return kStatus_PD_Error;
    }

    bitMape = (0x00000001u << (timrName & 0x1Fu));

    USB_OSA_ENTER_CRITICAL();
    pdInstance->timrsRunningState[(timrName / 32)] &= (~bitMape);
    pdInstance->timrsTimeOutState[(timrName / 32)] &= (~bitMape);
    USB_OSA_EXIT_CRITICAL();

    return kStatus_PD_Success;
}

pd_status_t PD_TimerStart(pd_handle pdHandle, uint8_t timrName, uint16_t timrTime)
{
    uint32_t bitMape;
    USB_OSA_SR_ALLOC();
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if (timrName >= PD_MAX_TIMER_COUNT)
    {
        return kStatus_PD_Error;
    }
    if (timrTime == 0)
    {
        return kStatus_PD_Error;
    }

    bitMape = (0x00000001u << (timrName & 0x1Fu));

    USB_OSA_ENTER_CRITICAL();
    pdInstance->timrsTimeValue[timrName] = timrTime;
    pdInstance->timrsTimeOutState[(timrName / 32)] &= (~(bitMape));
    pdInstance->timrsRunningState[(timrName / 32)] |= bitMape;
    USB_OSA_EXIT_CRITICAL();
    return kStatus_PD_Success;
}

uint8_t PD_TimerCheckInvalidOrTimeOut(pd_handle pdHandle, uint8_t timrName)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if (timrName >= PD_MAX_TIMER_COUNT)
    {
        return 1;
    }
    /* timr is not running */
    if (!(pdInstance->timrsRunningState[(timrName / 32)] & (0x00000001u << (timrName & 0x1Fu))))
    {
        return 1;
    }

    return 0;
}

uint8_t PD_TimerCheckValidTimeOut(pd_handle pdHandle, uint8_t timrName)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if (timrName >= PD_MAX_TIMER_COUNT)
    {
        return 1;
    }
    if (pdInstance->timrsTimeOutState[(timrName / 32)] & (0x00000001u << (timrName & 0x1Fu)))
    {
        return 1;
    }

    return 0;
}

void PD_TimerCancelAllTimers(pd_handle pdHandle, uint8_t begin, uint8_t end)
{
    uint32_t index;
    uint32_t bitMape;
    USB_OSA_SR_ALLOC();
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    if ((begin >= PD_MAX_TIMER_COUNT) || (end >= PD_MAX_TIMER_COUNT))
    {
        return;
    }
    USB_OSA_ENTER_CRITICAL();
    for (index = begin; index <= end; ++index)
    {
        bitMape = (0x00000001u << (index & 0x1Fu));
        pdInstance->timrsRunningState[(index / 32)] &= (~bitMape);
        pdInstance->timrsTimeOutState[(index / 32)] &= (~bitMape);
    }
    USB_OSA_EXIT_CRITICAL();
}

void PD_TimerIsrFunction(pd_handle pdHandle)
{
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;
    uint32_t index32;
    uint8_t index8;
    uint32_t bitMape;

    if (pdHandle == NULL)
    {
        return;
    }

    for (index32 = 0; index32 < ((tTimerCount + 31) / 32); ++index32)
    {
        if (pdInstance->timrsRunningState[index32] != 0)
        {
            for (index8 = 0; index8 < 32; ++index8)
            {
                bitMape = (0x00000001u << index8);
                if (pdInstance->timrsRunningState[index32] & bitMape)
                {
                    if (((pdInstance->timrsTimeValue[(index32 * 32) + index8])--) == 0)
                    {
                        PD_TimerCallback(pdInstance, (index32 * 32) + index8);
                        pdInstance->timrsTimeValue[(index32 * 32) + index8] = 0;
                        pdInstance->timrsRunningState[index32] &= (~(bitMape));
                        pdInstance->timrsTimeOutState[index32] |= (bitMape);
                    }
                }
            }
        }
    }
}

void PD_TimerInit(pd_handle pdHandle)
{
    uint32_t index32;
    pd_instance_t *pdInstance = (pd_instance_t *)pdHandle;

    for (index32 = 0; index32 < ((tTimerCount + 31) / 32); ++index32)
    {
        pdInstance->timrsRunningState[index32] = 0;
        pdInstance->timrsTimeOutState[index32] = 0;
    }
    for (index32 = 0; index32 < tTimerCount; ++index32)
    {
        pdInstance->timrsTimeValue[index32] = 0;
    }
}
