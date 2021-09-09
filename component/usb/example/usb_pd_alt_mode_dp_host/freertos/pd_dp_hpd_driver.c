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

#include <stdint.h>
#include <stdio.h>
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "string.h"
#include "pd_alt_mode.h"
#include "pd_alt_mode_dp.h"
#include "usb_io.h"
#include "pd_dp_hpd_driver.h"
#include "pd_app_misc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void HW_WaitUs(uint32_t us);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void PD_DpHpdDriver1msISR(pd_hpd_driver_t *hpdDriver)
{
    if (hpdDriver->hpdTime > 0)
    {
        hpdDriver->hpdTime--;
        if (hpdDriver->hpdTime == 0)
        {
            hpdDriver->hpdOperating = kDPDriver_None;
            PD_AltModeModuleTaskWakeUp(hpdDriver->altModehandle, hpdDriver->dpHandle);
        }
    }
}

/* 0 - success; other values - fail */
uint8_t PD_DpHpdDriverInit(pd_hpd_driver_t *hpdDriver, void *altModehandle, void *dpHandle, void *hpdConfig)
{
    hpdDriver->altModehandle = altModehandle;
    hpdDriver->dpHandle = dpHandle;
    hpdDriver->hpdHeldLow = 0;
    hpdDriver->hpdDriverQueueCount = 0;
    hpdDriver->hpdOperating = kDPDriver_None;
    hpdDriver->hpdConfig = (const pd_hpd_config_t *)hpdConfig;
    USB_GpioOutputInit(hpdDriver->hpdConfig->hpdControlGPIO, hpdDriver->hpdConfig->hpdControlPort,
                       hpdDriver->hpdConfig->hpdControlPin);
    PD_DpHpdDriverSetLow(hpdDriver);
    return 0;
}

uint8_t PD_DpHpdDriverDeinit(pd_hpd_driver_t *hpdDriver)
{
    return 0;
}

void PD_DpHpdDriverSetLow(pd_hpd_driver_t *hpdDriver)
{
    APP_CRITICAL_ALLOC();

    APP_ENTER_CRITICAL();
    if (!(hpdDriver->hpdHeldLow))
    {
        hpdDriver->hpdDriverQueueCount = 0;
        hpdDriver->hpdHeldLow = 1;
        hpdDriver->hpdOperating = kDPDriver_Waiting;
        APP_EXIT_CRITICAL();
        hpdDriver->hpdTime = HPD_DRIVE_MIN_LOW_TICKS;

        USB_GpioOutputWritePin(hpdDriver->hpdConfig->hpdControlGPIO, hpdDriver->hpdConfig->hpdControlPort,
                               hpdDriver->hpdConfig->hpdControlPin, 0);
    }
    else
    {
        APP_EXIT_CRITICAL();
    }
}

void PD_DpHpdDriverReleaseLow(pd_hpd_driver_t *hpdDriver)
{
    /* keep least low for HPD_DRIVE_MIN_TICKS_BEFORE_DRIVER_RELEASE from this call,
     * And keep low least HPD_DRIVE_MIN_LOW_TICKS for LOW from drive LOW. */
    APP_CRITICAL_ALLOC();

    APP_ENTER_CRITICAL();
    if (hpdDriver->hpdHeldLow)
    {
        hpdDriver->hpdHeldLow = 0;
        APP_EXIT_CRITICAL();
        if (hpdDriver->hpdOperating == kDPDriver_Waiting)
        {
            if (hpdDriver->hpdTime <= HPD_DRIVE_MIN_TICKS_BEFORE_DRIVER_RELEASE)
            {
                hpdDriver->hpdTime = HPD_DRIVE_MIN_TICKS_BEFORE_DRIVER_RELEASE;
            }
        }
        else
        {
            hpdDriver->hpdOperating = kDPDriver_Waiting;
            hpdDriver->hpdTime = HPD_DRIVE_MIN_TICKS_BEFORE_DRIVER_RELEASE;
        }
    }
    else
    {
        APP_EXIT_CRITICAL();
    }
}

static void PD_DpHpdDriverOps(pd_hpd_driver_t *hpdDriver, uint8_t driveVal)
{
    if (driveVal == kDPDriver_IRQ)
    {
        APP_CRITICAL_ALLOC();
        USB_GpioOutputWritePin(hpdDriver->hpdConfig->hpdControlGPIO, hpdDriver->hpdConfig->hpdControlPort,
                               hpdDriver->hpdConfig->hpdControlPin, 0);
        /* wait HPD_DRIVE_IRQ_LOW_USEC, make sure the time is accurate */
        APP_ENTER_CRITICAL();
        HW_WaitUs(HPD_DRIVE_IRQ_LOW_USEC);
        APP_EXIT_CRITICAL();
        USB_GpioOutputWritePin(hpdDriver->hpdConfig->hpdControlGPIO, hpdDriver->hpdConfig->hpdControlPort,
                               hpdDriver->hpdConfig->hpdControlPin, 1);
        hpdDriver->hpdTime = HPD_DRIVE_MIN_HIGH_TICKS;
    }
    else if (driveVal == kDPDriver_High)
    {
        USB_GpioOutputWritePin(hpdDriver->hpdConfig->hpdControlGPIO, hpdDriver->hpdConfig->hpdControlPort,
                               hpdDriver->hpdConfig->hpdControlPin, 1);
        hpdDriver->hpdTime = HPD_DRIVE_MIN_HIGH_TICKS;
    }
    else if (driveVal == kDPDriver_Low)
    {
        USB_GpioOutputWritePin(hpdDriver->hpdConfig->hpdControlGPIO, hpdDriver->hpdConfig->hpdControlPort,
                               hpdDriver->hpdConfig->hpdControlPin, 0);
        hpdDriver->hpdTime = HPD_DRIVE_MIN_LOW_TICKS;
    }
    else
    {
    }
}

void PD_DpHpdDriverControl(pd_hpd_driver_t *hpdDriver, uint8_t driveVal)
{
    APP_CRITICAL_ALLOC();

    APP_ENTER_CRITICAL();
    if ((hpdDriver->hpdOperating != kDPDriver_None) || (hpdDriver->hpdHeldLow))
    {
        if (driveVal == kDPDriver_Low)
        {
            hpdDriver->hpdDriverQueueCount = 0;
            if (!(hpdDriver->hpdHeldLow))
            {
                hpdDriver->hpdDriverQueue[hpdDriver->hpdDriverQueueCount++] = driveVal;
            }
        }
        else
        {
            if (hpdDriver->hpdDriverQueueCount < HPD_DRIVE_QUEUE_SIZE)
            {
                hpdDriver->hpdDriverQueue[hpdDriver->hpdDriverQueueCount++] = driveVal;
            }
        }
        APP_EXIT_CRITICAL();
    }
    else
    {
        hpdDriver->hpdOperating = driveVal;
        hpdDriver->hpdOperating = kDPDriver_Waiting;
        APP_EXIT_CRITICAL();
        PD_DpHpdDriverOps(hpdDriver, driveVal);
    }
}

void PD_DpHpdDrvierProcess(pd_hpd_driver_t *hpdDriver)
{
    uint8_t index = 0;
    uint8_t drive;

    APP_CRITICAL_ALLOC();

    APP_ENTER_CRITICAL();
    if ((hpdDriver->hpdHeldLow) || (hpdDriver->hpdOperating != kDPDriver_None))
    {
        APP_EXIT_CRITICAL();
        return;
    }

    if (hpdDriver->hpdDriverQueueCount > 0)
    {
        drive = hpdDriver->hpdDriverQueue[0];
        hpdDriver->hpdDriverQueueCount--;
        for (index = 0; index < hpdDriver->hpdDriverQueueCount; ++index)
        {
            hpdDriver->hpdDriverQueue[index] = hpdDriver->hpdDriverQueue[index + 1];
        }
        hpdDriver->hpdOperating = drive;
        hpdDriver->hpdOperating = kDPDriver_Waiting;
        APP_EXIT_CRITICAL();
        PD_DpHpdDriverOps(hpdDriver, drive);
    }
    else
    {
        APP_EXIT_CRITICAL();
    }
}
