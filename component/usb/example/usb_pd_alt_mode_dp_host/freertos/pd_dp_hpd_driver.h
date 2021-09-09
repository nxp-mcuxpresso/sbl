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

#ifndef __PD_DP_HPD_DRIVER_H__
#define __PD_DP_HPD_DRIVER_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define HPD_DRIVE_MIN_HIGH_TICKS (2)                  /* ms */
#define HPD_DRIVE_MIN_LOW_TICKS (3)                   /* ms */
#define HPD_DRIVE_MIN_TICKS_BEFORE_DRIVER_RELEASE (1) /* ms, the value is 600us */
#define HPD_DRIVE_IRQ_LOW_USEC (700)                  /* 700 us */

#define HPD_DRIVE_QUEUE_SIZE (5)

typedef enum _pd_dp_hpd_driver
{
    kDPDriver_None = 0,
    kDPDriver_IRQ,
    kDPDriver_Low,
    kDPDriver_High,
    kDPDriver_Waiting,
} pd_dp_hpd_driver_t;

typedef struct _pd_hpd_config
{
    uint8_t hpdControlGPIO;
    uint8_t hpdControlPort;
    uint8_t hpdControlPin;
} pd_hpd_config_t;

typedef struct _pd_hpd_driver
{
    void *altModehandle;
    void *dpHandle;
    uint32_t hpdTime;
    const pd_hpd_config_t *hpdConfig;
    uint8_t hpdHeldLow;
    uint8_t hpdOperating;
    uint8_t hpdDriverQueue[HPD_DRIVE_QUEUE_SIZE];
    uint8_t hpdDriverQueueCount;
} pd_hpd_driver_t;

/*******************************************************************************
 * API
 ******************************************************************************/

void PD_DpHpdDriver1msISR(pd_hpd_driver_t *hpdDriver);
/* 0 - success; other values - fail */
uint8_t PD_DpHpdDriverInit(pd_hpd_driver_t *hpdDriver, void *altModehandle, void *dpHandle, void *hpdConfig);
uint8_t PD_DpHpdDriverDeinit(pd_hpd_driver_t *hpdDriver);
void PD_DpHpdDriverSetLow(pd_hpd_driver_t *hpdDriver);
void PD_DpHpdDriverReleaseLow(pd_hpd_driver_t *hpdDriver);
void PD_DpHpdDriverControl(pd_hpd_driver_t *hpdDriver, uint8_t driveVal);
void PD_DpHpdDrvierProcess(pd_hpd_driver_t *hpdDriver);

#endif
