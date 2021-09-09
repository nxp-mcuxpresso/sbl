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
#include "fsl_common.h"
#include "soft_timer.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

static volatile uint32_t g_SoftTimerCount;
static volatile uint32_t g_SetTimer;

/*******************************************************************************
 * Code
 ******************************************************************************/

void PD_DemoSoftTimer1msProcess(void)
{
    g_SoftTimerCount++;
    if (g_SetTimer > 0)
    {
        g_SetTimer--;
    }
}

void PD_DemoSoftTimer_init(void)
{
    g_SoftTimerCount = 0;
}

void PD_DemoSoftTimer_deinit(void)
{
}

uint32_t PD_DemoSoftTimer_msGet(void)
{
    return g_SoftTimerCount;
}

uint32_t PD_DemoSoftTimer_getInterval(uint32_t startTime)
{
    if (g_SoftTimerCount > startTime)
    {
        return (g_SoftTimerCount - startTime);
    }
    else
    {
        return (0xFFFFFFFFu - (startTime - g_SoftTimerCount));
    }
}

void PD_DemoSoftTimer_msSleep(uint32_t delay)
{
    uint32_t tmp = g_SoftTimerCount;
    uint32_t interval;

    do
    {
        if (g_SoftTimerCount > tmp)
        {
            interval = g_SoftTimerCount - tmp;
        }
        else
        {
            interval = 0xFFFFFFFFu - (tmp - g_SoftTimerCount);
        }
    } while (interval < delay);
}
