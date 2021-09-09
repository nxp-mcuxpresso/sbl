// ############################################################################
// #             Copyright (C), NXP Semiconductors                            #
// #                       (C), NXP B.V. of Eindhoven                         #
// #                                                                          #
// # All rights are reserved. Reproduction in whole or in part is prohibited  #
// # without the written consent of the copyright owner.                      #
// # NXP reserves the right to make changes without notice at any time.       #
// # NXP makes no warranty, expressed, implied or statutory, including but    #
// # not limited to any implied warranty of merchantibility or fitness for    #
// # any particular purpose, or that the use will not infringe any third      #
// # party patent, copyright or trademark. NXP must not be liable for any     #
// # loss or damage arising from its use.                                     #
// ############################################################################

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

volatile uint32_t g_SoftTimerCount;
volatile uint32_t g_SetTimer;

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

void softTimer_init(void)
{
    g_SoftTimerCount = 0;
}

uint32_t softTimer_msGet(void)
{
    return g_SoftTimerCount;
}

void softTimer_msSleep(uint32_t delay)
{
    uint32_t tmp = g_SoftTimerCount;
    while (g_SoftTimerCount < (tmp + delay))
    {
        __ASM("nop");
    }
}

void softTimer_Start(uint32_t ms)
{
    g_SetTimer = ms;
}

uint8_t softTimer_TimeOut(void)
{
    return (g_SetTimer == 0);
}
