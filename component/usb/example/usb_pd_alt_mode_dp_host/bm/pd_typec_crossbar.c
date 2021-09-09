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
#include "board.h"
#include "pd_board_config.h"
#include "usb_pd.h"
#include "string.h"
#include "pd_alt_mode.h"
#include "pd_alt_mode_dp.h"
#include "pd_typec_crossbar.h"
#include "usb_io.h"

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

static void PD_CrossbarOnOffGPIOControl(pd_crossbar_instance_t *crossInstance, uint8_t mux)
{
    uint8_t gpioValue = 0;

    switch (mux)
    {
        case MUX_SHUTDOWN:
            gpioValue = 0;
            break;
        case MUX_DISABLED:
            gpioValue = 0;
            break;
        case MUX_SAFE_MODE: /* safe mode should preserve usb3 if not transitioning from 4lane */
        case MUX_USB3_ONLY:
        case MUX_DP2_LANE_USB3:
        case MUX_DP2_LANE_NO_USB:
        case MUX_DP4_LANE:
            gpioValue = 1;
            break;
        default:
            break;
    }
    crossInstance->crossEnablePinVal = gpioValue;

    if ((crossInstance->crossbarConfig->crossbarControlGPIO != 0xFFu) &&
        (crossInstance->crossbarConfig->crossbarControlPort != 0xFFu) &&
        (crossInstance->crossbarConfig->crossbarControlPin != 0xFFu))
    {
        USB_GpioOutputWritePin(crossInstance->crossbarConfig->crossbarControlGPIO,
                               crossInstance->crossbarConfig->crossbarControlPort,
                               crossInstance->crossbarConfig->crossbarControlPin, gpioValue);
    }
}

static void PD_CrossbarSync(pd_crossbar_instance_t *crossInstance)
{
    CMSIS_PortControlInterfaceWriteRegister(crossInstance->cmsisHandle,
                                            crossInstance->crossbarConfig->crossbarSlaveAddress, 0, 0,
                                            crossInstance->crossbarRegs, SW_CTRL_IDX + 1);
}

static void PD_CrossbarConfigureUSB(pd_crossbar_instance_t *crossInstance)
{
    uint8_t ccOrientReversed;
    PD_Control(crossInstance->pdHandle, PD_CONTROL_GET_TYPEC_ORIENTATION, &ccOrientReversed);
    crossInstance->crossbarRegs[SYS_CTRL_IDX] = SYS_CTRL_SW_EN_MASK;
    crossInstance->crossbarRegs[OP5_CTRL_IDX] = 0;
    crossInstance->crossbarRegs[CROSS_CTRL_IDX] = 0;
    if (ccOrientReversed)
    {
        crossInstance->crossbarRegs[OP1_CTRL_IDX] = 0;
        crossInstance->crossbarRegs[OP2_CTRL_IDX] = OP_CTRL_IP1_MASK;
        crossInstance->crossbarRegs[OP3_CTRL_IDX] = 0;
        crossInstance->crossbarRegs[OP4_CTRL_IDX] = OP_CTRL_IP4_MASK;
    }
    else
    {
        crossInstance->crossbarRegs[OP1_CTRL_IDX] = OP_CTRL_IP1_MASK;
        crossInstance->crossbarRegs[OP2_CTRL_IDX] = 0;
        crossInstance->crossbarRegs[OP3_CTRL_IDX] = OP_CTRL_IP4_MASK;
        crossInstance->crossbarRegs[OP4_CTRL_IDX] = 0;
    }
    crossInstance->crossbarRegs[SW_CTRL_IDX] = (SW_CTRL_OP_1TO4_MASK | SW_CTRL_OP5_SET_MASK | SW_CTRL_X5_SET_MASK);
    PD_CrossbarSync(crossInstance);
}

static void PD_CrossbarShutdownMux(pd_crossbar_instance_t *crossInstance)
{
    uint8_t index;
    /* disable */
    for (index = SYS_CTRL_IDX; index < SW_CTRL_IDX + 1; index++)
    {
        /* clear all registers and place in shutdown mode */
        crossInstance->crossbarRegs[index] = 0;
    }
    PD_CrossbarSync(crossInstance);
}

static void PD_CrossbarDisableMux(pd_crossbar_instance_t *crossInstance)
{
    uint8_t index;
    /* disable switches, keep outputs enabled */
    for (index = OP1_CTRL_IDX; index <= CROSS_CTRL_IDX; index++)
    {
        /* clear all registers and place in shutdown mode */
        crossInstance->crossbarRegs[index] = 0;
    }
    crossInstance->crossbarRegs[SW_CTRL_IDX] = (SW_CTRL_OP_1TO4_MASK | SW_CTRL_OP5_SET_MASK | SW_CTRL_X5_SET_MASK);
    PD_CrossbarSync(crossInstance);
}

static void PD_CrossbarSafeMode(pd_crossbar_instance_t *crossInstance)
{
    uint8_t ccOrientReversed;
    PD_Control(crossInstance->pdHandle, PD_CONTROL_GET_TYPEC_ORIENTATION, &ccOrientReversed);
    /* No need to change all values as this is an intermediate state
     * crossbar_regs[SYS_CTRL_IDX] = SYS_CTRL_SW_EN_MASK; */
    switch (crossInstance->preControlMux)
    {
        case MUX_DP2_LANE_USB3:
            /* Maintain the usb path (SSTX, SSRC) unchanged, isolate others */
            if (ccOrientReversed)
            {
                crossInstance->crossbarRegs[OP1_CTRL_IDX] = 0;
                crossInstance->crossbarRegs[OP3_CTRL_IDX] = 0;
            }
            else
            {
                crossInstance->crossbarRegs[OP2_CTRL_IDX] = 0;
                crossInstance->crossbarRegs[OP4_CTRL_IDX] = 0;
            }
            break;
        case MUX_DP2_LANE_NO_USB:
        case MUX_DP4_LANE:
            /* All USB signals were repurposed */
            crossInstance->crossbarRegs[OP1_CTRL_IDX] = 0;
            crossInstance->crossbarRegs[OP2_CTRL_IDX] = 0;
            crossInstance->crossbarRegs[OP3_CTRL_IDX] = 0;
            crossInstance->crossbarRegs[OP4_CTRL_IDX] = 0;
            break;
        default:
            /* No signals have been repurposed */
            return;
    }
    /* Always place the SBUs into safe mode */
    crossInstance->crossbarRegs[OP5_CTRL_IDX] = 0;
    crossInstance->crossbarRegs[CROSS_CTRL_IDX] = 0;
    /* No need to change all values as this is an intermediate state
     * crossbar_regs[SW_CTRL_IDX]  = (SW_CTRL_OP_1TO4_MASK|SW_CTRL_OP5_SET_MASK|SW_CTRL_X5_SET_MASK); */
    PD_CrossbarSync(crossInstance);
}

static void PD_CrossbarSetModeVDO(pd_crossbar_instance_t *crossInstance, uint8_t mux, uint32_t configVDO)
{
    uint8_t ccOrientReversed;
    PD_Control(crossInstance->pdHandle, PD_CONTROL_GET_TYPEC_ORIENTATION, &ccOrientReversed);
    uint8_t op1idx;
    uint8_t op2idx;
    uint8_t op3idx;
    uint8_t op4idx;
    pd_dp_configure_obj_t configueObj;
    crossInstance->crossbarRegs[SYS_CTRL_IDX] = SYS_CTRL_SW_EN_MASK;

    /* Aux channel */
    crossInstance->crossbarRegs[OP5_CTRL_IDX] = OP_CTRL_IP7_MASK; /* Don't expect to use use IP8 */

    if (ccOrientReversed)
    {
        op1idx = OP2_CTRL_IDX;
        op2idx = OP1_CTRL_IDX;
        op3idx = OP4_CTRL_IDX;
        op4idx = OP3_CTRL_IDX;
        crossInstance->crossbarRegs[CROSS_CTRL_IDX] = CROSS_CTRL_CROSS;
    }
    else
    {
        op1idx = OP1_CTRL_IDX;
        op2idx = OP2_CTRL_IDX;
        op3idx = OP3_CTRL_IDX;
        op4idx = OP4_CTRL_IDX;
        crossInstance->crossbarRegs[CROSS_CTRL_IDX] = CROSS_CTRL_PASS;
    }

    configueObj.configureVal = configVDO;
    switch (configueObj.bitFields.configureUFPUPin & kPinAssign_All)
    {
        case kPinAssign_C:
        case kPinAssign_E:
            crossInstance->crossbarRegs[op1idx] = OP_CTRL_IP2_MASK;
            crossInstance->crossbarRegs[op2idx] = OP_CTRL_IP3_MASK;
            crossInstance->crossbarRegs[op3idx] = OP_CTRL_IP5_MASK;
            crossInstance->crossbarRegs[op4idx] = OP_CTRL_IP6_MASK;
            crossInstance->crossbarRegs[SW_CTRL_IDX] =
                (SW_CTRL_OP_1TO4_MASK | SW_CTRL_OP5_SET_MASK | SW_CTRL_X5_SET_MASK);
            break;
        case kPinAssign_D:
        case kPinAssign_F:
            crossInstance->crossbarRegs[op2idx] = OP_CTRL_IP3_MASK;
            crossInstance->crossbarRegs[op4idx] = OP_CTRL_IP6_MASK;
            if (mux == MUX_DP2_LANE_USB3)
            {
                crossInstance->crossbarRegs[op1idx] = OP_CTRL_IP1_MASK;
                crossInstance->crossbarRegs[op3idx] = OP_CTRL_IP4_MASK;
            }
            else
            {
                crossInstance->crossbarRegs[op1idx] = 0;
                crossInstance->crossbarRegs[op3idx] = 0;
            }
            crossInstance->crossbarRegs[SW_CTRL_IDX] =
                (SW_CTRL_OP_1TO4_MASK | SW_CTRL_OP5_SET_MASK | SW_CTRL_X5_SET_MASK);
            break;
        case kPinAssign_A:
        case kPinAssign_B:
        /* Pin assignments A and B are not supported, so disable the crossbar */
        default:
            crossInstance->crossbarRegs[SW_CTRL_IDX] = 0;
            break;
    }

    PD_CrossbarSync(crossInstance);
}

/* 0 - intialize success; other values - fail */
uint8_t PD_CrossbarInit(pd_crossbar_instance_t *crossInstance, pd_handle pdHandle, void *config)
{
    /* initialize crossbar enable pin */
    crossInstance->pdHandle = pdHandle;
    crossInstance->crossbarConfig = (const pd_crossbar_config_t *)config;

    if ((crossInstance->crossbarConfig->crossbarControlGPIO != 0xFFu) &&
        (crossInstance->crossbarConfig->crossbarControlPort != 0xFFu) &&
        (crossInstance->crossbarConfig->crossbarControlPin != 0xFFu))
    {
        USB_GpioOutputInit(crossInstance->crossbarConfig->crossbarControlGPIO,
                           crossInstance->crossbarConfig->crossbarControlPort,
                           crossInstance->crossbarConfig->crossbarControlPin);
    }

    /* initialize I2C */
    CMSIS_PortControlInterfaceInit(&crossInstance->cmsisHandle, kInterface_i2c0 + BOARD_PD_I2C_INDEX, NULL);
    if (crossInstance->cmsisHandle == NULL)
    {
        return 1;
    }
    crossInstance->crossbarRegs[0] = SYS_CTRL_IDX;

    PD_CrossbarOnOffGPIOControl(crossInstance, MUX_SHUTDOWN);
    PD_CrossbarSetMux(crossInstance, MUX_DISABLED, 0);

    return 0;
}

uint8_t PD_CrossbarDeinit(pd_crossbar_instance_t *crossInstance)
{
    CMSIS_PortControlInterfaceDeinit(crossInstance->cmsisHandle);
    crossInstance->cmsisHandle = NULL;
    return 0;
}

void PD_CrossbarSetMux(pd_crossbar_instance_t *crossInstance, uint8_t mux, uint32_t modeVDO)
{
    PD_CrossbarOnOffGPIOControl(crossInstance, mux);
    switch (mux)
    {
        case MUX_USB3_ONLY:
            PD_CrossbarConfigureUSB(crossInstance);
            break;
        case MUX_SHUTDOWN:
            PD_CrossbarShutdownMux(crossInstance);
            break;
        case MUX_DISABLED:
            PD_CrossbarDisableMux(crossInstance);
            break;
        case MUX_SAFE_MODE:
            PD_CrossbarSafeMode(crossInstance);
            break;
        case MUX_DP2_LANE_USB3:
        case MUX_DP2_LANE_NO_USB:
        case MUX_DP4_LANE:
            PD_CrossbarSetModeVDO(crossInstance, mux, modeVDO);
            break;
        default:
            break;
    }

    crossInstance->preControlMux = mux;
}
