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
#include "pd_i2c_over_vdm.h"
#include "pd_command_interface.h"
#include "pd_app.h"
#include "fsl_gpio.h"
#include "pd_power_interface.h"

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

void PD_PowerBoardControlInit(pd_power_control_instance_t *powerControl, pd_handle *pdHandle)
{
    powerControl->sourceVbusVoltage = 0;
    powerControl->pdHandle = pdHandle;
}

void PD_PowerBoardControlDeinit(pd_power_control_instance_t *powerControl)
{
    powerControl->sourceVbusVoltage = 0;
    powerControl->pdHandle = NULL;
}

pd_status_t PD_PowerBoardReset(pd_power_control_instance_t *powerControl)
{
    pd_ptn5110_ctrl_pin_t phyPowerPinCtrl;

    phyPowerPinCtrl.enSRC = 0;
    phyPowerPinCtrl.enSNK1 = 0;
    PD_Control(powerControl->pdHandle, PD_CONTROL_PHY_POWER_PIN, &phyPowerPinCtrl);
    PD_Control(powerControl->pdHandle, PD_CONTROL_DISCHARGE_VBUS, NULL);
    return kStatus_PD_Success;
}

/***************source need implement follow vbus power related functions***************/

/***************sink need implement follow vbus power related functions***************/

pd_status_t PD_PowerBoardSinkEnableVbusPower(pd_power_control_instance_t *powerControl, pd_vbus_power_t vbusPower)
{
    pd_ptn5110_ctrl_pin_t phyPowerPinCtrl;
    uint32_t voltage;

    phyPowerPinCtrl.enSRC = 0;
    phyPowerPinCtrl.enSNK1 = 1;
    PD_Control(powerControl->pdHandle, PD_CONTROL_PHY_POWER_PIN, &phyPowerPinCtrl);
    voltage = (vbusPower.minVoltage * 50) | ((vbusPower.maxVoltage * 50) << 16);
    PD_Control(powerControl->pdHandle, PD_CONTROL_INFORM_VBUS_VOLTAGE_RANGE, &voltage);
    return kStatus_PD_Success;
}

/***************if support vconn, need implement the follow related functions***************/
pd_status_t PD_PowerBoardControlVconn(pd_power_control_instance_t *powerControl, uint8_t on)
{
    uint8_t controlVal;
    controlVal = (on ? 1 : 0);
    PD_Control(powerControl->pdHandle, PD_CONTROL_VCONN, &controlVal);
    return kStatus_PD_Success;
}
