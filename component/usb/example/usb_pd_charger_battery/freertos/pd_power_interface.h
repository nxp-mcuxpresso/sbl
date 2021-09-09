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

#ifndef __PD_POWER_INTERFACE_T__
#define __PD_POWER_INTERFACE_T__

#define PD_POWER_REQUEST_MIN_VOLTAGE (5000 / 50) /* 5V */

#define VSAFE5V_IN_50MV (5000 / 50)
#define VBUS_REQ_20V (20000 / 50)
#define VBUS_REQ_5V (5000 / 50)

typedef enum _pd_request_value_type
{
    kRequestPower_Current,
    kRequestPower_Power,
} pd_request_value_type_t;

typedef struct _pd_vbus_power
{
    uint32_t valueType : 2; /* pd_request_value_type_t */
    uint32_t minVoltage : 10;
    uint32_t maxVoltage : 10;
    uint32_t requestValue : 10; /* current or power according to valueType */
} pd_vbus_power_t;

typedef struct _pd_power_control_instance
{
    uint32_t sourceVbusVoltage;
    void *pdHandle;
} pd_power_control_instance_t;

pd_status_t PD_PowerSrcTurnOnDefaultVbus(void *callbackParam, uint8_t powerProgress);
pd_status_t PD_PowerSrcTurnOnRequestVbus(void *callbackParam, pd_rdo_t rdo);
pd_status_t PD_PowerSrcTurnOffVbus(void *callbackParam, uint8_t powerProgress);
pd_status_t PD_PowerSrcGotoMinReducePower(void *callbackParam);
pd_status_t PD_PowerSnkDrawTypeCVbus(void *callbackParam, uint8_t typecCurrentLevel, uint8_t powerProgress);
pd_status_t PD_PowerSnkDrawRequestVbus(void *callbackParam, pd_rdo_t rdo);
pd_status_t PD_PowerSnkStopDrawVbus(void *callbackParam, uint8_t powerProgress);
pd_status_t PD_PowerSnkGotoMinReducePower(void *callbackParam);
pd_status_t PD_PowerControlVconn(void *callbackParam, uint8_t on);

uint32_t *PD_PowerBoardGetPartnerSourceCaps(void *callbackParam);
uint32_t *PD_PowerBoardGetSelfSourceCaps(void *callbackParam);
pd_status_t PD_PowerBoardReset(pd_power_control_instance_t *powerControl);
pd_status_t PD_PowerBoardSourceEnableVbusPower(pd_power_control_instance_t *powerControl, pd_vbus_power_t vbusPower);
pd_status_t PD_PowerBoardSinkEnableVbusPower(pd_power_control_instance_t *powerControl, pd_vbus_power_t vbusPower);
pd_status_t PD_PowerBoardControlVconn(pd_power_control_instance_t *powerControl, uint8_t on);
void PD_PowerBoardControlInit(pd_power_control_instance_t *powerControl, pd_handle *pdHandle);

#endif
