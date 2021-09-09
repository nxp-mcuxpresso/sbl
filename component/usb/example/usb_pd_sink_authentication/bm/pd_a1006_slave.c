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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "pd_i2c_over_vdm.h"
#include "pd_command_interface.h"
#include "pd_app.h"
#include "pd_board_config.h"
#include "fsl_debug_console.h"

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

uint8_t PD_A1006SlaveInit(pd_app_t *pdAppInstance)
{
    /* initialize CMSIS i2c interface */
    pdAppInstance->cmsisHandle = NULL;
    CMSIS_PortControlInterfaceInit(&pdAppInstance->cmsisHandle, kInterface_i2c0 + BOARD_PD_I2C_INDEX, NULL);
    if (pdAppInstance->cmsisHandle == NULL)
    {
        return 1;
    }
    pd_i2c_over_vdm_config_t config;
    config.pdHandle = pdAppInstance->pdHandle;
    config.i2cSlaveAddress = 0u; /* slave don't need this parameter */
    config.cmsisHandle = pdAppInstance->cmsisHandle;
    config.callback = NULL;
    config.callbackParam = pdAppInstance;
    return PD_I2cOverVdmInit(&(pdAppInstance->i2cHandle), &config);
}

uint32_t PD_DemoStartUpGetBatteryValue(pd_app_t *pdAppInstance)
{
    return 10; /* default start up battery is 10% */
}

void PD_DemoBatteryChange(pd_app_t *pdAppInstance)
{
    if (pdAppInstance->contractValid)
    {
        if (pdAppInstance->batteryQuantity < 100)
        {
            pdAppInstance->batteryQuantity++;
        }

        /* battery charge faster */
        if (pdAppInstance->sinkRequestVoltage > 5000)
        {
            if (pdAppInstance->batteryQuantity < 100)
            {
                pdAppInstance->batteryQuantity++;
            }
        }
    }
    else
    {
        if (pdAppInstance->batteryQuantity > 0)
        {
            pdAppInstance->batteryQuantity--;
        }
    }
}

void PD_DemoReset(pd_app_t *pdAppInstance)
{
    pdAppInstance->batteryChange = 0;
    pdAppInstance->contractValid = 0;
}

void PD_DemoInit(void)
{
    uint8_t index;
    for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
    {
        g_PDAppInstanceArray[index]->batteryQuantity = PD_DemoStartUpGetBatteryValue(g_PDAppInstanceArray[index]);
        PD_DemoReset(g_PDAppInstanceArray[index]);
        if (PD_A1006SlaveInit(g_PDAppInstanceArray[index]))
        {
            PRINTF("a1006 slave init fail\r\n");
        }
    }
}

void PD_Demo1msIsrProcess(void)
{
    static volatile uint32_t delay = 0;
    uint8_t index;
    delay++;

    /* 2 s */
    if (delay >= 2000)
    {
        delay = 0;
        for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
        {
            PD_DemoBatteryChange(g_PDAppInstanceArray[index]);
            g_PDAppInstanceArray[index]->batteryChange = 1;
        }
    }
}

pd_status_t PD_DemoFindPDO(
    pd_app_t *pdAppInstance, pd_rdo_t *rdo, uint32_t requestVoltagemV, uint32_t requestCurrentmA, uint32_t *voltage)
{
    uint32_t index;
    pd_source_pdo_t sourcePDO;
    uint8_t findSourceCap = 0;

    if (pdAppInstance->partnerSourceCapNumber == 0)
    {
        return kStatus_PD_Error;
    }

    /* default rdo as 5V - 0.5A or less */
    *voltage = 5000;
    rdo->bitFields.objectPosition = 1;
    rdo->bitFields.giveBack = 0;
    rdo->bitFields.capabilityMismatch = 0;
    rdo->bitFields.usbCommunicationsCapable = 0;
    rdo->bitFields.noUsbSuspend = 1;
    rdo->bitFields.operateValue = 500 / PD_PDO_CURRENT_UNIT;
    if (rdo->bitFields.operateValue > pdAppInstance->partnerSourceCaps[0].fixedPDO.maxCurrent)
    {
        rdo->bitFields.operateValue = pdAppInstance->partnerSourceCaps[0].fixedPDO.maxCurrent;
    }
    rdo->bitFields.maxOrMinOperateValue = rdo->bitFields.operateValue;

    for (index = 0; index < pdAppInstance->partnerSourceCapNumber; ++index)
    {
        sourcePDO.PDOValue = pdAppInstance->partnerSourceCaps[index].PDOValue;
        switch (sourcePDO.commonPDO.pdoType)
        {
            case kPDO_Fixed:
            {
                if ((sourcePDO.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT == requestVoltagemV) &&
                    (sourcePDO.fixedPDO.maxCurrent * PD_PDO_CURRENT_UNIT >= requestCurrentmA))
                {
                    *voltage = sourcePDO.fixedPDO.voltage * PD_PDO_VOLTAGE_UNIT;
                    rdo->bitFields.objectPosition = (index + 1);
                    if (requestCurrentmA == 0u)
                    {
                        rdo->bitFields.operateValue = sourcePDO.fixedPDO.maxCurrent;
                    }
                    else
                    {
                        rdo->bitFields.operateValue = requestCurrentmA / PD_PDO_CURRENT_UNIT;
                    }
                    rdo->bitFields.maxOrMinOperateValue = rdo->bitFields.operateValue;
                    findSourceCap = 1;
                }
                break;
            }

            case kPDO_Variable:
            {
                if ((sourcePDO.variablePDO.minVoltage * PD_PDO_VOLTAGE_UNIT <= requestVoltagemV) &&
                    (sourcePDO.variablePDO.maxVoltage * PD_PDO_VOLTAGE_UNIT >= requestVoltagemV) &&
                    (sourcePDO.variablePDO.maxCurrent * PD_PDO_CURRENT_UNIT >= requestCurrentmA))
                {
                    *voltage = sourcePDO.variablePDO.minVoltage * PD_PDO_VOLTAGE_UNIT;
                    rdo->bitFields.objectPosition = (index + 1);
                    if (requestCurrentmA == 0u)
                    {
                        rdo->bitFields.operateValue = sourcePDO.variablePDO.maxCurrent;
                    }
                    else
                    {
                        rdo->bitFields.operateValue = requestCurrentmA / PD_PDO_CURRENT_UNIT;
                    }
                    rdo->bitFields.maxOrMinOperateValue = rdo->bitFields.operateValue;
                    findSourceCap = 1;
                }
                break;
            }

            case kPDO_Battery:
            {
                if ((sourcePDO.batteryPDO.minVoltage * PD_PDO_VOLTAGE_UNIT <= requestVoltagemV) &&
                    (sourcePDO.batteryPDO.maxVoltage * PD_PDO_VOLTAGE_UNIT >= requestVoltagemV) &&
                    (sourcePDO.batteryPDO.maxAllowPower * PD_PDO_POWER_UNIT >=
                     (requestVoltagemV * requestCurrentmA / 1000)))
                {
                    *voltage = sourcePDO.batteryPDO.minVoltage * PD_PDO_VOLTAGE_UNIT;
                    rdo->bitFields.objectPosition = (index + 1);
                    if (requestCurrentmA == 0u)
                    {
                        rdo->bitFields.operateValue = sourcePDO.batteryPDO.maxAllowPower;
                    }
                    else
                    {
                        rdo->bitFields.operateValue = (requestVoltagemV * requestCurrentmA) / 1000 / PD_PDO_POWER_UNIT;
                    }
                    rdo->bitFields.maxOrMinOperateValue = rdo->bitFields.operateValue;
                    findSourceCap = 1;
                }
                break;
            }

            default:
                break;
        }

        if (findSourceCap)
        {
            break;
        }
    }

    if (findSourceCap)
    {
        return kStatus_PD_Success;
    }
    return kStatus_PD_Error;
}

void PD_DemoTaskFn(void)
{
    uint8_t index;
    pd_app_t *pdAppInstance;

    for (index = 0; index < PD_DEMO_PORTS_COUNT; ++index)
    {
        pdAppInstance = g_PDAppInstanceArray[index];
        if (pdAppInstance->batteryChange == 1)
        {
            pdAppInstance->batteryChange = 0;

            PRINTF("battery percent:%d, charge voltage:%dV\r\n", pdAppInstance->batteryQuantity,
                   (pdAppInstance->contractValid) ? (pdAppInstance->sinkRequestVoltage / 1000) : 0u);
        }
    }
}
