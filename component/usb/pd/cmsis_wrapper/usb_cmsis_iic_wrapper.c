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

#include <stdint.h>
#include <stdbool.h>
#include "usb_pd_config.h"
#include "usb_pd.h"
#include "usb_cmsis_wrapper.h"
#include "Driver_I2C.h"

#if (defined PD_CONFIG_CMSIS_I2C_INTERFACE) && (PD_CONFIG_CMSIS_I2C_INTERFACE)

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum _cmsis_driver_state
{
    CMSIS_IDLE,
    CMSIS_TRANSFERING,
    CMSIS_TRANSFER_ERROR_DONE,
} cmsis_driver_state_t;

typedef struct _cmsis_i2c_driver_adapter
{
    void *cmsisInterface;
    void *callback;
    /* uint16_t i2cAddress; */
    uint8_t occupied;
    uint8_t interface;
#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    volatile uint8_t dataBuffer[140];
#else
    volatile uint8_t dataBuffer[64];
#endif
    volatile uint8_t cmsisState;
    volatile uint8_t cmsisResult;
    volatile uint8_t cmsisTry;
} cmsis_i2c_adapter_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void CMSIS_SignalEvent0(uint32_t event);
static void CMSIS_SignalEvent1(uint32_t event);
static void CMSIS_SignalEvent2(uint32_t event);
static void CMSIS_SignalEvent3(uint32_t event);
static void CMSIS_SignalEvent4(uint32_t event);
static void CMSIS_SignalEvent5(uint32_t event);

#define CMSIS_I2C_CALLBACK_COUNT (6)

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C0;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C1;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C2;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C3;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C4;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C5;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C6;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C7;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C8;
USB_WEAK_VAR ARM_DRIVER_I2C Driver_I2C9;
static ARM_DRIVER_I2C *s_DriverI2CArray[] = {&Driver_I2C0, &Driver_I2C1, &Driver_I2C2, &Driver_I2C3, &Driver_I2C4,
                                             &Driver_I2C5, &Driver_I2C6, &Driver_I2C7, &Driver_I2C8, &Driver_I2C9};
const ARM_I2C_SignalEvent_t s_CMSISI2CDriverCallback[CMSIS_I2C_CALLBACK_COUNT] = {
    CMSIS_SignalEvent0, CMSIS_SignalEvent1, CMSIS_SignalEvent2,
    CMSIS_SignalEvent3, CMSIS_SignalEvent4, CMSIS_SignalEvent5};
static cmsis_i2c_adapter_t s_CMSISI2CDrvInstance[CMSIS_DRIVER_WRAPPER_INSTANCE_COUNT];
#if (PD_CONFIG_MAX_PORT > CMSIS_DRIVER_WRAPPER_INSTANCE_COUNT)
#error "CMSIS driver error, please increase the wrapper instance count"
#endif
#if (PD_CONFIG_MAX_PORT > CMSIS_I2C_CALLBACK_COUNT)
#error "please increase cmsis i2c callback count"
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void *CMSIS_GetI2cInterface(uint8_t interface)
{
    if ((interface - kInterface_i2c0 + 1) > (sizeof(s_DriverI2CArray) / 4))
    {
        return NULL;
    }
    else
    {
        return (void *)(s_DriverI2CArray[interface - kInterface_i2c0]);
    }
}

USB_WEAK_FUN void BOARD_I2C0_ReleaseBus(void)
{
}

USB_WEAK_FUN void BOARD_I2C1_ReleaseBus(void)
{
}

USB_WEAK_FUN void BOARD_I2C2_ReleaseBus(void)
{
}

USB_WEAK_FUN void BOARD_I2C3_ReleaseBus(void)
{
}

USB_WEAK_FUN void BOARD_I2C4_ReleaseBus(void)
{
}

void CMSIS_I2cReleaseBus(uint8_t interface)
{
    switch (interface)
    {
        case kInterface_i2c0:
            BOARD_I2C0_ReleaseBus();
            break;

        case kInterface_i2c1:
            BOARD_I2C1_ReleaseBus();
            break;

        case kInterface_i2c2:
            BOARD_I2C2_ReleaseBus();
            break;

        case kInterface_i2c3:
            BOARD_I2C3_ReleaseBus();
            break;

        case kInterface_i2c4:
            BOARD_I2C4_ReleaseBus();
            break;
        default:
            break;
    }
}

static void CMSIS_SignalEventCommon(uint8_t index, uint32_t event)
{
    if (s_CMSISI2CDrvInstance[index].cmsisState != CMSIS_TRANSFERING)
    {
        return;
    }

    switch (event)
    {
        case ARM_I2C_EVENT_TRANSFER_DONE:
            s_CMSISI2CDrvInstance[index].cmsisState = CMSIS_IDLE;
            break;

        case ARM_I2C_EVENT_TRANSFER_INCOMPLETE:
        case ARM_I2C_EVENT_ADDRESS_NACK:
        case ARM_I2C_EVENT_ARBITRATION_LOST:
        case ARM_I2C_EVENT_BUS_ERROR:
        case ARM_I2C_EVENT_BUS_CLEAR:
        case ARM_I2C_EVENT_SLAVE_TRANSMIT:
        case ARM_I2C_EVENT_SLAVE_RECEIVE:
        case ARM_I2C_EVENT_GENERAL_CALL:
        default:
            s_CMSISI2CDrvInstance[index].cmsisState = CMSIS_TRANSFER_ERROR_DONE;
            break;
    }
}

static void CMSIS_SignalEvent0(uint32_t event)
{
    CMSIS_SignalEventCommon(0, event);
}

static void CMSIS_SignalEvent1(uint32_t event)
{
    CMSIS_SignalEventCommon(1, event);
}

static void CMSIS_SignalEvent2(uint32_t event)
{
    CMSIS_SignalEventCommon(2, event);
}

static void CMSIS_SignalEvent3(uint32_t event)
{
    CMSIS_SignalEventCommon(3, event);
}

static void CMSIS_SignalEvent4(uint32_t event)
{
    CMSIS_SignalEventCommon(4, event);
}

static void CMSIS_SignalEvent5(uint32_t event)
{
    CMSIS_SignalEventCommon(5, event);
}

int32_t CMSIS_I2CInterfaceInit(void **cmsisI2CDriver, uint8_t interface, void *interfaceConfig)
{
    uint8_t index = 0;
    int32_t status;
    /* pd_i2c_interface_config_t *i2cConfig = (pd_i2c_interface_config_t *)interfaceConfig; */
    void *cmsisInterface = NULL;
    cmsis_i2c_adapter_t *cmsisI2cAdapter = NULL;
    USB_OSA_SR_ALLOC();

    cmsisInterface = CMSIS_GetI2cInterface(interface);
    if (cmsisInterface == NULL)
    {
        return ARM_DRIVER_ERROR;
    }

    USB_OSA_ENTER_CRITICAL();
    for (; index < CMSIS_DRIVER_WRAPPER_INSTANCE_COUNT; index++)
    {
        if (s_CMSISI2CDrvInstance[index].occupied != 1)
        {
            uint8_t *buffer = (uint8_t *)&s_CMSISI2CDrvInstance[index];
            for (uint32_t j = 0U; j < sizeof(cmsis_i2c_adapter_t); j++)
            {
                buffer[j] = 0x00U;
            }
            s_CMSISI2CDrvInstance[index].occupied = 1;
            cmsisI2cAdapter = &s_CMSISI2CDrvInstance[index];
            break;
        }
    }
    USB_OSA_EXIT_CRITICAL();
    if (cmsisI2cAdapter == NULL)
    {
        return ARM_DRIVER_ERROR;
    }

    cmsisI2cAdapter->interface = interface;
    cmsisI2cAdapter->callback = (void *)s_CMSISI2CDriverCallback[index];
    /* cmsis->i2cAddress = i2cConfig->slaveAddress; */
    cmsisI2cAdapter->cmsisState = CMSIS_IDLE;
    cmsisI2cAdapter->cmsisInterface = cmsisInterface;

    status = ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))
                 ->Initialize((ARM_I2C_SignalEvent_t)cmsisI2cAdapter->callback);
    if (status == ARM_DRIVER_OK)
    {
        status = ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->PowerControl(ARM_POWER_FULL);
    }
    if (status == ARM_DRIVER_OK)
    {
        /*config transmit speed*/
        status =
            ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    }

    *cmsisI2CDriver = cmsisI2cAdapter;

    return status;
}

int32_t CMSIS_I2CInterfaceDeinit(void *cmsisI2CDriver)
{
    int32_t status;
    cmsis_i2c_adapter_t *cmsisI2cAdapter = (cmsis_i2c_adapter_t *)cmsisI2CDriver;
    USB_OSA_SR_ALLOC();

    status = ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->Uninitialize();
    USB_OSA_ENTER_CRITICAL();
    cmsisI2cAdapter->occupied = 0;
    USB_OSA_EXIT_CRITICAL();
    return status;
}

static int32_t CMSIS_I2CInterfaceTransfer(
    cmsis_i2c_adapter_t *cmsisI2cAdapter, uint32_t slave, uint8_t send, uint8_t *data, uint32_t num, uint8_t pending)
{
    int32_t result = ARM_DRIVER_ERROR;
    uint32_t waitCount;

    cmsisI2cAdapter->cmsisState = CMSIS_TRANSFERING;
    if (send)
    {
        result = ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))
                     ->MasterTransmit(slave, (const uint8_t *)data, num, pending);
    }
    else
    {
        result = ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->MasterReceive(slave, data, num, pending);
    }

    if (result == ARM_DRIVER_ERROR_BUSY)
    {
        ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->Control(ARM_I2C_ABORT_TRANSFER, 0);
        CMSIS_I2cReleaseBus(cmsisI2cAdapter->interface);
        ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))
            ->Initialize((ARM_I2C_SignalEvent_t)cmsisI2cAdapter->callback);
        ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->PowerControl(ARM_POWER_OFF);
        ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->PowerControl(ARM_POWER_FULL);
    }

    if (result == ARM_DRIVER_OK)
    {
        waitCount = 1000000;
        while (cmsisI2cAdapter->cmsisState == CMSIS_TRANSFERING)
        {
            waitCount--;
            if (waitCount == 0)
            {
                CMSIS_I2cReleaseBus(cmsisI2cAdapter->interface);
                ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))
                    ->Initialize((ARM_I2C_SignalEvent_t)cmsisI2cAdapter->callback);
                ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->Control(ARM_I2C_ABORT_TRANSFER, 0);
                ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->PowerControl(ARM_POWER_OFF);
                ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))->PowerControl(ARM_POWER_FULL);
                cmsisI2cAdapter->cmsisState = CMSIS_TRANSFER_ERROR_DONE;
                break;
            }
        }
        if (cmsisI2cAdapter->cmsisState == CMSIS_IDLE)
        {
            result = ARM_DRIVER_OK;
        }
        else
        {
            result = ARM_DRIVER_ERROR;
        }
    }

    return result;
}

int32_t CMSIS_I2CInterfaceWriteRegister(
    void *cmsisI2CDriver, uint32_t slave, uint32_t registerAddr, uint8_t registerLen, const uint8_t *data, uint32_t num)
{
    int32_t result;
    cmsis_i2c_adapter_t *cmsisI2cAdapter = (cmsis_i2c_adapter_t *)cmsisI2CDriver;
    uint8_t index;
    uint8_t *dataPtr = (uint8_t *)data;

    cmsisI2cAdapter->cmsisTry = CMSIS_TRANSFER_RETRY_COUNT;
    if (registerLen > 0)
    {
        if (registerLen == 1)
        {
            cmsisI2cAdapter->dataBuffer[0] = registerAddr;
        }
        else
        {
            cmsisI2cAdapter->dataBuffer[0] = registerAddr;
            cmsisI2cAdapter->dataBuffer[1] = (registerAddr >> 8);
            registerLen = 2;
        }
        for (index = 0; index < num; ++index)
        {
            cmsisI2cAdapter->dataBuffer[registerLen + index] = data[index];
        }
        dataPtr = (uint8_t *)&(cmsisI2cAdapter->dataBuffer[0]);
    }

    do
    {
        result = ARM_DRIVER_ERROR;
        result = CMSIS_I2CInterfaceTransfer(cmsisI2cAdapter, slave, 1, dataPtr, registerLen + num, 0);

        if (result == ARM_DRIVER_OK)
        {
            break;
        }
        else
        {
            CMSIS_I2cReleaseBus(cmsisI2cAdapter->interface);
            ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))
                ->Initialize((ARM_I2C_SignalEvent_t)cmsisI2cAdapter->callback);
        }
    } while (--cmsisI2cAdapter->cmsisTry);

    return result;
}

int32_t CMSIS_I2CInterfaceReadRegister(
    void *cmsisI2CDriver, uint32_t slave, uint32_t registerAddr, uint8_t registerLen, uint8_t *data, uint32_t num)
{
    int32_t result;
    cmsis_i2c_adapter_t *cmsisI2cAdapter = (cmsis_i2c_adapter_t *)cmsisI2CDriver;

    cmsisI2cAdapter->cmsisTry = CMSIS_TRANSFER_RETRY_COUNT;
    do
    {
        result = ARM_DRIVER_ERROR;
        result = CMSIS_I2CInterfaceTransfer(cmsisI2cAdapter, slave, 1, (uint8_t *)&registerAddr, registerLen, 1);
        if (result == ARM_DRIVER_OK)
        {
            result = CMSIS_I2CInterfaceTransfer(cmsisI2cAdapter, slave, 0, data, num, 0);
        }

        if (result == ARM_DRIVER_OK)
        {
            break;
        }
        else
        {
            CMSIS_I2cReleaseBus(cmsisI2cAdapter->interface);
            ((ARM_DRIVER_I2C *)(cmsisI2cAdapter->cmsisInterface))
                ->Initialize((ARM_I2C_SignalEvent_t)cmsisI2cAdapter->callback);
        }
    } while (--cmsisI2cAdapter->cmsisTry);

    return result;
}

#endif
