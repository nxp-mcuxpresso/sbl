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
#include <stdint.h>
#include <stdbool.h>
#include "usb_cmsis_wrapper.h"
#include "Driver_SPI.h"

#if (defined PD_CONFIG_CMSIS_SPI_INTERFACE) && (PD_CONFIG_CMSIS_SPI_INTERFACE)

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define CMSIS_DRIVER_INSTANCE_COUNT (3)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void CMSIS_SignalEvent0(uint32_t event);
void CMSIS_SignalEvent1(uint32_t event);
void CMSIS_SignalEvent2(uint32_t event);

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern ARM_DRIVER_SPI USB_WEAK Driver_SPI0;
extern ARM_DRIVER_SPI USB_WEAK Driver_SPI1;
extern ARM_DRIVER_SPI USB_WEAK Driver_SPI2;
static ARM_DRIVER_SPI *s_DriverSPIArray[CMSIS_DRIVER_INSTANCE_COUNT] = {&Driver_SPI0, &Driver_SPI1, &Driver_SPI2};
static cmsis_driver_adapter_t s_CMSISSPIDriverInstance[CMSIS_DRIVER_INSTANCE_COUNT];
const ARM_SPI_SignalEvent_t s_CMSISSPIDriverCallback[CMSIS_DRIVER_INSTANCE_COUNT] = {
    CMSIS_SignalEvent0, CMSIS_SignalEvent1, CMSIS_SignalEvent2};

#if (PD_CONFIG_CMSIS_SPI_INTERFACE > CMSIS_DRIVER_INSTANCE_COUNT)
#error "CMSIS driver error, please increase the instance count"
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

void *CMSIS_GetSPIInterface(uint8_t interface)
{
    if ((interface - kInterface_spi0 + 1) > CMSIS_DRIVER_INSTANCE_COUNT)
    {
        return NULL;
    }
    else
    {
        return (void *)(s_DriverSPIArray[interface - kInterface_spi0]);
    }
}

static void CMSIS_SignalEvent0(uint32_t event)
{
    if (s_CMSISSPIDriverInstance[0].cmsisState != CMSIS_TRANSFERING)
    {
        return;
    }

    switch (event)
    {
        case ARM_SPI_EVENT_TRANSFER_COMPLETE:
            s_CMSISSPIDriverInstance[0].cmsisState = CMSIS_IDLE;
            break;

        case ARM_SPI_EVENT_DATA_LOST:
        case ARM_SPI_EVENT_MODE_FAULT:
        default:
            s_CMSISSPIDriverInstance[0].cmsisState = CMSIS_TRANSFER_ERROR_DONE;
            break;
    }
}

static void CMSIS_SignalEvent1(uint32_t event)
{
    if (s_CMSISSPIDriverInstance[1].cmsisState != CMSIS_TRANSFERING)
    {
        return;
    }

    switch (event)
    {
        case ARM_SPI_EVENT_TRANSFER_COMPLETE:
            s_CMSISSPIDriverInstance[1].cmsisState = CMSIS_IDLE;
            break;

        case ARM_SPI_EVENT_DATA_LOST:
        case ARM_SPI_EVENT_MODE_FAULT:
        default:
            s_CMSISSPIDriverInstance[1].cmsisState = CMSIS_TRANSFER_ERROR_DONE;
            break;
    }
}

static void CMSIS_SignalEvent2(uint32_t event)
{
    if (s_CMSISSPIDriverInstance[2].cmsisState != CMSIS_TRANSFERING)
    {
        return;
    }

    switch (event)
    {
        case ARM_SPI_EVENT_TRANSFER_COMPLETE:
            s_CMSISSPIDriverInstance[2].cmsisState = CMSIS_IDLE;
            break;

        case ARM_SPI_EVENT_DATA_LOST:
        case ARM_SPI_EVENT_MODE_FAULT:
        default:
            s_CMSISSPIDriverInstance[2].cmsisState = CMSIS_TRANSFER_ERROR_DONE;
            break;
    }
}

int32_t CMSIS_SPIInterfaceInit(cmsis_driver_adapter_t **cmsisDriver, uint8_t interface, void *interfaceConfig)
{
    uint8_t index = 0;
    cmsis_driver_adapter_t *cmsis = NULL;
    int32_t status;
    pd_spi_interface_config_t *spiConfig = (pd_spi_interface_config_t *)interfaceConfig;
    void *cmsisInterface = NULL;
    USB_OSA_SR_ALLOC();

    cmsisInterface = CMSIS_GetSPIInterface(interface);
    if (cmsisInterface == NULL)
    {
        return ARM_DRIVER_ERROR;
    }

    USB_OSA_ENTER_CRITICAL();
    for (; index < PD_CONFIG_MAX_PORT; index++)
    {
        if (s_CMSISSPIDriverInstance[index].occupied != 1)
        {
            uint8_t *buffer = (uint8_t *)&s_CMSISSPIDriverInstance[index];
            for (uint32_t j = 0U; j < sizeof(cmsis_driver_adapter_t); j++)
            {
                buffer[j] = 0x00U;
            }
            s_CMSISSPIDriverInstance[index].occupied = 1;
            cmsis = &s_CMSISSPIDriverInstance[index];
            cmsis->callback = (void *)s_CMSISSPIDriverCallback[index];
            break;
        }
    }
    USB_OSA_EXIT_CRITICAL();
    if (cmsis == NULL)
    {
        return ARM_DRIVER_ERROR;
    }

    *cmsisDriver = cmsis;
    cmsis->interface = interface;
    cmsis->spiPCS = spiConfig->pcs;
    cmsis->cmsisState = CMSIS_IDLE;
    cmsis->cmsisInterface = cmsisInterface;

    ARM_SPI_CAPABILITIES spiCap = ((ARM_DRIVER_SPI *)(cmsis->cmsisInterface))->GetCapabilities();
    if (spiCap == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    return ((ARM_DRIVER_SPI *)(cmsis->cmsisInterface))->Initialize(cmsis->callback);
}

int32_t CMSIS_SPIInterfaceDeinit(cmsis_driver_adapter_t *cmsisDriver)
{
    return ((ARM_DRIVER_SPI *)(cmsis->cmsisInterface))->Uninitialize();
}

int32_t CMSIS_SPIInterfaceWriteRegister(
    cmsis_driver_adapter_t *cmsisDriver, uint32_t registerAddr, uint8_t registerLen, const uint8_t *data, uint32_t num)
{
    return ARM_DRIVER_OK;
}

int32_t CMSIS_SPIInterfaceReadRegister(
    cmsis_driver_adapter_t *cmsisDriver, uint32_t registerAddr, uint8_t registerLen, uint8_t *data, uint32_t num)
{
    return ARM_DRIVER_OK;
}

#endif
