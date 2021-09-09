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
#include "usb_pd.h"
#include "Driver_Common.h"
#include "usb_osa.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if (defined PD_CONFIG_CMSIS_I2C_INTERFACE) && (PD_CONFIG_CMSIS_I2C_INTERFACE)
extern int32_t CMSIS_I2CInterfaceInit(void **cmsisI2CDriver, uint8_t interface, void *interfaceConfig);
extern int32_t CMSIS_I2CInterfaceDeinit(void *cmsisI2CDriver);
extern int32_t CMSIS_I2CInterfaceWriteRegister(void *cmsisI2CDriver,
                                               uint32_t slave,
                                               uint32_t registerAddr,
                                               uint8_t registerLen,
                                               const uint8_t *data,
                                               uint32_t num);
extern int32_t CMSIS_I2CInterfaceReadRegister(
    void *cmsisI2CDriver, uint32_t slave, uint32_t registerAddr, uint8_t registerLen, uint8_t *data, uint32_t num);
#endif

#if (defined PD_CONFIG_CMSIS_SPI_INTERFACE) && (PD_CONFIG_CMSIS_SPI_INTERFACE)
extern int32_t CMSIS_SPIInterfaceInit(void **cmsisSPIDriver, uint8_t interface, void *interfaceConfig);
extern int32_t CMSIS_SPIInterfaceDeinit(void *cmsisSPIDriver);
extern int32_t CMSIS_SPIInterfaceWriteRegister(void *cmsisSPIDriver,
                                               uint32_t slave,
                                               uint32_t registerAddr,
                                               uint8_t registerLen,
                                               const uint8_t *data,
                                               uint32_t num);
extern int32_t CMSIS_SPIInterfaceReadRegister(
    void *cmsisSPIDriver, uint32_t slave, uint32_t registerAddr, uint8_t registerLen, uint8_t *data, uint32_t num);
#endif

static cmsis_driver_adapter_t s_CMSISDriverInstance[CMSIS_DRIVER_WRAPPER_INSTANCE_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

int32_t CMSIS_PortControlInterfaceInit(usb_cmsis_wrapper_handle *wrapperHandle,
                                       uint8_t interface,
                                       void *interfaceConfig)
{
    int32_t status = ARM_DRIVER_ERROR;
    cmsis_driver_adapter_t *cmsisWrapper = NULL;
    uint8_t index;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    for (index = 0; index < CMSIS_DRIVER_WRAPPER_INSTANCE_COUNT; index++)
    {
        if ((s_CMSISDriverInstance[index].refCount) && (s_CMSISDriverInstance[index].interface == interface))
        {
            s_CMSISDriverInstance[index].refCount++;
            *wrapperHandle = &s_CMSISDriverInstance[index];
            USB_OSA_EXIT_CRITICAL();
            return ARM_DRIVER_OK;
        }
    }

    for (index = 0; index < CMSIS_DRIVER_WRAPPER_INSTANCE_COUNT; index++)
    {
        if (s_CMSISDriverInstance[index].refCount == 0)
        {
            uint8_t *buffer = (uint8_t *)&s_CMSISDriverInstance[index];
            for (uint32_t j = 0U; j < sizeof(cmsis_driver_adapter_t); j++)
            {
                buffer[j] = 0x00U;
            }
            s_CMSISDriverInstance[index].refCount = 1;
            cmsisWrapper = &s_CMSISDriverInstance[index];
            break;
        }
    }
    USB_OSA_EXIT_CRITICAL();
    if (cmsisWrapper == NULL)
    {
        return ARM_DRIVER_ERROR;
    }
    cmsisWrapper->interface = interface;
    *wrapperHandle = cmsisWrapper;

#if (defined PD_CONFIG_CMSIS_I2C_INTERFACE) && (PD_CONFIG_CMSIS_I2C_INTERFACE)
    if (interface < kInterface_spi0)
    {
        status = CMSIS_I2CInterfaceInit(&cmsisWrapper->cmsisDrvHandle, interface, interfaceConfig);
    }
#endif

#if (defined PD_CONFIG_CMSIS_SPI_INTERFACE) && (PD_CONFIG_CMSIS_SPI_INTERFACE)
    if ((interface >= kInterface_spi0) && (interface < kInterface_end))
    {
        status = CMSIS_SPIInterfaceInit(&cmsisWrapper->cmsisDrvHandle, interface, interfaceConfig);
    }
#endif

    if ((status == ARM_DRIVER_OK) && (cmsisWrapper != NULL))
    {
        if (USB_OsaMutexCreate(&(cmsisWrapper->cmsisMutex)) != kStatus_USB_OSA_Success)
        {
            cmsisWrapper->cmsisMutex = NULL;
            status = ARM_DRIVER_ERROR;
        }
    }

    return status;
}

int32_t CMSIS_PortControlInterfaceDeinit(usb_cmsis_wrapper_handle wrapperHandle)
{
    int32_t status = ARM_DRIVER_ERROR;
    cmsis_driver_adapter_t *cmsisWrapper = (cmsis_driver_adapter_t *)wrapperHandle;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if (cmsisWrapper->refCount > 0)
    {
        cmsisWrapper->refCount--;
        if (cmsisWrapper->refCount > 0)
        {
            USB_OSA_EXIT_CRITICAL();
            return ARM_DRIVER_OK;
        }
    }
    USB_OSA_EXIT_CRITICAL();

#if (defined PD_CONFIG_CMSIS_I2C_INTERFACE) && (PD_CONFIG_CMSIS_I2C_INTERFACE)
    if (cmsisWrapper->interface < kInterface_spi0)
    {
        status = CMSIS_I2CInterfaceDeinit(cmsisWrapper->cmsisDrvHandle);
    }
#endif

#if (defined PD_CONFIG_CMSIS_SPI_INTERFACE) && (PD_CONFIG_CMSIS_SPI_INTERFACE)
    if ((cmsisWrapper->interface >= kInterface_spi0) && (cmsisWrapper->interface < kInterface_end))
    {
        status = CMSIS_SPIInterfaceDeinit(cmsisWrapper->cmsisDrvHandle);
    }
#endif

    if (cmsisWrapper->cmsisMutex != NULL)
    {
        USB_OsaMutexDestroy(cmsisWrapper->cmsisMutex);
        cmsisWrapper->cmsisMutex = NULL;
    }

    return status;
}

int32_t CMSIS_PortControlInterfaceWriteRegister(usb_cmsis_wrapper_handle wrapperHandle,
                                                uint32_t slave,
                                                uint32_t registerAddr,
                                                uint8_t registerLen,
                                                const uint8_t *data,
                                                uint32_t num)
{
    int32_t status = ARM_DRIVER_ERROR;
    cmsis_driver_adapter_t *cmsisWrapper = (cmsis_driver_adapter_t *)wrapperHandle;

    if (USB_OsaMutexLock(cmsisWrapper->cmsisMutex) != kStatus_USB_OSA_Success)
    {
        return ARM_DRIVER_ERROR;
    }

#if (defined PD_CONFIG_CMSIS_I2C_INTERFACE) && (PD_CONFIG_CMSIS_I2C_INTERFACE)
    if (cmsisWrapper->interface < kInterface_spi0)
    {
        status =
            CMSIS_I2CInterfaceWriteRegister(cmsisWrapper->cmsisDrvHandle, slave, registerAddr, registerLen, data, num);
    }
#endif

#if (defined PD_CONFIG_CMSIS_SPI_INTERFACE) && (PD_CONFIG_CMSIS_SPI_INTERFACE)
    if ((cmsisWrapper->interface >= kInterface_spi0) && (cmsisWrapper->interface < kInterface_end))
    {
        status =
            CMSIS_SPIInterfaceWriteRegister(cmsisWrapper->cmsisDrvHandle, slave, registerAddr, registerLen, data, num);
    }
#endif

    if (USB_OsaMutexUnlock(cmsisWrapper->cmsisMutex) != kStatus_USB_OSA_Success)
    {
        return ARM_DRIVER_ERROR;
    }

    return status;
}

int32_t CMSIS_PortControlInterfaceReadRegister(usb_cmsis_wrapper_handle wrapperHandle,
                                               uint32_t slave,
                                               uint32_t registerAddr,
                                               uint8_t registerLen,
                                               uint8_t *data,
                                               uint32_t num)
{
    int32_t status = ARM_DRIVER_ERROR;
    cmsis_driver_adapter_t *cmsisWrapper = (cmsis_driver_adapter_t *)wrapperHandle;

    if (USB_OsaMutexLock(cmsisWrapper->cmsisMutex) != kStatus_USB_OSA_Success)
    {
        return ARM_DRIVER_ERROR;
    }

#if (defined PD_CONFIG_CMSIS_I2C_INTERFACE) && (PD_CONFIG_CMSIS_I2C_INTERFACE)
    if (cmsisWrapper->interface < kInterface_spi0)
    {
        status =
            CMSIS_I2CInterfaceReadRegister(cmsisWrapper->cmsisDrvHandle, slave, registerAddr, registerLen, data, num);
    }
#endif

#if (defined PD_CONFIG_CMSIS_SPI_INTERFACE) && (PD_CONFIG_CMSIS_SPI_INTERFACE)
    if ((cmsisWrapper->interface >= kInterface_spi0) && (cmsisWrapper->interface < kInterface_end))
    {
        status =
            CMSIS_SPIInterfaceReadRegister(cmsisWrapper->cmsisDrvHandle, slave, registerAddr, registerLen, data, num);
    }
#endif

    if (USB_OsaMutexUnlock(cmsisWrapper->cmsisMutex) != kStatus_USB_OSA_Success)
    {
        return ARM_DRIVER_ERROR;
    }

    return status;
}
