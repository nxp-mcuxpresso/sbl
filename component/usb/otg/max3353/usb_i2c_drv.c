/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
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

#include "usb_otg.h"
#include "fsl_i2c.h"
#include "usb_i2c_drv.h"

usb_status_t USB_I2CMasterInit(void *base, uint32_t baudRate, uint32_t clockSource)
{
    i2c_master_config_t masterConfig;
    uint32_t sourceClock;

    /* initialize IIC */
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = baudRate;
    sourceClock = CLOCK_GetFreq((clock_name_t)clockSource);
    I2C_MasterInit(base, &masterConfig, sourceClock);

    return kStatus_USB_Success;
}

usb_status_t USB_I2CMasterDeinit(void *base)
{
    I2C_MasterDeinit(base);
    return kStatus_USB_Success;
}

usb_status_t USB_I2CMasterWriteRegister(void *base, uint8_t slaveAddress, uint8_t registerAddress, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    status_t status;

    masterXfer.slaveAddress = slaveAddress;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = registerAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(base, &masterXfer);
    if (status != kStatus_Success)
    {
        return kStatus_USB_Error;
    }

    return kStatus_USB_Success;
}

usb_status_t USB_I2CMasterReadRegister(void *base, uint8_t slaveAddress, uint8_t registerAddress, uint8_t *value)
{
    i2c_master_transfer_t masterXfer;
    status_t status;

    masterXfer.slaveAddress = slaveAddress;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = registerAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(base, &masterXfer);
    if (status != kStatus_Success)
    {
        return kStatus_USB_Error;
    }

    return kStatus_USB_Success;
}
