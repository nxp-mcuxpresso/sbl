/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#include "usb_otg_config.h"

#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))

#include "usb_otg.h"
#include "fsl_device_registers.h"
#include "usb_otg_khci.h"
#include "usb_otg_oci.h"
#include "usb_otg_max3353.h"
#include "fsl_port.h"
#include "usb_otg_mouse.h"
#include "usb_i2c_drv.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define BOARD_MAX3353_MID (0x6A0B5333)
#define BOARD_MAX3353_PID (0x485A4200)
#define USB_MAX3353_GPIO_INTERRUPT_PRIORITY (4)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static usb_status_t USB_Max3353ReadByte(uint8_t address, uint8_t *buffer);
static usb_status_t USB_Max3353WriteByte(uint8_t address, uint8_t value);
static void MAX3353_GPIOIRQHandler(usb_otg_handle otgHandle);
usb_status_t USB_OtgPeripheralDisable(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static usb_otg_max3353_run_t s_Max3353RunParameter;
extern usb_otg_handle g_OtgHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/

void GPIO_IRQHandler(void)
{
    MAX3353_GPIOIRQHandler(g_OtgHandle);
   /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping 
     exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

static void MAX3353_GPIOIRQHandler(usb_otg_handle otgHandle)
{
    if (BOARD_MAX3353_GPIO_PORT->ISFR & (1U << BOARD_MAX3353_GPIO_PIN))
    {
        BOARD_MAX3353_GPIO_PORT->ISFR = 1U << BOARD_MAX3353_GPIO_PIN;

        USB_OtgNotifyChange(otgHandle, kOtg_StatusChange, 1);
    }
   /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping 
     exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

static usb_status_t USB_Max3353ReadByte(uint8_t address, uint8_t *buffer)
{
    return USB_I2CMasterReadRegister(BOARD_MAX3353_I2C, BOARD_MAX3353_I2C_SLAVE_ADDR_7BIT, address, buffer);
}

static usb_status_t USB_Max3353WriteByte(uint8_t address, uint8_t value)
{
    return USB_I2CMasterWriteRegister(BOARD_MAX3353_I2C, BOARD_MAX3353_I2C_SLAVE_ADDR_7BIT, address, value);
}

usb_status_t USB_OtgMax3353Init(void)
{
    uint8_t index;
    uint8_t byteRead;
    uint32_t id;
    status_t status;

    s_Max3353RunParameter.statusRegister = 0x00U;
    s_Max3353RunParameter.control1Register = 0x00U;
    s_Max3353RunParameter.control2Register = 0x00U;
    s_Max3353RunParameter.peripheralStatus = 0x00U;

    USB_I2CMasterInit(BOARD_MAX3353_I2C, BOARD_MAX3353_I2C_BAUDRATE, BOARD_MAX3353_I2C_CLK_SRC);

    /* check peripheral */
    id = 0;
    for (index = 0; index < 4; ++index)
    {
        status = USB_Max3353ReadByte(index, &byteRead);
        if (kStatus_USB_Success != status)
        {
             return kStatus_USB_Error;
        }
        id <<= 8;
        id |= byteRead;
    }
    if (id != BOARD_MAX3353_MID)
    {
        return kStatus_USB_Error;
    }
    id = 0;
    for (index = 4; index < 8; ++index)
    {
        status = USB_Max3353ReadByte(index, &byteRead);
        if (kStatus_USB_Success != status)
        {
             return kStatus_USB_Error;
        }
        id <<= 8;
        id |= byteRead;
    }
    if (id != BOARD_MAX3353_PID)
    {
        return kStatus_USB_Error;
    }

    s_Max3353RunParameter.control1Register = 0x00U; /* reset value */
    while (USB_Max3353WriteByte(MAX3353_CONTROL1_ADDRESS, s_Max3353RunParameter.control1Register) !=
           kStatus_USB_Success)
    {
    }
    s_Max3353RunParameter.control2Register = 0x01U; /* reset value */
    while (USB_Max3353WriteByte(MAX3353_CONTROL2_ADDRESS, s_Max3353RunParameter.control2Register) !=
           kStatus_USB_Success)
    {
    }

    /* initialize port interrupt */
    PORT_SetPinInterruptConfig(BOARD_MAX3353_GPIO_PORT, BOARD_MAX3353_GPIO_PIN, kPORT_InterruptFallingEdge);
    BOARD_MAX3353_GPIO->PDDR &= ~(1U << BOARD_MAX3353_GPIO_PIN); /* gpio set as input */

    return kStatus_USB_Success;
}

usb_status_t USB_Max3353Deinit(void)
{
    USB_OtgPeripheralDisable();
    USB_I2CMasterDeinit(BOARD_MAX3353_I2C);
    return kStatus_USB_Success;
}

usb_status_t USB_OtgMax3353Update(void)
{
    uint8_t status = 0;
    uint8_t change = 0;
    uint8_t tmp = 0;
    uint8_t interruptLatch;

    /* clear interrupts */
    while (USB_Max3353ReadByte(MAX3353_INTERRUPT_LATCH_ADDRESS, &interruptLatch) != kStatus_USB_Success)
    {
    }
    do
    {
        /* get status */
        while (USB_Max3353ReadByte(MAX3353_STATUS_ADDRESS, &status) != kStatus_USB_Success)
        {
        }
        /* get edge */
        while (USB_Max3353ReadByte(MAX3353_INTERRUPT_EDGE_ADDRESS, &tmp) != kStatus_USB_Success)
        {
        }
        if (tmp != (uint8_t)((uint8_t)(~status) & 0x03))
        {
            /* change vbus and session interrupt edge */
            while (USB_Max3353WriteByte(MAX3353_INTERRUPT_EDGE_ADDRESS, (uint8_t)((uint8_t)(~status) & 0x03)) !=
                   kStatus_USB_Success)
            {
            }

            /* get edge */
            while (USB_Max3353ReadByte(MAX3353_INTERRUPT_EDGE_ADDRESS, &tmp) != kStatus_USB_Success)
            {
            }
        }

        change = s_Max3353RunParameter.statusRegister ^ status;
        s_Max3353RunParameter.statusRegister = status;

        if (change != 0)
        {
            if (change & MAX3353_STATUS_VBUS_VALID_MASK)
            {
                if (status & MAX3353_STATUS_VBUS_VALID_MASK)
                {
                    s_Max3353RunParameter.peripheralStatus |= kPeripheral_StatusVbusVld;
                }
                else
                {
                    s_Max3353RunParameter.peripheralStatus &= (uint8_t)(~kPeripheral_StatusVbusVld);
                }
            }

            if (change & MAX3353_STATUS_SESSION_END_MASK)
            {
                if (status & MAX3353_STATUS_SESSION_END_MASK)
                {
                    s_Max3353RunParameter.peripheralStatus &= (uint8_t)(~kPeripheral_StatusSessVld);
                }
            }

            if (change & MAX3353_STATUS_SESSION_VALID_MASK)
            {
                if (status & MAX3353_STATUS_SESSION_VALID_MASK)
                {
                    s_Max3353RunParameter.peripheralStatus |= kPeripheral_StatusSessVld;
                }
                else
                {
                    s_Max3353RunParameter.peripheralStatus &= (uint8_t)(~kPeripheral_StatusSessVld);
                }
            }

            if ((change & MAX3353_STATUS_ID_GND_MASK) || (change & MAX3353_STATUS_ID_FLOAT_MASK))
            {
                if (status & MAX3353_STATUS_ID_GND_MASK)
                {
                    s_Max3353RunParameter.peripheralStatus &= (uint8_t)(~kPeripheral_StatusId); /* A-device */
                }
                else if (status & MAX3353_STATUS_ID_FLOAT_MASK)
                {
                    s_Max3353RunParameter.peripheralStatus |= kPeripheral_StatusId; /* B-device */
                }
                else
                {
                }
            }
            if ((change & MAX3353_STATUS_A_HNP_MASK) && (status & MAX3353_STATUS_A_HNP_MASK))
            {
                /* keep the pull-up */
                s_Max3353RunParameter.statusRegister &= ~(MAX3353_STATUS_A_HNP_MASK);
                /* enter a_peripheral */
                s_Max3353RunParameter.peripheralStatus |= kPeripheral_StatusHNPdetected;
            }
            if ((change & MAX3353_STATUS_B_HNP_MASK) && (status & MAX3353_STATUS_B_HNP_MASK))
            {
                s_Max3353RunParameter.statusRegister &= ~(MAX3353_STATUS_B_HNP_MASK);
                /* enter b_host */
                s_Max3353RunParameter.peripheralStatus |= kPeripheral_StatusHNPdetected;
            }
        }
        if (interruptLatch)
        {
            while (USB_Max3353ReadByte(MAX3353_INTERRUPT_LATCH_ADDRESS, &interruptLatch) != kStatus_USB_Success)
            {
            }
        }
    } while (interruptLatch);

    return kStatus_USB_Success;
}

usb_status_t USB_OtgPeripheralEnable(void)
{
    uint8_t byteRead;

    /* initialize max3353 */
    s_Max3353RunParameter.control1Register = 0x02U;
    USB_Max3353WriteByte(MAX3353_CONTROL1_ADDRESS, 0x02U); /* interrupt mode */
    /* detect vbus by rising edge */
    USB_Max3353WriteByte(MAX3353_INTERRUPT_EDGE_ADDRESS, 0x01U);
    USB_Max3353WriteByte(MAX3353_INTERRUPT_MASK_ADDRESS, 0x3FU); /* enable interrupts */
    s_Max3353RunParameter.control2Register = 0x00U;
    USB_Max3353WriteByte(MAX3353_CONTROL2_ADDRESS, 0x00U); /* operating mode */
    do
    {
        byteRead = USB_Max3353ReadByte(MAX3353_INTERRUPT_LATCH_ADDRESS, &byteRead);
    } while (byteRead != 0); /* clear interrupts */

    /* get status */
    s_Max3353RunParameter.statusRegister = 0x00U;
    while (USB_Max3353ReadByte(MAX3353_STATUS_ADDRESS, &s_Max3353RunParameter.statusRegister) != kStatus_USB_Success)
    {
    }
    if (s_Max3353RunParameter.statusRegister & MAX3353_STATUS_VBUS_VALID_MASK)
    {
        s_Max3353RunParameter.peripheralStatus |= kPeripheral_StatusVbusVld;
    }
    if (s_Max3353RunParameter.statusRegister & MAX3353_STATUS_SESSION_VALID_MASK)
    {
        s_Max3353RunParameter.peripheralStatus |= kPeripheral_StatusSessVld;
    }
    if (s_Max3353RunParameter.statusRegister & MAX3353_STATUS_ID_FLOAT_MASK)
    {
        s_Max3353RunParameter.peripheralStatus |= kPeripheral_StatusId; /* B-device */
    }
    else
    {
    }

    NVIC_SetPriority((IRQn_Type)BOARD_MAX3353_GPIO_IRQ, USB_MAX3353_GPIO_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)BOARD_MAX3353_GPIO_IRQ);

    return kStatus_USB_Success;
}

usb_status_t USB_OtgPeripheralDisable(void)
{
    uint8_t byteRead;

    DisableIRQ((IRQn_Type)BOARD_MAX3353_GPIO_IRQ);
    /* disable max3353 */
    s_Max3353RunParameter.control2Register = 0x01U;
    USB_Max3353WriteByte(MAX3353_CONTROL2_ADDRESS, 0x01U); /* shutdown mode */
    do
    {
        byteRead = USB_Max3353ReadByte(MAX3353_INTERRUPT_LATCH_ADDRESS, &byteRead);
    } while (byteRead != 0); /* clear interrupts */

    return kStatus_USB_Success;
}

usb_status_t USB_OtgPeripheralGetStatus(uint32_t statusType, uint32_t *statusValue)
{
    switch (statusType)
    {
        case kPeripheral_StatusId:
            /* get status */
            if (s_Max3353RunParameter.peripheralStatus & kPeripheral_StatusId)
            {
                *statusValue = 1; /* B-device */
            }
            else
            {
                *statusValue = 0; /* A-device */
            }
            break;

        case kPeripheral_StatusSessVld:
            if (s_Max3353RunParameter.peripheralStatus & kPeripheral_StatusSessVld)
            {
                *statusValue = 1; /* valid */
            }
            else
            {
                *statusValue = 0; /* invalid */
            }
            break;

        case kPeripheral_StatusVbusVld:
            if (s_Max3353RunParameter.peripheralStatus & kPeripheral_StatusVbusVld)
            {
                *statusValue = 1; /* valid */
            }
            else
            {
                *statusValue = 0; /* invalid */
            }
            break;

        case kPeripheral_StatusHNPdetected:
            if (s_Max3353RunParameter.peripheralStatus & kPeripheral_StatusHNPdetected)
            {
                *statusValue = 1; /* valid */
            }
            else
            {
                *statusValue = 0; /* invalid */
            }
            break;

        case kPeripheral_StatusAll:
            *statusValue = s_Max3353RunParameter.peripheralStatus;
            break;

        default:
            break;
    }

    return kStatus_USB_Success;
}

usb_status_t USB_OtgPeripheralControl(usb_otg_controller_handle controllerHandle,
                                      uint32_t controlType,
                                      uint32_t controlValue1,
                                      uint32_t controlValue2)
{
    switch (controlType)
    {
        case kPeripheral_ControlUpdateStatus:
            USB_OtgMax3353Update();
            break;

        case kPeripheral_ControlVbus:
            if (controlValue1)
            {
                s_Max3353RunParameter.control2Register |= MAX3353_CONTROL2_VBUS_DRV_MASK;
                s_Max3353RunParameter.control2Register &= (~MAX3353_CONTROL2_VBUS_DISCHG_MASK);
            }
            else
            {
                s_Max3353RunParameter.control2Register &= (~MAX3353_CONTROL2_VBUS_DRV_MASK);
                s_Max3353RunParameter.control2Register |= MAX3353_CONTROL2_VBUS_DISCHG_MASK;
            }
            while (USB_Max3353WriteByte(MAX3353_CONTROL2_ADDRESS, s_Max3353RunParameter.control2Register) !=
                   kStatus_USB_Success)
            {
            }
            break;

        case kPeripheral_ControlHNPCheckEnable:
            if (controlValue1)
            {
                /* clear HNP detected flag */
                s_Max3353RunParameter.peripheralStatus &= (uint8_t)(~kPeripheral_StatusHNPdetected);
                s_Max3353RunParameter.control1Register |= MAX3353_CONTROL1_BDISC_ACONN_MASK;
                while (USB_Max3353WriteByte(MAX3353_CONTROL1_ADDRESS, s_Max3353RunParameter.control1Register) !=
                       kStatus_USB_Success)
                {
                }
            }
            else
            {
                /* clear HNP detected flag */
                s_Max3353RunParameter.peripheralStatus &= (uint8_t)(~kPeripheral_StatusHNPdetected);
                if (s_Max3353RunParameter.control1Register & MAX3353_CONTROL1_BDISC_ACONN_MASK)
                {
                    s_Max3353RunParameter.control1Register &= (~MAX3353_CONTROL1_BDISC_ACONN_MASK);
                    while (USB_Max3353WriteByte(MAX3353_CONTROL1_ADDRESS, s_Max3353RunParameter.control1Register) !=
                           kStatus_USB_Success)
                    {
                    }
                    USB_OtgNotifyChange(g_OtgHandle, kOtg_StatusChange, 1);
                }
            }
            break;

        default:
            break;
    }

    return kStatus_USB_Success;
}

#endif
