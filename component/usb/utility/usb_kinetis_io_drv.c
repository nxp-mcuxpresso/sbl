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

#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "usb_io.h"

void USB_GpioInterruptInit(uint8_t instance, uint8_t port, uint32_t pin, uint8_t trrigger, usb_gpio_callback_t callback)
{
    uint8_t trig;
    PORT_Type *portList[] = PORT_BASE_PTRS;
    GPIO_Type *gpioList[] = GPIO_BASE_PTRS;

    switch (trrigger)
    {
        case kUSB_GpioInterruptLogicZero:
            trig = kPORT_InterruptLogicZero;
            break;
        case kUSB_GpioInterruptLogicOne:
            trig = kPORT_InterruptLogicOne;
            break;

        case kUSB_GpioInterruptRisingEdge:
            trig = kPORT_InterruptRisingEdge;
            break;

        case kUSB_GpioInterruptFallingEdge:
            trig = kPORT_InterruptFallingEdge;
            break;

        default:
            trig = kPORT_InterruptEitherEdge;
            break;
    }
    /* initialize port interrupt */
    PORT_SetPinInterruptConfig(portList[instance], pin, (port_interrupt_t)trig);
    gpioList[instance]->PDDR &= ~(1U << pin); /* gpio set as input */
}

void USB_GpioInterruptEnable(uint8_t instance, uint8_t port, uint32_t pin, uint8_t enable)
{
}

void USB_GpioOutputInit(uint8_t instance, uint32_t port, uint32_t pin)
{
    GPIO_Type *instanceList[] = GPIO_BASE_PTRS;
    gpio_pin_config_t pin_config = {
        kGPIO_DigitalOutput, 0,
    };

    GPIO_PinInit(instanceList[instance], pin, &pin_config);
}

void USB_GpioOutputWritePin(uint8_t instance, uint32_t port, uint32_t pin, uint8_t output)
{
    GPIO_Type *instanceList[] = GPIO_BASE_PTRS;

    GPIO_PinWrite(instanceList[instance], pin, output);
}

void USB_GpioInputInit(uint8_t instance, uint32_t port, uint32_t pin)
{
    GPIO_Type *instanceList[] = GPIO_BASE_PTRS;
    gpio_pin_config_t pin_config = {
        kGPIO_DigitalInput, 0,
    };

    GPIO_PinInit(instanceList[instance], pin, &pin_config);
}

uint8_t USB_GpioInputReadPin(uint8_t instance, uint32_t port, uint32_t pin)
{
    GPIO_Type *instanceList[] = GPIO_BASE_PTRS;
    return GPIO_PinRead(instanceList[instance], pin);
}
