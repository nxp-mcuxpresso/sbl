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

#ifndef _USB_IO_H_
#define _USB_IO_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef void (*usb_gpio_callback_t)(void);

typedef enum _usb_gpio_interrupt_trrigger
{
    kUSB_GpioInterruptLogicZero = 0x1U,   /*!< Interrupt when logic zero. */
    kUSB_GpioInterruptRisingEdge = 0x2U,  /*!< Interrupt on rising edge. */
    kUSB_GpioInterruptFallingEdge = 0x3U, /*!< Interrupt on falling edge. */
    kUSB_GpioInterruptEitherEdge = 0x4U,  /*!< Interrupt on either edge. */
    kUSB_GpioInterruptLogicOne = 0x5U,    /*!< Interrupt when logic one. */
} usb_gpio_interrupt_trigger_t;

/*******************************************************************************
 * API
 ******************************************************************************/
void USB_GpioInit(uint8_t instance, uint8_t port, uint32_t pinMask, usb_gpio_callback_t callback);
void USB_GpioInt(uint8_t instance, uint8_t enable);
void USB_GpioInterruptInit(
    uint8_t instance, uint8_t port, uint32_t pin, uint8_t trrigger, usb_gpio_callback_t callback);
void USB_GpioInterruptEnable(uint8_t instance, uint8_t port, uint32_t pin, uint8_t enable);
void USB_GpioOutputInit(uint8_t instance, uint32_t port, uint32_t pin);
void USB_GpioOutputWritePin(uint8_t instance, uint32_t port, uint32_t pin, uint8_t output);
void USB_GpioInputInit(uint8_t instance, uint32_t port, uint32_t pin);
uint8_t USB_GpioInputReadPin(uint8_t instance, uint32_t port, uint32_t pin);

#endif /* _USB_IO_H_ */
