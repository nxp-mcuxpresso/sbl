/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#ifndef __USB_CCID_SMART_CARD_H__
#define __USB_CCID_SMART_CARD_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
#define CONTROLLER_ID kUSB_ControllerEhci0
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
#define CONTROLLER_ID kUSB_ControllerKhci0
#endif

#if defined(__GIC_PRIO_BITS)
#define USB_DEVICE_INTERRUPT_PRIORITY (25U)
#else
#define USB_DEVICE_INTERRUPT_PRIORITY (3U)
#endif

#define USB_DEVICE_CCID_BUFFER_4BYTE_ALIGN(n) (((n - 1U) & 0xFFFFFFFCU) + 0x00000004U)

#define USB_DEVICE_CCID_ATR_BUFFER_LENGTH USB_DEVICE_CCID_BUFFER_4BYTE_ALIGN(USB_DEVICE_CONFIG_CCID_MAX_MESSAGE_LENGTH)
#define USB_DEVICE_CCID_COMMAND_BUFFER_LENGTH \
    USB_DEVICE_CCID_BUFFER_4BYTE_ALIGN(USB_DEVICE_CONFIG_CCID_MAX_MESSAGE_LENGTH)
#define USB_DEVICE_CCID_RESPONSE_BUFFER_LENGTH \
    USB_DEVICE_CCID_BUFFER_4BYTE_ALIGN(USB_DEVICE_CONFIG_CCID_MAX_MESSAGE_LENGTH)

#define USB_DEVICE_CCID_PARAMETER_BUFFER_LENGTH \
    USB_DEVICE_CCID_BUFFER_4BYTE_ALIGN(sizeof(usb_device_ccid_set_parameters_command_common_t))

typedef struct _usb_ccid_smart_card_struct
{
    usb_device_handle deviceHandle;
    uint32_t clockFrequency;
    uint32_t dataRate;
    uint8_t slotsAtrBuffer[USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS][USB_DEVICE_CCID_ATR_BUFFER_LENGTH];
    uint8_t slotsCommandBuffer[USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS + 1U][USB_DEVICE_CCID_COMMAND_BUFFER_LENGTH];
    uint8_t slotsResponseBuffer[USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS + 1U][USB_DEVICE_CCID_RESPONSE_BUFFER_LENGTH];
    uint8_t slotsCurrentParameter[USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS][USB_DEVICE_CCID_PARAMETER_BUFFER_LENGTH];
    uint8_t slotsChangeBuffer[(USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS * 2 - 1U) / 8 + 1U + 1U];
    uint8_t slotsChanged;
    uint8_t clockStatus;
    uint8_t protocol;
    uint8_t nodeAddress;
    uint8_t remoteWakeup;
    uint8_t suspend;
    uint8_t speed;
    uint8_t attach;
} usb_ccid_smart_card_struct_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#endif /* __USB_CCID_SMART_CARD_H__ */
