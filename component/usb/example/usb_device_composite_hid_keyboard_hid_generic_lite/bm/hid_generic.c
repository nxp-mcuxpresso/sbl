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

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_hid.h"

#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "composite.h"

#include "hid_generic.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static usb_status_t USB_DeviceHidGenericInterruptIn(usb_device_handle handle,
                                                    usb_device_endpoint_callback_message_struct_t *message,
                                                    void *callbackParam);
static usb_status_t USB_DeviceHidGenericInterruptOut(usb_device_handle handle,
                                                     usb_device_endpoint_callback_message_struct_t *message,
                                                     void *callbackParam);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static usb_device_composite_struct_t *s_UsbDeviceComposite;
static usb_device_hid_generic_struct_t s_UsbDeviceHidGeneric;

/*******************************************************************************
 * Code
 ******************************************************************************/

static usb_status_t USB_DeviceHidGenericInterruptIn(usb_device_handle handle,
                                                    usb_device_endpoint_callback_message_struct_t *message,
                                                    void *callbackParam)
{
    if (s_UsbDeviceComposite->attach)
    {
    }

    return kStatus_USB_Error;
}

static usb_status_t USB_DeviceHidGenericInterruptOut(usb_device_handle handle,
                                                     usb_device_endpoint_callback_message_struct_t *message,
                                                     void *callbackParam)
{
    if (s_UsbDeviceComposite->attach)
    {
        USB_DeviceSendRequest(s_UsbDeviceComposite->deviceHandle, USB_HID_GENERIC_ENDPOINT_IN,
                              (uint8_t *)s_UsbDeviceHidGeneric.buffer[s_UsbDeviceHidGeneric.bufferIndex],
                              USB_HID_GENERIC_OUT_BUFFER_LENGTH);
        s_UsbDeviceHidGeneric.bufferIndex ^= 1U;
        return USB_DeviceRecvRequest(s_UsbDeviceComposite->deviceHandle, USB_HID_GENERIC_ENDPOINT_OUT,
                                     (uint8_t *)s_UsbDeviceHidGeneric.buffer[s_UsbDeviceHidGeneric.bufferIndex],
                                     USB_HID_GENERIC_OUT_BUFFER_LENGTH);
    }

    return kStatus_USB_Error;
}

usb_status_t USB_DeviceHidGenericClassRequest(usb_device_handle handle,
                                              usb_setup_struct_t *setup,
                                              uint8_t **buffer,
                                              uint32_t *length)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    if (setup->wIndex != USB_HID_GENERIC_INTERFACE_INDEX)
    {
        return error;
    }

    switch (setup->bRequest)
    {
        case USB_DEVICE_HID_REQUEST_GET_REPORT:
            break;
        case USB_DEVICE_HID_REQUEST_GET_IDLE:
            break;
        case USB_DEVICE_HID_REQUEST_GET_PROTOCOL:
            break;
        case USB_DEVICE_HID_REQUEST_SET_REPORT:
            break;
        case USB_DEVICE_HID_REQUEST_SET_IDLE:
            error = kStatus_USB_Success;
            s_UsbDeviceHidGeneric.idleRate = 125U;
            break;
        case USB_DEVICE_HID_REQUEST_SET_PROTOCOL:
            break;
    }

    return error;
}

usb_status_t USB_DeviceHidGenericSetConfigure(usb_device_handle handle, uint8_t configure)
{
    usb_device_endpoint_init_struct_t epInitStruct;
    usb_device_endpoint_callback_struct_t epCallback;

    if (USB_COMPOSITE_CONFIGURE_INDEX == configure)
    {
        epCallback.callbackFn = USB_DeviceHidGenericInterruptIn;
        epCallback.callbackParam = handle;

        epInitStruct.zlt = 0U;
        epInitStruct.transferType = USB_ENDPOINT_INTERRUPT;
        epInitStruct.endpointAddress =
            USB_HID_GENERIC_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        if (USB_SPEED_HIGH == s_UsbDeviceComposite->speed)
        {
            epInitStruct.maxPacketSize = HS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE;
        }
        else
        {
            epInitStruct.maxPacketSize = FS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE;
        }

        USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

        epCallback.callbackFn = USB_DeviceHidGenericInterruptOut;
        epCallback.callbackParam = handle;

        epInitStruct.zlt = 0U;
        epInitStruct.transferType = USB_ENDPOINT_INTERRUPT;
        epInitStruct.endpointAddress =
            USB_HID_GENERIC_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        if (USB_SPEED_HIGH == s_UsbDeviceComposite->speed)
        {
            epInitStruct.maxPacketSize = HS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE;
        }
        else
        {
            epInitStruct.maxPacketSize = FS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE;
        }

        USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);

        return USB_DeviceRecvRequest(s_UsbDeviceComposite->deviceHandle, USB_HID_GENERIC_ENDPOINT_OUT,
                                     (uint8_t *)s_UsbDeviceHidGeneric.buffer[s_UsbDeviceHidGeneric.bufferIndex],
                                     USB_HID_GENERIC_OUT_BUFFER_LENGTH);
    }
    return kStatus_USB_Error;
}

usb_status_t USB_DeviceHidGenericInit(usb_device_composite_struct_t *deviceComposite)
{
    s_UsbDeviceComposite = deviceComposite;
    s_UsbDeviceHidGeneric.bufferIndex = 0U;
    s_UsbDeviceHidGeneric.idleRate = 125U;
    return kStatus_USB_Success;
}
