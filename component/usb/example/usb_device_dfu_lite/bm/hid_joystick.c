/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
 * o Neither the name the copyright holder nor the names of its
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

#include "hid_joystick.h"
#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static usb_status_t USB_DeviceHidJoystickAction(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
static usb_device_composite_struct_t *s_UsbDeviceComposite;
static usb_device_hid_joystick_struct_t s_UsbDeviceHidJoystick;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief device joystick action.
 *
 * This function handles the joystick action like cursor moving, throttle status,
 * button pressing.
 */
static usb_status_t USB_DeviceHidJoystickAction(void)
{
    static int8_t x = 0xC0U;
    static int8_t y = 0xC0U;
    static uint8_t throttle = 0U;
#if (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U)
    static uint8_t delayCount = 0U;
    static uint8_t activeButton = 0U; /* button 1 */
#endif
    enum
    {
        RIGHT,
        DOWN,
        LEFT,
        UP
    };
    static uint8_t dir = RIGHT;
    /* update throttle value */
    throttle++;
    if (throttle == 0xFF)
    {
        throttle = 0U;
    }
    s_UsbDeviceHidJoystick.buffer[0] = throttle;

    switch (dir)
    {
        case RIGHT:
            /* Move right. Increase X value. */
            s_UsbDeviceHidJoystick.buffer[1] = x;
            s_UsbDeviceHidJoystick.buffer[2] = 0U;
            x++;
            if (x > 0x40U)
            {
                dir++;
            }
            break;
        case DOWN:
            /* Move down. Increase Y value. */
            s_UsbDeviceHidJoystick.buffer[1] = 0U;
            s_UsbDeviceHidJoystick.buffer[2] = y;
            y++;
            if (y > 0x40U)
            {
                dir++;
            }
            break;
        case LEFT:
            /* Move left. Decrease X value. */
            s_UsbDeviceHidJoystick.buffer[1] = x;
            s_UsbDeviceHidJoystick.buffer[2] = 0U;
            x--;
            if (x < 0xC0U)
            {
                dir++;
            }
            break;
        case UP:
            /* Move up. Decrease Y value. */
            s_UsbDeviceHidJoystick.buffer[1] = 0U;
            s_UsbDeviceHidJoystick.buffer[2] = y;
            y--;
            if (y < 0xC0U)
            {
                dir = RIGHT;
            }
            break;
        default:
            break;
    }

#if (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U)
    delayCount++;
    if (delayCount == 200U)
    {
        delayCount = 0U;
        /* toggle the button */
        activeButton = ~activeButton;
        if (activeButton)
        {
            s_UsbDeviceHidJoystick.buffer[3] = 0x01;
        }
        else
        {
            s_UsbDeviceHidJoystick.buffer[3] = 0x00;
        }
    }
#endif
    return USB_DeviceSendRequest(s_UsbDeviceComposite->deviceHandle, USB_HID_JOYSTICK_ENDPOINT_IN,
                             s_UsbDeviceHidJoystick.buffer, USB_HID_JOYSTICK_REPORT_LENGTH);
}
static usb_status_t USB_DeviceHidJoystickInterruptIn(usb_device_handle deviceHandle,
                                                  usb_device_endpoint_callback_message_struct_t *event,
                                                  void *arg)
{
    if (s_UsbDeviceComposite->attach)
    {
        return USB_DeviceHidJoystickAction();
    }

    return kStatus_USB_Error;
}
/*!
 * @brief device Joystick callback function.
 *
 * This function handles the Hid class specific event.
 * @param handle          The USB class  handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the class specific event.
 * @return kStatus_USB_Success or error.
 */
usb_status_t USB_DeviceHidJoystickClassRequest(usb_device_handle handle,
                                            usb_setup_struct_t *setup,
                                            uint8_t **buffer,
                                            uint32_t *length)
{
    usb_status_t error = kStatus_USB_Error;

    if (setup->wIndex != USB_HID_JOYSTICK_INTERFACE_INDEX)
    {
        return error;
    }

    switch (setup->bRequest)
    {
        case USB_DEVICE_HID_REQUEST_GET_REPORT:
        case USB_DEVICE_HID_REQUEST_SET_REPORT:
             error = kStatus_USB_InvalidRequest;
             break;
        case USB_DEVICE_HID_REQUEST_GET_IDLE:
        case USB_DEVICE_HID_REQUEST_GET_PROTOCOL:
        case USB_DEVICE_HID_REQUEST_SET_IDLE:
        case USB_DEVICE_HID_REQUEST_SET_PROTOCOL:
            break;
        default:
            break;
    }
    return error;
}


/*!
 * @brief device joystick  set configuration function.
 *
 * This function sets configuration for HID joystick interface.
 *
 * @param handle The Hid class handle.
 * @param configure Hid class configured index.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceHidJoystickSetConfigure(usb_device_handle handle, uint8_t configure)
{
    usb_device_endpoint_init_struct_t epInitStruct;
    usb_device_endpoint_callback_struct_t epCallback;
    if (USB_COMPOSITE_CONFIGURE_INDEX == configure)
    {
        epCallback.callbackFn = USB_DeviceHidJoystickInterruptIn;
        epCallback.callbackParam = handle;

        epInitStruct.zlt = 0U;
        epInitStruct.transferType = USB_ENDPOINT_INTERRUPT;
        epInitStruct.endpointAddress =
            USB_HID_JOYSTICK_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
        if (USB_SPEED_HIGH == s_UsbDeviceComposite->speed)
        {
            epInitStruct.maxPacketSize = HS_HID_JOYSTICK_INTERRUPT_IN_PACKET_SIZE;
        }
        else
        {
            epInitStruct.maxPacketSize = FS_HID_JOYSTICK_INTERRUPT_IN_PACKET_SIZE;
        }

        USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);
        return USB_DeviceHidJoystickAction();
    }
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceHidJoystickEndpointUnstall(usb_device_handle handle, uint8_t ep)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    USB_DeviceCancel(handle, ep);
    error = USB_DeviceUnstallEndpoint(handle, ep);
    USB_DeviceHidJoystickAction(); 
    return error;
}
usb_status_t USB_DeviceHidJoystickEndpointStall(usb_device_handle handle, uint8_t ep)
{
    USB_DeviceCancel(handle, ep);
    return USB_DeviceStallEndpoint(handle, ep);
}

/*!
 * @brief device joystick initialization function.
 *
 * This function initializes the device joystick with the composite device class information.
 *
 * @param deviceComposite          The pointer to the composite device structure.
 * @return kStatus_USB_Success .
 */
usb_status_t USB_DeviceHidJoystickInit(usb_device_composite_struct_t *deviceComposite)
{
    s_UsbDeviceComposite = deviceComposite;
    return kStatus_USB_Success;
}
#endif
