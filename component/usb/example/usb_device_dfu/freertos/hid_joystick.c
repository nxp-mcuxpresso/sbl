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

#include "usb_device_class.h"
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
    return USB_DeviceHidSend(s_UsbDeviceComposite->hidJoystickHandle, USB_HID_JOYSTICK_ENDPOINT_IN,
                             s_UsbDeviceHidJoystick.buffer, USB_HID_JOYSTICK_REPORT_LENGTH);
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
usb_status_t USB_DeviceHidJoystickCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;

    switch (event)
    {
        case kUSB_DeviceHidEventSendResponse:
            if (s_UsbDeviceComposite->attach)
            {
                return USB_DeviceHidJoystickAction();
            }
            break;
        case kUSB_DeviceHidEventGetReport:
        case kUSB_DeviceHidEventSetReport:
        case kUSB_DeviceHidEventRequestReportBuffer:
            error = kStatus_USB_InvalidRequest;
            break;
        case kUSB_DeviceHidEventGetIdle:
        case kUSB_DeviceHidEventGetProtocol:
        case kUSB_DeviceHidEventSetIdle:
        case kUSB_DeviceHidEventSetProtocol:
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
usb_status_t USB_DeviceHidJoystickSetConfigure(class_handle_t handle, uint8_t configure)
{
    if (USB_COMPOSITE_CONFIGURE_INDEX == configure)
    {
        return USB_DeviceHidJoystickAction();
    }
    return kStatus_USB_Success;
}

/*!
 * @brief device joystick set interface function.
 *
 * This function alternates joystick interface.
 *
 * @param handle The Hid class handle.
 * @param alternateSetting Hid class alternateSetting.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceHidJoystickSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting)
{
    if (USB_HID_JOYSTICK_INTERFACE_INDEX == interface)
    {
        return USB_DeviceHidJoystickAction(); /* run the Joystick action code */
    }
    return kStatus_USB_Success;
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
