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

#ifndef __USB_DEVICE_HID_JOYSTICK_H__
#define __USB_DEVICE_HID_JOYSTICK_H__

#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct _usb_device_hid_joystick_struct
{
    uint8_t buffer[USB_HID_JOYSTICK_REPORT_LENGTH];
    uint8_t idleRate;
} usb_device_hid_joystick_struct_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

extern usb_status_t USB_DeviceHidJoystickSetConfigure(usb_device_handle handle, uint8_t configure);
extern usb_status_t USB_DeviceHidJoystickClassRequest(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint8_t **buffer,
                                                   uint32_t *length);
extern usb_status_t USB_DeviceHidJoystickInit(usb_device_composite_struct_t *deviceComposite);
extern usb_status_t USB_DeviceHidJoystickEndpointUnstall(usb_device_handle handle, uint8_t ep);
extern usb_status_t USB_DeviceHidJoystickEndpointStall(usb_device_handle handle, uint8_t ep);
#if defined(__cplusplus)
extern "C" {
#endif
#endif
#endif /* __USB_DEVICE_HID_JOYSTICK_H__ */
