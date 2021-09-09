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

/*!
 * @brief device joystick initialization function.
 *
 * This function initializes the device joystick with the composite device class information.
 *
 * @param deviceComposite          The pointer to the composite device structure.
 * @return kStatus_USB_Success .
 */
extern usb_status_t USB_DeviceHidJoystickInit(usb_device_composite_struct_t *deviceComposite);

/*!
 * @brief device Joystick callback function.
 *
 * This function handles the Hid class specific event.
 * @param handle          The USB class  handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the class specific event.
 * @return kStatus_USB_Success or error.
 */
extern usb_status_t USB_DeviceHidJoystickCallback(class_handle_t handle, uint32_t event, void *param);

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
extern usb_status_t USB_DeviceHidJoystickSetConfigure(class_handle_t handle, uint8_t configure);

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
extern usb_status_t USB_DeviceHidJoystickSetInterface(class_handle_t handle,
                                                      uint8_t interface,
                                                      uint8_t alternateSetting);

#if defined(__cplusplus)
extern "C" {
#endif
#endif
#endif /* __USB_DEVICE_HID_JOYSTICK_H__ */
