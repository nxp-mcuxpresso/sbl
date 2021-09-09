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

#ifndef __USB_DEVICE_DESCRIPTOR_H__
#define __USB_DEVICE_DESCRIPTOR_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 (1)
#define USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 (0)
#define USB_DEVICE_SPECIFIC_BCD_VERSION (0x0200U)
#define USB_DEVICE_SPECIFIC_BCD_VERSION_DFU (0x0100U)
#define USB_DEVICE_DEMO_BCD_VERSION (0x0101U)

#define USB_DEVICE_CLASS (0x00U)
#define USB_DEVICE_SUBCLASS (0x00U)
#define USB_DEVICE_PROTOCOL (0x00U)

#define USB_DEVICE_MAX_POWER (0x32U)
#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL (sizeof(g_UsbDeviceConfigurationDescriptor))
#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL_DFU (sizeof(g_UsbDeviceConfigurationDescriptorDfu))
#else
#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL (sizeof(g_UsbDeviceConfigurationDescriptorDfu))
#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL_DFU (sizeof(g_UsbDeviceConfigurationDescriptorDfu))
#endif
#define USB_DESCRIPTOR_LENGTH_FUNCTINAL (9U)
#define USB_DESCRIPTOR_LENGTH_STRING0 (sizeof(g_UsbDeviceString0))
#define USB_DESCRIPTOR_LENGTH_STRING1 (sizeof(g_UsbDeviceString1))
#define USB_DESCRIPTOR_LENGTH_STRING2 (sizeof(g_UsbDeviceString2))
#define USB_DESCRIPTOR_LENGTH_STRING3 (sizeof(g_UsbDeviceString3))
#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
#define USB_DESCRIPTOR_LENGTH_STRING4 (sizeof(g_UsbDeviceString4))
#endif

#define USB_DFU_INTERFACE_INDEX (0U)
#define USB_DESCRIPTOR_TYPE_DFU_FUNCTIONAL (0x21)
#define USB_DFU_DETACH_TIMEOUT (0x6400)
#define USB_DFU_BIT_WILL_DETACH (1U)
#define USB_DFU_BIT_MANIFESTATION_TOLERANT (0U)
#define USB_DFU_BIT_CAN_UPLOAD (0U)
#define USB_DFU_BIT_CAN_DNLOAD (1U)
#define MAX_TRANSFER_SIZE (0x200)

#define USB_DEVICE_CONFIGURATION_COUNT (1U)
#define USB_DFU_INTERFACE_COUNT (1U)

#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
#define USB_DEVICE_STRING_COUNT (5U)
#else
#define USB_DEVICE_STRING_COUNT (4U)
#endif

#define USB_DEVICE_LANGUAGE_COUNT (1U)

#define USB_COMPOSITE_CONFIGURE_INDEX (1U)
#define USB_DFU_CONFIGURE_INDEX (1U)
#define USB_DFU_CLASS (0xFEU)
#define USB_DFU_SUBCLASS (0x01U)
#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
#define USB_DFU_MODE_PROTOCOL (0x02U)
#define USB_DFU_PROTOCOL (0x01U)
#else
#define USB_DFU_PROTOCOL (0x01U)
#define USB_DFU_MODE_PROTOCOL (0x02U)
#endif

#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
#if (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0)
#define USB_DESCRIPTOR_LENGTH_HID_JOYSTICK_REPORT (sizeof(g_UsbDeviceHidJoystickReportDescriptor))
#else
#define USB_DESCRIPTOR_LENGTH_HID_JOYSTICK_REPORT (sizeof(g_UsbDeviceHidJoystickReportDescriptor))
#endif
#define USB_DESCRIPTOR_LENGTH_HID (9U)
#define USB_HID_JOYSTICK_INTERFACE_COUNT (1U)

#define USB_HID_JOYSTICK_IN_BUFFER_LENGTH (8U)
#define USB_HID_JOYSTICK_ENDPOINT_COUNT (1U)
#define USB_HID_JOYSTICK_INTERFACE_INDEX (1U)
#define USB_HID_JOYSTICK_ENDPOINT_IN (1U)

#if (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U)
#define USB_HID_JOYSTICK_REPORT_LENGTH (0x04U)
#else
#define USB_HID_JOYSTICK_REPORT_LENGTH (0x03U)
#endif

#define USB_HID_JOYSTICK_CLASS (0x03U)
#define USB_HID_JOYSTICK_SUBCLASS (0x00U)
#define USB_HID_JOYSTICK_PROTOCOL (0x00U)

#define HS_HID_JOYSTICK_INTERRUPT_IN_PACKET_SIZE (8U)
#define FS_HID_JOYSTICK_INTERRUPT_IN_PACKET_SIZE (8U)
#define HS_HID_JOYSTICK_INTERRUPT_IN_INTERVAL (0x06U) /* 2^(6-1) = 1ms */
#define FS_HID_JOYSTICK_INTERRUPT_IN_INTERVAL (0x04U)
#define USB_COMPOSITE_INTERFACE_COUNT (USB_DFU_INTERFACE_COUNT + USB_HID_JOYSTICK_INTERFACE_COUNT)
#else
#define USB_COMPOSITE_INTERFACE_COUNT (USB_DFU_INTERFACE_COUNT)
#endif

/*******************************************************************************
 * API
 ******************************************************************************/
/* Get device descriptor request */
extern usb_status_t USB_DeviceGetDeviceDescriptor(usb_device_handle handle,
                                                  usb_device_get_device_descriptor_struct_t *deviceDescriptor);

/* Get device configuration descriptor request */
extern usb_status_t USB_DeviceGetConfigurationDescriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configurationDescriptor);

/* Get device string descriptor request */
extern usb_status_t USB_DeviceGetStringDescriptor(usb_device_handle handle,
                                                  usb_device_get_string_descriptor_struct_t *stringDescriptor);

#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
/* Configure the device according to the USB speed. */
extern usb_status_t USB_DeviceSetSpeed(usb_device_handle handle, uint8_t speed);

/* Get hid descriptor request */
extern usb_status_t USB_DeviceGetHidDescriptor(usb_device_handle handle,
                                               usb_device_get_hid_descriptor_struct_t *hidDescriptor);

/* Get hid report descriptor request */
extern usb_status_t USB_DeviceGetHidReportDescriptor(
    usb_device_handle handle, usb_device_get_hid_report_descriptor_struct_t *hidReportDescriptor);

/* Get hid physical descriptor request */
extern usb_status_t USB_DeviceGetHidPhysicalDescriptor(
    usb_device_handle handle, usb_device_get_hid_physical_descriptor_struct_t *hidPhysicalDescriptor);
#endif

#endif /* __USB_DEVICE_DESCRIPTOR_H__ */
