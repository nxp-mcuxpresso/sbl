/*
 * Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
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

#ifndef _API_USB_KSDK_H_
#define _API_USB_KSDK_H_

#include "bootloader_common.h"
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_hid.h"
#include "usb_device_msc.h"
#include "usb_device_dfu.h"
#include "usb_device_ch9.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Interface for the usb driver.
typedef struct USBDriverInterface
{
    usb_status_t (*USB_DeviceClassInit)(uint8_t controllerId,
                                        usb_device_class_config_list_struct_t *configList,
                                        usb_device_handle *handle);
    usb_status_t (*USB_DeviceClassDeinit)(uint8_t controllerId);
    usb_status_t (*USB_DeviceClassGetSpeed)(uint8_t controllerId, uint8_t *speed);
    usb_status_t (*USB_DeviceInit)(uint8_t controllerId,
                                   usb_device_callback_t deviceCallback,
                                   usb_device_handle *handle);
    usb_status_t (*USB_DeviceDeinit)(usb_device_handle handle);
    usb_status_t (*USB_DeviceRun)(usb_device_handle handle);
    usb_status_t (*USB_DeviceStop)(usb_device_handle handle);
    usb_status_t (*USB_DeviceSendRequest)(usb_device_handle handle,
                                          uint8_t endpointAddress,
                                          uint8_t *buffer,
                                          uint32_t length);
    usb_status_t (*USB_DeviceRecvRequest)(usb_device_handle handle,
                                          uint8_t endpointAddress,
                                          uint8_t *buffer,
                                          uint32_t length);
    usb_status_t (*USB_DeviceCancel)(usb_device_handle handle, uint8_t endpointAddress);
    usb_status_t (*USB_DeviceInitEndpoint)(usb_device_handle handle,
                                           usb_device_endpoint_init_struct_t *epInit,
                                           usb_device_endpoint_callback_struct_t *endpointCallback);

    usb_status_t (*USB_DeviceDeinitEndpoint)(usb_device_handle handle, uint8_t endpointAddress);

    usb_status_t (*USB_DeviceStallEndpoint)(usb_device_handle handle, uint8_t endpointAddress);

    usb_status_t (*USB_DeviceUnstallEndpoint)(usb_device_handle handle, uint8_t endpointAddress);
    usb_status_t (*USB_DeviceGetStatus)(usb_device_handle handle, usb_device_status_t type, void *param);
    usb_status_t (*USB_DeviceSetStatus)(usb_device_handle handle, usb_device_status_t type, void *param);
    void (*USB_DeviceIsrFunction)(void *deviceHandle);
    void (*USB_DeviceGetVersion)(uint32_t *version);
    usb_status_t (*USB_DeviceHidSend)(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);
    usb_status_t (*USB_DeviceHidRecv)(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);
} usb_driver_interface_t;

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

extern const usb_driver_interface_t g_usbDriverInterface;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

#if defined(__cplusplus)
}
#endif

/*!
 *@}
*/

#endif /* _API_USB_KSDK_H_ */
