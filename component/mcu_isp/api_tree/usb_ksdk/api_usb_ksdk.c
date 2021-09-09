/*
 * Copyright 2017 -2018 NXP
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

#include "usb_ksdk/api_usb_ksdk.h"
#include "bootloader_config.h"
/*******************************************************************************
* Variables
 ******************************************************************************/
#if BL_CONFIG_HS_USB_HID
//! @brief Function table for USB driver.
const usb_driver_interface_t g_usbDriverInterface = {
    .USB_DeviceClassInit = USB_DeviceClassInit,
    .USB_DeviceInit = USB_DeviceInit,
    .USB_DeviceRun = USB_DeviceRun,
    .USB_DeviceStop = USB_DeviceStop,
    .USB_DeviceDeinit = USB_DeviceDeinit,
    .USB_DeviceSendRequest = USB_DeviceSendRequest,
    .USB_DeviceRecvRequest = USB_DeviceRecvRequest,
    .USB_DeviceCancel = USB_DeviceCancel,
    .USB_DeviceInitEndpoint = USB_DeviceInitEndpoint,
    .USB_DeviceDeinitEndpoint = USB_DeviceDeinitEndpoint,
    .USB_DeviceStallEndpoint = USB_DeviceStallEndpoint,
    .USB_DeviceUnstallEndpoint = USB_DeviceUnstallEndpoint,
    .USB_DeviceGetStatus = USB_DeviceGetStatus,
    .USB_DeviceSetStatus = USB_DeviceSetStatus,
#if USB_DEVICE_CONFIG_KHCI
    .USB_DeviceIsrFunction = USB_DeviceKhciIsrFunction,
#elif USB_DEVICE_CONFIG_EHCI
    .USB_DeviceIsrFunction = USB_DeviceEhciIsrFunction,
#elif USB_DEVICE_CONFIG_LPCIP3511FS || USB_DEVICE_CONFIG_LPCIP3511HS
    .USB_DeviceIsrFunction = USB_DeviceLpcIp3511IsrFunction,
#endif
    .USB_DeviceGetVersion = USB_DeviceGetVersion,
    .USB_DeviceHidSend = USB_DeviceHidSend,
    .USB_DeviceHidRecv = USB_DeviceHidRecv,
};
#endif
