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

#ifndef __HOST_PRINTER_H__
#define __HOST_PRINTER_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USB_HOST_PRINTER_APP_RECEIVE_TRY_DELAY (500)
#define USB_HOST_PRINTER_APP_ONEMS_COUNT (200)
#define USB_HOST_PRINTER_APP_BUFFER_SIZE (300)

/*! @brief host app device attach/detach status */
typedef enum _usb_host_printer_app_state
{
    kStatus_DEV_Idle = 0, /*!< there is no device attach/detach */
    kStatus_DEV_Attached, /*!< device is attached */
    kStatus_DEV_Detached, /*!< device is detached */
} usb_host_printer_app_state_t;

/*! @brief host app run status */
typedef enum _usb_host_printer_run_state
{
    kRunIdle = 0,            /*!< idle */
    kRunSetInterface,        /*!< execute set interface code */
    kRunWaitSetInterface,    /*!< wait set interface done */
    kRunGetDeviceId,         /*!< get device id, get all the string */
    kRunWaitGetDeviceId,     /*!< wait get device id callback */
    kRunGetDeviceIdDone,     /*!< get device id success */
    kRunWaitGetDeviceIdAll,  /*!< get whole device id */
    kRunGetDeviceIdAllDone,  /*!< get whole device id done */
    kRunGetDeviceIdAllError, /*!< get whole device id error */
    kRunPrinterTest,         /*!< test the device printer */
    kRunPrimeReceive,        /*!< prime receive */
    kRunDataReceived,        /*!< receive data done */
    kRunParseDeviceId,       /*!< parse device id */
} usb_host_printer_run_state_t;

typedef enum _usb_host_printer_device_type
{
    kPrinter_NXPVirtual = 0u,
    kPrinter_PJLPostscriptor,
} usb_host_printer_device_type_t;

typedef struct _usb_host_printer_app
{
    usb_host_configuration_handle configHandle; /*!< the printer's configuration handle */
    usb_device_handle deviceHandle;             /*!< the printer's device handle */
    usb_host_class_handle classHandle;          /*!< the printer's class handle */
    usb_host_interface_handle interfaceHandle;  /*!< the printer's interface handle */
    usb_status_t callbackStatus;                /*!< keep the callback status */
    uint8_t *deviceIdBuffer;                    /*!< get device id */
    uint32_t receiveLength;                     /*!< received data length */
    uint32_t receiveDelay;                      /*!< receive periodical delay */
    uint8_t *printerAppBuffer;                  /*!< get device id and receive buffer, increasing 1 for \0 character */
    uint8_t deviceState;                        /*!< device attach/detach status */
    uint8_t prevState;                          /*!< device attach/detach previous status */
    uint8_t runState;                           /*!< printer application run status */
    uint8_t runWaitState; /*!< printer application wait status, go to next run status when the wait status success */
    uint8_t selectAlternateSetting; /*!< the supported alternate setting interface */
    uint8_t waitCallback;           /*!< wait callback label */
    uint8_t deviceLanguageType;     /*!< reference to #usb_host_printer_device_type_t */
} usb_host_printer_app_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
* @brief host printer task function.
*
* This function implements the host printer action, it is used to create task.
*
* @param param   the host printer instance pointer.
*/
extern void USB_HostPrinterAppTask(void *param);

/*!
 * @brief host printer callback function.
 *
 * This function should be called in the host callback function.
 *
 * @param deviceHandle        device handle.
 * @param configurationHandle attached device's configuration descriptor information.
 * @param eventCode           callback event code, please reference to enumeration host_event_t.
 *
 * @retval kStatus_USB_Success              The host is initialized successfully.
 * @retval kStatus_USB_NotSupported         The configuration don't contain printer interface.
 */
extern usb_status_t USB_HostPrinterAppEvent(usb_device_handle deviceHandle,
                                            usb_host_configuration_handle configurationHandle,
                                            uint32_t eventCode);

#endif
