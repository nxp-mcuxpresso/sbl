/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
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
#include "usb_device_printer.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "device_printer.h"
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "board.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#if ((defined USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI))
#include "usb_phy.h"
#endif /* USB_HOST_CONFIG_EHCI */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* The printer class callback */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

void BOARD_InitHardware(void);
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_PrinterBuffer[USB_PRINTER_BUFFER_SIZE];
/* printer class specifice transfer buffer */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_PrinterClassBuffer[64];
/* printer device id for interface zero */
const uint8_t g_PrinterID[] = "xxMFG:NXP;MDL:ksdk printer demo;CMD:POSTSCRIPT";

usb_device_printer_app_t g_DevicePrinterApp;

extern uint8_t g_UsbDeviceCurrentConfigure;
extern uint8_t g_UsbDeviceInterface[USB_PRINTER_INTERFACE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/

/* ksdk debug console must have been initialized. */
static void USB_PrinterPrintData(uint8_t *data, uint32_t length)
{
    while (length--)
    {
        PUTCHAR(*(data++));
    }
}

/*!
 * @brief bulk IN endpoint callback function.
 */
static usb_status_t USB_DevicePrinterBulkInCallback(usb_device_handle handle,
                                                    usb_device_endpoint_callback_message_struct_t *message,
                                                    void *callbackParam)
{
    USB_DeviceSendRequest(g_DevicePrinterApp.deviceHandle, USB_PRINTER_BULK_ENDPOINT_IN, g_DevicePrinterApp.sendBuffer,
                          g_DevicePrinterApp.sendLength);
    return kStatus_USB_Success;
}

/*!
 * @brief bulk OUT endpoint callback function.
 */
static usb_status_t USB_DevicePrinterBulkOutCallback(usb_device_handle handle,
                                                     usb_device_endpoint_callback_message_struct_t *message,
                                                     void *callbackParam)
{
    usb_status_t status = kStatus_USB_Error;

    if ((g_DevicePrinterApp.attach) && (g_DevicePrinterApp.printerState == kPrinter_Receiving))
    {
        if ((message != NULL) && (message->length != USB_UNINITIALIZED_VAL_32))
        {
            g_DevicePrinterApp.printerState = kPrinter_Received;
            g_DevicePrinterApp.dataReceiveLength = message->length;
        }
        else
        {
            g_DevicePrinterApp.printerState = kPrinter_Idle;
            status = USB_DeviceRecvRequest(g_DevicePrinterApp.deviceHandle, USB_PRINTER_BULK_ENDPOINT_OUT,
                                           g_DevicePrinterApp.printerBuffer, USB_PRINTER_BUFFER_SIZE);
            if (status == kStatus_USB_Success)
            {
                g_DevicePrinterApp.printerState = kPrinter_Receiving;
            }
        }
    }

    return status;
}

usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t status = kStatus_USB_Success;
    uint8_t *param8p = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
            /* Initialize the control IN and OUT pipes */
            USB_DeviceControlPipeInit(g_DevicePrinterApp.deviceHandle);
            g_DevicePrinterApp.attach = 0U;
            g_DevicePrinterApp.printerState = kPrinter_Idle;
            g_DevicePrinterApp.sendBuffer = NULL;
            g_DevicePrinterApp.sendLength = 0;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success ==
                USB_DeviceGetStatus(g_DevicePrinterApp.deviceHandle, kUSB_DeviceStatusSpeed, &g_DevicePrinterApp.speed))
            {
                USB_DeviceSetSpeed(g_DevicePrinterApp.speed);
            }
#endif
            status = kStatus_USB_Success;
            break;

#if (defined(USB_DEVICE_CONFIG_DETACH_ENABLE) && (USB_DEVICE_CONFIG_DETACH_ENABLE > 0U))

        case kUSB_DeviceEventAttach:
            usb_echo("USB device attached.\r\n");
            USB_DeviceRun(g_DevicePrinterApp.deviceHandle);
            break;

        case kUSB_DeviceEventDetach:
            usb_echo("USB device detached.\r\n");
            g_DevicePrinterApp.attach = 0;
            USB_DeviceStop(g_DevicePrinterApp.deviceHandle);
            break;
#endif

        case kUSB_DeviceEventSetConfiguration:
            if (USB_PRINTER_CONFIGURE_INDEX == (*param8p))
            {
                /* If the configuration is valid, initialize the printer pipes */
                usb_device_endpoint_init_struct_t epInitStruct;
                usb_device_endpoint_callback_struct_t epCallback;

                epInitStruct.zlt = 0U;
                epInitStruct.transferType = USB_ENDPOINT_BULK;
                epCallback.callbackParam = handle;

                /* initialize bulk out endpoint */
                epCallback.callbackFn = USB_DevicePrinterBulkOutCallback;
                epInitStruct.endpointAddress =
                    USB_PRINTER_BULK_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                if (USB_SPEED_HIGH == g_DevicePrinterApp.speed)
                {
                    epInitStruct.maxPacketSize = HS_PRINTER_BULK_OUT_PACKET_SIZE;
                }
                else
                {
                    epInitStruct.maxPacketSize = FS_PRINTER_BULK_OUT_PACKET_SIZE;
                }

                USB_DeviceInitEndpoint(g_DevicePrinterApp.deviceHandle, &epInitStruct, &epCallback);

                /* initialize bulk in endpoint */
                epCallback.callbackFn = USB_DevicePrinterBulkInCallback;
                epInitStruct.endpointAddress =
                    USB_PRINTER_BULK_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                if (USB_SPEED_HIGH == g_DevicePrinterApp.speed)
                {
                    epInitStruct.maxPacketSize = HS_PRINTER_BULK_IN_PACKET_SIZE;
                }
                else
                {
                    epInitStruct.maxPacketSize = FS_PRINTER_BULK_IN_PACKET_SIZE;
                }

                USB_DeviceInitEndpoint(g_DevicePrinterApp.deviceHandle, &epInitStruct, &epCallback);

                g_DevicePrinterApp.attach = 1U;
                g_DevicePrinterApp.printerState = kPrinter_Idle;
                /* demo run */
                status = USB_DeviceRecvRequest(g_DevicePrinterApp.deviceHandle, USB_PRINTER_BULK_ENDPOINT_OUT,
                                               g_DevicePrinterApp.printerBuffer, USB_PRINTER_BUFFER_SIZE);
                if (status == kStatus_USB_Success)
                {
                    g_DevicePrinterApp.printerState = kPrinter_Receiving;
                }

                USB_DeviceSendRequest(g_DevicePrinterApp.deviceHandle, USB_PRINTER_BULK_ENDPOINT_IN,
                                      g_DevicePrinterApp.sendBuffer, g_DevicePrinterApp.sendLength);
            }
            break;

        default:
            break;
    }

    return status;
}

/* Get setup buffer */
usb_status_t USB_DeviceGetSetupBuffer(usb_device_handle handle, usb_setup_struct_t **setupBuffer)
{
    /* Keep the setup is 4-byte aligned */
    static uint32_t printerAppSetup[2];
    if (NULL == setupBuffer)
    {
        return kStatus_USB_InvalidParameter;
    }
    *setupBuffer = (usb_setup_struct_t *)&printerAppSetup;
    return kStatus_USB_Success;
}

/* Configure device remote wakeup */
usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable)
{
    return kStatus_USB_InvalidRequest;
}

/* Configure the endpoint status (idle or stall) */
usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status)
{
    if (status)
    {
        if ((USB_PRINTER_BULK_ENDPOINT_OUT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
            ((ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) == USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT))
        {
            return USB_DeviceStallEndpoint(handle, ep);
        }
        else if ((USB_PRINTER_BULK_ENDPOINT_IN == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
                 ((ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                  USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN))
        {
            return USB_DeviceStallEndpoint(handle, ep);
        }
        else
        {
        }
    }
    else
    {
        if ((USB_PRINTER_BULK_ENDPOINT_OUT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
            ((ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) == USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT))
        {
            return USB_DeviceUnstallEndpoint(handle, ep);
        }
        else if ((USB_PRINTER_BULK_ENDPOINT_IN == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
                 ((ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                  USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN))
        {
            return USB_DeviceUnstallEndpoint(handle, ep);
        }
        else
        {
        }
    }
    return kStatus_USB_InvalidRequest;
}

/* Get class-specific request buffer */
usb_status_t USB_DeviceGetClassReceiveBuffer(usb_device_handle handle,
                                             usb_setup_struct_t *setup,
                                             uint32_t *length,
                                             uint8_t **buffer)
{
    *buffer = NULL;
    *length = 0;
    return kStatus_USB_InvalidRequest;
}

/* Handle class-specific request */
usb_status_t USB_DeviceProcessClassRequest(usb_device_handle handle,
                                           usb_setup_struct_t *setup,
                                           uint32_t *length,
                                           uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_Success;
    uint32_t len;

    if (setup->wIndex != USB_PRINTER_INTERFACE_INDEX)
    {
        return error;
    }

    switch (setup->bRequest)
    {
        case USB_DEVICE_PRINTER_GET_DEVICE_ID:
            if (((uint8_t)setup->wValue == 0U) && ((uint8_t)(setup->wIndex >> 8) == USB_PRINTER_INTERFACE_INDEX) &&
                ((uint8_t)(setup->wIndex) == 0))
            {
                for (len = 0; len < sizeof(g_PrinterID); ++len)
                {
                    s_PrinterClassBuffer[len] = g_PrinterID[len];
                }
                len = sizeof(g_PrinterID) - 1;
                s_PrinterClassBuffer[0] = ((uint8_t)(len >> 8));
                s_PrinterClassBuffer[1] = (uint8_t)len;
                *buffer = s_PrinterClassBuffer;
                *length = len;
            }
            else
            {
                *buffer = NULL;
                *length = 0U;
            }
            break;

        case USB_DEVICE_PRINTER_GET_PORT_STATUS:
            if ((uint8_t)(setup->wIndex) == USB_PRINTER_INTERFACE_INDEX)
            {
                s_PrinterClassBuffer[0] = g_DevicePrinterApp.printerPortStatus;
                *buffer = s_PrinterClassBuffer;
                *length = 1U;
            }
            else
            {
                *buffer = NULL;
                *length = 0U;
            }
            break;

        case USB_DEVICE_PRINTER_SOFT_RESET:
            if ((uint8_t)(setup->wIndex) == USB_PRINTER_INTERFACE_INDEX)
            {
                g_DevicePrinterApp.printerState = kPrinter_Idle;
            }
            break;
        default:
            break;
    }

    return error;
}

static void USB_DeviceApplicationInit(void)
{
    USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    /* set printer app to default state */
    g_DevicePrinterApp.printerPortStatus = USB_DEVICE_PRINTER_PORT_STATUS_DEFAULT_VALUE;
    g_DevicePrinterApp.printerState = kPrinter_Idle;
    g_DevicePrinterApp.speed = USB_SPEED_FULL;
    g_DevicePrinterApp.attach = 0U;
    g_DevicePrinterApp.deviceHandle = NULL;
    g_DevicePrinterApp.printerBuffer = s_PrinterBuffer;

    /* Initialize the usb stack and class drivers */
    if (kStatus_USB_Success != USB_DeviceInit(CONTROLLER_ID, USB_DeviceCallback, &g_DevicePrinterApp.deviceHandle))
    {
        usb_echo("USB device printer fail\r\n");
        return;
    }
    else
    {
        usb_echo("USB device printer demo\r\n");
    }

    /* Install isr, set priority, and enable IRQ. */
    USB_DeviceIsrEnable();

    /* Start USB printer demo */
    USB_DeviceRun(g_DevicePrinterApp.deviceHandle);
}

void USB_DevicePrinterAppTask(void *parameter)
{
    usb_device_printer_app_t *printerApp = (usb_device_printer_app_t *)parameter;
    usb_status_t status = kStatus_USB_Error;

    if (printerApp->attach)
    {
        if (printerApp->printerState == kPrinter_Received)
        {
            USB_PrinterPrintData(printerApp->printerBuffer, printerApp->dataReceiveLength);
            printerApp->printerState = kPrinter_Idle;
        }

        if (printerApp->printerState == kPrinter_Idle)
        {
            status = USB_DeviceRecvRequest(printerApp->deviceHandle, USB_PRINTER_BULK_ENDPOINT_OUT,
                                           printerApp->printerBuffer, USB_PRINTER_BUFFER_SIZE);

            if (status == kStatus_USB_Success)
            {
                printerApp->printerState = kPrinter_Receiving;
            }
        }
    }
}

#if defined(__CC_ARM) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
    BOARD_InitHardware();

    USB_DeviceApplicationInit();

    while (1U)
    {
#if USB_DEVICE_CONFIG_USE_TASK
        USB_DeviceTaskFn(g_DevicePrinterApp.deviceHandle);
#endif

        USB_DevicePrinterAppTask(&g_DevicePrinterApp);
    }
}
