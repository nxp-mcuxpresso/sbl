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

#include "usb_host_config.h"
#include "usb_host.h"
#include "usb_host_printer.h"
#include "host_printer.h"
#include "string.h"
#include "stdlib.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEVICE_ID_STRING_POSTSCRIPT "POSTSCRIPT"
#define DEVICE_ID_STRING_PJL "PJL"
#define DEVICE_ID_NXP_STRING "MFG:NXP"
#define DEVICE_ID_STRING_PCL "PCL"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#define ESCAPE 0x1BU

/*******************************************************************************
* Variables
******************************************************************************/

extern usb_host_handle g_HostHandle;

usb_host_printer_app_t g_HostPrinterApp;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_HostPrinterBuffer[USB_HOST_PRINTER_APP_BUFFER_SIZE + 1];

const uint8_t nxpVirtualPrinterTestStr[] =
    "\r\n"
    \
"                file name\r\n"
    \
"        NXP host printer test\r\n";

const uint8_t pjlPostscriptTestStr[] =
    " " /* escape character */
    \
"%-12345X@PJL \r\n"
    \
"@PJL ENTER LANGUAGE = POSTSCRIPT \r\n"
    \
"/inch {72 mul}def\r\n"
    \
"/GetFont {findfont exch scalefont setfont} bind def\r\n"
    \
"/Font1 {0.5 inch /Helvetica GetFont} def\r\n"
    \
"/PrintString {moveto show} bind def\r\n"
    \
"(NXP host printer test) 20 700 Font1 PrintString\r\n"
    \
"showpage"
    \
" " /* escape character */
    \
"%-12345X";

/*******************************************************************************
 * Code
 ******************************************************************************/

static void USB_HostPrinterAppBulkOutCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    usb_host_printer_app_t *printerApp = (usb_host_printer_app_t *)param;

    printerApp->callbackStatus = status;
    printerApp->waitCallback = 1;
}

static void USB_HostPrinterAppBulkInCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    usb_host_printer_app_t *printerApp = (usb_host_printer_app_t *)param;

    if (status == kStatus_USB_Success)
    {
        printerApp->receiveLength = dataLength;
        printerApp->runState = kRunDataReceived;
    }
    else
    {
        printerApp->receiveLength = 0U;
        printerApp->runState = kRunPrimeReceive;
    }
}

static void USB_HostPrinterAppControlCallback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    usb_host_printer_app_t *printerApp = (usb_host_printer_app_t *)param;

    if (printerApp->runState == kRunPrinterTest)
    {
        printerApp->waitCallback = 1;
        printerApp->callbackStatus = status;
    }
    else
    {
        if (status == kStatus_USB_Success)
        {
            if (printerApp->runWaitState == kRunWaitSetInterface) /* set interface finish */
            {
                printerApp->runState = kRunPrinterTest;
            }
            else if (printerApp->runWaitState == kRunWaitGetDeviceId) /* get device id finish */
            {
                printerApp->runState = kRunGetDeviceIdDone;
            }
            else if (printerApp->runWaitState == kRunWaitGetDeviceIdAll)
            {
                printerApp->runState = kRunGetDeviceIdAllDone;
            }
            else
            {
            }
            printerApp->runWaitState = kRunIdle;
        }
        else
        {
            if (printerApp->runWaitState == kRunWaitGetDeviceIdAll)
            {
                printerApp->runState = kRunGetDeviceIdAllError;
            }
        }
    }
}

static void USB_HostPrinterPrintPortStatus(uint8_t portStatus)
{
    if (portStatus & USB_HOST_PRINTER_PORT_STATUS_PAPER_EMPTRY_MASK)
    {
        usb_echo("Paper Empty; ");
    }
    else
    {
        usb_echo("Paper Not Empty; ");
    }

    if (portStatus & USB_HOST_PRINTER_PORT_STATUS_SELECT_MASK)
    {
        usb_echo("Selected; ");
    }
    else
    {
        usb_echo("Not Selected; ");
    }

    if (portStatus & USB_HOST_PRINTER_PORT_STATUS_NOT_ERROR_MASK)
    {
        usb_echo("No Error.\r\n");
    }
    else
    {
        usb_echo("Error.\r\n");
    }
}

static inline void USB_HostControllerTaskFunction(usb_host_handle hostHandle)
{
#if ((defined USB_HOST_CONFIG_KHCI) && (USB_HOST_CONFIG_KHCI))
    USB_HostKhciTaskFunction(hostHandle);
#endif /* USB_HOST_CONFIG_KHCI */
#if ((defined USB_HOST_CONFIG_EHCI) && (USB_HOST_CONFIG_EHCI))
    USB_HostEhciTaskFunction(hostHandle);
#endif /* USB_HOST_CONFIG_EHCI */
#if ((defined USB_HOST_CONFIG_OHCI) && (USB_HOST_CONFIG_OHCI > 0U))
    USB_HostOhciTaskFunction(g_HostHandle);
#endif /* USB_HOST_CONFIG_OHCI */
#if ((defined USB_HOST_CONFIG_IP3516HS) && (USB_HOST_CONFIG_IP3516HS > 0U))
    USB_HostIp3516HsTaskFunction(g_HostHandle);
#endif /* USB_HOST_CONFIG_IP3516HS */
}

static void USB_HostPrinterTest(usb_host_printer_app_t *printerApp)
{
    usb_status_t status;
    uint32_t index;

    /* Get port status */
    if (printerApp->deviceState != kStatus_DEV_Attached)
    {
        return;
    }
    usb_echo("get port status...");
    printerApp->waitCallback = 0;
    status = USB_HostPrinterGetPortStatus(printerApp->classHandle, printerApp->printerAppBuffer,
                                          USB_HostPrinterAppControlCallback, printerApp);
    if (status != kStatus_USB_Success)
    {
        usb_echo("error\r\n");
    }
    while (!printerApp->waitCallback)
    {
        USB_HostControllerTaskFunction(g_HostHandle);
    }
    if (printerApp->callbackStatus == kStatus_USB_Success)
    {
        USB_HostPrinterPrintPortStatus(printerApp->printerAppBuffer[0]);
    }
    else
    {
        usb_echo("fail\r\n");
        return;
    }

    /* printer text */
    if (printerApp->deviceState != kStatus_DEV_Attached)
    {
        return;
    }
    usb_echo("printer text...");
    printerApp->waitCallback = 0;
    if (printerApp->deviceLanguageType == kPrinter_NXPVirtual)
    {
        for (index = 0; index < sizeof(nxpVirtualPrinterTestStr); ++index)
        {
            printerApp->printerAppBuffer[index] = nxpVirtualPrinterTestStr[index];
        }
        status = USB_HostPrinterSend(printerApp->classHandle, printerApp->printerAppBuffer,
                                     sizeof(nxpVirtualPrinterTestStr), USB_HostPrinterAppBulkOutCallback, printerApp);
    }
    else if (printerApp->deviceLanguageType == kPrinter_PJLPostscriptor)
    {
        for (index = 0; index < sizeof(pjlPostscriptTestStr); ++index)
        {
            printerApp->printerAppBuffer[index] = pjlPostscriptTestStr[index];
        }
        /* to fix misra error */
        printerApp->printerAppBuffer[0] = ESCAPE;
        printerApp->printerAppBuffer[sizeof(pjlPostscriptTestStr) - 10] = ESCAPE;
        status = USB_HostPrinterSend(printerApp->classHandle, printerApp->printerAppBuffer,
                                     sizeof(pjlPostscriptTestStr), USB_HostPrinterAppBulkOutCallback, printerApp);
    }
    else
    {
        usb_echo("no data\r\n");
        return;
    }
    if (status != kStatus_USB_Success)
    {
        usb_echo("error\r\n");
    }
    while (!printerApp->waitCallback)
    {
        USB_HostControllerTaskFunction(g_HostHandle);
    }
    if (printerApp->callbackStatus == kStatus_USB_Success)
    {
        usb_echo("success\r\n");
        return;
    }
    else
    {
        usb_echo("fail\r\n");
    }

    /* if there is error */
    if (printerApp->deviceState != kStatus_DEV_Attached)
    {
        return;
    }
    usb_echo("printer soft reset...");
    printerApp->waitCallback = 0;
    status = USB_HostPrinterSoftReset(printerApp->classHandle, USB_HostPrinterAppControlCallback, printerApp);
    if (status != kStatus_USB_Success)
    {
        usb_echo("error\r\n");
    }
    while (!printerApp->waitCallback)
    {
        USB_HostControllerTaskFunction(g_HostHandle);
    }
    if (printerApp->callbackStatus == kStatus_USB_Success)
    {
        usb_echo("success\r\n");
    }
    else
    {
        usb_echo("fail\r\n");
    }
}

void USB_HostPrinterAppTask(void *param)
{
    usb_host_printer_app_t *printerApp = (usb_host_printer_app_t *)param;
    usb_status_t status;
    uint8_t *idBuffer;
    uint32_t idLength;
    uint8_t interfaceIndex;
    uint8_t support;

    /* device state changes, process once for each state */
    if (printerApp->deviceState != printerApp->prevState)
    {
        printerApp->prevState = printerApp->deviceState;
        switch (printerApp->deviceState)
        {
            case kStatus_DEV_Idle:
                break;

            case kStatus_DEV_Attached: /* device is attached and numeration is done */
                printerApp->runState = kRunGetDeviceId;
                /* printer class initialization */
                if (USB_HostPrinterInit(printerApp->deviceHandle, &printerApp->classHandle) != kStatus_USB_Success)
                {
                    usb_echo("host printer class initialize fail\r\n");
                }
                break;

            case kStatus_DEV_Detached: /* device is detached */
                printerApp->deviceState = kStatus_DEV_Idle;
                printerApp->runState = kRunIdle;
                /* printer class de-initialization */
                USB_HostPrinterDeinit(printerApp->deviceHandle, printerApp->classHandle);
                printerApp->classHandle = NULL;
                usb_echo("printer detached\r\n");
                break;

            default:
                break;
        }
    }

    /* run state */
    switch (printerApp->runState)
    {
        case kRunIdle:
            break;

        case kRunGetDeviceId: /* 1. get device id */
            printerApp->runState = kRunIdle;
            if (printerApp->deviceIdBuffer != NULL)
            {
                free(printerApp->deviceIdBuffer);
                printerApp->deviceIdBuffer = NULL;
            }
            printerApp->runWaitState = kRunWaitGetDeviceId;
            interfaceIndex = ((usb_host_interface_t *)printerApp->interfaceHandle)->interfaceDesc->bInterfaceNumber;
            status =
                USB_HostPrinterGetDeviceId(printerApp->classHandle, interfaceIndex, printerApp->selectAlternateSetting,
                                           printerApp->printerAppBuffer, USB_HOST_PRINTER_APP_BUFFER_SIZE,
                                           USB_HostPrinterAppControlCallback, printerApp);
            if (status != kStatus_USB_Success)
            {
                usb_echo("get device id error\r\n");
            }
            break;

        case kRunGetDeviceIdDone:
            idLength = printerApp->printerAppBuffer[0];
            idLength <<= 8;
            idLength |= printerApp->printerAppBuffer[1];

            printerApp->runState = kRunIdle;
            if (idLength > USB_HOST_PRINTER_APP_BUFFER_SIZE) /* the device id is longer */
            {
                printerApp->deviceIdBuffer = malloc(idLength + 1);
                if (printerApp->deviceIdBuffer == NULL)
                {
                    usb_echo("malloc error\r\n");
                    return;
                }
                printerApp->runWaitState = kRunWaitGetDeviceIdAll;
                interfaceIndex = ((usb_host_interface_t *)printerApp->interfaceHandle)->interfaceDesc->bInterfaceNumber;
                status = USB_HostPrinterGetDeviceId(printerApp->classHandle, interfaceIndex,
                                                    printerApp->selectAlternateSetting, printerApp->deviceIdBuffer,
                                                    idLength, USB_HostPrinterAppControlCallback, printerApp);
                if (status != kStatus_USB_Success)
                {
                    usb_echo("get device id error\r\n");
                }
            }
            else /* the device id is all */
            {
                printerApp->runState = kRunParseDeviceId;
            }
            break;

        case kRunGetDeviceIdAllDone: /* 2. get device id done */
            printerApp->runState = kRunParseDeviceId;
            break;

        case kRunGetDeviceIdAllError:
            printerApp->runState = kRunIdle;
            free(printerApp->deviceIdBuffer);
            printerApp->deviceIdBuffer = NULL;
            break;

        case kRunParseDeviceId:
            if (printerApp->deviceIdBuffer != NULL)
            {
                idBuffer = printerApp->deviceIdBuffer;
            }
            else
            {
                idBuffer = printerApp->printerAppBuffer;
            }
            idLength = idBuffer[0];
            idLength <<= 8;
            idLength |= idBuffer[1];

            idBuffer[idLength] = 0;
            usb_echo("%s\r\n", &idBuffer[2]);

            support = 0;
            if (strstr((char *)&idBuffer[2], DEVICE_ID_STRING_PJL))
            {
                if (strstr((char *)&idBuffer[2], DEVICE_ID_STRING_POSTSCRIPT)) /* pjl & postscriptor */
                {
                    support = 1U;
                    printerApp->deviceLanguageType = kPrinter_PJLPostscriptor;
                }
            }
            else if (strstr((char *)&idBuffer[2], DEVICE_ID_NXP_STRING)) /* device printer demo */
            {
                support = 1U;
                printerApp->deviceLanguageType = kPrinter_NXPVirtual;
            }
            else
            {
            }

            if (support)
            {
                printerApp->runState = kRunSetInterface;
            }
            else
            {
                usb_echo("unsupported printer language\r\n");
            }

            if (printerApp->deviceIdBuffer != NULL)
            {
                free(printerApp->deviceIdBuffer);
                printerApp->deviceIdBuffer = NULL;
            }
            break;

        case kRunSetInterface: /* 3. set supported printer interface */
            printerApp->runWaitState = kRunWaitSetInterface;
            printerApp->runState = kRunIdle;
            if (USB_HostPrinterSetInterface(printerApp->classHandle, printerApp->interfaceHandle,
                                            printerApp->selectAlternateSetting, USB_HostPrinterAppControlCallback,
                                            printerApp) != kStatus_USB_Success)
            {
                usb_echo("set interface error\r\n");
            }
            break;

        case kRunPrinterTest:
            USB_HostPrinterTest(printerApp);
            printerApp->runState = kRunPrimeReceive;
            break;

        case kRunPrimeReceive:
            if (printerApp->deviceState != kStatus_DEV_Attached)
            {
                return;
            }
            printerApp->receiveDelay++;
            if (printerApp->receiveDelay < (USB_HOST_PRINTER_APP_RECEIVE_TRY_DELAY * USB_HOST_PRINTER_APP_ONEMS_COUNT))
            {
                return;
            }

            printerApp->receiveDelay = 0;
            printerApp->runState = kRunIdle;
            /* receive data */
            status =
                USB_HostPrinterRecv(printerApp->classHandle, printerApp->printerAppBuffer,
                                    USB_HOST_PRINTER_APP_BUFFER_SIZE, USB_HostPrinterAppBulkInCallback, printerApp);
            if (status != kStatus_USB_Success)
            {
                usb_echo("error\r\n");
            }
            break;

        case kRunDataReceived:
            if (printerApp->receiveLength > 0)
            {
                printerApp->printerAppBuffer[printerApp->receiveLength] = 0;
                usb_echo("%s\r\n", printerApp->printerAppBuffer);
            }
            printerApp->runState = kRunPrimeReceive;
            break;

        default:
            break;
    }
}

usb_status_t USB_HostPrinterAppEvent(usb_device_handle deviceHandle,
                                     usb_host_configuration_handle configurationHandle,
                                     uint32_t eventCode)
{
    usb_status_t status = kStatus_USB_Success;
    usb_host_configuration_t *configuration;
    usb_host_interface_t *interface;
    uint32_t infoValue;
    usb_descriptor_interface_t *interfaceDesc;
    uint32_t endPos;
    usb_descriptor_union_t *unionDes;
    uint8_t interfaceIndex;
    uint8_t alternateIndex;
    uint8_t support;

    switch (eventCode)
    {
        case kUSB_HostEventAttach:
            /* judge whether is configurationHandle supported */
            configuration = (usb_host_configuration_t *)configurationHandle;
            for (interfaceIndex = 0; interfaceIndex < configuration->interfaceCount; ++interfaceIndex)
            {
                interface = &configuration->interfaceList[interfaceIndex];
                interfaceDesc = interface->interfaceDesc;
                if ((interfaceDesc->bInterfaceClass != USB_HOST_PRINTER_CLASS_CODE) ||
                    (interfaceDesc->bInterfaceSubClass != USB_HOST_PRINTER_SUBCLASS_CODE))
                {
                    continue;
                }

                support = 0;
                if (interfaceDesc->bInterfaceProtocol == USB_HOST_PRINTER_PROTOCOL_BIDIRECTION)
                {
                    support = 1;
                }
                else if (interface->alternateSettingNumber != 0) /* need check the alternate setting interfaces */
                {
                    unionDes = (usb_descriptor_union_t *)(interface->interfaceExtension);
                    endPos = (uint32_t)(interface->interfaceExtension + interface->interfaceExtensionLength);
                    for (alternateIndex = 0; alternateIndex < interface->alternateSettingNumber; ++alternateIndex)
                    {
                        interfaceDesc = NULL;
                        while ((uint32_t)unionDes < endPos)
                        {
                            if (unionDes->common.bDescriptorType != USB_DESCRIPTOR_TYPE_INTERFACE)
                            {
                                unionDes = (usb_descriptor_union_t *)((uint32_t)unionDes + unionDes->common.bLength);
                            }
                            else
                            {
                                interfaceDesc = (usb_descriptor_interface_t *)(&(unionDes->common.bLength));
                                break;
                            }
                        }

                        if ((interfaceDesc != NULL) &&
                            (interfaceDesc->bInterfaceClass == USB_HOST_PRINTER_CLASS_CODE) &&
                            (interfaceDesc->bInterfaceSubClass == USB_HOST_PRINTER_SUBCLASS_CODE) &&
                            (interfaceDesc->bInterfaceProtocol == USB_HOST_PRINTER_PROTOCOL_BIDIRECTION))
                        {
                            support = 1;
                            break;
                        }
                    }
                }
                else
                {
                }

                if (support)
                {
                    if (g_HostPrinterApp.deviceState == kStatus_DEV_Idle)
                    {
                        /* the interface is supported by the application */
                        g_HostPrinterApp.printerAppBuffer = s_HostPrinterBuffer;
                        g_HostPrinterApp.deviceHandle = deviceHandle;
                        g_HostPrinterApp.interfaceHandle = interface;
                        g_HostPrinterApp.configHandle = configurationHandle;
                        g_HostPrinterApp.selectAlternateSetting = interfaceDesc->bAlternateSetting;
                        return kStatus_USB_Success;
                    }
                }
            }
            status = kStatus_USB_NotSupported;
            break;

        case kUSB_HostEventNotSupported:
            break;

        case kUSB_HostEventEnumerationDone:
            if (g_HostPrinterApp.configHandle == configurationHandle)
            {
                if ((g_HostPrinterApp.deviceHandle != NULL) && (g_HostPrinterApp.interfaceHandle != NULL))
                {
                    /* the device enumeration is done */
                    if (g_HostPrinterApp.deviceState == kStatus_DEV_Idle)
                    {
                        g_HostPrinterApp.deviceState = kStatus_DEV_Attached;

                        USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDevicePID, &infoValue);
                        usb_echo("printer attached:pid=0x%x", infoValue);
                        USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDeviceVID, &infoValue);
                        usb_echo("vid=0x%x ", infoValue);
                        USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDeviceAddress, &infoValue);
                        usb_echo("address=%d\r\n", infoValue);
                    }
                    else
                    {
                        usb_echo("not idle printer app instance\r\n");
                        status = kStatus_USB_Error;
                    }
                }
            }
            break;

        case kUSB_HostEventDetach:
            if (g_HostPrinterApp.configHandle == configurationHandle)
            {
                /* the device is detached */
                g_HostPrinterApp.configHandle = NULL;
                if (g_HostPrinterApp.deviceState != kStatus_DEV_Idle)
                {
                    g_HostPrinterApp.deviceState = kStatus_DEV_Detached;
                }
            }
            break;

        default:
            break;
    }

    return status;
}
