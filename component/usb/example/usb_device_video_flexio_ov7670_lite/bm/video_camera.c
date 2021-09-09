/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
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

#include "usb_device_video.h"

#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "video_camera.h"

#include "fsl_camera_receiver.h"
#include "fsl_camera_device.h"

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

#include <stdio.h>
#include <stdlib.h>

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
#include "usb_phy.h"
#endif

#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define CAMERA_FRAME_BYTES                                                                    \
    (USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_WIDTH * USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_HEIGHT * \
     USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_DATA_BITS)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle);
#endif

static void USB_DeviceVideoPrepareVideoData(void);
static usb_status_t USB_DeviceVideoIsoIn(usb_device_handle deviceHandle,
                                         usb_device_endpoint_callback_message_struct_t *event,
                                         void *arg);
static void USB_DeviceVideoApplicationSetDefault(void);
static usb_status_t USB_DeviceVideoProcessClassVsProbeRequest(usb_device_handle handle,
                                                              usb_setup_struct_t *setup,
                                                              uint32_t *length,
                                                              uint8_t **buffer);
static usb_status_t USB_DeviceVideoProcessClassVsCommitRequest(usb_device_handle handle,
                                                               usb_setup_struct_t *setup,
                                                               uint32_t *length,
                                                               uint8_t **buffer);
static usb_status_t USB_DeviceVideoProcessClassVsStillProbeRequest(usb_device_handle handle,
                                                                   usb_setup_struct_t *setup,
                                                                   uint32_t *length,
                                                                   uint8_t **buffer);
static usb_status_t USB_DeviceVideoProcessClassVsStillCommitRequest(usb_device_handle handle,
                                                                    usb_setup_struct_t *setup,
                                                                    uint32_t *length,
                                                                    uint8_t **buffer);
static usb_status_t USB_DeviceVideoProcessClassVsStillImageTriggerRequest(usb_device_handle handle,
                                                                          usb_setup_struct_t *setup,
                                                                          uint32_t *length,
                                                                          uint8_t **buffer);
static usb_status_t USB_DeviceVideoProcessClassVsRequest(usb_device_handle handle,
                                                         usb_setup_struct_t *setup,
                                                         uint32_t *length,
                                                         uint8_t **buffer);
static usb_status_t USB_DeviceProcessClassVcInterfaceRequest(usb_device_handle handle,
                                                             usb_setup_struct_t *setup,
                                                             uint32_t *length,
                                                             uint8_t **buffer);
static usb_status_t USB_DeviceProcessClassVcCameraTerminalRequest(usb_device_handle handle,
                                                                  usb_setup_struct_t *setup,
                                                                  uint32_t *length,
                                                                  uint8_t **buffer);
static usb_status_t USB_DeviceProcessClassVcInputTerminalRequest(usb_device_handle handle,
                                                                 usb_setup_struct_t *setup,
                                                                 uint32_t *length,
                                                                 uint8_t **buffer);
static usb_status_t USB_DeviceProcessClassVcOutputTerminalRequest(usb_device_handle handle,
                                                                  usb_setup_struct_t *setup,
                                                                  uint32_t *length,
                                                                  uint8_t **buffer);
static usb_status_t USB_DeviceProcessClassVcProcessingUintRequest(usb_device_handle handle,
                                                                  usb_setup_struct_t *setup,
                                                                  uint32_t *length,
                                                                  uint8_t **buffer);
static usb_status_t USB_DeviceProcessClassVcRequest(usb_device_handle handle,
                                                    usb_setup_struct_t *setup,
                                                    uint32_t *length,
                                                    uint8_t **buffer);
static void USB_DeviceApplicationInit(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern camera_device_handle_t cameraDevice;
extern camera_receiver_handle_t cameraReceiver;

usb_video_flexio_camera_struct_t g_UsbDeviceVideoFlexioCamera;

extern const unsigned char g_UsbDeviceVideoMjpegData[];
extern const uint32_t g_UsbDeviceVideoMjpegLength;
extern uint8_t g_UsbDeviceCurrentConfigure;
extern uint8_t g_UsbDeviceInterface[USB_VIDEO_CAMERA_INTERFACE_COUNT];

USB_DMA_NONINIT_DATA_ALIGN(((32 > USB_DATA_ALIGN_SIZE) ? 32 : USB_DATA_ALIGN_SIZE))
uint8_t g_FlexioCameraFrameBuffer[2][USB_DATA_ALIGN_SIZE_MULTIPLE(CAMERA_FRAME_BYTES + 32)];

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Prepare next transfer payload */
static void USB_DeviceVideoPrepareVideoData(void)
{
    usb_device_video_mjpeg_payload_header_struct_t *payloadHeader;
    uint32_t fullBufferAddr;
    uint32_t maxPacketSize;
    uint32_t i;
    uint32_t sendLength;
    uint32_t temp32dwFrameInterval;

    g_UsbDeviceVideoFlexioCamera.currentTime += 10000U;

    if (g_UsbDeviceVideoFlexioCamera.imagePosition < CAMERA_FRAME_BYTES)
    {
        i = (uint32_t)&g_FlexioCameraFrameBuffer[g_UsbDeviceVideoFlexioCamera.fullBufferIndex]
                                                [g_UsbDeviceVideoFlexioCamera.imagePosition -
                                                 sizeof(usb_device_video_mjpeg_payload_header_struct_t) + 32];
        g_UsbDeviceVideoFlexioCamera.imageBuffer = (uint8_t *)i;
    }
    else
    {
        g_UsbDeviceVideoFlexioCamera.imageBuffer = (uint8_t *)&g_UsbDeviceVideoFlexioCamera.image_header[0];
    }
    payloadHeader = (usb_device_video_mjpeg_payload_header_struct_t *)g_UsbDeviceVideoFlexioCamera.imageBuffer;
    for (i = 0; i < sizeof(usb_device_video_mjpeg_payload_header_struct_t); i++)
    {
        g_UsbDeviceVideoFlexioCamera.imageBuffer[i] = 0x00U;
    }
    payloadHeader->bHeaderLength = sizeof(usb_device_video_mjpeg_payload_header_struct_t);
    payloadHeader->headerInfoUnion.bmheaderInfo = 0U;
    payloadHeader->headerInfoUnion.headerInfoBits.frameIdentifier = g_UsbDeviceVideoFlexioCamera.currentFrameId;
    g_UsbDeviceVideoFlexioCamera.imageBufferLength = sizeof(usb_device_video_mjpeg_payload_header_struct_t);

    if (g_UsbDeviceVideoFlexioCamera.stillImageTransmission)
    {
        payloadHeader->headerInfoUnion.headerInfoBits.stillImage = 1U;
        maxPacketSize =
            USB_LONG_FROM_LITTLE_ENDIAN_DATA(g_UsbDeviceVideoFlexioCamera.stillCommitStruct.dwMaxPayloadTransferSize);
    }
    else
    {
        maxPacketSize =
            USB_LONG_FROM_LITTLE_ENDIAN_DATA(g_UsbDeviceVideoFlexioCamera.commitStruct.dwMaxPayloadTransferSize);
    }
    maxPacketSize = maxPacketSize - sizeof(usb_device_video_mjpeg_payload_header_struct_t);
    if (g_UsbDeviceVideoFlexioCamera.waitForNewInterval)
    {
        temp32dwFrameInterval =
            USB_LONG_FROM_LITTLE_ENDIAN_DATA(g_UsbDeviceVideoFlexioCamera.commitStruct.dwFrameInterval);
        if (g_UsbDeviceVideoFlexioCamera.currentTime < temp32dwFrameInterval)
        {
            return;
        }
        else
        {
            g_UsbDeviceVideoFlexioCamera.imagePosition = 0U;
            g_UsbDeviceVideoFlexioCamera.currentTime = 0U;
            g_UsbDeviceVideoFlexioCamera.waitForNewInterval = 0U;
            payloadHeader->headerInfoUnion.headerInfoBits.endOfFrame = 1U;
            g_UsbDeviceVideoFlexioCamera.stillImageTransmission = 0U;
            g_UsbDeviceVideoFlexioCamera.currentFrameId ^= 1U;
            if (USB_DEVICE_VIDEO_STILL_IMAGE_TRIGGER_TRANSMIT_STILL_IMAGE ==
                g_UsbDeviceVideoFlexioCamera.stillImageTriggerControl)
            {
                g_UsbDeviceVideoFlexioCamera.stillImageTriggerControl =
                    USB_DEVICE_VIDEO_STILL_IMAGE_TRIGGER_NORMAL_OPERATION;
                g_UsbDeviceVideoFlexioCamera.stillImageTransmission = 1U;
            }
            return;
        }
    }

    if (g_UsbDeviceVideoFlexioCamera.imagePosition < CAMERA_FRAME_BYTES)
    {
        sendLength = CAMERA_FRAME_BYTES - g_UsbDeviceVideoFlexioCamera.imagePosition;
        if (sendLength > maxPacketSize)
        {
            sendLength = maxPacketSize;
        }
        g_UsbDeviceVideoFlexioCamera.imagePosition += sendLength;

        if (g_UsbDeviceVideoFlexioCamera.imagePosition >= CAMERA_FRAME_BYTES)
        {
            CAMERA_RECEIVER_SubmitEmptyBuffer(
                &cameraReceiver,
                (uint32_t)&g_FlexioCameraFrameBuffer[g_UsbDeviceVideoFlexioCamera.fullBufferIndex][32]);
            CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &fullBufferAddr);
            temp32dwFrameInterval =
                USB_LONG_FROM_LITTLE_ENDIAN_DATA(g_UsbDeviceVideoFlexioCamera.commitStruct.dwFrameInterval);
            if (((uint32_t)fullBufferAddr) == ((uint32_t)&g_FlexioCameraFrameBuffer[0][32]))
            {
                g_UsbDeviceVideoFlexioCamera.fullBufferIndex = 0;
            }
            else
            {
                g_UsbDeviceVideoFlexioCamera.fullBufferIndex = 1;
            }
            if (g_UsbDeviceVideoFlexioCamera.currentTime < temp32dwFrameInterval)
            {
                g_UsbDeviceVideoFlexioCamera.waitForNewInterval = 1U;
            }
            else
            {
                g_UsbDeviceVideoFlexioCamera.imagePosition = 0U;
                g_UsbDeviceVideoFlexioCamera.currentTime = 0U;
                payloadHeader->headerInfoUnion.headerInfoBits.endOfFrame = 1U;
                g_UsbDeviceVideoFlexioCamera.stillImageTransmission = 0U;
                g_UsbDeviceVideoFlexioCamera.currentFrameId ^= 1U;
                if (USB_DEVICE_VIDEO_STILL_IMAGE_TRIGGER_TRANSMIT_STILL_IMAGE ==
                    g_UsbDeviceVideoFlexioCamera.stillImageTriggerControl)
                {
                    g_UsbDeviceVideoFlexioCamera.stillImageTriggerControl =
                        USB_DEVICE_VIDEO_STILL_IMAGE_TRIGGER_NORMAL_OPERATION;
                    g_UsbDeviceVideoFlexioCamera.stillImageTransmission = 1U;
                }
            }
        }
        g_UsbDeviceVideoFlexioCamera.imageBufferLength += sendLength;
    }
}

/* USB device video ISO IN endpoint callback */
static usb_status_t USB_DeviceVideoIsoIn(usb_device_handle deviceHandle,
                                         usb_device_endpoint_callback_message_struct_t *event,
                                         void *arg)
{
    if (g_UsbDeviceVideoFlexioCamera.attach)
    {
        USB_DeviceVideoPrepareVideoData();
        return USB_DeviceSendRequest(deviceHandle, USB_VIDEO_CAMERA_STREAM_ENDPOINT_IN,
                                     g_UsbDeviceVideoFlexioCamera.imageBuffer,
                                     g_UsbDeviceVideoFlexioCamera.imageBufferLength);
    }

    return kStatus_USB_Error;
}

/* Set to default state */
static void USB_DeviceVideoApplicationSetDefault(void)
{
    g_UsbDeviceVideoFlexioCamera.speed = USB_SPEED_FULL;
    g_UsbDeviceVideoFlexioCamera.attach = 0U;
    g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize = FS_STREAM_IN_PACKET_SIZE;
    g_UsbDeviceVideoFlexioCamera.fullBufferIndex = 0U;
    g_UsbDeviceVideoFlexioCamera.imagePosition = 0U;

    g_UsbDeviceVideoFlexioCamera.probeStruct.bFormatIndex = USB_VIDEO_CAMERA_UNCOMPRESSED_FORMAT_INDEX;
    g_UsbDeviceVideoFlexioCamera.probeStruct.bFrameIndex = USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_INDEX;
    USB_LONG_TO_LITTLE_ENDIAN_DATA(USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_DEFAULT_INTERVAL,
                                   g_UsbDeviceVideoFlexioCamera.probeStruct.dwFrameInterval);
    USB_LONG_TO_LITTLE_ENDIAN_DATA(g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize,
                                   g_UsbDeviceVideoFlexioCamera.probeStruct.dwMaxPayloadTransferSize);
    USB_LONG_TO_LITTLE_ENDIAN_DATA(USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_MAX_FRAME_SIZE,
                                   g_UsbDeviceVideoFlexioCamera.probeStruct.dwMaxVideoFrameSize);

    g_UsbDeviceVideoFlexioCamera.commitStruct.bFormatIndex = USB_VIDEO_CAMERA_UNCOMPRESSED_FORMAT_INDEX;
    g_UsbDeviceVideoFlexioCamera.commitStruct.bFrameIndex = USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_INDEX;
    USB_LONG_TO_LITTLE_ENDIAN_DATA(USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_DEFAULT_INTERVAL,
                                   g_UsbDeviceVideoFlexioCamera.commitStruct.dwFrameInterval);
    USB_LONG_TO_LITTLE_ENDIAN_DATA(g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize,
                                   g_UsbDeviceVideoFlexioCamera.commitStruct.dwMaxPayloadTransferSize);
    USB_LONG_TO_LITTLE_ENDIAN_DATA(USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_MAX_FRAME_SIZE,
                                   g_UsbDeviceVideoFlexioCamera.commitStruct.dwMaxVideoFrameSize);

    g_UsbDeviceVideoFlexioCamera.probeInfo = 0x03U;
    g_UsbDeviceVideoFlexioCamera.probeLength = 26U;
    g_UsbDeviceVideoFlexioCamera.commitInfo = 0x03U;
    g_UsbDeviceVideoFlexioCamera.commitLength = 26U;

    g_UsbDeviceVideoFlexioCamera.stillProbeStruct.bFormatIndex = USB_VIDEO_CAMERA_UNCOMPRESSED_FORMAT_INDEX;
    g_UsbDeviceVideoFlexioCamera.stillProbeStruct.bFrameIndex = USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_INDEX;
    g_UsbDeviceVideoFlexioCamera.stillProbeStruct.bCompressionIndex = 0x01U;
    USB_LONG_TO_LITTLE_ENDIAN_DATA(g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize,
                                   g_UsbDeviceVideoFlexioCamera.stillProbeStruct.dwMaxPayloadTransferSize);
    USB_LONG_TO_LITTLE_ENDIAN_DATA(USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_MAX_FRAME_SIZE,
                                   g_UsbDeviceVideoFlexioCamera.stillProbeStruct.dwMaxVideoFrameSize);

    g_UsbDeviceVideoFlexioCamera.stillCommitStruct.bFormatIndex = USB_VIDEO_CAMERA_UNCOMPRESSED_FORMAT_INDEX;
    g_UsbDeviceVideoFlexioCamera.stillCommitStruct.bFrameIndex = USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_INDEX;
    g_UsbDeviceVideoFlexioCamera.stillCommitStruct.bCompressionIndex = 0x01U;
    USB_LONG_TO_LITTLE_ENDIAN_DATA(g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize,
                                   g_UsbDeviceVideoFlexioCamera.stillCommitStruct.dwMaxPayloadTransferSize);
    USB_LONG_TO_LITTLE_ENDIAN_DATA(USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_MAX_FRAME_SIZE,
                                   g_UsbDeviceVideoFlexioCamera.stillCommitStruct.dwMaxVideoFrameSize);

    g_UsbDeviceVideoFlexioCamera.stillProbeInfo = 0x03U;
    g_UsbDeviceVideoFlexioCamera.stillProbeLength = sizeof(g_UsbDeviceVideoFlexioCamera.stillProbeStruct);
    g_UsbDeviceVideoFlexioCamera.stillCommitInfo = 0x03U;
    g_UsbDeviceVideoFlexioCamera.stillCommitLength = sizeof(g_UsbDeviceVideoFlexioCamera.stillCommitStruct);

    g_UsbDeviceVideoFlexioCamera.currentTime = 0U;
    g_UsbDeviceVideoFlexioCamera.currentFrameId = 0U;
    g_UsbDeviceVideoFlexioCamera.currentStreamInterfaceAlternateSetting = 0U;
    g_UsbDeviceVideoFlexioCamera.imageBufferLength = 0U;
    g_UsbDeviceVideoFlexioCamera.imageIndex = 0U;
    g_UsbDeviceVideoFlexioCamera.waitForNewInterval = 0U;
    g_UsbDeviceVideoFlexioCamera.stillImageTransmission = 0U;
    g_UsbDeviceVideoFlexioCamera.stillImageTriggerControl = USB_DEVICE_VIDEO_STILL_IMAGE_TRIGGER_NORMAL_OPERATION;
}

/* Device callback */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Success;
    uint8_t *temp8 = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            USB_DeviceControlPipeInit(g_UsbDeviceVideoFlexioCamera.deviceHandle);
            USB_DeviceVideoApplicationSetDefault();
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success == USB_DeviceGetStatus(g_UsbDeviceVideoFlexioCamera.deviceHandle,
                                                           kUSB_DeviceStatusSpeed, &g_UsbDeviceVideoFlexioCamera.speed))
            {
                USB_DeviceSetSpeed(g_UsbDeviceVideoFlexioCamera.speed);
            }

            if (USB_SPEED_HIGH == g_UsbDeviceVideoFlexioCamera.speed)
            {
                g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize = HS_STREAM_IN_PACKET_SIZE;
            }
#endif
        }
        break;
        case kUSB_DeviceEventSetConfiguration:
            if (USB_VIDEO_CAMERA_CONFIGURE_INDEX == (*temp8))
            {
                g_UsbDeviceVideoFlexioCamera.attach = 1U;
            }
            break;
        case kUSB_DeviceEventSetInterface:
            if ((g_UsbDeviceVideoFlexioCamera.attach) && param)
            {
                uint8_t interface = (*temp8) & 0xFFU;
                uint8_t alternateSetting = g_UsbDeviceInterface[interface];

                if (g_UsbDeviceVideoFlexioCamera.currentStreamInterfaceAlternateSetting != alternateSetting)
                {
                    if (g_UsbDeviceVideoFlexioCamera.currentStreamInterfaceAlternateSetting)
                    {
                        USB_DeviceDeinitEndpoint(g_UsbDeviceVideoFlexioCamera.deviceHandle,
                                                 USB_VIDEO_CAMERA_STREAM_ENDPOINT_IN |
                                                     (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT));
                    }
                    else
                    {
                        usb_device_endpoint_init_struct_t epInitStruct;
                        usb_device_endpoint_callback_struct_t endpointCallback;

                        endpointCallback.callbackFn = USB_DeviceVideoIsoIn;
                        endpointCallback.callbackParam = handle;

                        epInitStruct.zlt = 0U;
                        epInitStruct.transferType = USB_ENDPOINT_ISOCHRONOUS;
                        epInitStruct.endpointAddress = USB_VIDEO_CAMERA_STREAM_ENDPOINT_IN |
                                                       (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
                        if (USB_SPEED_HIGH == g_UsbDeviceVideoFlexioCamera.speed)
                        {
                            epInitStruct.maxPacketSize = HS_STREAM_IN_PACKET_SIZE;
                        }
                        else
                        {
                            epInitStruct.maxPacketSize = FS_STREAM_IN_PACKET_SIZE;
                        }

                        USB_DeviceInitEndpoint(g_UsbDeviceVideoFlexioCamera.deviceHandle, &epInitStruct,
                                               &endpointCallback);
                        USB_DeviceVideoPrepareVideoData();
                        error = USB_DeviceSendRequest(
                            g_UsbDeviceVideoFlexioCamera.deviceHandle, USB_VIDEO_CAMERA_STREAM_ENDPOINT_IN,
                            g_UsbDeviceVideoFlexioCamera.imageBuffer, g_UsbDeviceVideoFlexioCamera.imageBufferLength);
                    }
                    g_UsbDeviceVideoFlexioCamera.currentStreamInterfaceAlternateSetting = alternateSetting;
                }
            }
            break;
        default:
            break;
    }

    return error;
}

/* Get the setup buffer */
usb_status_t USB_DeviceGetSetupBuffer(usb_device_handle handle, usb_setup_struct_t **setupBuffer)
{
    static uint32_t setup[2];
    if (NULL == setupBuffer)
    {
        return kStatus_USB_InvalidParameter;
    }
    *setupBuffer = (usb_setup_struct_t *)&setup;
    return kStatus_USB_Success;
}

/* Configure remote wakeup(Enable or disbale) */
usb_status_t USB_DeviceConfigureRemoteWakeup(usb_device_handle handle, uint8_t enable)
{
    return kStatus_USB_Success;
}
/* Configure endpoint status(Idle or stall) */
usb_status_t USB_DeviceConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status)
{
    if (status)
    {
        if ((USB_VIDEO_CAMERA_STREAM_ENDPOINT_IN == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
            (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceStallEndpoint(handle, ep);
        }
        else if ((USB_VIDEO_CAMERA_CONTROL_ENDPOINT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
                 (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceStallEndpoint(handle, ep);
        }
        else
        {
        }
    }
    else
    {
        if ((USB_VIDEO_CAMERA_STREAM_ENDPOINT_IN == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
            (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceUnstallEndpoint(handle, ep);
        }
        else if ((USB_VIDEO_CAMERA_CONTROL_ENDPOINT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
                 (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
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
    static uint32_t setupOut[(sizeof(usb_device_video_probe_and_commit_controls_struct_t) >> 2U) + 1U];
    if ((NULL == buffer) || ((*length) > sizeof(setupOut)))
    {
        return kStatus_USB_InvalidRequest;
    }
    *buffer = (uint8_t *)&setupOut[0];
    return kStatus_USB_Success;
}

/* Precess class-specific VS probe request */
static usb_status_t USB_DeviceVideoProcessClassVsProbeRequest(usb_device_handle handle,
                                                              usb_setup_struct_t *setup,
                                                              uint32_t *length,
                                                              uint8_t **buffer)
{
    usb_device_video_probe_and_commit_controls_struct_t *probe;
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint32_t temp32;

    if ((NULL == buffer) || (NULL == length))
    {
        return error;
    }

    probe = (usb_device_video_probe_and_commit_controls_struct_t *)(*buffer);

    switch (setup->bRequest)
    {
        case USB_DEVICE_VIDEO_REQUEST_CODE_SET_CUR:
            if ((*buffer == NULL) || (*length == 0U))
            {
                return error;
            }
            temp32 = USB_LONG_FROM_LITTLE_ENDIAN_DATA(probe->dwFrameInterval);
            if ((temp32 >= USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_MIN_INTERVAL) &&
                (temp32 <= USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_MAX_INTERVAL))
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(temp32, g_UsbDeviceVideoFlexioCamera.probeStruct.dwFrameInterval);
            }

            temp32 = USB_LONG_FROM_LITTLE_ENDIAN_DATA(probe->dwMaxPayloadTransferSize);
            if ((temp32) && (temp32 < g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize))
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(temp32,
                                               g_UsbDeviceVideoFlexioCamera.probeStruct.dwMaxPayloadTransferSize);
            }

            g_UsbDeviceVideoFlexioCamera.probeStruct.bFormatIndex = probe->bFormatIndex;
            g_UsbDeviceVideoFlexioCamera.probeStruct.bFrameIndex = probe->bFrameIndex;

            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_CUR:
            *buffer = (uint8_t *)&g_UsbDeviceVideoFlexioCamera.probeStruct;
            *length = g_UsbDeviceVideoFlexioCamera.probeLength;
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_MIN:
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_MAX:
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_RES:
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_LEN:
            *buffer = &g_UsbDeviceVideoFlexioCamera.probeLength;
            *length = sizeof(g_UsbDeviceVideoFlexioCamera.probeLength);
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_INFO:
            *buffer = &g_UsbDeviceVideoFlexioCamera.probeInfo;
            *length = sizeof(g_UsbDeviceVideoFlexioCamera.probeInfo);
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_DEF:
            break;
        default:
            break;
    }
    return error;
}

/* Precess class-specific VS commit request */
static usb_status_t USB_DeviceVideoProcessClassVsCommitRequest(usb_device_handle handle,
                                                               usb_setup_struct_t *setup,
                                                               uint32_t *length,
                                                               uint8_t **buffer)
{
    usb_device_video_probe_and_commit_controls_struct_t *commit;
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint32_t temp32;

    if ((NULL == buffer) || (NULL == length))
    {
        return error;
    }

    commit = (usb_device_video_probe_and_commit_controls_struct_t *)(*buffer);

    switch (setup->bRequest)
    {
        case USB_DEVICE_VIDEO_REQUEST_CODE_SET_CUR:
            if ((*buffer == NULL) || (*length == 0U))
            {
                return error;
            }

            temp32 = USB_LONG_FROM_LITTLE_ENDIAN_DATA(commit->dwFrameInterval);
            if ((temp32 >= USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_MIN_INTERVAL) &&
                (temp32 <= USB_VIDEO_CAMERA_UNCOMPRESSED_FRAME_MAX_INTERVAL))
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(temp32, g_UsbDeviceVideoFlexioCamera.commitStruct.dwFrameInterval);
            }

            temp32 = USB_LONG_FROM_LITTLE_ENDIAN_DATA(commit->dwMaxPayloadTransferSize);
            if ((temp32) && (temp32 < g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize))
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(temp32,
                                               g_UsbDeviceVideoFlexioCamera.commitStruct.dwMaxPayloadTransferSize);
            }

            g_UsbDeviceVideoFlexioCamera.commitStruct.bFormatIndex = commit->bFormatIndex;
            g_UsbDeviceVideoFlexioCamera.commitStruct.bFrameIndex = commit->bFrameIndex;
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_CUR:
            *buffer = (uint8_t *)&g_UsbDeviceVideoFlexioCamera.commitStruct;
            *length = g_UsbDeviceVideoFlexioCamera.commitLength;
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_LEN:
            *buffer = &g_UsbDeviceVideoFlexioCamera.commitLength;
            *length = sizeof(g_UsbDeviceVideoFlexioCamera.commitLength);
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_INFO:
            *buffer = &g_UsbDeviceVideoFlexioCamera.commitInfo;
            *length = sizeof(g_UsbDeviceVideoFlexioCamera.commitInfo);
            error = kStatus_USB_Success;
            break;
        default:
            break;
    }
    return error;
}

/* Precess class-specific VS STILL probe request */
static usb_status_t USB_DeviceVideoProcessClassVsStillProbeRequest(usb_device_handle handle,
                                                                   usb_setup_struct_t *setup,
                                                                   uint32_t *length,
                                                                   uint8_t **buffer)
{
    usb_device_video_still_probe_and_commit_controls_struct_t *still_probe;
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint32_t temp32;

    if ((NULL == buffer) || (NULL == length))
    {
        return error;
    }

    still_probe = (usb_device_video_still_probe_and_commit_controls_struct_t *)(*buffer);

    switch (setup->bRequest)
    {
        case USB_DEVICE_VIDEO_REQUEST_CODE_SET_CUR:
            if ((*buffer == NULL) || (*length == 0U))
            {
                return error;
            }

            temp32 = USB_LONG_FROM_LITTLE_ENDIAN_DATA(still_probe->dwMaxPayloadTransferSize);
            if ((temp32) && (temp32 < g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize))
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(temp32,
                                               g_UsbDeviceVideoFlexioCamera.stillProbeStruct.dwMaxPayloadTransferSize);
            }

            g_UsbDeviceVideoFlexioCamera.stillProbeStruct.bFormatIndex = still_probe->bFormatIndex;
            g_UsbDeviceVideoFlexioCamera.stillProbeStruct.bFrameIndex = still_probe->bFrameIndex;

            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_CUR:
            *buffer = (uint8_t *)&g_UsbDeviceVideoFlexioCamera.stillProbeStruct;
            *length = g_UsbDeviceVideoFlexioCamera.stillProbeLength;
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_MIN:
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_MAX:
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_RES:
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_LEN:
            *buffer = &g_UsbDeviceVideoFlexioCamera.stillProbeLength;
            *length = sizeof(g_UsbDeviceVideoFlexioCamera.stillProbeLength);
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_INFO:
            *buffer = &g_UsbDeviceVideoFlexioCamera.stillProbeInfo;
            *length = sizeof(g_UsbDeviceVideoFlexioCamera.stillProbeInfo);
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_DEF:
            break;
        default:
            break;
    }
    return error;
}

/* Precess class-specific VS STILL commit request */
static usb_status_t USB_DeviceVideoProcessClassVsStillCommitRequest(usb_device_handle handle,
                                                                    usb_setup_struct_t *setup,
                                                                    uint32_t *length,
                                                                    uint8_t **buffer)
{
    usb_device_video_still_probe_and_commit_controls_struct_t *still_commit;
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint32_t temp32;

    if ((NULL == buffer) || (NULL == length))
    {
        return error;
    }

    still_commit = (usb_device_video_still_probe_and_commit_controls_struct_t *)(*buffer);

    switch (setup->bRequest)
    {
        case USB_DEVICE_VIDEO_REQUEST_CODE_SET_CUR:
            if ((*buffer == NULL) || (*length == 0U))
            {
                return error;
            }

            temp32 = USB_LONG_FROM_LITTLE_ENDIAN_DATA(still_commit->dwMaxPayloadTransferSize);
            if ((temp32) && (temp32 < g_UsbDeviceVideoFlexioCamera.currentMaxPacketSize))
            {
                USB_LONG_TO_LITTLE_ENDIAN_DATA(temp32,
                                               g_UsbDeviceVideoFlexioCamera.stillCommitStruct.dwMaxPayloadTransferSize);
            }

            g_UsbDeviceVideoFlexioCamera.stillCommitStruct.bFormatIndex = still_commit->bFormatIndex;
            g_UsbDeviceVideoFlexioCamera.stillCommitStruct.bFrameIndex = still_commit->bFrameIndex;

            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_CUR:
            *buffer = (uint8_t *)&g_UsbDeviceVideoFlexioCamera.stillCommitStruct;
            *length = g_UsbDeviceVideoFlexioCamera.stillCommitLength;
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_LEN:
            *buffer = &g_UsbDeviceVideoFlexioCamera.stillCommitLength;
            *length = sizeof(g_UsbDeviceVideoFlexioCamera.stillCommitLength);
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_INFO:
            *buffer = &g_UsbDeviceVideoFlexioCamera.stillCommitInfo;
            *length = sizeof(g_UsbDeviceVideoFlexioCamera.stillCommitInfo);
            error = kStatus_USB_Success;
            break;
        default:
            break;
    }
    return error;
}

/* Precess class-specific VS STILL image trigger request */
static usb_status_t USB_DeviceVideoProcessClassVsStillImageTriggerRequest(usb_device_handle handle,
                                                                          usb_setup_struct_t *setup,
                                                                          uint32_t *length,
                                                                          uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    if ((NULL == buffer) || (NULL == length))
    {
        return error;
    }

    switch (setup->bRequest)
    {
        case USB_DEVICE_VIDEO_REQUEST_CODE_SET_CUR:
            if ((*buffer == NULL) || (*length == 0U))
            {
                return error;
            }
            g_UsbDeviceVideoFlexioCamera.stillImageTriggerControl = *(*buffer);
            error = kStatus_USB_Success;
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_CUR:
            break;
        case USB_DEVICE_VIDEO_REQUEST_CODE_GET_INFO:
            break;
        default:
            break;
    }
    return error;
}

/* Precess class-specific VS request */
static usb_status_t USB_DeviceVideoProcessClassVsRequest(usb_device_handle handle,
                                                         usb_setup_struct_t *setup,
                                                         uint32_t *length,
                                                         uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t cs = (setup->wValue >> 0x08U) & 0xFFU;

    switch (cs)
    {
        case USB_DEVICE_VIDEO_VS_PROBE_CONTROL:
            error = USB_DeviceVideoProcessClassVsProbeRequest(handle, setup, length, buffer);
            break;
        case USB_DEVICE_VIDEO_VS_COMMIT_CONTROL:
            error = USB_DeviceVideoProcessClassVsCommitRequest(handle, setup, length, buffer);
            break;
        case USB_DEVICE_VIDEO_VS_STILL_PROBE_CONTROL:
            error = USB_DeviceVideoProcessClassVsStillProbeRequest(handle, setup, length, buffer);
            break;
        case USB_DEVICE_VIDEO_VS_STILL_COMMIT_CONTROL:
            error = USB_DeviceVideoProcessClassVsStillCommitRequest(handle, setup, length, buffer);
            break;
        case USB_DEVICE_VIDEO_VS_STILL_IMAGE_TRIGGER_CONTROL:
            error = USB_DeviceVideoProcessClassVsStillImageTriggerRequest(handle, setup, length, buffer);
            break;
        case USB_DEVICE_VIDEO_VS_STREAM_ERROR_CODE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_VS_GENERATE_KEY_FRAME_CONTROL:
            break;
        case USB_DEVICE_VIDEO_VS_UPDATE_FRAME_SEGMENT_CONTROL:
            break;
        case USB_DEVICE_VIDEO_VS_SYNCH_DELAY_CONTROL:
            break;
        default:
            break;
    }
    return error;
}

/* Precess class-specific VC interface request */
static usb_status_t USB_DeviceProcessClassVcInterfaceRequest(usb_device_handle handle,
                                                             usb_setup_struct_t *setup,
                                                             uint32_t *length,
                                                             uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t cs = (setup->wValue >> 0x08U) & 0xFFU;

    if (USB_DEVICE_VIDEO_VC_VIDEO_POWER_MODE_CONTROL == cs)
    {
    }
    else if (USB_DEVICE_VIDEO_VC_REQUEST_ERROR_CODE_CONTROL == cs)
    {
    }
    else
    {
    }
    return error;
}

/* Precess class-specific VC camera terminal request */
static usb_status_t USB_DeviceProcessClassVcCameraTerminalRequest(usb_device_handle handle,
                                                                  usb_setup_struct_t *setup,
                                                                  uint32_t *length,
                                                                  uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t cs = (setup->wValue >> 0x08U) & 0xFFU;

    switch (cs)
    {
        case USB_DEVICE_VIDEO_CT_SCANNING_MODE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_AE_MODE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_AE_PRIORITY_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_EXPOSURE_TIME_ABSOLUTE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_EXPOSURE_TIME_RELATIVE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_FOCUS_ABSOLUTE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_FOCUS_RELATIVE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_FOCUS_AUTO_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_IRIS_ABSOLUTE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_IRIS_RELATIVE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_ZOOM_ABSOLUTE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_ZOOM_RELATIVE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_PANTILT_ABSOLUTE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_PANTILT_RELATIVE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_ROLL_ABSOLUTE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_ROLL_RELATIVE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_PRIVACY_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_FOCUS_SIMPLE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_WINDOW_CONTROL:
            break;
        case USB_DEVICE_VIDEO_CT_REGION_OF_INTEREST_CONTROL:
            break;
        default:
            break;
    }
    return error;
}

/* Precess class-specific VC input terminal request */
static usb_status_t USB_DeviceProcessClassVcInputTerminalRequest(usb_device_handle handle,
                                                                 usb_setup_struct_t *setup,
                                                                 uint32_t *length,
                                                                 uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    if (USB_DEVICE_VIDEO_ITT_CAMERA == USB_VIDEO_CAMERA_VC_INPUT_TERMINAL_TYPE)
    {
        error = USB_DeviceProcessClassVcCameraTerminalRequest(handle, setup, length, buffer);
    }

    return error;
}

/* Precess class-specific VC onput terminal request */
static usb_status_t USB_DeviceProcessClassVcOutputTerminalRequest(usb_device_handle handle,
                                                                  usb_setup_struct_t *setup,
                                                                  uint32_t *length,
                                                                  uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    return error;
}

/* Precess class-specific VC processing unit request */
static usb_status_t USB_DeviceProcessClassVcProcessingUintRequest(usb_device_handle handle,
                                                                  usb_setup_struct_t *setup,
                                                                  uint32_t *length,
                                                                  uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t cs = (setup->wValue >> 0x08U) & 0xFFU;

    switch (cs)
    {
        case USB_DEVICE_VIDEO_PU_BACKLIGHT_COMPENSATION_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_BRIGHTNESS_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_CONTRAST_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_GAIN_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_POWER_LINE_FREQUENCY_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_HUE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_SATURATION_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_SHARPNESS_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_GAMMA_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_WHITE_BALANCE_TEMPERATURE_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_WHITE_BALANCE_COMPONENT_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_DIGITAL_MULTIPLIER_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_HUE_AUTO_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_ANALOG_VIDEO_STANDARD_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_ANALOG_LOCK_STATUS_CONTROL:
            break;
        case USB_DEVICE_VIDEO_PU_CONTRAST_AUTO_CONTROL:
            break;
        default:
            break;
    }

    return error;
}

/* Precess class-specific VC request */
static usb_status_t USB_DeviceProcessClassVcRequest(usb_device_handle handle,
                                                    usb_setup_struct_t *setup,
                                                    uint32_t *length,
                                                    uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t entityId = (uint8_t)(setup->wIndex >> 0x08U);
    if (0U == entityId)
    {
        error = USB_DeviceProcessClassVcInterfaceRequest(handle, setup, length, buffer);
    }
    else if (USB_VIDEO_CAMERA_VC_INPUT_TERMINAL_ID == entityId)
    {
        error = USB_DeviceProcessClassVcInputTerminalRequest(handle, setup, length, buffer);
    }
    else if (USB_VIDEO_CAMERA_VC_OUTPUT_TERMINAL_ID == entityId)
    {
        error = USB_DeviceProcessClassVcOutputTerminalRequest(handle, setup, length, buffer);
    }
    else if (USB_VIDEO_CAMERA_VC_PROCESSING_UNIT_ID == entityId)
    {
        error = USB_DeviceProcessClassVcProcessingUintRequest(handle, setup, length, buffer);
    }
    else
    {
    }
    return error;
}

/* Precess class-specific request */
usb_status_t USB_DeviceProcessClassRequest(usb_device_handle handle,
                                           usb_setup_struct_t *setup,
                                           uint32_t *length,
                                           uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t interface_index = (uint8_t)setup->wIndex;

    switch (setup->bmRequestType)
    {
        case USB_DEVICE_VIDEO_GET_REQUEST_INTERFACE:
            if (USB_VIDEO_CAMERA_CONTROL_INTERFACE_INDEX == interface_index)
            {
                error = USB_DeviceProcessClassVcRequest(handle, setup, length, buffer);
            }
            else if (USB_VIDEO_CAMERA_STREAM_INTERFACE_INDEX == interface_index)
            {
                error = USB_DeviceVideoProcessClassVsRequest(handle, setup, length, buffer);
            }
            else
            {
            }
            break;
        case USB_DEVICE_VIDEO_SET_REQUEST_INTERFACE:
            if (USB_VIDEO_CAMERA_CONTROL_INTERFACE_INDEX == interface_index)
            {
                error = USB_DeviceProcessClassVcRequest(handle, setup, length, buffer);
            }
            else if (USB_VIDEO_CAMERA_STREAM_INTERFACE_INDEX == interface_index)
            {
                error = USB_DeviceVideoProcessClassVsRequest(handle, setup, length, buffer);
            }
            else
            {
            }
            break;
        default:
            break;
    }

    return error;
}

static void USB_DeviceApplicationInit(void)
{
    const camera_config_t cameraConfig = {
        .pixelFormat = kVIDEO_PixelFormatRGB565,
        .bytesPerPixel = 2,
        .resolution = kVIDEO_ResolutionQQVGA,
        .framePerSec = 25,
    };

    USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
    SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

    USB_DeviceVideoApplicationSetDefault();

    if (kStatus_USB_Success !=
        USB_DeviceInit(CONTROLLER_ID, USB_DeviceCallback, &g_UsbDeviceVideoFlexioCamera.deviceHandle))
    {
        usb_echo("USB device video camera failed\r\n");
        return;
    }
    else
    {
        usb_echo("USB device video camera demo\r\n");
    }

    USB_DeviceIsrEnable();

    CAMERA_DEVICE_Init(&cameraDevice, &cameraConfig);

    CAMERA_DEVICE_Start(&cameraDevice);

    CAMERA_RECEIVER_Init(&cameraReceiver, &cameraConfig, NULL, NULL);

    CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)&g_FlexioCameraFrameBuffer[1][32]);

    CAMERA_RECEIVER_Start(&cameraReceiver);

    USB_DeviceRun(g_UsbDeviceVideoFlexioCamera.deviceHandle);

    USB_DeviceRun(g_UsbDeviceVideoFlexioCamera.deviceHandle);
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
        USB_DeviceTaskFn(g_UsbDeviceVideoFlexioCamera.deviceHandle);
#endif
    }
}
