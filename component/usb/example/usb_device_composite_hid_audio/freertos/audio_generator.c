/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
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

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_audio.h"
#include "usb_audio_config.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "composite.h"

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"

#include <stdio.h>
#include <stdlib.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void USB_PrepareData(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/

extern uint8_t s_wavBuff[];

static usb_device_composite_struct_t *g_deviceComposite;

/*******************************************************************************
* Code
******************************************************************************/
/*!
 * @brief Audio class specific request function.
 *
 * This function handles the Audio class specific requests.
 *
 * @param handle           The USB device handle.
 * @param event            The USB device event type.
 * @param param            The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioRequest(class_handle_t handle, uint32_t event, void *param)
{
    usb_device_control_request_struct_t *request = (usb_device_control_request_struct_t *)param;
    usb_status_t error = kStatus_USB_Success;
    uint16_t volume;

    switch (event)
    {
        case USB_DEVICE_AUDIO_GET_CUR_MUTE_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.curMute;
            request->length = sizeof(g_deviceComposite->audioGenerator.curMute);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_VOLUME_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.curVolume;
            request->length = sizeof(g_deviceComposite->audioGenerator.curVolume);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_BASS_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.curBass;
            request->length = sizeof(g_deviceComposite->audioGenerator.curBass);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_MID_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.curMid;
            request->length = sizeof(g_deviceComposite->audioGenerator.curMid);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_TREBLE_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.curTreble;
            request->length = sizeof(g_deviceComposite->audioGenerator.curTreble);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_AUTOMATIC_GAIN_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.curAutomaticGain;
            request->length = sizeof(g_deviceComposite->audioGenerator.curAutomaticGain);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_DELAY_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.curDelay;
            request->length = sizeof(g_deviceComposite->audioGenerator.curDelay);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_SAMPLING_FREQ_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.curSamplingFrequency;
            request->length = sizeof(g_deviceComposite->audioGenerator.curSamplingFrequency);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_VOLUME_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.minVolume;
            request->length = sizeof(g_deviceComposite->audioGenerator.minVolume);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_BASS_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.minBass;
            request->length = sizeof(g_deviceComposite->audioGenerator.minBass);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_MID_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.minMid;
            request->length = sizeof(g_deviceComposite->audioGenerator.minMid);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_TREBLE_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.minTreble;
            request->length = sizeof(g_deviceComposite->audioGenerator.minTreble);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_DELAY_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.minDelay;
            request->length = sizeof(g_deviceComposite->audioGenerator.minDelay);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_SAMPLING_FREQ_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.minSamplingFrequency;
            request->length = sizeof(g_deviceComposite->audioGenerator.minSamplingFrequency);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_VOLUME_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.maxVolume;
            request->length = sizeof(g_deviceComposite->audioGenerator.maxVolume);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_BASS_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.maxBass;
            request->length = sizeof(g_deviceComposite->audioGenerator.maxBass);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_MID_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.maxMid;
            request->length = sizeof(g_deviceComposite->audioGenerator.maxMid);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_TREBLE_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.maxTreble;
            request->length = sizeof(g_deviceComposite->audioGenerator.maxTreble);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_DELAY_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.maxDelay;
            request->length = sizeof(g_deviceComposite->audioGenerator.maxDelay);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_SAMPLING_FREQ_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.maxSamplingFrequency;
            request->length = sizeof(g_deviceComposite->audioGenerator.maxSamplingFrequency);
            break;
        case USB_DEVICE_AUDIO_GET_RES_VOLUME_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.resVolume;
            request->length = sizeof(g_deviceComposite->audioGenerator.resVolume);
            break;
        case USB_DEVICE_AUDIO_GET_RES_BASS_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.resBass;
            request->length = sizeof(g_deviceComposite->audioGenerator.resBass);
            break;
        case USB_DEVICE_AUDIO_GET_RES_MID_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.resMid;
            request->length = sizeof(g_deviceComposite->audioGenerator.resMid);
            break;
        case USB_DEVICE_AUDIO_GET_RES_TREBLE_CONTROL:
            request->buffer = &g_deviceComposite->audioGenerator.resTreble;
            request->length = sizeof(g_deviceComposite->audioGenerator.resTreble);
            break;
        case USB_DEVICE_AUDIO_GET_RES_DELAY_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.resDelay;
            request->length = sizeof(g_deviceComposite->audioGenerator.resDelay);
            break;
        case USB_DEVICE_AUDIO_GET_RES_SAMPLING_FREQ_CONTROL:
            request->buffer = g_deviceComposite->audioGenerator.resSamplingFrequency;
            request->length = sizeof(g_deviceComposite->audioGenerator.resSamplingFrequency);
            break;

        case USB_DEVICE_AUDIO_SET_CUR_VOLUME_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.curVolume;
            }
            else
            {
                volume = (uint16_t)((uint16_t)g_deviceComposite->audioGenerator.curVolume[1] << 8U);
                volume |= (uint8_t)(g_deviceComposite->audioGenerator.curVolume[0]);
                usb_echo("Set Cur Volume : %x\r\n", volume);
            }
            break;
        case USB_DEVICE_AUDIO_SET_CUR_MUTE_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.curMute;
            }
            else
            {
                usb_echo("Set Cur Mute : %x\r\n", g_deviceComposite->audioGenerator.curMute);
            }
            break;
        case USB_DEVICE_AUDIO_SET_CUR_BASS_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.curBass;
            }
            break;
        case USB_DEVICE_AUDIO_SET_CUR_MID_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.curMid;
            }
            break;
        case USB_DEVICE_AUDIO_SET_CUR_TREBLE_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.curTreble;
            }
            break;
        case USB_DEVICE_AUDIO_SET_CUR_AUTOMATIC_GAIN_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.curAutomaticGain;
            }
            break;
        case USB_DEVICE_AUDIO_SET_CUR_DELAY_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.curDelay;
            }
            break;
        case USB_DEVICE_AUDIO_SET_CUR_SAMPLING_FREQ_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.curSamplingFrequency;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MIN_VOLUME_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.minVolume;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MIN_BASS_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.minBass;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MIN_MID_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.minMid;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MIN_TREBLE_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.minTreble;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MIN_DELAY_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.minDelay;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MIN_SAMPLING_FREQ_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.minSamplingFrequency;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MAX_VOLUME_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.maxVolume;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MAX_BASS_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.maxBass;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MAX_MID_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.maxMid;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MAX_TREBLE_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.maxTreble;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MAX_DELAY_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.maxDelay;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MAX_SAMPLING_FREQ_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.maxSamplingFrequency;
            }
            break;
        case USB_DEVICE_AUDIO_SET_RES_VOLUME_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.resVolume;
            }
            break;
        case USB_DEVICE_AUDIO_SET_RES_BASS_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.resBass;
            }
            break;
        case USB_DEVICE_AUDIO_SET_RES_MID_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.resMid;
            }
            break;
        case USB_DEVICE_AUDIO_SET_RES_TREBLE_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = &g_deviceComposite->audioGenerator.resTreble;
            }
            break;
        case USB_DEVICE_AUDIO_SET_RES_DELAY_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.resDelay;
            }
            break;
        case USB_DEVICE_AUDIO_SET_RES_SAMPLING_FREQ_CONTROL:
            if (request->isSetup == 1U)
            {
                request->buffer = g_deviceComposite->audioGenerator.resSamplingFrequency;
            }
            break;
        default:
            error = kStatus_USB_InvalidRequest;
            break;
    }
    return error;
}

/*!
 * @brief device Audio callback function.
 *
 * This function handle the Audio class specified event.
 * @param handle          The USB class  handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the class specific event.
 * @return kStatus_USB_Success or error.
 */
usb_status_t USB_DeviceAudioGeneratorCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    usb_device_endpoint_callback_message_struct_t *ep_cb_param;
    ep_cb_param = (usb_device_endpoint_callback_message_struct_t *)param;

    switch (event)
    {
        case kUSB_DeviceAudioEventStreamSendResponse:
            if ((g_deviceComposite->audioGenerator.attach) &&
                (ep_cb_param->length == ((USB_SPEED_HIGH == g_deviceComposite->audioGenerator.speed) ?
                                             HS_ISO_IN_ENDP_PACKET_SIZE :
                                             FS_ISO_IN_ENDP_PACKET_SIZE)))
            {
                USB_PrepareData();
                error = USB_DeviceAudioSend(handle, USB_AUDIO_STREAM_ENDPOINT, s_wavBuff,
                                            (USB_SPEED_HIGH == g_deviceComposite->audioGenerator.speed) ?
                                                HS_ISO_IN_ENDP_PACKET_SIZE :
                                                FS_ISO_IN_ENDP_PACKET_SIZE);
            }
            break;

        default:
            if (param && (event > 0xFF))
            {
                error = USB_DeviceAudioRequest(handle, event, param);
            }
            break;
    }

    return error;
}
/*!
 * @brief Audio set configuration function.
 *
 * This function sets configuration for msc class.
 *
 * @param handle The Audio class handle.
 * @param configure The Audio class configure index.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioGeneratorSetConfigure(class_handle_t handle, uint8_t configure)
{
    if (USB_COMPOSITE_CONFIGURE_INDEX == configure)
    {
        g_deviceComposite->audioGenerator.attach = 1U;
    }
    return kStatus_USB_Success;
}

usb_status_t USB_DeviceAudioGeneratorSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting)
{
    if (alternateSetting == 1U)
    {
        USB_PrepareData();
        USB_DeviceAudioSend(g_deviceComposite->audioGenerator.audioHandle, USB_AUDIO_STREAM_ENDPOINT, s_wavBuff,
                            (USB_SPEED_HIGH == g_deviceComposite->audioGenerator.speed) ? HS_ISO_IN_ENDP_PACKET_SIZE :
                                                                                          FS_ISO_IN_ENDP_PACKET_SIZE);
    }
    return kStatus_USB_Error;
}
/*!
 * @brief Audio init function.
 *
 * This function initializes the device with the composite device class information.
 *
 * @param device_composite          The pointer to the composite device structure.
 * @return kStatus_USB_Success .
 */
usb_status_t USB_DeviceAudioGeneratorInit(usb_device_composite_struct_t *device_composite)
{
    g_deviceComposite = device_composite;
    g_deviceComposite->audioGenerator.copyProtect = 0x01U;
    g_deviceComposite->audioGenerator.curMute = 0x01U;
    g_deviceComposite->audioGenerator.curVolume[0] = 0x00U;
    g_deviceComposite->audioGenerator.curVolume[1] = 0x80U;
    g_deviceComposite->audioGenerator.minVolume[0] = 0x00U;
    g_deviceComposite->audioGenerator.minVolume[1] = 0x80U;
    g_deviceComposite->audioGenerator.maxVolume[0] = 0xFFU;
    g_deviceComposite->audioGenerator.maxVolume[1] = 0X7FU;
    g_deviceComposite->audioGenerator.resVolume[0] = 0x01U;
    g_deviceComposite->audioGenerator.resVolume[1] = 0x00U;
    g_deviceComposite->audioGenerator.curBass = 0x00U;
    g_deviceComposite->audioGenerator.curBass = 0x00U;
    g_deviceComposite->audioGenerator.minBass = 0x80U;
    g_deviceComposite->audioGenerator.maxBass = 0x7FU;
    g_deviceComposite->audioGenerator.resBass = 0x01U;
    g_deviceComposite->audioGenerator.curMid = 0x00U;
    g_deviceComposite->audioGenerator.minMid = 0x80U;
    g_deviceComposite->audioGenerator.maxMid = 0x7FU;
    g_deviceComposite->audioGenerator.resMid = 0x01U;
    g_deviceComposite->audioGenerator.curTreble = 0x01U;
    g_deviceComposite->audioGenerator.minTreble = 0x80U;
    g_deviceComposite->audioGenerator.maxTreble = 0x7FU;
    g_deviceComposite->audioGenerator.resTreble = 0x01U;
    g_deviceComposite->audioGenerator.curAutomaticGain = 0x01U;
    g_deviceComposite->audioGenerator.curDelay[0] = 0x00U;
    g_deviceComposite->audioGenerator.curDelay[1] = 0x40U;
    g_deviceComposite->audioGenerator.minDelay[0] = 0x00U;
    g_deviceComposite->audioGenerator.minDelay[1] = 0x00U;
    g_deviceComposite->audioGenerator.maxDelay[0] = 0xFFU;
    g_deviceComposite->audioGenerator.maxDelay[1] = 0xFFU;
    g_deviceComposite->audioGenerator.resDelay[0] = 0x00U;
    g_deviceComposite->audioGenerator.resDelay[1] = 0x01U;
    g_deviceComposite->audioGenerator.curLoudness = 0x01U;
    g_deviceComposite->audioGenerator.curSamplingFrequency[0] = 0x00U;
    g_deviceComposite->audioGenerator.curSamplingFrequency[1] = 0x00U;
    g_deviceComposite->audioGenerator.curSamplingFrequency[2] = 0x01U;
    g_deviceComposite->audioGenerator.minSamplingFrequency[0] = 0x00U;
    g_deviceComposite->audioGenerator.minSamplingFrequency[1] = 0x00U;
    g_deviceComposite->audioGenerator.minSamplingFrequency[2] = 0x01U;
    g_deviceComposite->audioGenerator.maxSamplingFrequency[0] = 0x00U;
    g_deviceComposite->audioGenerator.maxSamplingFrequency[1] = 0x00U;
    g_deviceComposite->audioGenerator.maxSamplingFrequency[2] = 0x01U;
    g_deviceComposite->audioGenerator.resSamplingFrequency[0] = 0x00U;
    g_deviceComposite->audioGenerator.resSamplingFrequency[1] = 0x00U;
    g_deviceComposite->audioGenerator.resSamplingFrequency[2] = 0x01U;
    return kStatus_USB_Success;
}
