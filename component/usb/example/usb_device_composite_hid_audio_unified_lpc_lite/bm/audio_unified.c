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

#include "usb_device_audio.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "composite.h"

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include <stdio.h>
#include <stdlib.h>
#include "fsl_sctimer.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define USB_AUDIO_ENTER_CRITICAL() \
    \
USB_OSA_SR_ALLOC();                \
    \
USB_OSA_ENTER_CRITICAL()

#define USB_AUDIO_EXIT_CRITICAL() USB_OSA_EXIT_CRITICAL()
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
extern void BOARD_USB_AUDIO_KEYBOARD_Init(void);
extern void BOARD_USB_Audio_TxRxInit(uint32_t samplingRate);
extern void BOARD_I2C_LPI2C_Init(void);
extern void BOARD_Codec_Init(void);
extern void BOARD_SetCodecMuteUnmute(bool);

extern void BOARD_DMA_EDMA_Config(void);
extern void BOARD_DMA_EDMA_Set_AudioFormat(void);
extern void BOARD_DMA_EDMA_Enable_Audio_Interrupts(void);
extern void BOARD_DMA_EDMA_Start(void);
extern void audio_trim_up(void);
extern void audio_trim_down(void);
/*******************************************************************************
 * Variables
 ******************************************************************************/
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t audioPlayDataBuff[AUDIO_SPEAKER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_OUT_ENDP_PACKET_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t audioPlayPacket[FS_ISO_OUT_ENDP_PACKET_SIZE];

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t audioRecDataBuff[AUDIO_RECORDER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_IN_ENDP_PACKET_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t audioRecPacket[(FS_ISO_IN_ENDP_PACKET_SIZE)];

#if (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
static uint32_t eventCounterL = 0;
static uint32_t captureRegisterNumber;
static sctimer_config_t sctimerInfo;
#endif

volatile bool g_CodecMuteUnmute = false;

uint8_t g_InterfaceIsSet = 0;

static usb_device_composite_struct_t *g_deviceComposite;

usb_status_t USB_DeviceAudioProcessTerminalRequest(uint32_t audioCommand, uint32_t *length, uint8_t **buffer);

/*******************************************************************************
* Code
******************************************************************************/
/* Initialize the structure information for sai. */
void Init_Board_Sai_Codec(void)
{
    usb_echo("Init Audio SAI and CODEC\r\n");

    BOARD_USB_AUDIO_KEYBOARD_Init();

    BOARD_USB_Audio_TxRxInit(AUDIO_SAMPLING_RATE);
    BOARD_I2C_LPI2C_Init();
    BOARD_Codec_Init();
    BOARD_DMA_EDMA_Config();
    BOARD_DMA_EDMA_Start();
}
/* The AudioSpeakerBufferSpaceUsed() function gets the reserved speaker ringbuffer size */
uint32_t AudioSpeakerBufferSpaceUsed(void)
{
    if (g_deviceComposite->audioUnified.tdReadNumberPlay > g_deviceComposite->audioUnified.tdWriteNumberPlay)
    {
        g_deviceComposite->audioUnified.speakerReservedSpace =
            g_deviceComposite->audioUnified.tdReadNumberPlay - g_deviceComposite->audioUnified.tdWriteNumberPlay;
    }
    else
    {
        g_deviceComposite->audioUnified.speakerReservedSpace =
            g_deviceComposite->audioUnified.tdReadNumberPlay +
            AUDIO_SPEAKER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_OUT_ENDP_PACKET_SIZE -
            g_deviceComposite->audioUnified.tdWriteNumberPlay;
    }
    return g_deviceComposite->audioUnified.speakerReservedSpace;
}

/* The USB_AudioRecorderBufferSpaceAvailable() function gets the reserved recorder ringbuffer size */
uint32_t USB_AudioRecorderBufferSpaceAvailable(void)
{
    if (g_deviceComposite->audioUnified.tdReadNumberRec > g_deviceComposite->audioUnified.tdWriteNumberRec)
    {
        g_deviceComposite->audioUnified.recorderReservedSpace =
            g_deviceComposite->audioUnified.tdReadNumberRec - g_deviceComposite->audioUnified.tdWriteNumberRec;
    }
    else
    {
        g_deviceComposite->audioUnified.recorderReservedSpace =
            g_deviceComposite->audioUnified.tdReadNumberRec +
            AUDIO_RECORDER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_IN_ENDP_PACKET_SIZE -
            g_deviceComposite->audioUnified.tdWriteNumberRec;
    }
    return g_deviceComposite->audioUnified.recorderReservedSpace;
}

/* The USB_RecorderDataMatch() function increase/decrease the adjusted packet interval according to the reserved
 * ringbuffer size */
uint32_t USB_RecorderDataMatch(uint32_t reservedspace)
{
    uint32_t epPacketSize = 0;
    if (reservedspace >= AUDIO_BUFFER_UPPER_LIMIT(AUDIO_RECORDER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_IN_ENDP_PACKET_SIZE))
    {
        epPacketSize = FS_ISO_IN_ENDP_PACKET_SIZE + AUDIO_FORMAT_SIZE * AUDIO_FORMAT_CHANNELS;
    }
    else if ((reservedspace >=
              AUDIO_BUFFER_LOWER_LIMIT(AUDIO_RECORDER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_IN_ENDP_PACKET_SIZE)) &&
             (reservedspace <
              AUDIO_BUFFER_UPPER_LIMIT(AUDIO_RECORDER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_IN_ENDP_PACKET_SIZE)))
    {
        epPacketSize = FS_ISO_IN_ENDP_PACKET_SIZE;
    }
    else if (reservedspace <
             AUDIO_BUFFER_LOWER_LIMIT(AUDIO_RECORDER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_IN_ENDP_PACKET_SIZE))
    {
        epPacketSize = FS_ISO_IN_ENDP_PACKET_SIZE - AUDIO_FORMAT_SIZE * AUDIO_FORMAT_CHANNELS;
    }
    else
    {
    }
    return epPacketSize;
}

/* The USB_AudioRecorderGetBuffer() function gets audioRecPacket from the audioRecDataBuff in every callback*/
void USB_AudioRecorderGetBuffer(uint8_t *buffer, uint32_t size)
{
    while (size)
    {
        *buffer = audioRecDataBuff[g_deviceComposite->audioUnified.tdWriteNumberRec];
        g_deviceComposite->audioUnified.tdWriteNumberRec++;
        buffer++;
        size--;

        if (g_deviceComposite->audioUnified.tdWriteNumberRec >=
            AUDIO_RECORDER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_IN_ENDP_PACKET_SIZE)
        {
            g_deviceComposite->audioUnified.tdWriteNumberRec = 0;
        }
    }
}

/* The USB_AudioSpeakerPutBuffer() function fills the audioRecDataBuff with audioPlayPacket in every callback*/
void USB_AudioSpeakerPutBuffer(uint8_t *buffer, uint32_t size)
{
    while (size)
    {
        audioPlayDataBuff[g_deviceComposite->audioUnified.tdReadNumberPlay] = *buffer;
        g_deviceComposite->audioUnified.tdReadNumberPlay++;
        buffer++;
        size--;

        if (g_deviceComposite->audioUnified.tdReadNumberPlay >=
            AUDIO_SPEAKER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_OUT_ENDP_PACKET_SIZE)
        {
            g_deviceComposite->audioUnified.tdReadNumberPlay = 0;
        }
    }
}

#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
void USB_AudioFSSync()
{
    uint32_t usedSpace = 0;

    if (g_deviceComposite->audioUnified.speakerIntervalCount != AUDIO_CALCULATE_Ff_INTERVAL)
    {
        g_deviceComposite->audioUnified.speakerIntervalCount++;
        return;
    }
    g_deviceComposite->audioUnified.speakerIntervalCount = 1;

    usedSpace = AudioSpeakerBufferSpaceUsed();
    if (usedSpace >= AUDIO_BUFFER_UPPER_LIMIT(AUDIO_SPEAKER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_OUT_ENDP_PACKET_SIZE))
    {
        audio_trim_up();
    }
    else if (usedSpace <
             AUDIO_BUFFER_LOWER_LIMIT(AUDIO_SPEAKER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_OUT_ENDP_PACKET_SIZE))
    {
        audio_trim_down();
    }
    else
    {
    }
}
#endif
/*!
 * @brief Audio wav data prepare function.
 *
 * This function prepare audio wav data before send.
 */
/* USB device audio ISO OUT endpoint callback */
usb_status_t USB_DeviceAudioIsoIN(usb_device_handle deviceHandle,
                                  usb_device_endpoint_callback_message_struct_t *event,
                                  void *arg)
{
    usb_status_t error = kStatus_USB_Error;
    usb_device_endpoint_callback_message_struct_t *ep_cb_param;
    ep_cb_param = (usb_device_endpoint_callback_message_struct_t *)event;
    if ((g_deviceComposite->audioUnified.attach) && (ep_cb_param->length != (USB_UNINITIALIZED_VAL_32)))
    {
        if (g_deviceComposite->audioUnified.startRec == 0)
        {
            g_deviceComposite->audioUnified.startRec = 1;
        }
        if ((g_deviceComposite->audioUnified.tdReadNumberRec >=
             AUDIO_RECORDER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_IN_ENDP_PACKET_SIZE / 2) &&
            (g_deviceComposite->audioUnified.startRecHalfFull == 0))
        {
            g_deviceComposite->audioUnified.startRecHalfFull = 1;
        }
        if (g_deviceComposite->audioUnified.startRecHalfFull)
        {

            USB_AudioRecorderGetBuffer(audioRecPacket, FS_ISO_IN_ENDP_PACKET_SIZE);

            error = USB_DeviceSendRequest(deviceHandle, USB_AUDIO_RECORDER_STREAM_ENDPOINT, &audioRecPacket[0],
                                          FS_ISO_IN_ENDP_PACKET_SIZE);

            g_deviceComposite->audioUnified.usbSendTimes++;
        }
        else
        {
            error = USB_DeviceSendRequest(deviceHandle, USB_AUDIO_RECORDER_STREAM_ENDPOINT, &audioRecDataBuff[0],
                                          FS_ISO_IN_ENDP_PACKET_SIZE);
        }
    }

    return error;
}

/*!
 * @brief Audio wav data prepare function.
 *
 * This function prepare audio wav data before send.
 */
/* USB device audio ISO OUT endpoint callback */
usb_status_t USB_DeviceAudioIsoOUT(usb_device_handle deviceHandle,
                                   usb_device_endpoint_callback_message_struct_t *event,
                                   void *arg)
{
    usb_status_t error = kStatus_USB_Error;
    usb_device_endpoint_callback_message_struct_t *ep_cb_param;
    ep_cb_param = (usb_device_endpoint_callback_message_struct_t *)event;
    if ((g_deviceComposite->audioUnified.attach) && (ep_cb_param->length != (USB_UNINITIALIZED_VAL_32)))
    {
        if (g_deviceComposite->audioUnified.startPlay == 0)
        {
            g_deviceComposite->audioUnified.startPlay = 1;
        }
        if ((g_deviceComposite->audioUnified.tdReadNumberPlay >=
             AUDIO_SPEAKER_DATA_WHOLE_BUFFER_LENGTH * FS_ISO_OUT_ENDP_PACKET_SIZE / 2) &&
            (g_deviceComposite->audioUnified.startPlayHalfFull == 0))
        {
            g_deviceComposite->audioUnified.startPlayHalfFull = 1;
        }
        USB_AudioSpeakerPutBuffer(audioPlayPacket, ep_cb_param->length);
        g_deviceComposite->audioUnified.usbRecvCount += ep_cb_param->length;
        g_deviceComposite->audioUnified.usbRecvTimes++;
#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
        USB_AUDIO_ENTER_CRITICAL();
        USB_AudioFSSync();
        USB_AUDIO_EXIT_CRITICAL();
#endif
        error = USB_DeviceRecvRequest(deviceHandle, USB_AUDIO_SPEAKER_STREAM_ENDPOINT, &audioPlayPacket[0],
                                      (FS_ISO_OUT_ENDP_PACKET_SIZE));
    }
    return error;
}
usb_status_t USB_DeviceAudioGetControlTerminal(usb_device_handle handle,
                                               usb_setup_struct_t *setup,
                                               uint32_t *length,
                                               uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint32_t audioCommand = 0U;

    switch (setup->bRequest)
    {
        /* Copy Protect Control only supports the CUR attribute!*/
        case USB_DEVICE_AUDIO_GET_CUR_VOLUME_REQUEST:

            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);
    return error;
}

usb_status_t USB_DeviceAudioGetCurAudioFeatureUnit(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint32_t *length,
                                                   uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;
    /* Select SET request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_MUTE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_MUTE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_AUTOMATIC_GAIN_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_AUTOMATIC_GAIN_CONTROL;
            break;
        case USB_DEVICE_AUDIO_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_DELAY_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_BOOST_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_BASS_BOOST_CONTROL;
            break;
        case USB_DEVICE_AUDIO_LOUDNESS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_CUR_LOUDNESS_CONTROL;
            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);

    return error;
}

usb_status_t USB_DeviceAudioGetMinAudioFeatureUnit(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint32_t *length,
                                                   uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;

    /* Select SET request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MIN_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MIN_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MIN_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MIN_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MIN_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MIN_DELAY_CONTROL;
            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);
    return error;
}

usb_status_t USB_DeviceAudioGetMaxAudioFeatureUnit(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint32_t *length,
                                                   uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;

    /* Select SET request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MAX_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MAX_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MAX_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MAX_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MAX_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_MAX_DELAY_CONTROL;
            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);
    return error;
}

usb_status_t USB_DeviceAudioGetResAudioFeatureUnit(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint32_t *length,
                                                   uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;
    /* Select SET request Control Feature Unit Module */
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_RES_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_RES_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_RES_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_RES_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_RES_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_GET_RES_DELAY_CONTROL;
            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);

    return error;
}

usb_status_t USB_DeviceAudioSetCurAudioFeatureUnit(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint32_t *length,
                                                   uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;

    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_MUTE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_MUTE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_AUTOMATIC_GAIN_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_AUTOMATIC_GAIN_CONTROL;
            break;
        case USB_DEVICE_AUDIO_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_DELAY_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_BOOST_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_BASS_BOOST_CONTROL;
            break;
        case USB_DEVICE_AUDIO_LOUDNESS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_CUR_LOUDNESS_CONTROL;
            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);
    return error;
}

usb_status_t USB_DeviceAudioSetMinAudioFeatureUnit(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint32_t *length,
                                                   uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MIN_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MIN_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MIN_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MIN_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MIN_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MIN_DELAY_CONTROL;
            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);

    return error;
}

usb_status_t USB_DeviceAudioSetMaxAudioFeatureUnit(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint32_t *length,
                                                   uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MAX_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MAX_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MAX_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MAX_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MAX_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_MAX_DELAY_CONTROL;
            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);

    return error;
}

usb_status_t USB_DeviceAudioSetResAudioFeatureUnit(usb_device_handle handle,
                                                   usb_setup_struct_t *setup,
                                                   uint32_t *length,
                                                   uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;
    switch (controlSelector)
    {
        case USB_DEVICE_AUDIO_VOLUME_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_RES_VOLUME_CONTROL;
            break;
        case USB_DEVICE_AUDIO_BASS_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_RES_BASS_CONTROL;
            break;
        case USB_DEVICE_AUDIO_MID_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_RES_MID_CONTROL;
            break;
        case USB_DEVICE_AUDIO_TREBLE_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_RES_TREBLE_CONTROL;
            break;
        case USB_DEVICE_AUDIO_GRAPHIC_EQUALIZER_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_RES_GRAPHIC_EQUALIZER_CONTROL;
            break;
        case USB_DEVICE_AUDIO_DELAY_CONTROL_SELECTOR:
            audioCommand = USB_DEVICE_AUDIO_SET_RES_DELAY_CONTROL;
            break;
        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);
    return error;
}

usb_status_t USB_DeviceAudioGetFeatureUnit(usb_device_handle handle,
                                           usb_setup_struct_t *setup,
                                           uint32_t *length,
                                           uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    /* Select SET request Control Feature Unit Module */
    switch (setup->bRequest)
    {
        case USB_DEVICE_AUDIO_GET_CUR_VOLUME_REQUEST:
            error = USB_DeviceAudioGetCurAudioFeatureUnit(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_VOLUME_REQUEST:
            error = USB_DeviceAudioGetMinAudioFeatureUnit(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_VOLUME_REQUEST:
            error = USB_DeviceAudioGetMaxAudioFeatureUnit(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_GET_RES_VOLUME_REQUEST:
            error = USB_DeviceAudioGetResAudioFeatureUnit(handle, setup, length, buffer);
            break;
        default:
            break;
    }
    return error;
}

usb_status_t USB_DeviceAudioSetFeatureUnit(usb_device_handle handle,
                                           usb_setup_struct_t *setup,
                                           uint32_t *length,
                                           uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    /* Select SET request Control Feature Unit Module */
    switch (setup->bRequest)
    {
        case USB_DEVICE_AUDIO_SET_CUR_VOLUME_REQUEST:
            error = USB_DeviceAudioSetCurAudioFeatureUnit(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_SET_MIN_VOLUME_REQUEST:
            error = USB_DeviceAudioSetMinAudioFeatureUnit(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_SET_MAX_VOLUME_REQUEST:
            error = USB_DeviceAudioSetMaxAudioFeatureUnit(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_SET_RES_VOLUME_REQUEST:
            error = USB_DeviceAudioSetResAudioFeatureUnit(handle, setup, length, buffer);
            break;
        default:
            break;
    }
    return error;
}

usb_status_t USB_DeviceAudioSetControlTerminal(usb_device_handle handle,
                                               usb_setup_struct_t *setup,
                                               uint32_t *length,
                                               uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    return error;
}

usb_status_t USB_DeviceAudioSetRequestInterface(usb_device_handle handle,
                                                usb_setup_struct_t *setup,
                                                uint32_t *length,
                                                uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t entity_id = (uint8_t)(setup->wIndex >> 0x08);

    if ((entity_id == USB_AUDIO_RECORDER_CONTROL_OUTPUT_TERMINAL_ID) ||
        (entity_id == USB_AUDIO_SPEAKER_CONTROL_OUTPUT_TERMINAL_ID))
    {
        error = USB_DeviceAudioSetControlTerminal(handle, setup, length, buffer);
    }
    else if ((entity_id == USB_AUDIO_RECORDER_CONTROL_FEATURE_UNIT_ID) ||
             (entity_id == USB_AUDIO_SPEAKER_CONTROL_FEATURE_UNIT_ID))
    {
        error = USB_DeviceAudioSetFeatureUnit(handle, setup, length, buffer);
    }
    else
    {
    }

    return error;
}

usb_status_t USB_DeviceAudioGetRequestInterface(usb_device_handle handle,
                                                usb_setup_struct_t *setup,
                                                uint32_t *length,
                                                uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t entity_id = (uint8_t)(setup->wIndex >> 0x08);
    if ((entity_id == USB_AUDIO_RECORDER_CONTROL_INPUT_TERMINAL_ID) ||
        (entity_id == USB_AUDIO_SPEAKER_CONTROL_INPUT_TERMINAL_ID))
    {
        error = USB_DeviceAudioGetControlTerminal(handle, setup, length, buffer);
    }
    else if (entity_id == USB_AUDIO_RECORDER_CONTROL_FEATURE_UNIT_ID)
    {
        error = USB_DeviceAudioGetFeatureUnit(handle, setup, length, buffer);
    }
    else if (entity_id == USB_AUDIO_SPEAKER_CONTROL_FEATURE_UNIT_ID)
    {
        error = USB_DeviceAudioGetFeatureUnit(handle, setup, length, buffer);
    }
    else
    {
    }
    return error;
}

usb_status_t USB_DeviceAudioSetRequestEndpoint(usb_device_handle handle,
                                               usb_setup_struct_t *setup,
                                               uint32_t *length,
                                               uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;

    /* Select SET request Control Feature Unit Module */
    switch (setup->bRequest)
    {
        case USB_DEVICE_AUDIO_SET_CUR_VOLUME_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_SET_CUR_SAMPLING_FREQ_CONTROL;
                    break;
                case USB_DEVICE_AUDIO_PITCH_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_SET_CUR_PITCH_CONTROL;
                    break;
                default:
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MIN_VOLUME_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_SET_MIN_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_SET_MAX_VOLUME_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_SET_MAX_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_SET_RES_VOLUME_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_SET_RES_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);
    return error;
}

usb_status_t USB_DeviceAudioGetRequestEndpoint(usb_device_handle handle,
                                               usb_setup_struct_t *setup,
                                               uint32_t *length,
                                               uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t controlSelector = (setup->wValue >> 0x08) & 0xFFU;
    uint32_t audioCommand = 0U;
    /* Select SET request Control Feature Unit Module */
    switch (setup->bRequest)
    {
        case USB_DEVICE_AUDIO_GET_CUR_VOLUME_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_SAMPLING_FREQ_CONTROL_SELECTOR:

                    audioCommand = USB_DEVICE_AUDIO_GET_CUR_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_GET_MIN_VOLUME_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_SAMPLING_FREQ_CONTROL_SELECTOR:

                    audioCommand = USB_DEVICE_AUDIO_GET_MIN_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_GET_MAX_VOLUME_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_SAMPLING_FREQ_CONTROL_SELECTOR:

                    audioCommand = USB_DEVICE_AUDIO_GET_MAX_SAMPLING_FREQ_CONTROL;
                    break;
                default:
                    break;
            }
            break;
        case USB_DEVICE_AUDIO_GET_RES_VOLUME_REQUEST:
            switch (controlSelector)
            {
                case USB_DEVICE_AUDIO_SAMPLING_FREQ_CONTROL_SELECTOR:
                    audioCommand = USB_DEVICE_AUDIO_GET_RES_SAMPLING_FREQ_CONTROL;

                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
    error = USB_DeviceAudioProcessTerminalRequest(audioCommand, length, buffer);
    return error;
}

usb_status_t USB_DeviceAudioUnifiedClassRequest(usb_device_handle handle,
                                                usb_setup_struct_t *setup,
                                                uint32_t *length,
                                                uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_InvalidRequest;

    switch (setup->bmRequestType)
    {
        case USB_DEVICE_AUDIO_SET_REQUEST_INTERFACE:
            error = USB_DeviceAudioSetRequestInterface(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_GET_REQUEST_INTERFACE:
            error = USB_DeviceAudioGetRequestInterface(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_SET_REQUEST_ENDPOINT:
            error = USB_DeviceAudioSetRequestEndpoint(handle, setup, length, buffer);
            break;
        case USB_DEVICE_AUDIO_GET_REQUEST_ENDPOINT:
            error = USB_DeviceAudioGetRequestEndpoint(handle, setup, length, buffer);
            break;
        default:
            break;
    }

    return error;
}

usb_status_t USB_DeviceAudioProcessTerminalRequest(uint32_t audioCommand, uint32_t *length, uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_Success;
    uint16_t volume = 0;
    uint8_t *volBuffAddr;

    switch (audioCommand)
    {
        case USB_DEVICE_AUDIO_GET_CUR_MUTE_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.curMute;
            *length = sizeof(g_deviceComposite->audioUnified.curMute);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_VOLUME_CONTROL:
            *buffer = g_deviceComposite->audioUnified.curVolume;
            *length = sizeof(g_deviceComposite->audioUnified.curVolume);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_BASS_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.curBass;
            *length = sizeof(g_deviceComposite->audioUnified.curBass);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_MID_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.curMid;
            *length = sizeof(g_deviceComposite->audioUnified.curMid);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_TREBLE_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.curTreble;
            *length = sizeof(g_deviceComposite->audioUnified.curTreble);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_AUTOMATIC_GAIN_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.curAutomaticGain;
            *length = sizeof(g_deviceComposite->audioUnified.curAutomaticGain);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_DELAY_CONTROL:
            *buffer = g_deviceComposite->audioUnified.curDelay;
            *length = sizeof(g_deviceComposite->audioUnified.curDelay);
            break;
        case USB_DEVICE_AUDIO_GET_CUR_SAMPLING_FREQ_CONTROL:
            *buffer = g_deviceComposite->audioUnified.curSamplingFrequency;
            *length = sizeof(g_deviceComposite->audioUnified.curSamplingFrequency);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_VOLUME_CONTROL:
            *buffer = g_deviceComposite->audioUnified.minVolume;
            *length = sizeof(g_deviceComposite->audioUnified.minVolume);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_BASS_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.minBass;
            *length = sizeof(g_deviceComposite->audioUnified.minBass);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_MID_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.minMid;
            *length = sizeof(g_deviceComposite->audioUnified.minMid);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_TREBLE_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.minTreble;
            *length = sizeof(g_deviceComposite->audioUnified.minTreble);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_DELAY_CONTROL:
            *buffer = g_deviceComposite->audioUnified.minDelay;
            *length = sizeof(g_deviceComposite->audioUnified.minDelay);
            break;
        case USB_DEVICE_AUDIO_GET_MIN_SAMPLING_FREQ_CONTROL:
            *buffer = g_deviceComposite->audioUnified.minSamplingFrequency;
            *length = sizeof(g_deviceComposite->audioUnified.minSamplingFrequency);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_VOLUME_CONTROL:
            *buffer = g_deviceComposite->audioUnified.maxVolume;
            *length = sizeof(g_deviceComposite->audioUnified.maxVolume);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_BASS_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.maxBass;
            *length = sizeof(g_deviceComposite->audioUnified.maxBass);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_MID_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.maxMid;
            *length = sizeof(g_deviceComposite->audioUnified.maxMid);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_TREBLE_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.maxTreble;
            *length = sizeof(g_deviceComposite->audioUnified.maxTreble);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_DELAY_CONTROL:
            *buffer = g_deviceComposite->audioUnified.maxDelay;
            *length = sizeof(g_deviceComposite->audioUnified.maxDelay);
            break;
        case USB_DEVICE_AUDIO_GET_MAX_SAMPLING_FREQ_CONTROL:
            *buffer = g_deviceComposite->audioUnified.maxSamplingFrequency;
            *length = sizeof(g_deviceComposite->audioUnified.maxSamplingFrequency);
            break;
        case USB_DEVICE_AUDIO_GET_RES_VOLUME_CONTROL:
            *buffer = g_deviceComposite->audioUnified.resVolume;
            *length = sizeof(g_deviceComposite->audioUnified.resVolume);
            break;
        case USB_DEVICE_AUDIO_GET_RES_BASS_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.resBass;
            *length = sizeof(g_deviceComposite->audioUnified.resBass);
            break;
        case USB_DEVICE_AUDIO_GET_RES_MID_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.resMid;
            *length = sizeof(g_deviceComposite->audioUnified.resMid);
            break;
        case USB_DEVICE_AUDIO_GET_RES_TREBLE_CONTROL:
            *buffer = &g_deviceComposite->audioUnified.resTreble;
            *length = sizeof(g_deviceComposite->audioUnified.resTreble);
            break;
        case USB_DEVICE_AUDIO_GET_RES_DELAY_CONTROL:
            *buffer = g_deviceComposite->audioUnified.resDelay;
            *length = sizeof(g_deviceComposite->audioUnified.resDelay);
            break;
        case USB_DEVICE_AUDIO_GET_RES_SAMPLING_FREQ_CONTROL:
            *buffer = g_deviceComposite->audioUnified.resSamplingFrequency;
            *length = sizeof(g_deviceComposite->audioUnified.resSamplingFrequency);
            break;

        case USB_DEVICE_AUDIO_SET_CUR_VOLUME_CONTROL:
            volBuffAddr = *buffer;
            g_deviceComposite->audioUnified.curVolume[0] = *volBuffAddr;
            g_deviceComposite->audioUnified.curVolume[1] = *(volBuffAddr + 1);
            volume = (uint16_t)((uint16_t)g_deviceComposite->audioUnified.curVolume[1] << 8U);
            volume |= (uint8_t)(g_deviceComposite->audioUnified.curVolume[0]);
            g_deviceComposite->audioUnified.codecTask |= VOLUME_CHANGE_TASK;
            break;
        case USB_DEVICE_AUDIO_SET_CUR_MUTE_CONTROL:
            g_deviceComposite->audioUnified.curMute = **(buffer);
            if (g_deviceComposite->audioUnified.curMute)
            {
                g_deviceComposite->audioUnified.codecTask |= MUTE_CODEC_TASK;
            }
            else
            {
                g_deviceComposite->audioUnified.codecTask |= UNMUTE_CODEC_TASK;
            }
            break;
        case USB_DEVICE_AUDIO_SET_CUR_BASS_CONTROL:
            g_deviceComposite->audioUnified.curBass = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_CUR_MID_CONTROL:
            g_deviceComposite->audioUnified.curMid = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_CUR_TREBLE_CONTROL:
            g_deviceComposite->audioUnified.curTreble = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_CUR_AUTOMATIC_GAIN_CONTROL:
            g_deviceComposite->audioUnified.curAutomaticGain = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_CUR_DELAY_CONTROL:
            g_deviceComposite->audioUnified.curDelay[0] = **(buffer);
            g_deviceComposite->audioUnified.curDelay[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_CUR_SAMPLING_FREQ_CONTROL:
            g_deviceComposite->audioUnified.curSamplingFrequency[0] = **(buffer);
            g_deviceComposite->audioUnified.curSamplingFrequency[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_MIN_VOLUME_CONTROL:
            g_deviceComposite->audioUnified.minVolume[0] = **(buffer);
            g_deviceComposite->audioUnified.minVolume[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_MIN_BASS_CONTROL:
            g_deviceComposite->audioUnified.minBass = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_MIN_MID_CONTROL:
            g_deviceComposite->audioUnified.minMid = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_MIN_TREBLE_CONTROL:
            g_deviceComposite->audioUnified.minTreble = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_MIN_DELAY_CONTROL:
            g_deviceComposite->audioUnified.minDelay[0] = **(buffer);
            g_deviceComposite->audioUnified.minDelay[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_MIN_SAMPLING_FREQ_CONTROL:
            g_deviceComposite->audioUnified.minSamplingFrequency[0] = **(buffer);
            g_deviceComposite->audioUnified.minSamplingFrequency[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_MAX_VOLUME_CONTROL:
            g_deviceComposite->audioUnified.maxVolume[0] = **(buffer);
            g_deviceComposite->audioUnified.maxVolume[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_MAX_BASS_CONTROL:
            g_deviceComposite->audioUnified.maxBass = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_MAX_MID_CONTROL:
            g_deviceComposite->audioUnified.maxMid = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_MAX_TREBLE_CONTROL:
            g_deviceComposite->audioUnified.maxTreble = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_MAX_DELAY_CONTROL:
            g_deviceComposite->audioUnified.maxDelay[0] = **(buffer);
            g_deviceComposite->audioUnified.maxDelay[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_MAX_SAMPLING_FREQ_CONTROL:
            g_deviceComposite->audioUnified.maxSamplingFrequency[0] = **(buffer);
            g_deviceComposite->audioUnified.maxSamplingFrequency[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_RES_VOLUME_CONTROL:
            g_deviceComposite->audioUnified.resVolume[0] = **(buffer);
            g_deviceComposite->audioUnified.resVolume[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_RES_BASS_CONTROL:
            g_deviceComposite->audioUnified.resBass = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_RES_MID_CONTROL:
            g_deviceComposite->audioUnified.resMid = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_RES_TREBLE_CONTROL:
            g_deviceComposite->audioUnified.resTreble = **(buffer);
            break;
        case USB_DEVICE_AUDIO_SET_RES_DELAY_CONTROL:
            g_deviceComposite->audioUnified.resDelay[0] = **(buffer);
            g_deviceComposite->audioUnified.resDelay[1] = **(buffer + 1);
            break;
        case USB_DEVICE_AUDIO_SET_RES_SAMPLING_FREQ_CONTROL:
            g_deviceComposite->audioUnified.resSamplingFrequency[0] = **(buffer);
            g_deviceComposite->audioUnified.resSamplingFrequency[1] = **(buffer + 1);
            break;
        default:
            error = kStatus_USB_InvalidRequest;
            break;
    }
    return error;
}

usb_status_t USB_DeviceAudioUnifiedConfigureEndpointStatus(usb_device_handle handle, uint8_t ep, uint8_t status)
{
    usb_status_t error = kStatus_USB_Error;
    if (status)
    {
        if ((USB_AUDIO_RECORDER_STREAM_ENDPOINT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
            (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceStallEndpoint(handle, ep);
        }
        else if ((USB_AUDIO_CONTROL_ENDPOINT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
                 (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceStallEndpoint(handle, ep);
        }
        else if ((USB_AUDIO_SPEAKER_STREAM_ENDPOINT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
                 (!(ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK)))
        {
            return USB_DeviceStallEndpoint(handle, ep);
        }
        else
        {
        }
    }
    else
    {
        if ((USB_AUDIO_RECORDER_STREAM_ENDPOINT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
            (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceUnstallEndpoint(handle, ep);
        }
        else if ((USB_AUDIO_CONTROL_ENDPOINT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
                 (ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
        {
            return USB_DeviceUnstallEndpoint(handle, ep);
        }
        else if ((USB_AUDIO_SPEAKER_STREAM_ENDPOINT == (ep & USB_ENDPOINT_NUMBER_MASK)) &&
                 (!(ep & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK)))
        {
            return USB_DeviceUnstallEndpoint(handle, ep);
        }
        else
        {
        }
    }
    return error;
}

/* The USB_DeviceAudioRecorderStatusReset() function resets the audio recorder status to the initialized status */
void USB_DeviceAudioRecorderStatusReset(void)
{
    g_deviceComposite->audioUnified.startRec = 0;
    g_deviceComposite->audioUnified.startRecHalfFull = 0;
    g_deviceComposite->audioUnified.audioRecvCount = 0;
    g_deviceComposite->audioUnified.usbSendTimes = 0;
    g_deviceComposite->audioUnified.tdReadNumberRec = 0;
    g_deviceComposite->audioUnified.tdWriteNumberRec = 0;
    g_deviceComposite->audioUnified.recorderReservedSpace = 0;
}

/* The USB_DeviceAudioSpeakerStatusReset() function resets the audio speaker status to the initialized status */
void USB_DeviceAudioSpeakerStatusReset(void)
{
    g_deviceComposite->audioUnified.startPlay = 0;
    g_deviceComposite->audioUnified.startPlayHalfFull = 0;
    g_deviceComposite->audioUnified.tdReadNumberPlay = 0;
    g_deviceComposite->audioUnified.tdWriteNumberPlay = 0;
    g_deviceComposite->audioUnified.audioSendCount = 0;
    g_deviceComposite->audioUnified.usbRecvCount = 0;
    g_deviceComposite->audioUnified.lastAudioSendCount = 0;
    g_deviceComposite->audioUnified.audioSendTimes = 0;
    g_deviceComposite->audioUnified.usbRecvTimes = 0;
    g_deviceComposite->audioUnified.speakerIntervalCount = 0;
    g_deviceComposite->audioUnified.speakerReservedSpace = 0;
    g_deviceComposite->audioUnified.timesFeedbackCalculate = 0;
    g_deviceComposite->audioUnified.speakerDetachOrNoInput = 0;
    g_deviceComposite->audioUnified.curAudioPllFrac = AUDIO_PLL_FRACTIONAL_DIVIDER;
    g_deviceComposite->audioUnified.audioPllTicksPrev = 0;
    g_deviceComposite->audioUnified.audioPllTicksDiff = 0;
    g_deviceComposite->audioUnified.audioPllTicksEma = AUDIO_PLL_USB1_SOF_INTERVAL_COUNT;
    g_deviceComposite->audioUnified.audioPllTickEmaFrac = 0;
    g_deviceComposite->audioUnified.audioPllStep = AUDIO_PLL_FRACTIONAL_CHANGE_STEP;
}

/*!
 * @brief Audio Generator device set configuration function.
 *
 * This function sets configuration for Audio class.
 *
 * @param handle The Audio class handle.
 * @param configure The Audio class configure index.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioUnifiedSetConfigure(usb_device_handle handle, uint8_t configure)
{
    usb_status_t error = kStatus_USB_Success;
    if (USB_COMPOSITE_CONFIGURE_INDEX == configure)
    {
        g_deviceComposite->audioUnified.attach = 1U;
    }
    return error;
}

usb_status_t USB_DeviceAudioRecorderSetInterface(usb_device_handle handle, uint8_t interface, uint8_t alternateSetting)
{
    usb_device_endpoint_init_struct_t epInitStruct;
    usb_device_endpoint_callback_struct_t epCallback;
    usb_status_t error = kStatus_USB_Error;

    epCallback.callbackFn = USB_DeviceAudioIsoIN;
    epCallback.callbackParam = handle;

    epInitStruct.zlt = 0U;
    epInitStruct.transferType = USB_ENDPOINT_ISOCHRONOUS;
    epInitStruct.endpointAddress =
        USB_AUDIO_RECORDER_STREAM_ENDPOINT | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    if (USB_SPEED_HIGH == g_deviceComposite->speed)
    {
        epInitStruct.maxPacketSize = HS_ISO_IN_ENDP_PACKET_SIZE;
    }
    else
    {
        epInitStruct.maxPacketSize = FS_ISO_IN_ENDP_PACKET_SIZE;
    }

    USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);
    error =
        USB_DeviceSendRequest(handle, USB_AUDIO_RECORDER_STREAM_ENDPOINT, &audioRecDataBuff[0],
                              (USB_SPEED_HIGH == g_deviceComposite->audioUnified.speed) ? (HS_ISO_IN_ENDP_PACKET_SIZE) :
                                                                                          (FS_ISO_IN_ENDP_PACKET_SIZE));
    return error;
}

usb_status_t USB_DeviceAudioSpeakerSetInterface(usb_device_handle handle, uint8_t interface, uint8_t alternateSetting)
{
    usb_device_endpoint_init_struct_t epInitStruct;
    usb_device_endpoint_callback_struct_t epCallback;
    usb_status_t error = kStatus_USB_Error;

    epCallback.callbackFn = USB_DeviceAudioIsoOUT;
    epCallback.callbackParam = handle;

    epInitStruct.zlt = 0U;
    epInitStruct.transferType = USB_ENDPOINT_ISOCHRONOUS;
    epInitStruct.endpointAddress =
        USB_AUDIO_SPEAKER_STREAM_ENDPOINT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT);
    if (USB_SPEED_HIGH == g_deviceComposite->speed)
    {
        epInitStruct.maxPacketSize = (HS_ISO_OUT_ENDP_PACKET_SIZE);
    }
    else
    {
        epInitStruct.maxPacketSize = (FS_ISO_OUT_ENDP_PACKET_SIZE);
    }
    USB_DeviceInitEndpoint(handle, &epInitStruct, &epCallback);
    error =
        USB_DeviceRecvRequest(handle, USB_AUDIO_SPEAKER_STREAM_ENDPOINT, &audioPlayDataBuff[0],
                              (USB_SPEED_HIGH == g_deviceComposite->audioUnified.speed) ? HS_ISO_OUT_ENDP_PACKET_SIZE :
                                                                                          FS_ISO_OUT_ENDP_PACKET_SIZE);

    return error;
}

#if (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
void SCTIMER_SOF_TOGGLE_HANDLER()
{
    uint32_t currentSctCap = 0, pllCountPeriod = 0, pll_change = 0;
    uint32_t usedSpace = 0;
    static int32_t pllCount = 0, pllDiff = 0;
    static int32_t err, abs_err;
    if (SCTIMER_GetStatusFlags(SCT0) & (1 << eventCounterL))
    {
        /* Clear interrupt flag.*/
        SCTIMER_ClearStatusFlags(SCT0, (1 << eventCounterL));
    }

    if (g_deviceComposite->audioUnified.speakerIntervalCount != 100)
    {
        g_deviceComposite->audioUnified.speakerIntervalCount++;
        return;
    }
    g_deviceComposite->audioUnified.speakerIntervalCount = 1;
    currentSctCap = SCT0->SCTCAP[0];
    pllCountPeriod = currentSctCap - g_deviceComposite->audioUnified.audioPllTicksPrev;
    g_deviceComposite->audioUnified.audioPllTicksPrev = currentSctCap;
    pllCount = pllCountPeriod;
    if (g_deviceComposite->audioUnified.attach)
    {
        if (abs(pllCount - AUDIO_PLL_USB1_SOF_INTERVAL_COUNT) < (AUDIO_PLL_USB1_SOF_INTERVAL_COUNT >> 7))
        {
            pllDiff = pllCount - g_deviceComposite->audioUnified.audioPllTicksEma;
            g_deviceComposite->audioUnified.audioPllTickEmaFrac += (pllDiff % 8);
            g_deviceComposite->audioUnified.audioPllTicksEma +=
                (pllDiff / 8) + g_deviceComposite->audioUnified.audioPllTickEmaFrac / 8;
            g_deviceComposite->audioUnified.audioPllTickEmaFrac =
                (g_deviceComposite->audioUnified.audioPllTickEmaFrac % 8);

            err = g_deviceComposite->audioUnified.audioPllTicksEma - AUDIO_PLL_USB1_SOF_INTERVAL_COUNT;
            abs_err = abs(err);
            if (abs_err > g_deviceComposite->audioUnified.audioPllStep)
            {
                if (err > 0)
                {
                    g_deviceComposite->audioUnified.curAudioPllFrac -=
                        abs_err / g_deviceComposite->audioUnified.audioPllStep;
                }
                else
                {
                    g_deviceComposite->audioUnified.curAudioPllFrac +=
                        abs_err / g_deviceComposite->audioUnified.audioPllStep;
                }
                pll_change = 1;
            }
            if (g_deviceComposite->audioUnified.startPlayHalfFull)
            {
                usedSpace = AudioSpeakerBufferSpaceUsed();
                if (usedSpace > ((AUDIO_SPEAKER_DATA_WHOLE_BUFFER_LENGTH / 2 + 1) * HS_ISO_OUT_ENDP_PACKET_SIZE))
                {
                    g_deviceComposite->audioUnified.curAudioPllFrac++;
                    pll_change = 1;
                }
                else if (usedSpace <
                         ((AUDIO_SPEAKER_DATA_WHOLE_BUFFER_LENGTH / 2 - 1) * HS_ISO_OUT_ENDP_PACKET_SIZE))
                {
                    g_deviceComposite->audioUnified.curAudioPllFrac--;
                    pll_change = 1;
                }
            }
            if (pll_change)
            {
                SYSCON->AUDPLLFRAC = g_deviceComposite->audioUnified.curAudioPllFrac;
                SYSCON->AUDPLLFRAC =
                    g_deviceComposite->audioUnified.curAudioPllFrac | (1U << SYSCON_AUDPLLFRAC_REQ_SHIFT);
            }
        }
    }
}

void SCTIMER_CaptureInit(void)
{
    INPUTMUX->SCT0_INMUX[eventCounterL] = 0x10U; /* 0x10U for USB1 and 0x0FU for USB0. */
    SCTIMER_GetDefaultConfig(&sctimerInfo);

    /* Switch to 16-bit mode */
    sctimerInfo.clockMode = kSCTIMER_Input_ClockMode;
    sctimerInfo.clockSelect = kSCTIMER_Clock_On_Rise_Input_7;

    /* Initialize SCTimer module */
    SCTIMER_Init(SCT0, &sctimerInfo);

    if (SCTIMER_SetupCaptureAction(SCT0, kSCTIMER_Counter_L, &captureRegisterNumber, eventCounterL) == kStatus_Fail)
    {
        usb_echo("SCT Setup Capture failed!\r\n");
    }
    SCT0->EVENT[0].STATE = 0x1;
    SCT0->EVENT[0].CTRL = (0x01 << 10) | (0x2 << 12);

    /* Enable interrupt flag for event associated with out 4, we use the interrupt to update dutycycle */
    SCTIMER_EnableInterrupts(SCT0, (1 << eventCounterL));

    /* Receive notification when event is triggered */
    SCTIMER_SetCallback(SCT0, SCTIMER_SOF_TOGGLE_HANDLER, eventCounterL);

    /* Enable at the NVIC */
    EnableIRQ(SCT0_IRQn);

    /* Start the L counter */
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_L);
}
#endif

/*!
 * @brief Audio Generator device initialization function.
 *
 * This function initializes the device with the composite device class information.
 *
 * @param deviceComposite The pointer to the composite device structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceAudioUnifiedInit(usb_device_composite_struct_t *deviceComposite)
{
    g_deviceComposite = deviceComposite;
    g_deviceComposite->audioUnified.copyProtect = 0x01U;
    g_deviceComposite->audioUnified.curMute = 0x00U;
    g_deviceComposite->audioUnified.curVolume[0] = 0x00U;
    g_deviceComposite->audioUnified.curVolume[1] = 0x80U;
    g_deviceComposite->audioUnified.minVolume[0] = 0x00U;
    g_deviceComposite->audioUnified.minVolume[1] = 0x80U;
    g_deviceComposite->audioUnified.maxVolume[0] = 0xFFU;
    g_deviceComposite->audioUnified.maxVolume[1] = 0X7FU;
    g_deviceComposite->audioUnified.resVolume[0] = 0x01U;
    g_deviceComposite->audioUnified.resVolume[1] = 0x00U;
    g_deviceComposite->audioUnified.curBass = 0x00U;
    g_deviceComposite->audioUnified.curBass = 0x00U;
    g_deviceComposite->audioUnified.minBass = 0x80U;
    g_deviceComposite->audioUnified.maxBass = 0x7FU;
    g_deviceComposite->audioUnified.resBass = 0x01U;
    g_deviceComposite->audioUnified.curMid = 0x00U;
    g_deviceComposite->audioUnified.minMid = 0x80U;
    g_deviceComposite->audioUnified.maxMid = 0x7FU;
    g_deviceComposite->audioUnified.resMid = 0x01U;
    g_deviceComposite->audioUnified.curTreble = 0x01U;
    g_deviceComposite->audioUnified.minTreble = 0x80U;
    g_deviceComposite->audioUnified.maxTreble = 0x7FU;
    g_deviceComposite->audioUnified.resTreble = 0x01U;
    g_deviceComposite->audioUnified.curAutomaticGain = 0x01U;
    g_deviceComposite->audioUnified.curDelay[0] = 0x00U;
    g_deviceComposite->audioUnified.curDelay[1] = 0x40U;
    g_deviceComposite->audioUnified.minDelay[0] = 0x00U;
    g_deviceComposite->audioUnified.minDelay[1] = 0x00U;
    g_deviceComposite->audioUnified.maxDelay[0] = 0xFFU;
    g_deviceComposite->audioUnified.maxDelay[1] = 0xFFU;
    g_deviceComposite->audioUnified.resDelay[0] = 0x00U;
    g_deviceComposite->audioUnified.resDelay[1] = 0x01U;
    g_deviceComposite->audioUnified.curLoudness = 0x01U;
    g_deviceComposite->audioUnified.curSamplingFrequency[0] = 0x00U;
    g_deviceComposite->audioUnified.curSamplingFrequency[1] = 0x00U;
    g_deviceComposite->audioUnified.curSamplingFrequency[2] = 0x01U;
    g_deviceComposite->audioUnified.minSamplingFrequency[0] = 0x00U;
    g_deviceComposite->audioUnified.minSamplingFrequency[1] = 0x00U;
    g_deviceComposite->audioUnified.minSamplingFrequency[2] = 0x01U;
    g_deviceComposite->audioUnified.maxSamplingFrequency[0] = 0x00U;
    g_deviceComposite->audioUnified.maxSamplingFrequency[1] = 0x00U;
    g_deviceComposite->audioUnified.maxSamplingFrequency[2] = 0x01U;
    g_deviceComposite->audioUnified.resSamplingFrequency[0] = 0x00U;
    g_deviceComposite->audioUnified.resSamplingFrequency[1] = 0x00U;
    g_deviceComposite->audioUnified.resSamplingFrequency[2] = 0x01U;
    g_deviceComposite->audioUnified.speed = USB_SPEED_FULL;
    g_deviceComposite->audioUnified.tdReadNumberPlay = 0;
    g_deviceComposite->audioUnified.tdWriteNumberPlay = 0;
    g_deviceComposite->audioUnified.tdReadNumberRec = 0;
    g_deviceComposite->audioUnified.tdWriteNumberRec = 0;
    g_deviceComposite->audioUnified.audioSendCount = 0;
    g_deviceComposite->audioUnified.lastAudioSendCount = 0;
    g_deviceComposite->audioUnified.usbRecvCount = 0;
    g_deviceComposite->audioUnified.audioSendTimes = 0;
    g_deviceComposite->audioUnified.usbRecvTimes = 0;
    g_deviceComposite->audioUnified.audioRecvCount = 0;
    g_deviceComposite->audioUnified.usbSendTimes = 0;
    g_deviceComposite->audioUnified.startPlay = 0;
    g_deviceComposite->audioUnified.startPlayHalfFull = 0;
    g_deviceComposite->audioUnified.startRec = 0;
    g_deviceComposite->audioUnified.startRecHalfFull = 0;
    g_deviceComposite->audioUnified.speakerIntervalCount = 0;
    g_deviceComposite->audioUnified.speakerReservedSpace = 0;
    g_deviceComposite->audioUnified.recorderReservedSpace = 0;
    g_deviceComposite->audioUnified.timesFeedbackCalculate = 0;
    g_deviceComposite->audioUnified.speakerDetachOrNoInput = 0;
    g_deviceComposite->audioUnified.curAudioPllFrac = AUDIO_PLL_FRACTIONAL_DIVIDER;
    g_deviceComposite->audioUnified.audioPllTicksPrev = 0;
    g_deviceComposite->audioUnified.audioPllTicksDiff = 0;
    g_deviceComposite->audioUnified.audioPllTicksEma = AUDIO_PLL_USB1_SOF_INTERVAL_COUNT;
    g_deviceComposite->audioUnified.audioPllTickEmaFrac = 0;
    g_deviceComposite->audioUnified.audioPllStep = AUDIO_PLL_FRACTIONAL_CHANGE_STEP;
    for (uint8_t i = 0; i < USB_AUDIO_COMPOSITE_INTERFACE_COUNT; i++)
    {
        g_deviceComposite->audioUnified.currentInterfaceAlternateSetting[i] = 0;
    }
    return kStatus_USB_Success;
}

void USB_AudioCodecTask(void)
{
    if (g_deviceComposite->audioUnified.codecTask & MUTE_CODEC_TASK)
    {
        usb_echo("Set Cur Mute : %x\r\n", g_deviceComposite->audioUnified.curMute);
        BOARD_SetCodecMuteUnmute(true);
        g_deviceComposite->audioUnified.codecTask &= ~MUTE_CODEC_TASK;
        g_CodecMuteUnmute = true;
    }
    if (g_deviceComposite->audioUnified.codecTask & UNMUTE_CODEC_TASK)
    {
        usb_echo("Set Cur Mute : %x\r\n", g_deviceComposite->audioUnified.curMute);
        BOARD_SetCodecMuteUnmute(false);
        g_deviceComposite->audioUnified.codecTask &= ~UNMUTE_CODEC_TASK;
        g_CodecMuteUnmute = true;
    }
    if (g_deviceComposite->audioUnified.codecTask & VOLUME_CHANGE_TASK)
    {
        usb_echo("Set Cur Volume : %x\r\n", (uint16_t)(g_deviceComposite->audioUnified.curVolume[1] << 8U) |
                                                g_deviceComposite->audioUnified.curVolume[0]);
        g_deviceComposite->audioUnified.codecTask &= ~VOLUME_CHANGE_TASK;
    }
}

void USB_AudioSpeakerResetTask(void)
{
    if (g_deviceComposite->audioUnified.speakerDetachOrNoInput)
    {
        USB_DeviceAudioSpeakerStatusReset();
    }
}
