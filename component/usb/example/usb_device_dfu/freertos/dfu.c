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

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_dfu.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "dfu.h"
#include "dfu_timer.h"
#include "usb_flash.h"
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "composite.h"
#include <stdio.h>
#include <stdlib.h>
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef usb_status_t (*dfu_state_func)(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);

static usb_status_t USB_DeviceDfuDetachReqest(uint16_t wTimeout);
static usb_status_t USB_DeviceDfuDownLoadReqest(uint16_t wLength, uint8_t **data);
static usb_status_t USB_DeviceDfuUpLoadReqest(uint32_t *length, uint8_t **data);
static usb_status_t USB_DeviceDfuGetStatusReqest(uint8_t *length, uint8_t **data);
static usb_status_t USB_DeviceDfuClearStatusReqest(void);
static usb_status_t USB_DeviceDfuGetStateReqest(uint8_t *length, uint8_t **data);
static usb_status_t USB_DeviceDfuAbortReqest(void);
static usb_status_t USB_DeviceStateAppIdle(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateAppDetach(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuIdle(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuDnLoadSync(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuDnBusy(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuDnLoadIdle(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuManifestSync(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuManifest(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuManifestWaitReset(usb_dfu_struct_t *dfu_dev,
                                                        usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuUpLoadIdle(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);
static usb_status_t USB_DeviceStateDfuError(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* DFU state function table. */
const static dfu_state_func s_dfuStateFunc[11] = {
    USB_DeviceStateAppIdle,         USB_DeviceStateAppDetach,   USB_DeviceStateDfuIdle,
    USB_DeviceStateDfuDnLoadSync,   USB_DeviceStateDfuDnBusy,   USB_DeviceStateDfuDnLoadIdle,
    USB_DeviceStateDfuManifestSync, USB_DeviceStateDfuManifest, USB_DeviceStateDfuManifestWaitReset,
    USB_DeviceStateDfuUpLoadIdle,   USB_DeviceStateDfuError};

/* Instance of a DFU demo structure. */
static usb_dfu_struct_t s_UsbDeviceDfuDemo;
/* Temp buffer is used to store DFU downloaded data */
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_tempBuff[MAX_TRANSFER_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t dfuFirmwareBlock[MAX_TRANSFER_SIZE];


#if USB_DFU_BIT_CAN_UPLOAD
#define UPLOAD_SIZE (2 * MAX_TRANSFER_SIZE + 2)
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t updateLoadData[UPLOAD_SIZE];
#endif
/* Flag which indicates if it has received a detach request. This flag is stored in a specific section
 * to avoid being initialized to zero after reset.
 */
uint8_t g_detachRequest;
/* DFU event queue. */
static dfu_queue_t s_DfuEventQueue;
/* Flag which indicates if it has sent a short frame in UPLOAD request. */
static bool s_isShortFrame = false;
/* DFU CRC table list */
static uint32_t s_dfuCRCTableList[256];

uint32_t crcValue;

extern usb_device_composite_struct_t g_UsbDeviceComposite;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief DFU CRC table creation function.
 *
 * This function creates the CRC table for CRC calculation.
 *
 * @return None.
 */

static void USB_DeviceDfuCreateCRCTableList(void)
{
    /* CRC32 - polynomial reserved */
    uint32_t polynomial = 0xEDB88320U;
    for (uint16_t index = 0U; index < 256U; index++)
    {
        uint32_t crcElement = index;
        uint32_t topBit = 0x00000001U;
        for (uint8_t i = 0U; i < 8U; i++)
        {
            if (crcElement & topBit)
            {
                crcElement = (crcElement >> 1U) ^ polynomial;
            }
            else
            {
                crcElement = (crcElement >> 1U);
            }
        }
        s_dfuCRCTableList[index] = crcElement;
    }
}

/*!
 * @brief DFU CRC calculation function.
 *
 * This function calculates the CRC over a buffer.
 *
 * @return CRC value.
 */
static uint32_t USB_DeviceDfuCalculateCRC(uint32_t crc, uint8_t *data, uint32_t length)
{
    uint8_t crcIndex = 0U;
    uint32_t crcReturn = crc;
    for (uint32_t i = 0U; i < length; i++)
    {
        crcIndex = (uint8_t)((crcReturn & 0x000000FFU) ^ data[i]);
        crcReturn = s_dfuCRCTableList[crcIndex] ^ (crcReturn >> 8U);
    }
    return (crcReturn);
}
/*!
 * @brief host cdc enter critical.
 *
 * This function is used to enter critical disable interrupt .
 *
 */
static void USB_DfuEnterCritical(uint8_t *sr)
{
    *sr = DisableGlobalIRQ();
    __ASM("CPSID I");
}
/*!
 * @brief host cdc exit critical.
 *
 * This function is used to exit critical ,enable interrupt .
 *
 */
static void USB_DfuExitCritical(uint8_t sr)
{
    EnableGlobalIRQ(sr);
}
/*!
 * @brief DFU set state function.
 *
 * This function sets the state for a device.
 *
 * @return kStatus_USB_Success.
 */
static usb_status_t USB_DeviceDfuSetState(usb_dfu_state_struct_t state)
{
    USB_DEVICE_DFU_ENTER_CRITICAL();
    s_UsbDeviceDfuDemo.dfuStatus.bState = state;
    USB_DEVICE_DFU_EXIT_CRITICAL();
    return kStatus_USB_Success;
}

/*!
 * @brief DFU get state function.
 *
 * This function gets the state for a device.
 *
 * @return The state of the device.
 */
static usb_dfu_state_struct_t USB_DeviceDfuGetState(void)
{
    return (usb_dfu_state_struct_t)s_UsbDeviceDfuDemo.dfuStatus.bState;
}

/*!
 * @brief DFU set status function.
 *
 * This function sets the status for a device.
 *
 * @return kStatus_USB_Success.
 */
static usb_status_t USB_DeviceDfuSetStatus(uint32_t status)
{
    USB_DEVICE_DFU_ENTER_CRITICAL();
    s_UsbDeviceDfuDemo.dfuStatus.bStatus = status;
    USB_DEVICE_DFU_EXIT_CRITICAL();
    return kStatus_USB_Success;
}

/*!
 * @brief DFU get status function.
 *
 * This function gets the status for a device.
 *
 * @return The status of the device.
 */
static uint32_t USB_DeviceDfuGetStatus(void)
{
    return s_UsbDeviceDfuDemo.dfuStatus.bStatus;
}
#if USB_DFU_BIT_WILL_DETACH
#elif
/*!
 * @brief DFU detach timeout routine.
 *
 * This function serves as the timeout routine for DETACH request.
 *
 * @return None.
 */
static void USB_DeviceDfuDetachTimeoutIsr(void)
{
    usb_status_t error = kStatus_USB_Success;
    usb_device_dfu_event_struct_t event;
    event.name = kUSB_DeviceDfuEventDetachTimeout;
    event.wValue = 0;
    event.wLength = 0;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
}
#endif
/*!
 * @brief DFU poll timeout routine.
 *
 * This function serves as the timeout routine for GET_STATUS request.
 *
 * @return None.
 */
static void USB_DeviceDfuPollTimeoutIsr(void)
{
    usb_status_t error = kStatus_USB_Success;
    usb_device_dfu_event_struct_t event;
    event.name = kUSB_DeviceDfuEventPollTimeout;
    event.wValue = 0;
    event.wLength = 0;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
}

/*!
 * @brief DFU DETACH request function.
 *
 * This function validates the request against the current state And informs the
 * state function via the event queue.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceDfuDetachReqest(uint16_t wTimeout)
{
    usb_status_t error = kStatus_USB_Success;
    usb_dfu_state_struct_t state;
    usb_device_dfu_event_struct_t event;
    uint32_t status;
    state = USB_DeviceDfuGetState();
    if (kState_AppIdle != state)
    {
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
    }
    status = USB_DeviceDfuGetStatus();
    event.name = kUSB_DeviceDfuEventDetachReq;
    event.wValue = wTimeout;
    event.wLength = 0;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
    if (USB_DFU_STATUS_ERR_STALLEDPKT == status)
    {
        /* Stall the control pipe. */
        error = kStatus_USB_InvalidRequest;
    }
    else
    {
        error = kStatus_USB_Success;
    }
    return error;
}

/*!
 * @brief DFU DNLOAD request function.
 *
 * This function validates the request against the current state And informs the
 * state function via the event queue.
 *
 * @return A USB error code or kStatus_USB_Success.
 */

static usb_status_t USB_DeviceDfuDownLoadReqest(uint16_t wLength, uint8_t **data)
{
    usb_status_t error = kStatus_USB_Success;
    usb_dfu_state_struct_t state;
    usb_device_dfu_event_struct_t event;
    uint32_t status;
    state = USB_DeviceDfuGetState();
    if ((kState_DfuIdle == state) || (kState_DfuDnLoadIdle == state))
    {
        if (USB_DFU_BIT_CAN_DNLOAD && (wLength > 0))
        {
            /* store the fimware block data */
            memcpy((void *)s_UsbDeviceDfuDemo.dfuFirmwareBlock, *data, wLength);
        }
        else if (0 == wLength)
        {
            status = USB_DeviceDfuGetStatus();

            if (USB_DFU_STATUS_ERR_NOT_DONE == status)
            {
                USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
            }
        }
        else
        {
            USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
        }
    }
    else
    {
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
    }
    status = USB_DeviceDfuGetStatus();
    event.name = kUSB_DeviceDfuEventDnloadReq;
    event.wValue = 0;
    event.wLength = wLength;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
    if (USB_DFU_STATUS_ERR_STALLEDPKT == status)
    {
        /* Stall the control pipe. */
        error = kStatus_USB_InvalidRequest;
    }
    else
    {
        error = kStatus_USB_Success;
    }
    return error;
}

/*!
 * @brief DFU UPLOAD request function.
 *
 * This function validates the request against the current state, assign the appropriate buffer
 * and length for the UPLOAD request.  And informs the state function via the event queue.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceDfuUpLoadReqest(uint32_t *length, uint8_t **data)
{
    usb_status_t error = kStatus_USB_Success;
    usb_dfu_state_struct_t state;
    usb_device_dfu_event_struct_t event;
    uint32_t status;
    uint32_t uploadLength;
    state = USB_DeviceDfuGetState();
    if (kState_DfuIdle == state)
    {
#if USB_DFU_BIT_CAN_UPLOAD
        s_isShortFrame = false;

        /* get firmware start address from USB_DFU_APP_ADDRESS address */
        s_UsbDeviceDfuDemo.dfuFirmwareAddress =
            (uint32_t)&updateLoadData[0]; 
        /* get firmware size from USB_DFU_APP_ADDRESS + 4 address */
        s_UsbDeviceDfuDemo.dfuFirmwareSize = UPLOAD_SIZE;
        if ((0xFFFFFFFFU == s_UsbDeviceDfuDemo.dfuFirmwareSize) ||
            (0xFFFFFFFFU == s_UsbDeviceDfuDemo.dfuFirmwareAddress))
        {
            USB_DeviceDfuSetState(kState_DfuError);
            USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
            error = kStatus_USB_InvalidRequest;
        }
        else
        {
            usb_echo("\nUploading firmware ...\n");
            s_UsbDeviceDfuDemo.dfuStatus.bState = kState_DfuUpLoadIdle;
            s_UsbDeviceDfuDemo.dfuCurrentUploadLenght = 0U;

            uploadLength = *((uint32_t *)length);
            for (uint32_t i = 0; i < uploadLength; i++)
            {
                s_UsbDeviceDfuDemo.dfuFirmwareBlock[i] = (((uint8_t *)s_UsbDeviceDfuDemo.dfuFirmwareAddress))[i];
            }
            s_UsbDeviceDfuDemo.dfuCurrentUploadLenght += uploadLength;
            *data = s_UsbDeviceDfuDemo.dfuFirmwareBlock;
        }
#else
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
#endif
    }
    else if (kState_DfuUpLoadIdle == state)
    {
        /* uploading indicator */
        usb_echo("&");
        if (s_UsbDeviceDfuDemo.dfuFirmwareSize - s_UsbDeviceDfuDemo.dfuCurrentUploadLenght >= *length)
        {
            uploadLength = *((uint32_t *)length);
            for (uint32_t i = 0; i < uploadLength; i++)
            {
                s_UsbDeviceDfuDemo.dfuFirmwareBlock[i] =
                    (((uint8_t *)s_UsbDeviceDfuDemo.dfuFirmwareAddress) + s_UsbDeviceDfuDemo.dfuCurrentUploadLenght)[i];
            }
            s_UsbDeviceDfuDemo.dfuCurrentUploadLenght += uploadLength;
            *data = s_UsbDeviceDfuDemo.dfuFirmwareBlock;
        }
        else
        {
            usb_echo("\nUploading firmware completed.\n");
            uploadLength = *((uint32_t *)length);
            *length = s_UsbDeviceDfuDemo.dfuFirmwareSize - s_UsbDeviceDfuDemo.dfuCurrentUploadLenght;
            for (uint32_t i = 0; i < *length; i++)
            {
                s_UsbDeviceDfuDemo.dfuFirmwareBlock[i] =
                    (((uint8_t *)s_UsbDeviceDfuDemo.dfuFirmwareAddress) + s_UsbDeviceDfuDemo.dfuCurrentUploadLenght)[i];
            }
            s_UsbDeviceDfuDemo.dfuCurrentUploadLenght += *length;
            *data = s_UsbDeviceDfuDemo.dfuFirmwareBlock;
            s_isShortFrame = true;
        }
    }
    else
    {
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
    }
    status = USB_DeviceDfuGetStatus();
    event.name = kUSB_DeviceDfuEventUploadReq;
    event.wValue = 0;
    event.wLength = *length;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
    if (USB_DFU_STATUS_ERR_STALLEDPKT == status)
    {
        /* Stall the control pipe. */
        error = kStatus_USB_InvalidRequest;
    }
    else
    {
        error = kStatus_USB_Success;
    }
    return error;
}

/*!
 * @brief DFU GET_STATUS request function.
 *
 * This function validates the request against the current state, sends the current status. And informs the
 * state function via the event queue.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceDfuGetStatusReqest(uint8_t *length, uint8_t **data)
{
    usb_status_t error = kStatus_USB_Success;
    usb_dfu_state_struct_t state;
    usb_device_dfu_event_struct_t event;
    uint32_t status;
    state = USB_DeviceDfuGetState();
    if ((kState_DfuDnBusy == state) || (kState_DfuManifest == state))
    {
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
    }
    else
    {
        /* Device returns the status response */
        *data = (uint8_t *)&s_UsbDeviceDfuDemo.dfuStatus;
        *length = 6U;
    }
    status = USB_DeviceDfuGetStatus();
    event.name = kUSB_DeviceDfuEventGetStatusReq;
    event.wValue = 0;
    event.wLength = *length;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
    if (USB_DFU_STATUS_ERR_STALLEDPKT == status)
    {
        /* Stall the control pipe. */
        error = kStatus_USB_InvalidRequest;
    }
    else
    {
        error = kStatus_USB_Success;
    }
    return error;
}

/*!
 * @brief DFU CLEAR_STATUS request function.
 *
 * This function validates the request against the current state. And informs the
 * state function via the event queue.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceDfuClearStatusReqest(void)
{
    usb_status_t error = kStatus_USB_Success;
    usb_dfu_state_struct_t state;
    usb_device_dfu_event_struct_t event;
    uint32_t status;
    state = USB_DeviceDfuGetState();
    if (kState_DfuError == state)
    {
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_OK);
    }
    else
    {
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
    }
    status = USB_DeviceDfuGetStatus();
    event.name = kUSB_DeviceDfuEventClearStatusReq;
    event.wValue = 0;
    event.wLength = 0;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
    if (USB_DFU_STATUS_ERR_STALLEDPKT == status)
    {
        /* Stall the control pipe. */
        error = kStatus_USB_InvalidRequest;
    }
    else
    {
        error = kStatus_USB_Success;
    }
    return error;
}

/*!
 * @brief DFU GET_STATE request function.
 *
 * This function validates the request against the current state, sends the current
 * state. And informs the state function via the event queue.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceDfuGetStateReqest(uint8_t *length, uint8_t **data)
{
    usb_status_t error = kStatus_USB_Success;
    usb_dfu_state_struct_t state;
    usb_device_dfu_event_struct_t event;
    uint32_t status;
    state = USB_DeviceDfuGetState();
    if ((kState_DfuDnBusy == state) || (kState_DfuManifest == state))
    {
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
    }
    else
    {
        /* Device returns the state response */
        *data = (uint8_t *)&s_UsbDeviceDfuDemo.dfuStatus.bState;
        *length = 1U;
    }
    status = USB_DeviceDfuGetStatus();
    event.name = kUSB_DeviceDfuEventGetStateReq;
    event.wValue = 0;
    event.wLength = *length;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
    if (USB_DFU_STATUS_ERR_STALLEDPKT == status)
    {
        /* Stall the control pipe. */
        error = kStatus_USB_InvalidRequest;
    }
    else
    {
        error = kStatus_USB_Success;
    }
    return error;
}

/*!
 * @brief DFU ABORT request function.
 *
 * This function validates the request against the current state. And informs the
 * state function via the event queue.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceDfuAbortReqest(void)
{
    usb_status_t error = kStatus_USB_Success;
    usb_dfu_state_struct_t state;
    usb_device_dfu_event_struct_t event;
    uint32_t status;
    state = USB_DeviceDfuGetState();
    if ((kState_DfuIdle == state) || (kState_DfuDnLoadIdle == state) || (kState_DfuUpLoadIdle == state))
    {
    }
    else
    {
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_STALLEDPKT);
    }
    status = USB_DeviceDfuGetStatus();
    event.name = kUSB_DeviceDfuEventAbortReq;
    event.wValue = 0;
    event.wLength = 0;
    error = USB_DeviceDfuQueuePut(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success != error)
    {
        /* The queue is full, set the status to error unknown */
        USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
    }
    if (USB_DFU_STATUS_ERR_STALLEDPKT == status)
    {
        /* Stall the control pipe. */
        error = kStatus_USB_InvalidRequest;
    }
    else
    {
        error = kStatus_USB_Success;
    }
    return error;
}

/*!
 * @brief DFU class callback function.
 *
 * This function serves as the callback function for DFU class specific requests.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceDfuDemoCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    usb_device_control_request_struct_t *controlRequest = (usb_device_control_request_struct_t *)param;
    switch (event)
    {
        case kUSB_DeviceDfuEventDetach:
            error = USB_DeviceDfuDetachReqest(controlRequest->setup->wValue);
            break;
        case kUSB_DeviceDfuEventDownLoad:
            if ((controlRequest->isSetup) && (controlRequest->setup->wLength > 0))
            {
                controlRequest->buffer = s_tempBuff;
                error = kStatus_USB_Success;
            }
            else
            {
                error = USB_DeviceDfuDownLoadReqest(controlRequest->setup->wLength, &controlRequest->buffer);
            }
            break;
        case kUSB_DeviceDfuEventUpLoad:
            error = USB_DeviceDfuUpLoadReqest(&controlRequest->length, &controlRequest->buffer);
            break;
        case kUSB_DeviceDfuEventGetStatus:
            error = USB_DeviceDfuGetStatusReqest((uint8_t *)&controlRequest->length, &controlRequest->buffer);
            break;
        case kUSB_DeviceDfuEventClearStatus:
            error = USB_DeviceDfuClearStatusReqest();
            break;
        case kUSB_DeviceDfuEventGetState:
            error = USB_DeviceDfuGetStateReqest((uint8_t *)&controlRequest->length, &controlRequest->buffer);
            break;
        case kUSB_DeviceDfuEventAbort:
            error = USB_DeviceDfuAbortReqest();
            break;
        default:
            break;
    }

    return error;
}


/*!
 * @brief DFU switch mode function.
 *
 * This function switches the device from DFU mode to APP mode.
 *
 * @return None.
 */
void USB_DeviceDfuSwitchMode(void)
{
    {
        uint32_t address;
        address = (uint32_t)(USB_DFU_APP_ADDRESS);
        if (0xFFFFFFFFU != address)
        {
            uint32_t newSP, newPC;
            newSP = ((uint32_t *)address)[0U];
            newPC = ((uint32_t *)address)[1U];
            if ((0xFFFFFFFFU != newSP) && (0xFFFFFFFFU != newPC))
            {
                void (*switchToApplicationMode)(void);
                /* load new value to stack pointer and program counter */
                __set_CONTROL(0x00000000U);
                __set_MSP(newSP);
                switchToApplicationMode = (void (*)(void))newPC;
#if (0U != __CORTEX_M)
                /* reset FAULTMASK register */
                __set_FAULTMASK(0x00000000U);
#endif
                /* switch to application mode */
                switchToApplicationMode();
            }
        }
    }
}
/*!
 * @brief DFU USB bus reset function.
 *
 * This function resets the USB bus by resetting the system.
 *
 * @return None.
 */
void USB_DeviceDfuBusReset(void)
{
    switch (s_UsbDeviceDfuDemo.dfuStatus.bState)
    {
        case kState_AppIdle:
            /* Do nothing */
            break;
        case kState_AppDetach:
            /* Enter DFU mode */
            USB_DeviceDfuSetState(kState_DfuIdle);

            /*g_detachRequest = 1U;*/
            /* Switch to DFU mode */
            /*NVIC_SystemReset();*/
            break;
        case kState_DfuIdle:
        case kState_DfuDnLoadSync:
        case kState_DfuDnBusy:
        case kState_DfuDnLoadIdle:
        case kState_DfuManifestSync:
        case kState_DfuManifest:
        case kState_DfuManifestWaitReset:
        case kState_DfuUpLoadIdle:
        case kState_DfuError:
            if (s_UsbDeviceDfuDemo.dfuIsDownloadingFinished)
            {
                s_UsbDeviceDfuDemo.dfuStatus.bState = kState_AppIdle;
                s_UsbDeviceDfuDemo.dfuIsDownloadingFinished = 0U;
                /* switch to APP mode */
                /*NVIC_SystemReset();*/
            }
            else
            {
                if (USB_DFU_BLOCK_TRANSFER_UNDEFINED != s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus)
                {
                    /* The firmware is downloading to the device but the bus reset occurs, change
                       the state to Error */
                    s_UsbDeviceDfuDemo.dfuStatus.bState = kState_DfuError;
                    s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus = USB_DFU_BLOCK_TRANSFER_UNDEFINED;
                }
                else
                {
                    /* the downloading is not started yet, there is no firmware is downloaded */
                    /* do nothing */
                }
            }
            break;
        default:
            break;
    }
}

/*!
 * @brief DFU demo initialization function.
 *
 * This function initializes the state of the device..
 *
 * @return None.
 */
void USB_DeviceDfuDemoInit(void)
{
    /* flash status */
    usb_flash_status_t flashStatus = kStatus_USB_FlashErrorUnknown;
    /* Default DFU status */
    s_UsbDeviceDfuDemo.dfuStatus.bStatus = USB_DFU_STATUS_OK;
    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[0U] = 0xc8U;
    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[1U] = 0U;
    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[2U] = 0U;
#if ((USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V1 > 0U) || (USB_DFU_JOYSTICK_COMPOSITE_EXAMPLE_V2 > 0U))
    USB_DeviceDfuSetState(kState_AppIdle);
#else
    USB_DeviceDfuSetState(kState_DfuIdle);
#endif
    s_UsbDeviceDfuDemo.dfuStatus.iString = 0U;
    s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus = USB_DFU_BLOCK_TRANSFER_UNDEFINED;
    s_UsbDeviceDfuDemo.dfuManifestationPhaseStatus = USB_DFU_MANIFEST_UNDEFINED;
    s_UsbDeviceDfuDemo.dfuFirmwareAddress = USB_DFU_APP_ADDRESS;
    s_UsbDeviceDfuDemo.dfuFirmwareSize = 0U;
    s_UsbDeviceDfuDemo.dfuIsTheFirstBlock = 0U;
    s_UsbDeviceDfuDemo.dfuCRC = 0XFFFFFFFF;
    s_UsbDeviceDfuDemo.dfuFirmwareBlock = &dfuFirmwareBlock[0];

    g_detachRequest = 0U;
    USB_DeviceDfuQueueInit(&s_DfuEventQueue);

    USB_DeviceDfuCreateCRCTableList();

    flashStatus = USB_FlashInit();
    if (kStatus_USB_FlashErrorSecure == flashStatus)
    {
        s_UsbDeviceDfuDemo.dfuStatus.bStatus = USB_DFU_STATUS_ERR_WRITE;
    }
    else if (kStatus_USB_FlashErrorUnknown == flashStatus)
    {
        s_UsbDeviceDfuDemo.dfuStatus.bStatus = USB_DFU_STATUS_ERR_UNKNOWN;
    }
    else
    {
    }
#if USB_DFU_BIT_CAN_UPLOAD
    uint32_t *temp;
    temp = (uint32_t *)&updateLoadData[0];
    for (uint32_t i = 0; i < (UPLOAD_SIZE / 4 - 2); i++)
    {
        temp[i] = i;
    }
#endif
    DFU_TimerInit();
}

/*!
 * @brief DFU manifest function.
 *
 * This function does manifestation operation in MANIFEST state.
 *
 * @return None.
 */
void USB_DeviceDfuManifest(void)
{
    /*do some thing to manifest*/
    /* store the fimware block data */
    if (USB_DFU_MANIFEST_IN_PROGRESS == s_UsbDeviceDfuDemo.dfuManifestationPhaseStatus)
    {
        crcValue = 0;
        uint32_t remainingLen = s_UsbDeviceDfuDemo.dfuFirmwareSize - 4;
        uint8_t *startAddress = (uint8_t *)USB_DFU_APP_ADDRESS;

        s_UsbDeviceDfuDemo.dfuCRC = 0xffffffff;
        uint32_t wLength = MAX_TRANSFER_SIZE;
        uint32_t readLen = 0;
        if (remainingLen < MAX_TRANSFER_SIZE)
        {
         wLength = remainingLen;
        }
        while (remainingLen)
        {
          memcpy((void *)s_UsbDeviceDfuDemo.dfuFirmwareBlock, (uint8_t *)(startAddress + readLen), wLength);
          readLen += wLength;
          /* calculate DFU CRC */
          s_UsbDeviceDfuDemo.dfuCRC = USB_DeviceDfuCalculateCRC(
              s_UsbDeviceDfuDemo.dfuCRC, (uint8_t *)&s_UsbDeviceDfuDemo.dfuFirmwareBlock[0], wLength);
          remainingLen -= wLength;
          if (remainingLen < MAX_TRANSFER_SIZE)
          {
              wLength = remainingLen;
          }

        }
        memcpy((void *)&crcValue, (uint8_t *)(startAddress + s_UsbDeviceDfuDemo.dfuFirmwareSize - 4), 4);

        s_UsbDeviceDfuDemo.crcCheck = 0;
        if (s_UsbDeviceDfuDemo.dfuCRC != crcValue)
        {
          usb_echo("crc check error\r\n");
        }
        else
        {
          usb_echo("crc check ok\r\n");
          s_UsbDeviceDfuDemo.crcCheck = 1;
        }

        s_UsbDeviceDfuDemo.dfuManifestationPhaseStatus = USB_DFU_MANIFEST_COMPLETE;
    }
}

/*!
 * @brief DFU download function.
 *
 * This function does download operation in DNLOAD state.
 *
 * @return None.
 */


void USB_DeviceDfuDnload(void)
{
    if (USB_DFU_BLOCK_TRANSFER_IN_PROGRESS == s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus)
    {
        usb_flash_status_t flashStatus = kStatus_USB_FlashErrorUnknown;
        uint32_t firmwareAddress = USB_DFU_APP_ADDRESS + s_UsbDeviceDfuDemo.dfuFirmwareSize;
        if (s_UsbDeviceDfuDemo.dfuIsTheFirstBlock)
        {
            /* Erase application region */

            uint8_t usbOsaCurrentSr;

            USB_DfuEnterCritical(&usbOsaCurrentSr);
            flashStatus = USB_FlashErase((uint32_t)USB_DFU_APP_ADDRESS, USB_DFU_APP_SIZE);

            USB_DfuExitCritical(usbOsaCurrentSr);
            if (kStatus_USB_FlashErrorErase == flashStatus)
            {
                s_UsbDeviceDfuDemo.dfuStatus.bStatus = USB_DFU_STATUS_ERR_ERASE;
            }
            else if (kStatus_USB_FlashErrorEraseVerify == flashStatus)
            {
                s_UsbDeviceDfuDemo.dfuStatus.bStatus = USB_DFU_STATUS_ERR_CHECK_ERASED;
            }
            else if (kStatus_USB_FlashErrorUnknown == flashStatus)
            {
                s_UsbDeviceDfuDemo.dfuStatus.bStatus = USB_DFU_STATUS_ERR_UNKNOWN;
            }
            else
            {
            }
            s_UsbDeviceDfuDemo.dfuIsTheFirstBlock = 0U;
        }
        if (s_UsbDeviceDfuDemo.dfuFirmwareBlockLength > 0U)
        {
            /* Update the firmware size */
            s_UsbDeviceDfuDemo.dfuFirmwareSize += s_UsbDeviceDfuDemo.dfuFirmwareBlockLength;
            /* firmware flashing */

            uint8_t usbOsaCurrentSr;

            USB_DfuEnterCritical(&usbOsaCurrentSr);

            flashStatus = USB_FlashProgram(firmwareAddress, (uint8_t *)&s_UsbDeviceDfuDemo.dfuFirmwareBlock[0],
                                           s_UsbDeviceDfuDemo.dfuFirmwareBlockLength);
            USB_DfuExitCritical(usbOsaCurrentSr);

            if (kStatus_USB_FlashSuccess == flashStatus)
            {
                /* downloading indicator */
                /*usb_echo("&");*/
                s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus = USB_DFU_BLOCK_TRANSFER_COMPLETE;
            }
            else if (flashStatus == kStatus_USB_FlashErrorProgram)
            {
                USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_PROG);
            }
            else if (flashStatus == kStatus_USB_FlashErrorProgramAddress)
            {
                USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_ADDRESS);
            }
            else if (flashStatus == kStatus_USB_FlashErrorProgramVerify)
            {
                USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_VERIFY);
            }
            else if (flashStatus == kStatus_USB_FlashErrorUnknown)
            {
                USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
            }
            else
            {
            }

        }
    }
}

/*!
 * @brief DFU APP_IDLE state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateAppIdle(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventDetachReq:
                USB_DeviceDfuDemoInit();
#if USB_DFU_BIT_WILL_DETACH
                g_detachRequest = 1U;
                /* Device generates a detach-attach sequence on the bus */
                USB_DeviceStop(g_UsbDeviceComposite.deviceHandle);
                for (int i = 0; i < 5000; i++)
                {
                    __ASM("nop");
                }
                USB_DeviceRun(g_UsbDeviceComposite.deviceHandle);
#else
                g_detachRequest = 1U;
                if (event->wValue > USB_DFU_DETACH_TIMEOUT)
                {
                    USB_DeviceDfuSetStatus(USB_DFU_STATUS_ERR_UNKNOWN);
                    error = kStatus_USB_InvalidParameter;
                }
                else
                {
                    dfu_timer_object_t dfuTimerObject;
                    dfuTimerObject.timerCount = event->wValue;
                    dfuTimerObject.timerCallback = (dfu_timer_callback)USB_DeviceDfuDetachTimeoutIsr;
                    s_UsbDeviceDfuDemo.dfuTimerId = DFU_AddTimerQueue(&dfuTimerObject);
                }
#endif
                USB_DeviceDfuSetState(kState_AppDetach);
                break;
            case kUSB_DeviceDfuEventGetStatusReq:
                USB_DeviceDfuSetState(kState_AppIdle);
                break;
            case kUSB_DeviceDfuEventGetStateReq:
                USB_DeviceDfuSetState(kState_AppIdle);
                break;
            default:
                USB_DeviceDfuSetState(kState_AppIdle);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU APP_DETACH state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateAppDetach(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
                USB_DeviceDfuSetState(kState_AppDetach);
                break;
            case kUSB_DeviceDfuEventGetStateReq:
                USB_DeviceDfuSetState(kState_AppDetach);
                break;
            case kUSB_DeviceDfuEventDetachReq:
            case kUSB_DeviceDfuEventDnloadReq:
            case kUSB_DeviceDfuEventAbortReq:
            case kUSB_DeviceDfuEventUploadReq:

                USB_DeviceDfuSetState(kState_AppIdle);
                break;
            case kUSB_DeviceDfuEventDetachTimeout:
            default:
                USB_DeviceDfuSetState(kState_AppIdle);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU IDLE state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuIdle(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventDnloadReq:
                if ((event->wLength > 0) && USB_DFU_BIT_CAN_DNLOAD)
                {
                    /* update firmware block length */
                    s_UsbDeviceDfuDemo.dfuFirmwareBlockLength = event->wLength;
                    /* update firmware block status */
                    s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus = USB_DFU_BLOCK_TRANSFER_IN_PROGRESS;
                    /* update firmware size */
                    s_UsbDeviceDfuDemo.dfuFirmwareSize = 0U;
                    /* update current download length */
                    s_UsbDeviceDfuDemo.dfuIsTheFirstBlock = 1U;
                    /* reset the downloading flag */
                    s_UsbDeviceDfuDemo.dfuIsDownloadingFinished = 0U;
                    /* reset manifestation phase flag */
                    s_UsbDeviceDfuDemo.dfuManifestationPhaseStatus = USB_DFU_MANIFEST_UNDEFINED;
                    /* this time (5000 ms) is used to erase all the APP code region */
                    /* and flash the first firmware block data */
                    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[0] = 1000 & 0xFF;
                    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[1] = (1000 >> 8) & 0xFF;
                    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[2] = (1000 >> 16) & 0xFF;

                    USB_DeviceDfuSetState(kState_DfuDnLoadSync);
                }
                else
                {
                    USB_DeviceDfuSetState(kState_DfuError);
                }

                break;
            case kUSB_DeviceDfuEventUploadReq:
                if (USB_DFU_BIT_CAN_UPLOAD)
                {
                    USB_DeviceDfuSetState(kState_DfuUpLoadIdle);
                }
                else
                {
                    USB_DeviceDfuSetState(kState_DfuError);
                }

                USB_DeviceDfuSetState(kState_AppIdle);
                break;
            case kUSB_DeviceDfuEventAbortReq:
            case kUSB_DeviceDfuEventGetStatusReq:
            case kUSB_DeviceDfuEventGetStateReq:
                USB_DeviceDfuSetState(kState_DfuIdle);
                break;
            case kUSB_DeviceDfuEventDetachReq:
                USB_DeviceDfuSetState(kState_DfuIdle);
                break;
            default:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU DNLOAD_SYNC state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuDnLoadSync(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;
    dfu_timer_object_t dfuTimerObject;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
                if (USB_DFU_BLOCK_TRANSFER_IN_PROGRESS == s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus)
                {
                    dfuTimerObject.timerCount = (uint32_t)(
                        (uint32_t)(0xFFFFFF & ((uint32_t)s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[2] << 16U)) +
                        (uint32_t)(0xFFFF & ((uint32_t)s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[1] << 8U)) +
                        (uint32_t)(0xFF & (uint32_t)s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[0]));
                    dfuTimerObject.timerCount = (dfuTimerObject.timerCount >> 1);

                    dfuTimerObject.timerCallback = (dfu_timer_callback)USB_DeviceDfuPollTimeoutIsr;

                    s_UsbDeviceDfuDemo.dfuTimerId = DFU_AddTimerQueue(&dfuTimerObject);
                    USB_DeviceDfuSetState(kState_DfuDnBusy);
                }
                else if (USB_DFU_BLOCK_TRANSFER_COMPLETE == s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus)
                {
                    USB_DeviceDfuSetState(kState_DfuDnLoadIdle);
                }
                break;

            case kUSB_DeviceDfuEventGetStateReq:
                USB_DeviceDfuSetState(kState_DfuDnLoadSync);
                break;
            case kUSB_DeviceDfuEventDnloadReq:
            case kUSB_DeviceDfuEventUploadReq:
            case kUSB_DeviceDfuEventAbortReq:
            case kUSB_DeviceDfuEventDetachReq:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
            default:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU DNLOAD_BUSY state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuDnBusy(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
            case kUSB_DeviceDfuEventGetStateReq:
            case kUSB_DeviceDfuEventDnloadReq:
            case kUSB_DeviceDfuEventUploadReq:
            case kUSB_DeviceDfuEventAbortReq:
            case kUSB_DeviceDfuEventDetachReq:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
            case kUSB_DeviceDfuEventPollTimeout:
                USB_DeviceDfuSetState(kState_DfuDnLoadSync);
                break;
            default:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU DNLOAD_IDLE state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuDnLoadIdle(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;
    uint32_t status;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
            case kUSB_DeviceDfuEventGetStateReq:
                USB_DeviceDfuSetState(kState_DfuDnLoadIdle);
                break;
            case kUSB_DeviceDfuEventDnloadReq:
                if (event->wLength > 0)
                {
                    /* update firmware block length */
                    s_UsbDeviceDfuDemo.dfuFirmwareBlockLength = event->wLength;
                    /* update firmware block status */
                    s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus = USB_DFU_BLOCK_TRANSFER_IN_PROGRESS;
                    /* update timeout value */
                    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[0] = 0xc8;
                    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[1] = 0;
                    s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[2] = 0;
                    USB_DeviceDfuSetState(kState_DfuDnLoadSync);
                }
                else
                {
                    status = USB_DeviceDfuGetStatus();
                    if (USB_DFU_STATUS_ERR_NOT_DONE != status)
                    {
                        s_UsbDeviceDfuDemo.dfuFirmwareBlockStatus = USB_DFU_BLOCK_TRANSFER_UNDEFINED;
                        s_UsbDeviceDfuDemo.dfuManifestationPhaseStatus = USB_DFU_MANIFEST_IN_PROGRESS;
                        USB_DeviceDfuSetState(kState_DfuManifestSync);
                        s_UsbDeviceDfuDemo.dfuIsDownloadingFinished = 1;
                    }
                    else
                    {
                        USB_DeviceDfuSetState(kState_DfuError);
                    }
                }
                break;
            case kUSB_DeviceDfuEventAbortReq:
                USB_DeviceDfuSetState(kState_DfuIdle);
                break;
            case kUSB_DeviceDfuEventUploadReq:
            case kUSB_DeviceDfuEventDetachReq:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
            default:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU MANIFEST_SYNC state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuManifestSync(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;
    dfu_timer_object_t dfuTimerObject;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
                if (USB_DFU_MANIFEST_IN_PROGRESS == s_UsbDeviceDfuDemo.dfuManifestationPhaseStatus)
                {
                    USB_DeviceDfuSetState(kState_DfuManifest);
                    dfuTimerObject.timerCount = (uint32_t)(
                        (uint32_t)(0xFFFFFF & ((uint32_t)s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[2] << 16U)) +
                        (uint32_t)(0xFFFF & ((uint32_t)s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[1] << 8U)) +
                        (uint32_t)(0xFF & (uint32_t)s_UsbDeviceDfuDemo.dfuStatus.bwPollTimeout[0]));
                    dfuTimerObject.timerCount = (dfuTimerObject.timerCount >> 1);
                    dfuTimerObject.timerCallback = (dfu_timer_callback)USB_DeviceDfuPollTimeoutIsr;
                    s_UsbDeviceDfuDemo.dfuTimerId = DFU_AddTimerQueue(&dfuTimerObject);
                }
                else if (USB_DFU_MANIFEST_COMPLETE == s_UsbDeviceDfuDemo.dfuManifestationPhaseStatus)
                {
#if USB_DFU_BIT_MANIFESTATION_TOLERANT
                    USB_DeviceDfuSetState(kState_DfuIdle);

#endif
                }
                else
                {
                }
                break;
            case kUSB_DeviceDfuEventGetStateReq:
                USB_DeviceDfuSetState(kState_DfuManifestSync);
                break;
            case kUSB_DeviceDfuEventDnloadReq:
            case kUSB_DeviceDfuEventAbortReq:
            case kUSB_DeviceDfuEventUploadReq:
            case kUSB_DeviceDfuEventDetachReq:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
            default:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU MANIFEST state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuManifest(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
            case kUSB_DeviceDfuEventGetStateReq:
            case kUSB_DeviceDfuEventDnloadReq:
            case kUSB_DeviceDfuEventAbortReq:
            case kUSB_DeviceDfuEventUploadReq:
            case kUSB_DeviceDfuEventDetachReq:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
            case kUSB_DeviceDfuEventPollTimeout:
#if USB_DFU_BIT_MANIFESTATION_TOLERANT
                USB_DeviceDfuSetState(kState_DfuManifestSync);
#else
                USB_DeviceDfuSetState(kState_DfuManifestWaitReset);
                s_UsbDeviceDfuDemo.dfuReboot = 1;
#endif
                break;
            default:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU MANIFEST_WAIT_RESET state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuManifestWaitReset(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
                s_UsbDeviceDfuDemo.dfuReboot = 1;
            case kUSB_DeviceDfuEventGetStateReq:
            case kUSB_DeviceDfuEventDnloadReq:
            case kUSB_DeviceDfuEventAbortReq:
            case kUSB_DeviceDfuEventUploadReq:
            case kUSB_DeviceDfuEventDetachReq:
            case kUSB_DeviceDfuEventPollTimeout:
            default:
                USB_DeviceDfuSetState(kState_DfuManifestWaitReset);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU UPLOAD_IDLE state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuUpLoadIdle(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
            case kUSB_DeviceDfuEventGetStateReq:
                USB_DeviceDfuSetState(kState_DfuUpLoadIdle);
                break;
            case kUSB_DeviceDfuEventAbortReq:
                USB_DeviceDfuSetState(kState_DfuIdle);
                break;
            case kUSB_DeviceDfuEventUploadReq:

                if (s_isShortFrame)
                {
                    USB_DeviceDfuSetState(kState_DfuIdle);
                }
                else if (event->wLength > 0)
                {
                    USB_DeviceDfuSetState(kState_DfuUpLoadIdle);
                }
                break;
            case kUSB_DeviceDfuEventDnloadReq:
            case kUSB_DeviceDfuEventDetachReq:
            case kUSB_DeviceDfuEventPollTimeout:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
            default:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU ERROR state function.
 *
 * This function validates the event against the current state. And sets
 * the next state of the device.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
static usb_status_t USB_DeviceStateDfuError(usb_dfu_struct_t *dfu_dev, usb_device_dfu_event_struct_t *event)
{
    usb_status_t error = kStatus_USB_Success;

    if (NULL != event)
    {
        switch (event->name)
        {
            case kUSB_DeviceDfuEventGetStatusReq:
            case kUSB_DeviceDfuEventGetStateReq:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
            case kUSB_DeviceDfuEventClearStatusReq:
                USB_DeviceDfuSetState(kState_DfuIdle);
                break;
            case kUSB_DeviceDfuEventAbortReq:
            case kUSB_DeviceDfuEventUploadReq:
            case kUSB_DeviceDfuEventDnloadReq:
            case kUSB_DeviceDfuEventDetachReq:
            case kUSB_DeviceDfuEventPollTimeout:
                break;
            default:
                USB_DeviceDfuSetState(kState_DfuError);
                break;
        }
    }
    return error;
}

/*!
 * @brief DFU state update function.
 *
 * This function updates the DFU state according to the event.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceStateUpdate(void)
{
    usb_status_t error = kStatus_USB_Success;
    usb_device_dfu_event_struct_t event;
    usb_dfu_state_struct_t state;
    usb_dfu_struct_t *dfu_dev = &s_UsbDeviceDfuDemo;

    state = USB_DeviceDfuGetState();
    error = USB_DeviceDfuQueueGet(&s_DfuEventQueue, &event);
    if (kStatus_USB_Success == error)
    {
        if (kState_DfuError >= state)
        {
            s_dfuStateFunc[state](dfu_dev, &event);
        }
    }
    return error;
}

/*!
 * @brief DFU task function.
 *
 * This function gets the current state of the device, do downloading
 * or manifestation if it needs.
 *
 * @return None.
 */
void USB_DeviceDfuTask(void)
{
    usb_dfu_state_struct_t state;

    USB_DeviceStateUpdate();

    state = USB_DeviceDfuGetState();

    if (kState_DfuDnBusy == state)
    {
        USB_DeviceDfuDnload();
    }
    else if (kState_DfuManifest == state)
    {
        USB_DeviceDfuManifest();
    }
    if(s_UsbDeviceDfuDemo.crcCheck)
    {
        if (1 == s_UsbDeviceDfuDemo.dfuReboot)
        {
            s_UsbDeviceDfuDemo.dfuReboot = 0;

            USB_DeviceDfuSwitchMode();

        }
    }
}
