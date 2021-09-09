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

#ifndef __USB_DFU_H__
#define __USB_DFU_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Pragma to put code in m_usb_dfu section. */
#if defined(__ICCARM__)

#define USB_DFU _Pragma("location = \"m_usb_dfu\"")

#elif defined(__CC_ARM)

#define USB_DFU __attribute__((section("m_usb_dfu"))) __attribute__((zero_init))

#elif defined(__GNUC__)

#define USB_DFU __attribute__((section("m_usb_dfu")))

#else
#error The tool-chain is not supported.
#endif

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
#define CONTROLLER_ID kUSB_ControllerEhci0
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
#define CONTROLLER_ID kUSB_ControllerKhci0
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Fs0
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Hs0
#endif

#define USB_DEVICE_INTERRUPT_PRIORITY (3U)
/*! @brief DFU application address */
#define USB_DFU_APP_ADDRESS (0x10000U)
#define USB_DFU_APP_SIZE (0x8000U)
/*! @brief DFU status definition */
#define USB_DFU_STATUS_OK (0x00U)         /*!< No error condition is present */
#define USB_DFU_STATUS_ERR_TARGET (0x01U) /*!< File is not targeted for use by this device */
#define USB_DFU_STATUS_ERR_FILE                                                                                       \
    (0x02U) /*!< File is for this device but fails some vendor-specific verification test \ \ \ \ \ \ \ \ \ \ \ \ \ \ \
             * \                                                                                                      \
             * \ \                                                                                                                    \
             * \ \ \                                                                                                                    \
             * \ \ \ \                                                                                                                    \
               */
#define USB_DFU_STATUS_ERR_WRITE (0x03U)        /*!< Device is unable to write memory */
#define USB_DFU_STATUS_ERR_ERASE (0x04U)        /*!< Memory erase function failed */
#define USB_DFU_STATUS_ERR_CHECK_ERASED (0x05U) /*!< Memory erase check failed */
#define USB_DFU_STATUS_ERR_PROG (0x06U)         /*!< Program memory function failed */
#define USB_DFU_STATUS_ERR_VERIFY (0x07U)       /*!< Programed memory failed verification */
#define USB_DFU_STATUS_ERR_ADDRESS (0x08U)      /*!< Received address is out of range */
#define USB_DFU_STATUS_ERR_NOT_DONE \
    (0x09U) /*!< Received DFU DNLOAD request with length = 0 but device does not think it has all of the data yet */
#define USB_DFU_STATUS_ERR_FIRMWARE (0x0AU)   /*!< Firmware is corrupt */
#define USB_DFU_STATUS_ERR_VENDOR (0x0BU)     /*!< Vendor specific error */
#define USB_DFU_STATUS_ERR_USBR (0x0CU)       /*!< Detect unexpected USB reset signaling */
#define USB_DFU_STATUS_ERR_POR (0x0DU)        /*!< Detect unexpected power on reset */
#define USB_DFU_STATUS_ERR_UNKNOWN (0x0EU)    /*!< Unknown error */
#define USB_DFU_STATUS_ERR_STALLEDPKT (0x0FU) /*!< Device stalled an unexpected request */

/*! @brief DFU Block status */
#define USB_DFU_BLOCK_TRANSFER_COMPLETE (0x00U)    /*!< Block transfer complete */
#define USB_DFU_BLOCK_TRANSFER_IN_PROGRESS (0x01U) /*!< Block transfer is still in progress */
#define USB_DFU_BLOCK_TRANSFER_UNDEFINED (0xFFU)   /*!< Block transfer status undefined */

/*! @brief DFU manifestation phase status */
#define USB_DFU_MANIFEST_COMPLETE (0x00U)    /*!< Manifestation phase complete */
#define USB_DFU_MANIFEST_IN_PROGRESS (0x01U) /*!< Manifestation phase in progress */
#define USB_DFU_MANIFEST_UNDEFINED (0xFFU)   /*!< Manifestation phase undefined */

/*! @brief DFU vendor request */
#define USB_DFU_VENDOR_REQUEST_SET_SUFFIX (0x00U) /*!< Set DFU file suffix request */
#define USB_DFU_VENDOR_REQUEST_GET_SUFFIX (0x01U) /*!< Get DFU file suffix request */

#define DFU_EVENT_QUEUE_MAX (16)

/*! @brief DFU state definition. */
typedef enum _usb_dfu_state_struct
{
    kState_AppIdle = 0U,         /* App idle */
    kState_AppDetach,            /* App detach */
    kState_DfuIdle,              /* DFU idle */
    kState_DfuDnLoadSync,        /* DFU dnload sync */
    kState_DfuDnBusy,            /* DFU dnload busy */
    kState_DfuDnLoadIdle,        /* DFU dnload idle */
    kState_DfuManifestSync,      /* DFU manifest sync */
    kState_DfuManifest,          /* DFU manifest */
    kState_DfuManifestWaitReset, /* DFU manifest wait reset */
    kState_DfuUpLoadIdle,        /* DFU upload idle */
    kState_DfuError,
} usb_dfu_state_struct_t;

/*! @brief DFU file suffix definition. */
typedef struct _usb_dfu_suffix_struct
{
    uint8_t bcdDevice[2];      /* release number of the device associated with firmware file */
    uint8_t idProduct[2];      /* product ID */
    uint8_t idVendor[2];       /* Vendor ID */
    uint8_t bcdDFU[2];         /* DFU specification number */
    uint8_t ucDfuSignature[3]; /* DFU signature */
    uint8_t bLength;           /* Length of DFU suffix */
    uint8_t dwCRC[4];          /* The CRC of entire file */
} usb_dfu_suffix_struct_t;

/*! @brief DFU status definition. */
typedef struct _usb_dfu_status_struct
{
    uint8_t bStatus;           /* status result */
    uint8_t bwPollTimeout[3U]; /* The minimum time host should wait before sending
                                  a subsequent DFU GETSTATUS request */
    uint8_t bState;            /* dfu state */
    uint8_t iString;           /* Index of status description in string table */
} usb_dfu_status_struct_t;

/*! @brief DFU device definition. */
typedef struct _usb_dfu_struct
{
    usb_dfu_status_struct_t dfuStatus;
    uint32_t dfuFirmwareBlockLength;
    uint32_t dfuIsTheFirstBlock;
    uint32_t dfuCRC;
    uint8_t *dfuFirmwareBlock;
    uint8_t dfuFirmwareBlockStatus;
    uint8_t dfuIsDownloadingFinished;
    uint8_t dfuManifestationPhaseStatus;
    uint32_t dfuFirmwareAddress;
    uint32_t dfuFirmwareSize;
    uint32_t dfuCurrentUploadLenght;
    uint8_t dfuSuffix[0xFF];
    uint8_t dfuReboot;
    uint8_t crcCheck;
    uint8_t dfuTimerId;
} usb_dfu_struct_t;

/*! @brief DFU device event definition. */
typedef enum _usb_device_dfu_state_event
{
    kUSB_DeviceDfuEventDetachReq,
    kUSB_DeviceDfuEventGetStatusReq,
    kUSB_DeviceDfuEventClearStatusReq,
    kUSB_DeviceDfuEventGetStateReq,
    kUSB_DeviceDfuEventDnloadReq,
    kUSB_DeviceDfuEventAbortReq,
    kUSB_DeviceDfuEventUploadReq,
    kUSB_DeviceDfuEventDetachTimeout,
    kUSB_DeviceDfuEventPollTimeout,
} usb_device_dfu_state_event_t;

/* Disable interrupt to enter critical section. */
#define USB_DEVICE_DFU_ENTER_CRITICAL() \
    USB_OSA_SR_ALLOC();                 \
    USB_OSA_ENTER_CRITICAL()

/* Enable interrupt to exit critical section. */
#define USB_DEVICE_DFU_EXIT_CRITICAL() USB_OSA_EXIT_CRITICAL()

/* Define DFU event struct */
typedef struct _usb_device_dfu_event_struct
{
    usb_device_dfu_state_event_t name;
    uint16_t wValue;
    uint16_t wLength;
} usb_device_dfu_event_struct_t;

/* Define DFU_ENET queue struct */
typedef struct _dfu_queue
{
    uint32_t head;
    uint32_t tail;
    uint32_t maxSize;
    uint32_t curSize;
    usb_osa_mutex_handle mutex;
    usb_device_dfu_event_struct_t qArray[DFU_EVENT_QUEUE_MAX];
} dfu_queue_t;

/*******************************************************************************
* API
******************************************************************************/

/*!
 * @brief Initialize the queue.
 *
 * @return Error code.
 */
static inline usb_status_t USB_DeviceDfuQueueInit(dfu_queue_t *q)
{
    usb_status_t error = kStatus_USB_Error;
    USB_DEVICE_DFU_ENTER_CRITICAL();
    (q)->head = 0;
    (q)->tail = 0;
    (q)->maxSize = DFU_EVENT_QUEUE_MAX;
    (q)->curSize = 0;
    if (kStatus_USB_OSA_Success != USB_OsaMutexCreate(&((q)->mutex)))
    {
        usb_echo("queue mutex create error!");
    }
    error = kStatus_USB_Success;
    USB_DEVICE_DFU_EXIT_CRITICAL();
    return error;
}

/*!
 * @brief Delete the queue.
 *
 * @return Error code.
 */
static inline usb_status_t USB_DeviceDfuQueueDelete(dfu_queue_t *q)
{
    usb_status_t error = kStatus_USB_Error;
    USB_DEVICE_DFU_ENTER_CRITICAL();
    (q)->head = 0;
    (q)->tail = 0;
    (q)->maxSize = 0;
    (q)->curSize = 0;
    error = kStatus_USB_Success;
    USB_DEVICE_DFU_EXIT_CRITICAL();
    return error;
}

/*!
 * @brief Check if the queue is empty.
 *
 * @return 1: queue is empty, 0: not empty.
 */
static inline uint8_t USB_DeviceDfuQueueIsEmpty(dfu_queue_t *q)
{
    return ((q)->curSize == 0) ? 1 : 0;
}

/*!
 * @brief Check if the queue is full.
 *
 * @return 1: queue is full, 0: not full.
 */
static inline uint8_t USB_DeviceDfuQueueIsFull(dfu_queue_t *q)
{
    return ((q)->curSize >= (q)->maxSize) ? 1 : 0;
}

/*!
 * @brief Get the size of the queue.
 *
 * @return Size of the quue.
 */
static inline uint32_t USB_DeviceDfuQueueSize(dfu_queue_t *q)
{
    return (q)->curSize;
}

/*!
 * @brief Put element into the queue.
 *
 * @return Error code.
 */
static inline usb_status_t USB_DeviceDfuQueuePut(dfu_queue_t *q, usb_device_dfu_event_struct_t *e)
{
    usb_status_t error = kStatus_USB_Error;
    USB_DEVICE_DFU_ENTER_CRITICAL();
    if (0 == USB_DeviceDfuQueueIsFull(q))
    {
        (q)->qArray[(q)->head++] = *(e);
        if ((q)->head == (q)->maxSize)
        {
            (q)->head = 0;
        }
        (q)->curSize++;
        error = kStatus_USB_Success;
    }
    USB_DEVICE_DFU_EXIT_CRITICAL();
    return error;
}

/*!
 * @brief Get element from the queue.
 *
 * @return Error code.
 */
static inline usb_status_t USB_DeviceDfuQueueGet(dfu_queue_t *q, usb_device_dfu_event_struct_t *e)
{
    usb_status_t error = kStatus_USB_Error;
    USB_DEVICE_DFU_ENTER_CRITICAL();
    if (0 == USB_DeviceDfuQueueIsEmpty(q))
    {
        *(e) = (q)->qArray[(q)->tail++];
        if ((q)->tail == (q)->maxSize)
        {
            (q)->tail = 0;
        }
        (q)->curSize--;
        error = kStatus_USB_Success;
    }
    USB_DEVICE_DFU_EXIT_CRITICAL();
    return error;
}

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

extern usb_status_t USB_DeviceDfuDemoCallback(class_handle_t handle, uint32_t event, void *param);
extern usb_status_t USB_DeviceDfuDemoVendorCallback(usb_device_handle handle, void *param);
extern void USB_DeviceDfuBusReset(void);
extern void USB_DeviceDfuSwitchMode(void);
extern void USB_DeviceDfuDemoInit(void);
extern void USB_DeviceDfuTask(void);

#if defined(__cplusplus)
}
#endif
#endif /* __USB_DFU_H__ */
