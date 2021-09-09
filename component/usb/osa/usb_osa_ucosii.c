/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "stdint.h"
#include "stdlib.h"
#include "fsl_device_registers.h"
#include "usb.h"
#include "usb_osa.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Constant to pass as timeout value to wait indefinitely. */
#define USB_OSA_WAIT_FOREVER 0xFFFFFFFFU

#define MSEC_TO_TICK(msec) ((1000L + ((uint32_t)OS_TICKS_PER_SEC * (uint32_t)(msec - 1U))) / 1000L)
#define TICKS_TO_MSEC(tick) ((tick)*1000uL / (uint32_t)OS_TICKS_PER_SEC)

/*! @brief Type for an event group object in uCOS-II */
typedef struct _usb_osa_event_struct
{
    OS_FLAG_GRP *handle; /*!< Pointer to uCOS-II's event entity */
    uint32_t flag;       /*!< Auto clear or manual clear        */
} usb_osa_event_struct_t;

/*! @brief Type for message queue in uCOS-II */
typedef struct _usb_osa_msgq_struct
{
    OS_EVENT *pQueue; /*!< Pointer to the queue                                     */
    void **msgTable;  /*!< Pointer to the array that saves the pointers to messages */
    OS_MEM *pMemory;  /*!< Pointer to memory where save the messages                */
    void *msgs;       /*!< Memory to save the messages                              */
    uint32_t size;    /*!< Size of the message in words                             */
} usb_osa_msgq_struct_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void *USB_OsaMemoryAllocate(uint32_t length)
{
    void *p;
    uint8_t *temp;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    p = (void *)malloc(length);
    USB_OSA_EXIT_CRITICAL();
    temp = (uint8_t *)p;
    if (p)
    {
        for (uint32_t count = 0U; count < length; count++)
        {
            temp[count] = 0U;
        }
    }
    return p;
}

void USB_OsaMemoryFree(void *p)
{
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    free(p);
    USB_OSA_EXIT_CRITICAL();
}

void USB_OsaEnterCritical(uint8_t *sr)
{
#if OS_CRITICAL_METHOD == 3u
    *sr = OS_CPU_SR_Save();
#elif OS_CRITICAL_METHOD == 4u
    OS_ENTER_CRITICAL();
#endif
}

void USB_OsaExitCritical(uint8_t sr)
{
#if OS_CRITICAL_METHOD == 3u
    OS_CPU_SR_Restore(sr);
#elif OS_CRITICAL_METHOD == 4u
    OS_EXIT_CRITICAL();
#endif
}

usb_osa_status_t USB_OsaEventCreate(usb_osa_event_handle *handle, uint32_t flag)
{
    usb_osa_event_struct_t *event;
    uint8_t error;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }
    event = (usb_osa_event_struct_t *)USB_OsaMemoryAllocate(sizeof(usb_osa_event_struct_t));
    if (NULL == event)
    {
        return kStatus_USB_OSA_Error;
    }

    event->handle = OSFlagCreate(0u, &error);
    if (OS_ERR_NONE != error)
    {
        USB_OsaMemoryFree(event);
        return kStatus_USB_OSA_Error;
    }
    event->flag = flag;
    *handle = event;
    return kStatus_USB_OSA_Success;
}

usb_osa_status_t USB_OsaEventDestroy(usb_osa_event_handle handle)
{
    usb_osa_event_struct_t *event = (usb_osa_event_struct_t *)handle;
    uint8_t error;
    if (handle)
    {
        if (event->handle)
        {
            OSFlagDel(event->handle, OS_DEL_ALWAYS, &error);
            if (OS_ERR_NONE != error)
            {
                return kStatus_USB_OSA_Error;
            }
        }
        USB_OsaMemoryFree(handle);
        return kStatus_USB_OSA_Success;
    }
    return kStatus_USB_OSA_Error;
}

usb_osa_status_t USB_OsaEventSet(usb_osa_event_handle handle, uint32_t bitMask)
{
    usb_osa_event_struct_t *event = (usb_osa_event_struct_t *)handle;
    uint8_t error;
    if (handle)
    {
        OSFlagPost(event->handle, bitMask, OS_FLAG_SET, &error);
        if (OS_ERR_NONE == error)
        {
            return kStatus_USB_OSA_Success;
        }
    }
    return kStatus_USB_OSA_Error;
}

usb_osa_status_t USB_OsaEventWait(
    usb_osa_event_handle handle, uint32_t bitMask, uint32_t flag, uint32_t timeout, uint32_t *bitSet)
{
    usb_osa_event_struct_t *event = (usb_osa_event_struct_t *)handle;
    OS_FLAGS bits;
    uint8_t error;
    uint8_t option;

    if (handle)
    {
        if (flag)
        {
            option = OS_FLAG_WAIT_SET_ALL;
        }
        else
        {
            option = OS_FLAG_WAIT_SET_ANY;
        }

        if (kUSB_OsaEventAutoClear == event->flag)
        {
            option |= OS_FLAG_CONSUME;
        }

        if (0U == timeout)
        {
            timeout = USB_OSA_WAIT_FOREVER; /* wait for event forever */
        }
        else
        {
            timeout = MSEC_TO_TICK(timeout);
        }
        bits = OSFlagPend(event->handle, (OS_FLAGS)bitMask, option, timeout, &error);

        if (OS_ERR_NONE == error)
        {
            *bitSet = ((uint32_t)bits) & bitMask;
            return kStatus_USB_OSA_Success;
        }
        else if ((OS_ERR_TIMEOUT == error) || (OS_ERR_FLAG_NOT_RDY == error))
        {
            return kStatus_USB_OSA_TimeOut;
        }
        else
        {
        }
    }
    return kStatus_USB_OSA_Error;
}

usb_osa_status_t USB_OsaEventCheck(usb_osa_event_handle handle, uint32_t bitMask, uint32_t *bitSet)
{
    usb_osa_event_struct_t *event = (usb_osa_event_struct_t *)handle;
    OS_FLAGS bits;
    uint8_t error;

    if (handle)
    {
        bits = OSFlagQuery(event->handle, &error);

        bits = (bits & bitMask);
        if (bits)
        {
            if (bitSet)
            {
                *bitSet = (uint32_t)bits;
            }
            return kStatus_USB_OSA_Success;
        }
    }
    return kStatus_USB_OSA_Error;
}

usb_osa_status_t USB_OsaEventClear(usb_osa_event_handle handle, uint32_t bitMask)
{
    usb_osa_event_struct_t *event = (usb_osa_event_struct_t *)handle;
    uint8_t error;

    if (handle)
    {
        OSFlagPost(event->handle, (OS_FLAGS)bitMask, OS_FLAG_CLR, &error);
        if (OS_ERR_NONE == error)
        {
            return kStatus_USB_OSA_Success;
        }
    }
    return kStatus_USB_OSA_Error;
}

usb_osa_status_t USB_OsaSemCreate(usb_osa_sem_handle *handle, uint32_t count)
{
    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }

    *handle = (usb_osa_sem_handle)OSSemCreate(count);
    if (NULL == (*handle))
    {
        return kStatus_USB_OSA_Error;
    }
    return kStatus_USB_OSA_Success;
}

usb_osa_status_t USB_OsaSemDestroy(usb_osa_sem_handle handle)
{
    uint8_t error;

    if (handle)
    {
        OSSemDel((OS_EVENT *)handle, OS_DEL_ALWAYS, &error);
        if (OS_ERR_NONE == error)
        {
            return kStatus_USB_OSA_Success;
        }
    }
    return kStatus_USB_OSA_Error;
}

usb_osa_status_t USB_OsaSemPost(usb_osa_sem_handle handle)
{
    OS_EVENT *sem = (OS_EVENT *)handle;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }

    if (OS_ERR_NONE == OSSemPost(sem))
    {
        return kStatus_USB_OSA_Success;
    }
    return kStatus_USB_OSA_Error;
}

usb_osa_status_t USB_OsaSemWait(usb_osa_sem_handle handle, uint32_t timeout)
{
    OS_EVENT *sem = (OS_EVENT *)handle;
    uint8_t error;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }

    if (!timeout)
    {
        timeout = USB_OSA_WAIT_FOREVER;
    }
    else
    {
        timeout = MSEC_TO_TICK(timeout);
    }

    OSSemPend(sem, timeout, &error);
    if (OS_ERR_NONE == error)
    {
        return kStatus_USB_OSA_Success;
    }
    else if (OS_ERR_TIMEOUT == error)
    {
        return kStatus_USB_OSA_TimeOut;
    }
    else
    {
        return kStatus_USB_OSA_Error;
    }
}

usb_osa_status_t USB_OsaMutexCreate(usb_osa_mutex_handle *handle)
{
    uint8_t error;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }
    /* NOTE: uC/OS-II does not support priority inherit but only priority protect */
    *handle = (usb_osa_mutex_handle)OSMutexCreate(OS_PRIO_MUTEX_CEIL_DIS, &error);
    if (OS_ERR_NONE == error)
    {
        return kStatus_USB_OSA_Success;
    }
    else
    {
        return kStatus_USB_OSA_Error;
    }
}

usb_osa_status_t USB_OsaMutexDestroy(usb_osa_mutex_handle handle)
{
    uint8_t error;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }
    OSMutexDel((OS_EVENT *)handle, OS_DEL_ALWAYS, &error);
    if (OS_ERR_NONE == error)
    {
        return kStatus_USB_OSA_Success;
    }
    else
    {
        return kStatus_USB_OSA_Error;
    }
}

usb_osa_status_t USB_OsaMutexLock(usb_osa_mutex_handle handle)
{
    OS_EVENT *mutex = (OS_EVENT *)handle;
    uint8_t error;

    if (handle)
    {
        return kStatus_USB_OSA_Error;
    }
    /* Avoid to get the mutex current task has already got. */
    if ((mutex->OSEventPtr) == ((void *)OSTCBCur))
    {
        return kStatus_USB_OSA_Success;
    }

    OSMutexPend(mutex, USB_OSA_WAIT_FOREVER, &error);

    if (OS_ERR_NONE == error)
    {
        return kStatus_USB_OSA_Success;
    }
    else if (OS_ERR_TIMEOUT == error)
    {
        return kStatus_USB_OSA_TimeOut;
    }
    else
    {
        return kStatus_USB_OSA_Error;
    }
}

usb_osa_status_t USB_OsaMutexUnlock(usb_osa_mutex_handle handle)
{
    if (handle)
    {
        return kStatus_USB_OSA_Error;
    }

    if (OS_ERR_NONE == OSMutexPost((OS_EVENT *)handle))
    {
        return kStatus_USB_OSA_Success;
    }

    return kStatus_USB_OSA_Error;
}

usb_osa_status_t USB_OsaMsgqCreate(usb_osa_msgq_handle *handle, uint32_t count, uint32_t size)
{
    usb_osa_msgq_struct_t *msgq;
    uint8_t *p;
    uint8_t error;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }
    msgq = (usb_osa_msgq_struct_t *)USB_OsaMemoryAllocate(sizeof(usb_osa_msgq_struct_t) +
                                                          ((size * sizeof(int32_t) + sizeof(void *)) * count));
    if (NULL == msgq)
    {
        return kStatus_USB_OSA_Error;
    }
    p = (uint8_t *)msgq;
    p += sizeof(usb_osa_msgq_struct_t);
    msgq->msgTable = (void **)p;
    p += sizeof(void *) * count;
    msgq->msgs = (void *)p;
    msgq->pQueue = OSQCreate(msgq->msgTable, count);
    if (!msgq->pQueue)
    {
        return kStatus_USB_OSA_Error;
    }
    /* Use OS_MEM to manage the memory which is used to save messages*/
    msgq->pMemory = OSMemCreate(msgq->msgs, count, count * sizeof(int32_t), &error);
    if (NULL == msgq->pMemory)
    {
        OSQDel(msgq->pQueue, OS_DEL_ALWAYS, &error);
        USB_OsaMemoryFree(msgq);
        return kStatus_USB_OSA_Error;
    }
    msgq->size = size;
    *handle = (usb_osa_msgq_handle)msgq;
    return kStatus_USB_OSA_Success;
}

usb_osa_status_t USB_OsaMsgqDestroy(usb_osa_msgq_handle handle)
{
    usb_osa_msgq_struct_t *msgq = (usb_osa_msgq_struct_t *)handle;
    uint8_t error;
    USB_OSA_SR_ALLOC();

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }
    OSQDel(msgq->pQueue, OS_DEL_ALWAYS, &error);
    if (OS_ERR_NONE != error)
    {
        return kStatus_USB_OSA_Error;
    }

    USB_OSA_ENTER_CRITICAL();
    msgq->pMemory->OSMemFreeList = OSMemFreeList;
    OSMemFreeList = msgq->pMemory;
    USB_OSA_EXIT_CRITICAL();

    USB_OsaMemoryFree(msgq);

    return kStatus_USB_OSA_Success;
}

usb_osa_status_t USB_OsaMsgqSend(usb_osa_msgq_handle handle, void *msg)
{
    usb_osa_msgq_struct_t *msgq = (usb_osa_msgq_struct_t *)handle;
    void *msgMemorybuffer = NULL;
    int32_t *sourceBuffer;
    int32_t *targetBuffer;
    uint32_t msgSize;
    uint8_t error;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }

    /* Get a memory to save the message. */
    msgMemorybuffer = OSMemGet(msgq->pMemory, &error);
    if (NULL == msgMemorybuffer)
    {
        return kStatus_USB_OSA_Error;
    }

    /* Copy msg to msgMemorybuffer */
    sourceBuffer = (int32_t *)msg;
    targetBuffer = (int32_t *)msgMemorybuffer;

    msgSize = msgq->size;

    while (msgSize--)
    {
        *targetBuffer++ = *sourceBuffer++;
    }

    if (OS_ERR_NONE != OSQPost(msgq->pQueue, msgMemorybuffer))
    {
        OSMemPut(msgq->pMemory, msgMemorybuffer);
        return kStatus_USB_OSA_Error;
    }

    return kStatus_USB_OSA_Success;
}

usb_osa_status_t USB_OsaMsgqRecv(usb_osa_msgq_handle handle, void *msg, uint32_t timeout)
{
    usb_osa_msgq_struct_t *msgq = (usb_osa_msgq_struct_t *)handle;
    void *msgMemorybuffer;
    int32_t *sourceBuffer;
    int32_t *targetBuffer;
    uint32_t msgSize;
    uint8_t error;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }

    if (!timeout)
    {
        timeout = USB_OSA_WAIT_FOREVER;
    }
    else
    {
        timeout = MSEC_TO_TICK(timeout);
    }
    msgMemorybuffer = OSQPend(msgq->pQueue, timeout, &error);

    msgSize = msgq->size;

    if (OS_ERR_NONE == error)
    {
        /* Copy msgMemorybuffer to msg */
        targetBuffer = (int32_t *)(msg);
        sourceBuffer = (int32_t *)msgMemorybuffer;
        while (msgSize--)
        {
            *targetBuffer++ = *sourceBuffer++;
        }

        OSMemPut(msgq->pMemory, msgMemorybuffer);

        return kStatus_USB_OSA_Success;
    }
    else if ((OS_ERR_Q_EMPTY == error) || (OS_ERR_TIMEOUT == error))
    {
        return kStatus_USB_OSA_TimeOut;
    }
    else
    {
        return kStatus_USB_OSA_Error;
    }
}

usb_osa_status_t USB_OsaMsgqCheck(usb_osa_msgq_handle handle, void *msg)
{
    usb_osa_msgq_struct_t *msgq = (usb_osa_msgq_struct_t *)handle;
    void *msgMemorybuffer;
    int32_t *sourceBuffer;
    int32_t *targetBuffer;
    uint32_t msgSize;
    uint8_t error;

    if (!handle)
    {
        return kStatus_USB_OSA_Error;
    }

    msgMemorybuffer = OSQAccept(msgq->pQueue, &error);

    msgSize = msgq->size;

    if (OS_ERR_NONE == error)
    {
        /* Copy msgMemorybuffer to msg */
        targetBuffer = (int32_t *)(msg);
        sourceBuffer = (int32_t *)msgMemorybuffer;
        while (msgSize--)
        {
            *targetBuffer++ = *sourceBuffer++;
        }

        OSMemPut(msgq->pMemory, msgMemorybuffer);

        return kStatus_USB_OSA_Success;
    }
    else if ((OS_ERR_Q_EMPTY == error) || (OS_ERR_TIMEOUT == error))
    {
        return kStatus_USB_OSA_TimeOut;
    }
    else
    {
        return kStatus_USB_OSA_Error;
    }
}
