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
#include "fsl_device_registers.h"
#include "board.h"
#include "dfu_timer.h"
#if ((defined DFU_MAX_TIMER_OBJECTS) && (DFU_MAX_TIMER_OBJECTS > 0))
#include "stdio.h"
#include "string.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief array of timer objects */
dfu_timer_object_t s_dfuTimerObjectArray[DFU_MAX_TIMER_OBJECTS];

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief timer initialization.
 *
 * This function initializes the timer object queue and system clock counter.
 *
 * @param controller_ID     the identify of timer controller.
 *
 * @retval success of error.
 */
void DFU_TimerInit(void)
{

    /* Clear timer object array */
    (void)memset(s_dfuTimerObjectArray, 0U, sizeof(s_dfuTimerObjectArray));
    DFU_TimerHWInit();
}

/*!
 * @brief add timer queue.
 *
 * This function is called to add timer object to timer queue.
 *
 * @param timerObject       Timer object.
 *
 * @retval timerIndex       The timer queue is not full.
 * @retval -1               The input timer object is NULL.
                            The timer queue is full.
 */
uint8_t DFU_AddTimerQueue(dfu_timer_object_t *timerObject)
{
    uint8_t index;
    index = DFU_MAX_TIMER_OBJECTS;
    if (NULL != timerObject)
    {
        /* Timer Index return value */
        uint8_t timerId = 0U;
        /* Queue full checking */
        uint8_t isQueueFull = 1U;
        /* Disable the timer */
        HW_TimerControl(0U);
        /* Add timerObject to queue */
        for (timerId = 0U; timerId < DFU_MAX_TIMER_OBJECTS; timerId++)
        {
            if (s_dfuTimerObjectArray[timerId].timerCallback == NULL)
            {
                isQueueFull = 0U;
                (void)memcpy(&s_dfuTimerObjectArray[timerId], timerObject, sizeof(dfu_timer_object_t));
                break;
            }
        }
        if (isQueueFull)
        {
            /* Timer queue is full */
           index = DFU_MAX_TIMER_OBJECTS;
        }
        else
        {
            /* only enable the timer if queue is not full*/
            HW_TimerControl(1U);
            index = timerId;
        }
    }
    /* Invalid parameter */
    return index;
}

/*!
 * @brief remove timer queue.
 *
 * This function is called to remove timer object from timer queue.
 *
 * @param timerId      index of timer object in queue.
 */
void DFU_RemoveTimerQueue(uint8_t timerId)
{
    if (timerId < DFU_MAX_TIMER_OBJECTS)
    {
        /* Disable the  timer */
        HW_TimerControl(0U);
        if (NULL != s_dfuTimerObjectArray[timerId].timerCallback)
        {
            /* Clear the time object in queue corresponding with timerId */
            (void)memset(&s_dfuTimerObjectArray[timerId], 0U, sizeof(dfu_timer_object_t));
            s_dfuTimerObjectArray[timerId].timerCallback = NULL;
        }
        /* Queue empty checking */
        for (uint8_t i = 0U; i < DFU_MAX_TIMER_OBJECTS; i++)
        {
            if (NULL != s_dfuTimerObjectArray[i].timerCallback)
            {
                /* Queue is not empty, enable the timer again */
                HW_TimerControl(1U);
                break;
            }
        }

    }
}


/*!
 * @brief timer interrupt service function.
 *
 * This function services programmable interrupt timer when a timer object
 * expired, then removes the timer object from timer queue and calls to the
 * callback function (if registered).
 */
void DFU_TimerISR(void)
{
    uint8_t index;
    for (index = 0U; index < DFU_MAX_TIMER_OBJECTS; index++)
    {
        if (NULL != s_dfuTimerObjectArray[index].timerCallback)
        {
            dfu_timer_object_t *timerObject = &s_dfuTimerObjectArray[index];
            timerObject->timerCount--;
            if (timerObject->timerCount <= 0U)
            {
                /* Call Pending Timer CallBacks */
                timerObject->timerCallback();
                /* remove timer object from timer queue */
                DFU_RemoveTimerQueue(index);
            }
        }
    }
}
#endif
