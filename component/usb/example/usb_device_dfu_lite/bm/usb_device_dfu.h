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

#ifndef __USB_DEVICE_DFU_H__
#define __USB_DEVICE_DFU_H__

/*!
 * @addtogroup usb_device_dfu_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief The class code of the DFU class */
#define USB_DEVICE_CONFIG_DFU_CLASS_CODE (0xFEU)

/*! @brief DFU class request */
#define USB_DEVICE_DFU_DETACH (0x00U)
#define USB_DEVICE_DFU_DNLOAD (0x01U)
#define USB_DEVICE_DFU_UPLOAD (0x02U)
#define USB_DEVICE_DFU_GETSTATUS (0x03U)
#define USB_DEVICE_DFU_CLRSTATUS (0x04U)
#define USB_DEVICE_DFU_GETSTATE (0x05U)
#define USB_DEVICE_DFU_ABORT (0x06U)

/*! @brief Available common EVENT types in dfu class callback */
typedef enum _usb_device_dfu_event
{
    kUSB_DeviceDfuEventDetach = 0x01U, /*!< Detach request */
    kUSB_DeviceDfuEventDownLoad,       /*!< Download request */
    kUSB_DeviceDfuEventUpLoad,         /*!< Upload request */
    kUSB_DeviceDfuEventGetStatus,      /*!< Get status request */
    kUSB_DeviceDfuEventClearStatus,    /*!< Clear status request */
    kUSB_DeviceDfuEventGetState,       /*!< Get state request */
    kUSB_DeviceDfuEventAbort,          /*!< Abort request */
} usb_device_dfu_event_t;



/*! @}*/

#endif /* __USB_DEVICE_DFU_H__ */
