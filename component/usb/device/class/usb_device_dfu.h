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

/*! @brief The DFU device class status structure */
typedef struct _usb_device_dfu_struct
{
    usb_device_handle handle;                       /*!< The device handle */
    usb_device_class_config_struct_t *configStruct; /*!< The configuration of the class. */
} usb_device_dfu_struct_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize the dfu class.
 *
 * This function is used to initialize the dfu class. This function only can be called by #USB_DeviceClassInit.
 *
 * @param[in] controllerId   The controller id of the USB IP. Please refer to the enumeration #usb_controller_index_t.
 * @param[in] config         The class configuration information.
 * @param[out] handle        It is out parameter, is used to return pointer of the dfu class handle to the caller.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceDfuInit(uint8_t controllerId,
                                      usb_device_class_config_struct_t *config,
                                      class_handle_t *handle);

/*!
 * @brief De-initialize the device dfu class.
 *
 * The function de-initializes the device dfu class. This function only can be called by #USB_DeviceClassDeinit.
 *
 * @param[in] handle The dfu class handle got from usb_device_class_config_struct_t::classHandle.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
extern usb_status_t USB_DeviceDfuDeinit(class_handle_t handle);

/*!
 * @brief Handle the event passed to the dfu class.
 *
 * This function handles the event passed to the dfu class. This function can only be called by #USB_DeviceClassEvent.
 *
 * @param[in] handle          The dfu class handle, got from the usb_device_class_config_struct_t::classHandle.
 * @param[in] event           The event codes. Please refer to the enumeration usb_device_class_event_t.
 * @param[in,out] param       The param type is determined by the event code.
 *
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success              Free device handle successfully.
 * @retval kStatus_USB_InvalidParameter     The device handle not be found.
 * @retval kStatus_USB_InvalidRequest       The request is invalid, and the control pipe will be stalled by the caller.
 */
extern usb_status_t USB_DeviceDfuEvent(void *handle, uint32_t event, void *param);

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __USB_DEVICE_DFU_H__ */
