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

#ifndef __USB_OTG_PERIPHERAL_H__
#define __USB_OTG_PERIPHERAL_H__

#include "usb_otg_config.h"

/*!
 * @addtogroup usb_otg_peripheral_driver
 * @{
 */

#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))

/*!
 * @brief Enable OTG peripheral.
 *
 * This function enable OTG peripheral function.
 *
 * @retval kStatus_USB_Success              success.
 * @retval other values                     Fail.
 */
usb_status_t USB_OtgPeripheralEnable(void);

/*!
 * @brief Disable OTG peripheral.
 *
 * This function disable OTG peripheral function.
 *
 * @retval kStatus_USB_Success              success.
 * @retval other values                     Fail.
 */
usb_status_t USB_OtgPeripheralDisable(void);

/*!
 * @brief Get the peripheral status.
 *
 * This function is nonblocking, return the result immediately.
 *
 * @param[in] statusType      Please reference to #usb_otg_status_type_t.
 * @param[out] statusValue    The status value.
 *
 * @retval kStatus_USB_Success              success.
 * @retval other values                     Fail.
 */
usb_status_t USB_OtgPeripheralGetStatus(uint32_t statusType, uint32_t *statusValue);

/*!
 * @brief Control the peripheral.
 *
 * This function control the peripheral to implement the different functions.
 *
 * @param controllerHandle  The controller instance handle.
 * @param controlType       The control type, please reference to #usb_otg_control_t.
 * @param controlValue1     The control value, it is 0 or 1 usually.
 * @param controlValue2     It only be used in the kOtg_ControlRequestStatus control now.
 *
 * @retval kStatus_USB_Success              success.
 * @retval other values                     Fail.
 */
usb_status_t USB_OtgPeripheralControl(usb_otg_controller_handle controllerHandle,
                                      uint32_t controlType,
                                      uint32_t controlValue1,
                                      uint32_t controlValue2);

#endif

/*! @}*/

#endif
