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

#ifndef __USB_DEVICE_PRINTER_H__
#define __USB_DEVICE_PRINTER_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief The class code of the printer class */
#define USB_DEVICE_CONFIG_PRINTER_CLASS_CODE (0x07)

/*! @brief class-specific request GET_DEVICE_ID */
#define USB_DEVICE_PRINTER_GET_DEVICE_ID (0x00U)
/*! @brief class-specific request GET_PORT_STATUS */
#define USB_DEVICE_PRINTER_GET_PORT_STATUS (0x01U)
/*! @brief class-specific request SOFT_RESET */
#define USB_DEVICE_PRINTER_SOFT_RESET (0x02U)

/*! @brief Paper empty bit mask for GET_PORT_STATUS */
#define USB_DEVICE_PRINTER_PORT_STATUS_PAPER_EMPTRY_MASK (0x20U)
/*! @brief Select bit mask for GET_PORT_STATUS */
#define USB_DEVICE_PRINTER_PORT_STATUS_SELECT_MASK (0x10U)
/*! @brief Error bit mask for GET_PORT_STATUS */
#define USB_DEVICE_PRINTER_PORT_STATUS_NOT_ERROR_MASK (0x08U)

#define USB_DEVICE_PRINTER_PORT_STATUS_DEFAULT_VALUE \
    (USB_DEVICE_PRINTER_PORT_STATUS_SELECT_MASK | USB_DEVICE_PRINTER_PORT_STATUS_NOT_ERROR_MASK)

#endif /* __USB_DEVICE_PRINTER_H__ */
