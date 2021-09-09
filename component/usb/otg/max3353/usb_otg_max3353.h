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

#ifndef __USB_OTG_MAX3353_H__
#define __USB_OTG_MAX3353_H__

#include "usb_otg_config.h"

#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* MAX3353 registers address */
#define MAX3353_CONTROL1_ADDRESS 0x10
#define MAX3353_CONTROL2_ADDRESS 0x11
#define MAX3353_STATUS_ADDRESS 0x13
#define MAX3353_INTERRUPT_MASK_ADDRESS 0x14
#define MAX3353_INTERRUPT_EDGE_ADDRESS 0x15
#define MAX3353_INTERRUPT_LATCH_ADDRESS 0x16

/* status register bit */
#define MAX3353_STATUS_VBUS_VALID_MASK (0x01U)
#define MAX3353_STATUS_SESSION_VALID_MASK (0x02U)
#define MAX3353_STATUS_SESSION_END_MASK (0x04U)
#define MAX3353_STATUS_ID_GND_MASK (0x08U)
#define MAX3353_STATUS_ID_FLOAT_MASK (0x10U)
#define MAX3353_STATUS_A_HNP_MASK (0x20U)
#define MAX3353_STATUS_B_HNP_MASK (0x40U)

#define MAX3353_STATUS_VBUS_VALID_BIT (0U)
#define MAX3353_STATUS_SESSION_VALID_BIT (1U)
#define MAX3353_STATUS_SESSION_END_BIT (2U)
#define MAX3353_STATUS_ID_GND_BIT (3U)
#define MAX3353_STATUS_ID_FLOAT_BIT (4U)
#define MAX3353_STATUS_A_HNP_BIT (5U)
#define MAX3353_STATUS_B_HNP_BIT (6U)

/* control2 register bit */
#define MAX3353_CONTROL2_SDWN_MASK (0x01U)
#define MAX3353_CONTROL2_VBUS_CHG1_MASK (0x02U)
#define MAX3353_CONTROL2_VBUS_CHG2_MASK (0x04U)
#define MAX3353_CONTROL2_VBUS_DRV_MASK (0x08U)
#define MAX3353_CONTROL2_VBUS_DISCHG_MASK (0x10U)

/* control1 register bit */
#define MAX3353_CONTROL1_IRQ_MODE_MASK (0x02U)
#define MAX3353_CONTROL1_BDISC_ACONN_MASK (0x04U)
#define MAX3353_CONTROL1_DP_PULLUP_MASK (0x10U)
#define MAX3353_CONTROL1_DM_PULLUP_MASK (0x20U)
#define MAX3353_CONTROL1_DP_PULLDWN_MASK (0x40U)
#define MAX3353_CONTROL1_DM_PULLDWN_MASK (0x80U)

/* The status for controling max3353 */
typedef struct _usb_otg_max3353_run
{
    uint8_t statusRegister;
    uint8_t control1Register;
    uint8_t control2Register;
    uint8_t peripheralStatus;
} usb_otg_max3353_run_t;
/*******************************************************************************
 * API
 ******************************************************************************/

/* Initialize max3353, call from application */
extern usb_status_t USB_OtgMax3353Init(void);

#endif

#endif
