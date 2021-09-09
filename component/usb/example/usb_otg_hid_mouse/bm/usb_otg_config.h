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

#ifndef __USB_0TG_CONFIG_H__
#define __USB_0TG_CONFIG_H__

/*!
 * @brief otg khci instance count, meantime it indicates khci enable or disable.
 *        - if 0, otg khci driver is disable.
 *        - if greater than 0, otg khci driver is enable.
 */
#define USB_OTG_CONFIG_KHCI (1U)

/*! if 1, the otg srp is enable; if 0, the otg srp is disbale */
#define USB_OTG_SRP_ENABLE (1U)

/*! if 1, the otg hnp is enable; if 0, the otg hnp is disbale */
#define USB_OTG_HNP_ENABLE (1U)

/*! if 1, the otg adp is enable; if 0, the otg adp is disbale */
#define USB_OTG_ADP_ENABLE (0U)

#if ((defined USB_OTG_CONFIG_KHCI) && (USB_OTG_CONFIG_KHCI))
/*!
 * @brief otg khci instance count, meantime it indicates khci enable or disable.
 *        - if 0, otg khci driver is disable.
 *        - if greater than 0, otg khci driver is enable.
 */
#define USB_OTG_KHCI_PERIPHERAL_ENABLE (1U)

#endif

#endif
