/*
 * The Clear BSD License
 * Copyright 2016 - 2017 NXP
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

#ifndef __PD_CONFIG_H__
#define __PD_CONFIG_H__

#include "usb_pd.h"

/*!
 * @brief PD PTN5110 PHY driver instance count, meantime it indicates PTN5110 PHY driver enable or disable.
 *        - if 0, PTN5110 driver is disable.
 *        - if greater than 0, PTN5110 driver is enable.
 */
#define PD_CONFIG_PTN5110_PORT (1)

/*!
 * @brief PD stack instance max count.
 */
#define PD_CONFIG_MAX_PORT (PD_CONFIG_PTN5110_PORT)

/*! @brief PD revision that this stack support */
#define PD_CONFIG_REVISION (PD_SPEC_REVISION_20)

/*! @brief PD structured VDM version that this stack support */
#define PD_CONFIG_STRUCTURED_VDM_VERSION (PD_SPEC_STRUCTURED_VDM_VERSION_10)

/*!
 * @brief Enable CMSIS I2C driver.
 */
#define PD_CONFIG_CMSIS_I2C_INTERFACE (1)

/*!
 * @brief Enable CMSIS SPI driver.
 */
#define PD_CONFIG_CMSIS_SPI_INTERFACE (0)

/*!
 * @brief Enable PD stack source role function.
 */
#define PD_CONFIG_SOURCE_ROLE_ENABLE (1)

/*!
 * @brief Enable PD stack sink role function.
 */
#define PD_CONFIG_SINK_ROLE_ENABLE (0)

/*!
 * @brief Enable PD stack dual power role function.
 */
#define PD_CONFIG_DUAL_POWER_ROLE_ENABLE (0)

/*!
 * @brief Enable PD stack dual data role function.
 */
#define PD_CONFIG_DUAL_DATA_ROLE_ENABLE (0)

/*!
 * @brief Enable Vconn support (vconn_swap, vconn supply).
 */
#define PD_CONFIG_VCONN_SUPPORT (0)

/*!
 * @brief Enable Vconn discharge function.
 */
#define USBPD_ENABLE_VCONN_DISCHARGE (0)

/*!
 * @brief Enable vendor defined message function.
 */
#define PD_CONFIG_VENDOR_DEFINED_MESSAGE_ENABLE (1)

/*!
 * @brief Enable alternate mode function.
 */
#define PD_CONFIG_ALTERNATE_MODE_SUPPORT (0)

/*!
 * @brief Enable auto discovery cable plug function.
 */
#define PD_CONFIG_SRC_AUTO_DISCOVER_CABLE_PLUG (0)

/*!
 * @brief Enable cable communication function.
 */
#define PD_CONFIG_CABLE_COMMUNICATION_ENABLE (0)

/*!
 * @brief Config the detach detect way
 *  - PD_SINK_DETACH_ON_VBUS_ABSENT: detach is detected when vubs absent.
 *  - PD_SINK_DETACH_ON_CC_OPEN: detach is detected when CC is open.
 */
#define PD_CONFIG_SINK_DETACH_DETECT_WAY (PD_SINK_DETACH_ON_VBUS_ABSENT)

/*!
 * @brief Enable PD3.0 extended message function.
 */
#define PD_CONFIG_EXTENDED_MSG_SUPPORT (0)

/*!
 * @brief Enable fast role swap function.
 */
#define PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE (0)

/*!
 * @brief Enable PD3.0 AMS collision avoid function.
 */
#define PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE (0)

/*!
 * @brief Set the discharge time.
 * 1 - use the 14ms.
 * 0 - use the 650ms.
 */
#define PD_CONFIG_MIN_DISCHARGE_TIME_ENABLE (1)

/*!
 * @brief Enable Try.SNK function.
 */
#define PD_CONFIG_TRY_SNK_SUPPORT (0)

/*!
 * @brief Enable Try.SRC function.
 */
#define PD_CONFIG_TRY_SRC_SUPPORT (0)

/*!
 * @brief Enable sink accessory function.
 */
#define PD_CONFIG_SINK_ACCESSORY_SUPPORT (0)

/*!
 * @brief Enable audio accessory function.
 */
#define PD_CONFIG_AUDIO_ACCESSORY_SUPPORT (0)

/*!
 * @brief Enable debug accessory function.
 */
#define PD_CONFIG_DEBUG_ACCESSORY_SUPPORT (0)

/*!
 * @brief Enable compliance test function.
 */
#define PD_CONFIG_COMPLIANCE_TEST_ENABLE (0)

#endif
