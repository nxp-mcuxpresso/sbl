/*
 * The Clear BSD License
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

#ifndef __PD_TYPEC_CROSSBAR_H__
#define __PD_TYPEC_CROSSBAR_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Register indexes */
#define SYS_CTRL_IDX 0x01
#define OP1_CTRL_IDX 0x02
#define OP2_CTRL_IDX 0x03
#define OP3_CTRL_IDX 0x04
#define OP4_CTRL_IDX 0x05
#define OP5_CTRL_IDX 0x06
#define CROSS_CTRL_IDX 0x07
#define SW_CTRL_IDX 0x08
#define REVISION_IDX 0x09

#define SYS_CTRL_SW_EN_MASK 0x80

/* For  registers OP{1,2}_CTRL */
#define OP_CTRL_IP1_MASK 0x01
#define OP_CTRL_IP2_MASK 0x02
#define OP_CTRL_IP3_MASK 0x04

/* For  registers OP{3,4}_CTRL */
#define OP_CTRL_IP4_MASK 0x08
#define OP_CTRL_IP5_MASK 0x10
#define OP_CTRL_IP6_MASK 0x20

/* For  register OP5_CTRL */
#define OP_CTRL_IP7_MASK 0x40
#define OP_CTRL_IP8_MASK 0x80

/* For CROSS_CTRL */
#define CROSS_CTRL_PASS 0x01
#define CROSS_CTRL_CROSS 0x02

/* For SW_CTRL */
#define SW_CTRL_OP1_SET_MASK 0x01
#define SW_CTRL_OP2_SET_MASK 0x02
#define SW_CTRL_OP3_SET_MASK 0x04
#define SW_CTRL_OP4_SET_MASK 0x08
#define SW_CTRL_OP_1TO4_MASK (SW_CTRL_OP1_SET_MASK | SW_CTRL_OP2_SET_MASK | SW_CTRL_OP3_SET_MASK | SW_CTRL_OP4_SET_MASK)
#define SW_CTRL_OP5_SET_MASK 0x10
#define SW_CTRL_X5_SET_MASK 0x20

/* For REVISION */
#define REVISION_ID 0xA0

/** Pre defined multiplexing for alternate modes
 * Used to control GPIO and/or external i2c interface.
 * By default these are set with CrossbarSetMux() function.
 */
typedef enum MuxControl
{
    /*! Power down multipexer device. */
    /*! XSDN negated */
    MUX_SHUTDOWN,
    /*! Disable multiplexer device. Temporary state that is not powered down. */
    /*! XSDN negated (to support Rev2 apps board), muxes isolated. */
    MUX_DISABLED,
    /*! Place Re-purposed SuperSpeed Signals into the Safe State and Isolate SBUs. */
    /*! Behaviour may change depending on the previous multiplexer state. */
    MUX_SAFE_MODE,
    /*! USB3 only. This enables standard USB type c operation before an alternate mode is entered. */
    MUX_USB3_ONLY,
    /*! 2 Lane Display Port and USB3. */
    /*! Used to implement the multi function mode in the display port alternate mode. */
    /*! Must be set with CrossbarSetMuxWithPDO() to specify the pin scheme. */
    MUX_DP2_LANE_USB3,
    /*! 2 Lane Display Port and USB3 disabled. */
    /*! Used to implement authentication control, where USB is disabled. */
    /*! Must be set with CrossbarSetMuxWithPDO() to specify the pin scheme. */
    MUX_DP2_LANE_NO_USB,
    /*! 4 Lane Display Port. */
    /*! SUPERSPEED_USB can not function in this setting. */
    /*! Must be set with CrossbarSetMuxWithPDO() to specify the pin scheme. */
    MUX_DP4_LANE,
#if 0
    /*! Thunderbolt alternate mode */
    /*! */
    MUX_THUNDERBOLT_MODE,
    /*! Thunderbolt alternate mode, when USB2 is enabled for billboard display. */
    MUX_THUNDERBOLT_USB2_MODE,
#endif

} MuxControl_t;

typedef enum _pd_typec_crossbar_set_val
{
    kTypeCSignal_SafeSate = 0,
    kTypeCSignal_USB,
    kTypeCSignal_DP,
} pd_typec_crossbar_set_val_t;

typedef struct _pd_crossbar_config
{
    uint8_t crossbarSlaveAddress;
    uint8_t crossbarControlGPIO;
    uint8_t crossbarControlPort;
    uint8_t crossbarControlPin;
} pd_crossbar_config_t;

typedef struct _pd_crossbar_instance
{
    pd_handle pdHandle;
    usb_cmsis_wrapper_handle cmsisHandle;
    const pd_crossbar_config_t *crossbarConfig;
    uint8_t crossEnablePinVal;
    uint8_t preControlMux;

    uint8_t crossbarRegs[SW_CTRL_IDX + 1];
} pd_crossbar_instance_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/* 0 - intialize success; other values - fail */
uint8_t PD_CrossbarInit(pd_crossbar_instance_t *crossInstance, pd_handle pdHandle, void *config);
/* 0 - de-intialize success; other values - fail */
uint8_t PD_CrossbarDeinit(pd_crossbar_instance_t *crossInstance);

void PD_CrossbarSetMux(pd_crossbar_instance_t *crossInstance, uint8_t mux, uint32_t modeVDO);
#endif
