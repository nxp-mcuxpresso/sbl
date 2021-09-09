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

#ifndef __PD_DP_ALT_MODE_H__
#define __PD_DP_ALT_MODE_H__

#include "pd_typec_crossbar.h"
#include "pd_dp_hpd_driver.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PD_DP_MAX_PORT (4)

/* #define DP_TASK_EVENT_DFP_RETRY_COMMAND (0x00000001u) */
/* #define DP_TASK_EVENT_DFP_DP_CHECK_UFP_STATUS (0x00000002u) */
/* #define DP_TASK_EVENT_DFP_SEND_COMMAND (0x00000004u) */
/* #define DP_TASK_EVENT_DFP_EXIT_MODE_TRIGGER (0x00000008u) */
#define DP_TASK_EVENT_DFP_HPD_PROCESS (0x00000010u))

typedef enum _pd_displayport_vdm_command
{
    kDPVDM_StatusUpdate = 0x10,
    kDPVDM_Configure,
} pd_displayport_vdm_command_t;

/* displayport alt mode's state (enter, configured etc) */
typedef enum _pd_dp_mode_state
{
    kDPMode_Invalid = 0,
    kDPMode_Exited,
    kDPMode_EnterDPDone,
    kDPMode_StatusUpdateDone,
    kDPMode_ConfigureDone,

} pd_dp_mode_state_t;

typedef enum _pd_status_connected_val
{
    kDFP_D_NonConnected = 0,
    kDFP_D_Connected = 1,
    kUFP_D_Connected = 2,
    kUFP_D_BothConnected = 3,
} pd_status_connected_val_t;

typedef enum _pd_configure_set_config_val
{
    kDPConfig_USB = 0,
    kDPConfig_DFPD = 1,
    kDPConfig_UFPD = 2,
} pd_configure_set_config_val_t;

typedef enum _dp_mode_port_cap_val
{
    kDPPortCap_UFPD = 1,
    kDPPortCap_DFPD = 2,
    kDPPortCap_Both = 3,
} dp_mode_port_cap_val_t;

typedef enum _dp_mode_signal_val
{
    kDPSignal_Unspecified = 0,
    kDPSignal_DP = 1,
    kDPSignal_USBGEN2 = 2,
} dp_mode_signal_t;

typedef enum _dp_mode_pin_assign_val
{
    kPinAssign_DeSelect = 0,
    /* GEN2_BR signal, 4 lanes */
    kPinAssign_A = 0x01u,
    /* GEN2_BR signal, 2 lanes */
    kPinAssign_B = 0x02u,
    /* DP_BR signal, 4 lanes */
    kPinAssign_C = 0x04u,
    /* DP_BR signal, 2 lanes */
    kPinAssign_D = 0x08u,
    /* DP_BR signal, 4 lanes */
    kPinAssign_E = 0x10u,
    kPinAssign_F = 0x20u,
    kPinAssign_All = 0xFFu,
} dp_mode_pin_assign_val_t;

typedef struct _pd_dp_mode_obj
{
    union
    {
        /*! union DP mode object */
        struct
        {
            uint32_t portCap : 2;
            uint32_t signalSupport : 4;
            uint32_t receptacleIndication : 1;
            uint32_t usb2SignalNotUsed : 1;
            uint32_t DFPDPinSupport : 8;
            uint32_t UFPDPinSupport : 8;
            uint32_t reseerved : 8;
        } bitFields;
        /*! union 32bits value */
        uint32_t modeVal;
    };
} pd_dp_mode_obj_t;

typedef struct _pd_dp_status_obj
{
    union
    {
        /*! union DP status object */
        struct
        {
            uint32_t DFPDUFPDConnected : 2;
            uint32_t powerLow : 1;
            uint32_t enabled : 1;
            uint32_t multiFunctionPreferred : 1;
            uint32_t USBConfigReq : 1;
            uint32_t exitDPModeReq : 1;
            uint32_t HPDState : 1;
            uint32_t HPDInterrupt : 1;
            uint32_t reserved : 23;
        } bitFields;
        /*! union 32bits value */
        uint32_t statusVal;
    };
} pd_dp_status_obj_t;

typedef struct _pd_dp_configure_obj
{
    union
    {
        /*! union DP configure object */
        struct
        {
            uint32_t setConfig : 2;
            uint32_t setSignal : 4;
            uint32_t reserved : 2;
            uint32_t configureUFPUPin : 8;
            uint32_t reserved1 : 16;
        } bitFields;
        /*! union 32bits value */
        uint32_t configureVal;
    };
} pd_dp_configure_obj_t;

/* customer can define this structure by self
 * For example: hpd/crossbar driver can differ for different customers' solution.
 */
typedef struct _pd_alt_mode_displayport_config
{
    const uint32_t *modesList;
    uint8_t modesCount;
    uint8_t supportPinAssigns;
    uint8_t multiFunctionPrefered;
    void *crossbarConfig;
    void *hpdConfig;
} pd_alt_mode_dp_config_t;

typedef struct _pd_alt_mode_displayport
{
    pd_handle pdHandle;
    void *altModeHandle;
    pd_alt_mode_dp_config_t *dpConfigParam;
    pd_crossbar_instance_t crossbarInstance;
    pd_hpd_driver_t hpdDriverInstance;

    uint32_t taskEvent;
    uint32_t hpdTime;
    uint32_t pdMsgBuffer[7];
    uint32_t pdMsgReceivedBuffer[7];
    volatile uint32_t delayTime;
    volatile uint32_t delayEvents;
    pd_dp_status_obj_t dpSelfStatus;
    pd_dp_status_obj_t dpPartnerStatus;
    pd_dp_configure_obj_t dpConfigure;
    pd_structured_vdm_header_t pdVDMMsgReceivedHeader;
    volatile uint32_t retryCount;
    volatile uint8_t retryCountCommand;
    uint8_t pdMsgReceivedVDOCount;
    uint8_t selectModeIndex;
    uint8_t occupied;
    uint8_t dpState; /* pd_dp_state_t */
    /* uint8_t retryCommand; */
    uint8_t triggerCommand;
    uint8_t dfpCommand;

    uint8_t waitSendResult : 1;
    uint8_t dfpCommandDoing : 1;
} pd_alt_mode_dp_t;

/*******************************************************************************
 * API
 ******************************************************************************/

pd_status_t PD_DPInit(pd_handle pdHandle, void *altModeHandle, const void *moduleConfig, void **moduleInstance);
pd_status_t PD_DPDeinit(void *moduleInstance);
pd_status_t PD_DPControl(void *moduleInstance, uint32_t controlCode, void *controlParam);
pd_status_t PD_DPCallbackEvent(void *moduleInstance, uint32_t processCode, uint16_t msgSVID, void *param);
void PD_DPTask(void *taskParam);
void PD_DPModule1msISR(void *moduleInstance);

#endif
