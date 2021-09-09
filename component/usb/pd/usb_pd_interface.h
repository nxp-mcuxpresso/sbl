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

#ifndef __PD_INTERFACE_H__
#define __PD_INTERFACE_H__

#include "usb_osa.h"
#include "usb_cmsis_wrapper.h"
#include "usb_pd_spec.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum
{
    CONFIG_DEBUG_ACCESSORY_NONE = 0,
    CONFIG_DEBUG_ACCESSORY_TS = 1,
    CONFIG_DEBUG_ACCESSORY_DTS = 2,
} pd_debug_acc_role_t;

#define PD_SPEC_REVISION_ISSUE_FIX (1)

#define PSM_SECONDARY_STATE_COUNT 3

#define PD_CONFIG_DEBUG_ACCESSORY_ROLE (CONFIG_DEBUG_ACCESSORY_DTS)

#define PD_WAIT_EVENT_TIME (10)

/* private */
typedef enum _pd_task_event_type
{
    PD_TASK_EVENT_RECEIVED_HARD_RESET = 0x01u,
    PD_TASK_EVENT_PD_MSG = 0x02u,
    PD_TASK_EVENT_DPM_MSG = 0x04u,
    PD_TASK_EVENT_SEND_DONE = 0x08u,
    PD_TASK_EVENT_TIME_OUT = 0x10u,
    PD_TASK_EVENT_PHY_STATE_CHAGNE = 0x20u,
    PD_TASK_EVENT_OTHER = 0x40u,
    PD_TASK_EVENT_FR_SWAP_SINGAL = 0x80u,
    PD_TASK_EVENT_TYPEC_STATE_PROCESS = 0x100u,
} pd_task_event_type_t;

typedef enum _pd_connect_state
{
    kConnectState_NotStable,
    kConnectState_Connected,
    kConnectState_Disconnected,
} pd_connect_state_t;

typedef struct _pd_instance
{
    pd_phy_handle pdPhyHandle;
    pd_instance_config_t *pdConfig;
    pd_power_port_config_t *pdPowerPortConfig;
    const pd_phy_api_interface_t *phyInterface;
    pd_stack_callback_t pdCallback;
    pd_power_handle_callback_t *callbackFns;
    void *callbackParam;
    usb_osa_event_handle taskEventHandle;
    /* work as sink */
    pd_rdo_t rdoRequest;
    pd_rdo_t rdoSuccessRequest;
    /* work as source */
    pd_rdo_t partnerRdoRequest;
    pd_source_pdo_t selfOrPartnerFirstSourcePDO;
    pd_phy_vendor_info_t phyInfo;
    /* timr */
    volatile uint32_t timrsRunningState[(PD_MAX_TIMER_COUNT + 31) / 32];
    volatile uint32_t timrsTimeOutState[(PD_MAX_TIMER_COUNT + 31) / 32];
    volatile uint16_t timrsTimeValue[PD_MAX_TIMER_COUNT];
    /* pd msg process */
    uint32_t psmCableIdentities[7];
    uint8_t *receivingDataBuffer;
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint32_t receivingData[67];
    uint32_t receivingChunkedData[8];
#else
    uint32_t receivingData[8];
#endif
#if (defined PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
    uint32_t sendingData[67];
#else
    uint32_t sendingData[8];
#endif
    volatile uint32_t *receivedData;
    volatile uint32_t receivedLength;
    volatile pd_msg_header_t sendingMsgHeader;
    volatile pd_status_t receiveResult;
    volatile pd_status_t sendingResult;
    volatile uint8_t receiveState; /* 0 - no pending receive; 1 - receiving; 2 - received data */
    volatile uint8_t receivedSop;
    volatile uint8_t hardResetReceived;
    volatile uint8_t sendingState; /* 0 - no pending send; 1 - sending; 2 - send callback done */
    volatile uint8_t occupied;
    uint8_t phyType;
    /* DPM commands */
    pd_svdm_command_param_t structuredVdmCommandParameter;
    pd_unstructured_vdm_command_param_t unstructuredVdmCommandParameter;
    pd_command_data_param_t commandExtParam;         /* command initiator's parameter */
    pd_command_data_param_t commandExtParamCallback; /* command callback parameter */
    uint32_t dpmMsgBits;
    uint32_t alertADO;
    uint32_t vdmExitReceived[3];
    volatile uint8_t commandIsInitiator;
    pd_psm_state_t psmCurState;
    pd_psm_state_t psmNewState;
    pd_psm_state_t psmInterruptedState;
    pd_psm_state_t psmSecondaryState[3];
    pd_psm_state_t psmNewSecondaryState[3];
    pd_psm_state_t psmDrSwapPrevState;
    pd_psm_state_t psmVconnSwapPrevState;
    TypeCState_t curConnectState;
    uint8_t commandEvaluateResult;
    volatile uint8_t asmVdmReplyMsg;
    uint8_t getBatteryCapDataBlock;
    /* PD stack state machine */
    uint16_t dpmCableMaxCurrent;
    uint8_t vdmExitReceivedSOP[3];
    volatile uint8_t initializeLabel;
    volatile uint8_t isConnected;
    volatile uint8_t connectedResult;
    uint8_t dpmStateMachine;
    uint8_t connectState;
    uint8_t curPowerRole;
    uint8_t curDataRole;
    uint8_t commandProcessing;
    uint8_t ccUsed;
    uint8_t inProgress;
    uint8_t vbusDischargeInProgress;
    volatile uint8_t pendingSOP;
    uint8_t raPresent;
    uint8_t psmPresentlyVconnSource;
    uint8_t psmVdmActiveModeValidMask;
    uint8_t psmVDMBusyWaitDpmMsg;
    uint8_t psmHardResetCount;
    uint8_t psmCapsCounter;
    uint8_t psmSoftResetSop;
    uint8_t revision;
#if 0
    volatile uint8_t typeCStateNeedRecovery;
#endif
    volatile uint8_t noConnectButVBusExit;
    volatile uint8_t rdRpErrorCount;
    volatile uint16_t waitTime;
#if 0
        uint8_t *extDataBuffer;
        uint16_t extAllDataSize;
        uint16_t extRemainSize;
        pd_extended_msg_header_t extRequestChunkHeader;
        uint8_t extChunkNumber;
#endif

    /* 1 bit */
    uint8_t trySNKState : 1;
    uint8_t psmGotoMinTx : 1;
    uint8_t psmGotoMinRx : 1;
    uint8_t enterTrySNKFromPoweredAcc : 1;
    uint8_t cc1Monitor : 1;
    uint8_t cc2Monitor : 1;
    uint8_t psmHardResetNeedsVSafe0V : 1;
    uint8_t psmPresentlyPdConnected : 1;
    uint8_t psmPreviouslyPdConnected : 1;
    uint8_t psmCablePlugResetNeeded : 1;
    uint8_t psmCableDiscoveried : 1;
    uint8_t psmSnkReceiveRdoWaitRetry : 1;
    uint8_t psmExplicitContractExisted : 1;
    volatile uint8_t asmHardResetSnkProcessing : 1;
    uint8_t unchunkedFeature : 1;
    volatile uint8_t commandSrcOwner : 1;
    volatile uint8_t alertWaitReply : 1;
    volatile uint8_t fr5VOpened : 1;
    /* 1. fr_swap CC signal is sent && state is not in PE_FRS_SRC_SNK_CC_Signa.
     * 2. in case the message is not proccessed
     *    when the state machine is not in the source rdy state.
     */
    volatile uint8_t frSwapReceived : 1;
    /* need wait the fr swap msg after the fr signal */
    volatile uint8_t frSignaledWaitFrSwap : 1;
    volatile uint8_t enableReceive : 1;
    volatile uint8_t enterSrcFromSwap : 1;
} pd_instance_t;

/*******************************************************************************
 * API
 ******************************************************************************/

void PD_WaitUsec(uint32_t us);

#endif
