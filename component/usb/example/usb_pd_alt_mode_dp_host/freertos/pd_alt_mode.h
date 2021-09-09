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

#ifndef __PD_ALT_MODE_H__
#define __PD_ALT_MODE_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PD_ALT_MODE_MAX_PORT (4)
#define PD_ALT_MODE_COMMAND_RETRY_COUNT (30)

#define DP_SVID (0xFF01)
#define AME_TIMEOUT_VALUE (1000)               /* ms */
#define PD_ALT_MODE_ERROR_RETRY_WAIT_TIME (10) /* ms */
#define PD_ALT_MODE_DR_SWAP_DELAY_TIME (50)    /* ms */

#define PD_ALT_MODE_EVENT_MODULES (0x000000FFu)
#define PD_ALT_MODE_EVENT_MODULE1 (0x00000001u)
#define PD_ALT_MODE_EVENT_MODULE2 (0x00000002u)
#define PD_ALT_MODE_EVENT_MODULE3 (0x00000004u)
#define PD_ALT_MODE_EVENT_MODULE4 (0x00000008u)

#define PD_ALT_MODE_EVENT_AME_TIMEOUT (0x00000100u)
#define PD_ALT_MODE_EVENT_COMMAND (0x00000200u)

typedef enum _pd_alt_mode_control_code
{
    kAltMode_Invalid = 0,
    kAltMode_TriggerEnterMode,
    kAltMode_TriggerExitMode,
    kAltMode_GetModeState,
} pd_alt_mode_control_code_t;

typedef enum _pd_alt_mode_callback_code
{
    kAltMode_EventInvalid = 0,
    kAltMode_Attach,
    kAltMode_Detach,
    kAltMode_HardReset,
    /* receive partner's vdm request msg */
    kAltMode_StructedVDMMsgReceivedProcess,
    /* slef send VMD message, and receive result (ACK) */
    kAltMode_StructedVDMMsgSuccess,
    /* slef send VMD message, (NAK, BUSY) or send success/fail */
    kAltMode_StructedVDMMsgFail,
    kAltMode_UnstructedVDMMsgReceived,
    kAltMode_UnstructedVDMMsgSentResult,
} pd_alt_mode_callback_code_t;

typedef struct _pd_alt_mode_module
{
    uint32_t SVID;
    const void *config; /* customer defined */
    pd_status_t (*pd_alt_mode_init)(pd_handle pdHandle,
                                    void *altModeHandle,
                                    const void *moduleConfig,
                                    void **moduleInstance);
    pd_status_t (*pd_alt_mode_deinit)(void *moduleInstance);
    pd_status_t (*pd_alt_mode_callback_event)(void *moduleInstance,
                                              uint32_t processCode,
                                              uint16_t msgSVID,
                                              void *param);
    pd_status_t (*pd_alt_mode_control)(void *moduleInstance, uint32_t controlCode, void *controlParam);
    void (*pd_alt_mode_task)(void *taskParam);
    void (*pd_alt_mode_1ms_isr)(void *moduleInstance);
} pd_alt_mode_module_t;

typedef struct _pd_dpm_alt_mode_config
{
    /* UFP */
    uint32_t *identityData;
    uint16_t identityObjectCount;
    uint8_t moduleCount;
    uint8_t workAsDFP;
    /* the first module has high priority */
    const pd_alt_mode_module_t *modules;
} pd_alt_mode_config_t;

typedef struct _pd_alt_mode_state
{
    uint32_t mode;
    uint16_t SVID;
} pd_alt_mode_state_t;

typedef struct _pd_dpm_alt_mode
{
    pd_handle pdHandle;
    const pd_alt_mode_config_t *altModeConfig;
    void *altModeModuleInstance[7];
    uint32_t altModeMsgBuffer[7];
    EventGroupHandle_t altModeTaskEvent;
    volatile uint32_t delayTime;
    volatile uint32_t delayEvents;
    uint32_t AMETime;
    volatile uint32_t retryCount;
    volatile uint8_t retryCountCommand;
    uint8_t retryCommand;
    uint8_t dfpCommand;

    volatile uint8_t occupied;

} pd_alt_mode_t;

/*******************************************************************************
 * API
 ******************************************************************************/

pd_status_t PD_AltModeInit(pd_handle pdHandle, const pd_alt_mode_config_t *altModeConfig, void **altModeHandle);
pd_status_t PD_AltModeDeinit(void *altModeHandle);

/* start to alt mode sequence. */
pd_status_t PD_AltModeEnter(void *altModeHandle);

/* exit one alt mode */
pd_status_t PD_AltModeExit(void *altModeHandle, pd_alt_mode_state_t *enteredMode);

/* get the state (the entered alt mode) */
pd_status_t PD_AltModeState(void *altModeHandle, pd_alt_mode_state_t *enteredMode);

/* start/stop AME time UFP */
pd_status_t PD_AltModeStartAMETime(void *altModeHandle);
pd_status_t PD_AltModeStopAMETime(void *altModeHandle);

/* the whole alt mode instance task, it will trigger modules' task if call PD_AltModeModuleTaskWakeUp */
void PD_AltModeTask(void *altModeHandle);

/* PD stack callback events */
pd_status_t PD_AltModeCallback(void *callbackParam, uint32_t event, void *param);

/* this function will be called in one 1ms timer ISR */
void PD_AltModeTimer1msISR(void *altModeHandle);

/* In this implementation, use one task to process all modules' task. so module can use this API to wake up self task to
 * excute. */
void PD_AltModeModuleTaskWakeUp(void *altModeHandle, void *moduleHandle);

#endif
