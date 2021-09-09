/*
 * The Clear BSD License
 * Copyright 2015 - 2017 NXP
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

#include "usb_pd_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_ptn5110.h"
#include "usb_pd_ptn5110_register.h"
#include "usb_pd_timer.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*! **************************************************************************
    \brief Control the ILIM_5V_VBUS pin
    \note Only to be called from CLP task
******************************************************************************/
void PDPTN5110_IocSetIlim(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t enable)
{
    /* Set the ILIM output */
    if (enable)
    {
        Reg_BusSetBit(ptn5110Instance, ptn5110_ext_gpio_control, PTN5110_EXT_GPIO_CONTROL_ILIM_5V_VBUS_MASK);
    }
    else
    {
        Reg_BusClrBit(ptn5110Instance, ptn5110_ext_gpio_control, PTN5110_EXT_GPIO_CONTROL_ILIM_5V_VBUS_MASK);
    }
    /* Enable the ILIM output */
    Reg_BusSetBit(ptn5110Instance, ptn5110_ext_gpio_config, PTN5110_EXT_GPIO_CONFIG_ILIM_5V_VBUS_MASK);
}

/*! **************************************************************************
  \brief Callback when connection is detached or disabled (unused, I think the original code use this error)
******************************************************************************/
void PDPTN5110_ConnectReleasePowerControl(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Disable AutoDischarge before PR Swap, which ensure that the Typec connection state is updated to correct state as
     * well */
    Reg_CacheWriteField(ptn5110Instance, CONTROL, power_control, TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK, 0);
    /* Update POWER_CONTROL immediately when disconnection */
    Reg_BusWriteByte(ptn5110Instance, power_control, Reg_CacheRead(ptn5110Instance, CONTROL, power_control));
}

/*! **************************************************************************
    \brief Initialise the hardware for bypass state machine
******************************************************************************/
void PDPTN5110_ConnectInit(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    ptn5110Instance->pwrInProgress = kVbusPower_Invalid;
    PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_CCStatus);
}

/*! **************************************************************************
    \brief Read the current pin state for a given CC pin
    \note Only called from CLP task
******************************************************************************/
void PDPTN5110_ConnectGetCC(pd_phy_ptn5110_instance_t *ptn5110Instance, pd_phy_get_cc_state_t *ccStates)
{
    uint8_t cc1Status;
    uint8_t cc2Status;
    uint8_t roleControl = Reg_CacheRead(ptn5110Instance, CONTROL, role_control);
    uint8_t rawStatus = Reg_CacheRead(ptn5110Instance, CC_STATUS, cc_status);
    ccStates->cc1State = kCCState_Unknown;
    ccStates->cc2State = kCCState_Unknown;

    if (PD_TimerCheckInvalidOrTimeOut(ptn5110Instance->pdHandle, timrCCFilter))
    {
        if (rawStatus == 0xFFu)
        {
            PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_CCStatus);
            /* ptn5110Instance->ccStatusNoChangeTime = 0; */
        }
    }
#if 0
    if (rawStatus != ptn5110Instance->ccPrevStatus)
    {
        ptn5110Instance->ccPrevStatus = rawStatus;
        ptn5110Instance->ccStatusNoChangeTime = 0;
    }
    else
    {
        ptn5110Instance->ccStatusNoChangeTime++;
    }
    if (ptn5110Instance->ccStatusNoChangeTime >= 50)
    {
        ptn5110Instance->ccStatusNoChangeTime = 0;
        PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_CCStatus);
    }
#endif

    rawStatus = Reg_CacheRead(ptn5110Instance, CC_STATUS, cc_status);
    cc1Status = rawStatus & TCPC_CC_STATUS_CC1_STATE_MASK;
    cc2Status = rawStatus & TCPC_CC_STATUS_CC2_STATE_MASK;

    /* If we are still looking, then just use the previous value */
    if (rawStatus & TCPC_CC_STATUS_LOOKING4_CONNECTION_MASK)
    {
        if (rawStatus == 0xFFu)
        {
            ccStates->cc1State = kCCState_Unstable;
            ccStates->cc2State = kCCState_Unstable;
        }
        return;
    }

    if (!(rawStatus & TCPC_CC_STATUS_CONNECT_RESULT_MASK)) /* Rp */
    {
        if ((roleControl & TCPC_ROLE_CONTROL_CC1_MASK) == ROLE_CONTROL_CC1_OPEN)
        {
            ccStates->cc1State = kCCState_Unknown;
        }
        else
        {
            switch (cc1Status)
            {
                case CC_STATUS_CC1_STATE_SRC_OPEN:
                    ccStates->cc1State = kCCState_SrcOpen;
                    break;
                case CC_STATUS_CC1_STATE_SRC_RA:
                    ccStates->cc1State = kCCState_SrcRa;
                    break;
                case CC_STATUS_CC1_STATE_SRC_RD:
                    ccStates->cc1State = kCCState_SrcRd;
                    break;
                default:
                    ccStates->cc1State = kCCState_Unknown;
                    break;
            }
        }

        if ((roleControl & TCPC_ROLE_CONTROL_CC2_MASK) == ROLE_CONTROL_CC2_OPEN)
        {
            ccStates->cc2State = kCCState_Unknown;
        }
        else
        {
            switch (cc2Status)
            {
                case CC_STATUS_CC2_STATE_SRC_OPEN:
                    ccStates->cc2State = kCCState_SrcOpen;
                    break;
                case CC_STATUS_CC2_STATE_SRC_RA:
                    ccStates->cc2State = kCCState_SrcRa;
                    break;
                case CC_STATUS_CC2_STATE_SRC_RD:
                    ccStates->cc2State = kCCState_SrcRd;
                    break;
                default:
                    ccStates->cc2State = kCCState_Unknown;
                    break;
            }
        }
    }
    else /* Rd */
    {
        if ((roleControl & TCPC_ROLE_CONTROL_CC1_MASK) == ROLE_CONTROL_CC1_OPEN)
        {
            ccStates->cc1State = kCCState_Unknown;
        }
        else
        {
            switch (cc1Status)
            {
                case CC_STATUS_CC1_STATE_SNK_DEFAULT:
                    ccStates->cc1State = kCCState_SnkRpDefault;
                    break;
                case CC_STATUS_CC1_STATE_SNK_POWER1_5:
                    ccStates->cc1State = kCCState_SnkRp1_5;
                    break;
                case CC_STATUS_CC1_STATE_SNK_POWER3_0:
                    ccStates->cc1State = kCCState_SnkRp3_0;
                    break;
                case CC_STATUS_CC1_STATE_SNK_OPEN:
                    ccStates->cc1State = kCCState_SnkOpen;
                    break;
                default:
                    ccStates->cc1State = kCCState_Unknown;
                    break;
            }
        }

        if ((roleControl & TCPC_ROLE_CONTROL_CC2_MASK) == ROLE_CONTROL_CC2_OPEN)
        {
            ccStates->cc2State = kCCState_Unknown;
        }
        else
        {
            switch (cc2Status)
            {
                case CC_STATUS_CC2_STATE_SNK_DEFAULT:
                    ccStates->cc2State = kCCState_SnkRpDefault;
                    break;
                case CC_STATUS_CC2_STATE_SNK_POWER1_5:
                    ccStates->cc2State = kCCState_SnkRp1_5;
                    break;
                case CC_STATUS_CC2_STATE_SNK_POWER3_0:
                    ccStates->cc2State = kCCState_SnkRp3_0;
                    break;
                case CC_STATUS_CC2_STATE_SNK_OPEN:
                    ccStates->cc2State = kCCState_SnkOpen;
                    break;
                default:
                    ccStates->cc2State = kCCState_Unknown;
                    break;
            }
        }
    }

    return;
}

/*! **************************************************************************
    \brief Enable interrupts for connection
    \note Only called from CLP task context
    (call one time in the task, so put in the init function)
******************************************************************************/
void PDPTN5110_ConnectEnableTypeCConnection(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* reenable CC_Status change alert */
    PDPTN5110_IntcIrqClearAndEnable(ptn5110Instance, TCPC_ALERT_MASK_CC_STATUS_INTERRUPT_MASK_MASK);
}

/*! **************************************************************************
  \brief Callback when connection is detached or disabled
  (enter into detach or disable state, so put in the PD_PHY_RESET_CONNECT_DETECTION)
******************************************************************************/
void PDPTN5110_ConnectDetachOrDisableCallback(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* need to disable Vbus monitor when detached */
    PDPTN5110_VbusDisableMonitorAndDetect(ptn5110Instance);

    /* Disable AutoDischarge before waiting for connection */
    Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                   TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK, 0);

    /* need to stop BIST Test Data mode when detached */
    ptn5110Instance->tcpcRegCache.CONTROL.tcpc_control &= (uint8_t) ~(TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK);
    Reg_BusClrBit(ptn5110Instance, tcpc_control, TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK);

    /* reenable CC_Status change alert */
    PDPTN5110_IntcIrqClearAndEnable(ptn5110Instance, TCPC_ALERT_MASK_CC_STATUS_INTERRUPT_MASK_MASK);
}

/*! **************************************************************************
  \brief Update victoria register and internal state
  (call through control API interface)
******************************************************************************/
void PDPTN5110_ConnectRawVbusDischarge(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t enable)
{
    PDPTN5110_EnableVSafe0VComparator(ptn5110Instance, (enable));
    Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control, TCPC_POWER_CONTROL_FORCE_DISCHARGE_MASK,
                                   (enable == 0) ? (0u << TCPC_POWER_CONTROL_FORCE_DISCHARGE_LSB) :
                                                   (1u << TCPC_POWER_CONTROL_FORCE_DISCHARGE_LSB));
}

/*! **************************************************************************
    \brief disble CC monitor temoporarily after disconnection
    (put in the PD_PHY_RESET_CONNECT_DETECTION, original codes call this in TYPEC_TRY_SRC, TYPEC_TRY_SNK,
TYPEC_TRY_WAIT_SRC too)
******************************************************************************/
void PDPTN5110_ConnectDisableComparators(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* TODO */
}

void PDPTN5110_ConnectSetCC(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t usedCC)
{
    ptn5110Instance->usedCC = usedCC;

    if (usedCC != kPD_CCInvalid)
    {
        PDPTN5110_ConnectSWitchCCComms(ptn5110Instance, 1, usedCC);
    }
    else
    {
        PDPTN5110_ConnectSWitchCCComms(ptn5110Instance, 0, usedCC);
    }
    Reg_BusWriteByte(ptn5110Instance, tcpc_control,
                     Reg_CacheRead(ptn5110Instance, CONTROL, tcpc_control)
#if (defined USBPD_ENABLE_PTN5110_A0R1_WORKAROUNDS) && (USBPD_ENABLE_PTN5110_A0R1_WORKAROUNDS)
                         /* Allow the TCPM to work-around incorrect entry to debug accessory */
                         | (((Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP) &&
                             (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110) &&
                             (Reg_CacheRead(ptn5110Instance, GLOBAL, device_id) < DEVICE_ID_PTN5110_A0R2)) ?
                                TCPC_TCPC_CONTROL_DEBUG_ACCESSORY_CONTROL_MASK :
                                0)
#endif
                         );
}

/*! **************************************************************************
    \brief Assert and monitor Rp
    \note Called from both EC and CLP task contexts
******************************************************************************/
void PDPTN5110_ConnectAssertRpAttached(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t usedCC, uint8_t rpCfg)
{
    uint8_t powerControl = 0;
    /* Map from configuiration table values to Rp_Value_t value in roleControl */
    uint8_t cfgRpMap[3] = {
        ROLE_CONTROL_RP_DEFAULT,    /*USBPD_CURLIM_STD_USB*/
        ROLE_CONTROL_RP_VALUE_RP_1, /*USBPD_CURLIM_1A5*/
        ROLE_CONTROL_RP_VALUE_RP_3, /*USBPD_CURLIM_3A*/
    };

    /* Set Rp & Rp current value */
    {
        uint8_t rpSel = cfgRpMap[rpCfg - 1];
        if (usedCC == kPD_CC2)
        {
            Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                           (uint8_t)ROLE_CONTROL_CC1_OPEN | (uint8_t)ROLE_CONTROL_CC2_RP | rpSel);
        }
        else
        {
            Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                           (uint8_t)ROLE_CONTROL_CC1_RP | (uint8_t)ROLE_CONTROL_CC2_OPEN | rpSel);
        }

        ptn5110Instance->roleControlUpdated = true;
    }

    if (ptn5110Instance->pwrInProgress != kVbusPower_Stable)
    {
        powerControl = TCPC_POWER_CONTROL_DISABLE_VOLTAGE_ALARMS_MASK;
    }
    else
    {
        powerControl =
            (TCPC_POWER_CONTROL_DISABLE_VOLTAGE_ALARMS_MASK | TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK);
    }
    /* Enable DisableVoltageAlarms and AutoDischarge after connection */
    Reg_CacheWriteField(ptn5110Instance, CONTROL, power_control, powerControl, powerControl);
}

/*! **************************************************************************
    \brief Assert and monitor Rp in unattached state
******************************************************************************/
void PDPTN5110_ConnectAssertRpUnattached(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t srcRp)
{
    /* Both Rp pull ups */
    Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                   ((uint8_t)ROLE_CONTROL_CC1_RP | (uint8_t)ROLE_CONTROL_CC2_RP) |
                       (uint8_t)((uint8_t)srcRp << TCPC_ROLE_CONTROL_RP_VALUE_LSB));
    ptn5110Instance->roleControlUpdated = true;
}

/*! **************************************************************************
    \brief Assert and monitor Rp as DTS source
******************************************************************************/
void PDPTN5110_ConnectAssertRpDbgAccSrc(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Both Rp pull ups, one at Rp3.0 and one at Rp1.5 */
    /* (but there is no way in TCPC to set different RP strengths) */
    Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                   ((uint8_t)ROLE_CONTROL_CC1_RP | (uint8_t)ROLE_CONTROL_CC2_RP));
    ptn5110Instance->roleControlUpdated = true;
}

/*! **************************************************************************
    \brief Assert and monitor Rd as DTS sink
******************************************************************************/
void PDPTN5110_ConnectAssertRdDbgAccSnk(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* pull down both pins, one to Rd and the other to Ra (except TCPC cannot pull down to Ra) */
    Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                   ((uint8_t)ROLE_CONTROL_CC1_RD | (uint8_t)ROLE_CONTROL_CC2_RD));
    ptn5110Instance->roleControlUpdated = true;
}

/*! **************************************************************************
    \brief Assert and monitor Rd in unattached state
******************************************************************************/
void PDPTN5110_ConnectAssertRdAttached(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t usedCC, uint8_t detectWay)
{
    /* Disable CC_Status change alert when sinking */
    /* note: Current Advertisement comes from CC_STATUS alert, So we cannot disable the CC_STATUS alert if we need to */
    /* detect the current advertisement change when sinking VBUS */
    /* PDPTN5110_IntcIrqDisable(ptn5110Instance, TCPC_ALERT_MASK_CC_STATUS_INTERRUPT_MASK_MASK); */

    /* Just the connected pull down */
    Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                   (usedCC == kPD_CC2) ? ((uint8_t)ROLE_CONTROL_CC1_OPEN | (uint8_t)ROLE_CONTROL_CC2_RD) :
                                         ((uint8_t)ROLE_CONTROL_CC1_RD | (uint8_t)ROLE_CONTROL_CC2_OPEN));
    ptn5110Instance->roleControlUpdated = true;

    /* Enable AutoDischarge after connection */
    if (ptn5110Instance->pwrInProgress == kVbusPower_Stable)
    {
        Reg_CacheWriteField(ptn5110Instance, CONTROL, power_control, TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK,
                            (1 << TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_LSB));
    }
}

/*! **************************************************************************
    \brief Assert and monitor Rd in unattached state
******************************************************************************/
void PDPTN5110_ConnectAssertRdUnattached(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* pull down both pins */
    Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                   ((uint8_t)ROLE_CONTROL_CC1_RD | (uint8_t)ROLE_CONTROL_CC2_RD));
    ptn5110Instance->roleControlUpdated = true;
}

/*! **************************************************************************
    \brief Toggle and monitor Rp/Rd in unattached state
******************************************************************************/
void PDPTN5110_ConnectAssertDrpUnattached(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t startSrc, uint8_t srcRp)
{
    uint8_t regVal;

    if (startSrc)
    {
        regVal = TCPC_ROLE_CONTROL_DRP_MASK | ROLE_CONTROL_CC2_RP | ROLE_CONTROL_CC1_RP |
                 (uint8_t)((uint8_t)srcRp << TCPC_ROLE_CONTROL_RP_VALUE_LSB);
    }
    else
    {
        regVal = TCPC_ROLE_CONTROL_DRP_MASK | ROLE_CONTROL_CC2_RD | ROLE_CONTROL_CC1_RD |
                 (uint8_t)((uint8_t)srcRp << TCPC_ROLE_CONTROL_RP_VALUE_LSB);
    }
    Reg_CacheWrite(ptn5110Instance, CONTROL, role_control, regVal);
    ptn5110Instance->roleControlUpdated = true;
}

/*! **************************************************************************
    \brief Set register when dbg acc sink is attached
******************************************************************************/
void PDPTN5110_ConnectDbgAccSnkAttached(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Note: Don't change Role Control, CC line should be kept as Rd/Rd when working at Dbg Acc sink state */

    /* Enable AutoDischarge after connection */
    Reg_CacheWriteField(ptn5110Instance, CONTROL, power_control, TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK,
                        (1 << TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_LSB));
    Reg_BusWriteByte(ptn5110Instance, power_control, Reg_CacheRead(ptn5110Instance, CONTROL, power_control));
}

/*! **************************************************************************
    \brief Set register when dbg acc source is attached
******************************************************************************/
void PDPTN5110_ConnectDbgAccSrcAttached(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Note: Don't change Role Control, CC line should be kept as Rp/Rp when working at Dbg Acc source state */

    /* Enable AutoDischarge after connection */
    Reg_CacheWriteField(ptn5110Instance, CONTROL, power_control, TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK,
                        (1 << TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_LSB));
    Reg_BusWriteByte(ptn5110Instance, power_control, Reg_CacheRead(ptn5110Instance, CONTROL, power_control));
}

/*! **************************************************************************
    \brief Set CC to monitor for accessory disconnection on CC1
******************************************************************************/
void PDPTN5110_ConnectAudioAccessoryAttached(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t cc)
{
    uint8_t cc1Orcc2 = (cc == kPD_CC2) ? ((uint8_t)ROLE_CONTROL_CC2_RP | (uint8_t)ROLE_CONTROL_CC1_OPEN) :
                                         ((uint8_t)ROLE_CONTROL_CC1_RP | (uint8_t)ROLE_CONTROL_CC2_OPEN);
    Reg_CacheWrite(ptn5110Instance, CONTROL, role_control, cc1Orcc2);
    ptn5110Instance->roleControlUpdated = true;
}

/*! **************************************************************************
    \brief Select the CC line to route to the PHY
    \param enable   true - Enable commands and route according to cc_orient_reverse
                    false - Disable switch
******************************************************************************/
void PDPTN5110_ConnectSWitchCCComms(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t enable, uint8_t cc)
{
    if (enable)
    {
        if (cc != kPD_CCInvalid)
        {
            Reg_CacheWriteField(ptn5110Instance, CONTROL, tcpc_control, TCPC_TCPC_CONTROL_PLUG_ORIENTATION_MASK,
                                ((cc == kPD_CC2) << TCPC_TCPC_CONTROL_PLUG_ORIENTATION_LSB));
        }
    }
    else
    {
        Reg_CacheWriteField(ptn5110Instance, CONTROL, tcpc_control, TCPC_TCPC_CONTROL_PLUG_ORIENTATION_MASK, 0);
    }
}

/*! **************************************************************************
    \brief Select the CC line to route to the VCONN source
    \param enable   true - Enable commands and route according to cc_orient_reverse
                    false - Disable switch
******************************************************************************/
void PDPTN5110_ConnectSwitchVConn(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t enable)
{
    uint8_t vconnSelect =
        enable ? (TCPC_POWER_CONTROL_ENABLE_VCONN_MASK | TCPC_POWER_CONTROL_VCONN_POWER_SUPPORTED_MASK) : 0;
    Reg_CacheWriteField(ptn5110Instance, CONTROL, power_control,
                        TCPC_POWER_CONTROL_ENABLE_VCONN_MASK | TCPC_POWER_CONTROL_VCONN_POWER_SUPPORTED_MASK,
                        vconnSelect);

    /* Enable Vconn must be set after roleControl to ensure there is no overlap */
    Reg_BusWriteByte(ptn5110Instance, power_control, Reg_CacheRead(ptn5110Instance, CONTROL, power_control));
}

#ifdef USBPD_ENABLE_VCONN_DISCHARGE
/*! **************************************************************************
  \brief Enable or disable the discharge function of VConn
******************************************************************************/
void PDPTN5110_ConnectRawVconnDischarge(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t discharge_enable)
{
    if (Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP)
    {
        if (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110)
        {
            Reg_BusModifyByteField(ptn5110Instance, ptn5110_ext_control, PTN5110_EXT_CONTROL_VCONN_FORCE_DISCHARGE_MASK,
                                   discharge_enable ? PTN5110_EXT_CONTROL_VCONN_FORCE_DISCHARGE_MASK : 0u);
        }
    }
}
#endif

/*! **************************************************************************
    \brief Syncronize hardware to CC state (roleControl, tcpc_control, fault_control, power_control)
******************************************************************************/
void PDPTN5110_ConnectSync(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint8_t roleControl;

    /* Need to be in normal power mode to write to typec block */
    PDPTN5110_PwrDoApplyCurState(ptn5110Instance, kPhyPower_NormalMode);

    roleControl = Reg_CacheRead(ptn5110Instance, CONTROL, role_control);

    Reg_BusWriteByte(ptn5110Instance, tcpc_control,
                     Reg_CacheRead(ptn5110Instance, CONTROL, tcpc_control)
#if (defined USBPD_ENABLE_PTN5110_A0R1_WORKAROUNDS) && (USBPD_ENABLE_PTN5110_A0R1_WORKAROUNDS)
                         /* Allow the TCPM to work-around incorrect entry to debug accessory */
                         | (((Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP) &&
                             (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110) &&
                             (Reg_CacheRead(ptn5110Instance, GLOBAL, device_id) < DEVICE_ID_PTN5110_A0R2)) ?
                                TCPC_TCPC_CONTROL_DEBUG_ACCESSORY_CONTROL_MASK :
                                0)
#endif
                         );
    Reg_BusWriteByte(ptn5110Instance, fault_control, Reg_CacheRead(ptn5110Instance, CONTROL, fault_control));

    if (ptn5110Instance->roleControlUpdated)
    {
        Reg_CacheWrite(ptn5110Instance, CC_STATUS, cc_status, 0xFF);
        PD_TimerStart(ptn5110Instance->pdHandle, timrCCFilter, T_CC_FILTER_MAX);

#if (defined USBPD_ENABLE_PTN5110_A0R2_WORKAROUNDS) && (USBPD_ENABLE_PTN5110_A0R2_WORKAROUNDS)
        if ((Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP) &&
            (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110) &&
            (Reg_CacheRead(ptn5110Instance, GLOBAL, device_id) < DEVICE_ID_PTN5110_A0R2))
        {
            /* If we are starting to toggle, then preset the hardware state machine to the right levels */
            if (RoleControlIsToggleSrcFirst(roleControl))
            {
                Reg_BusWriteByte(ptn5110Instance, role_control,
                                 (uint8_t)(roleControl & (uint8_t) ~(uint8_t)TCPC_ROLE_CONTROL_DRP_MASK));
            }
            else if (RoleControlIsToggleSnkFirst(roleControl))
            {
                Reg_BusWriteByte(ptn5110Instance, role_control,
                                 (uint8_t)(roleControl & (uint8_t) ~(uint8_t)TCPC_ROLE_CONTROL_DRP_MASK));
            }
            else
            {
            }
        }
#endif

        Reg_BusWriteByte(ptn5110Instance, role_control, roleControl);
        PDPTN5110_IocSetIlim(ptn5110Instance, ((roleControl >> 4) & 0x03u) == 0x02u);
    }

    /* Enable Vconn must be set after roleControl to ensure there is no overlap */
    Reg_BusWriteByte(ptn5110Instance, power_control, Reg_CacheRead(ptn5110Instance, CONTROL, power_control));

    /* send Look4connection command when Drp/Rp/Rp or Drp/Rd/Rd */
    if ((ptn5110Instance->roleControlUpdated) &&
        ((RoleControlIsToggleSrcFirst(roleControl)) || (RoleControlIsToggleSnkFirst(roleControl))))
    {
        Reg_BusWriteByte(ptn5110Instance, command, TCPC_LOOK4CONNECTION);
    }

    ptn5110Instance->roleControlUpdated = false;
}

/*! **************************************************************************
    \brief Set CC to Z and disable comparators
******************************************************************************/
void PDPTN5110_ConnectDisableAll(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Clear terminations */
    Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                   ((uint8_t)ROLE_CONTROL_CC1_OPEN | (uint8_t)ROLE_CONTROL_CC2_OPEN));
    ptn5110Instance->roleControlUpdated = true;

    /* Disable AutoDischarge before waiting for connection */
    PDPTN5110_ConnectReleasePowerControl(ptn5110Instance);
}

/*! **************************************************************************
    \brief Update hardware after a power role swap has completed (e.g. read current advert of new source).
******************************************************************************/
void PDPTN5110_ConnectUpdateAfterPRSwap(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t powerRole)
{
    /* reenable CC_Status change alert when PR swap to src */
    if (powerRole == kPD_PowerRoleSource)
    {
        PDPTN5110_IntcIrqClearAndEnable(ptn5110Instance, TCPC_ALERT_MASK_CC_STATUS_INTERRUPT_MASK_MASK);
    }

    /* Power role swap does not prompt an interrupt - update the cached status/role info */
    PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_CCStatus);

    /* Update the current status to cover the case where current sense was stable while swap in progress asserted */
    PDPTN5110_ConnectINTcTypeCCurrentStable(ptn5110Instance);
}

/*! **************************************************************************
    \brief Read the current current limit
******************************************************************************/
uint8_t PDPTN5110_ConnectGetTypeCCurrent(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t usedCC)
{
    uint8_t typecCurrent = kCurrent_Invalid;
    uint8_t ccStatus = Reg_CacheRead(ptn5110Instance, CC_STATUS, cc_status);

    /* Update Typec Current status only when connectionresult shows we're presenting Rd */
    if (ccStatus & TCPC_CC_STATUS_CONNECT_RESULT_MASK)
    {
        typecCurrent = (usedCC == kPD_CC2) ? TcpcReadField(ccStatus, CC_STATUS, CC2_STATE) :
                                             TcpcReadField(ccStatus, CC_STATUS, CC1_STATE);
    }
#if defined(PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE) && (PD_CONFIG_PD3_AMS_COLLISION_AVOID_ENABLE)
    if (typecCurrent == kCurrent_1A5)
    {
        ptn5110Instance->amsSinkTxOK = false;
    }
    else
    {
        ptn5110Instance->amsSinkTxOK = true;
    }
#endif

    return typecCurrent;
}

/*! **************************************************************************
    \brief Callback to be called when TypeC current sense is stable
******************************************************************************/
void PDPTN5110_ConnectINTcTypeCCurrentStable(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* uint8_t typecCurrent = kCurrent_Invalid; */

    ptn5110Instance->currentStable = 1;
    /*
    typecCurrent = PDPTN5110_ConnectGetTypeCCurrent(ptn5110Instance, ptn5110Instance->usedCC);
    PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_SINK_TYPEC_CURRENT_VALUE, &typecCurrent);
    */
}

/*! **************************************************************************
    \brief Callback to check for detach during hard reset
    The CC state change as OPEN when doing hard_reset,
    set the inProgress as normal state so the normal detach can be detected.
   \NOTE Called from ISR context
   (do not implement it)
******************************************************************************/
void PDPTN5110_ConnectInTcChecKDetacHDurIngHardReset(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* If we see a detach condition during a hard reset, then exit the hard reset condition */
    if (ptn5110Instance->pwrInProgress == kVbusPower_InHardReset)
    {
        uint8_t ccStatus = Reg_CacheRead(ptn5110Instance, CC_STATUS, cc_status);
        bool orientReverse = Reg_CacheRead(ptn5110Instance, CONF_STD_OUT, config_standard_output) &
                             TCPC_CONFIGURE_STANDARD_OUTPUT_CONNECTOR_ORIENTATION_GPIO0_MASK;
        /* Check for open condition on the attached CC line */
        if ((orientReverse ? TcpcReadField(ccStatus, CC_STATUS, CC2_STATE) :
                             TcpcReadField(ccStatus, CC_STATUS, CC1_STATE)) == 0)
        {
            /* ConnectClpSetInProgress(ptn5110Instance, kVbusPower_Stable); */
        }
    }
}

/*! **************************************************************************
    \brief Inform the Hardware chip that an activity is in progress
    \note Called from both ISR and CLP task context
******************************************************************************/
void PDPTN5110_ConnectSetInProgress(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t state)
{
    if (ptn5110Instance->pwrInProgress != state)
    {
        if (state != kVbusPower_Stable)
        {
            ptn5110Instance->currentStable = 0;
            /* Disable any automatic disconnection during a swap */
            Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                           TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK, 0);
        }
        else
        {
            if (ptn5110Instance->pwrInProgress == kVbusPower_InPRSwap)
            {
                ptn5110Instance->currentStable = 1;
                PDPTN5110_ConnectUpdateAfterPRSwap(ptn5110Instance, ptn5110Instance->phyPowerRole);
            }
            /* Enable automatic disconnection after resuming normal operation */
            Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                           TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK,
                                           TCPC_POWER_CONTROL_AUTO_DISCHARGE_DISCONNECT_MASK);
        }

        ptn5110Instance->pwrInProgress = state;
    }
}

void PDPTN5110_ConnectSrcSetTypecCurrent(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t usedCC, uint8_t val)
{
    /* Map from configuiration table values to Rp_Value_t value in roleControl */
    uint8_t cfgRpMap[3] = {
        ROLE_CONTROL_RP_DEFAULT,    /*USBPD_CURLIM_STD_USB*/
        ROLE_CONTROL_RP_VALUE_RP_1, /*USBPD_CURLIM_1A5*/
        ROLE_CONTROL_RP_VALUE_RP_3, /*USBPD_CURLIM_3A*/
    };
    uint8_t rpSel = cfgRpMap[val - 1];

    if (usedCC == kPD_CC2)
    {
        Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                       (uint8_t)ROLE_CONTROL_CC1_OPEN | (uint8_t)ROLE_CONTROL_CC2_RP | rpSel);
    }
    else
    {
        Reg_CacheWrite(ptn5110Instance, CONTROL, role_control,
                       (uint8_t)ROLE_CONTROL_CC1_RP | (uint8_t)ROLE_CONTROL_CC2_OPEN | rpSel);
    }

    /* TODO: will set "look4connection in sync function. */
    ptn5110Instance->roleControlUpdated = true;
}
