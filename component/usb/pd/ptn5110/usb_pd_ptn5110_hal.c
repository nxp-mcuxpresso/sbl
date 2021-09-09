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

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TX_ABORT_MASK (TCPC_ALERT_TRANSMIT_SOP_MESSAGE_FAILED_MASK | TCPC_ALERT_TRANSMIT_SOP_MESSAGE_DISCARDED_MASK)
#define TX_DONE_MASK (TX_ABORT_MASK | TCPC_ALERT_TRANSMIT_SOP_MESSAGE_SUCCESSFUL_MASK)
#define TX_HARDRESET_MASK \
    (TCPC_ALERT_TRANSMIT_SOP_MESSAGE_FAILED_MASK | TCPC_ALERT_TRANSMIT_SOP_MESSAGE_SUCCESSFUL_MASK)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void PD_WaitUsec(uint32_t us);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*! **************************************************************************
    \brief Read the remote registers to local buffer
******************************************************************************/
void PDPTN5110_RegCacheSynC(pd_phy_ptn5110_instance_t *ptn5110Instance, uint16_t mask)
{
    if (mask & kRegModule_Intc)
    {
        Reg_BusReadBlock(ptn5110Instance, alert, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.INTC.alert);
    }

    /* update alert_mask / power_status_mask / fault_status_mask together */
    if (mask & kRegModule_Mask)
    {
        Reg_BusReadBlock(ptn5110Instance, alert_mask, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.MASK.alert_mask);
        Reg_BusReadBlock(ptn5110Instance, power_status_mask, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.MASK.power_status_mask);
        Reg_BusReadBlock(ptn5110Instance, fault_status_mask, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.MASK.fault_status_mask);
        if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
        {
            Reg_BusReadBlock(ptn5110Instance, extended_status_mask, 1,
                             (uint8_t *)&ptn5110Instance->tcpcRegCache.MASK.extended_status_mask);
            Reg_BusReadBlock(ptn5110Instance, alert_extended_mask, 1,
                             (uint8_t *)&ptn5110Instance->tcpcRegCache.MASK.alert_extended_mask);
        }
    }

    /* update receive_byte_count / rx_buf_frame_type / rxBufHeader together, it is unnecessary to update */
    /* message_header_info / receive_detect */
    if (mask & kRegModule_MsgRX)
    {
        /* Use a block read to lessen the interrupt time. The memory for these variables must be contiguous. */
        Reg_BusReadBlock(ptn5110Instance, receive_byte_count, 4,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.MSG_RX.receive_byte_count);
    }

    if (mask & kRegModule_ConfStdOut)
    {
        Reg_BusReadBlock(ptn5110Instance, config_standard_output, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CONF_STD_OUT.config_standard_output);
    }

    /* update tcpc_control / roleControl / fault_control / power_control together */
    if (mask & kRegModule_Control)
    {
        Reg_BusReadBlock(ptn5110Instance, tcpc_control, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CONTROL.tcpc_control);
        Reg_BusReadBlock(ptn5110Instance, role_control, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CONTROL.role_control);
        Reg_BusReadBlock(ptn5110Instance, fault_control, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CONTROL.fault_control);
        Reg_BusReadBlock(ptn5110Instance, power_control, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CONTROL.power_control);
    }

    if (mask & kRegModule_CCStatus)
    {
        Reg_BusReadBlock(ptn5110Instance, cc_status, 1, (uint8_t *)&ptn5110Instance->tcpcRegCache.CC_STATUS.cc_status);
    }

    if (mask & kRegModule_PowerStatus)
    {
        Reg_BusReadBlock(ptn5110Instance, power_status, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.POWER_STATUS.power_status);
    }

    if (mask & kRegModule_FaultStatus)
    {
        Reg_BusReadBlock(ptn5110Instance, fault_status, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.FAULT_STATUS.fault_status);
    }

    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        if (mask & kRegModule_ExtendedStatus)
        {
            Reg_BusReadBlock(ptn5110Instance, extended_status, 1,
                             (uint8_t *)&ptn5110Instance->tcpcRegCache.EXTENDED_STATUS.extended_status);
        }

        if (mask & kRegModule_AlertExtended)
        {
            Reg_BusReadBlock(ptn5110Instance, alert_extended, 1,
                             (uint8_t *)&ptn5110Instance->tcpcRegCache.ALERT_EXTENDED.alert_extended);
        }
    }

    /* update device_capabilities_1/device_capabilities_2/standard_input_capabilities/standard_output_capabilities */
    /* together */
    if (mask & kRegModule_Capability)
    {
        Reg_BusReadBlock(ptn5110Instance, device_capabilities_1, 2,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CAPABILITY.device_capabilities_1);
        Reg_BusReadBlock(ptn5110Instance, device_capabilities_2, 2,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CAPABILITY.device_capabilities_2);
        Reg_BusReadBlock(ptn5110Instance, standard_input_capabilities, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CAPABILITY.standard_input_capabilities);
        Reg_BusReadBlock(ptn5110Instance, standard_output_capabilities, 1,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.CAPABILITY.standard_output_capabilities);
    }

    /* update */
    /* vbus_voltage/vbus_sink_disconnect_threshold/vbus_stop_discharge_threshold/vbus_voltage_alarm_hi_cfg/vbus_voltage_alarm_lo_cfg
     */
    /* together */
    if (mask & kRegModule_Vbus)
    {
        Reg_BusReadBlock(ptn5110Instance, vbus_voltage, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.VBUS.vbus_voltage);
        Reg_BusReadBlock(ptn5110Instance, vbus_sink_disconnect_threshold, 2,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.VBUS.vbus_sink_disconnect_threshold);
        Reg_BusReadBlock(ptn5110Instance, vbus_stop_discharge_threshold, 2,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.VBUS.vbus_stop_discharge_threshold);
        Reg_BusReadBlock(ptn5110Instance, vbus_voltage_alarm_hi_cfg, 2,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.VBUS.vbus_voltage_alarm_hi_cfg);
        Reg_BusReadBlock(ptn5110Instance, vbus_voltage_alarm_lo_cfg, 2,
                         (uint8_t *)&ptn5110Instance->tcpcRegCache.VBUS.vbus_voltage_alarm_lo_cfg);
    }
}

uint32_t PDPTN5110_GetVbusVoltage(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint32_t voltage;
    Reg_BusReadBlock(ptn5110Instance, vbus_voltage, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.VBUS.vbus_voltage);
    voltage = Reg_CacheRead(ptn5110Instance, VBUS, vbus_voltage);
    voltage = TcpcReadField(voltage, VBUS_VOLTAGE, VBUS_VOLTAGE_MEASUREMENT) *
              (1 << TcpcReadField(voltage, VBUS_VOLTAGE, VBUS_SCALE_FACTOR));

    return voltage;
}

/*! **************************************************************************
    \brief Enable the Vsafe0v analog comparator
******************************************************************************/
void PDPTN5110_EnableVSafe0VComparator(pd_phy_ptn5110_instance_t *ptn5110Instance, bool discharge)
{
    /* Set "Disable Voltage Alarms" bit to 1 in order to disable Voltage Alarms */
    Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                   TCPC_POWER_CONTROL_DISABLE_VOLTAGE_ALARMS_MASK,
                                   TCPC_POWER_CONTROL_DISABLE_VOLTAGE_ALARMS_MASK);
    /* Set the high voltage alarm to an unrealistically high value */
    Reg_CacheAndBusModifyWordField(ptn5110Instance, VBUS, vbus_voltage_alarm_hi_cfg,
                                   TCPC_VBUS_VOLTAGE_ALARM_HI_CFG_VBUS_LO_VOLTAGE_TRIP_POINT_MASK,
                                   TCPC_VBUS_VOLTAGE_ALARM_HI_CFG_VBUS_LO_VOLTAGE_TRIP_POINT_MASK);
    /* Set threshold */
    if (discharge)
    {
        Reg_CacheAndBusModifyWordField(ptn5110Instance, VBUS, vbus_voltage_alarm_lo_cfg,
                                       TCPC_VBUS_VOLTAGE_ALARM_LO_CFG_VBUS_HI_VOLTAGE_TRIP_POINT_MASK,
                                       VBUS_VSAFE0V_MIN_THRESHOLD
                                           << TCPC_VBUS_VOLTAGE_ALARM_LO_CFG_VBUS_HI_VOLTAGE_TRIP_POINT_LSB);
    }
    else
    {
        Reg_CacheAndBusModifyWordField(ptn5110Instance, VBUS, vbus_voltage_alarm_lo_cfg,
                                       TCPC_VBUS_VOLTAGE_ALARM_LO_CFG_VBUS_HI_VOLTAGE_TRIP_POINT_MASK,
                                       VBUS_VSAFE0V_MAX_THRESHOLD
                                           << TCPC_VBUS_VOLTAGE_ALARM_LO_CFG_VBUS_HI_VOLTAGE_TRIP_POINT_LSB);
    }
    /* Clear "Disable Voltage Alarms" bit to 0 in order to enable Voltage Alarms */
    Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                   TCPC_POWER_CONTROL_DISABLE_VOLTAGE_ALARMS_MASK, 0);
    ptn5110Instance->halWaitVsafe0V = true;
}

/*! **************************************************************************
  \brief Enable the Callback for power supply change interrupt, initalize state according to the state at this time
  (call in the init function)
******************************************************************************/
void PDPTN5110_PsStatChangeEnableInterrupt(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Need to Read Power_Status before clear Alert, otherwise Power_status Alert cannot be cleared in Mead */
    PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_PowerStatus);
    PDPTN5110_IntcIrqClearAndEnable(ptn5110Instance, TCPC_ALERT_PORT_POWER_STATUS_MASK);

    /* PDPTN5110_ReportVbusPresence(CLP_PORT); */
}

void PDPTN5110_PsStatChangeIntCallback(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint8_t curStat1;
    uint8_t risingEdges1;
    uint8_t fallingEdges1;

    curStat1 = Reg_CacheRead(ptn5110Instance, POWER_STATUS, power_status);

    risingEdges1 = curStat1 & (uint8_t) ~(uint8_t)(ptn5110Instance->prevPwrStat);
    fallingEdges1 = (uint8_t) ~(uint8_t)curStat1 & (uint8_t)(ptn5110Instance->prevPwrStat);

    /* Inform Vsafe5V Only when vbus_present change from 0 to 1 and not sourcing high voltage */
    if ((risingEdges1 & TCPC_POWER_STATUS_VBUS_PRESENT_MASK) &&
        ((curStat1 & TCPC_POWER_STATUS_SOURCING_HIGH_VOLTAGE_MASK) == 0))
    {
        PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_VBUS_STATE_CHANGE, NULL);
        /* Enable low voltage alarm to use VBUS Voltage Alarm Lo alert to check whether we inform Vsafe0V when
         * VBUS_Prsent
         * gone */
        PDPTN5110_EnableVSafe0VComparator(ptn5110Instance, true);
    }

    /* Set Vbus present status when vbus_present status is changing */
    if ((uint8_t)(risingEdges1 | fallingEdges1) & TCPC_POWER_STATUS_VBUS_PRESENT_MASK)
    {
        PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_VBUS_STATE_CHANGE, NULL);
    }

    /* Always expect PSM/CLP will need to handle the event */
    ptn5110Instance->prevPwrStat = curStat1;
}

/*! **************************************************************************
  \brief Callback for VBUS Voltage Alarm Lo alert
  \param port passed from ISR context
******************************************************************************/
void PDPTN5110_VbusVoltageAlarmLoIntCallback(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Set "Disable Voltage Alarms" bit to 1 in order to disable Voltage Alarms */
    Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                   TCPC_POWER_CONTROL_DISABLE_VOLTAGE_ALARMS_MASK,
                                   TCPC_POWER_CONTROL_DISABLE_VOLTAGE_ALARMS_MASK);
    /* Inform Vsafe0V if waiting Vsafe0V and VBUS Voltage Alarm Lo alert is asserted */
    if (ptn5110Instance->halWaitVsafe0V)
    {
        ptn5110Instance->halWaitVsafe0V = false;
        /* halWaitVsafe0V is used for this inform */
        PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_VBUS_STATE_CHANGE, NULL);
    }
}

/*! **************************************************************************
  \brief Enable Vbus monitor and detect
  put in the attach detecion, the original code enable this when TYPEC_ATTACH_WAIT_SNK or TYPEC_ATTACH_WAIT_SRC
******************************************************************************/
void PDPTN5110_VbusEnableMonitorAndDetect(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Turn on VBUS Voltage monitor.  0 means enabled. */
    Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                   TCPC_POWER_CONTROL_VBUS_VOLTAGE_MONITOR_MASK, 0);
    Reg_BusWriteByte(ptn5110Instance, command, TCPC_ENABLEVBUSDETECT);
}

/* call in the detach or disable state */
/*! **************************************************************************
  \brief Disable Vbus monitor and detect
******************************************************************************/
void PDPTN5110_VbusDisableMonitorAndDetect(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Turn off VBUS Voltage monitor.  1 means disabled. */
    Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                   TCPC_POWER_CONTROL_VBUS_VOLTAGE_MONITOR_MASK, 1);
    Reg_BusWriteByte(ptn5110Instance, command, TCPC_DISABLEVBUSDETECT);
}

void PDPTN5110_SetFetControl(pd_phy_ptn5110_instance_t *ptn5110Instance, pd_ptn5110_ctrl_pin_t *set)
{
    Reg_BusReadBlock(ptn5110Instance, power_status, 1,
                     (uint8_t *)&ptn5110Instance->tcpcRegCache.POWER_STATUS.power_status);
    if (set->enSRC)
    {
        if (ptn5110Instance->tcpcRegCache.POWER_STATUS.power_status & TCPC_POWER_STATUS_SINKING_VBUS_MASK)
        {
            Reg_BusWriteByte(ptn5110Instance, command, TCPC_DISABLESINKVBUS);
            Reg_BusReadBlock(ptn5110Instance, power_status, 1,
                             (uint8_t *)&ptn5110Instance->tcpcRegCache.POWER_STATUS.power_status);
        }

        Reg_BusWriteByte(ptn5110Instance, command, TCPC_SOURCEVBUSDEFAULTVOLTAGE);
    }

    if (set->enSNK1)
    {
        if (ptn5110Instance->tcpcRegCache.POWER_STATUS.power_status & TCPC_POWER_STATUS_SOURCING_VBUS_MASK)
        {
            Reg_BusWriteByte(ptn5110Instance, command, TCPC_DISABLESOURCEVBUS);
            Reg_BusReadBlock(ptn5110Instance, power_status, 1,
                             (uint8_t *)&ptn5110Instance->tcpcRegCache.POWER_STATUS.power_status);
        }

        Reg_BusWriteByte(ptn5110Instance, command, TCPC_SINKVBUS);
    }

    if ((!(set->enSRC)) && (!(set->enSNK1)))
    {
        Reg_BusWriteByte(ptn5110Instance, command, TCPC_DISABLESOURCEVBUS);
        Reg_BusWriteByte(ptn5110Instance, command, TCPC_DISABLESINKVBUS);
    }
}

void PDPTN5110_ResetFetControl(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    pd_ptn5110_ctrl_pin_t set;

    set.enSNK1 = 0;
    set.enSRC = 0;

    PDPTN5110_SetFetControl(ptn5110Instance, &set);
}

uint8_t PDPTN5110_VsysIsPresent(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Always true for TCPM since no way to get this info from TCPC, we will assume Vsys present if LPC is running */
    return 1;
}

/*! **************************************************************************
  \brief Return true if vbus is within loose bounds of VSafe0V
******************************************************************************/

void PDPTN5110_HalInit(pd_phy_handle pdPhyHandle)
{
    pd_phy_ptn5110_instance_t *ptn5110Instance = (pd_phy_ptn5110_instance_t *)pdPhyHandle;

    PDPTN5110_PsStatChangeEnableInterrupt(ptn5110Instance);

    Reg_BusReadBlock(ptn5110Instance, vendor_id, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.GLOBAL.vendor_id);
    Reg_BusReadBlock(ptn5110Instance, product_id, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.GLOBAL.product_id);
    Reg_BusReadBlock(ptn5110Instance, device_id, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.GLOBAL.device_id);
    Reg_BusReadBlock(ptn5110Instance, usbtypec_rev, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.GLOBAL.usbtypec_rev);
    Reg_BusReadBlock(ptn5110Instance, usbpd_rev_ver, 2, (uint8_t *)&ptn5110Instance->tcpcRegCache.GLOBAL.usbpd_rev_ver);
    Reg_BusReadBlock(ptn5110Instance, pd_interface_rev, 2,
                     (uint8_t *)&ptn5110Instance->tcpcRegCache.GLOBAL.pd_interface_rev);

    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_Mask);
        PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_ExtendedStatus);
        PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_AlertExtended);
    }
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
    PDPTN5110_DisableFRSwap(ptn5110Instance);
#endif

    /* Save the initial CC_STATUS so we can determine if we are dead battery or debug accessory on reset */
    ptn5110Instance->initialCcStatus = Reg_CacheRead(ptn5110Instance, CC_STATUS, cc_status);

    if (Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP)
    {
        if (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110)
        {
            Reg_BusModifyByteField(ptn5110Instance, ptn5110_ext_fault_config,
                                   PTN5110_EXT_FAULT_CONFIG_OPEN_FETS_ON_ADC_ALARM_MASK, 0);
        }
    }

    PDPTN5110_IntcTypecChipInit(ptn5110Instance);
}

/*! **************************************************************************
    \brief only enable the current sense when we don't have PD active
******************************************************************************/
void PDPTN5110_PsmDisableCurrentStable(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
}

/*! **************************************************************************
    \brief Configure the VCONN faults according to config table parameters
    \param init  When set the existing fault status will not be cleared. When not set the fault status will be cleared.
******************************************************************************/
void PDPTN5110_ConfigVconnProtection(pd_phy_ptn5110_instance_t *ptn5110Instance, bool init)
{
}

#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
/*! **************************************************************************
  \brief Signal FR Swap
******************************************************************************/
void PDPTN5110_SignalFRSwap(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Do the following actions manually and atomically, for the shortest latency */
    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        Reg_BusWriteByte(ptn5110Instance, command, TCPC_SENDFRSWAPSIGNAL); /* generate the FRS signal */
    }
    else
    {
        Reg_BusWriteByte(ptn5110Instance, ptn5110_ext_command, TCPC_SEND_FRS); /* generate the FRS signal */
    }
}

/*! **************************************************************************
  \brief Enable FR Swap
******************************************************************************/
void PDPTN5110_EnableFRSwap(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint16_t registerValue;

    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        Reg_CacheAndBusModifyByteField(ptn5110Instance, MASK, alert_extended_mask,
                                       TCPC_ALERT_EXTENDED_MASK_SOURCE_FAST_ROLE_SWAP_MASK_MASK |
                                           TCPC_ALERT_EXTENDED_MASK_SINK_FAST_ROLE_SWAP_MASK_MASK,
                                       TCPC_ALERT_EXTENDED_MASK_SOURCE_FAST_ROLE_SWAP_MASK_MASK |
                                           TCPC_ALERT_EXTENDED_MASK_SINK_FAST_ROLE_SWAP_MASK_MASK);
        Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                       TCPC_POWER_CONTROL_FAST_ROLE_SWAP_ENABLE_MASK,
                                       1 << TCPC_POWER_CONTROL_FAST_ROLE_SWAP_ENABLE_LSB);
    }
    else /* vendor defined */
    {
        if (Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP)
        {
            if (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110)
            {
                registerValue = 0;
                Reg_BusReadBlock(ptn5110Instance, ptn5110_ext_alert_mask, 2, (uint8_t *)&registerValue);
                if ((registerValue & (PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_VSAFE5V_BEING_SOURCED_MASK |
                                      PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_REQUEST_RECEIVED_MASK)) !=
                    (PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_VSAFE5V_BEING_SOURCED_MASK |
                     PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_REQUEST_RECEIVED_MASK))
                {
                    registerValue |= (PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_VSAFE5V_BEING_SOURCED_MASK |
                                      PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_REQUEST_RECEIVED_MASK);
                    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&registerValue, 2, ptn5110_ext_alert_mask);
                }

                registerValue = 0;
                Reg_BusReadBlock(ptn5110Instance, ptn5110_ext_config, 2, (uint8_t *)&registerValue);
                if (!(registerValue & PTN5110_EXT_CONFIG_FAST_ROLE_SWAP_ENABLE_MASK))
                {
                    registerValue |= (PTN5110_EXT_CONFIG_FAST_ROLE_SWAP_ENABLE_MASK);
                    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&registerValue, 2, ptn5110_ext_config);
                }
            }
        }
    }
}
#endif

/*! **************************************************************************
  \brief Disable FR Swap
******************************************************************************/
void PDPTN5110_DisableFRSwap(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint16_t registerValue;

    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        Reg_CacheAndBusModifyByteField(ptn5110Instance, CONTROL, power_control,
                                       TCPC_POWER_CONTROL_FAST_ROLE_SWAP_ENABLE_MASK, 0);
        Reg_CacheAndBusModifyByteField(ptn5110Instance, MASK, alert_extended_mask,
                                       TCPC_ALERT_EXTENDED_MASK_SOURCE_FAST_ROLE_SWAP_MASK_MASK |
                                           TCPC_ALERT_EXTENDED_MASK_SINK_FAST_ROLE_SWAP_MASK_MASK,
                                       0);
    }
    else
    {
        if (Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP)
        {
            if (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110)
            {
                registerValue = 0;
                Reg_BusReadBlock(ptn5110Instance, ptn5110_ext_config, 2, (uint8_t *)&registerValue);
                if (registerValue & PTN5110_EXT_CONFIG_FAST_ROLE_SWAP_ENABLE_MASK)
                {
                    registerValue &= (uint16_t)(~(uint16_t)PTN5110_EXT_CONFIG_FAST_ROLE_SWAP_ENABLE_MASK);
                    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&registerValue, 2, ptn5110_ext_config);
                }

                registerValue = 0;
                Reg_BusReadBlock(ptn5110Instance, ptn5110_ext_alert_mask, 2, (uint8_t *)&registerValue);
                if (registerValue & (PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_VSAFE5V_BEING_SOURCED_MASK |
                                     PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_REQUEST_RECEIVED_MASK))
                {
                    registerValue &=
                        (uint16_t)(~(uint16_t)(PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_VSAFE5V_BEING_SOURCED_MASK |
                                               PTN5110_EXT_ALERT_MASK_FAST_ROLE_SWAP_REQUEST_RECEIVED_MASK));
                    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&registerValue, 2, ptn5110_ext_alert_mask);
                }
            }
        }
    }
}

/*! **************************************************************************
    \brief Report whether we are sinking Vbus
    (not used yet, for supporting dead battery)
******************************************************************************/
uint8_t PDPTN5110_StartedDeadBatterySink(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* Check for an existing sink connection */
    if (ptn5110Instance->tcpcRegCache.POWER_STATUS.power_status & TCPC_POWER_STATUS_VBUS_PRESENT_MASK)
    {
        uint8_t rawStatus = ptn5110Instance->initialCcStatus;
        uint8_t cc1Status = rawStatus & TCPC_CC_STATUS_CC1_STATE_MASK;
        uint8_t cc2Status = rawStatus & TCPC_CC_STATUS_CC2_STATE_MASK;
        /* Check for an existing oriented debug accessory sink connection */
        if ((rawStatus & TCPC_CC_STATUS_CONNECT_RESULT_MASK) &&
            ((cc1Status == CC_STATUS_CC1_STATE_SNK_DEFAULT) || (cc1Status == CC_STATUS_CC1_STATE_SNK_POWER1_5) ||
             (cc1Status == CC_STATUS_CC1_STATE_SNK_POWER3_0)) &&
            ((cc2Status == CC_STATUS_CC2_STATE_SNK_DEFAULT) || (cc2Status == CC_STATUS_CC2_STATE_SNK_POWER1_5) ||
             (cc2Status == CC_STATUS_CC2_STATE_SNK_POWER3_0)) &&
            (cc1Status != cc2Status))
        {
            return 0;
        }
        return 1;
    }
    return 0;
}

/*! **************************************************************************
    \brief Initialise this module
******************************************************************************/
void PDPTN5110_IntcTypecChipInit(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    static const uint16_t blockdata = 0xFFFF;

    ptn5110Instance->intcLastStatus = 0;
    /* Hardware default state - all interrupts are enabled (bit 15 is used to enable extend alert) */
    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
    {
        ptn5110Instance->cacheEna = 0xFFFF;
    }
    else
    {
        ptn5110Instance->cacheEna = 0x8FFF;
    }

    PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_Intc);

    /*  Enable and clear all interrupt */
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&blockdata, 2, alert_mask);
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&blockdata, 2, alert);
    if (Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP)
    {
        if (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110)
        {
            Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&blockdata, 2, ptn5110_ext_alert_mask);
            Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&blockdata, 2, ptn5110_ext_alert);
        }
    }
    /* Unmask power_status_mask & fault_status_mask */
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&blockdata, 2, power_status_mask);
}

/*! **************************************************************************
    \brief Callback to be called on assertion of INT_N by victoria
******************************************************************************/
void PDPTN5110_IntcIntNCallback(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint8_t tmp8Val = Reg_BusReadByte(ptn5110Instance, receive_detect);
    if ((tmp8Val == 0) && (ptn5110Instance->msgRxSopMask != 0))
    {
        tmp8Val = ptn5110Instance->msgRxSopMask;
        ptn5110Instance->msgRxSopMask = 0;
        PDPTN5110_MsgHalSetRxSopEnable(ptn5110Instance, tmp8Val);
    }

    PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_Intc);

    PDPTN5110_IntcProcessIntAll(ptn5110Instance);
}

/*! **************************************************************************
    \brief Clear and enable external interrupt
******************************************************************************/
void PDPTN5110_IntcIrqClearAndEnable(pd_phy_ptn5110_instance_t *ptn5110Instance, uint16_t mask)
{
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&mask, 2, alert);
    ptn5110Instance->cacheEna |= mask;
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->cacheEna, 2, alert_mask);
}

/*! **************************************************************************
    \brief Clear and enable external interrupt
******************************************************************************/
void PDPTN5110_IntcIrqEnable(pd_phy_ptn5110_instance_t *ptn5110Instance, uint16_t mask)
{
    ptn5110Instance->cacheEna |= mask;
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->cacheEna, 2, alert_mask);
}

/*! **************************************************************************
    \brief Disable external interrupt
******************************************************************************/
void PDPTN5110_IntcIrqDisable(pd_phy_ptn5110_instance_t *ptn5110Instance, uint16_t mask)
{
    ptn5110Instance->cacheEna &= (uint16_t) ~(uint16_t)mask;
    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->cacheEna, 2, alert_mask);
}

/*! **************************************************************************
    \brief Query if an external interrupts has bneen seen
    \note must not be called from ISR, only from task
    \note The status will be accumalative since the last time it was called, calling will clear the status.
******************************************************************************/
static uint16_t PDPTN5110_IntcGetLastExtIntStatus(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint16_t ret;
    /* NOTE: we use a CS here, but not within the ISR */
    /* Each ISR can only write to one location, so cannot be pre-empted */
    /* and this function cannot interrupt and ISR.... but can be interrupted */
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    ret = ptn5110Instance->intcLastStatus;
    ptn5110Instance->intcLastStatus = 0;
    USB_OSA_EXIT_CRITICAL();
    return ret;
}

/*! **************************************************************************
    \brief Query if an external tx done interrupts has been seen
    \note must not be called from ISR, only from task
******************************************************************************/
uint16_t PDPTN5110_IntcTxDoneSeen(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    return (PDPTN5110_IntcGetLastExtIntStatus(ptn5110Instance) & TX_DONE_MASK);
}

/*! **************************************************************************
    \brief Handler for Int10
******************************************************************************/
void PDPTN5110_IntcProcessIntAll(pd_phy_handle pdPhyHandle)
{
    pd_phy_ptn5110_instance_t *ptn5110Instance = (pd_phy_ptn5110_instance_t *)pdPhyHandle;

    uint16_t ClrValue = Reg_CacheRead(ptn5110Instance, INTC, alert);

    if (ClrValue)
    {
        /* Clear only SEEN status */
        uint16_t RegValue = ClrValue & ptn5110Instance->cacheEna;

        /* We use the Transmit status bits out of this */
        ptn5110Instance->intcLastStatus |= ClrValue;

        if ((RegValue == 0) && ((ClrValue & (uint16_t) ~(uint16_t)(ptn5110Instance->cacheEna)) != 0))
        {
            /* Report an error if only a masked interrupt is seen */
        }

        /*  NOTE: We MUST process msg_rcvd_and_available first */
        /* the TxDone callback will use the status to evaluate a TX/RX collision */
        /* We will always receive the msg no matter RX_BUFFER_OVERFLOW or not */
        /* 1.ALERT_RECEIVE_SOP_MESSAGE_STATUS only, which means a normal msg is waiting to be received */
        /* 2.ALERT_RECEIVE_SOP_MESSAGE_STATUS + ALERT_RX_BUFFER_OVERFLOW, which means that B msg is receiving by TCPC */
        /* when A msg has been received by TCPC. */
        /*   In this case, B msg will be discarded and alert ALERT_RX_BUFFER_OVERFLOW, A msg is still waiting to be */
        /*   received. B won't disrupt A. */
        if (RegValue & TCPC_ALERT_RECEIVE_SOP_MESSAGE_STATUS_MASK)
        {
            if (!(ptn5110Instance->tcpcRegCache.CONTROL.tcpc_control & TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK))
            {
                PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_MsgRX);
                PDPTN5110_MsgInterruptCallbackMsgRcvd(ptn5110Instance);
            }
        }

        if ((RegValue & TX_HARDRESET_MASK) == TX_HARDRESET_MASK)
        {
            /* Hard reset clears the alert_mask, so reinstate it here */
            Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->cacheEna, 2, alert_mask);
        }

        if (RegValue & TCPC_ALERT_RECEIVED_HARD_RESET_MASK)
        {
            /* Hard reset clears the alert_mask, so reinstate it here */
            Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ptn5110Instance->cacheEna, 2, alert_mask);
            PDPTN5110_MsgInterruptCallbackHardReset(ptn5110Instance);
        }

        /* TX done is last so it can inspect the above status */
        if (RegValue & TX_ABORT_MASK)
        {
            /* PDPTN5110_MsgInterruptCallbackTxAbort(ptn5110Instance); */
            PDPTN5110_MsgHalSendAbortIsrProcess(ptn5110Instance);
        }
        else if (RegValue & TCPC_ALERT_TRANSMIT_SOP_MESSAGE_SUCCESSFUL_MASK)
        {
            PDPTN5110_MsgHalSendSuccessIsrProcess(ptn5110Instance);
        }
        else
        {
        }

        /* Check VBUS Voltage Alarm Lo alert */
        if (RegValue & TCPC_ALERT_VBUS_VOLTAGE_ALARM_LO_MASK)
        {
            PDPTN5110_VbusVoltageAlarmLoIntCallback(ptn5110Instance);
        }

        /* Fault status */
        if (RegValue & TCPC_ALERT_FAULT_MASK)
        {
            uint8_t faultStatus;
            PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_FaultStatus);
            faultStatus = Reg_CacheRead(ptn5110Instance, FAULT_STATUS, fault_status);
            Reg_CacheAndBusModifyByteField(ptn5110Instance, FAULT_STATUS, fault_status, 0xFF, faultStatus);

            /* TODO: fault process */
            if (faultStatus & TCPC_FAULT_STATUS_FORCE_DISCHARGE_FAILED_MASK)
            {
                PDPTN5110_ConnectRawVbusDischarge(ptn5110Instance, 0);
            }
            if (faultStatus & (TCPC_FAULT_STATUS_INTERNAL_OR_EXTERNAL_VBUS_OVER_CURRENT_PROTECTION_FAULT_MASK |
                               TCPC_FAULT_STATUS_INTERNAL_OR_EXTERNAL_VBUS_OVER_VOLTAGE_PROTECTION_FAULT_MASK))
            {
                /* when dead battery boot up,
                 * TCPC_FAULT_STATUS_INTERNAL_OR_EXTERNAL_VBUS_OVER_VOLTAGE_PROTECTION_FAULT_MASK is set
                 * and TCPC_FAULT_STATUS_MASK_ALL_REGISTERS_RESET_TO_DEFAULT_INTERRUPT_STATUS_MASK_MASK is set */
                if (!(faultStatus & TCPC_FAULT_STATUS_MASK_ALL_REGISTERS_RESET_TO_DEFAULT_INTERRUPT_STATUS_MASK_MASK))
                {
                    /* remove TCPC_FAULT_STATUS_I2_C_INTERFACE_ERROR_MASK, because dead battery boot up with this I2C
                     * error
                     */
                    PDPTN5110_ConnectDisableAll(ptn5110Instance);
                    PDPTN5110_ConnectSync(ptn5110Instance);
                    PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_FAULT_RECOVERY, &faultStatus);
                    PDPTN5110_ResetFetControl(ptn5110Instance);
                }
            }
        }

        /* Extended status */
        if (RegValue & TCPC_ALERT_EXTENDED_STATUS_MASK)
        {
            PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_ExtendedStatus);
        }

        /* Power status */
        if (RegValue & TCPC_ALERT_PORT_POWER_STATUS_MASK)
        {
            /* When using the autonomous state machine, power_control may change, so sync it here */
            if (ptn5110Instance->roleControlUpdated)
            {
                PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_PowerStatus);
            }
            else
            {
                PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_PowerStatus | kRegModule_Control);
            }
            PDPTN5110_PsStatChangeIntCallback(ptn5110Instance);
        }

        /* ALERT_RX_BUFFER_OVERFLOW only, which means that a message longer than TCPC extend buffer has been received */
        /* and discarded */
        /* In this case, no Normal SOP is received, no ALERT_RECEIVE_SOP_MESSAGE_STATUS, we just need to clear this */
        /* alert */
        /* But we still need to use both ALERT_RECEIVE_SOP_MESSAGE_STATUS and ALERT_RX_BUFFER_OVERFLOW to clear */
        /* ALERT_RX_BUFFER_OVERFLOW, as per TCPC spec. */
        if (RegValue & TCPC_ALERT_RX_BUFFER_OVERFLOW_MASK)
        {
            /* Must write back the receive bit to clear the overflow flag */
            ClrValue |= TCPC_ALERT_RECEIVE_SOP_MESSAGE_STATUS_MASK;
        }
    }

    /********************************* TODO ***************************************/
    /*********************** handle other interrupts ******************************/

    if ((Reg_CacheRead(ptn5110Instance, GLOBAL, vendor_id) == PD_VENDOR_ID_NXP) &&
        (Reg_CacheRead(ptn5110Instance, GLOBAL, product_id) == PRODUCT_ID_PTN5110))
    {
        uint16_t ExtClrValue = 0;

        if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
        {
            if (ClrValue & TCPC_ALERT_ALERT_EXTENDED_MASK)
            {
                ExtClrValue = Reg_BusReadByte(ptn5110Instance, alert_extended);
            }
        }
        else
        {
            /* Handle A0-01 and A0-02 silicon differently */
            if ((Reg_CacheRead(ptn5110Instance, GLOBAL, device_id) == DEVICE_ID_PTN5110_A0R1) ||
                (ClrValue & TCPC_ALERT_VENDOR_DEFINED_EXTENDED_MASK))
            {
                ExtClrValue = Reg_BusReadByte(ptn5110Instance, ptn5110_ext_alert);
            }
        }

        if (ExtClrValue)
        {
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            uint16_t ExtRegValue = ExtClrValue;
            uint16_t frsSigReceivedMask = PTN5110_EXT_ALERT_FAST_ROLE_SWAP_REQUEST_RECEIVED_MASK;
#endif

            /* Read this just for information */
            Reg_BusReadByte(ptn5110Instance, ptn5110_ext_status);

#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
            {
                frsSigReceivedMask = TCPC_ALERT_EXTENDED_SINK_FAST_ROLE_SWAP_MASK;
            }
            /* Check FR Swap signal recieved */
            if (ExtRegValue & frsSigReceivedMask)
            {
                PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_FR_SWAP_SINGAL_RECEIVED, NULL);
            }

            /* Check VSafe5V has been sourced */
            if (ExtRegValue & PTN5110_EXT_ALERT_FAST_ROLE_SWAP_VSAFE5V_BEING_SOURCED_MASK)
            {
                ptn5110Instance->frSwapSourceOK = 1;
            }
#endif

            /* Clear all valid extend interrupts after all interrupts have been processed */
            if (ExtClrValue)
            {
                if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
                {
                    Reg_BusWriteByte(ptn5110Instance, alert_extended, (uint8_t)(ExtClrValue & 0xFF));
                }
                else
                {
                    Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ExtClrValue, 2, ptn5110_ext_alert);
                }
            }
            /* SendSignal = true; */
        }
    }

    /********************************* TODO ***************************************/
    /*********************** handle other extended interrupts ******************************/

    if (ClrValue)
    {
#if defined(USBPD_SUPPORT_DEVICE_PTN9601_A1_WORKAROUND) && defined(USBPD_TCPC_AUTO_SM) && defined(USBPD_DEVICE_TCPC)
        uint8_t connection_state;
#endif

        /* Reset the flooding interrupt counter when we see any valid interrupt */
        /* Clear all valid interrupts after all interrupts have been processed */
        Reg_BusWriteBlock(ptn5110Instance, (uint8_t *)&ClrValue, 2, alert);

        /* Read CC_STATUS after clearing the alert, as it could be updated */
        if (ClrValue & TCPC_ALERT_CC_STATUS_ALERT_MASK)
        {
            PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_CCStatus);
            /* Update EC registers */
            PDPTN5110_ConnectINTcTypeCCurrentStable(ptn5110Instance);
            /* Check for detach during hard reset */
            PDPTN5110_ConnectInTcChecKDetacHDurIngHardReset(ptn5110Instance);
        }
    }
}

/*! **************************************************************************
    \brief  Hardware specific register settings for power  mode
******************************************************************************/
bool PDPTN5110_PwrDoApplyCurState(pd_phy_ptn5110_instance_t *ptn5110Instances, uint8_t pwrState)
{
    ptn5110Instances->lowPowerState = pwrState;
    return true;
}

/*! **************************************************************************
    \brief  Check for PorComplete status and wait if needed
    \note Only called from init
******************************************************************************/
void PDPTN5110_PwrWaitForPOrCOmpletEAndEnableClocks(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    uint8_t count = 0;

    count = 0;

    while ((Reg_BusReadByte(ptn5110Instance, power_status) & TCPC_POWER_STATUS_TCPC_INITIALIZATION_STATUS_MASK) ||
           /* Check for NAK which returns contents of zero */
           (Reg_BusReadByte(ptn5110Instance, pd_interface_rev) == 0x0000))
    {
        if (count++ > 200)
        {
            /* Assume the chip has booted and PHY is broken after 200ms */
            break;
        }
        PD_WaitUsec(1000);
    }
    /* Use the lowest level interface for this, cause tasks are not running yet */
    ptn5110Instance->lowPowerState = kPhyPower_NormalMode;
    PDPTN5110_PwrDoApplyCurState(ptn5110Instance, kPhyPower_NormalMode);
}

/*! **************************************************************************
    \brief Initialise this module
******************************************************************************/
void PDPTN5110_IocTypecChipInit(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    /* TODO */
}

/*! **************************************************************************
    \brief Control the DBGACC_FOUND pin
    \note Only to be called from CLP task
******************************************************************************/
void PDPTN5110_IocSetDbgAccFound(pd_phy_ptn5110_instance_t *ptn5110Instance, bool enable)
{
    /* TODO */
}

/*! **************************************************************************
    \brief Control the CC_ORIENT pin
    \note Only to be called from CLP task
******************************************************************************/
void PDPTN5110_IocHalSetCCOrient(pd_phy_ptn5110_instance_t *ptn5110Instance, bool cc_orient_reverse)
{
    /* TODO */
}
