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

#include "usb_pd_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "usb_pd.h"
#include "usb_pd_phy.h"
#include "usb_pd_ptn5110.h"
#include "usb_pd_ptn5110_register.h"
#include "usb_pd_spec.h"
#include "string.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PDPTN5110_VbusIsPresent(PORT) \
    ((bool)(Reg_CacheReadField(ptn5110Instance, POWER_STATUS, power_status, TCPC_POWER_STATUS_VBUS_PRESENT_MASK)))

#define PDPTN5110_VbusIsVSafe5V(ptn5110Instance) PDPTN5110_VbusIsPresent(ptn5110Instance)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void PDPTN5110_ConnectRawVconnDischarge(pd_phy_ptn5110_instance_t *ptn5110Instance, uint8_t discharge_enable);
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
void PDPTN5110_SignalFRSwap(pd_phy_ptn5110_instance_t *ptn5110Instance);
void PDPTN5110_EnableFRSwap(pd_phy_ptn5110_instance_t *ptn5110Instance);
#endif
void PDPTN5110_MsgHalSetupBist(pd_phy_ptn5110_instance_t *ptn5110Instance, pd_bist_mst_t mode);

/*******************************************************************************
 * Variables
 ******************************************************************************/

USB_RAM_ADDRESS_ALIGNMENT(4) pd_phy_ptn5110_instance_t s_PDPhyPTN5110Instance[PD_CONFIG_PTN5110_PORT];

/*******************************************************************************
 * Code
 ******************************************************************************/

static void PDPTN5110_DetectAttach(pd_phy_ptn5110_instance_t *ptn5110Instance, void *param)
{
    pd_attach_detection_param_t *detectParam = (pd_attach_detection_param_t *)param;

    /* Disable AutoDischarge before PR Swap, which ensure that the Typec connection state is updated to correct state as
     * well */
    if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) < PD_INTERFACE_REV2VER1)
    {
        PDPTN5110_ConnectReleasePowerControl(ptn5110Instance);
    }
    if (!(detectParam->isDRP))
    {
        if (detectParam->powerRole == kPD_PowerRoleSource)
        {
            PDPTN5110_ConnectAssertRpUnattached(ptn5110Instance, detectParam->srcRpCurrent - 1);
        }
        else
        {
            PDPTN5110_ConnectAssertRdUnattached(ptn5110Instance);
        }
    }
    else
    {
        if (detectParam->powerRole == kPD_PowerRoleSource)
        {
            PDPTN5110_ConnectAssertDrpUnattached(ptn5110Instance, 1, detectParam->srcRpCurrent - 1);
        }
        else
        {
            PDPTN5110_ConnectAssertDrpUnattached(ptn5110Instance, 0, detectParam->srcRpCurrent - 1);
        }
    }

    PDPTN5110_ConnectSync(ptn5110Instance);
}

static void PDPTN5110_DetectDetach(pd_phy_ptn5110_instance_t *ptn5110Instance, void *param)
{
    pd_detach_detection_param_t *detectParam = (pd_detach_detection_param_t *)param;

    if (detectParam->powerRole == kPD_PowerRoleSource)
    {
        if (detectParam->typecConnectState == kTYPEC_ConnectSource)
        {
            PDPTN5110_ConnectAssertRpAttached(ptn5110Instance, detectParam->usedCC, detectParam->srcRpCurrent);
        }
        else if (detectParam->typecConnectState == kTYPEC_ConnectDebugAccessory)
        {
            if (detectParam->debugUnoriented)
            {
                if (detectParam->debugDTS)
                {
                    PDPTN5110_ConnectAssertRpDbgAccSrc(ptn5110Instance);
                }
            }
            else
            {
                PDPTN5110_ConnectDbgAccSrcAttached(ptn5110Instance);
            }
        }
        else if (detectParam->typecConnectState == kTYPEC_ConnectAudioAccessory)
        {
            PDPTN5110_ConnectAudioAccessoryAttached(ptn5110Instance, detectParam->usedCC);
        }
        else
        {
        }
    }
    else
    {
        if (detectParam->typecConnectState == kTYPEC_ConnectSink)
        {
            PDPTN5110_ConnectAssertRdAttached(ptn5110Instance, detectParam->usedCC, detectParam->snkDetachDetectCCOpen);
        }
        else if (detectParam->typecConnectState == kTYPEC_ConnectDebugAccessory)
        {
            if (detectParam->debugUnoriented)
            {
                if (detectParam->debugDTS)
                {
                    PDPTN5110_ConnectAssertRdDbgAccSnk(ptn5110Instance);
                }
            }
            else
            {
                PDPTN5110_ConnectDbgAccSnkAttached(ptn5110Instance);
            }
        }
        else
        {
        }
    }

    PDPTN5110_ConnectSync(ptn5110Instance);
}

static pd_phy_ptn5110_instance_t *PDPTN5110_GetInstance(void)
{
    uint8_t i = 0;
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    for (; i < PD_CONFIG_MAX_PORT; i++)
    {
        if (s_PDPhyPTN5110Instance[i].occupied != 1)
        {
            uint8_t *buffer = (uint8_t *)&s_PDPhyPTN5110Instance[i];
            for (uint32_t j = 0U; j < sizeof(pd_phy_ptn5110_instance_t); j++)
            {
                buffer[j] = 0x00U;
            }
            s_PDPhyPTN5110Instance[i].occupied = 1;
            USB_OSA_EXIT_CRITICAL();
            return &s_PDPhyPTN5110Instance[i];
        }
    }
    USB_OSA_EXIT_CRITICAL();
    return NULL;
}

static void PDPTN5110_ReleaseInstance(pd_phy_ptn5110_instance_t *ptn5110Instance)
{
    USB_OSA_SR_ALLOC();
    USB_OSA_ENTER_CRITICAL();
    ptn5110Instance->occupied = 0;
    USB_OSA_EXIT_CRITICAL();
}

pd_status_t PDPTN5110_Init(pd_handle upperLayerHandle, pd_phy_handle *pdPhyHandle, pd_phy_config_t *phyConfig)
{
    pd_phy_ptn5110_instance_t *ptn5110Instance;
    uint8_t *buffer;

    ptn5110Instance = PDPTN5110_GetInstance();
    if (ptn5110Instance == NULL)
    {
        return kStatus_PD_Error;
    }
    ptn5110Instance->interfaceParam = phyConfig->interfaceParam;
    ptn5110Instance->phyDataRole = kPD_DataRoleNone;
    ptn5110Instance->phyPowerRole = kPD_PowerRoleNone;
    ptn5110Instance->cacheRxValid = 0;
    ptn5110Instance->pdHandle = upperLayerHandle;
    ptn5110Instance->txHave = 0;
    ptn5110Instance->rxHave = 0;
    *pdPhyHandle = ptn5110Instance;

    CMSIS_PortControlInterfaceInit(&(ptn5110Instance->cmsisAdapter), phyConfig->interface, NULL);

    for (buffer = (uint8_t *)&ptn5110Instance->tcpcRegCache;
         buffer < ((uint8_t *)&ptn5110Instance->tcpcRegCache) + sizeof(pd_phy_TCPC_reg_cache_t); buffer++)
    {
        *buffer = 0;
    }

    PDPTN5110_HalInit(ptn5110Instance);
    PDPTN5110_ConnectInit(ptn5110Instance);

    PDPTN5110_MsgHalDisableMessageRx(ptn5110Instance);

    PDPTN5110_PwrWaitForPOrCOmpletEAndEnableClocks(ptn5110Instance);

    PDPTN5110_ConnectEnableTypeCConnection(ptn5110Instance);
    /* PDPTN5110_ConnectDisableAll(ptn5110Instance); remove it for dead battery */
    PDPTN5110_ConnectSync(ptn5110Instance);

    return kStatus_PD_Success;
}

pd_status_t PDPTN5110_Deinit(pd_phy_handle pdPhyHandle)
{
    pd_phy_ptn5110_instance_t *ptn5110Instance = (pd_phy_ptn5110_instance_t *)pdPhyHandle;
    CMSIS_PortControlInterfaceDeinit(ptn5110Instance->cmsisAdapter);
    ptn5110Instance->cmsisAdapter = NULL;
    PDPTN5110_ReleaseInstance(ptn5110Instance);
    return kStatus_PD_Success;
}

pd_status_t PDPTN5110_Control(pd_phy_handle pdPhyHandle, uint32_t control, void *param)
{
    uint32_t paramVal;
    pd_status_t status = kStatus_PD_Success;
    pd_phy_ptn5110_instance_t *ptn5110Instance = (pd_phy_ptn5110_instance_t *)pdPhyHandle;

    /* common process */
    switch (control)
    {
        case PD_PHY_FR_SWAP_CHECK_VBUS_APPLIED:
        case PD_PHY_GET_VBUS_POWER_STATE:
        case PD_PHY_SNK_GET_TYPEC_CURRENT_CAP:
        case PD_PHY_GET_CC_LINE_STATE:
        case PD_PHY_GET_LOWPOWER_STATE:
        case PD_PHY_GET_BIST_MODE:
        case PD_PHY_GET_BIST_ERR_COUNT:
        case PD_PHY_SET_MSG_HEADER_INFO:
            if (param == NULL)
            {
                return kStatus_PD_Error;
            }
            break;

        default:
            break;
    }

    switch (control)
    {
        case PD_PHY_SIGNAL_FR_SWAP:
        {
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
            if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
            {
                ptn5110Instance->frSwapSourceOK = 0;
                PDPTN5110_EnableFRSwap(ptn5110Instance);
            }
            PDPTN5110_SignalFRSwap(ptn5110Instance);
#endif
            break;
        }

        case PD_PHY_CONTROL_FR_SWAP:
        {
            if (*((uint8_t *)param))
            {
#if defined(PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE) && (PD_CONFIG_PD3_FAST_ROLE_SWAP_ENABLE)
                ptn5110Instance->frSwapSourceOK = 0;
                PDPTN5110_EnableFRSwap(ptn5110Instance);
#endif
            }
            else
            {
                PDPTN5110_DisableFRSwap(ptn5110Instance);
            }
        }
        break;

        case PD_PHY_UPDATE_STATE:
        {
            PDPTN5110_IntcIntNCallback(ptn5110Instance);
            break;
        }

        case PD_PHY_GET_PHY_VENDOR_INFO:
        {
            pd_phy_vendor_info_t *phyInfo = (pd_phy_vendor_info_t *)param;
            phyInfo->vendorID = ptn5110Instance->tcpcRegCache.GLOBAL.vendor_id;
            phyInfo->productID = ptn5110Instance->tcpcRegCache.GLOBAL.product_id;
            phyInfo->deviceID = ptn5110Instance->tcpcRegCache.GLOBAL.device_id;
            break;
        }

        case PD_PHY_CONTROL_VBUS_DETECT:
            if (*((uint8_t *)param))
            {
                PDPTN5110_VbusEnableMonitorAndDetect(ptn5110Instance);
            }
            else
            {
                PDPTN5110_VbusDisableMonitorAndDetect(ptn5110Instance);
            }
            break;

        case PD_PHY_SET_VBUS_TRANSFORM_STATE:
            paramVal = *((uint8_t *)param);
            if (paramVal != kVbusPower_Invalid)
            {
                PDPTN5110_ConnectSetInProgress(ptn5110Instance, (uint8_t)paramVal);
            }
            break;

        case PD_PHY_CONFIG_ATTACH_DETECTION:
            PDPTN5110_DetectAttach(ptn5110Instance, param);
            break;

        case PD_PHY_CONFIG_DETACH_DETECTION:
            PDPTN5110_DetectDetach(ptn5110Instance, param);
            break;

        /* disable attach and detach detection:
           when connect state machine start, or enter disable, error state */
        case PD_PHY_RESET_CONNECT_DETECTION:
            PDPTN5110_ConnectDisableComparators(ptn5110Instance);
            PDPTN5110_ConnectDisableAll(ptn5110Instance);
            PDPTN5110_ConnectDetachOrDisableCallback(ptn5110Instance);
            PDPTN5110_ConnectSync(ptn5110Instance);
            break;

        /* enter power state */
        case PD_PHY_ENTER_LOW_POWER_STATE:
            PDPTN5110_PwrDoApplyCurState(ptn5110Instance, *((uint8_t *)param));
            break;

        /* pd_ptn5110_ctrl_pin_t* */
        case PD_PHY_CONTROL_POWER_PIN:
            PDPTN5110_SetFetControl(ptn5110Instance, (pd_ptn5110_ctrl_pin_t *)param);
            break;

        /* pd_cc_control_t */
        case PD_PHY_CONNECT_SET_CC:
            PDPTN5110_ConnectSetCC(ptn5110Instance, *((uint8_t *)param));
            break;

        /* enable/disable */
        case PD_PHY_DISCHARGE_VBUS:
            /* discharge the vbus */
            PDPTN5110_ConnectRawVbusDischarge(ptn5110Instance, *((uint8_t *)param));
            break;

        /* pd_power_role_t */
        /* pd_data_role_t */
        case PD_PHY_SET_MSG_HEADER_INFO:
        {
            pd_phy_msg_header_info_t *headerInfo = (pd_phy_msg_header_info_t *)param;
            ptn5110Instance->phyPowerRole = headerInfo->powerRole;
            ptn5110Instance->phyDataRole = headerInfo->dataRole;
            ptn5110Instance->revision = headerInfo->revision;

            PDPTN5110_MsgHalSetPortRole(ptn5110Instance, headerInfo->revision, headerInfo->powerRole,
                                        headerInfo->dataRole);
            break;
        }

        /* enable/disable */
        case PD_PHY_CONTROL_VCONN:
            PDPTN5110_ConnectSwitchVConn(ptn5110Instance, *((uint8_t *)param));
            break;

        /* enable/disable */
        case PD_PHY_DISCHARGE_VCONN:
#ifdef USBPD_ENABLE_VCONN_DISCHARGE
            PDPTN5110_ConnectRawVconnDischarge(ptn5110Instance, *((uint8_t *)param));
            PDPTN5110_ConnectSync(ptn5110Instance);
#endif
            break;

        /* typec Rp current value */
        case PD_PHY_SRC_SET_TYPEC_CURRENT_CAP:
            PDPTN5110_ConnectSrcSetTypecCurrent(ptn5110Instance, ptn5110Instance->usedCC, *((uint8_t *)param));
            PDPTN5110_ConnectSync(ptn5110Instance);
            break;

        /* there is no param */
        /* reset phy protocol layer message function */
        case PD_PHY_RESET_MSG_FUNCTION:
            PDPTN5110_MsgSendCompLete(pdPhyHandle, kStatus_PD_Abort);
            PDPTN5110_MsgReceiveCompLete(pdPhyHandle, kStatus_PD_Abort);
            PDPTN5110_MsgHalClearPendingAndAbort(ptn5110Instance);
            PDPTN5110_MsgResetAllMsgId(ptn5110Instance);
            PDPTN5110_MsgHalProtocolLayerResetAndPowerUp(ptn5110Instance);
            break;

        /* disable message rx */
        case PD_PHY_DISABLE_MSG_RX:
            PDPTN5110_MsgHalDisableMessageRx(ptn5110Instance);
            PDPTN5110_MsgHalPowerDownRx(ptn5110Instance);
            break;

        /* disable message tx */
        case PD_PHY_DISABLE_MSG_TX:
            PDPTN5110_MsgHalPowerDownTx(ptn5110Instance);
            break;

        /* bist mode */
        case PD_PHY_ENTER_BIST:
            PDPTN5110_MsgHalSetupBist(ptn5110Instance, (pd_bist_mst_t) * ((uint8_t *)param));
            break;

        case PD_PHY_EXIT_BIST:
            ptn5110Instance->tcpcRegCache.CONTROL.tcpc_control &=
                (uint8_t) ~(uint8_t)(TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK);
            Reg_BusClrBit(ptn5110Instance, tcpc_control, TCPC_TCPC_CONTROL_BIST_TEST_MODE_MASK);
            break;

        /* bist */
        case PD_PHY_RESET_BIST:
            break;

        /* cancel transfering tx */
        case PD_PHY_CANCEL_MSG_TX:
            PDPTN5110_MsgSendCompLete(pdPhyHandle, kStatus_PD_Cancel);
            break;

        /* cancel primed rx */
        case PD_PHY_CANCEL_MSG_RX:
            PDPTN5110_MsgReceiveCompLete(pdPhyHandle, kStatus_PD_Cancel);
            break;

        /* hard_reset and cable_reset */
        case PD_PHY_SEND_CABLE_RESET:
        case PD_PHY_SEND_HARD_RESET:
            /* TODO: if it is cable_reset, this code should be wrong.
             * TODO: set and check ptn5110Instance->pwrInProgress == kVbusPower_InHardReset. */
            if (control == PD_PHY_SEND_HARD_RESET)
            {
                PDPTN5110_MsgHalSendReset(ptn5110Instance, 0); /* hard reset */
            }
            else
            {
                PDPTN5110_MsgHalSendReset(ptn5110Instance, 1); /* cable reset */
            }
            break;

        case PD_PHY_FR_SWAP_CHECK_VBUS_APPLIED:
            if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
            {
                *((uint8_t *)param) = (Reg_CacheReadField(ptn5110Instance, POWER_STATUS, power_status,
                                                          TCPC_POWER_STATUS_SOURCING_VBUS_MASK)) ?
                                          1 :
                                          0;
            }
            else
            {
                *((uint8_t *)param) = ptn5110Instance->frSwapSourceOK;
            }
            break;

        case PD_PHY_GET_VBUS_POWER_STATE:
        {
            uint32_t voltage = PDPTN5110_GetVbusVoltage(ptn5110Instance);
            paramVal = 0;

            if ((voltage <= VBUS_VSAFE5V_MAX_THRESHOLD) && (voltage >= VBUS_VSAFE5V_MIN_THRESHOLD))
            {
                if (!PDPTN5110_VbusIsPresent(ptn5110Instance))
                {
                    PDPTN5110_RegCacheSynC(ptn5110Instance, kRegModule_PowerStatus);
                }
                paramVal |= PD_VBUS_POWER_STATE_VSAFE5V_MASK;
            }

            if (voltage <= VBUS_VSAFE0V_MAX_THRESHOLD)
            {
                paramVal |= PD_VBUS_POWER_STATE_VSAFE0V_MASK;
            }

            if (PDPTN5110_VbusIsPresent(ptn5110Instance))
            {
                paramVal |= PD_VBUS_POWER_STATE_VBUS_MASK;
            }

            if (PDPTN5110_VsysIsPresent(ptn5110Instance))
            {
                paramVal |= PD_VBUS_POWER_STATE_VSYS_MASK;
            }

            *((uint8_t *)param) = (uint8_t)paramVal;
            break;
        }

        /* Read the current current limit */
        case PD_PHY_SNK_GET_TYPEC_CURRENT_CAP:
            if (ptn5110Instance->currentStable)
            {
                *((uint8_t *)param) = PDPTN5110_ConnectGetTypeCCurrent(ptn5110Instance, ptn5110Instance->usedCC);
            }
            else
            {
                *((uint8_t *)param) = kCurrent_Invalid;
            }
            break;

        case PD_PHY_GET_CC_LINE_STATE:
            PDPTN5110_ConnectGetCC(ptn5110Instance, (pd_phy_get_cc_state_t *)param);
            break;

        case PD_PHY_GET_LOWPOWER_STATE:
            *((uint8_t *)param) = ptn5110Instance->lowPowerState;
            break;

        case PD_PHY_INFORM_VBUS_VOLTAGE_RANGE:
            if (Reg_CacheRead(ptn5110Instance, GLOBAL, pd_interface_rev) >= PD_INTERFACE_REV2VER1)
            {
                ptn5110Instance->vbusVoltage = *((uint32_t *)param);
                paramVal = (ptn5110Instance->vbusVoltage & 0x0000FFFFu);
                paramVal = (paramVal - paramVal * 15 / 100) / 25; /* 25mv unit */
                if (paramVal < VBUS_VSAFE0V_MIN_THRESHOLD)
                {
                    paramVal = VBUS_VSAFE0V_MIN_THRESHOLD;
                }
                Reg_CacheAndBusModifyWordField(
                    ptn5110Instance, VBUS, vbus_voltage_alarm_lo_cfg,
                    TCPC_VBUS_VOLTAGE_ALARM_LO_CFG_VBUS_HI_VOLTAGE_TRIP_POINT_MASK,
                    (paramVal << TCPC_VBUS_VOLTAGE_ALARM_LO_CFG_VBUS_HI_VOLTAGE_TRIP_POINT_LSB));

                paramVal = (ptn5110Instance->vbusVoltage & 0xFFFF0000u) >> 16u;
                paramVal = (paramVal + paramVal * 15 / 100) / 25; /* 25mv unit */
                Reg_CacheAndBusModifyWordField(
                    ptn5110Instance, VBUS, vbus_voltage_alarm_hi_cfg,
                    TCPC_VBUS_VOLTAGE_ALARM_HI_CFG_VBUS_LO_VOLTAGE_TRIP_POINT_MASK,
                    (paramVal << TCPC_VBUS_VOLTAGE_ALARM_HI_CFG_VBUS_LO_VOLTAGE_TRIP_POINT_LSB));
            }
            break;

        case PD_PHY_GET_BIST_MODE:
            *((uint8_t *)param) = 0;
            break;

        case PD_PHY_GET_BIST_ERR_COUNT:
            *((uint8_t *)param) = 0; /* TCPC doesn't support ErrorCnt, so response 0 here. */
            break;

        default:
            status = kStatus_PD_Error;
            break;
    }

    return status;
}

pd_status_t PDPTN5110_Send(pd_phy_handle pdPhyHandle, uint8_t startOfPacket, uint8_t *buffer, uint32_t length)
{
    pd_msg_header_t msgHeader;
    pd_phy_ptn5110_instance_t *ptn5110Instance = (pd_phy_ptn5110_instance_t *)pdPhyHandle;
    uint8_t sendState = kStatus_PD_Error;

    msgHeader.msgHeaderVal = USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(buffer);
    ptn5110Instance->msgTxSop = startOfPacket;
    ptn5110Instance->revision = msgHeader.bitFields.specRevision;

    ptn5110Instance->msgTxBuf = (buffer - 2);
    /* non-extended message */
    if (!(msgHeader.bitFields.extended))
    {
        if (msgHeader.bitFields.NumOfDataObjs == 0)
        {
            /* control msg */
            ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS] = 2;
            sendState = PDPTN5110_MsgHalSendControl(ptn5110Instance, startOfPacket, msgHeader.bitFields.messageType);
        }
        else
        {
            /* data msg */
            ptn5110Instance->msgTxBuf[TCPC_TRANSMIT_BYTE_COUNT_POS] = length;
            sendState = PDPTN5110_MsgHalSendData(ptn5110Instance, startOfPacket, msgHeader.bitFields.messageType,
                                                 (length - 2) >> 2);
        }
    }
    else
    {
#if defined(PD_CONFIG_EXTENDED_MSG_SUPPORT) && (PD_CONFIG_EXTENDED_MSG_SUPPORT)
        /* extend message */
        uint32_t extendMsgHeader = USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(((uint8_t *)(&buffer[2])));
        if (((extendMsgHeader & PD_MSG_EXT_HEADER_CHUNKED_MASK) >> PD_MSG_EXT_HEADER_CHUNKED_POS) == 0)
        {
            /* unchunked message */
            sendState = PDPTN5110_MsgHalSendUnchunked(ptn5110Instance, startOfPacket, msgHeader.bitFields.messageType,
                                                      (length - 2));
        }
        else
        {
            /* chunked message */
            sendState = PDPTN5110_MsgHalSendChunked(ptn5110Instance, startOfPacket, msgHeader.bitFields.messageType,
                                                    (length - 2 + 3) >> 2);
        }
#endif
    }

    if (sendState)
    {
        /* all send/receive call will in the pd stack task context */
        ptn5110Instance->txHave = 1;
        return kStatus_PD_Success;
    }
    else
    {
        return kStatus_PD_Error;
    }
}

pd_status_t PDPTN5110_Receive(pd_phy_handle pdPhyHandle, uint8_t startOfPacketMask, uint8_t *buffer, uint32_t length)
{
    pd_phy_ptn5110_instance_t *ptn5110Instance = (pd_phy_ptn5110_instance_t *)pdPhyHandle;
    USB_OSA_SR_ALLOC();

    USB_OSA_ENTER_CRITICAL();
    if (ptn5110Instance->cacheRxValid)
    {
        ptn5110Instance->cacheRxValid = 0;
        USB_OSA_EXIT_CRITICAL();

        memcpy(buffer, ptn5110Instance->msgRxCacheBuf, ptn5110Instance->cacheRxResult.rxLength);

        PD_Notify(ptn5110Instance->pdHandle, PD_PHY_EVENT_RECEIVE_COMPLETE, (void *)&(ptn5110Instance->cacheRxResult));
        return kStatus_PD_Success;
    }
    ptn5110Instance->rxHave = 1u;
    ptn5110Instance->rxDataBuffer = buffer;
    ptn5110Instance->rxDataLength = length;
    USB_OSA_EXIT_CRITICAL();
    PDPTN5110_MsgHalSetRxSopEnable(ptn5110Instance, startOfPacketMask);

    return kStatus_PD_Success;
}

void PD_PTN5110IsrFunction(pd_handle pdHandle)
{
    if (pdHandle != NULL)
    {
        PD_Notify(pdHandle, PD_PHY_EVENT_STATE_CHANGE, NULL);
    }
}
