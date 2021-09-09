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

#ifndef __PD_H__
#define __PD_H__
#include "fsl_common.h"
#include "usb_osa.h"
#include "usb_misc.h"
#include "usb_cmsis_wrapper.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

struct _pd_phy_config;

/*!
 * @addtogroup usb_pd_stack
 * @{
 */

/*! @brief NXP's USB vendor id */
#define PD_VENDOR_ID_NXP 0x1FC9u
/*! @brief The Specification Revision 1.0 value */
#define PD_SPEC_REVISION_10 (0x00)
/*! @brief The Specification Revision 2.0 value */
#define PD_SPEC_REVISION_20 (0x01)
/*! @brief The Specification Revision 3.0 value */
#define PD_SPEC_REVISION_30 (0x02)
/*! @brief The structured VDM version 1.0 value */
#define PD_SPEC_STRUCTURED_VDM_VERSION_10 (0x00)
/*! @brief The structured VDM version 2.0 value */
#define PD_SPEC_STRUCTURED_VDM_VERSION_20 (0x01)
/*! @brief PD standard ID */
#define PD_STANDARD_ID 0xFF00
/*! @brief source/sink capability's voltage unit is 50mV */
#define PD_PDO_VOLTAGE_UNIT (50)
/*! @brief source/sink capability's current unit is 10mA */
#define PD_PDO_CURRENT_UNIT (10)
/*! @brief source/sink capability's power unit is 250mW */
#define PD_PDO_POWER_UNIT (250)
/*! @brief check detach based on vbus absent */
#define PD_SINK_DETACH_ON_VBUS_ABSENT (1)
/*! @brief check detach based CC line open*/
#define PD_SINK_DETACH_ON_CC_OPEN (2)

/**
* @brief pd error code used for function return value or error status.
*/
typedef enum _pd_status
{
    /*! Failed */
    kStatus_PD_Error = 0x00U,
    /*! Success */
    kStatus_PD_Success,
    /*! abort operation */
    kStatus_PD_Abort,
    /*! cancel operation */
    kStatus_PD_Cancel,
} pd_status_t;

/**
* @brief PD phy type, used in the #pd_instance_config_t.
*/
typedef enum _pd_phy_type
{
    /*! PHY is PTN5110 */
    kPD_PhyPTN5110,
    /*! PHY is PTN5100 */
    kPD_PhyPTN5100,
} pd_phy_type_t;

/**
* @brief The device's type, used in the #pd_instance_config_t.
*/
typedef enum _pd_device_type
{
    /*! The device works as normal power port, for example: source, sink and DRP. */
    kDeviceType_NormalPowerPort,
    /*! The device works as cable. */
    kDeviceType_Cable,
    /*! The device works as audio accessory. */
    kDeviceType_AudioAccDevice,
    /*! The device works as debug accessory. */
    kDeviceType_DebugAccDevice,
    /*! The device works as one type alternate mode, for example; displayport. */
    kDeviceType_AlternateModeProduct,
} pd_device_type_t;

/**
* @brief configure device's type, used in the #pd_instance_config_t.
*/
typedef enum _typec_power_role_config
{
    /*! only work as sink */
    kPowerConfig_SinkOnly,
    /*! work as default sink, but have ability to swap as source.
     * only can connect with source*/
    kPowerConfig_SinkDefault,
    /*! only work as source */
    kPowerConfig_SourceOnly,
    /*! work as default source, but have ability to swap as sink.
     * only can connect with sink*/
    kPowerConfig_SourceDefault,
    /*! DRP toggling, please reference to TypeC spec
     * can connect with source or sink*/
    kPowerConfig_DRPToggling,
    /*! DRP, can work as USB device but cannot work as USB host, please reference to TypeC spec
     * can connect with source or sink*/
    kPowerConfig_DRPSourcingDevice,
    /*! DRP, can work as USB host but cannot work as USB device, please reference to TypeC spec
     * can connect with source or sink*/
    kPowerConfig_DRPSinkingHost,
} typec_power_role_config_t;

/**
* @brief config typec sink role
*
* configure sink typec role (normal, audio accessory or debug accessory),
* used in the #pd_instance_config_t.
*/
typedef enum _typec_sink_role_config
{
    /*! normal sink
     * add two Rd in two CC, the cable's one CC must be Ra or cut-off */
    kSinkConfig_Normal,
    /*! audio accessory
     * add two Ra in two CC, the cable's one CC must be Ra or line connected */
    kSinkConfig_AudioAcc,
    /*! debug accessory,
     * add two Rd in two CC, the cable's two CC must be line connected. */
    kSinkConfig_DebugAcc,
} typec_sink_role_config_t;

/**
* @brief configure try type function, used in the #pd_instance_config_t.
*/
typedef enum _typec_try
{
    /*! invalid value */
    kTypecTry_None,
    /*! Try-SRC */
    kTypecTry_Src,
    /*! Try-SNK */
    kTypecTry_Snk,
} typec_try_t;

/**
* @brief configure device's data function, used in the #pd_instance_config_t.
*/
typedef enum _typec_data_function_config
{
    /*! invalid value */
    kDataConfig_None,
    /*! downstream facing port. If supporting USB, it correspond to USB host function */
    kDataConfig_DFP,
    /*! upstream facing port, If supporting USB, it correspond to USB device function */
    kDataConfig_UFP,
    /*! dual role datam, it support dr swap. If supporting USB, it can swap between Host and Device function. */
    kDataConfig_DRD,
} typec_data_function_config_t;

/**
* @brief PHY interface
*
* configure PHY's communication interface (I2C or SPI etc),
* used in the #pd_instance_config_t.
*/
typedef enum pd_phy_interface
{
    /*! i2c instance 0 */
    kInterface_i2c0 = 0u,
    /*! i2c instance 1 */
    kInterface_i2c1 = 1U,
    /*! i2c instance 2 */
    kInterface_i2c2 = 2U,
    /*! i2c instance 3 */
    kInterface_i2c3 = 3U,
    /*! i2c instance 4 */
    kInterface_i2c4 = 4U,
    /*! i2c instance 5 */
    kInterface_i2c5 = 5U,
    /*! i2c instance 6 */
    kInterface_i2c6 = 6U,
    /*! spi instance 0 */
    kInterface_spi0 = 0x10u,
    /*! spi instance 1 */
    kInterface_spi1 = 0x11u,
    /*! spi instance 2 */
    kInterface_spi2 = 0x12u,
    /*! spi instance 3 */
    kInterface_spi3 = 0x13u,
    /*! spi instance 4 */
    kInterface_spi4 = 0x15u,
    /*! spi instance 5 */
    kInterface_spi5 = 0x16u,
    /*! spi instance 6 */
    kInterface_spi6 = 0x17u,
    kInterface_end = 0x18u,
} pd_phy_interface_t;

/**
* @brief start of packet's type.
*/
typedef enum _start_of_packet
{
    kPD_MsgSOP = 0,            /*!< sop*/
    kPD_MsgSOPp = 1,           /*!< sop'*/
    kPD_MsgSOPpp = 2,          /*!< sop''*/
    kPD_MsgSOPDbg = 3,         /*!< sop debug*/
    kPD_MsgSOPpDbg = 4,        /*!< sop' debug*/
    kPD_MsgSOPInvalid = 0xFFu, /*!< invalid value*/

    kPD_MsgSOPMask = 0x01u,     /*!< sop mask for msg receive function*/
    kPD_MsgSOPpMask = 0x02u,    /*!< sop' mask for msg receive function*/
    kPD_MsgSOPppMask = 0x04u,   /*!< sop'' mask for msg receive function*/
    kPD_MsgSOPDbgMask = 0x08u,  /*!< sop debug mask for msg receive function*/
    kPD_MsgSOPpDbgMask = 0x10u, /*!< sop' debug mask for msg receive function*/
} start_of_packet_t;

/**
* @brief pd command error code.
*
* For example: if partner reply reject for pr_swap,
* the error code kCommandResult_Reject will return to APP.
*/
typedef enum _pd_command_result
{
    kCommandResult_Accept = 0x01U,                  /*!< partner accept the command or the command success */
    kCommandResult_Success = kCommandResult_Accept, /*!< partner accept the command or the command success */
    kCommandResult_Reject = 0x02U,                  /*!< partner reply reject for this command */
    kCommandResult_Wait = 0x03U,                    /*!< partner reply wait for this command */
    kCommandResult_Error = 0x04U,                   /*!< msg send error or time out without msg received */
    kCommandResult_NotSupported = 0x05U,            /*!< partner reply Not_Supported for this command */
    kCommandResult_VDMACK = 0x06U,                  /*!< partner reply ACK for this VDM command */
    kCommandResult_VDMNAK = 0x07U,                  /*!< partner reply NAK for this VDM command */
    kCommandResult_VDMBUSY = 0x08U,                 /*!< partner reply BUSY for this VDM command */
} pd_command_result_t;

/**
* @brief Typec current level related to Rp value.
*/
typedef enum _typec_current
{
    kCurrent_Invalid = 0, /*!< invalid value */
    kCurrent_StdUSB = 1,  /*!< standard USB current (900mA) */
    kCurrent_1A5 = 2,     /*!< 1.5A */
    kCurrent_3A = 3,      /*!< 3.0A */
} typec_current_val_t;

/**
* @brief PD running power role type.
*/
typedef enum _pd_power_role
{
    kPD_PowerRoleSink,   /*!< sink */
    kPD_PowerRoleSource, /*!< source */
    kPD_PowerRoleNone,   /*!< invalid value */
} pd_power_role_t;

/**
* @brief PD running data role type.
*/
typedef enum _pd_data_role
{
    kPD_DataRoleUFP,  /*!< UFP */
    kPD_DataRoleDFP,  /*!< DFP */
    kPD_DataRoleNone, /*!< invalid value */
} pd_data_role_t;

/**
* @brief Vconn role type (Vconn source or not).
*/
typedef enum _pd_vconn_role
{
    kPD_NotVconnSource, /*!< is vconn source */
    kPD_IsVconnSource,  /*!< is not vconn source */
    kPD_VconnNone,      /*!< invalid value */
} pd_vconn_role_t;

/**
* @brief TypeC connection state, self or partner's device type.
*/
typedef enum _typec_connection_state
{
    /*! invalid value, or not connected */
    kTYPEC_ConnectNone = 0,
    /*!
     * Self is normal source port.
     */
    kTYPEC_ConnectSource = 1,
    /*!
     * Self is normal sink port.
     */
    kTYPEC_ConnectSink = 2,
    /*!
     * If typec connection role is this value, there are two cases:
     * - Self is source port, the connected cable is active.
     * - Self is active cable.
     */
    kTYPEC_ConnectPoweredCable = 3,
    /*!
     * If typec connection role is this value, there are three cases:
     * - Self is source port, the connected partner is normal sink device and the cable is active cable;
     * - Self is source port, the connected partner is vconn powered accessory;
     * - Self is vconn powered accessory.
     */
    kTYPEC_ConnectPoweredCableWithSink = 4,
    /*!
     * Same as kTYPEC_ConnectPoweredCableWithSink
     */
    kTYPEC_ConnectVconnPoweredAccessory = 4,
    /*!
     * If typec connection role is this value, there are two cases:
     * - Self is source port, the connected partner is audio accessory device;
     * - Self is audio accessory.
     */
    kTYPEC_ConnectAudioAccessory = 5,
    /*!
     * If typec connection role is this value, there are two cases:
     * - Self is source port, the connected partner is debug accessory device;
     * - Self is debug accessory.
     */
    kTYPEC_ConnectDebugAccessory = 6,
} typec_port_connect_state_t;

/**
* @brief PHY power state (low power).
*
* This type is not related to the PD power, it is PHY low power state.
*/
typedef enum _pd_phy_pwer_state
{
    kPhyPower_NormalMode,    /*!< normal mode (full powered) */
    kPhyPower_SleepMode,     /*!< sleep mode */
    kPhyPower_DeepSleepMode, /*!< deep sleep mode */
    kPhyPower_PowerDownMode, /*!< power down mode */
} pd_phy_pwr_state_t;

/**
* @brief Structured VDM command values.
*/
typedef enum pd_vdm_command
{
    kVDM_DiscoverIdentity = 1, /*!< discovery identity */
    kVDM_DiscoverSVIDs,        /*!< discovery SVIDs */
    kVDM_DiscoverModes,        /*!< discovery Modes */
    kVDM_EnterMode,            /*!< enter mode */
    kVDM_ExitMode,             /*!< exit mode */
    kVDM_Attention,            /*!< attention */
} pd_vdm_command_t;

/**
* @brief Structured VDM command type values.
*/
typedef enum pd_vdm_command_type
{
    kVDM_Initiator,     /*!< initiator */
    kVDM_ResponderACK,  /*!< ACK */
    kVDM_ResponderNAK,  /*!< NAK */
    kVDM_ResponderBUSY, /*!< BUSY */
} pd_vdm_command_type_t;

/**
* @brief The callback events.
*
* There are two types events here:
* - command flow events.
*   For example: PD_DPM_SNK_HARD_RESET_REQUEST and PD_DPM_SRC_HARD_RESET_REQUEST are for hard reset command.
* - device state callback events.
*   For example: PD_CONNECTED indicate connection
*/
typedef enum _pd_dpm_callback
{
    /* hard reset */
    /*! hard reset sent or received. (only sink receive this event) */
    PD_DPM_SNK_HARD_RESET_REQUEST,
    /*! hard reset sent or received. (only source receive this event) */
    PD_DPM_SRC_HARD_RESET_REQUEST,

    /* pr swap */
    /*! pr swap request, need return accept or reject negotiation result */
    PD_DPM_PR_SWAP_REQUEST,
    /*! pr swap success, indicate power role is changed */
    PD_DPM_PR_SWAP_SUCCESS,
    /*! pr swap fail because reject, error etc */
    PD_DPM_PR_SWAP_FAIL,

    /* fr_swap */
    /*! fast role swap request, need return accept or reject negotiation result */
    PD_DPM_FR_SWAP_REQUEST,
    /*! fast role swap success, indicate power role is changed */
    PD_DPM_FR_SWAP_SUCCESS,
    /*! fast role swap fail because reject, error etc */
    PD_DPM_FR_SWAP_FAIL,

    /* rdo request and power negotiation and get_source_cap */
    /*! rdo request, need return accept or reject negotiation result (only source receive this event) */
    PD_DPM_SRC_RDO_REQUEST,
    /*! check the prev contract valid or not (only source receive this event) */
    PD_DPM_SRC_CONTRACT_STILL_VALID,
    /*! send source cap fail (only source receive this event) */
    PD_DPM_SRC_SEND_SRC_CAP_FAIL,
    /*! rdo request command success (only source receive this event) */
    PD_DPM_SRC_RDO_SUCCESS,
    /*! rdo request fail (only source receive this event) */
    PD_DPM_SRC_RDO_FAIL,
    /*! sink receive partner's source capabilities (only sink receive this event) */
    PD_DPM_SNK_RECEIVE_PARTNER_SRC_CAP,
    /*! sink get request rdo from application (only sink receive this event) */
    PD_DPM_SNK_GET_RDO,
    /*! sink rdo request command success, can use the requested power (only sink receive this event) */
    PD_DPM_SNK_RDO_SUCCESS,
    /*! sink rdo request fail (only sink receive this event) */
    PD_DPM_SNK_RDO_FAIL,

    /* get source cap */
    /*! get partner's source capabilities fail, partner don't support or message transfer fail */
    PD_DPM_GET_PARTNER_SRC_CAP_FAIL,
    /*! receive partner's source capabilities */
    PD_DPM_GET_PARTNER_SRC_CAP_SUCCESS,

    /* goto min */
    /*! source goto min command success (only source receive this event) */
    PD_DPM_SRC_GOTOMIN_SUCCESS,
    /*! sink goto min command success (only sink receive this event) */
    PD_DPM_SNK_GOTOMIN_SUCCESS,
    /*! source goto min command fail, message transfer fail (only source receive this event) */
    PD_DPM_SRC_GOTOMIN_FAIL,
    /*! sink goto min command success, message transfer fail (only sink receive this event) */
    PD_DPM_SNK_GOTOMIN_FAIL,

    /* get partner snk cap */
    /*! receive partner's sink capabilities */
    PD_DPM_GET_PARTNER_SNK_CAP_SUCCESS,
    /*! get partner's sink capabilities fail, partner don't support or message transfer fail */
    PD_DPM_GET_PARTNER_SNK_CAP_FAIL,

    /* dr swap */
    /*! data role swap request, need return accept or reject negotiation result */
    PD_DPM_DR_SWAP_REQUEST,
    /*! data role swap success, indicate data role is changed */
    PD_DPM_DR_SWAP_SUCCESS,
    /*! data role swap fail because reject, error etc */
    PD_DPM_DR_SWAP_FAIL,

    /* vconn swap */
    /*! vconn swap request, need return accept or reject negotiation result */
    PD_DPM_VCONN_SWAP_REQUEST,
    /*! vconn swap success, indicate vconn role is changed */
    PD_DPM_VCONN_SWAP_SUCCESS,
    /*! data role swap fail because reject, error etc */
    PD_DPM_VCONN_SWAP_FAIL,

    /* soft reset */
    /*! If application call PD_Command to send soft_reset, this event indicate the command success */
    PD_DPM_SOFT_RESET_SUCCESS,
    /*! receive soft reset or PD stack send soft reset actively because stack error and send success */
    PD_DPM_SOFT_RESET_REQUEST,
    /*! If application call PD_Command to send soft_reset, this event indicate the command fail */
    PD_DPM_SOFT_RESET_FAIL,

    /* structured vdm:
       discovery identity, discovery svids, discovery modes,
       enter modes, exit modes, attention, vendor structured vdm */
    /*! structured vdm command received, application need return reply in this event */
    PD_DPM_STRUCTURED_VDM_REQUEST,
    /*! structured vdm command success, and receive reply or don't need reply */
    PD_DPM_STRUCTURED_VDM_SUCCESS,
    /*! structured vdm command fail, reply with NAK or BUSY or doesn't receive reply */
    PD_DPM_STRUCTURED_VDM_FAIL,

    /* unstructured vdm */
    /*! unstructured vdm message received */
    PD_DPM_UNSTRUCTURED_VDM_RECEIVED,
    /*! send unstructured vdm message successfully */
    PD_DPM_SEND_UNSTRUCTURED_VDM_SUCCESS,
    /*! send unstructured vdm message fail */
    PD_DPM_SEND_UNSTRUCTURED_VDM_FAIL,

    /* get source ext cap */
    /*! receive partner's request for source extended capabilities, need return capabilities data or not supported */
    PD_DPM_GIVE_SRC_EXT_CAP,
    /*! receive partner's reply data for request */
    PD_DPM_GET_SRC_EXT_CAP_SUCCESS,
    /*! request fail because receiving not supported or message error */
    PD_DPM_GET_SRC_EXT_CAP_FAIL,

    /* get status */
    /*! receive partner's request for status, need return status data or not supported */
    PD_DPM_GIVE_STATUS,
    /*! receive partner's reply data for request */
    PD_DPM_GET_STATUS_SUCCESS,
    /*! request fail because receiving not supported or message error */
    PD_DPM_GET_STATUS_FAIL,

    /* get battery cap */
    /*! receive partner's request for battery cap, need return cap data or not supported */
    PD_DPM_GIVE_BATTERY_CAP,
    /*! receive partner's reply data for request */
    PD_DPM_GET_BATTERY_CAP_SUCCESS,
    /*! request fail because receiving not supported or message error */
    PD_DPM_GET_BATTERY_CAP_FAIL,

    /* get battery status */
    /*! receive partner's request for battery status, need return status data or not supported */
    PD_DPM_GIVE_BATTERY_STATUS,
    /*! receive partner's reply data for request */
    PD_DPM_GET_BATTERY_STATUS_SUCCESS,
    /*! request fail because receiving not supported or message error */
    PD_DPM_GET_BATTERY_STATUS_FAIL,

    /* get manufacturer info */
    /*! receive partner's request for manufacturer info, need return info data or not supported */
    PD_DPM_GIVE_MANUFACTURER_INFO,
    /*! receive partner's reply data for request */
    PD_DPM_GET_MANUFACTURER_INFO_SUCCESS,
    /*! request fail because receiving not supported or message error */
    PD_DPM_GET_MANUFACTURER_INFO_FAIL,

    /* alert */
    /*! receive partner's alert message */
    PD_DPM_ALERT_RECEIVED,
    /*! send alert message success */
    PD_DPM_SEND_ALERT_SUCCESS,
    /*! send alert message fail */
    PD_DPM_SEND_ALERT_FAIL,

    /* calbe reset */
    /*! receive cable reset */
    PD_DPM_CABLE_RESET_REQUEST,
    /*! DFP send cable reset successfully */
    PD_DPM_CABLE_RESET_COMPLETE,

#if 0
    /* security request */
    PD_DPM_RESPONSE_SECURITY_REQUEST,
    PD_DPM_SECURITY_REQUEST_SUCCESS,
    PD_DPM_SECURITY_RESPONSE_RECEIVED,
    PD_DPM_SECURITY_REQUEST_FAIL,
#endif

    /* others */
    /*! PD stack enter the disabled state, need application restart */
    PD_FUNCTION_DISABLED,
    /*! partner is connected */
    PD_CONNECTED,
    /*! connect role change, for example Try.SRC */
    PD_CONNECT_ROLE_CHANGE,
    /*! partner is disconnected */
    PD_DISCONNECTED,
    /*! the product is alternate mode device, and time out before enter alternate mode as typec spec define */
    PD_ALTERNATE_MODE_ENTER_TIME_OUT,
} pd_dpm_callback_event_t;

/**
* @brief The command used in PD_Command.
*
* The are related to PD3.0 stack's AMS.
* The command's result is return by the callback events.
*/
typedef enum _pd_command
{
    /*! invalid command */
    PD_DPM_INVALID = 0,
    /*! power negotation, it is only used in source when source power change */
    PD_DPM_CONTROL_POWER_NEGOTIATION = 1,
    /*! RDO request, it is only used in sink */
    PD_DPM_CONTROL_REQUEST = 2,
    /*! goto min request, it is only used in source */
    PD_DPM_CONTROL_GOTO_MIN = 3,
    /*! application can send soft_reset actively */
    PD_DPM_CONTROL_SOFT_RESET = 4,
    /*! application can send hard_reset actively */
    PD_DPM_CONTROL_HARD_RESET = 5,
    /*! power role swap */
    PD_DPM_CONTROL_PR_SWAP = 6,
    /*! data role swap */
    PD_DPM_CONTROL_DR_SWAP = 7,
    /*! vconn role swap */
    PD_DPM_CONTROL_VCONN_SWAP = 8,
    /*! get partner source capabilities */
    PD_DPM_CONTROL_GET_PARTNER_SOURCE_CAPABILITIES = 9,
    /*! get partner sink capabilities */
    PD_DPM_CONTROL_GET_PARTNER_SINK_CAPABILITIES = 10,
    /*! get partner source extended capabilities */
    PD_DPM_GET_SRC_EXT_CAP = 11,
    /*! get partner status */
    PD_DPM_GET_STATUS = 12,
    /*! get partner battery cap */
    PD_DPM_GET_BATTERY_CAP = 13,
    /*! get partner battery status */
    PD_DPM_GET_BATTERY_STATUS = 14,
    /*! get partner manufacturer info */
    PD_DPM_GET_MANUFACTURER_INFO = 15,
#if 0
    /*! security request, the partner will reply with security response info */
    PD_DPM_SECURITY_REQUEST,
    /*! firmware update */
    PD_DPM_FIRMWARE_UPDATE_REQUEST,
#endif
    /*! fast role swap */
    PD_DPM_FAST_ROLE_SWAP = 16,
    /*! alert */
    PD_DPM_ALERT = 17,
    /*! discovery identity */
    PD_DPM_CONTROL_DISCOVERY_IDENTITY = 18,
    /*! discovery SVIDs */
    PD_DPM_CONTROL_DISCOVERY_SVIDS = 19,
    /*! discovery Modes */
    PD_DPM_CONTROL_DISCOVERY_MODES = 20,
    /*! enter mode */
    PD_DPM_CONTROL_ENTER_MODE = 21,
    /*! exit mode */
    PD_DPM_CONTROL_EXIT_MODE = 22,
    /*! attention */
    PD_DPM_CONTROL_SEND_ATTENTION = 23,
    /*! cable reset */
    PD_DPM_CONTROL_CABLE_RESET = 24,
    /*! send vendor defined structured vdm */
    PD_DPM_SEND_VENDOR_STRUCTURED_VDM = 25,
    /*! send standard/vendor unstructured vdm */
    PD_DPM_SEND_UNSTRUCTURED_VDM = 26,
    PD_DPM_CONTROL_COUNT = 27,
} pd_command_t;

/**
* @brief PD control code.
*
* PD_Control implement these control functions.
*/
typedef enum _pd_control
{
    /*! get the power role, return value is #pd_power_role_t */
    PD_CONTROL_GET_POWER_ROLE,
    /*! get data role, return value is #pd_data_role_t */
    PD_CONTROL_GET_DATA_ROLE,
    /*! get vconn role, return value is #pd_vconn_role_t */
    PD_CONTROL_GET_VCONN_ROLE,
    /*! get typec connection role, return value is #typec_port_connect_state_t  */
    PD_CONTROL_GET_TYPEC_CONNECT_STATE,
    /*! get phy low power state, return value is #pd_phy_pwr_state_t */
    PD_CONTROL_GET_PHY_LOW_POWER_STATE,
    /*! get typec current level, return value is #typec_current_val_t */
    PD_CONTROL_GET_SNK_TYPEC_CURRENT_CAP,
    /*! control phy switches for vbus power, if PHY has this function this control code is valid */
    PD_CONTROL_PHY_POWER_PIN,
    /*! discharge vbus, if PHY has this function this control code is valid */
    PD_CONTROL_DISCHARGE_VBUS,
    /*! control vconn, if PHY has this function this control code is valid */
    PD_CONTROL_VCONN,
    /*! get the typec orientation info */
    PD_CONTROL_GET_TYPEC_ORIENTATION,
    /*!  inform vbus, let the PHY knows the actual vbus voltage, voltage unit is 1mV */
    PD_CONTROL_INFORM_VBUS_VOLTAGE_RANGE,
} pd_control_t;

#if 0
/** Debug Accessory Role */
typedef enum
{
    CONFIG_DEBUG_ACCESSORY_NONE = 0,
    CONFIG_DEBUG_ACCESSORY_TS = 1,
    CONFIG_DEBUG_ACCESSORY_DTS = 2,
} UsbpdDebugAccessoryRole_t;
#endif

/**
* @brief fast role swap current in the fixed supply PDO.
*/
typedef enum _pd_fr_swap_current
{
    /*! fast role swap not supported */
    kFRSwap_NotSupported = 0,
    /*! default usb current */
    kFRSwap_CurrentDefaultUSB = 1,
    /*! 1.5A@5V */
    kFRSwap_Current15A = 2,
    /*! 3.0A@5V */
    kFRSwap_Not30A = 3,
} pd_fr_swap_current_t;

/**
* @brief pdo type
*/
typedef enum _pd_pdo_type
{
    /*! Fixed pdo */
    kPDO_Fixed = 0u,
    /*! Battery pdo */
    kPDO_Battery = 1u,
    /*! Variable pdo */
    kPDO_Variable = 2u,
} pd_pdo_type_t;

/**
* @brief vbus power change progress
*/
typedef enum _pd_vbus_power_progress
{
    /*! invalid value */
    kVbusPower_Invalid,
    /*! vbus is stable */
    kVbusPower_Stable,
    /*! vubs is changing in hard reset */
    kVbusPower_InHardReset,
    /*! vbus is changing in power role swap */
    kVbusPower_InPRSwap,
    /*! vbus is changing in fast role swap */
    kVbusPower_InFRSwap,
    /*! vbus is changing positive or negative */
    kVbusPower_ChangeInProgress,
} pd_vbus_power_progress_t;

/*! @brief pd instance handle, return by PD_InstanceInit */
typedef void *pd_handle;
/*! @brief pd phy instance handle */
typedef void *pd_phy_handle;
/* callback to application events and parameter structures */
/*! @brief pd instance callback function.
 *
 * It is one parameter of PD_InstanceInit.
 * PD stack notify application command flow and connect/disconnect state by this callback.
 */
typedef pd_status_t (*pd_stack_callback_t)(void *callbackParam, uint32_t event, void *param);

typedef struct _pd_power_port_config
{
    /*! source caps, if don't support set as NULL */
    uint32_t *sourceCaps;
    /*! sink caps, if don't support set as NULL */
    uint32_t *sinkCaps;
    /*! source caps count, if don't support set as 0 */
    uint8_t sourceCapCount;
    /*! sink caps count, if don't support set as 0 */
    uint8_t sinkCapCount;
    /*! device's typec role, for example: sink, source or DRP.
     * it's value is #typec_power_role_config_t
     */
    uint8_t typecRole;
    /*! It only is valid for source, this value set the TypeC Rp current level
     * it's value is #typec_current_val_t
     */
    uint8_t typecSrcCurrent;
    /*! (not supported yet) It only is valid for DRP, this value configure try-sink, try-source or no try function.
     * it's value is #typec_try_t
     */
    uint8_t drpTryFunction;
    /* typec_data_function_t: none, Host->DFP, Device->UFP, all-> */
    /*! It configure data role function of the device, it is DFP, UFP, DRD.
     * For USB function, DFP correspond to Host; UFP correspond to Device; DRD correspond to OTG.
     * it's value is #typec_data_function_config_t
     */
    uint8_t dataFunction;
    /*! value is 1: support vconn swap and vconn power;
     * value is 0: don't support vconn swap or vconn power.
     */
    uint8_t vconnSupported;
    uint8_t reserved1;
    /*! (For feature extension) source config */
    void *sourceConfig;
    /*! (For feature extension) sink config */
    void *sinkConfig;
    /*! (For feature extension) DRP config */
    void *drpConfig;
} pd_power_port_config_t;

/**
* @brief PD instance config
*
* used in PD_InstanceInit function
*/
typedef struct _pd_instance_config
{
    /*! The device type, for example: normal power port or cable, the value is #pd_device_type_t */
    uint8_t deviceType;
    /*! The PHY GPIO interrupt number.
     * PD stack need the interrupt number to prevent nested operation to PHY.
     */
    uint8_t phyInterruptNum;
    /*! The PHY type, the value is #pd_phy_type_t */
    uint8_t phyType;
    /*! The PHY interface (I2C, SPI etc), the value is #pd_phy_interface_t */
    uint8_t phyInterface;
    /*! for i2c: it is slave address; for spi: it is spi PCS info */
    uint32_t interfaceParam;
    /*! The type is based on the deviceType value.
     * - kDeviceType_NormalPowerPort: #pd_power_port_config_t
     * - kDeviceType_Cable: not supported yet.
     * - kDeviceType_AudioAccDevice: not supported yet.
     * - kDeviceType_DebugAccDevice: not supported yet.
     * - kDeviceType_AlternateModeProduct: not supported yet.
     */
    void *deviceConfig;
} pd_instance_config_t;

/**
* @brief fixed source PDO
*/
typedef struct _pd_source_fixed_pdo
{
    uint32_t maxCurrent : 10;              /*!< max current */
    uint32_t voltage : 10;                 /*!< voltage value, unit is 50mV */
    uint32_t peakCurrent : 2;              /*!< peak current */
    uint32_t reserved : 2;                 /*!< reserved field */
    uint32_t unchunkedSupported : 1;       /*!< unchunked is supported or not */
    uint32_t dualRoleData : 1;             /*!< dual data role */
    uint32_t usbCommunicationsCapable : 1; /*!< usb communication capable or not */
    uint32_t externalPowered : 1;          /*!< external powered */
    uint32_t usbSuspendSupported : 1;      /*!< usb suspend supported or not */
    uint32_t dualRolePower : 1;            /*!< dual power role */
    uint32_t fixedSupply : 2;              /*!< pdo type */
} pd_source_fixed_pdo_t;

/**
* @brief fixed sink PDO
*/
typedef struct _pd_sink_fixed_pdo
{
    uint32_t operateCurrent : 10;          /*!< operate current */
    uint32_t voltage : 10;                 /*!< voltage */
    uint32_t reserved : 3;                 /*!< reserved */
    uint32_t frSwapRequiredCurrent : 2;    /*!< The value is #pd_fr_swap_current_t value */
    uint32_t dualRoleData : 1;             /*!< dual data role */
    uint32_t usbCommunicationsCapable : 1; /*!< usb communication capable or not */
    uint32_t externalPowered : 1;          /*!< external powered */
    uint32_t higherCapability : 1;         /*!< higher capability */
    uint32_t dualRolePower : 1;            /*!< dual power role */
    uint32_t fixedSupply : 2;              /*!< pdo type */
} pd_sink_fixed_pdo_t;

/**
* @brief variable source PDO
*/
typedef struct _pd_source_variable_pdo
{
    uint32_t maxCurrent : 10;    /*!< max current */
    uint32_t minVoltage : 10;    /*!< min voltage */
    uint32_t maxVoltage : 10;    /*!< max voltage */
    uint32_t variableSupply : 2; /*!< pdo type */
} pd_source_variable_pdo_t;

/**
* @brief variable sink PDO
*/
typedef struct _pd_sink_variable_pdo
{
    uint32_t operateCurrent : 10; /*!< operate current */
    uint32_t minVoltage : 10;     /*!< min voltage */
    uint32_t maxVoltage : 10;     /*!< max voltage */
    uint32_t variableSupply : 2;  /*!< pdo type */
} pd_sink_variable_pdo_t;

/**
* @brief battery source PDO
*/
typedef struct _pd_source_battery_pdo
{
    uint32_t maxAllowPower : 10; /*!< max power */
    uint32_t minVoltage : 10;    /*!< min voltage */
    uint32_t maxVoltage : 10;    /*!< max voltage */
    uint32_t battery : 2;        /*!< pdo type */
} pd_source_battery_pdo_t;

/**
* @brief battery sink PDO
*/
typedef struct _pd_sink_battery_pdo
{
    uint32_t operatePower : 10; /*!< operate power */
    uint32_t minVoltage : 10;   /*!< min voltage */
    uint32_t maxVoltage : 10;   /*!< max voltage */
    uint32_t battery : 2;       /*!< pdo type */
} pd_sink_battery_pdo_t;

/**
* @brief common PDO for union
*/
typedef struct _pd_pdo_common
{
    uint32_t reserved : 30; /*!< reserved */
    uint32_t pdoType : 2;   /*!< pdo type, the value is #pd_pdo_type_t */
} pd_pdo_common_t;

#if defined(__CC_ARM)
#pragma anon_unions
#endif
/**
* @brief source PDO union
*/
typedef struct _pd_source_pdo
{
    union
    {
        uint32_t PDOValue;                    /*!< union pdo 32 bits value */
        pd_pdo_common_t commonPDO;            /*!< union, common pdo */
        pd_source_fixed_pdo_t fixedPDO;       /*!< union, fixed pdo */
        pd_source_variable_pdo_t variablePDO; /*!< union, variable pdo */
        pd_source_battery_pdo_t batteryPDO;   /*!< union, battery pdo */
    };
} pd_source_pdo_t;

/**
* @brief sink PDO union
*/
typedef struct _pd_sink_pdo
{
    union
    {
        uint32_t PDOValue;                  /*!< union, pdo 32 bits value */
        pd_pdo_common_t commonPDO;          /*!< union, common pdo */
        pd_sink_fixed_pdo_t fixedPDO;       /*!< union, fixed pdo */
        pd_sink_variable_pdo_t variablePDO; /*!< union, variable pdo */
        pd_sink_battery_pdo_t batteryPDO;   /*!< union, battery pdo */
    };
} pd_sink_pdo_t;

/**
* @brief PD structured VDM header
*/
typedef struct _pd_structured_vdm_header
{
    union
    {
        /*! union structured VDM header structure */
        struct
        {
            uint32_t command : 5;     /*!< command */
            uint32_t reserved1 : 1;   /*!< reserved */
            uint32_t commandType : 2; /*!< command type */
            uint32_t objPos : 3;      /*!< object position */
            uint32_t reserved2 : 2;   /*!< reserved */
            uint32_t vdmVersion : 2;  /*!< vdm version */
            uint32_t vdmType : 1;     /*!< vdm type, structured vdm */
            uint32_t SVID : 16;       /*!< SVID value */
        } bitFields;
        /*! union 32bits value */
        uint32_t structuredVdmHeaderVal;
    };
} pd_structured_vdm_header_t;

/**
* @brief PD unstructured VDM header
*/
typedef struct _pd_unstructured_vdm_header
{
    union
    {
        /*! union unstructured vdm header structure */
        struct
        {
            uint32_t vendorUse : 15; /*!< vendor use fields */
            uint32_t vdmType : 1;    /*!< vdm type, unstructured vdm */
            uint32_t SVID : 16;      /*!< SVID */
        } bitFields;
        /*! union 32bits value */
        uint32_t unstructuredVdmHeaderVal;
    };
} pd_unstructured_vdm_header_t;

/**
* @brief PD extended message header
*/
typedef struct _pd_extended_msg_header
{
    union
    {
        /*! union extended message header structure */
        struct
        {
            uint16_t dataSize : 9;     /*!< data size */
            uint16_t reserved : 1;     /*!< reserved */
            uint16_t requestChunk : 1; /*!< request chunk */
            uint16_t chunkNumber : 4;  /*!< chunk number value */
            uint16_t chunked : 1;      /*!< chunked or not */
        } bitFields;
        /*! union 16bits value */
        uint16_t extendedMsgHeaderVal;
    };
} pd_extended_msg_header_t;

/**
* @brief PD message header
*/
typedef struct _pd_msg_header
{
    union
    {
        /*! union message header structure */
        struct
        {
            uint16_t messageType : 5;              /*!< message type */
            uint16_t portDataRole : 1;             /*!< port data role */
            uint16_t specRevision : 2;             /*!< spec revision */
            uint16_t portPowerRoleOrCablePlug : 1; /*!< port power role or cable plug */
            uint16_t messageID : 3;                /*!< message id */
            uint16_t NumOfDataObjs : 3;            /*!< number of data objects */
            uint16_t extended : 1;                 /*!< extended or not */
        } bitFields;
        /*! union 16bits value */
        uint16_t msgHeaderVal;
    };
} pd_msg_header_t;

/**
* @brief PD structured vdm command request (negotiation)
*
* It is used in PD_DPM_STRUCTURED_VDM_REQUEST event callback.
* It provide vdm message information to application,
* application need reply ACK (contain data), NAK or BUSY.
*/
typedef struct _pd_svdm_request
{
    /*! vdm data buffer address */
    uint32_t *vdoData;
    /*! vdm header */
    pd_structured_vdm_header_t vdmHeader;
    /*! vdm data length (unit is 4 bytes) */
    uint8_t vdoCount;
    /*! vdm message's sop type */
    uint8_t vdoSop;
    /*! application need return the negotiation result to PD stack, the value is #pd_command_result_t */
    uint8_t requestResultStatus;
} pd_svdm_command_request_t;

/**
* @brief PD structured vdm command result
*
* It is used in PD_DPM_STRUCTURED_VDM_SUCCESS and PD_DPM_STRUCTURED_VDM_FAIL events callback.
* It provide vdm command reply message information to application,
* it may be ACK (contain data), NAK or BUSY.
*/
typedef struct _pd_svdm_command_result
{
    /*! vdm data buffer address */
    uint32_t *vdoData;
    /*! vdm header, even structured vdm don't have reply or fail,
     *  the header still have the sent message's SVID info */
    pd_structured_vdm_header_t vdmHeader;
    /*! vdm's VID */
    uint16_t vdmSVID;
    /*! vdm data length (unit is 4 bytes) */
    uint8_t vdoCount;
    /*! vdm message's sop type */
    uint8_t vdoSop;
    /*! vdm command, the value is #pd_vdm_command_t */
    uint8_t vdmCommand;
    /*! vdm command's result: success with data or fail. The value is #pd_command_result_t */
    uint8_t vdmCommandResult;
} pd_svdm_command_result_t;

/**
* @brief PD structured vdm command parameter
*
* It is used in PD_Command for PD_DPM_CONTROL_DISCOVERY_IDENTITY,
* PD_DPM_CONTROL_DISCOVERY_SVIDS, PD_DPM_CONTROL_DISCOVERY_MODES,
* PD_DPM_CONTROL_ENTER_MODE, PD_DPM_CONTROL_EXIT_MODE or PD_DPM_CONTROL_SEND_ATTENTION.
* it provide vdm command information to PD stack,
* PD stack will start the command with the information.
*/
typedef struct _pd_svdm_param
{
    /*! vdm data buffer address */
    uint32_t *vdoData;
    /*! vdm header */
    pd_structured_vdm_header_t vdmHeader;
    /*! vdm data length (unit is 4 bytes) */
    uint8_t vdoCount;
    /*! vdm message's sop type */
    uint8_t vdmSop;
    /*! discovery_identity need response, but attention don't need.
     * Vendor defined structured VDM may need or not.
     */
    uint8_t vendorVDMNeedResponse;
} pd_svdm_command_param_t;

/**
* @brief PD unstructured vdm command parameter
*
* It is used in PD_Command for PD_DPM_SEND_UNSTRUCTURED_VDM
* and in callback for PD_DPM_UNSTRUCTURED_VDM_RECEIVED.
* For PD_DPM_SEND_UNSTRUCTURED_VDM: it provide vdm command information to PD stack,
* PD stack will start the command with the information.
* For PD_DPM_UNSTRUCTURED_VDM_RECEIVED: it provide information for the received unstructured vdm to APP.
*/
typedef struct _pd_unstructured_vdm_param
{
    /*! unstructured vdm message data */
    uint32_t *vdmHeaderAndVDOsData;
    /*! message sop */
    uint8_t vdmSop;
    /*! message length (unit is 4 bytes) */
    uint8_t vdmHeaderAndVDOsCount;
} pd_unstructured_vdm_command_param_t;

/**
* @brief For some command with data use this structure as parameter.
*
* The callback events of src_ext_cap, status, battery_cap, battery_status,
* manufacturer_info and alert command will use this structure.
* it provide the command data.
*/
typedef struct _pd_command_data_param
{
    /*! data length, the max length is 260 (extended msg) */
    uint32_t dataLength;
    /*! data buffer */
    uint8_t *dataBuffer;
    /*! message sop */
    uint8_t sop;
    /*! command's result: success with data or fail. The value is #pd_command_result_t */
    uint8_t resultStatus;
} pd_command_data_param_t;

/**
* @brief PD request data object.
*
* It provide the request power information.
*/
typedef struct _pd_rdo
{
    union
    {
        struct
        {
            /*!
             * - For fixed and variable request.
             *    - giveBack == 0: it is max operate current.
             *    - giveBack == 1: it is min operate current.
             * - For battery request.
             *    - giveBack == 0: it is max operate power.
             *    - giveBack == 1: it is min operate power.
             */
            uint32_t maxOrMinOperateValue : 10;
            /*! For fixed and variable request: it is current value;
             * For battery request: it is power value.
             */
            uint32_t operateValue : 10;
            /*! reserved */
            uint32_t reserved : 3;
            /*! unchunked supported or not */
            uint32_t unchunkedSupported : 1;
            /*! usb suspend supported or not */
            uint32_t noUsbSuspend : 1;
            /*! usb communications capable */
            uint32_t usbCommunicationsCapable : 1;
            /*! capability mismatch */
            uint32_t capabilityMismatch : 1;
            /*! give back flag */
            uint32_t giveBack : 1;
            /*! object position */
            uint32_t objectPosition : 3;
            /*! reserved */
            uint32_t reserved1 : 1;
        } bitFields;
        uint32_t rdoVal;
    };
} pd_rdo_t;

/**
* @brief PD capabilities info
*
* Used in PD_DPM_GET_PARTNER_SRC_CAP_SUCCESS, PD_DPM_GET_PARTNER_SNK_CAP_SUCCESS
* and PD_DPM_SNK_RECEIVE_PARTNER_SRC_CAP callback.
*/
typedef struct _pd_capabilities
{
    /*! capabilities buffer address */
    uint32_t *capabilities;
    /*! capabilities count (unit is 4 bytes) */
    uint8_t capabilitiesCount;
} pd_capabilities_t;

/**
* @brief negotiate power rdo request
*
* used in PD_DPM_SRC_RDO_REQUEST callback.
*/
typedef struct _pd_negotiate_power_request
{
    /*! request rdo */
    pd_rdo_t rdo;
    /*! capabilities result, the value is #pd_command_result_t */
    uint8_t negotiateResult;
} pd_negotiate_power_request_t;

/**
* @brief BIST data object
*/
typedef struct _pd_bist_object
{
    union
    {
        struct
        {
            uint32_t reserved1 : 28;
            /*! test mode */
            uint32_t testMode : 4;
        } bitFields;
        uint32_t objVal;
    };
} pd_bist_object_t;

#if 0
typedef struct _pd_ptn5100_ctrl_fet
{
    uint8_t usbsrcOpen : 1;
    uint8_t usbfet1Open : 1;
    uint8_t usbfet2Open : 1;
} pd_ptn5100_ctrl_fet_t;
#endif

/**
* @brief control PTN5110 pin for power control
*/
typedef struct _pd_ptn5110_ctrl_pin
{
    /* control EN_SRC pin. 0 - output 0; 1 - output 1. */
    uint8_t enSRC : 1;
    /* control EN_SNK1 pin. 0 - output 0; 1 - output 1. */
    uint8_t enSNK1 : 1;
} pd_ptn5110_ctrl_pin_t;

/**
* @brief power control interface.
*
* Application need implement this interface and pass to PD_InstanceInit
*/
typedef struct _pd_power_callback
{
    /*!
     * @brief source provide default typec vbus power.
     *
     * @param callbackParam
     * @param powerProgress hard_reset, pr_swap or fr_swap.
     *                      Normally it is useless, just in case application need this info to provide power.
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_SrcTurnOnTypeCVbus)(void *callbackParam, uint8_t powerProgress);
    /*!
     * @brief source provide rdo request vbus power.
     *
     * @param callbackParam
     * @param rdo       request rdo from partner
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_SrcTurnOnRequestVbus)(void *callbackParam, pd_rdo_t rdo);
    /*!
     * @brief source turn off vbus power.
     *
     * @param callbackParam
     * @param powerProgress hard_reset, pr_swap or fr_swap.
     *                      Normally it is useless, just in case application need this info to provide power.
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_SrcTurnOffVbus)(void *callbackParam, uint8_t powerProgress);
    /*!
     * @brief source reduce power for goto min.
     *
     * @param callbackParam
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_SrcGotoMinReducePower)(void *callbackParam);
    /*!
     * @brief sink can draw the default type-c vbus power.
     *
     * @param callbackParam
     * @param typecCurrentLevel the value is #typec_current_val_t
     * @param powerProgress hard_reset, pr_swap or fr_swap.
     *                      Normally it is useless, just in case application need this info to provide power.
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_SnkDrawTypeCVbus)(void *callbackParam, uint8_t typecCurrentLevel, uint8_t powerProgress);
    /*!
     * @brief sink can draw the request rdo vbus power.
     *
     * @param callbackParam
     * @param rdo         request rdo from partner
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_SnkDrawRequestVbus)(void *callbackParam, pd_rdo_t rdo);
    /*!
     * @brief sink stop draw vbus power.
     *
     * @param callbackParam
     * @param powerProgress hard_reset, pr_swap or fr_swap.
     *                      Normally it is useless, just in case application need this info to provide power.
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_SnkStopDrawVbus)(void *callbackParam, uint8_t powerProgress);
    /*!
     * @brief sink reduce power for goto min.
     *
     * @param callbackParam
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_SnkGotoMinReducePower)(void *callbackParam);
    /*!
     * @brief control vconn
     *
     * @param callbackParam
     * @param on      0 - turn off vconn; 1 - turn on vconn.
     *
     * @retval kStatus_PD_Success              success
     * @retval kStatus_PD_Error                error
     */
    pd_status_t (*PD_ControlVconn)(void *callbackParam, uint8_t on);
} pd_power_handle_callback_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Initialize PD stack instance.
 *
 * This function initializes the PD stack module specified by the parameter,
   it will associate with the PD PHY that is specified by parameter.
 *
 * @param[out] pdHandle    Returns the PD stack instance handle, other API will use this handle as parameter;
 * @param[in] callbackFn  PD stack callback function, it will notify the connect/disconnect and PD command's process
 flow and result;
 * @param[in] callbackParam the callbackFn's parameter.
 * @param[in] config     The PD instance configuration table, see the struct #pd_instance_config_t
 *
 * @retval kStatus_USB_Success    initialization success.
 * @retval other value            error code.
 */
pd_status_t PD_InstanceInit(pd_handle *pdHandle,
                            pd_stack_callback_t callbackFn,
                            pd_power_handle_callback_t *callbackFunctions,
                            void *callbackParam,
                            pd_instance_config_t *config);

/*!
 * @brief de-initialize PD stack instance.
 *
 * This function de-initializes the PD stack module specified by the pdHandle.
 *
 * @param[in] pdHandle   the pdHandle that is got through PD_InstanceInit.
 *
 * @retval kStatus_USB_Success    success.
 * @retval other value            error code.
 */
pd_status_t PD_InstanceDeinit(pd_handle pdHandle);

/*!
 * @brief start up PD command.
 *
 * This function trigger the AMS command functions that are defined in the Section 8.3.2 in PD3.0 spec,
 * you can see the AMS summary in the Table 8-4.
 * This function only start the command, the command need communicate the partner, so the command flow is
 * asynchronous.
 * The command process flow and result are notified by the callback function that is one parameter in the
 * PD_InstanceInit API.
 *
 * @param[in] pdHandle  the pdHandle that is got through PD_InstanceInit.
 * @param[in] command   the AMS command enumeration, see the #pd_command_t.
 * @param[in] param     the command's parameter.
 *
 * @retval kStatus_USB_Success    command start success.
 * @retval other value            error code.
 */
pd_status_t PD_Command(pd_handle pdHandle, uint32_t command, void *param);

/*!
 * @brief Control PD stack and get info from PD stack.
 *
 * This function is blocking and synchronous, it implement some controls that will used in the PD application.
 *
 * @param[in] pdHandle      the pdHandle that is got through PD_InstanceInit.
 * @param[in] controlCode   see the #pd_control_t
 * @param[in] param         the param is different for different control.
 *
 * @retval kStatus_USB_Success    function success.
 * @retval other value            error code.
 */
pd_status_t PD_Control(pd_handle pdHandle, uint32_t controlCode, void *param);

/*!
 * @brief PD stack's instance task
 *
 * User need keep calling this function endlessly in the PD application using one task.
 *
 * @param[in] pdHandle      the pdHandle that is got through PD_InstanceInit.
 */
void PD_InstanceTask(pd_handle pdHandle);

/*!
 * @brief PD PTN5110 PHY ISR function
 *
 * User need call this function in the PHY GPIO ISR.
 *
 * @param[in] pdHandle      the pdHandle that is got through PD_InstanceInit.
 */
void PD_PTN5110IsrFunction(pd_handle pdHandle);
#if 0
void PD_PTN5100IsrFunction(pd_handle pdHandle);
#endif

/*!
 * @brief PD stack timer function
 *
 * User need call this function in the 1ms Timer ISR.
 *
 * @param[in] pdHandle      the pdHandle that is got through PD_InstanceInit.
 */
void PD_TimerIsrFunction(pd_handle pdHandle);

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif
