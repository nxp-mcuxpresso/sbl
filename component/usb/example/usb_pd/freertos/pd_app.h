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

#ifndef __DPM_EXTERNAL_H__
#define __DPM_EXTERNAL_H__

#include "pd_power_interface.h"
#include "pd_board_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define PD_CONFIG_APP_MODE (0x00000001u)
#define PD_CONFIG_PID (0x0100)
#define PD_VENDOR_VID (0x1FC9u)
#define PD_CONFIG_XID (0x0000u)
#define PD_CONFIG_FW_VER (0x01)
#define PD_CONFIG_HW_VER (0x01)
#define PD_CONFIG_BCD_DEVICE (0x0100)

#define PD_DEMO_INPUT_EVENT_BIT (0x01u)
#define PD_DEMO_SW_EVENT_BIT (0x02u)

typedef struct _pd_source_cap_ext_data_block
{
    uint16_t vid;
    uint16_t pid;
    uint32_t xid;
    uint8_t fwVersion;
    uint8_t hwVersion;
    uint8_t voltageRegulation;
    uint8_t holdupTime;
    uint8_t compliance;
    uint8_t touchCurrent;
    uint16_t peakCurrent1;
    uint16_t peakCurrent2;
    uint16_t peakCurrent3;
    uint8_t touchTemp;
    uint8_t sourceInputs;
    uint8_t batteries;
} pd_source_cap_ext_data_block_t;

typedef struct _pd_status_data_block
{
    uint8_t internalTemp;
    uint8_t presentInput;
    uint8_t presentBatteryInput;
} pd_status_data_block_t;

#if defined(__CC_ARM)
#pragma anon_unions
#endif
typedef struct _pd_alert_data_object
{
    union
    {
        uint32_t alertValue;
        struct
        {
            uint32_t reserved : 16;
            uint32_t hostSwappableBatteries : 4;
            uint32_t fixedBatteries : 4;
            uint32_t typeOfAlert : 8;
        } bitFields;
    };
} pd_alert_data_object_t;

typedef struct _pd_battery_cap_data_block
{
    uint16_t vid;
    uint16_t pid;
    uint16_t batteryDesignCap;
    uint16_t batteryLastFullChargeCap;
    uint8_t batteryType;
} pd_battery_cap_data_block_t;

typedef struct _pd_battery_status_data_object
{
    uint8_t reserved;
    uint8_t batterInfo;
    uint16_t batteryPC;
} pd_battery_status_data_object_t;

typedef struct _pd_manufac_info_data_block
{
    uint16_t vid;
    uint16_t pid;
    uint8_t manufacturerString[22];
} pd_manufac_info_data_block_t;

typedef struct _pd_id_heaer_vdo
{
    union
    {
        uint32_t vdoValue;
        struct
        {
            uint32_t usbVendorID : 16;
            uint32_t reserved : 7;
            uint32_t productTypeDFP : 3;
            uint32_t modalOperateSupport : 1;
            uint32_t productTypeUFPOrCablePlug : 3;
            uint32_t usbCommunicationCapableAsDevice : 1;
            uint32_t usbCommunicationCapableAsHost : 1;
        } bitFields;
    };
} pd_id_header_vdo_t;

typedef struct _pd_vdm_identity_data
{
    pd_id_header_vdo_t idHeaderVDO;
    uint32_t certStatVDO;
    uint16_t bcdDevice;
    uint16_t pid;
} pd_vdm_identity_data_t;

typedef enum _pd_demo_sw_state
{
    kDEMO_SWIdle = 0,
    kDEMO_SWIsrTrigger,
    kDEMO_SWPending,
    kDEMO_SWShortPress,
    kDEMO_SWLongPress,
    kDEMO_SWProcessed,
} pd_demo_sw_state_t;

typedef struct _pd_app
{
    pd_handle pdHandle;
    pd_power_control_instance_t powerControlInstance;
    pd_instance_config_t *pdConfigParam;
    pd_vdm_identity_data_t selfVdmIdentity;
    uint32_t selfVdmSVIDs;
    uint32_t selfVdmModes;
    pd_rdo_t sinkRequestRDO; /* sink - the self requested RDO; source - the partner sink requested RDO */
    pd_svdm_command_param_t structuredVDMCommandParam;
    pd_structured_vdm_header_t defaultSVDMCommandHeader;
    pd_unstructured_vdm_header_t unstructuredVDMCommandHeader;
    pd_source_cap_ext_data_block_t selfExtCap;
    pd_status_data_block_t selfStatus;
    pd_alert_data_object_t selfAlert;
    pd_battery_cap_data_block_t selfBatteryCap;
    pd_battery_status_data_object_t selfBatteryStatus;
    pd_manufac_info_data_block_t selfManufacInfo;
    uint32_t sourceVbusVoltage;
    uint32_t sinkRequestVoltage;
    uint8_t commonData[8];
    uint16_t partnerSVIDs[8];
    uint16_t partnerModes[8];
    pd_source_pdo_t partnerSourceCaps[7];
    pd_source_pdo_t partnerSinkCaps[7];
    uint8_t partnerSourceCapNumber; /* partner */
    uint8_t partnerSinkCapNumber;   /* partner */
    uint8_t getBatteryCapDataBlock;
    uint8_t msgSop;
    uint8_t reqestResponse;
    uint8_t appPowerRole;
    uint8_t appDataRole;
    uint8_t connectionRole;
    volatile uint8_t selfHasEnterAlernateMode;
    uint8_t portNumber;

    /* application can maintain these values for command evaluation */
    volatile uint32_t prSwapAccept : 1;    /* pr_swap and fr_swap */
    volatile uint32_t drSwapAccept : 1;    /* dr_swap */
    volatile uint32_t vconnSwapAccept : 1; /* vconn_swap */
} pd_app_t;

typedef struct _pd_demo_global
{
    volatile uint32_t consoleInputChar;
    EventGroupHandle_t demoEvent;
    volatile uint16_t powerRequestSWTime;
    volatile uint16_t powerChangeSWTime;
    volatile uint8_t powerRequestSWState;
    volatile uint8_t powerChangeSWState;
    volatile uint8_t processPort;
} pd_demo_global_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if (defined PD_DEMO_PORT1_ENABLE) && (PD_DEMO_PORT1_ENABLE > 0)
extern pd_app_t g_PDAppInstancePort1;
#endif
#if (defined PD_DEMO_PORT2_ENABLE) && (PD_DEMO_PORT2_ENABLE > 0)
extern pd_app_t g_PDAppInstancePort2;
#endif
#if (defined PD_DEMO_PORT3_ENABLE) && (PD_DEMO_PORT3_ENABLE > 0)
extern pd_app_t g_PDAppInstancePort3;
#endif
#if (defined PD_DEMO_PORT4_ENABLE) && (PD_DEMO_PORT4_ENABLE > 0)
extern pd_app_t g_PDAppInstancePort4;
#endif
extern pd_demo_global_t g_DemoGlobal;
extern pd_app_t *g_PDAppInstanceArray[];

#endif
