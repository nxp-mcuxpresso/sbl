/*
 * The Clear BSD License
 * Copyright 2016 - 2017 NXP
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

#include "usb_pd_config.h"
#include "usb_pd.h"
#include "string.h"
#include "pd_app.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "board.h"
#include "pd_power_interface.h"
#include "pd_command_interface.h"

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

pd_status_t PD_DpmAppCommandCallback(void *callbackParam, uint32_t event, void *param)
{
    pd_status_t status = kStatus_PD_Error;

    switch (event)
    {
        /* hard reset */
        case PD_DPM_SNK_HARD_RESET_REQUEST:
        case PD_DPM_SRC_HARD_RESET_REQUEST:
            status = PD_DpmHardResetCallback(callbackParam);
            break;

        /* rdo request and power negotiation */
        case PD_DPM_SRC_RDO_REQUEST:
            status = PD_DpmSrcRDORequestCallback(callbackParam, ((pd_negotiate_power_request_t *)param)->rdo,
                                                 &(((pd_negotiate_power_request_t *)param)->negotiateResult));
            break;
        case PD_DPM_SRC_CONTRACT_STILL_VALID:
            status = PD_DpmSrcPreContractStillValidCallback(callbackParam, (uint8_t *)param);
            break;
        case PD_DPM_SRC_SEND_SRC_CAP_FAIL:
            status = PD_DpmSrcRDOResultCallback(callbackParam, 0, kCommandResult_Error);
            break;
        case PD_DPM_SRC_RDO_SUCCESS:
            status = PD_DpmSrcRDOResultCallback(callbackParam, 1, 0);
            break;
        case PD_DPM_SRC_RDO_FAIL:
            status = PD_DpmSrcRDOResultCallback(callbackParam, 0, *((uint8_t *)param));
            break;

        /* goto min */
        case PD_DPM_SRC_GOTOMIN_FAIL:
            status = PD_DpmSrcGotoMinResultCallback(callbackParam, 0, *((uint8_t *)param));
            break;
        case PD_DPM_SRC_GOTOMIN_SUCCESS:
            status = PD_DpmSrcGotoMinResultCallback(callbackParam, 1, 0);
            break;

        /* get sink cap */
        case PD_DPM_GET_PARTNER_SNK_CAP_SUCCESS:
            status = PD_DpmReceivePartnerSnkCapsCallback(callbackParam, (pd_capabilities_t *)param);
            break;
        case PD_DPM_GET_PARTNER_SNK_CAP_FAIL:
            status = PD_DpmGetPartnerSnkCapsFailCallback(callbackParam, *((uint8_t *)param));
            break;

        /* soft reset */
        case PD_DPM_SOFT_RESET_SUCCESS:
        case PD_DPM_SOFT_RESET_REQUEST:
            status = PD_DpmSoftResetCallback(callbackParam);
            break;
        case PD_DPM_SOFT_RESET_FAIL:
            /* in normal situation, app don't need process it PD stack will do hard_reset if soft_reset fail. */
            break;

        default:
            break;
    }
    return status;
}
