/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#include "usb_otg_config.h"
#include "usb_otg.h"
#include "usb_otg_oci.h"
#include "fsl_device_registers.h"
#include "usb_otg_khci.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void _USB_OtgGetControllerInterface(uint8_t controllerId,
                                           const usb_otg_controller_interface_t **controllerTable);
static void _USB_OtgStartTimer(usb_otg_instance_t *otgInstance, uint32_t timeValue);
static void _USB_OtgCancelTimer(usb_otg_instance_t *otgInstance);
static void _USB_OtgEnterStateStart(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateStart(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue);
static void _USB_OtgEnterStateAIdle(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateAIdle(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue);
static void _USB_OtgEnterStateAWaitVrise(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateAWaitVrise(usb_otg_instance_t *otgInstance,
                                           uint32_t otgChangeType,
                                           uint32_t changeValue);
static void _USB_OtgEnterStateAWaitVfall(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateAWaitVfall(usb_otg_instance_t *otgInstance,
                                           uint32_t otgChangeType,
                                           uint32_t changeValue);
static void _USB_OtgEnterStateAWaitBcon(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateAWaitBcon(usb_otg_instance_t *otgInstance,
                                          uint32_t otgChangeType,
                                          uint32_t changeValue);
static void _USB_OtgEnterStateAHost(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateAHost(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue);
static void _USB_OtgEnterStateASuspend(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateASuspend(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue);
static void _USB_OtgEnterStateAPeripheral(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateAPeripheral(usb_otg_instance_t *otgInstance,
                                            uint32_t otgChangeType,
                                            uint32_t changeValue);
static void _USB_OtgEnterStateAVbusErr(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateAVbusErr(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue);
static void _USB_OtgEnterStateBIdle(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateBIdle(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue);
static void _USB_OtgEnterStateBSrpInit(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateBSrpInit(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue);
static void _USB_OtgEnterStateBPeripheral(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateBPeripheral(usb_otg_instance_t *otgInstance,
                                            uint32_t otgChangeType,
                                            uint32_t changeValue);
static void _USB_OtgEnterStateBWaitAcon(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateBWaitAcon(usb_otg_instance_t *otgInstance,
                                          uint32_t otgChangeType,
                                          uint32_t changeValue);
static void _USB_OtgEnterStateBHost(usb_otg_instance_t *otgInstance);
static void _USB_OtgProcessStateBHost(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue);

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if (USB_OTG_CONFIG_KHCI)
static const usb_otg_controller_interface_t s_KhciInterface = {USB_OtgKhciInit, USB_OtgKhciDeinit, USB_OtgKhciControl};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

static void _USB_OtgGetControllerInterface(uint8_t controllerId, const usb_otg_controller_interface_t **controllerTable)
{
#if (USB_OTG_CONFIG_KHCI)
    if (controllerId == kUSB_ControllerKhci0)
    {
        *controllerTable = &s_KhciInterface;
    }
#endif
}

static void _USB_OtgStartTimer(usb_otg_instance_t *otgInstance, uint32_t timeValue)
{
#if (USB_OTG_CONFIG_KHCI)
    otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlSetTimer, timeValue,
                                                        0);
#endif
}

/* todo: cancel timer in the otg stack level, the timer cannot be canceled timely */
static void _USB_OtgCancelTimer(usb_otg_instance_t *otgInstance)
{
#if (USB_OTG_CONFIG_KHCI)
    otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlCancelTimer, 0, 0);
#endif
    if (otgInstance->hasTimeOutMsg > 0)
    {
        otgInstance->cancelTime = 1;
    }
}

static void _USB_OtgEnterStateStart(usb_otg_instance_t *otgInstance)
{
    otgInstance->otgDeviceState = kOtg_State_Start;
    otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                        kOtg_StatusId, kOtg_State_Start);

    USB_OtgNotifyChange(otgInstance, kOtg_StatusPowerUp, 1);
}

static void _USB_OtgProcessStateStart(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    if (otgInstance->otgControllerStatus & kOtg_StatusBusDrop) /* a_bus_drop/ */
    {
        return;
    }

    switch (otgChangeType)
    {
        case kOtg_StatusId:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusId;

                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            else
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusId);

                _USB_OtgEnterStateAIdle(otgInstance); /* go to a_idle */
            }
            break;

        case kOtg_StatusPowerUp:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusPowerUp;
            }
            break;

        default:
            break;
    }
}

static void _USB_OtgEnterStateAIdle(usb_otg_instance_t *otgInstance)
{
    if (otgInstance->otgControllerStatus & kOtg_StatusId)
    {
        _USB_OtgEnterStateBIdle(otgInstance);
    }
    else if ((!(otgInstance->otgControllerStatus & kOtg_StatusBusDrop)) &&
             (otgInstance->otgControllerStatus & kOtg_StatusPowerUp))
    {
        _USB_OtgEnterStateAWaitVrise(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_AIdle;
        otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBConn); /* clear b_conn */

#if ((defined USB_OTG_ADP_ENABLE) && (USB_OTG_ADP_ENABLE))
/* todo: adp */
#endif

        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusId | kOtg_StatusSrpDet | kOtg_StatusAdpChange,
                                                            kOtg_State_AIdle);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_AIdle);

        if (otgInstance->idChangeAsFalse)
        {
            otgInstance->idChangeAsFalse = 0U;
            /* a change in id from TRUE to FALSE causes a_bus_req to be asserted unless the device is ADP capable. */
            USB_OtgNotifyChange(otgInstance, kOtg_StatusBusReq, 1);
        }
    }
}

static void _USB_OtgProcessStateAIdle(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusBusReq:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusReq;

                if (!(otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
                {
                    _USB_OtgEnterStateAWaitVrise(otgInstance); /* go to a_wait_vrise */
                }
            }
            else
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBusReq);
            }
            break;

#if (USB_OTG_SRP_ENABLE)
        case kOtg_StatusSrpDet:
            if (changeValue)
            {
                /* todo: update controller state */

                if (!(otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
                {
                    _USB_OtgEnterStateAWaitVrise(otgInstance); /* go to a_wait_vrise */
                }
            }
            break;
#endif

#if ((defined USB_OTG_ADP_ENABLE) && (USB_OTG_ADP_ENABLE))
        case kOtg_StatusAdpChange:
            if (changeValue)
            {
                /* todo: update controller state */

                if (!(otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
                {
                    _USB_OtgEnterStateAWaitVrise(otgInstance); /* go to a_wait_vrise */
                }
            }
            break;
#endif

        case kOtg_StatusPowerUp:
            /* when power up, the kOtg_StatusBusDrop must be false */
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusPowerUp);

                if (!(otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
                {
                    _USB_OtgEnterStateAWaitVrise(otgInstance); /* go to a_wait_vrise */
                }
            }
            break;

        case kOtg_StatusId:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusId;

                _USB_OtgEnterStateBHost(otgInstance); /* go to b_host */
            }
            break;

        case kOtg_StatusBusDrop:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusDrop;
            }
            else
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBusDrop);

                if ((otgInstance->otgControllerStatus & kOtg_StatusBusReq) ||
                    (otgInstance->otgControllerStatus & kOtg_StatusSrpDet) ||
                    (otgInstance->otgControllerStatus & kOtg_StatusAdpChange))
                {
                    _USB_OtgEnterStateAWaitVrise(otgInstance); /* go to a_wait_vrise */
                }
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateAWaitVrise(usb_otg_instance_t *otgInstance)
{
    if ((otgInstance->otgControllerStatus & kOtg_StatusId) || (otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
    {
        _USB_OtgEnterStateAWaitVfall(otgInstance);
    }
    else if (otgInstance->otgControllerStatus & kOtg_StatusVbusVld)
    {
        _USB_OtgEnterStateAWaitBcon(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_AWaitVrise;

        /* driver vbus */
        if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
        {
            otgInstance->otgControllerStatus |= kOtg_StatusVbusVld;
            otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlVbus, 1, 0);
        }
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusId | kOtg_StatusVbusVld, kOtg_State_AWaitVrise);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_AWaitVrise);

        /* set timer */
        _USB_OtgStartTimer(otgInstance, USB_OTG_TIMER_A_WAIT_VRISE_TMR);
    }
}

static void _USB_OtgProcessStateAWaitVrise(usb_otg_instance_t *otgInstance,
                                           uint32_t otgChangeType,
                                           uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusBusDrop:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusDrop;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusId:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusId;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusTimeOut:
            /* todo: update controller state */

            if (otgInstance->otgControllerStatus & kOtg_StatusVbusVld)
            {
                _USB_OtgEnterStateAWaitBcon(otgInstance); /* go to a_wait_bcon */
            }
            else
            {
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusVbusVld:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusVbusVld;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateAWaitBcon(otgInstance); /* go to a_wait_bcon */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateAWaitVfall(usb_otg_instance_t *otgInstance)
{
    otgInstance->otgDeviceState = kOtg_State_AWaitVfall;

    USB_OtgKhciControl(otgInstance->controllerHandle, kOtg_ControlPullUp, 0, 0);

    /* if there is bus req, the bus req fail. Or there is bus drop. */
    otgInstance->otgControllerStatus &= (~kOtg_StatusBusReq);

    /* close vbus */
    if (otgInstance->otgControllerStatus & kOtg_StatusVbusVld)
    {
        otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusVbusVld);
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlVbus, 0, 0);
    }
    otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus, 0,
                                                        kOtg_State_AWaitVfall);

    otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_AWaitVfall);

    /* set timer */
    _USB_OtgStartTimer(otgInstance, USB_OTG_TIMER_A_WAIT_VFALL_TMR);
}

static void _USB_OtgProcessStateAWaitVfall(usb_otg_instance_t *otgInstance,
                                           uint32_t otgChangeType,
                                           uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusTimeOut:
            /* todo: update controller state */

            _USB_OtgEnterStateAIdle(otgInstance); /* go to a_idle */
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateAWaitBcon(usb_otg_instance_t *otgInstance)
{
    if ((otgInstance->otgControllerStatus & kOtg_StatusId) || (otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
    {
        _USB_OtgEnterStateAWaitVfall(otgInstance);
    }
    else if (otgInstance->otgControllerStatus & kOtg_StatusBConn)
    {
        _USB_OtgEnterStateAHost(otgInstance);
    }
    else if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
    {
        _USB_OtgEnterStateAVbusErr(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_AWaitBcon;

        /* driver vbus */
        if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
        {
            otgInstance->otgControllerStatus |= kOtg_StatusVbusVld;
            otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlVbus, 1, 0);
        }
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusId | kOtg_StatusVbusInvld | kOtg_StatusBConn,
                                                            kOtg_State_AWaitBcon);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_AWaitBcon);

/* set timer */
#if (USB_OTG_TIMER_A_WAIT_BCON_TMR != 0xFFFFFFFFU)
        _USB_OtgStartTimer(otgInstance, USB_OTG_TIMER_A_WAIT_BCON_TMR);
#endif
    }
}

static void _USB_OtgProcessStateAWaitBcon(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusTimeOut:
            /* todo: update controller state */

            _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            break;

        case kOtg_StatusId:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusId;

                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusBusDrop:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusDrop;

                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusBConn:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBConn;

                _USB_OtgEnterStateAHost(otgInstance); /* go to a_host */
            }
            break;

        case kOtg_StatusVbusVld:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= ~kOtg_StatusVbusVld;

                _USB_OtgEnterStateAVbusErr(otgInstance); /* go to a_vbus_err */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateAHost(usb_otg_instance_t *otgInstance)
{
    if ((otgInstance->otgControllerStatus & kOtg_StatusId) || (otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
    {
        _USB_OtgEnterStateAWaitVfall(otgInstance);
    }
    else if (!(otgInstance->otgControllerStatus & kOtg_StatusBConn))
    {
        _USB_OtgEnterStateAWaitBcon(otgInstance);
    }
    /*
    else if (!(otgInstance->otgControllerStatus & kOtg_StatusBusReq))
    {
        _USB_OtgEnterStateASuspend(otgInstance);
    }
    */
    else if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
    {
        _USB_OtgEnterStateAVbusErr(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_AHost;
        otgInstance->otgControllerStatus &= (~kOtg_StatusBHNPFeature);

        /* driver vbus */
        if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
        {
            otgInstance->otgControllerStatus |= kOtg_StatusVbusVld;
            otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlVbus, 1, 0);
        }
        /* pull down dp and dm */
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullDown,
                                                            kOtg_PullDp | kOtg_PullDm, 0);
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusId | kOtg_StatusBDisconn | kOtg_StatusVbusInvld,
                                                            kOtg_State_AHost);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_AHost);

#if (USB_OTG_TIME_WAIT_DEVICE_INIT != 0U)
        otgInstance->waitInit = 1;
        _USB_OtgStartTimer(otgInstance, USB_OTG_TIME_WAIT_DEVICE_INIT); /* wait the B-Device init the device stack */
#else
        /* start work as host */
        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStackInit, kOtg_StackHostInit);
#endif
    }
}

static void _USB_OtgExitHost(usb_otg_instance_t *otgInstance)
{
    otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStackInit,
                             kOtg_StackHostDeinit); /* host stack de-init */
    otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullDown, 0, 0);
}

static void _USB_OtgProcessStateAHost(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
#if (USB_OTG_TIME_WAIT_DEVICE_INIT != 0U)
        case kOtg_StatusTimeOut:
            if (otgInstance->waitInit) /* init host */
            {
                /* start work as host */
                otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStackInit,
                                         kOtg_StackHostInit); /* host stack init */
                otgInstance->waitInit = 0;
            }
            break;
#endif

        case kOtg_StatusBConn:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBConn);

                _USB_OtgExitHost(otgInstance);
                _USB_OtgEnterStateAWaitBcon(otgInstance); /* go to a_wait_bcon */
            }
            break;

        case kOtg_StatusId:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusId;

                _USB_OtgExitHost(otgInstance);
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusBusDrop:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusDrop;

                _USB_OtgExitHost(otgInstance);
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusBusReq: /* hnp or host release bus */
            if (!changeValue)   /* bus release */
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBusReq);

                _USB_OtgExitHost(otgInstance);
                /* USB_HostSuspend(otgInstance->hostHandle); */
                _USB_OtgEnterStateASuspend(otgInstance); /* go to a_suspend */
            }
            else /* HNP bus request */
            {
                /* todo: update controller state */
                /* otgInstance->otgControllerStatus &= (~kOtg_StatusBusReq); */

                _USB_OtgExitHost(otgInstance);
                /* USB_HostSuspend(otgInstance->hostHandle); */
                _USB_OtgEnterStateASuspend(otgInstance); /* go to a_suspend */
            }
            break;

        case kOtg_StatusVbusVld:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusVbusVld);

                _USB_OtgExitHost(otgInstance);
                _USB_OtgEnterStateAVbusErr(otgInstance); /* go to a_vbus_err */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateASuspend(usb_otg_instance_t *otgInstance)
{
    if ((otgInstance->otgControllerStatus & kOtg_StatusId) || (otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
    {
        _USB_OtgEnterStateAWaitVfall(otgInstance);
    }
    else if (!(otgInstance->otgControllerStatus & kOtg_StatusBConn))
    {
        if (otgInstance->otgControllerStatus & kOtg_StatusBHNPFeature)
        {
            _USB_OtgEnterStateAPeripheral(otgInstance);
        }
        else
        {
            _USB_OtgEnterStateAWaitBcon(otgInstance);
        }
    }
    else if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
    {
        _USB_OtgEnterStateAVbusErr(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_ASuspend;

        /* driver vbus */
        if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
        {
            otgInstance->otgControllerStatus |= kOtg_StatusVbusVld;
            otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlVbus, 1, 0);
        }
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusId | kOtg_StatusBDisconn | kOtg_StatusVbusInvld,
                                                            kOtg_State_ASuspend);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_ASuspend);

        if (otgInstance->otgControllerStatus & kOtg_StatusBHNPFeature)
        {
            /* set timer */
            _USB_OtgStartTimer(otgInstance, USB_OTG_TIMER_A_AIDL_BDIS_TMR);
        }
    }
}

static void _USB_OtgProcessStateASuspend(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusBConn:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBConn);
                _USB_OtgCancelTimer(otgInstance);
                if (!(otgInstance->otgControllerStatus & kOtg_StatusBHNPFeature))
                {
                    _USB_OtgEnterStateAWaitBcon(otgInstance); /* go to a_wait_bcon */
                }
                else
                {
                    _USB_OtgEnterStateAPeripheral(otgInstance); /* go to a_peripheral */
                }
            }
            break;

        case kOtg_StatusTimeOut:
            if (otgInstance->otgControllerStatus & kOtg_StatusBHNPFeature)
            {
                if (changeValue)
                {
                    /* todo: update controller state */

                    _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
                }
            }
            break;

        case kOtg_StatusBusReq: /* resume */
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusReq;

                _USB_OtgCancelTimer(otgInstance);
                otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlResume,
                                                                    20, 0);

                _USB_OtgEnterStateAHost(otgInstance); /* go to a_host */
            }
            break;

        case kOtg_StatusId:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusId;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusBusDrop:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusDrop;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusVbusVld:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusVbusVld);

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateAVbusErr(otgInstance); /* go to a_vbus_err */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateAPeripheral(usb_otg_instance_t *otgInstance)
{
    if ((otgInstance->otgControllerStatus & kOtg_StatusId) || (otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
    {
        _USB_OtgEnterStateAWaitVfall(otgInstance);
    }
    else if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
    {
        _USB_OtgEnterStateAVbusErr(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_APeripheral;
        otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBConn);

        /* start work as device */
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullUp,
                                                            kOtg_PullDp, 0);
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullDown,
                                                            kOtg_PullDm, 0);
        /* driver vbus */
        if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
        {
            otgInstance->otgControllerStatus |= kOtg_StatusVbusVld;
            otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlVbus, 1, 0);
        }
        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStackInit,
                                 kOtg_StackDeviceInit); /* device stack init */
#if (USB_OTG_TIME_WAIT_BHOST != 0U)
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusId | kOtg_StatusVbusInvld,
                                                            kOtg_State_APeripheral);
#else
        otgInstance->controllerInterface->controllerControl(
            otgInstance->controllerHandle, kOtg_ControlRequestStatus,
            kOtg_StatusId | kOtg_StatusVbusInvld | kOtg_StatusCheckIdleInAPeripheral, kOtg_State_APeripheral);
#endif

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_APeripheral);

#if (USB_OTG_TIME_WAIT_BHOST != 0U)
        /* wait the B-Device init the host stack */
        otgInstance->waitInit = 1;
        _USB_OtgStartTimer(otgInstance, USB_OTG_TIME_WAIT_BHOST);
#endif
    }
}

static void _USB_OtgExitDevice(usb_otg_instance_t *otgInstance)
{
    otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullUp, 0, 0);
    otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullDown, 0, 0);
    /* otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullUp, 0); */
    otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStackInit,
                             kOtg_StackDeviceDeinit); /* device stack de-init */
}

static void _USB_OtgProcessStateAPeripheral(usb_otg_instance_t *otgInstance,
                                            uint32_t otgChangeType,
                                            uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusBusReq:
            if (changeValue)
            {
                /* todo: set hnp flag for polling */
                otgInstance->otgControllerStatus |= kOtg_StatusBusReq;
            }
            break;

        case kOtg_StatusId:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusId;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgExitDevice(otgInstance);
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusBusDrop:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusDrop;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgExitDevice(otgInstance);
                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusTimeOut:
#if (USB_OTG_TIME_WAIT_BHOST != 0U)
            if (otgInstance->waitInit) /* wait init host */
            {
                otgInstance->controllerInterface->controllerControl(
                    otgInstance->controllerHandle, kOtg_ControlRequestStatus, kOtg_StatusCheckIdleInAPeripheral,
                    kOtg_State_APeripheral);
                otgInstance->waitInit = 0;
            }
            else
            {
#endif
                if (changeValue)
                {
                    /* todo: update controller state */

                    _USB_OtgExitDevice(otgInstance);
                    _USB_OtgEnterStateAWaitBcon(otgInstance); /* go to a_wait_bcon */
                }
#if (USB_OTG_TIME_WAIT_BHOST != 0U)
            }
#endif
            break;

        case kOtg_StatusVbusVld:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusVbusVld);

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgExitDevice(otgInstance);
                _USB_OtgEnterStateAVbusErr(otgInstance); /* go to a_vbus_err */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateAVbusErr(usb_otg_instance_t *otgInstance)
{
    if ((otgInstance->otgControllerStatus & kOtg_StatusId) || (otgInstance->otgControllerStatus & kOtg_StatusBusDrop))
    {
        _USB_OtgEnterStateAWaitVfall(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_AVbusErr;

        /* don't driver vbus */
        if (otgInstance->otgControllerStatus & kOtg_StatusVbusVld)
        {
            otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusVbusVld);
            otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlVbus, 0, 0);
        }
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusId, kOtg_State_AVbusErr);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_AVbusErr);
    }
}

static void _USB_OtgProcessStateAVbusErr(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusId:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusId;

                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusBusDrop:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusDrop;

                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        case kOtg_StatusClrErr:
            if (!changeValue)
            {
                /* todo: update controller state */

                _USB_OtgEnterStateAWaitVfall(otgInstance); /* go to a_wait_vfall */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateBIdle(usb_otg_instance_t *otgInstance)
{
    if (!(otgInstance->otgControllerStatus & kOtg_StatusId))
    {
        otgInstance->otgControllerStatus = 0U; /* default controller status */
        _USB_OtgEnterStateAIdle(otgInstance);
    }
    else if (otgInstance->otgControllerStatus & kOtg_StatusSessVld)
    {
        otgInstance->otgControllerStatus = kOtg_StatusId;
        _USB_OtgEnterStateBPeripheral(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_BIdle;
        otgInstance->otgControllerStatus = kOtg_StatusId;

#if ((defined USB_OTG_ADP_ENABLE) && (USB_OTG_ADP_ENABLE))
/* todo: adp */
#endif

        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullUp, 0,
                                                            0); /* disable pull-up */
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullDown, 0,
                                                            0); /* disable DP&DM pulldown */
        otgInstance->controllerInterface->controllerControl(
            otgInstance->controllerHandle, kOtg_ControlRequestStatus,
            kOtg_StatusId | kOtg_StatusAdpChange | kOtg_StatusSessVld | kOtg_StatusSe0Srp | kOtg_StatusSsendSrp,
            kOtg_State_BIdle);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_BIdle);
    }
}

static void _USB_OtgProcessStateBIdle(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusId:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus = 0U; /* default controller status */

                _USB_OtgEnterStateAIdle(otgInstance); /* go to a_idle */
            }
            break;

        case kOtg_StatusSsendSrp:
            otgInstance->otgControllerStatus |= kOtg_StatusSsendSrp;
            if (otgInstance->otgControllerStatus & kOtg_StatusSe0Srp)
            {
                if ((otgInstance->otgControllerStatus & kOtg_StatusPowerUp) ||
                    (otgInstance->otgControllerStatus & kOtg_StatusBusReq) ||
                    (otgInstance->otgControllerStatus & kOtg_StatusAdpChange))
                {
                    otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusPowerUp);
                    _USB_OtgEnterStateBSrpInit(otgInstance); /* go to b_srp_init */
                }
            }
            break;

        case kOtg_StatusSe0Srp:
            otgInstance->otgControllerStatus |= kOtg_StatusSe0Srp;
            if (otgInstance->otgControllerStatus & kOtg_StatusSsendSrp)
            {
                if ((otgInstance->otgControllerStatus & kOtg_StatusPowerUp) ||
                    (otgInstance->otgControllerStatus & kOtg_StatusBusReq) ||
                    (otgInstance->otgControllerStatus & kOtg_StatusAdpChange))
                {
                    otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusPowerUp);
                    _USB_OtgEnterStateBSrpInit(otgInstance); /* go to b_srp_init */
                }
            }
            break;

        case kOtg_StatusPowerUp:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusPowerUp;

                if ((otgInstance->otgControllerStatus & kOtg_StatusSsendSrp) &&
                    (otgInstance->otgControllerStatus & kOtg_StatusSe0Srp))
                {
                    otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusPowerUp);
                    _USB_OtgEnterStateBSrpInit(otgInstance); /* go to b_srp_init */
                }
            }
            break;

        case kOtg_StatusBusReq:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusReq;

                if ((otgInstance->otgControllerStatus & kOtg_StatusSsendSrp) &&
                    (otgInstance->otgControllerStatus & kOtg_StatusSe0Srp))
                {
                    _USB_OtgEnterStateBSrpInit(otgInstance); /* go to b_srp_init */
                }
            }
            break;

        case kOtg_StatusAdpChange:
            if (changeValue)
            {
                /* todo: update controller state */

                if ((otgInstance->otgControllerStatus & kOtg_StatusSsendSrp) &&
                    (otgInstance->otgControllerStatus & kOtg_StatusSe0Srp))
                {
                    _USB_OtgEnterStateBSrpInit(otgInstance); /* go to b_srp_init */
                }
            }
            break;

        case kOtg_StatusSessVld:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusSessVld;

                _USB_OtgEnterStateBPeripheral(otgInstance); /* go to b_peripheral */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateBSrpInit(usb_otg_instance_t *otgInstance)
{
    otgInstance->otgControllerStatus = (~kOtg_StatusBusReq); /* clear b_bus_req */

    if (!(otgInstance->otgControllerStatus & kOtg_StatusId))
    {
        _USB_OtgEnterStateBIdle(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_BSrpInit;

        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlDataPulse, 1, 0);

        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusId | kOtg_StatusBSrpDone, kOtg_State_BSrpInit);
        /* todo: check SRP fail */
        /* _USB_OtgStartTimer(otgInstance, USB_OTG_TIME_B_SRP_FAIL); */

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_BSrpInit);
    }
}

static void _USB_OtgProcessStateBSrpInit(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusId:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusId);

                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            break;

        case kOtg_StatusBSrpDone:
            if (changeValue)
            {
                /* todo: update controller state */

                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            break;

#if 0
        case kOtg_StatusTimeOut: /* b_srp_done and srp fail */
            if (changeValue)
            {
                /* todo: update controller state */

                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            break;

        case kOtg_StatusSessVld: /* this change is not align with spec state machine */
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusSessVld;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            break;
#endif

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateBPeripheral(usb_otg_instance_t *otgInstance)
{
    otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusAConn);

    if ((!(otgInstance->otgControllerStatus & kOtg_StatusId)) ||
        (!(otgInstance->otgControllerStatus & kOtg_StatusSessVld)))
    {
        _USB_OtgEnterStateBIdle(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_BPeripheral;
        otgInstance->otgControllerStatus &= (~kOtg_StatusBHNPFeature);

        /* start work as device */
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullUp,
                                                            kOtg_PullDp, 0);
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullDown,
                                                            kOtg_PullDm, 0);
        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStackInit,
                                 kOtg_StackDeviceInit); /* device stack init */

        otgInstance->controllerInterface->controllerControl(
            otgInstance->controllerHandle, kOtg_ControlRequestStatus,
            kOtg_StatusId | kOtg_StatusSessInvld | kOtg_StatusBusSuspend, kOtg_State_BPeripheral);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_BPeripheral);
    }
}

static void _USB_OtgProcessStateBPeripheral(usb_otg_instance_t *otgInstance,
                                            uint32_t otgChangeType,
                                            uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusId:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusId);

                _USB_OtgExitDevice(otgInstance);
                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            break;

        case kOtg_StatusSessVld:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusSessVld);

                _USB_OtgExitDevice(otgInstance);
                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            break;

        case kOtg_StatusVbusVld:
            if (!changeValue)
            {
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusVbusVld);
                _USB_OtgExitDevice(otgInstance);
                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            else
            {
                otgInstance->otgControllerStatus |= (uint32_t)(kOtg_StatusVbusVld);
            }
            break;

        case kOtg_StatusBusReq:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusBusReq;
            }
            break;

        case kOtg_StatusBusSuspend:
            if (changeValue)
            {
                /* todo: update controller state */

                if ((otgInstance->otgControllerStatus & kOtg_StatusBHNPFeature) &&
                    (otgInstance->otgControllerStatus & kOtg_StatusBusReq))
                {
                    otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle,
                                                                        kOtg_ControlPullUp, 0, 0);
                    _USB_OtgExitDevice(otgInstance);
                    _USB_OtgEnterStateBWaitAcon(otgInstance); /* go to b_wait_acon */
                }
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateBWaitAcon(usb_otg_instance_t *otgInstance)
{
    if ((!(otgInstance->otgControllerStatus & kOtg_StatusId)) ||
        (!(otgInstance->otgControllerStatus & kOtg_StatusSessVld)))
    {
        _USB_OtgEnterStateBIdle(otgInstance);
    }
    else if (otgInstance->otgControllerStatus & kOtg_StatusAConn)
    {
        _USB_OtgEnterStateBHost(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_BWaitAcon;

        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullUp, 0, 0);
        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusBusResume | kOtg_StatusAConn,
                                                            kOtg_State_BWaitAcon);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_BWaitAcon);

        /* set timer */
        _USB_OtgStartTimer(otgInstance, USB_OTG_TIMER_B_ASE0_BRST_TMR);
    }
}

static void _USB_OtgProcessStateBWaitAcon(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
        case kOtg_StatusAConn:
            if (changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus |= kOtg_StatusAConn;

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateBHost(otgInstance); /* go to b_host */
            }
            break;

        case kOtg_StatusId:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusId);

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            break;

        case kOtg_StatusSessVld:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusSessVld);

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateBIdle(otgInstance); /* go to b_idle */
            }
            break;

        case kOtg_StatusBusResume:
            if (changeValue)
            {
                /* todo: update controller state */

                _USB_OtgCancelTimer(otgInstance);
                _USB_OtgEnterStateBPeripheral(otgInstance); /* go to b_peripheral */
            }
            break;

        case kOtg_StatusTimeOut:
            if (changeValue)
            {
                /* todo: update controller state */

                _USB_OtgEnterStateBPeripheral(otgInstance); /* go to b_peripheral */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

static void _USB_OtgEnterStateBHost(usb_otg_instance_t *otgInstance)
{
    if ((!(otgInstance->otgControllerStatus & kOtg_StatusAConn)) ||
        (!(otgInstance->otgControllerStatus & kOtg_StatusBusReq)))
    {
        _USB_OtgEnterStateBPeripheral(otgInstance);
    }
    else if (!(otgInstance->otgControllerStatus & kOtg_StatusVbusVld))
    {
        _USB_OtgEnterStateBPeripheral(otgInstance);
    }
    else
    {
        otgInstance->otgDeviceState = kOtg_State_BHost;
        otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBusReq);
        otgInstance->otgControllerStatus |= kOtg_StatusBHNPFeature;

        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlPullDown,
                                                            kOtg_PullDp | kOtg_PullDm, 0);

        otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlRequestStatus,
                                                            kOtg_StatusADisconn, kOtg_State_BHost);

        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStateChange, kOtg_State_BHost);

#if (USB_OTG_TIME_WAIT_DEVICE_INIT != 0U)
        otgInstance->waitInit = 1;
        _USB_OtgStartTimer(otgInstance, USB_OTG_TIME_WAIT_DEVICE_INIT);
#else
        /* start work as host */
        otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStackInit, kOtg_StackHostInit);
#endif
    }
}

static void _USB_OtgProcessStateBHost(usb_otg_instance_t *otgInstance, uint32_t otgChangeType, uint32_t changeValue)
{
    switch (otgChangeType)
    {
#if (USB_OTG_TIME_WAIT_DEVICE_INIT != 0U)
        case kOtg_StatusTimeOut:
            if (otgInstance->waitInit) /* wait device init */
            {
                /* start work as host */
                otgInstance->waitInit = 0;
                otgInstance->otgCallback(otgInstance->otgCallbackParameter, kOtg_EventStackInit,
                                         kOtg_StackHostInit); /* host stack init */
            }
            break;
#endif

        case kOtg_StatusBusReq:
            if (!changeValue) /* B release bus */
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusBusReq);

                _USB_OtgExitHost(otgInstance);
                _USB_OtgEnterStateBPeripheral(otgInstance); /* go to b_peripheral */
            }
            else /* A request bus */
            {
                /* todo: update controller state */

                _USB_OtgExitHost(otgInstance);
                _USB_OtgEnterStateBPeripheral(otgInstance); /* go to b_peripheral */
            }
            break;

        case kOtg_StatusAConn:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusAConn);

                _USB_OtgExitHost(otgInstance);
                _USB_OtgEnterStateBPeripheral(otgInstance); /* go to b_peripheral */
            }
            break;

        case kOtg_StatusVbusVld:
            if (!changeValue)
            {
                /* todo: update controller state */
                otgInstance->otgControllerStatus &= (uint32_t)(~kOtg_StatusVbusVld);

                _USB_OtgExitHost(otgInstance);
                _USB_OtgEnterStateBPeripheral(otgInstance); /* go to b_peripheral */
            }
            break;

        default:
            if (changeValue)
            {
                otgInstance->otgControllerStatus |= otgChangeType;
            }
            else
            {
                otgInstance->otgControllerStatus &= (~otgChangeType);
            }
            break;
    }
}

usb_status_t USB_OtgInit(uint8_t controllerId,
                         usb_otg_handle *otgHandle,
                         usb_otg_callback_t otgCallbackFn,
                         void *callbackParameter)
{
    usb_otg_instance_t *otgInstance = NULL;

    if (otgHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    /* 1. initialize otg stack */
    otgInstance = (usb_otg_instance_t *)USB_OsaMemoryAllocate(sizeof(usb_otg_instance_t));
    if (otgInstance == NULL)
    {
        return kStatus_USB_AllocFail;
    }
    /* initialize msg queue */
    if (kStatus_USB_OSA_Success !=
        USB_OsaMsgqCreate(&otgInstance->otgMsgHandle, USB_OTG_MSG_COUNT, sizeof(usb_otg_msg_t) / 4))
    {
        USB_OsaMemoryFree(otgInstance);
        return kStatus_USB_Error;
    }
    /* otg instance structure filed initialization */
    otgInstance->otgControllerStatus = 0U; /* default controller status */
    otgInstance->otgCallback = otgCallbackFn;
    otgInstance->otgCallbackParameter = callbackParameter;
    otgInstance->hasUpdateMsg = 0U;

    /* 2. initialize controller */
    _USB_OtgGetControllerInterface(controllerId, &otgInstance->controllerInterface);
    if ((otgInstance->controllerInterface == NULL) || (otgInstance->controllerInterface->controllerInit == NULL) ||
        (otgInstance->controllerInterface->controllerDeinit == NULL) ||
        (otgInstance->controllerInterface->controllerControl == NULL))
    {
        USB_OsaMsgqDestroy(otgInstance->otgMsgHandle);
        USB_OsaMemoryFree(otgInstance);
        return kStatus_USB_Error;
    }
    if (otgInstance->controllerInterface->controllerInit(controllerId, otgInstance, &otgInstance->controllerHandle) !=
        kStatus_USB_Success)
    {
        USB_OsaMsgqDestroy(otgInstance->otgMsgHandle);
        USB_OsaMemoryFree(otgInstance);
        return kStatus_USB_Error;
    }

    _USB_OtgEnterStateStart(otgInstance);

    *otgHandle = otgInstance;
    return kStatus_USB_Success;
}

usb_status_t USB_OtgDeinit(usb_otg_handle otgHandle)
{
    usb_otg_instance_t *otgInstance = (usb_otg_instance_t *)otgHandle;

    if (otgHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    /* 1. de-initialize controller */
    otgInstance->controllerInterface->controllerDeinit(otgInstance->controllerHandle);

    /* 2. de-initialize otg stack */
    USB_OsaMsgqDestroy(otgInstance->otgMsgHandle);
    USB_OsaMemoryFree(otgInstance);

    return kStatus_USB_Success;
}

void USB_OtgTaskFunction(usb_otg_handle otgHandle)
{
    usb_otg_instance_t *otgInstance = (usb_otg_instance_t *)otgHandle;
    usb_otg_msg_t otgMsg;

    if (otgHandle == NULL)
    {
        return;
    }

    /* wait forever for one message */
    if (USB_OsaMsgqRecv(otgInstance->otgMsgHandle, &otgMsg, 0) == kStatus_USB_OSA_Success)
    {
        if (otgMsg.otgStatusType == kOtg_StatusChange)
        {
            otgInstance->hasUpdateMsg = 0;
            otgInstance->controllerInterface->controllerControl(otgInstance->controllerHandle, kOtg_ControlUpdateStatus,
                                                                0, 0);
        }
        else
        {
            if (otgMsg.otgStatusType == kOtg_StatusTimeOut)
            {
                otgInstance->hasTimeOutMsg--;
                if (otgInstance->cancelTime)
                {
                    otgInstance->cancelTime = 0;
                    return;
                }
            }

            switch (otgInstance->otgDeviceState)
            {
                case kOtg_State_Start:
                    _USB_OtgProcessStateStart(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_AIdle:
                    _USB_OtgProcessStateAIdle(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_AWaitVrise:
                    _USB_OtgProcessStateAWaitVrise(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_AWaitBcon:
                    _USB_OtgProcessStateAWaitBcon(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_AHost:
                    _USB_OtgProcessStateAHost(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_AWaitVfall:
                    _USB_OtgProcessStateAWaitVfall(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_ASuspend:
                    _USB_OtgProcessStateASuspend(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_APeripheral:
                    _USB_OtgProcessStateAPeripheral(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_AVbusErr:
                    _USB_OtgProcessStateAVbusErr(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_BIdleEh:
                    /* The device is OTG device */
                    break;

                case kOtg_State_BIdle:
                    _USB_OtgProcessStateBIdle(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_BSrpInit:
                    _USB_OtgProcessStateBSrpInit(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_BPeripheral:
                    _USB_OtgProcessStateBPeripheral(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_BWaitAcon:
                    _USB_OtgProcessStateBWaitAcon(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                case kOtg_State_BHost:
                    _USB_OtgProcessStateBHost(otgInstance, otgMsg.otgStatusType, otgMsg.otgStatusValue);
                    break;

                default:
                    break;
            }
        }
    }
}

usb_status_t USB_OtgBusDrop(usb_otg_handle otgHandle, uint8_t drop)
{
    usb_otg_instance_t *otgInstance = (usb_otg_instance_t *)otgHandle;

    if (otgHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if ((otgInstance->otgDeviceState >= kOtg_State_AIdle) && (otgInstance->otgDeviceState <= kOtg_State_AVbusErr))
    {
        return USB_OtgNotifyChange(otgHandle, kOtg_StatusBusDrop, drop);
    }
    else
    {
        return kStatus_USB_Error;
    }
}

usb_status_t USB_OtgBusRequest(usb_otg_handle otgHandle)
{
    if (otgHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    return USB_OtgNotifyChange(otgHandle, kOtg_StatusBusReq, 1);
}

usb_status_t USB_OtgBusRelease(usb_otg_handle otgHandle)
{
    if (otgHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    return USB_OtgNotifyChange(otgHandle, kOtg_StatusBusReq, 0);
}

usb_status_t USB_OtgClearError(usb_otg_handle otgHandle)
{
    usb_otg_instance_t *otgInstance = (usb_otg_instance_t *)otgHandle;

    if (otgHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (otgInstance->otgDeviceState == kOtg_State_AVbusErr)
    {
        return USB_OtgNotifyChange(otgHandle, kOtg_StatusClrErr, 1);
    }
    else
    {
        return kStatus_USB_Error;
    }
}

usb_status_t USB_OtgNotifyChange(usb_otg_handle otgHandle, uint32_t statusType, uint32_t statusValue)
{
    usb_otg_msg_t otgMsg;
    usb_otg_instance_t *otgInstance = (usb_otg_instance_t *)otgHandle;

    if (otgHandle == NULL)
    {
        return kStatus_USB_InvalidHandle;
    }

    if (statusType == kOtg_StatusTimeOut)
    {
        otgInstance->hasTimeOutMsg++;
    }
    else if (statusType == kOtg_StatusChange)
    {
        if (otgInstance->hasUpdateMsg == 1)
        {
            return kStatus_USB_Success;
        }
    }
    else if (statusType == kOtg_StatusBHNPFeature)
    {
        if (statusValue)
        {
            otgInstance->otgControllerStatus |= kOtg_StatusBHNPFeature;
        }
        else
        {
            otgInstance->otgControllerStatus &= (~kOtg_StatusBHNPFeature);
        }
    }
    else if (statusType == kOtg_StatusId)
    {
        if (statusValue == 0U)
        {
            otgInstance->idChangeAsFalse = 1U;
        }
    }
    else
    {
    }

    otgMsg.otgStatusType = statusType;
    otgMsg.otgStatusValue = statusValue;
    if (USB_OsaMsgqSend(otgInstance->otgMsgHandle, &otgMsg) == kStatus_USB_OSA_Success)
    {
        if (statusType == kOtg_StatusChange)
        {
            otgInstance->hasUpdateMsg = 1;
        }
        return kStatus_USB_Success;
    }

    return kStatus_USB_Error;
}
