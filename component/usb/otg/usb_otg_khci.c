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

#if ((defined USB_OTG_CONFIG_KHCI) && (USB_OTG_CONFIG_KHCI))

#include "fsl_device_registers.h"
#include "usb_otg.h"
#include "usb_otg_oci.h"
#include "usb_otg_khci.h"
#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))
#include "usb_otg_peripheral.h"
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void _USB_OtgKhciRequestState(usb_otg_khci_instance_t *otgKhciInstance,
                                     uint32_t requestState,
                                     uint32_t stateMachine);
static usb_status_t _USB_OtgKhciCheckSrp(usb_otg_khci_instance_t *otgKhciInstance);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static usb_status_t _USB_OtgKhciCheckSrp(usb_otg_khci_instance_t *otgKhciInstance)
{
    otgKhciInstance->checkTime++;

    if (otgKhciInstance->checkTime >= USB_OTG_TIME_SRP_TIME_OUT)
    {
        otgKhciInstance->checkTime = 0;
        otgKhciInstance->checkSrpState = 0;
    }

    if (otgKhciInstance->checkSrpState == 0) /* check SE0 */
    {
        if (otgKhciInstance->se0State) /* se0 */
        {
            otgKhciInstance->checkSrpState = 1;
        }
    }
    else if (otgKhciInstance->checkSrpState == 1) /* check J */
    {
        if (otgKhciInstance->jState) /* J */
        {
            otgKhciInstance->checkSrpState = 2;
            otgKhciInstance->checkTime = 0;
        }
        else if ((!(otgKhciInstance->se0State)) &&
                 (otgKhciInstance->usbRegBase->OTGSTAT & USB_OTGSTAT_LINESTATESTABLE_MASK))
        {
            otgKhciInstance->checkSrpState = 0;
        }
        else
        {
        }
    }
    else if (otgKhciInstance->checkSrpState == 2) /* check SE0 */
    {
        if (otgKhciInstance->se0State) /* se0 */
        {
            otgKhciInstance->checkSrpState = 0; /* check next srp */
            if ((otgKhciInstance->checkTime >= USB_OTG_TIME_B_DATA_PLS_MIN) &&
                (otgKhciInstance->checkTime <= USB_OTG_TIME_B_DATA_PLS_MAX))
            {
                otgKhciInstance->checkTime = 0;
                USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusSrpDet, 1);
                return kStatus_USB_Success;
            }
        }
        else if ((!(otgKhciInstance->jState)) &&
                 (otgKhciInstance->usbRegBase->OTGSTAT & USB_OTGSTAT_LINESTATESTABLE_MASK))
        {
            otgKhciInstance->checkSrpState = 0;
        }
        else
        {
        }
    }
    else
    {
    }

    return kStatus_USB_Error;
}

void USB_OtgKhciParsePeripheralStatus(usb_otg_khci_instance_t *otgKhciInstance, uint32_t newStatus)
{
    uint32_t changes = 0;
    uint32_t value = 0;

    changes = otgKhciInstance->peripheralStatus ^ newStatus;
    otgKhciInstance->peripheralStatus = newStatus;

    if (otgKhciInstance->checkType == kOtg_CheckBHNP)
    {
        if (newStatus & kPeripheral_StatusHNPdetected)
        {
            if (otgKhciInstance->lastState == kOtg_State_ASuspend)
            {
                /* keep the pull-up */
                USB_OtgKhciControl(otgKhciInstance, kOtg_ControlPullUp, kOtg_PullDp, 0);
                /* disable hnp check */
                USB_OtgPeripheralControl(otgKhciInstance, kPeripheral_ControlHNPCheckEnable, 0, 0);
                USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusBConn, 0); /* enter a_peripheral */
            }
            else if (otgKhciInstance->lastState == kOtg_State_BWaitAcon)
            {
                USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusAConn, 1); /* enter b_host */
            }
            else
            {
            }
        }
    }

    if (changes & kPeripheral_StatusId)
    {
        value = 0;
        if (newStatus & kPeripheral_StatusId)
        {
            value = 1;
        }
        USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusId, value);
    }

    if (changes & kPeripheral_StatusSessVld)
    {
        value = 0;
        if (newStatus & kPeripheral_StatusSessVld)
        {
            value = 1;
        }
        USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusSessVld, value);
    }

    if (changes & kPeripheral_StatusVbusVld)
    {
        value = 0;
        if (newStatus & kPeripheral_StatusVbusVld)
        {
            value = 1;
        }
        USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusVbusVld, value);
    }
}

static void _USB_OtgKhciRequestState(usb_otg_khci_instance_t *otgKhciInstance,
                                     uint32_t requestState,
                                     uint32_t stateMachine)
{
    uint32_t value;
    otgKhciInstance->checkType = kOtg_CheckNone;

    if ((stateMachine == kOtg_State_ASuspend) || (stateMachine == kOtg_State_BWaitAcon))
    {
        /* enable hnp check */
        USB_OtgPeripheralControl(otgKhciInstance, kPeripheral_ControlHNPCheckEnable, 1, 0);
        otgKhciInstance->checkTime = USB_OTG_TIME_CHECK_BHNP_PERIODIC;
        otgKhciInstance->checkType = kOtg_CheckBHNP;
    }
    else
    {
        /* disable hnp check */
        USB_OtgPeripheralControl(otgKhciInstance, kPeripheral_ControlHNPCheckEnable, 0, 0);
    }

    /*
     * Don't need process the follow requested state because they are notified by interrupt:
     * kOtg_StatusId, kOtg_StatusVbusVld, kOtg_StatusVbusInvld, kOtg_StatusSessInvld, kOtg_StatusSessVld
    */
    if (requestState & kOtg_StatusAdpChange)
    {
        /* todo: not yet */
    }

    if (requestState & kOtg_StatusSrpDet)
    {
        otgKhciInstance->checkTime = 0;
        otgKhciInstance->checkType = kOtg_CheckSrp;
    }

    if (requestState & kOtg_StatusAConn)
    {
        /* check max3353 BHNP periodic */
        otgKhciInstance->checkTime = USB_OTG_TIME_CHECK_BHNP_PERIODIC;
        otgKhciInstance->checkType = kOtg_CheckBHNP;
    }

    if (requestState & kOtg_StatusBusResume)
    {
        /* todo: check resume, not yet */
    }

    if (requestState & kOtg_StatusBusSuspend)
    {
        otgKhciInstance->checkType = kOtg_CheckSuspend;
    }

    if (requestState & kOtg_StatusSe0Srp)
    {
        otgKhciInstance->checkTime = 0;
        otgKhciInstance->checkType = kOtg_CheckSsendSe0Srp; /* check session invalid */
    }

    if (requestState & kOtg_StatusSsendSrp)
    {
        if (otgKhciInstance->bssendsrpCheck > USB_OTG_TIME_B_SSEND_SRP)
        {
            USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusSsendSrp, 1);
        }
    }

    if (requestState & kOtg_StatusBConn)
    {
        if (otgKhciInstance->lastState == kOtg_State_AWaitVrise)
        {
            otgKhciInstance->checkTime = USB_OTG_TIME_A_BCON_LDB;
        }
        else
        {
            otgKhciInstance->checkTime = USB_OTG_TIME_A_BCON_SDB;
        }
        otgKhciInstance->checkType = kOtg_CheckConn;
    }

    if (requestState & kOtg_StatusBDisconn)
    {
        if (stateMachine != kOtg_State_ASuspend)
        {
            otgKhciInstance->checkTime = USB_OTG_TIME_CHECK_DETACH;
            otgKhciInstance->checkType = kOtg_CheckNonConn;
        }
    }

    if (requestState & kOtg_StatusADisconn)
    {
        otgKhciInstance->checkTime = USB_OTG_TIME_CHECK_DETACH;
        otgKhciInstance->checkType = kOtg_CheckNonConn;
    }

    if (requestState & kOtg_StatusCheckIdleInAPeripheral)
    {
        otgKhciInstance->checkType = kOtg_CheckIdleTimeOut;
        otgKhciInstance->lineStableTime = 0U;
    }

#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))
    USB_OtgPeripheralGetStatus(kPeripheral_StatusAll, &value);
    USB_OtgKhciParsePeripheralStatus(otgKhciInstance, value);

    /* at start, don't know the id state, so need notify the id state no matter the id value */
    if ((requestState & kOtg_StatusId) && (stateMachine == kOtg_State_Start))
    {
        USB_OtgPeripheralGetStatus(kPeripheral_StatusAll, &value);
        otgKhciInstance->peripheralStatus = value; /* initialized status */
        if (value & kPeripheral_StatusId)
        {
            value = 1U;
        }
        else
        {
            value = 0U;
        }
        USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusId, value);
    }
#endif

    otgKhciInstance->lastState = stateMachine;
}

usb_status_t USB_OtgKhciInit(uint8_t controllerId,
                             usb_otg_handle otgHandle,
                             usb_otg_controller_handle *controllerHandle)
{
    usb_otg_khci_instance_t *otgKhciInstance;
    uint32_t usbfsBaseAddrs[] = USB_BASE_ADDRS;

    if ((uint32_t)(controllerId - kUSB_ControllerKhci0) >= (sizeof(usbfsBaseAddrs) / sizeof(usbfsBaseAddrs[0])))
    {
        return kStatus_USB_ControllerNotFound;
    }
    if (otgHandle == NULL)
    {
        return kStatus_USB_Error;
    }

    otgKhciInstance = (usb_otg_khci_instance_t *)USB_OsaMemoryAllocate(sizeof(usb_otg_khci_instance_t));
    if (otgKhciInstance == NULL)
    {
        return kStatus_USB_AllocFail;
    }
    otgKhciInstance->otgHandle = otgHandle;
    otgKhciInstance->usbRegBase = (USB_Type *)usbfsBaseAddrs[controllerId - kUSB_ControllerKhci0];
    otgKhciInstance->externalTimerEnable = 0;

    /* initialize controller */
    otgKhciInstance->usbRegBase->ISTAT = 0xFFU; /* clear all interrupts */
    otgKhciInstance->usbRegBase->OTGISTAT = 0xFFU;
    otgKhciInstance->usbRegBase->OTGCTL |= USB_OTGCTL_OTGEN_MASK; /* enable otg */
    otgKhciInstance->usbRegBase->OTGICR |=
        (USB_OTGICR_LINESTATEEN_MASK | USB_OTGICR_ONEMSECEN_MASK); /* 1ms and LINE_STAT_CHG interrupts */

    *controllerHandle = otgKhciInstance;
#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))
    return USB_OtgPeripheralEnable();
#else
    return kStatus_USB_Success;
#endif
}

usb_status_t USB_OtgKhciDeinit(usb_otg_controller_handle controllerHandle)
{
    usb_otg_khci_instance_t *otgKhciInstance = (usb_otg_khci_instance_t *)controllerHandle;

    if (controllerHandle == NULL)
    {
        return kStatus_USB_Error;
    }

    /* clear all interrupts */
    otgKhciInstance->usbRegBase->OTGCTL &= (~USB_OTGCTL_OTGEN_MASK); /* disable otg */
    otgKhciInstance->usbRegBase->ISTAT = 0xFFU;
    otgKhciInstance->usbRegBase->OTGISTAT = 0xFFU;

    USB_OsaMemoryFree(otgKhciInstance);

#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))
    return USB_OtgPeripheralDisable();
#else
    return kStatus_USB_Success;
#endif
}

usb_status_t USB_OtgKhciControl(usb_otg_controller_handle controllerHandle,
                                uint32_t controlType,
                                uint32_t controlValue1,
                                uint32_t controlValue2)
{
    usb_otg_khci_instance_t *otgKhciInstance = (usb_otg_khci_instance_t *)controllerHandle;
    uint32_t statusValue = 0;

    if (controllerHandle == NULL)
    {
        return kStatus_USB_Error;
    }

    switch (controlType)
    {
        case kOtg_ControlPullUp:
            if (controlValue1 & kOtg_PullDp)
            {
#if (FSL_FEATURE_USB_KHCI_OTG_ENABLED)
                otgKhciInstance->usbRegBase->OTGCTL |= USB_OTGCTL_DPHIGH_MASK;
                otgKhciInstance->usbRegBase->CONTROL |= USB_CONTROL_DPPULLUPNONOTG_MASK;
#else
                otgKhciInstance->usbRegBase->CONTROL |= USB_CONTROL_DPPULLUPNONOTG_MASK;
#endif
            }
            else
            {
#if (FSL_FEATURE_USB_KHCI_OTG_ENABLED)
                otgKhciInstance->usbRegBase->OTGCTL &= (~USB_OTGCTL_DPHIGH_MASK);
                otgKhciInstance->usbRegBase->CONTROL &= (~USB_CONTROL_DPPULLUPNONOTG_MASK);
#else
                otgKhciInstance->usbRegBase->CONTROL &= (~USB_CONTROL_DPPULLUPNONOTG_MASK);
#endif
            }
            break;

        case kOtg_ControlPullDown:
            if (controlValue1 & kOtg_PullDp)
            {
                otgKhciInstance->usbRegBase->OTGCTL |= USB_OTGCTL_DPLOW_MASK;
            }
            else
            {
                otgKhciInstance->usbRegBase->OTGCTL &= (~USB_OTGCTL_DPLOW_MASK);
            }

            if (controlValue1 & kOtg_PullDm)
            {
                otgKhciInstance->usbRegBase->OTGCTL |= USB_OTGCTL_DMLOW_MASK;
            }
            else
            {
                otgKhciInstance->usbRegBase->OTGCTL &= (~USB_OTGCTL_DMLOW_MASK);
            }
            break;

        case kOtg_ControlResume:
            if (controlValue1)
            {
                otgKhciInstance->usbRegBase->CTL |= USB_CTL_RESUME_MASK;

                otgKhciInstance->internalTimerValue = controlValue1; /* ms */
                while (otgKhciInstance->internalTimerValue > 0)
                {
                }

                otgKhciInstance->usbRegBase->CTL &= (~USB_CTL_RESUME_MASK);
            }
            break;
#if ((defined USB_OTG_ADP_ENABLE) && (USB_OTG_ADP_ENABLE))
        case kOtg_ControlAdpPrb:
            break;
#endif

        case kOtg_ControlDataPulse:
            /* send srp */
            USB_OtgKhciControl(otgKhciInstance, kOtg_ControlPullUp, kOtg_PullDp, 0);
            otgKhciInstance->internalTimerValue = USB_OTG_TIME_B_DATA_PLS; /* ms */
            while (otgKhciInstance->internalTimerValue > 0)
            {
            }
            USB_OtgKhciControl(otgKhciInstance, kOtg_ControlPullUp, 0, 0);
            USB_OtgNotifyChange(otgKhciInstance->otgHandle, kOtg_StatusBSrpDone, 1);
            break;

        case kOtg_ControlSetTimer:
            otgKhciInstance->externalTimerEnable = 1;
            otgKhciInstance->externalTimerValue = controlValue1;
            break;

        case kOtg_ControlCancelTimer:
            otgKhciInstance->externalTimerEnable = 0;
            break;

        case kOtg_ControlVbus:
#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))
            USB_OtgPeripheralControl(otgKhciInstance, kPeripheral_ControlVbus, controlValue1, 0);
#endif
            break;

        case kOtg_ControlUpdateStatus:
#if ((defined USB_OTG_KHCI_PERIPHERAL_ENABLE) && (USB_OTG_KHCI_PERIPHERAL_ENABLE))
            USB_OtgPeripheralControl(otgKhciInstance, kPeripheral_ControlUpdateStatus, controlValue1, 0);
#endif
            USB_OtgPeripheralGetStatus(kPeripheral_StatusAll, &statusValue);
            USB_OtgKhciParsePeripheralStatus(otgKhciInstance, statusValue);
            break;

        case kOtg_ControlRequestStatus:
            _USB_OtgKhciRequestState(otgKhciInstance, controlValue1, controlValue2);
            break;

        default:
            break;
    }

    return kStatus_USB_Success;
}

void USB_OtgKhciIsrFunction(usb_otg_handle otgHandle)
{
    usb_otg_khci_instance_t *otgKhciInstance;
    uint8_t otgInterrupts;
    uint32_t value;

    if (otgHandle == NULL)
    {
        return;
    }
    otgKhciInstance = (usb_otg_khci_instance_t *)(((usb_otg_instance_t *)otgHandle)->controllerHandle);

    otgInterrupts = (otgKhciInstance->usbRegBase->OTGISTAT & otgKhciInstance->usbRegBase->OTGICR);
    otgKhciInstance->usbRegBase->OTGISTAT = otgInterrupts; /* clear interrupts */

    if ((otgInterrupts & USB_OTGISTAT_LINE_STATE_CHG_MASK) &&
        (otgKhciInstance->usbRegBase->OTGSTAT & USB_OTGSTAT_LINESTATESTABLE_MASK)) /* line state change and stable */
    {
        otgKhciInstance->lineStableTime = 0U;
        otgKhciInstance->se0State = otgKhciInstance->usbRegBase->CTL & USB_CTL_SE0_MASK;
        if (otgKhciInstance->se0State)
        {
            otgKhciInstance->jState = 0;
        }
        else
        {
            otgKhciInstance->jState = otgKhciInstance->usbRegBase->CTL & USB_CTL_JSTATE_MASK;
        }
    }

    if (otgInterrupts & USB_OTGISTAT_ONEMSEC_MASK) /* 1ms */
    {
        if (!(otgKhciInstance->usbRegBase->OTGSTAT & USB_OTGSTAT_LINESTATESTABLE_MASK))
        {
            otgKhciInstance->lineStableTime = 0U;
            otgKhciInstance->se0State = otgKhciInstance->usbRegBase->CTL & USB_CTL_SE0_MASK;
            otgKhciInstance->jState = 0;
        }
        else
        {
            if (otgKhciInstance->lineStableTime != 0xFFFFU)
            {
                otgKhciInstance->lineStableTime++;
            }
        }

        if ((otgKhciInstance->externalTimerEnable) && (otgKhciInstance->externalTimerValue > 0))
        {
            otgKhciInstance->externalTimerValue--;
            if (otgKhciInstance->externalTimerValue == 0)
            {
                otgKhciInstance->externalTimerEnable = 0;
                USB_OtgNotifyChange(otgHandle, kOtg_StatusTimeOut, 1);
            }
        }

        if (otgKhciInstance->internalTimerValue > 0)
        {
            otgKhciInstance->internalTimerValue--;
        }

        switch (otgKhciInstance->checkType)
        {
            case kOtg_CheckNonConn:
                if ((otgKhciInstance->se0State) && (otgKhciInstance->lineStableTime > otgKhciInstance->checkTime))
                {
                    otgKhciInstance->checkType = kOtg_CheckNone;
                    if (otgKhciInstance->lastState == kOtg_State_BHost)
                    {
                        USB_OtgNotifyChange(otgHandle, kOtg_StatusAConn, 0);
                    }
                    else if ((otgKhciInstance->lastState == kOtg_State_AHost))
                    {
                        USB_OtgNotifyChange(otgHandle, kOtg_StatusBConn, 0);
                    }
                    else
                    {
                    }
                }
                break;

            case kOtg_CheckConn:
                if ((!(otgKhciInstance->se0State)) && (otgKhciInstance->jState) &&
                    (otgKhciInstance->usbRegBase->OTGSTAT & USB_OTGSTAT_LINESTATESTABLE_MASK))
                {
                    if (otgKhciInstance->lastState == kOtg_State_AWaitBcon)
                    {
                        if (otgKhciInstance->lineStableTime >= otgKhciInstance->checkTime)
                        {
                            otgKhciInstance->checkType = kOtg_CheckNone;
                            USB_OtgNotifyChange(otgHandle, kOtg_StatusBConn, 1);
                        }
                    }
                }
                break;

            case kOtg_CheckSsendSe0Srp: /* b_idle */
                if ((otgKhciInstance->se0State) &&
                    (otgKhciInstance->lineStableTime >= USB_OTG_TIME_B_SE0_SRP)) /* se0 */
                {
                    otgKhciInstance->lineStableTime = 0;
                    USB_OtgNotifyChange(otgHandle, kOtg_StatusSe0Srp, 1);
                }
                break;

            case kOtg_CheckIdleTimeOut: /* a_peripheral */
                if ((otgKhciInstance->lineStableTime >= USB_OTG_TIME_A_BIDL_ADIS))
                {
                    otgKhciInstance->checkType = kOtg_CheckNone;
                    USB_OtgNotifyChange(otgHandle, kOtg_StatusTimeOut, 1);
                }
                break;

            case kOtg_CheckSrp:
                _USB_OtgKhciCheckSrp(otgKhciInstance);
                break;

            case kOtg_CheckBHNP:
                otgKhciInstance->checkTime--;
                if (otgKhciInstance->checkTime == 0)
                {
                    otgKhciInstance->checkTime = USB_OTG_TIME_CHECK_BHNP_PERIODIC;
                    /* poll for HNP because max3353 has no interrupt for it */
                    USB_OtgNotifyChange(otgHandle, kOtg_StatusChange, 1);
                }
                break;

            case kOtg_CheckSuspend:
                if (otgKhciInstance->lineStableTime >= USB_OTG_TIME_B_AIDL_BDIS) /* suspend */
                {
                    otgKhciInstance->lineStableTime = 0;
                    USB_OtgNotifyChange(otgHandle, kOtg_StatusBusSuspend, 1);
                }
                break;

            default:
                break;
        }

        if (otgKhciInstance->bssendsrpCheck <= USB_OTG_TIME_B_SSEND_SRP)
        {
            USB_OtgPeripheralGetStatus(kPeripheral_StatusSessVld, &value);
            if (value)
            {
                otgKhciInstance->peripheralStatus |= kPeripheral_StatusSessVld;
            }
            else
            {
                otgKhciInstance->peripheralStatus &= (~kPeripheral_StatusSessVld);
            }
            if (!value) /* invalid */
            {
                otgKhciInstance->bssendsrpCheck++;
                if (otgKhciInstance->bssendsrpCheck > USB_OTG_TIME_B_SSEND_SRP)
                {
                    USB_OtgNotifyChange(otgHandle, kOtg_StatusSsendSrp, 1);
                }
            }
#if 0
            else /* valid */
            {
                otgKhciInstance->bssendsrpCheck = 0; /* stop check */
                USB_OtgNotifyChange(otgHandle, kOtg_StatusSessVld, 1);
            }
#endif
        }
    }
}

#endif
