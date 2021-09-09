/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
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

#ifndef __USB_OTG_OCI_H__
#define __USB_OTG_OCI_H__

/*!
 * @addtogroup usb_otg_controller_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* OTG state machine timer and time periodic */
/*! @brief a_wait_vrise_tmr in OTG spec, VBUS Rise Time, 100ms */
#define USB_OTG_TIMER_A_WAIT_VRISE_TMR (100U)
/*! @brief a_wait_vfall_tmr in OTG spec, Session end to VOTG_VBUS_LKG, 1sec */
#define USB_OTG_TIMER_A_WAIT_VFALL_TMR (1000U)
/*! @brief a_wait_bcon_tmr in OTG spec, Wait for B-Connect, 1.1sec ~ 30^15sec */
#define USB_OTG_TIMER_A_WAIT_BCON_TMR (2000U)
/*! @brief a_aidl_bdis_tmr in OTG spec, A-Idle to B-Disconnect, 200ms ~ infinity */
#define USB_OTG_TIMER_A_AIDL_BDIS_TMR (500)
/*! @brief b_ase0_brst_tmr in OTG spec, A-SE0 to B-Reset, 155ms ~ 200ms */
#define USB_OTG_TIMER_B_ASE0_BRST_TMR (155)
/*! @brief TB_DATA_PLS in OTG spec, Data-Line Pulse Time, 5ms ~ 10ms. generate the data pulse using this time value. */
#define USB_OTG_TIME_B_DATA_PLS (7)
/*! @brief TB_DATA_PLS in OTG spec, Data-Line Pulse Time's minimum value */
#define USB_OTG_TIME_B_DATA_PLS_MIN (5)
/*! @brief TB_DATA_PLS in OTG spec, Data-Line Pulse Time's maximum value */
#define USB_OTG_TIME_B_DATA_PLS_MAX (10)
/*! @brief TA_BCON_LDB in OTG spec, B-Connect Long Debounce, 100ms ~ infinity */
#define USB_OTG_TIME_A_BCON_LDB (100)
/*! @brief TA_BCON_SDB in OTG spec, B-Connect Short Debounce, 2.5us ~ infinity */
#define USB_OTG_TIME_A_BCON_SDB (1)
/*! @brief TB_SSEND_SRP in OTG spec, Session end to SRP init, 1.5sec ~ infinity */
#define USB_OTG_TIME_B_SSEND_SRP (1500)
/*! @brief TB_SE0_SRP in OTG spec, SE0 Time Before SRP, 1sec ~ infinity */
#define USB_OTG_TIME_B_SE0_SRP (1000)
/*! @brief TB_AIDL_BDIS in OTG spec, A-Idle to B-Disconnect, 4ms ~ 150ms */
#define USB_OTG_TIME_B_AIDL_BDIS (100)
/*! @brief TA_BIDL_ADIS in OTG spec, B-Idle to A-Disconnect,
Used by an A-device to determine when the B-device has finished being host, 155ms ~ 200ms */
#define USB_OTG_TIME_A_BIDL_ADIS (190)

/* self-defined constant time value */
/*! @brief wait another device initialize device stack before initializing the host stack */
#define USB_OTG_TIME_WAIT_DEVICE_INIT (200U)
/*! @brief delay this time before check idle in a_peripheral state, wait another device initialize host stack */
#define USB_OTG_TIME_WAIT_BHOST (1000U)

/*! @brief The control types */
typedef enum _usb_otg_control
{
    kOtg_ControlVbus,           /*! control vbus */
    kOtg_ControlPullUp,         /*! pull dp/dm up */
    kOtg_ControlPullDown,       /*! pull dp/dm down */
    kOtg_ControlResume,         /*! do resume */
    kOtg_ControlAdpPrb,         /*! probe adp */
    kOtg_ControlDataPulse,      /*! generate data pulse */
    kOtg_ControlHNPCheckEnable, /*! start to check HNP */
    kOtg_ControlSetTimer,       /*! start timer */
    kOtg_ControlCancelTimer,    /*! cancel timer */
    kOtg_ControlRequestStatus,  /*! request the status values #usb_otg_status_type_t */
    kOtg_ControlUpdateStatus,   /*! notify the controller to update the newest status */
} usb_otg_control_t;

/*! @brief Pull up/down parameters */
typedef enum _usb_otg_pull_control
{
    /*! pull DP line */
    kOtg_PullDp = 0x01U,
    /*! pull DM line */
    kOtg_PullDm = 0x02U,
} usb_otg_pull_control_t;

typedef struct _usb_otg_controller_interface
{
    usb_status_t (*controllerInit)(
        uint8_t controllerId,
        usb_otg_handle upperLayerHandle,
        usb_otg_controller_handle *controllerHandle); /*!< Init a controller instance function prototype*/
    usb_status_t (*controllerDeinit)(
        usb_otg_controller_handle controllerHandle); /*!< Deinit a controller instance function prototype*/
    usb_status_t (*controllerControl)(usb_otg_controller_handle controllerHandle,
                                      uint32_t controlType,
                                      uint32_t controlValue1,
                                      uint32_t controlValue2); /*!< Control a controller function prototype*/
} usb_otg_controller_interface_t;

/*! @brief OTG stack task message */
typedef struct _usb_otg_msg
{
    uint32_t otgStatusType;  /*!< The status types please reference to #usb_otg_status_type_t */
    uint32_t otgStatusValue; /*!< The status values */
} usb_otg_msg_t;

/*! @}*/

/*!
 * @addtogroup usb_otg_drv
 * @{
 */

/*! @brief USB OTG instance structure */
typedef struct _usb_otg_instance
{
    usb_otg_controller_handle controllerHandle;                /*!< The low level controller handle */
    usb_otg_callback_t otgCallback;                            /*!< OTG callback function*/
    void *otgCallbackParameter;                                /*!< OTG callback function parameter */
    usb_osa_msgq_handle otgMsgHandle;                          /*!< OTG task message queue handle */
    const usb_otg_controller_interface_t *controllerInterface; /*!< controller interface APIs */
    uint32_t otgControllerStatus;                              /*!< please reference to #usb_otg_status_type_t */
    uint8_t otgDeviceState;                                    /*!< please reference to #usb_otg_device_state_t */
    volatile uint8_t hasTimeOutMsg;                            /*!< There is timer out message in the message queue */
    volatile uint8_t hasUpdateMsg;                             /*!< There is update message in the message queue */
    uint8_t cancelTime;                                        /*!< Don't process the timer out message */
    uint8_t waitInit; /*!< Waiting the opposite side board's device stack or host stack initializing */
    volatile uint8_t idChangeAsFalse;
} usb_otg_instance_t;

/*! @}*/

#endif /* __USB_OTG_OCI_H__ */
