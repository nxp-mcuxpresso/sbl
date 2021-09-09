/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_msc.h"
#include "usb_device_hid.h"
#include "usb_device_ch9.h"
#if BL_DEVICE_IS_LPC_SERIES
#include "usb_device_descriptor.h"
#else
#include "usb_descriptor.h"
#endif

#include "composite.h"
#include "fsl_device_registers.h"

#include <stdio.h>
#include <stdlib.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
usb_device_class_config_struct_t g_composite_device0[USB_COMPOSITE_INTERFACE_COUNT] = {
#if USB_DEVICE_CONFIG_DFU
    {
        usb_device_dfu_callback, /* DFU demo class callback pointer */
        (class_handle_t)NULL,    /* The DFU class handle, This field is set by USB_DeviceClassInit */
        &g_dfu_class,            /* The DFU demo configuration, including class code, subcode, and protocol, class
                                    type */
    },
#endif
#if USB_DEVICE_CONFIG_HID
    {
        usb_device_hid_generic_callback, /* HID class callback pointer */
        (class_handle_t)NULL,            /* The HID class handle, This field is set by USB_DeviceClassInit */
        &g_hid_generic_class,            /* The HID configuration, including class code, subcode, and protocol,
                                          class type, transfer type, endpoint address, max packet size, etc.*/
    },
#endif
#if USB_DEVICE_CONFIG_MSC
    {
        usb_device_msc_callback, (class_handle_t)NULL, &g_msc_class,
    },
#endif
#if ((USB_DEVICE_CONFIG_HID == 0) && (USB_DEVICE_CONFIG_DFU == 0) && (USB_DEVICE_CONFIG_MSC == 0))
    {
        NULL, NULL, NULL,
    }
#endif
};

usb_device_class_config_struct_t g_composite_device1[USB_COMPOSITE_INTERFACE_COUNT] = {
#if USB_DEVICE_CONFIG_DFU
    {
        usb_device_dfu_callback, /* DFU demo class callback pointer */
        (class_handle_t)NULL,    /* The DFU class handle, This field is set by USB_DeviceClassInit */
        &g_dfu_class,            /* The DFU demo configuration, including class code, subcode, and protocol, class
                                    type */
    },
#endif
#if USB_DEVICE_CONFIG_HID
    {
        usb_device_hid_generic_callback, /* HID class callback pointer */
        (class_handle_t)NULL,            /* The HID class handle, This field is set by USB_DeviceClassInit */
        &g_hid_generic_class,            /* The HID configuration, including class code, subcode, and protocol,
                                          class type, transfer type, endpoint address, max packet size, etc.*/
    },
#endif
#if USB_DEVICE_CONFIG_MSC
    {
        usb_device_msc_callback, (class_handle_t)NULL, &g_msc_class,
    },
#endif
#if ((USB_DEVICE_CONFIG_HID == 0) && (USB_DEVICE_CONFIG_DFU == 0) && (USB_DEVICE_CONFIG_MSC == 0))
    {
        NULL, NULL, NULL,
    }
#endif
};

usb_device_class_config_list_struct_t g_composite_device_config_list[2] = {{
    g_composite_device0,            /* Class configurations */
    usb_device_callback,           /* Device callback pointer */
    USB_COMPOSITE_INTERFACE_COUNT, /* Class count */
},{
    g_composite_device1,            /* Class configurations */
    usb_device_callback,           /* Device callback pointer */
    USB_COMPOSITE_INTERFACE_COUNT, /* Class count */
}};

uint8_t g_usbControllerId;
