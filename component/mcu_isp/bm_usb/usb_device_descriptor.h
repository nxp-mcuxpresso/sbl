/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DEVICE_DESCRIPTOR_H__
#define __USB_DEVICE_DESCRIPTOR_H__

#include "bootloader/bl_version.h"
#include "usb_ksdk/api_usb_ksdk.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USB_DFU_HID_COMPOSITE_APP (1)
#define USB_DFU_CONFIG_WCID (0)
#define USB_BCD_VERSION (0x0200U)
#define USB_BCD_VERSION_DFU (0x0100U)
#define USB_DEVICE_OS_DESCRIPTOR_BCD_VERSION (0x0100U)
// ROM shall set bcdDevice field in device descriptor to ROM version number
//  in BCD format (Major version in upper byte, minor version lower byte).
#define USB_APP_BCD_VERSION_HIGH (kBootloader_Version_Major)
#define USB_APP_BCD_VERSION_LOW (kBootloader_Version_Minor)

#define USB_DEVICE_CLASS (0x00U)
#define USB_DEVICE_SUBCLASS (0x00U)
#define USB_DEVICE_PROTOCOL (0x00U)
#define USB_DEVICE_CONFIGURATION_COUNT (1U)
#define USB_DEVICE_STRING_COUNT (5U)
#define USB_DEVICE_LANGUAGE_COUNT (1U)
#define USB_COMPOSITE_CONFIGURE_INDEX (1U)

#define USB_DEVICE_MAX_POWER (0x32U)
#define USB_CONFIGURE_DRAWN (0x32)

/*******************************************************************************
 * For Kibble HID configurations
 *******************************************************************************/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// ROM need to fill in the report size for each endpoints (1,2,3,4)
//  See g_hid_generic_report_descriptor[] for more details.
//  SW needs to fill in the descriptor by using
//                   - HID_USAGE_HIDTC_DATA_OUT(__id, __count, __size)
//                   - HID_USAGE_HIDTC_DATA_IN(__id, __count, __size)
//
//  (__size * __count) / 8(bits) = BL_ACTUAL_REPORT_SIZE
//  Considering that both __size and __count are just 1-bit field,
//  The formula of calculation for HID related fields is as follows:
//  For HID communication: The wMaxPacketSize = BL_ACTUAL_REPORT_SIZE + 1
//  BL_HS_REPORT_SIZE * BL_HS_CONFIG_REPORT_SIZE = BL_ACTUAL_REPORT_SIZE * 8 (bits)
//  In which:
//  - __size = BL_xS_CONFIG_REPORT_SIZE = 8*BL_CONFIG_REPORT_SIZE_MULTIPLER (4 < BL_CONFIG_REPORT_SIZE_MULTIPLER < 64)
//  - __count = BL_HS_REPORT_SIZE = BL_ACTUAL_REPORT_SIZE/BL_CONFIG_REPORT_SIZE_MULTIPLER(0 < BL_HS_REPORT_SIZE < 256)
//
//  For FS HID, due to the wMaxPacketSize <= 64
//      -   The BL_FS_CONFIG_REPORT_SIZE can be just 8
//      -   The BL_FS_REPORT_SIZE can be (BL_MIN_PACKET_SIZE + BL_PACKET_SIZE_HEADER_SIZE)
//
//  For HS HID, due to the wMaxPacket can be up to 1024
//      -  The BL_HS_CONFIG_REPORT_SIZE and BL_HS_REPORT_SIZE needs to be selected by above formula. Some typical
//          values are listed below.
//
//
//          -- Note--:
//          In later wMaxPacketSize configuration, the wMaxPacketSize must be at least :
//          (BL_ACTUAL_REPORT_SIZE + 1)
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define BL_MIN_PACKET_SIZE (BL_EXPANDED_USB_HID_PACKET_SIZE)
#define BL_FS_MAX_PACKET_SIZE (60)
#define BL_PACKET_SIZE_HEADER_SIZE (3) // alignment byte + length lsb + length msb (does not include report id)

#if (BL_MIN_PACKET_SIZE == 1017) || (BL_MIN_PACKET_SIZE == 1012)
#define BL_CONFIG_REPORT_SIZE_MULTIPLER (5)
#elif(BL_MIN_PACKET_SIZE == 1019)
#define BL_CONFIG_REPORT_SIZE_MULTIPLER (7)
#elif(BL_MIN_PACKET_SIZE < 64)
#define BL_CONFIG_REPORT_SIZE_MULTIPLER (1)
#endif

#define BL_HS_ACTUAL_REPORT_SIZE (BL_MIN_PACKET_SIZE + BL_PACKET_SIZE_HEADER_SIZE)
#define BL_FS_ACTUAL_REPORT_SIZE (BL_FS_MAX_PACKET_SIZE + BL_PACKET_SIZE_HEADER_SIZE)
#define BL_FS_REPORT_SIZE BL_FS_ACTUAL_REPORT_SIZE
#define BL_HS_REPORT_SIZE (BL_HS_ACTUAL_REPORT_SIZE / BL_CONFIG_REPORT_SIZE_MULTIPLER)

// Static check for the validity of BL_HS_ACTUAL_REPORT_SIZE and BL_CONFIG_REPORT_SIZE_MULTIPLER
#if (BL_HS_ACTUAL_REPORT_SIZE > 63)
#if (BL_HS_REPORT_SIZE * BL_CONFIG_REPORT_SIZE_MULTIPLER != BL_HS_ACTUAL_REPORT_SIZE)
#error BL_HS_ACTUAL_REPORT_SIZE must be divisible by BL_CONFIG_REPORT_SIZE_MULTIPLER
#endif
#if (BL_CONFIG_REPORT_SIZE_MULTIPLER < 5) || (BL_CONFIG_REPORT_SIZE_MULTIPLER > 63)
#error BL_CONFIG_REPORT_SIZE_MULTIPLER must be greater than 4 and less than 64
#endif
#endif

// Defines the length of g_usb_str_x[]
////////////////////////////////////////////////////////////////////////////////
#define USB_DESCRIPTOR_LENGTH_FUNCTINAL (9U)
#define USB_DESCRIPTOR_LENGTH_STRING0 (sizeof(g_usb_str_0))
#define USB_DESCRIPTOR_LENGTH_STRING1 (sizeof(g_usb_str_1))
#define USB_DESCRIPTOR_LENGTH_STRING2 (sizeof(g_usb_str_2))
#define USB_DESCRIPTOR_LENGTH_STRING3 (sizeof(g_usb_str_3))
#define USB_DESCRIPTOR_LENGTH_STRING4 (sizeof(g_usb_str_4))
#define USB_DESCRIPTOR_LENGTH_OSExended (sizeof(g_UsbDeviceOSExendedDescriptor))
#define USB_DESCRIPTOR_LENGTH_COMPAT (sizeof(g_UsbDeviceCompatibleIDDescriptor))

// DFU
////////////////////////////////////////////////////////////////////////////////
#define USB_DESCRIPTOR_TYPE_DFU_FUNCTIONAL (0x21)
#define USB_DFU_DETACH_TIMEOUT (0x6400)
#define USB_DFU_BIT_WILL_DETACH (1U)
#define USB_DFU_BIT_MANIFESTATION_TOLERANT (0U)
#define USB_DFU_BIT_CAN_UPLOAD (0U)
#define USB_DFU_BIT_CAN_DNLOAD (1U)
#define USB_DFU_MAX_TRANSFER_SIZE (0x400)
#define USB_DFU_INTERFACE_COUNT (1U)
#define USB_DFU_CONFIGURE_INDEX (1U)
#define USB_DFU_CLASS (0xFEU)
#define USB_DFU_SUBCLASS 0x01
#define USB_DFU_PROTOCOL (0x01U)
#define USB_DFU_MODE_PROTOCOL (0x02U)

#define USB_MICROSOFT_EXTENDED_COMPAT_ID (0x0004U)
#define USB_MICROSOFT_EXTENDED_PROPERTIES_ID (0x0005U)
// HID
////////////////////////////////////////////////////////////////////////////////
#define USB_HID_REPORT_DESC_SIZE (sizeof(g_hid_generic_report_descriptor))
#define USB_HID_GENERIC_DESCRIPTOR_LENGTH (32)
#define USB_HID_DESCRIPTOR_LENGTH (9)
#define USB_IAD_DESC_SIZE (8)
#define USB_HID_GENERIC_INTERFACE_COUNT (1)
#define USB_HID_GENERIC_CONFIGURE_INDEX (1)
#define USB_HID_GENERIC_IN_BUFFER_LENGTH (8)
#define USB_HID_GENERIC_OUT_BUFFER_LENGTH (8)
#define USB_HID_GENERIC_ENDPOINT_COUNT (2)
#define USB_HID_GENERIC_ENDPOINT_IN (1)
#define USB_HID_GENERIC_ENDPOINT_OUT (2)
#define USB_HID_GENERIC_CLASS (0x03)
#define USB_HID_GENERIC_SUBCLASS (0x00)
#define USB_HID_GENERIC_PROTOCOL (0x00)

/*
 * o Configure wMaxPacketSize, Maximum packet size this endpoint is capable of
 *   sending or receiving when this configuration is selected;
 * o Configure bInterval, Interval for polling endpoint for data transfers.
 *   Expressed in frames or microframes depending on the device operating
 *   speed (i.e., either 1 millisecond or 125 µs units),For interrupt endpoints,
 *   the bInterval value is used as the exponent for a 2^(bInterval-1) value;
 *   High-speed endpoints can specify a desired period 2^(bInterval-1)x125 µs,
 *   where bInterval value is in the range 1 to (including)16, and 2^(bInterval-1)x1ms
 *   for full-speed where the value may be from 1 to 255;
 * o For the HS/FS USB-HID devices, the max supported usb-hid device
 *	 numbers(Num) in one usb hub are calculated as follows:
 *   Num = (T_DRAT * Period * Percentage / endpointNumbers) / wMaxPacketSize
 *   Where the high-speed and full-speed data rate (T_DRAT) are nominally
 *	 480.00 Mb/s, 12.00 Mb/s. High-speed endpoints can be allocated at most
 *   80%(Percentage) of a microframe for periodic transfers. The USB requires that
 *   no more than 90%(Percentage) of any frame be allocated for periodic full-speed
 *   transfers, and the transfer speed is calculated as wMaxPacketSize / Period.
 *   Period is calculated as 2^(bInterval-1) * (frames or microframes).
 */
#define USB_HID_REPORT_ID_SIZE (1)
#define HS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE (BL_HS_ACTUAL_REPORT_SIZE + USB_HID_REPORT_ID_SIZE)
#define HS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE (BL_HS_ACTUAL_REPORT_SIZE + USB_HID_REPORT_ID_SIZE)
#define HS_HID_GENERIC_INTERRUPT_OUT_INTERVAL (0x03) /* 2^(3-1) *0.125ms = 0.5ms */
#define HS_HID_GENERIC_INTERRUPT_IN_INTERVAL (0x03) /* 2^(3-1) *0.125ms = 0.5ms */
#define FS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE (BL_FS_ACTUAL_REPORT_SIZE + USB_HID_REPORT_ID_SIZE)
#define FS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE (BL_FS_ACTUAL_REPORT_SIZE + USB_HID_REPORT_ID_SIZE)
#define FS_HID_GENERIC_INTERRUPT_OUT_INTERVAL (0x01) /* 2^(1-1) *1ms = 1ms */
#define FS_HID_GENERIC_INTERRUPT_IN_INTERVAL (0x01) /* 2^(1-1) *1ms = 1ms */

// MSC
////////////////////////////////////////////////////////////////////////////////
#define USB_MSC_CLASS (0x08)
#define USB_MSC_SUBCLASS (0x06)
#define USB_MSC_PROTOCOL (0x50)
#define HS_BULK_IN_PACKET_SIZE (512)
#define HS_BULK_OUT_PACKET_SIZE (512)
#define FS_BULK_IN_PACKET_SIZE (64)
#define FS_BULK_OUT_PACKET_SIZE (64)
#define USB_MSC_CONFIGURE_INDEX (1)
#define USB_MSC_ENDPOINT_COUNT (2)
#define USB_MSC_BULK_IN_ENDPOINT (3)
#define USB_MSC_BULK_OUT_ENDPOINT (4)
#define USB_MSC_INTERFACE_COUNT (1)

// Defines the length of g_config_descriptor[] and g_config_descriptor_dfu[]
////////////////////////////////////////////////////////////////////////////////
#if (USB_DEVICE_CONFIG_HID >= 1)
#define USB_DESCRIPTOR_LENGTH_HID_PART \
    (USB_DESCRIPTOR_LENGTH_FUNCTINAL + USB_HID_DESCRIPTOR_LENGTH + 2 * USB_DESCRIPTOR_LENGTH_ENDPOINT)
#else
#define USB_DESCRIPTOR_LENGTH_HID_PART (0)
#endif
#if (USB_DEVICE_CONFIG_DFU == 1)
#define USB_DESCRIPTOR_LENGTH_DFU_PART (USB_DESCRIPTOR_LENGTH_INTERFACE + USB_DESCRIPTOR_LENGTH_FUNCTINAL)
#else
#define USB_DESCRIPTOR_LENGTH_DFU_PART (0)
#endif
#if (USB_DEVICE_CONFIG_MSC == 1)
#define USB_DESCRIPTOR_LENGTH_MSC_PART (USB_DESCRIPTOR_LENGTH_FUNCTINAL + 2 * USB_DESCRIPTOR_LENGTH_ENDPOINT)
#else
#define USB_DESCRIPTOR_LENGTH_MSC_PART (0)
#endif

#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL                                                          \
    (USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_DFU_PART + USB_DESCRIPTOR_LENGTH_HID_PART + \
     USB_DESCRIPTOR_LENGTH_MSC_PART)
#define USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL_DFU (USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_DFU_PART)

// Defines the configuration index of each class in g_config_descriptor[]
////////////////////////////////////////////////////////////////////////////////
#define USB_DFU_CONFIG_INDEX (USB_DESCRIPTOR_LENGTH_CONFIGURE)
#define USB_HID_CONFIG_INDEX (USB_DFU_CONFIG_INDEX + USB_DESCRIPTOR_LENGTH_DFU_PART)
#define USB_MSC_CONFIG_INDEX (USB_HID_CONFIG_INDEX + USB_DESCRIPTOR_LENGTH_HID_PART)

// Defines the interface index and count of each class in g_composite_device[]
////////////////////////////////////////////////////////////////////////////////
#if (USB_DEVICE_CONFIG_HID >= 1) && (USB_DEVICE_CONFIG_DFU == 1) && (USB_DEVICE_CONFIG_MSC == 0)
#define USB_COMPOSITE_INTERFACE_COUNT (USB_HID_GENERIC_INTERFACE_COUNT + USB_DFU_INTERFACE_COUNT)
#define USB_DFU_INTERFACE_INDEX (0)
#define USB_HID_GENERIC_INTERFACE_INDEX (1)
#define USB_MSC_INTERFACE_INDEX (0)
#elif(USB_DEVICE_CONFIG_HID >= 1) && (USB_DEVICE_CONFIG_DFU == 0) && (USB_DEVICE_CONFIG_MSC == 1)
#define USB_COMPOSITE_INTERFACE_COUNT (USB_HID_GENERIC_INTERFACE_COUNT + USB_MSC_INTERFACE_COUNT)
#define USB_DFU_INTERFACE_INDEX (0)
#define USB_HID_GENERIC_INTERFACE_INDEX (0)
#define USB_MSC_INTERFACE_INDEX (1)
#elif(USB_DEVICE_CONFIG_HID >= 1) && (USB_DEVICE_CONFIG_DFU == 0) && (USB_DEVICE_CONFIG_MSC == 0)
#define USB_COMPOSITE_INTERFACE_COUNT (USB_HID_GENERIC_INTERFACE_COUNT)
#define USB_DFU_INTERFACE_INDEX (0)
#define USB_HID_GENERIC_INTERFACE_INDEX (0)
#define USB_MSC_INTERFACE_INDEX (0)
#else
#define USB_COMPOSITE_INTERFACE_COUNT (USB_HID_GENERIC_INTERFACE_COUNT)
#define USB_DFU_INTERFACE_INDEX (0)
#define USB_HID_GENERIC_INTERFACE_INDEX (0)
#define USB_MSC_INTERFACE_INDEX (0)
#endif

enum _usb_descriptor_index
{
    kUsbDescriptorIndex_VidLow = 8,
    kUsbDescriptorIndex_VidHigh = 9,
    kUsbDescriptorIndex_PidLow = 10,
    kUsbDescriptorIndex_PidHigh = 11
};

typedef struct _usb_hid_config_descriptor
{
    usb_descriptor_interface_t interface;   /* Interface descriptor */
    usb_descriptor_interface_t hid_report;  /* Interface descriptor */
    usb_descriptor_endpoint_t endpoint_in;  /* Endpoint descriptor */
    usb_descriptor_endpoint_t endpoint_out; /* Endpoint descriptor */
} usb_hid_config_descriptor_t;

typedef struct _usb_msc_config_descriptor
{
    usb_descriptor_interface_t interface;   /* Interface descriptor */
    usb_descriptor_endpoint_t endpoint_in;  /* Endpoint descriptor */
    usb_descriptor_endpoint_t endpoint_out; /* Endpoint descriptor */
} usb_msc_config_descriptor_t;

extern usb_device_class_struct_t g_hid_generic_class;
extern usb_device_class_struct_t g_msc_class;
extern usb_device_class_struct_t g_dfu_class;

extern uint8_t g_device_descriptor[];
extern usb_language_list_t g_language_list;
extern usb_language_list_t *g_language_ptr;
extern uint8_t g_usb_str_4[];

/*******************************************************************************
 * API
 ******************************************************************************/
/* Get device descriptor request */
usb_status_t usb_device_get_device_descriptor(usb_device_handle handle,
                                              usb_device_get_device_descriptor_struct_t *device_descriptor);

/* Get device configuration descriptor request */
usb_status_t usb_device_get_configuration_descriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configuration_descriptor);

/* Get device string descriptor request */
usb_status_t usb_device_get_string_descriptor(usb_device_handle handle,
                                              usb_device_get_string_descriptor_struct_t *string_descriptor);

/* Configure the device according to the USB speed. */
extern usb_status_t usb_device_set_speed(usb_device_handle handle, uint8_t speed);

/* Get hid descriptor request */
usb_status_t usb_device_get_hid_descriptor(usb_device_handle handle,
                                           usb_device_get_hid_descriptor_struct_t *hid_descriptor);

/* Get hid report descriptor request */
usb_status_t usb_device_get_hid_report_descriptor(usb_device_handle handle,
                                                  usb_device_get_hid_report_descriptor_struct_t *hid_report_descriptor);

/* Get hid physical descriptor request */
usb_status_t usb_device_get_hid_physical_descriptor(
    usb_device_handle handle, usb_device_get_hid_physical_descriptor_struct_t *hid_physical_descriptor);

#if USB_DFU_CONFIG_WCID
/* Get device vendor descriptor request */
extern usb_status_t USB_DeviceGetVerdorDescriptor(usb_device_handle handle, void *param);
#endif
#endif /* __USB_DEVICE_DESCRIPTOR_H__ */
