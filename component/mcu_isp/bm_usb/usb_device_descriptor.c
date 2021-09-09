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
#include "usb_device_hid.h"
#include "usb_device_msc.h"
#include "usb_device_dfu.h"
#include "bootloader_hid_report_ids.h"
#include "usb_device_descriptor.h"
#include "composite.h"

#define BL_CONFIG_REPORT_SIZE_BASE (8)
#define BL_FS_CONFIG_REPORT_SIZE (BL_CONFIG_REPORT_SIZE_BASE)
#define BL_HS_CONFIG_REPORT_SIZE (BL_CONFIG_REPORT_SIZE_BASE * BL_CONFIG_REPORT_SIZE_MULTIPLER)

/* hidtc data buffer out report descriptor */
#define HID_USAGE_HIDTC_DATA_OUT(__id, __count, __size)          \
    0x85, ((uint8_t)(__id)),        /*	 REPORT_ID (__id) */      \
        0x19, 0x01,                 /*   USAGE_MINIMUM (1)*/     \
        0x29, 0x01,                 /*   USAGE_MAXIMUM (1)*/     \
        0x15, 0x00,                 /*   LOGICAL_MINIMUM (0)*/   \
        0x26, 0xff, 0x00,           /*   LOGICAL_MAXIMUM (255)*/ \
        0x75, ((uint8_t)(__size)),  /*	 REPORT_SIZE (n)*/        \
        0x95, ((uint8_t)(__count)), /*	 REPORT_COUNT (n)*/       \
        0x91, 0x02                  /*   OUTPUT (Data,Var,Abs) */

/* hidtc data buffer in report descriptor */
#define HID_USAGE_HIDTC_DATA_IN(__id, __count, __size)           \
    0x85, ((uint8_t)(__id)),        /*	 REPORT_ID (__id) */      \
        0x19, 0x01,                 /*   USAGE_MINIMUM (1)*/     \
        0x29, 0x01,                 /*   USAGE_MAXIMUM (1)*/     \
        0x15, 0x00,                 /*   LOGICAL_MINIMUM (0)*/   \
        0x26, 0xff, 0x00,           /*   LOGICAL_MAXIMUM (255)*/ \
        0x75, ((uint8_t)(__size)),  /*	 REPORT_SIZE (n)*/        \
        0x95, ((uint8_t)(__count)), /*	 REPORT_COUNT (n)*/       \
        0x81, 0x02                  /*   INPUT (Data,Var,Abs) */

/* msc disk information */
////////////////////////////////////////////////////////////////////////////////
/* Define endpoint for MSC class */
usb_device_endpoint_struct_t g_msc_disk_endpoints[USB_MSC_ENDPOINT_COUNT] = {
    {
        USB_MSC_BULK_IN_ENDPOINT | (USB_IN << 7), USB_ENDPOINT_BULK, FS_BULK_IN_PACKET_SIZE,
    },
    {
        USB_MSC_BULK_OUT_ENDPOINT | (USB_OUT << 7), USB_ENDPOINT_BULK, FS_BULK_OUT_PACKET_SIZE,
    }
};

/* Define interface for MSC class */
usb_device_interface_struct_t g_msc_disk_interface[] = { {
    0,
    {
        USB_MSC_ENDPOINT_COUNT, g_msc_disk_endpoints,
    },
} };

/* Define interfaces for MSC disk */
usb_device_interfaces_struct_t g_msc_disk_interfaces[USB_MSC_INTERFACE_COUNT] = { {
    USB_MSC_CLASS, USB_MSC_SUBCLASS, USB_MSC_PROTOCOL, USB_MSC_INTERFACE_INDEX, g_msc_disk_interface,
    sizeof(g_msc_disk_interface) / (sizeof(usb_device_interfaces_struct_t)),
} };

/* Define configurations for MSC disk */
usb_device_interface_list_t g_msc_disk_interface_list[USB_DEVICE_CONFIGURATION_COUNT] = {
    {
        USB_MSC_INTERFACE_COUNT, g_msc_disk_interfaces,
    },
};

/* Define class information for MSC disk */
usb_device_class_struct_t g_msc_class = {
    g_msc_disk_interface_list, kUSB_DeviceClassTypeMsc, USB_DEVICE_CONFIGURATION_COUNT,
};

/* dfu device information */
////////////////////////////////////////////////////////////////////////////////
/* Define interfaces for DFU device */
usb_device_interfaces_struct_t g_dfu_interfaces[USB_DFU_INTERFACE_COUNT] = {
    {
        USB_DFU_CLASS,    /* DFU class code */
        USB_DFU_SUBCLASS, /* DFU subclass code */
        USB_DFU_PROTOCOL, /* DFU protocol code */
        0U,               /* The interface number of the DFU */
        NULL,             /* Interfaces handle */
        0U,
    },
};

usb_device_interface_list_t g_dfu_interface_list[USB_DEVICE_CONFIGURATION_COUNT] = {
    {
        USB_DFU_INTERFACE_COUNT, /* The interface count of the DFU demo */
        g_dfu_interfaces,        /* The interfaces handle */
    },
};

usb_device_class_struct_t g_dfu_class = {
    g_dfu_interface_list,           /* The interface list of the DFU demo */
    kUSB_DeviceClassTypeDfu,        /* The DFU class type */
    USB_DEVICE_CONFIGURATION_COUNT, /* The configuration count */
};

/* hid device information */
////////////////////////////////////////////////////////////////////////////////
/* hid generic endpoint information */
usb_device_endpoint_struct_t g_hid_generic_endpoints[USB_HID_GENERIC_ENDPOINT_COUNT] = {
    /* HID generic interrupt IN pipe */
    {
        USB_HID_GENERIC_ENDPOINT_IN | (USB_IN << 7), USB_ENDPOINT_INTERRUPT, FS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE,
    },
    /* HID generic interrupt OUT pipe */
    {
        USB_HID_GENERIC_ENDPOINT_OUT | (USB_OUT << 7), USB_ENDPOINT_INTERRUPT, FS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE,
    }
};

/* HID generic interface information */
usb_device_interface_struct_t g_hid_generic_interface[] = { {
    0, /* The alternate setting of the interface */
    {
        USB_HID_GENERIC_ENDPOINT_COUNT, /* Endpoint count */
        g_hid_generic_endpoints,        /* Endpoints handle */
    },
} };

usb_device_interfaces_struct_t g_hid_generic_interfaces[USB_HID_GENERIC_INTERFACE_COUNT] = { {
    USB_HID_GENERIC_CLASS,           /* HID generic class code */
    USB_HID_GENERIC_SUBCLASS,        /* HID generic subclass code */
    USB_HID_GENERIC_PROTOCOL,        /* HID generic protocol code */
    USB_HID_GENERIC_INTERFACE_INDEX, /* The interface number of the HID generic */
    g_hid_generic_interface,         /* Interfaces handle */
    sizeof(g_hid_generic_interface) / (sizeof(usb_device_interfaces_struct_t)),
} };

usb_device_interface_list_t g_hid_generic_interface_list[USB_DEVICE_CONFIGURATION_COUNT] = {
    {
        USB_HID_GENERIC_INTERFACE_COUNT, /* The interface count of the HID generic */
        g_hid_generic_interfaces,        /* The interfaces handle */
    },
};

usb_device_class_struct_t g_hid_generic_class = {
    g_hid_generic_interface_list,   /* The interface list of the HID generic */
    kUSB_DeviceClassTypeHid,        /* The HID class type */
    USB_DEVICE_CONFIGURATION_COUNT, /* The configuration count */
};

uint8_t g_hid_generic_report_descriptor[] = {
    0x06,
    0x00,
    0xFF, /* Usage Page (Vendor Defined Page 1)*/
    0x09,
    0x01, /* USAGE (Vendor 1) */
    0xA1,
    0x01, /* Collection (Application) */
#if BL_CONFIG_HS_USB_HID
    HID_USAGE_HIDTC_DATA_OUT(kBootloaderReportID_CommandOut, BL_HS_REPORT_SIZE, BL_HS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_OUT(kBootloaderReportID_DataOut, BL_HS_REPORT_SIZE, BL_HS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_IN(kBootloaderReportID_CommandIn, BL_HS_REPORT_SIZE, BL_HS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_IN(kBootloaderReportID_DataIn, BL_HS_REPORT_SIZE, BL_HS_CONFIG_REPORT_SIZE),
#else
    HID_USAGE_HIDTC_DATA_OUT(kBootloaderReportID_CommandOut, BL_FS_REPORT_SIZE, BL_FS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_OUT(kBootloaderReportID_DataOut, BL_FS_REPORT_SIZE, BL_FS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_IN(kBootloaderReportID_CommandIn, BL_FS_REPORT_SIZE, BL_FS_CONFIG_REPORT_SIZE),
    HID_USAGE_HIDTC_DATA_IN(kBootloaderReportID_DataIn, BL_FS_REPORT_SIZE, BL_FS_CONFIG_REPORT_SIZE),
#endif
    0xC0 /* end collection */
};

uint8_t g_device_descriptor[] = {
    USB_DESCRIPTOR_LENGTH_DEVICE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_DEVICE,   /* DEVICE Descriptor Type */
    USB_SHORT_GET_LOW(USB_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_BCD_VERSION), /* USB Specification Release Number in
                                                            Binary-Coded Decimal (i.e., 2.10 is 210H). */
    USB_DEVICE_CLASS,                    /* Class code (assigned by the USB-IF). */
    USB_DEVICE_SUBCLASS,                 /* Subclass code (assigned by the USB-IF). */
    USB_DEVICE_PROTOCOL,                 /* Protocol code (assigned by the USB-IF). */
    USB_CONTROL_MAX_PACKET_SIZE,         /* Maximum packet size for endpoint zero
                                            (only 8, 16, 32, or 64 are valid) */
    kProduct_USB_VID_low, kProduct_USB_VID_high,                        /* Vendor ID (assigned by the USB-IF) */
    kProduct_USB_PID_low, kProduct_USB_PID_high,                        /* Product ID (assigned by the manufacturer) */
    /* Device release number in binary-coded decimal */
    USB_APP_BCD_VERSION_LOW, USB_APP_BCD_VERSION_HIGH, 0x01U, /* Index of string descriptor describing manufacturer */
    0x02U,                                                    /* Index of string descriptor describing product */
/*  Serial number string index */
#if (USB_DEVICE_CONFIG_MSC > 0)
    0x03U,
#else
    0x00U,
#endif
    USB_DEVICE_CONFIGURATION_COUNT, /* Number of possible configurations */
};

uint8_t g_device_descriptor_dfu[] = {
    USB_DESCRIPTOR_LENGTH_DEVICE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_DEVICE,   /* DEVICE Descriptor Type */
    USB_SHORT_GET_LOW(USB_BCD_VERSION_DFU),
    USB_SHORT_GET_HIGH(USB_BCD_VERSION_DFU), /* USB Specification Release Number in
                                                            Binary-Coded Decimal (i.e., 2.10 is 210H). */
    USB_DEVICE_CLASS,                        /* Class code (assigned by the USB-IF). */
    USB_DEVICE_SUBCLASS,                     /* Subclass code (assigned by the USB-IF). */
    USB_DEVICE_PROTOCOL,                     /* Protocol code (assigned by the USB-IF). */
    USB_CONTROL_MAX_PACKET_SIZE,             /* Maximum packet size for endpoint zero
                                                (only 8, 16, 32, or 64 are valid) */
    // By default ROM shall use VID = 0x1FC9 and PID = 0x0020.
    0xC9U, 0x1FU, /* Vendor ID (assigned by the USB-IF) */
    0x0CU, 0x00U, /* Product ID (assigned by the manufacturer) */
    /* Device release number in binary-coded decimal */
    USB_APP_BCD_VERSION_LOW, USB_APP_BCD_VERSION_HIGH, 0x01U, /* Index of string descriptor describing manufacturer */
    0x02U,                                                    /* Index of string descriptor describing product */
    0x00U,                                                    /* Index of string descriptor describing the
                                                                 device's serial number */
    USB_DEVICE_CONFIGURATION_COUNT,                           /* Number of possible configurations */
};

/* Define configuration descriptor */
uint8_t g_config_descriptor[] = {
    USB_DESCRIPTOR_LENGTH_CONFIGURE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CONFIGURE,   /* CONFIGURATION Descriptor Type */
    USB_SHORT_GET_LOW(USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL),
    USB_SHORT_GET_HIGH(USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL),
    /* Total length of data returned for this configuration. */
    USB_COMPOSITE_INTERFACE_COUNT, /* Number of interfaces supported by this configuration */
    USB_COMPOSITE_CONFIGURE_INDEX, /* Value to use as an argument to the
                                            SetConfiguration() request to select this configuration */
    0x00U,                         /* Index of string descriptor describing this configuration */
    /*  Attributes.support RemoteWakeup and self power */
    (USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK) |
        (USB_DEVICE_CONFIG_SELF_POWER << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT) |
        (USB_DEVICE_CONFIG_REMOTE_WAKEUP << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT),

    /* Configuration characteristics
         D7: Reserved (set to one)
         D6: Self-powered
         D5: Remote Wakeup
         D4...0: Reserved (reset to zero)
    */
    USB_DEVICE_MAX_POWER, /* Maximum power consumption of the USB
                           * device from the bus in this specific
                           * configuration when the device is fully
                           * operational. Expressed in 2 mA units
                           * (i.e., 50 = 100 mA).
                           */

#if USB_DEVICE_CONFIG_DFU
    /* DFU Interface Descriptor */
    USB_DESCRIPTOR_LENGTH_INTERFACE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_INTERFACE,   /* INTERFACE Descriptor Type */
    USB_DFU_INTERFACE_INDEX,         /* Number of this interface. */
    0x00U,                           /* Value used to select this alternate setting
                                        for the interface identified in the prior field */
    0x00,                            /* Only the control endpoint is used */
    USB_DFU_CLASS,                   /* Class code (assigned by the USB-IF). */
    USB_DFU_SUBCLASS,                /* Subclass code (assigned by the USB-IF). */
    USB_DFU_PROTOCOL,                /* Protocol code (assigned by the USB). */
    0x03U,                           /* Index of string descriptor describing this interface */

    USB_DESCRIPTOR_LENGTH_FUNCTINAL,    /* size of DFU functional descriptor in bytes */
    USB_DESCRIPTOR_TYPE_DFU_FUNCTIONAL, /* DFU functional descriptor type */
    (USB_DFU_BIT_WILL_DETACH << 3U) | (USB_DFU_BIT_MANIFESTATION_TOLERANT << 2U) | (USB_DFU_BIT_CAN_UPLOAD << 1U) |
        USB_DFU_BIT_CAN_DNLOAD, /* DFU attributes */
    USB_SHORT_GET_LOW(USB_DFU_DETACH_TIMEOUT),
    USB_SHORT_GET_HIGH(USB_DFU_DETACH_TIMEOUT),                                                  /* wDetachTimeout */
    USB_SHORT_GET_LOW(USB_DFU_MAX_TRANSFER_SIZE), USB_SHORT_GET_HIGH(USB_DFU_MAX_TRANSFER_SIZE), /* Max transfer size */
    USB_APP_BCD_VERSION_LOW, USB_APP_BCD_VERSION_HIGH,                                           /* bcdDFUVersion */
#endif // USB_DEVICE_CONFIG_DFU

#if USB_DEVICE_CONFIG_HID
    /* Interface Descriptor */
    USB_DESCRIPTOR_LENGTH_INTERFACE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_INTERFACE,   /* INTERFACE Descriptor Type */
    USB_HID_GENERIC_INTERFACE_INDEX, /* Number of this interface. */
    0x00U,                           /* Value used to select this alternate setting
                                       for the interface identified in the prior field */
    USB_HID_GENERIC_ENDPOINT_COUNT,  /* Number of endpoints used by this
                                       interface (excluding endpoint zero). */
    USB_HID_GENERIC_CLASS,           /* Class code (assigned by the USB-IF). */
    USB_HID_GENERIC_SUBCLASS,        /* Subclass code (assigned by the USB-IF). */
    USB_HID_GENERIC_PROTOCOL,        /* Protocol code (assigned by the USB). */
    0x03,                            /* Index of string descriptor describing this interface */

    /* HID descriptor */
    USB_HID_DESCRIPTOR_LENGTH,      /* Numeric expression that is the total size of the
                                     HID descriptor. */
    USB_DESCRIPTOR_TYPE_HID,        /* Constant name specifying type of HID
                                     descriptor. */
    0x00U, 0x01U,                   /* Numeric expression identifying the HID Class
                                   Specification release. */
    0x00U,                          /* Numeric expression identifying country code of
                                    the localized hardware */
    0x01U,                          /* Numeric expression specifying the number of
                                    class descriptors(at least one report descriptor) */
    USB_DESCRIPTOR_TYPE_HID_REPORT, /* Constant name identifying type of class descriptor. */
    USB_SHORT_GET_LOW(USB_HID_REPORT_DESC_SIZE), USB_SHORT_GET_HIGH(USB_HID_REPORT_DESC_SIZE),

    /*Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,   /* ENDPOINT Descriptor Type */
    USB_HID_GENERIC_ENDPOINT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),
    /* The address of the endpoint on the USB device
       described by this descriptor. */
    USB_ENDPOINT_INTERRUPT, /* This field describes the endpoint's attributes */
    USB_SHORT_GET_LOW(FS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE),
    /* Maximum packet size this endpoint is capable of
       sending or receiving when this configuration is
       selected. */
    FS_HID_GENERIC_INTERRUPT_IN_INTERVAL, /* Interval for polling endpoint for data transfers. */

    /*Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_ENDPOINT,   /* ENDPOINT Descriptor Type */
    USB_HID_GENERIC_ENDPOINT_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),
    /* The address of the endpoint on the USB device
       described by this descriptor. */
    USB_ENDPOINT_INTERRUPT, /* This field describes the endpoint's attributes */
                            /* Maximum packet size this endpoint is capable of
                               sending or receiving when this configuration is
                               selected. */
    USB_SHORT_GET_LOW(FS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE),
    FS_HID_GENERIC_INTERRUPT_OUT_INTERVAL, /* Interval for polling endpoint for data transfers. */

#endif // USB_DEVICE_CONFIG_HID

#if USB_DEVICE_CONFIG_MSC
    /* MSC Interface Descriptor */
    USB_DESCRIPTOR_LENGTH_FUNCTINAL, USB_DESCRIPTOR_TYPE_INTERFACE, USB_MSC_INTERFACE_INDEX, 0x00,
    USB_MSC_ENDPOINT_COUNT, USB_MSC_CLASS, USB_MSC_SUBCLASS, USB_MSC_PROTOCOL,
    0x03, /* Interface Description String Index*/

    /*Bulk IN Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT, USB_MSC_BULK_IN_ENDPOINT | (USB_IN << 7),
    USB_ENDPOINT_BULK, USB_SHORT_GET_LOW(FS_BULK_IN_PACKET_SIZE), USB_SHORT_GET_HIGH(FS_BULK_IN_PACKET_SIZE),
    0x00, /* This value is ignored for Bulk ENDPOINT */

    /*Bulk OUT Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT, USB_MSC_BULK_OUT_ENDPOINT | (USB_OUT << 7),
    USB_ENDPOINT_BULK, USB_SHORT_GET_LOW(FS_BULK_OUT_PACKET_SIZE), USB_SHORT_GET_HIGH(FS_BULK_OUT_PACKET_SIZE),
    0x00, /* This value is ignored for Bulk ENDPOINT */
#endif    // USB_DEVICE_CONFIG_MSC
};

uint8_t g_config_descriptor_dfu[] = {
    USB_DESCRIPTOR_LENGTH_CONFIGURE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CONFIGURE,   /* CONFIGURATION Descriptor Type */
    USB_SHORT_GET_LOW(USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL_DFU),
    USB_SHORT_GET_HIGH(USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL_DFU),
    /* Total length of data returned for this configuration. */
    USB_DFU_INTERFACE_COUNT, /* Number of interfaces supported by this configuration */
    USB_DFU_CONFIGURE_INDEX, /* Value to use as an argument to the
                                        SetConfiguration() request to select this configuration */
    0x00U,                   /* Index of string descriptor describing this configuration */
    (USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK) |
        (USB_DEVICE_CONFIG_SELF_POWER << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT) |
        (USB_DEVICE_CONFIG_REMOTE_WAKEUP << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT),
    /* Configuration characteristics
         D7: Reserved (set to one)
         D6: Self-powered
         D5: Remote Wakeup
         D4...0: Reserved (reset to zero)
    */
    USB_DEVICE_MAX_POWER,            /* Maximum power consumption of the USB
                                      * device from the bus in this specific
                                      * configuration when the device is fully
                                      * operational. Expressed in 2 mA units
                                      * (i.e., 50 = 100 mA).
                                      */
    USB_DESCRIPTOR_LENGTH_INTERFACE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_INTERFACE,   /* INTERFACE Descriptor Type */
    USB_DFU_INTERFACE_INDEX,         /* Number of this interface. */
    0x00U,                           /* Value used to select this alternate setting
                                        for the interface identified in the prior field */
    0x00,                            /* Only the control endpoint is used */
    USB_DFU_CLASS,                   /* Class code (assigned by the USB-IF). */
    USB_DFU_SUBCLASS,                /* Subclass code (assigned by the USB-IF). */
    USB_DFU_MODE_PROTOCOL,           /* Protocol code (assigned by the USB). */
    0x03U,                           /* Index of string descriptor describing this interface */

    USB_DESCRIPTOR_LENGTH_FUNCTINAL,    /* size of DFU functional descriptor in bytes */
    USB_DESCRIPTOR_TYPE_DFU_FUNCTIONAL, /* DFU functional descriptor type */
    (USB_DFU_BIT_WILL_DETACH << 3U) | (USB_DFU_BIT_MANIFESTATION_TOLERANT << 2U) | (USB_DFU_BIT_CAN_UPLOAD << 1U) |
        USB_DFU_BIT_CAN_DNLOAD, /* DFU attributes */
    USB_SHORT_GET_LOW(USB_DFU_DETACH_TIMEOUT),
    USB_SHORT_GET_HIGH(USB_DFU_DETACH_TIMEOUT),                                                  /* wDetachTimeout */
    USB_SHORT_GET_LOW(USB_DFU_MAX_TRANSFER_SIZE), USB_SHORT_GET_HIGH(USB_DFU_MAX_TRANSFER_SIZE), /* Max transfer size */
    USB_APP_BCD_VERSION_LOW, USB_APP_BCD_VERSION_HIGH,                                           /* bcdDFUVersion */
};

/* Define string descriptor */
uint8_t g_usb_str_0[] = {
    2U + 2U, USB_DESCRIPTOR_TYPE_STRING, 0x09U, 0x04U,
};

uint8_t g_usb_str_1[] = { 2U + 2U * 22U, USB_DESCRIPTOR_TYPE_STRING,
                          'N',           0x00U,
                          'X',           0x00U,
                          'P',           0x00U,
                          ' ',           0x00U,
                          'S',           0x00U,
                          'E',           0x00U,
                          'M',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          'O',           0x00U,
                          'N',           0x00U,
                          'D',           0x00U,
                          'U',           0x00U,
                          'C',           0x00U,
                          'T',           0x00U,
                          'O',           0x00U,
                          'R',           0x00U,
                          ' ',           0x00U,
                          'I',           0x00U,
                          'N',           0x00U,
                          'C',           0x00U,
                          '.',           0x00U };

uint8_t g_usb_str_2[] = {
    2U + 2U * 20U, USB_DESCRIPTOR_TYPE_STRING,
    'U',           0x00U,
    'S',           0x00U,
    'B',           0x00U,
    ' ',           0x00U,
    'C',           0x00U,
    'O',           0x00U,
    'M',           0x00U,
    'P',           0x00U,
    'O',           0x00U,
    'S',           0x00U,
    'I',           0x00U,
    'T',           0x00U,
    'E',           0x00U,
    ' ',           0x00U,
    'D',           0x00U,
    'E',           0x00U,
    'V',           0x00U,
    'I',           0x00U,
    'C',           0x00U,
    'E',           0x00U,
};

#if ((USB_DEVICE_CONFIG_HID > 0U) && (USB_DEVICE_CONFIG_DFU > 0U) && (USB_DEVICE_CONFIG_MSC == 0U)) // DFU + HID
uint8_t g_usb_str_3[] = { 2U + 2U * 30U, USB_DESCRIPTOR_TYPE_STRING,
                          'M',           0x00U,
                          'C',           0x00U,
                          'U',           0x00U,
                          ' ',           0x00U,
                          'D',           0x00U,
                          'F',           0x00U,
                          'U',           0x00U,
                          ' ',           0x00U,
                          'A',           0x00U,
                          'N',           0x00U,
                          'D',           0x00U,
                          ' ',           0x00U,
                          'H',           0x00U,
                          'I',           0x00U,
                          'D',           0x00U,
                          ' ',           0x00U,
                          'G',           0x00U,
                          'E',           0x00U,
                          'N',           0x00U,
                          'E',           0x00U,
                          'R',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          ' ',           0x00U,
                          'D',           0x00U,
                          'E',           0x00U,
                          'V',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          'E',           0x00U };
#elif((USB_DEVICE_CONFIG_HID > 0U) && (USB_DEVICE_CONFIG_DFU == 0U) && (USB_DEVICE_CONFIG_MSC > 0U)) // MSC + HID
uint8_t g_usb_str_3[] = { 2U + 2U * 30U, USB_DESCRIPTOR_TYPE_STRING,
                          'M',           0x00U,
                          'C',           0x00U,
                          'U',           0x00U,
                          ' ',           0x00U,
                          'M',           0x00U,
                          'S',           0x00U,
                          'C',           0x00U,
                          ' ',           0x00U,
                          'A',           0x00U,
                          'N',           0x00U,
                          'D',           0x00U,
                          ' ',           0x00U,
                          'H',           0x00U,
                          'I',           0x00U,
                          'D',           0x00U,
                          ' ',           0x00U,
                          'G',           0x00U,
                          'E',           0x00U,
                          'N',           0x00U,
                          'E',           0x00U,
                          'R',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          ' ',           0x00U,
                          'D',           0x00U,
                          'E',           0x00U,
                          'V',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          'E',           0x00U };
#elif(USB_DEVICE_CONFIG_HID > 0U)                                                                    // Only HID
uint8_t g_usb_str_3[] = { 2U + 2U * 22U, USB_DESCRIPTOR_TYPE_STRING,
                          'M',           0x00U,
                          'C',           0x00U,
                          'U',           0x00U,
                          ' ',           0x00U,
                          'H',           0x00U,
                          'I',           0x00U,
                          'D',           0x00U,
                          ' ',           0x00U,
                          'G',           0x00U,
                          'E',           0x00U,
                          'N',           0x00U,
                          'E',           0x00U,
                          'R',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          ' ',           0x00U,
                          'D',           0x00U,
                          'E',           0x00U,
                          'V',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          'E',           0x00U };
#elif(USB_DEVICE_CONFIG_DFU > 0U)                                                                    // Only DFU
uint8_t g_usb_str_3[] = { 2U + 2U * 14U, USB_DESCRIPTOR_TYPE_STRING,
                          'M',           0x00U,
                          'C',           0x00U,
                          'U',           0x00U,
                          ' ',           0x00U,
                          'D',           0x00U,
                          'F',           0x00U,
                          'U',           0x00U,
                          ' ',           0x00U,
                          'D',           0x00U,
                          'E',           0x00U,
                          'V',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          'E',           0x00U };
#elif(USB_DEVICE_CONFIG_MSC > 0U)                                                                    // Only MSC
uint8_t g_usb_str_3[] = { 2U + 2U * 14U, USB_DESCRIPTOR_TYPE_STRING,
                          'M',           0x00U,
                          'C',           0x00U,
                          'U',           0x00U,
                          ' ',           0x00U,
                          'M',           0x00U,
                          'S',           0x00U,
                          'C',           0x00U,
                          ' ',           0x00U,
                          'D',           0x00U,
                          'E',           0x00U,
                          'V',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          'E',           0x00U };
#else
uint8_t g_usb_str_3[] = { 2U + 2U * 14U, USB_DESCRIPTOR_TYPE_STRING,
                          'N',           0x00U,
                          'O',           0x00U,
                          'N',           0x00U,
                          ' ',           0x00U,
                          'U',           0x00U,
                          'S',           0x00U,
                          'B',           0x00U,
                          ' ',           0x00U,
                          'D',           0x00U,
                          'E',           0x00U,
                          'V',           0x00U,
                          'I',           0x00U,
                          'C',           0x00U,
                          'E',           0x00U };
#endif

uint8_t g_usb_str_4[] = { 2U + 2U * 16U, USB_DESCRIPTOR_TYPE_STRING,
                          'U',           0x00U,
                          'N',           0x00U,
                          'D',           0x00U,
                          'E',           0x00U,
                          'F',           0x00U,
                          'I',           0x00U,
                          'N',           0x00U,
                          'E',           0x00U,
                          'D',           0x00U,
                          ' ',           0x00U,
                          'S',           0x00U,
                          'T',           0x00U,
                          'R',           0x00U,
                          'I',           0x00U,
                          'N',           0x00U,
                          'G',           0x00U };

uint8_t g_UsbDeviceOSString[] = {
    (8 * 2 + 2),
    USB_DESCRIPTOR_TYPE_STRING,
    'M',                                       /*Signature:*/
    0x00U,
    'S',
    0x00U,
    'F',
    0x00U,
    'T',
    0x00U,
    '1',
    0x00U,
    '0',
    0x00U,
    '0',
    0x00U,
    0x00,                                       /*Vendor Code*/
    0x00U,
};

uint8_t g_UsbDeviceCompatibleIDDescriptor[] = {    /*Microsoft Compatible ID Feature Descriptor*/
    /*The Header Section*/
    0x28U,0x00U,0x00U,0x00U,                       /*Descriptor length of the complete extended compat ID descriptor*/
    0x00U,0x01U,                                   /*Descriptor's version number*/
    0x04U,0x00U,                                   /*Compatibility ID Descriptor index*/
    0x01U,                                         /*Number of sections */
    0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,     /*Reserved */
    /*The Function Section*/
    0x00U,                                         /*The interface or function number*/
    0x01U,                                         /*Reserved */
    'W','I','N','U','S','B',0x0U,0x0U,             /*Compatible ID("WINUSB\0\0") */
    0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00,/*Sub-Compatible ID*/
    0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,           /*Reserved*/
};


uint8_t g_UsbDeviceOSExendedDescriptor[] = {                 /*Extended Properties Feature Descriptor*/
    /* Header Section*/
    0x8E, 0x00, 0x00, 0x00,                                  /*the length, in bytes, of the complete extended properties descriptor*/
    USB_SHORT_GET_LOW(USB_DEVICE_OS_DESCRIPTOR_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_OS_DESCRIPTOR_BCD_VERSION), /* The descriptor's version number. */
    0x05U,0x00U,                                              /*Descriptor index*/
    0x01U,0x00U,                                              /* The number of custom property sections that follow the header section */
    /*custom Property Section*/
    0x84, 0x00, 0x00, 0x00,                                      /*Size of the property section*/
    0x01, 0x00, 0x00, 0x00,                                   /* Property data type 1 stands for Unicode REG_SZ)  */
    0x28, 0x00,                                                  /* Property name length (40 bytes) */
    /* Property Name ("DeviceInterfaceGUID")  */
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0, 'I', 0, 'n', 0,
    't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0, 'G', 0,
    'U', 0, 'I', 0, 'D', 0, 0, 0,
    0x4E, 0x00, 0x00, 0x00,                                       /* Property data length (78 bytes) */
    /* Property Name "{36FC9E60-C465-11CF-8056-444553540000}"*/
    '{', 0, '3', 0, '6', 0, 'f', 0, 'c', 0, '9', 0, 'e', 0, '6', 0,
    '0', 0, '-', 0, 'c', 0, '4', 0, '6', 0, '5', 0, '-', 0, '1', 0,
    '1', 0, 'c', 0, 'f', 0, '-', 0, '8', 0, '0', 0, '5', 0, '6', 0,
    '-', 0, '4', 0, '4', 0, '4', 0, '5', 0, '5', 0, '3', 0, '5', 0,
    '4', 0, '0', 0, '0', 0, '0', 0, '0', 0, '}', 0, 0, 0,
};

uint32_t g_string_desc_size[USB_DEVICE_STRING_COUNT] = {
    sizeof(g_usb_str_0), sizeof(g_usb_str_1), sizeof(g_usb_str_2), sizeof(g_usb_str_3), sizeof(g_usb_str_4),

};

uint8_t *g_string_descriptors[USB_DEVICE_STRING_COUNT] = {
    g_usb_str_0, g_usb_str_1, g_usb_str_2, g_usb_str_3, g_usb_str_4,
};

usb_language_t g_usb_lang[USB_DEVICE_LANGUAGE_COUNT] = { {
    g_string_descriptors, g_string_desc_size, (uint16_t)0x0409,
} };

usb_language_list_t g_language_list = {
    g_usb_str_0, sizeof(g_usb_str_0), g_usb_lang, USB_DEVICE_LANGUAGE_COUNT,
};

usb_language_list_t *g_language_ptr;

extern usb_device_composite_struct_t g_device_composite;

/*******************************************************************************
 * Code
 ******************************************************************************/
#if USB_DFU_CONFIG_WCID
/* Get verdor descriptor request */
usb_status_t USB_DeviceGetVerdorDescriptor(usb_device_handle handle,
                                           void *param)
{
    usb_status_t errorReturn = kStatus_USB_Error;
    usb_device_control_request_struct_t *controlRequest;
    controlRequest = (usb_device_control_request_struct_t *)param;
    if(g_UsbDeviceOSString[16] != controlRequest->setup->bRequest)
    {
        /*only handle request to its own verdor*/
        return errorReturn;
    }
    if((controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) == USB_REQUEST_TYPE_RECIPIENT_DEVICE)
    {
        if(USB_MICROSOFT_EXTENDED_COMPAT_ID == controlRequest->setup->wIndex)
        {
            controlRequest->buffer = g_UsbDeviceCompatibleIDDescriptor;
            controlRequest->length = USB_DESCRIPTOR_LENGTH_COMPAT;
        }
        if(USB_MICROSOFT_EXTENDED_PROPERTIES_ID == controlRequest->setup->wIndex)
        {
            controlRequest->buffer = g_UsbDeviceOSExendedDescriptor;
            controlRequest->length = USB_DESCRIPTOR_LENGTH_OSExended;
        }
        errorReturn = kStatus_USB_Success;
    }
    if((controlRequest->setup->bmRequestType & USB_REQUEST_TYPE_RECIPIENT_MASK) == USB_REQUEST_TYPE_RECIPIENT_INTERFACE)
    {
        /*add this based on wiki.*/
        /*https://github.com/pbatard/libwdi/wiki/WCID-Devices, Defining a Device Interface GUID or other device specific properties IMPORTANT NOTE 1*/
        if(USB_MICROSOFT_EXTENDED_PROPERTIES_ID == controlRequest->setup->wIndex)
        {
            controlRequest->buffer = g_UsbDeviceOSExendedDescriptor;
            controlRequest->length = USB_DESCRIPTOR_LENGTH_OSExended;
        }
        errorReturn = kStatus_USB_Success;
    }

    return errorReturn;
}
#endif

/*!
 * @brief USB device get device descriptor function.
 *
 * This function gets the device descriptor of the USB devcie.
 *
 * @param handle The USB device handle.
 * @param device_descriptor The pointer to the device descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t usb_device_get_device_descriptor(usb_device_handle handle,
                                              usb_device_get_device_descriptor_struct_t *device_descriptor)
{
#if BL_CONFIG_USB_DFU || BL_CONFIG_HS_USB_DFU
    if (1U == g_device_composite.dfu_downloader.isDfuRequestDetached)
    {
        /*g_device_composite.dfu_downloader.isDfuRequestDetached = 0U;*/
        device_descriptor->buffer = g_device_descriptor_dfu;
        device_descriptor->length = USB_DESCRIPTOR_LENGTH_DEVICE;
    }
    else
#endif
    {
        device_descriptor->buffer = g_device_descriptor;
        device_descriptor->length = USB_DESCRIPTOR_LENGTH_DEVICE;
    }
    return kStatus_USB_Success;
}

/*!
 * @brief USB device get configuration descriptor function.
 *
 * This function gets the configuration descriptor of the USB devcie.
 *
 * @param handle The USB device handle.
 * @param configuration_descriptor The pointer to the configuration descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t usb_device_get_configuration_descriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configuration_descriptor)
{
    if (USB_COMPOSITE_CONFIGURE_INDEX > configuration_descriptor->configuration)
    {
#if BL_CONFIG_USB_DFU || BL_CONFIG_HS_USB_DFU
        if (1U == g_device_composite.dfu_downloader.isDfuRequestDetached)
        {
            /*g_device_composite.dfu_downloader.isDfuRequestDetached = 0U;*/
            configuration_descriptor->buffer = g_config_descriptor_dfu;
            configuration_descriptor->length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL_DFU;
        }
        else
#endif
        {
            configuration_descriptor->buffer = g_config_descriptor;
            configuration_descriptor->length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL;
        }
    }
    return kStatus_USB_Success;
}

/*!
 * @brief USB device get string descriptor function.
 *
 * This function gets the string descriptor of the USB devcie.
 *
 * @param handle The USB device handle.
 * @param string_descriptor Pointer to the string descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t usb_device_get_string_descriptor(usb_device_handle handle,
                                              usb_device_get_string_descriptor_struct_t *string_descriptor)
{
    if (string_descriptor->stringIndex == 0)
    {
        string_descriptor->buffer = (uint8_t *)g_language_list.languageString;
        string_descriptor->length = g_language_list.stringLength;
    }
    else
    {
        uint8_t lang_id = 0;
        uint8_t lang_index = USB_DEVICE_STRING_COUNT;

        for (; lang_id < USB_DEVICE_STRING_COUNT; lang_id++)
        {
            if (string_descriptor->languageId == g_language_list.languageList[lang_id].languageId)
            {
                if (string_descriptor->stringIndex < USB_DEVICE_STRING_COUNT)
                {
                    lang_index = string_descriptor->stringIndex;
                }
                break;
            }
        }
#if USB_DFU_CONFIG_WCID
        if (0xEE == string_descriptor->stringIndex)
        {
            string_descriptor->buffer = (uint8_t *)g_UsbDeviceOSString;
            string_descriptor->length =  sizeof(g_UsbDeviceOSString);
            return kStatus_USB_Success;
        }
#endif
        if (USB_DEVICE_STRING_COUNT == lang_index)
        {
            return kStatus_USB_InvalidRequest;
        }
        string_descriptor->buffer = (uint8_t *)g_language_list.languageList[lang_id].string[lang_index];
        string_descriptor->length = g_language_list.languageList[lang_id].length[lang_index];
    }
    return kStatus_USB_Success;
}

/* Get hid descriptor request */
usb_status_t usb_device_get_hid_descriptor(usb_device_handle handle,
                                           usb_device_get_hid_descriptor_struct_t *hid_descriptor)
{
    return kStatus_USB_InvalidRequest;
}

/* Get hid report descriptor request */
usb_status_t usb_device_get_hid_report_descriptor(usb_device_handle handle,
                                                  usb_device_get_hid_report_descriptor_struct_t *hid_report_descriptor)
{
    if (USB_HID_GENERIC_INTERFACE_INDEX == hid_report_descriptor->interfaceNumber)
    {
        hid_report_descriptor->buffer = g_hid_generic_report_descriptor;
        //        hid_report_descriptor->length = USB_HID_REPORT_DESC_SIZE;
        hid_report_descriptor->length = sizeof(g_hid_generic_report_descriptor);
    }
    else
    {
        return kStatus_USB_InvalidRequest;
    }
    return kStatus_USB_Success;
}

/* Get hid physical descriptor request */
usb_status_t usb_device_get_hid_physical_descriptor(
    usb_device_handle handle, usb_device_get_hid_physical_descriptor_struct_t *hid_physical_descriptor)
{
    return kStatus_USB_InvalidRequest;
}

/*!
 * @brief USB device set speed function.
 *
 * This function sets the speed of the USB devcie.
 *
 * @param handle The USB device handle.
 * @param speed Speed type. USB_SPEED_HIGH/USB_SPEED_FULL/USB_SPEED_LOW.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t usb_device_set_speed(usb_device_handle handle, uint8_t speed)
{
    usb_hid_config_descriptor_t *ptr_hid = NULL;
    usb_msc_config_descriptor_t *ptr_msc = NULL;

#if (USB_DEVICE_CONFIG_HID >= 1) && (USB_DEVICE_CONFIG_MSC == 1)
    ptr_hid = (usb_hid_config_descriptor_t *)&g_config_descriptor[USB_HID_CONFIG_INDEX];
    ptr_msc = (usb_msc_config_descriptor_t *)&g_config_descriptor[USB_MSC_CONFIG_INDEX];
#elif(USB_DEVICE_CONFIG_HID >= 1) && (USB_DEVICE_CONFIG_MSC == 0)
    // HID only
    ptr_hid = (usb_hid_config_descriptor_t *)&g_config_descriptor[USB_HID_CONFIG_INDEX];
#elif(USB_DEVICE_CONFIG_HID == 0) && (USB_DEVICE_CONFIG_MSC == 1)
    // MSC only
    ptr_msc = (usb_msc_config_descriptor_t *)&g_config_descriptor[USB_MSC_CONFIG_INDEX];
#endif

#if USB_DEVICE_CONFIG_DFU
    if (g_device_composite.dfu_downloader.isDfuRequestDetached)
    {
        /*if current device has enter dfu mode, don't need set */
        return kStatus_USB_Success;
    }
#endif

    if (USB_SPEED_HIGH == speed)
    {
        if (ptr_hid != NULL)
        {
            // HID interface
            USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE,
                                               ptr_hid->endpoint_in.wMaxPacketSize);
            ptr_hid->endpoint_in.bInterval = HS_HID_GENERIC_INTERRUPT_IN_INTERVAL;

            USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE,
                                               ptr_hid->endpoint_out.wMaxPacketSize);
            ptr_hid->endpoint_out.bInterval = HS_HID_GENERIC_INTERRUPT_OUT_INTERVAL;
        }
        if (ptr_msc != NULL)
        {
            // MSC interface
            USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_BULK_IN_PACKET_SIZE, ptr_msc->endpoint_in.wMaxPacketSize);
            USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_BULK_OUT_PACKET_SIZE, ptr_msc->endpoint_out.wMaxPacketSize);
        }
    }
    else
    {
        if (ptr_hid != NULL)
        {
            // HID interface
            USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE,
                                               ptr_hid->endpoint_in.wMaxPacketSize);
            ptr_hid->endpoint_in.bInterval = FS_HID_GENERIC_INTERRUPT_IN_INTERVAL;

            USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE,
                                               ptr_hid->endpoint_out.wMaxPacketSize);
            ptr_hid->endpoint_out.bInterval = FS_HID_GENERIC_INTERRUPT_OUT_INTERVAL;
        }
        if (ptr_msc != NULL)
        {
            // MSC interface
            USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_BULK_IN_PACKET_SIZE, ptr_msc->endpoint_in.wMaxPacketSize);
            USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_BULK_OUT_PACKET_SIZE, ptr_msc->endpoint_out.wMaxPacketSize);
        }
    }

    for (uint32_t i = 0; i < USB_HID_GENERIC_ENDPOINT_COUNT; i++)
    {
        if (USB_SPEED_HIGH == speed)
        {
            if (g_hid_generic_endpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN)
            {
                g_hid_generic_endpoints[i].maxPacketSize = HS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE;
            }
            else
            {
                g_hid_generic_endpoints[i].maxPacketSize = HS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE;
            }
        }
        else
        {
            if (g_hid_generic_endpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN)
            {
                g_hid_generic_endpoints[i].maxPacketSize = FS_HID_GENERIC_INTERRUPT_IN_PACKET_SIZE;
            }
            else
            {
                g_hid_generic_endpoints[i].maxPacketSize = FS_HID_GENERIC_INTERRUPT_OUT_PACKET_SIZE;
            }
        }
    }

    for (uint32_t i = 0; i < USB_MSC_ENDPOINT_COUNT; i++)
    {
        if (USB_SPEED_HIGH == speed)
        {
            if ((USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) ==
                (g_msc_disk_endpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
            {
                g_msc_disk_endpoints[i].maxPacketSize = HS_BULK_IN_PACKET_SIZE;
            }
            else
            {
                g_msc_disk_endpoints[i].maxPacketSize = HS_BULK_OUT_PACKET_SIZE;
            }
        }
        else
        {
            if ((USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT) ==
                (g_msc_disk_endpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK))
            {
                g_msc_disk_endpoints[i].maxPacketSize = FS_BULK_IN_PACKET_SIZE;
            }
            else
            {
                g_msc_disk_endpoints[i].maxPacketSize = FS_BULK_OUT_PACKET_SIZE;
            }
        }
    }
    return kStatus_USB_Success;
}
