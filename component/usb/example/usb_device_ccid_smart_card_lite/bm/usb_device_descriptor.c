/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
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

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_ccid.h"

#include "usb_device_descriptor.h"
#include "smart_card.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

extern usb_status_t USB_DeviceCallback(usb_device_handle deviceHandle, uint32_t event, void *param);

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_UsbDeviceCurrentConfigure = 0U;
uint8_t g_UsbDeviceInterface[USB_DEVICE_CCID_SMART_CARD_INTERFACE_COUNT];

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceDescriptor[] = {
    USB_DESCRIPTOR_LENGTH_DEVICE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_DEVICE,   /* DEVICE Descriptor Type */
    USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION), /* USB Specification Release Number in
                                                            Binary-Coded Decimal (i.e., 2.10 is 210H). */
    USB_DEVICE_CLASS,                                    /* Class code (assigned by the USB-IF). */
    USB_DEVICE_SUBCLASS,                                 /* Subclass code (assigned by the USB-IF). */
    USB_DEVICE_PROTOCOL,                                 /* Protocol code (assigned by the USB-IF). */
    USB_CONTROL_MAX_PACKET_SIZE,                         /* Maximum packet size for endpoint zero
                                                            (only 8, 16, 32, or 64 are valid) */
    0xC9U, 0x1FU,                                        /* Vendor ID (assigned by the USB-IF) */
    0x9CU, 0x00U,                                        /* Product ID (assigned by the manufacturer) */
    USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION), /* Device release number in binary-coded decimal */
    0x01U,                                           /* Index of string descriptor describing manufacturer */
    0x02U,                                           /* Index of string descriptor describing product */
    0x00U,                                           /* Index of string descriptor describing the
                                                        device's serial number */
    USB_DEVICE_CONFIGURATION_COUNT,                  /* Number of possible configurations */
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceConfigurationDescriptor[] = {
    USB_DESCRIPTOR_LENGTH_CONFIGURE, /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_TYPE_CONFIGURE,   /* CONFIGURATION Descriptor Type */
    USB_SHORT_GET_LOW(USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE +
                      USB_DEVICE_CCID_DESCRIPTOR_LENGTH + USB_DESCRIPTOR_LENGTH_ENDPOINT +
                      USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT),
    USB_SHORT_GET_HIGH(USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE +
                       USB_DEVICE_CCID_DESCRIPTOR_LENGTH + USB_DESCRIPTOR_LENGTH_ENDPOINT +
                       USB_DESCRIPTOR_LENGTH_ENDPOINT +
                       USB_DESCRIPTOR_LENGTH_ENDPOINT), /* Total length of data returned for this configuration. */
    USB_DEVICE_CCID_SMART_CARD_INTERFACE_COUNT,         /* Number of interfaces supported by this configuration */
    USB_DEVICE_CCID_SMART_CARD_CONFIGURE_INDEX,         /* Value to use as an argument to the
                                               SetConfiguration() request to select this configuration */
    0x00U,                                              /* Index of string descriptor describing this configuration */
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
                          *  (i.e., 50 = 100 mA).
                          */
    /* Standard VS Interface Descriptor */
    USB_DESCRIPTOR_LENGTH_INTERFACE,            /* Size of this descriptor */
    USB_DESCRIPTOR_TYPE_INTERFACE,              /* INTERFACE Descriptor */
    USB_DEVICE_CCID_SMART_CARD_INTERFACE_INDEX, /* Index of stream interface */
    0x00U,                                      /* Index of the interface setting */
    USB_DEVICE_CCID_SMART_CARD_ENDPOINT_COUNT,  /* One endpoint of stream pipe */
    USB_DEVICE_CCID_CLASS,                      /* Class */
    USB_DEVICE_CCID_SUBCLASS,                   /* Sub class */
    USB_DEVICE_CCID_PROTOCOL,                   /* Protocol */
    0x03U,                                      /* Index of this string descriptor */

    USB_DEVICE_CCID_DESCRIPTOR_LENGTH, USB_DEVICE_CCID_DESCRIPTOR_TYPE, USB_SHORT_GET_LOW(USB_DEVICE_CCID_VERSION),
    USB_SHORT_GET_HIGH(USB_DEVICE_CCID_VERSION), USB_DEVICE_CCID_SMART_CARD_MAX_SLOTS,
    USB_DEVICE_CCID_SMART_CARD_VOLTAGE_SUPPORT, USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_PROTOCOLS), /* Protocol */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_PROTOCOLS), USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_PROTOCOLS),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_PROTOCOLS),
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_DEFAULT_CLOCK), /* Default clock */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_DEFAULT_CLOCK),
    USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_DEFAULT_CLOCK),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_DEFAULT_CLOCK),
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_MAXIMUM_CLOCK), /* Maximum clock */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_MAXIMUM_CLOCK),
    USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_MAXIMUM_CLOCK),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_MAXIMUM_CLOCK), USB_DEVICE_CCID_SMART_CARD_NUM_CLOCK_SUPPORTED,
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_DATA_RATE), /* Data rate */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_DATA_RATE), USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_DATA_RATE),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_DATA_RATE),
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_MAXIMUM_DATA_RATE), /* Maximum clock */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_MAXIMUM_DATA_RATE),
    USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_MAXIMUM_DATA_RATE),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_MAXIMUM_DATA_RATE),
    USB_DEVICE_CCID_SMART_CARD_NUM_DATA_RATE_SUPPORTED,
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_MAX_IFSD), /* Maximum IFSD */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_MAX_IFSD), USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_MAX_IFSD),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_MAX_IFSD),
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_SYNCH_PROTOCOLS), /* Synch protocols */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_SYNCH_PROTOCOLS),
    USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_SYNCH_PROTOCOLS),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_SYNCH_PROTOCOLS),
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_MECHANICAL), /* Mechanical */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_MECHANICAL),
    USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_MECHANICAL),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_MECHANICAL),
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_FEATURES), /* Features */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_FEATURES), USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_FEATURES),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_FEATURES),
    USB_LONG_GET_BYTE0(USB_DEVICE_CCID_SMART_CARD_MAX_MESSAGE_LENGTH), /* Max message length */
    USB_LONG_GET_BYTE1(USB_DEVICE_CCID_SMART_CARD_MAX_MESSAGE_LENGTH),
    USB_LONG_GET_BYTE2(USB_DEVICE_CCID_SMART_CARD_MAX_MESSAGE_LENGTH),
    USB_LONG_GET_BYTE3(USB_DEVICE_CCID_SMART_CARD_MAX_MESSAGE_LENGTH), USB_DEVICE_CCID_SMART_CARD_CLASS_GET_RESPONSE,
    USB_DEVICE_CCID_SMART_CARD_CLASS_ENVELOPE, USB_SHORT_GET_LOW(USB_DEVICE_CCID_SMART_CARD_LCD_LAYOUT),
    USB_SHORT_GET_HIGH(USB_DEVICE_CCID_SMART_CARD_LCD_LAYOUT), USB_DEVICE_CCID_SMART_CARD_PIN_SUPPORT,
    USB_DEVICE_CCID_SMART_CARD_MAX_BUSY_SLOTS,

    /*Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT,
    USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),
    USB_ENDPOINT_BULK, USB_SHORT_GET_LOW(FS_BULK_IN_PACKET_SIZE), USB_SHORT_GET_HIGH(FS_BULK_IN_PACKET_SIZE),
    /* Max Packet Size */
    0x00U, /* Interval */

    /*Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT,
    USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_OUT | (USB_OUT << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),
    USB_ENDPOINT_BULK, USB_SHORT_GET_LOW(FS_BULK_OUT_PACKET_SIZE), USB_SHORT_GET_HIGH(FS_BULK_OUT_PACKET_SIZE),
    /* Max Packet Size */
    0x00U, /* Interval */

    /*Endpoint descriptor */
    USB_DESCRIPTOR_LENGTH_ENDPOINT, USB_DESCRIPTOR_TYPE_ENDPOINT,
    USB_DEVICE_CCID_SMART_CARD_ENDPOINT_INTERRUPT_IN | (USB_IN << USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT),
    USB_ENDPOINT_INTERRUPT, USB_SHORT_GET_LOW(FS_INTERRUPT_IN_PACKET_SIZE),
    USB_SHORT_GET_HIGH(FS_INTERRUPT_IN_PACKET_SIZE),
    /* Max Packet Size */
    FS_INTERRUPT_IN_INTERVAL, /* Interval */
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString0[] = {
    2U + 2U, USB_DESCRIPTOR_TYPE_STRING, 0x09U, 0x04U,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString1[] = {
    2U + 2U * 18U, USB_DESCRIPTOR_TYPE_STRING,
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
    'S',           0x00U,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString2[] = {
    2U + 2U * 9U, USB_DESCRIPTOR_TYPE_STRING,
    'C',          0x00U,
    'C',          0x00U,
    'I',          0x00U,
    'D',          0x00U,
    ' ',          0x00U,
    'D',          0x00U,
    'E',          0x00U,
    'M',          0x00U,
    'O',          0x00U,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString3[] = {2U + 2U * 15U, USB_DESCRIPTOR_TYPE_STRING,
                                'S',           0x00U,
                                'm',           0x00U,
                                'a',           0x00U,
                                'r',           0x00U,
                                't',           0x00U,
                                ' ',           0x00U,
                                'C',           0x00U,
                                'a',           0x00U,
                                'r',           0x00U,
                                'd',           0x00U,
                                ' ',           0x00U,
                                'D',           0x00U,
                                'e',           0x00U,
                                'm',           0x00U,
                                'o',           0x00U};

uint32_t g_UsbDeviceStringDescriptorLength[USB_DEVICE_STRING_COUNT] = {
    sizeof(g_UsbDeviceString0), sizeof(g_UsbDeviceString1), sizeof(g_UsbDeviceString2), sizeof(g_UsbDeviceString3),
};

uint8_t *g_UsbDeviceStringDescriptorArray[USB_DEVICE_STRING_COUNT] = {
    g_UsbDeviceString0, g_UsbDeviceString1, g_UsbDeviceString2, g_UsbDeviceString3,
};

usb_language_t g_UsbDeviceLanguage[USB_DEVICE_LANGUAGE_COUNT] = {{
    g_UsbDeviceStringDescriptorArray, g_UsbDeviceStringDescriptorLength, (uint16_t)0x0409U,
}};

usb_language_list_t g_UsbDeviceLanguageList = {
    g_UsbDeviceString0, sizeof(g_UsbDeviceString0), g_UsbDeviceLanguage, USB_DEVICE_LANGUAGE_COUNT,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Get descriptor request */
usb_status_t USB_DeviceGetDescriptor(usb_device_handle handle,
                                     usb_setup_struct_t *setup,
                                     uint32_t *length,
                                     uint8_t **buffer)
{
    usb_status_t error = kStatus_USB_Success;
    uint8_t descriptorType = (uint8_t)((setup->wValue & 0xFF00U) >> 8U);
    uint8_t descriptorIndex = (uint8_t)((setup->wValue & 0x00FFU));
    if (USB_REQUEST_STANDARD_GET_DESCRIPTOR != setup->bRequest)
    {
        return kStatus_USB_InvalidRequest;
    }
    switch (descriptorType)
    {
        case USB_DESCRIPTOR_TYPE_STRING:
        {
            /* Get string descriptor */
            if (0U == descriptorIndex)
            {
                *buffer = (uint8_t *)g_UsbDeviceLanguageList.languageString;
                *length = g_UsbDeviceLanguageList.stringLength;
            }
            else
            {
                uint8_t languageId = 0U;
                uint8_t languageIndex = USB_DEVICE_STRING_COUNT;

                for (; languageId < USB_DEVICE_LANGUAGE_COUNT; languageId++)
                {
                    if (setup->wIndex == g_UsbDeviceLanguageList.languageList[languageId].languageId)
                    {
                        if (descriptorIndex < USB_DEVICE_STRING_COUNT)
                        {
                            languageIndex = descriptorIndex;
                        }
                        break;
                    }
                }

                if (USB_DEVICE_STRING_COUNT == languageIndex)
                {
                    return kStatus_USB_InvalidRequest;
                }
                *buffer = (uint8_t *)g_UsbDeviceLanguageList.languageList[languageId].string[languageIndex];
                *length = g_UsbDeviceLanguageList.languageList[languageId].length[languageIndex];
            }
        }
        break;
        case USB_DESCRIPTOR_TYPE_DEVICE:
        {
            /* Get device descriptor */
            *buffer = g_UsbDeviceDescriptor;
            *length = USB_DESCRIPTOR_LENGTH_DEVICE;
        }
        break;
        case USB_DESCRIPTOR_TYPE_CONFIGURE:
        {
            /* Get configuration descriptor */
            *buffer = g_UsbDeviceConfigurationDescriptor;
            *length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL;
        }
        break;
        default:
            error = kStatus_USB_InvalidRequest;
            break;
    } /* End Switch */
    return error;
}

/* Set current confgiuration request */
usb_status_t USB_DeviceSetConfigure(usb_device_handle handle, uint8_t configure)
{
    if (!configure)
    {
        return kStatus_USB_Error;
    }
    g_UsbDeviceCurrentConfigure = configure;
    return USB_DeviceCallback(handle, kUSB_DeviceEventSetConfiguration, &configure);
}

/* Get current confgiuration request */
usb_status_t USB_DeviceGetConfigure(usb_device_handle handle, uint8_t *configure)
{
    *configure = g_UsbDeviceCurrentConfigure;
    return kStatus_USB_Success;
}

/* Set current alternate settting of the interface request */
usb_status_t USB_DeviceSetInterface(usb_device_handle handle, uint8_t interface, uint8_t alternateSetting)
{
    g_UsbDeviceInterface[interface] = alternateSetting;
    return USB_DeviceCallback(handle, kUSB_DeviceEventSetInterface, &interface);
}

/* Get current alternate settting of the interface request */
usb_status_t USB_DeviceGetInterface(usb_device_handle handle, uint8_t interface, uint8_t *alternateSetting)
{
    *alternateSetting = g_UsbDeviceInterface[interface];
    return kStatus_USB_Success;
}

/* Due to the difference of HS and FS descriptors, the device descriptors and configurations need to be updated to match
 * current speed.
 * As the default, the device descriptors and configurations are configured by using FS parameters for both EHCI and
 * KHCI.
 * When the EHCI is enabled, the application needs to call this fucntion to update device by using current speed.
 * The updated information includes endpoint max packet size, endpoint interval, etc. */
usb_status_t USB_DeviceSetSpeed(uint8_t speed)
{
    usb_descriptor_union_t *descriptorHead;
    usb_descriptor_union_t *descriptorTail;

    descriptorHead = (usb_descriptor_union_t *)&g_UsbDeviceConfigurationDescriptor[0];
    descriptorTail =
        (usb_descriptor_union_t *)(&g_UsbDeviceConfigurationDescriptor[USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL - 1U]);

    while (descriptorHead < descriptorTail)
    {
        if (descriptorHead->common.bDescriptorType == USB_DESCRIPTOR_TYPE_ENDPOINT)
        {
            if (USB_SPEED_HIGH == speed)
            {
                if (USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN ==
                    (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_BULK_IN_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize);
                }
                else if (USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_OUT ==
                         (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_BULK_OUT_PACKET_SIZE,
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
                else
                {
                    descriptorHead->endpoint.bInterval = HS_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(HS_INTERRUPT_IN_PACKET_SIZE,
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
            }
            else
            {
                if (USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_IN ==
                    (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_BULK_IN_PACKET_SIZE, descriptorHead->endpoint.wMaxPacketSize);
                }
                else if (USB_DEVICE_CCID_SMART_CARD_ENDPOINT_BULK_OUT ==
                         (descriptorHead->endpoint.bEndpointAddress & USB_ENDPOINT_NUMBER_MASK))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_BULK_OUT_PACKET_SIZE,
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
                else
                {
                    descriptorHead->endpoint.bInterval = FS_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(FS_INTERRUPT_IN_PACKET_SIZE,
                                                       descriptorHead->endpoint.wMaxPacketSize);
                }
            }
        }
        descriptorHead = (usb_descriptor_union_t *)((uint8_t *)descriptorHead + descriptorHead->common.bLength);
    }

    return kStatus_USB_Success;
}
