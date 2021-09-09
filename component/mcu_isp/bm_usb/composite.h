/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DEVICE_COMPOSITE_H__
#define __USB_DEVICE_COMPOSITE_H__

#include "msc_disk.h"
#include "hid_bootloader.h"
#if (BL_CONFIG_USB_DFU || BL_CONFIG_HS_USB_DFU)
#include "dfu_downloader.h"
#endif
#include "usb_device_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern uint8_t g_usbControllerId;

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)
#define CONTROLLER_ID kUSB_ControllerEhci0
#endif
#if defined(USB_DEVICE_CONFIG_KHCI) && (USB_DEVICE_CONFIG_KHCI > 0U)
#define CONTROLLER_ID kUSB_ControllerKhci0
#endif
/*
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Fs0
#endif
*/
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
#define CONTROLLER_ID kUSB_ControllerLpcIp3511Hs0
#endif


//#define USB_DEVICE_INTERRUPT_PRIORITY (3U)

typedef struct _usb_device_composite_struct
{
    usb_device_handle device_handle; // USB device handle.
#if USB_DEVICE_CONFIG_HID
    usb_hid_generic_struct_t hid_generic; // HID device structure
#endif
#if USB_DEVICE_CONFIG_MSC
    usb_msc_struct_t msc_disk; // MSC disk device structure.
#endif                         // USB_DEVICE_CONFIG_MSC
#if USB_DEVICE_CONFIG_DFU
    usb_dfu_struct_t dfu_downloader; // DFU device structure.
#endif
    uint8_t speed;                 // Speed of USB device. USB_SPEED_FULL/USB_SPEED_LOW/USB_SPEED_HIGH.
    uint8_t attach;                // A flag to indicate whether a usb device is attached. 1: attached, 0: not attached
    uint8_t current_configuration; // Current configuration value.
    uint8_t current_interface_alternate_setting[USB_COMPOSITE_INTERFACE_COUNT]; // Current alternate setting value for
                                                                                // each interface.
} usb_device_composite_struct_t;

extern usb_status_t usb_device_callback(usb_device_handle handle, uint32_t event, void *param);

#if (BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID)
// HID device initialization function
extern usb_status_t usb_device_hid_generic_init(usb_device_composite_struct_t *device_composite,uint32_t hidInfoIndex);
// HID class specific callback function
extern usb_status_t usb_device_hid_generic_callback(class_handle_t handle, uint32_t event, void *param);
// HID device set configuration function
extern usb_status_t usb_device_hid_generic_set_configure(class_handle_t handle, uint8_t configure);
// HID device set interface function
extern usb_status_t usb_device_hid_generic_set_interface(class_handle_t handle,
                                                         uint8_t interface,
                                                         uint8_t alternate_setting);
// HID device de-initialization function
extern usb_status_t usb_device_hid_generic_deinit(usb_device_composite_struct_t *device_composite,uint32_t hidInfoIndex);
#endif // #if BL_CONFIG_USB_HID

#if (BL_CONFIG_USB_MSC || BL_CONFIG_HS_USB_MSC)
// MSC disk device initialization function
extern usb_status_t usb_device_msc_disk_init(usb_device_composite_struct_t *device_composite);
// MSC class specific callback function
extern usb_status_t usb_device_msc_callback(class_handle_t handle, uint32_t event, void *param);
// MSC disk device set configuration function
extern usb_status_t usb_device_msc_disk_set_configure(class_handle_t handle, uint8_t configure);
// MSC disk device de-initialization function
extern usb_status_t usb_device_msc_disk_deinit(usb_device_composite_struct_t *device_composite);

void usb_device_msc_disk_pump(void);
#endif // #if BL_CONFIG_USB_MSC

#if (BL_CONFIG_USB_DFU || BL_CONFIG_HS_USB_DFU)
// DFU class specific callback function
extern usb_status_t usb_device_dfu_callback(class_handle_t handle, uint32_t event, void *param);

extern usb_status_t usb_device_dfu_vendor_callback(usb_device_handle handle, void *param);
extern void usb_device_dfu_bus_reset(void);
extern void usb_device_dfu_switch_mode(void);

extern void usb_device_dfu_init(usb_device_composite_struct_t *device_composite);

void usb_device_dfu_downloader_pump(void);

void usb_device_dfu_mem_read(uint8_t *dest, uint32_t size);
#endif //

extern usb_device_class_config_list_struct_t g_composite_device_config_list[2];
extern int32_t find_controller_struct_index(uint32_t id);
/*******************************************************************************
 * API
 ******************************************************************************/

#endif /* __USB_DEVICE_COMPOSITE_H__ */
