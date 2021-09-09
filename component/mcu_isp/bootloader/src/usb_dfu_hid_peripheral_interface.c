/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "bootloader/bootloader.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "bootloader_core.h"
#if (!BL_FEATURE_MIN_PROFILE) || (!BL_FEATURE_HAS_NO_RECEIVE_SB_FILE)
#include "sbloader/sbloader.h"
#include "sbloader/sb_file_format.h"
#endif
#include "bootloader_hid_report_ids.h"
#include "utilities/fsl_rtos_abstraction.h"
#include <string.h>
#include "property/property.h"

//#include "usb_ksdk/api_usb_ksdk.h"
#include "usb_device_descriptor.h"
#include "composite.h"
//#include "dfu_downloader.h"
#if BL_FEATURE_PUT_BCA_IN_FUSE
#include "fusemap.h"
#endif

#include <stdio.h>
#include <stdlib.h>

#if (BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID || BL_CONFIG_USB_DFU || BL_CONFIG_HS_USB_DFU)

//! @addtogroup usb_dfu_hid_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#define REQ_DATA_SIZE (1)

#define USB_DFU_INDEX (0)
#define USB_HID_INDEX (1)

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static bool usb_hid_poll_for_activity(const peripheral_descriptor_t *self);
static status_t usb_device_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
static void usb_device_full_shutdown(const peripheral_descriptor_t *self);
static void usb_dfu_pump(const peripheral_descriptor_t *self);

static status_t usb_hid_packet_init(const peripheral_descriptor_t *self);
static status_t usb_hid_packet_read(const peripheral_descriptor_t *self,
                                    uint8_t **packet,
                                    uint32_t *packetLength,
                                    packet_type_t packetType);
static status_t usb_hid_packet_write(const peripheral_descriptor_t *self,
                                     const uint8_t *packet,
                                     uint32_t byteCount,
                                     packet_type_t packetType);
static void usb_hid_packet_abort_data_phase(const peripheral_descriptor_t *self);
static status_t usb_hid_packet_finalize(const peripheral_descriptor_t *self);
static uint32_t usb_hid_packet_get_max_packet_size(const peripheral_descriptor_t *self);

static bool s_dDfuHidActivity[2] = { false };
static bool s_dDfuHidInitialized = false;

#if BL_FEATURE_PUT_BCA_IN_FUSE || BL_FEATURE_PUT_BCA_IN_IFR
extern void usb_update_serial_number_str_desc_according_to_uuid(void);
extern void usb_get_usbid_from_fuse(uint16_t *vid, uint16_t *pid);
#endif
#if BL_FEATURE_PUT_BCA_IN_IFR
extern void usb_get_usbid_from_ifr(uint16_t *vid, uint16_t *pid);
#endif

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const peripheral_control_interface_t g_usbHidControlInterface = {.pollForActivity = usb_hid_poll_for_activity,
                                                                 .init = usb_device_full_init,
                                                                 .shutdown = usb_device_full_shutdown,
                                                                 .pump = usb_dfu_pump };

const peripheral_packet_interface_t g_usbHidPacketInterface = {.init = usb_hid_packet_init,
                                                               .readPacket = usb_hid_packet_read,
                                                               .writePacket = usb_hid_packet_write,
                                                               .abortDataPhase = usb_hid_packet_abort_data_phase,
                                                               .finalize = usb_hid_packet_finalize,
                                                               .getMaxPacketSize = usb_hid_packet_get_max_packet_size,
                                                               .byteReceivedCallback = 0 };

usb_device_composite_struct_t g_device_composite BL_SECTION(".dfuBuffer");

usb_status_t usb_device_callback(usb_device_handle handle, uint32_t event, void *param);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

bool usb_hid_poll_for_activity(const peripheral_descriptor_t *self)
{
    bool hid_active = false;
    bool dfu_active = false;
#if USB_DEVICE_CONFIG_HID
    hid_active = g_device_composite.hid_generic.hid_packet.didReceiveFirstReport;
#endif //  USB_DEVICE_CONFIG_HID
#if USB_DEVICE_CONFIG_DFU
    usb_device_dfu_downloader_pump();
    dfu_active = g_device_composite.dfu_downloader.isDfuRequestDetached;
#endif //  USB_DEVICE_CONFIG_DFU

    s_dDfuHidActivity[USB_HID_INDEX] = hid_active;
    s_dDfuHidActivity[USB_DFU_INDEX] = dfu_active;

    return (g_device_composite.attach && (hid_active || dfu_active));
}

usb_status_t usb_device_callback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_Error;
    uint16_t *temp16 = (uint16_t *)param;
    uint8_t *temp8 = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
        {
            /* USB bus reset signal detected */
            g_device_composite.attach = 0U;
            error = kStatus_USB_Success;
#if (BL_CONFIG_USB_DFU || BL_CONFIG_HS_USB_DFU)
            usb_device_dfu_bus_reset();
#endif
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &g_device_composite.speed))
            {
                usb_device_set_speed(handle, g_device_composite.speed);
            }
#endif
        }
        break;

        case kUSB_DeviceEventSetConfiguration:
            if (param)
            {
                /* Set device configuration request */
                g_device_composite.attach = 1U;
                error = kStatus_USB_Success;
#if USB_DEVICE_CONFIG_HID
                g_device_composite.current_configuration = *temp8;
                usb_device_hid_generic_set_configure(g_device_composite.hid_generic.hid_handle, *temp8);
#endif
            }
            break;

        case kUSB_DeviceEventSetInterface:
            if (g_device_composite.attach)
            {
                /* Set device interface request */
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternate_setting = (uint8_t)(*temp16 & 0x00FFU);
                if (interface < USB_COMPOSITE_INTERFACE_COUNT)
                {
                    g_device_composite.current_interface_alternate_setting[interface] = alternate_setting;
#if USB_DEVICE_CONFIG_HID
                    usb_device_hid_generic_set_interface(g_device_composite.hid_generic.hid_handle, interface,
                                                         alternate_setting);
#endif
                    error = kStatus_USB_Success;
                }
            }
            break;

        case kUSB_DeviceEventGetConfiguration:
            if (param)
            {
                /* Get current configuration request */
                *temp8 = g_device_composite.current_configuration;
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceEventGetInterface:
            if (param)
            {
                /* Get current alternate setting of the interface request */
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                if (interface < USB_COMPOSITE_INTERFACE_COUNT)
                {
                    *temp16 = (*temp16 & 0xFF00U) | g_device_composite.current_interface_alternate_setting[interface];
                    error = kStatus_USB_Success;
                }
                else
                {
                    error = kStatus_USB_InvalidRequest;
                }
            }
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param)
            {
                /* Get device descriptor request */
                error = usb_device_get_device_descriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param)
            {
                /* Get device configuration descriptor request */
                error = usb_device_get_configuration_descriptor(
                    handle, (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetStringDescriptor:
            if (param)
            {
                /* Get device string descriptor request */
                error = usb_device_get_string_descriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetHidDescriptor:
            if (param)
            {
                /* Get hid descriptor request */
                error = usb_device_get_hid_descriptor(handle, (usb_device_get_hid_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetHidReportDescriptor:
            if (param)
            {
                /* Get hid report descriptor request */
                error = usb_device_get_hid_report_descriptor(handle,
                                                             (usb_device_get_hid_report_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetHidPhysicalDescriptor:
            if (param)
            {
                /* Get hid physical descriptor request */
                error = usb_device_get_hid_physical_descriptor(
                    handle, (usb_device_get_hid_physical_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventVendorRequest:
        {
/* Handle DFU vendor request */
#if USB_DFU_CONFIG_WCID
            USB_DeviceGetVerdorDescriptor(handle, param);
#endif
        }
        break;
        default:
            break;
    }
    return error;
}

status_t usb_device_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    // Not used for USB
    (void)function;

    /* Install isr, set priority, and enable IRQ. */
    uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
    uint8_t usbDeviceIP3511Irq[] = USB_IRQS;
    irqNumber = usbDeviceIP3511Irq[CONTROLLER_ID - kUSB_ControllerLpcIp3511Fs0];
#elif defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
    uint8_t usbDeviceIP3511Irq[] = USBHSD_IRQS;
    irqNumber = usbDeviceIP3511Irq[CONTROLLER_ID - kUSB_ControllerLpcIp3511Hs0];
#endif

    // Init the state info.
    memset_s(&g_device_composite, sizeof(g_device_composite), 0, sizeof(g_device_composite));

    // Init the usb clock, if usb clock cannot be enabled, clear corresponding bit even it is enabled by user.
    if (!usb_clock_init(0))
    {
        g_bootloaderContext.propertyInterface->store->configurationData.enabledPeripherals &=
            ~(kPeripheralType_USB_HID | kPeripheralType_USB_DFU);
        g_bootloaderContext.propertyInterface->store->availablePeripherals &=
            ~(kPeripheralType_USB_HID | kPeripheralType_USB_DFU);
        return kStatus_Fail;
    }

    uint16_t usbPid = 0;
    uint16_t usbVid = 0;
#if BL_FEATURE_PUT_BCA_IN_FUSE
    // For customers who plan to use USB ISP in end product to get the production
    // firmware they would need to use their own USB-IF approved vendor and product
    // IDs. These IDs are provided through OTP fuses.
    usb_get_usbid_from_fuse(&usbVid, &usbPid);
#elif BL_FEATURE_PUT_BCA_IN_IFR
    usb_get_usbid_from_ifr(&usbVid, &usbPid);
#else
    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;
    usbPid = propertyStore->configurationData.usbPid;
    usbVid = propertyStore->configurationData.usbVid;
#endif
    if ((usbPid != (uint16_t)0xFFFF) && (usbVid != (uint16_t)0xFFFF) && (usbPid != 0) && (usbVid != 0))
    {
        g_device_descriptor[kUsbDescriptorIndex_VidLow] = (usbVid & 0xFF);
        g_device_descriptor[kUsbDescriptorIndex_VidHigh] = (usbVid & 0xFF00) >> 8;
        g_device_descriptor[kUsbDescriptorIndex_PidLow] = (usbPid & 0xFF);
        g_device_descriptor[kUsbDescriptorIndex_PidHigh] = (usbPid & 0xFF00) >> 8;
    }

#if BL_FEATURE_PUT_BCA_IN_FUSE || BL_FEATURE_PUT_BCA_IN_IFR
    usb_update_serial_number_str_desc_according_to_uuid();
#else
    if (propertyStore->configurationData.usbStringsPointer != 0xFFFFFFFF)
    {
        g_language_ptr = (usb_language_list_t *)propertyStore->configurationData.usbStringsPointer;
    }
    else
#endif
    {
        g_language_ptr = &g_language_list;
    }

    /* Set composite device to default state */
    g_device_composite.speed = USB_SPEED_FULL;
    g_device_composite.attach = 0U;
#if USB_DEVICE_CONFIG_HID
    g_device_composite.hid_generic.hid_handle = (class_handle_t)NULL;
#endif
    g_device_composite.device_handle = NULL;
    if (kStatus_USB_Success !=
        USB_DeviceClassInit(CONTROLLER_ID, &g_composite_device_config_list[0], &g_device_composite.device_handle))
    {
        return kStatus_Fail;
    }
    else
    {
#if USB_DEVICE_CONFIG_HID
#if USB_DEVICE_CONFIG_DFU
        g_device_composite.hid_generic.hid_handle = g_composite_device_config_list.config[1].classHandle;
#else
        g_device_composite.hid_generic.hid_handle = g_composite_device_config_list[0].config[0].classHandle;
#endif
        usb_device_hid_generic_init(&g_device_composite, 0);
#endif
#if USB_DEVICE_CONFIG_DFU
        usb_device_dfu_init(&g_device_composite);
#endif
    }

/* Install isr, set priority, and enable IRQ. */
#if defined(__GIC_PRIO_BITS)
    GIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
#else
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
#endif
    EnableIRQ((IRQn_Type)irqNumber);

    /* Start the device function */
    USB_DeviceRun(g_device_composite.device_handle);

    s_dDfuHidInitialized = true;

    return kStatus_Success;
}

void usb_device_full_shutdown(const peripheral_descriptor_t *self)
{
    if (!s_dDfuHidInitialized)
    {
        return;
    }
    if (kStatus_USB_Success != USB_DeviceClassDeinit(CONTROLLER_ID))
    {
        return;
    }
    else
    {
// Shutdown class driver
#if USB_DEVICE_CONFIG_HID
        usb_device_hid_generic_deinit(&g_device_composite, 0);
#endif // USB_DEVICE_CONFIG_HID
    }

// Make sure we are clocking to the peripheral to ensure there
// are no bus errors
#if USB_DEVICE_CONFIG_LPCIP3511FS
    if ((CONTROLLER_ID == kUSB_ControllerLpcIp3511Fs0))
    {
        // Disable the USB interrupt
        NVIC_DisableIRQ(USB0_IRQn);

        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USB0_IRQn);
    }
    else
#endif
#if USB_DEVICE_CONFIG_LPCIP3511HS
        if ((CONTROLLER_ID == kUSB_ControllerLpcIp3511Hs0))
    {
#if FSL_FEATURE_SOC_USB_COUNT
        // Disable the USB interrupt
        NVIC_DisableIRQ(USB1_IRQn);

        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USB1_IRQn);
#else
        // Disable the USB interrupt
        NVIC_DisableIRQ(USB_IRQ_NUMBER);

        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USB_IRQ_NUMBER);
#endif
    }
    else
#endif
    {
    }
}

//! @brief Run the sbloader state machine.
//!
//! This function is called repeatedly by the main application loop. We use it
//! to run the sbloader state machine from non-interrupt context.
void usb_dfu_pump(const peripheral_descriptor_t *self)
{
    //s_dDfuHidActivity[USB_HID_INDEX] = false;
    //s_dDfuHidActivity[USB_DFU_INDEX] = true;

#if (BL_CONFIG_USB_DFU || BL_CONFIG_HS_USB_DFU)
    usb_device_dfu_downloader_pump();
#endif

    //s_dDfuHidActivity[USB_HID_INDEX] = true;
}

static status_t usb_hid_packet_init(const peripheral_descriptor_t *self)
{
#if USB_DEVICE_CONFIG_HID
    sync_init(&g_device_composite.hid_generic.hid_packet.receiveSync, false);
    sync_init(&g_device_composite.hid_generic.hid_packet.sendSync, false);

    // Check for any received data that may be pending
    sync_signal(&g_device_composite.hid_generic.hid_packet.receiveSync);
#endif
    return kStatus_Success;
}

static status_t usb_hid_packet_read(const peripheral_descriptor_t *self,
                                    uint8_t **packet,
                                    uint32_t *packetLength,
                                    packet_type_t packetType)
{
    if (!packet || !packetLength)
    {
        //        debug_printf("Error: invalid packet\r\n");
        return kStatus_InvalidArgument;
    }
    *packetLength = 0;

    // Determine report ID based on packet type.
    uint8_t reportID;
    switch (packetType)
    {
        case kPacketType_Command:
            reportID = kBootloaderReportID_CommandOut;
            break;
        case kPacketType_Data:
            reportID = kBootloaderReportID_DataOut;
            break;
        default:
            //            debug_printf("usbhid: unsupported packet type %d\r\n", (int)packetType);
            return kStatus_Fail;
    };
#if USB_DEVICE_CONFIG_HID
    if (s_dDfuHidActivity[USB_HID_INDEX])
    {
        // The first receive data request was initiated after enumeration.
        // After that we wait until we are ready to read data before
        // we request more. This mechanism prevents data loss
        // by allowing the USB controller to hold off the host with NAKs
        // on the interrupt out pipe until we are ready.
        if (g_device_composite.hid_generic.hid_packet.isReceiveDataRequestRequired)
        {
            // Initiate receive on interrupt out pipe.
            USB_DeviceHidRecv(g_device_composite.hid_generic.hid_handle, USB_HID_GENERIC_ENDPOINT_OUT,
                              (uint8_t *)&g_device_composite.hid_generic.hid_packet.report.header,
                              sizeof(g_device_composite.hid_generic.hid_packet.report));
        }

        g_device_composite.hid_generic.hid_packet.isReceiveDataRequestRequired = true;

        // Wait until we have received a report.

        sync_wait(&g_device_composite.hid_generic.hid_packet.receiveSync, kSyncWaitForever);

        // Check the report ID, the first byte of the report buffer.
        if (g_device_composite.hid_generic.hid_packet.report.header.reportID != reportID)
        {
            // If waiting for a command but get data, this is a flush after a data abort.
            if ((reportID == kBootloaderReportID_CommandOut) &&
                (g_device_composite.hid_generic.hid_packet.report.header.reportID == kBootloaderReportID_DataOut))
            {
                return kStatus_AbortDataPhase;
            }
            //        debug_printf("usbhid: received unexpected report=%x\r\n",
            //        g_device_composite.hid_generic.hid_packet.report.header.reportID);
            return kStatus_Fail;
        }

        // Extract the packet length encoded as bytes 1 and 2 of the report. The packet length
        // is transferred in little endian byte order.
        uint16_t lengthOfPacket = g_device_composite.hid_generic.hid_packet.report.header.packetLengthLsb |
                                  (g_device_composite.hid_generic.hid_packet.report.header.packetLengthMsb << 8);

        // Make sure we got all of the packet. Some hosts (Windows) may send up to the maximum
        // report size, so there may be extra trailing bytes.
        if ((g_device_composite.hid_generic.hid_packet.reportSize -
             sizeof(g_device_composite.hid_generic.hid_packet.report.header)) < lengthOfPacket)
        {
            //        debug_printf("usbhid: received only %d bytes of packet with length %d\r\n",
            //        s_hidInfo[hidInfoIndex].reportSize - 3, lengthOfPacket);
            return kStatus_Fail;
        }

        // Return packet to caller.
        *packet = g_device_composite.hid_generic.hid_packet.report.packet;
        *packetLength = lengthOfPacket;
    }
#endif // USB_DEVICE_CONFIG_HID
    return kStatus_Success;
}

static status_t usb_hid_packet_write(const peripheral_descriptor_t *self,
                                     const uint8_t *packet,
                                     uint32_t byteCount,
                                     packet_type_t packetType)
{
#if USB_DEVICE_CONFIG_HID
    if (s_dDfuHidActivity[USB_HID_INDEX])
    {
        if (byteCount > kMinUsbHidPacketBufferSize)
        {
            debug_printf("Error: invalid packet size %d\r\n", byteCount);
            return kStatus_InvalidArgument;
        }

        // Determine report ID based on packet type.
        uint8_t reportID;
        switch (packetType)
        {
            case kPacketType_Command:
                reportID = kBootloaderReportID_CommandIn;
                break;
            case kPacketType_Data:
                reportID = kBootloaderReportID_DataIn;
                break;
            default:
                debug_printf("usbhid: unsupported packet type %d\r\n", (int)packetType);
                return kStatus_Fail;
        };

        // Check for data phase aborted by receiver.
        lock_acquire();
        if (g_device_composite.hid_generic.hid_packet.didReceiveDataPhaseAbort)
        {
            g_device_composite.hid_generic.hid_packet.didReceiveDataPhaseAbort = false;
            lock_release();
            return kStatus_AbortDataPhase;
        }
        lock_release();

        // Construct report contents.
        g_device_composite.hid_generic.hid_packet.report.header.reportID = reportID;
        g_device_composite.hid_generic.hid_packet.report.header._padding = 0;
        g_device_composite.hid_generic.hid_packet.report.header.packetLengthLsb = byteCount & 0xff;
        g_device_composite.hid_generic.hid_packet.report.header.packetLengthMsb = (byteCount >> 8) & 0xff;
        if (packet && byteCount > 0)
        {
            memcpy(&g_device_composite.hid_generic.hid_packet.report.packet, packet, byteCount);
        }
        if (g_device_composite.hid_generic.attach == 1)
        {
            // Send the maximum report size since that's what the host expects.
            // There may be extra trailing bytes.
            USB_DeviceHidSend(g_device_composite.hid_generic.hid_handle, USB_HID_GENERIC_ENDPOINT_IN,
                              (uint8_t *)&g_device_composite.hid_generic.hid_packet.report.header,
                              sizeof(g_device_composite.hid_generic.hid_packet.report));
            sync_wait(&g_device_composite.hid_generic.hid_packet.sendSync, kSyncWaitForever);
        }
    }
#endif // USB_DEVICE_CONFIG_HID
    return kStatus_Success;
}

static void usb_hid_packet_abort_data_phase(const peripheral_descriptor_t *self)
{
    status_t status = self->packetInterface->writePacket(self, NULL, 0, kPacketType_Command);
    if (status != kStatus_Success)
    {
        debug_printf("Error: usb_hid_packet_abort write packet returned status 0x%x\r\n", status);
        return;
    }
}

static status_t usb_hid_packet_finalize(const peripheral_descriptor_t *self)
{
    return kStatus_Success;
}

static uint32_t usb_hid_packet_get_max_packet_size(const peripheral_descriptor_t *self)
{
    return kMinUsbHidPacketBufferSize;
}

#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0)
#if defined(CPU_MIMXRT595SFFOB_cm33)
void USB0_IRQHandler(void)
#else
void USB_IRQHandler(void)
#endif
{
    USB_DeviceLpcIp3511IsrFunction(g_device_composite.device_handle);
}
#endif

#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0)
#if FSL_FEATURE_SOC_USB_COUNT
void USB1_IRQHandler(void)
#else
#if defined(CPU_MIMXRT595SFFOB_cm33)
void USB0_IRQHandler(void)
#else
void USB_IRQHandler(void)
#endif
#endif
{
    USB_DeviceLpcIp3511IsrFunction(g_device_composite.device_handle);
}
#endif

//! @}

#endif // (BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID || BL_CONFIG_USB_DFU || BL_CONFIG_HS_USB_DFU)

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
