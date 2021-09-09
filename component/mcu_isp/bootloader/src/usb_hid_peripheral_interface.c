/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "bootloader/bootloader.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "usb_dci_kinetis.h"
#include "usb_hid.h"
#include "usb_device/bootloader_hid_report_ids.h"
#include "usb_device/driver/usb_devapi.h"
#include "utilities/fsl_rtos_abstraction.h"
#include <string.h>
#include "property/property.h"
#include "usb_descriptor.h"

#if BL_CONFIG_USB_HID
//! @addtogroup usb_hid_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define REQ_DATA_SIZE (1)

//! @brief Request parameters.
enum
{
    kAppRequestParam_IdleRate = 0,
    kAppRequestParam_Protocol,
    kAppRequestParamCount
};

//! @brief State information for the USB HID packetizer.
typedef struct _usb_hid_packetizer_info
{
    bool isEnumerated;                 //!< Whether the device has enumerated and is configured.
    bool didReceiveFirstReport;        //!< Whether the first report has been received.
    bool didReceiveDataPhaseAbort;     //!< Whether we received a data phase abort request.
    bool isReceiveDataRequestRequired; //!< Whether an interrupt out pipe receive data request is required.
    uint8_t appRequestParams[kAppRequestParamCount]; //!< Storage for request parameter values.
    sync_object_t receiveSync;                       //!< Sync object used for reading packets.
    sync_object_t sendSync;                          //!< Sync object used for sending packets.
    uint32_t reportSize; //!< The size in bytes of a received report. May be greater than the packet contained within
    //! the report plus the header, as the host can send up to the max report size bytes.
    bl_hid_report_t report; //!< Buffer used to hold HID reports for sending and receiving.
} usb_hid_packetizer_info_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static void usb_hid_app_callback(uint8_t controller_ID, uint8_t event_type, void *val);
static uint8_t usb_hid_app_param_callback(
    uint8_t request, uint16_t value, uint16_t wIndex, uint8_t **data, USB_PACKET_SIZE *size);

static bool usb_hid_poll_for_activity(const peripheral_descriptor_t *self);
static status_t usb_hid_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
static void usb_hid_full_shutdown(const peripheral_descriptor_t *self);

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

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const peripheral_control_interface_t g_usbHidControlInterface = {.pollForActivity = usb_hid_poll_for_activity,
                                                                 .init = usb_hid_full_init,
                                                                 .shutdown = usb_hid_full_shutdown,
                                                                 .pump = USB_Class_HID_Periodic_Task };

const peripheral_packet_interface_t g_usbHidPacketInterface = {.init = usb_hid_packet_init,
                                                               .readPacket = usb_hid_packet_read,
                                                               .writePacket = usb_hid_packet_write,
                                                               .abortDataPhase = usb_hid_packet_abort_data_phase,
                                                               .finalize = usb_hid_packet_finalize,
                                                               .getMaxPacketSize = usb_hid_packet_get_max_packet_size,
                                                               .byteReceivedCallback = 0 };

static const IRQn_Type usb_irq_ids[USB_INSTANCE_COUNT] = {
#if BL_HAS_MULTI_CORE
    NVIC0_USB0_IRQn,
#else
    USB0_IRQn,
#endif // #if BL_HAS_MULTI_CORE
};

//! @brief Current state of the USB HID packetizer.
static usb_hid_packetizer_info_t s_hidInfo;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

static void usb_set_clock_gate(uint32_t instance, PeripheralClockSetting set)
{
    uint32_t mask;
    uint32_t *address;

    switch (instance)
    {
#if defined(PCC_BASE_ADDRS)
        case 0:
            mask = PCC_CLKCFGn_CGC_MASK;
            address = (uint32_t *)PCC_ADDR_CLKCFGn(PCC0_BASE, PCC_CLKCFG_ADDR_USB0FS_OFFSET);
            break;
#else
        case 0:
            mask = SIM_SCGC4_USBOTG_MASK;
            address = (uint32_t *)&SIM_SCGC4;
            break;
#endif // #if defined(PCC_BASE_ADDRS)
    }

    if (set == kPeripheralSetClock)
    {
        *address = *address | mask;
    }
    else
    {
        *address = *address & ~mask;
    }
}

bool usb_hid_poll_for_activity(const peripheral_descriptor_t *self)
{
    return s_hidInfo.isEnumerated && s_hidInfo.didReceiveFirstReport;
}

/*!
 * @brief This function handles the callback
 *
 * This function is called from the class layer whenever reset occurs or enum
 * is complete. after the enum is complete this function sets a variable so
 * that the application can start
 *
 * @param controller_ID  Controller ID
 * @param event_type     value of the event
 * @param val            gives the configuration value
 */
void usb_hid_app_callback(uint8_t controller_ID, uint8_t event_type, void *val)
{
    UNUSED(controller_ID);
    UNUSED(val);

    if ((event_type == USB_APP_BUS_RESET) || (event_type == USB_APP_CONFIG_CHANGED))
    {
        s_hidInfo.isEnumerated = false;
    }
    else if (event_type == USB_APP_ENUM_COMPLETE)
    {
        // Enumeration is complete.
        s_hidInfo.isEnumerated = true;

        // Initiate first receive on interrupt out pipe.
        _usb_device_recv_data(CONTROLLER_ID, HID_OUT_ENDPOINT, (uint8_t *)&s_hidInfo.report.header,
                              sizeof(s_hidInfo.report));
    }
    else if (event_type == USB_APP_SEND_COMPLETE)
    {
        // Protect against extra complete due to data abort.
        if (s_hidInfo.sendSync > 0)
        {
            // Signal that write has completed.
            sync_signal(&s_hidInfo.sendSync);
        }
    }
    else if (event_type == USB_APP_DATA_RECEIVED)
    {
        // Save the report size.
        PTR_USB_DEV_EVENT_STRUCT event = (PTR_USB_DEV_EVENT_STRUCT)val;
        s_hidInfo.reportSize = event->len;

        // Remember that we received a report.
        s_hidInfo.didReceiveFirstReport = true;

        // Wake up the read packet handler.
        sync_signal(&s_hidInfo.receiveSync);
    }
}

/*!
 * @brief This function handles callbacks for USB HID Class request
 *
 * This function is called whenever a HID class request is received. This
 * function handles these class requests
 *
 * @param request  request type
 * @param value    give report type and id
 * @param data     pointer to the data
 * @param size     size of the transfer
 *
 * @return      status
 *                 USB_OK  :  if successful
 *                 else return error
 */
uint8_t usb_hid_app_param_callback(
    uint8_t request, uint16_t value, uint16_t wIndex, uint8_t **data, USB_PACKET_SIZE *size)
{
    UNUSED(wIndex);

    uint8_t status = USB_OK;
    uint8_t index = (uint8_t)((request - 2) & USB_HID_REQUEST_TYPE_MASK);
    USB_PACKET_SIZE incomingSize = *size;
    const bl_hid_header_t *header;

    // Set default return size of 0.
    *size = 0;

    // handle the class request
    switch (request)
    {
        case USB_HID_GET_REPORT_REQUEST:
            // Send an empty report. The host should only be reading reports via the interrupt pipe.
            memset(&s_hidInfo.report, 0, sizeof(s_hidInfo.report));
            *data = (uint8_t *)&s_hidInfo.report.header; // point to the report to send
            // size is set to 0 above
            break;

        case USB_HID_SET_REPORT_REQUEST:
            // Check for data phase abort packet.
            header = (bl_hid_header_t *)*data;
            if ((header->packetLengthLsb == 0) && (header->packetLengthMsb == 0) &&
                (header->reportID == kBootloaderReportID_CommandOut))
            {
                s_hidInfo.didReceiveDataPhaseAbort = true;
                break;
            }

            // Copy the report data into our local buffer.
            memcpy(&s_hidInfo.report.header, *data, incomingSize);

            // Save the report size.
            s_hidInfo.reportSize = incomingSize;

            // Remember that we received a report.
            s_hidInfo.didReceiveFirstReport = true;

            // Wake up the read packet handler.
            sync_signal(&s_hidInfo.receiveSync);
            break;

        case USB_HID_GET_IDLE_REQUEST:
            // point to the current idle rate
            *data = &s_hidInfo.appRequestParams[index];
            *size = REQ_DATA_SIZE;
            break;

        case USB_HID_SET_IDLE_REQUEST:
            // set the idle rate sent by the host
            s_hidInfo.appRequestParams[index] = (uint8_t)((value & MSB_MASK) >> HIGH_BYTE_SHIFT);
            break;

        case USB_HID_GET_PROTOCOL_REQUEST:
            // point to the current protocol code
            //  0 = Boot Protocol
            //  1 = Report Protocol
            *data = &s_hidInfo.appRequestParams[index];
            *size = REQ_DATA_SIZE;
            break;

        case USB_HID_SET_PROTOCOL_REQUEST:
            // set the protocol sent by the host
            //      0 = Boot Protocol
            //      1 = Report Protocol
            s_hidInfo.appRequestParams[index] = (uint8_t)(value);
            break;
    }

    return status;
}

status_t usb_hid_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    // Not used for USB
    (void)function;

    // Init the state info.
    memset(&s_hidInfo, 0, sizeof(s_hidInfo));

    // Init the usb clock, if usb clock cannot be enabled, clear corresponding bit even it is enabled by user.
    if (!usb_clock_init())
    {
        g_bootloaderContext.propertyInterface->store->configurationData.enabledPeripherals &= ~kPeripheralType_USB_HID;
        g_bootloaderContext.propertyInterface->store->availablePeripherals &= ~kPeripheralType_USB_HID;
        return kStatus_Fail;
    }

    // Clear any pending interrupts on USB
    NVIC_ClearPendingIRQ(usb_irq_ids[self->instance]);

    // Enable interrupts from USB module
    NVIC_EnableIRQ(usb_irq_ids[self->instance]);

    property_store_t *propertyStore = g_bootloaderContext.propertyInterface->store;

    if ((propertyStore->configurationData.usbPid != (uint16_t)0xFFFF) ||
        (propertyStore->configurationData.usbVid != (uint16_t)0xFFFF))
    {
        g_device_descriptor[kUsbDescriptorIndex_VidLow] = ((propertyStore->configurationData.usbVid) & 0xFF);
        g_device_descriptor[kUsbDescriptorIndex_VidHigh] = ((propertyStore->configurationData.usbVid) & 0xFF00) >> 8;
        g_device_descriptor[kUsbDescriptorIndex_PidLow] = ((propertyStore->configurationData.usbPid) & 0xFF);
        g_device_descriptor[kUsbDescriptorIndex_PidHigh] = ((propertyStore->configurationData.usbPid) & 0xFF00) >> 8;
    }

    if (propertyStore->configurationData.usbStringsPointer != 0xFFFFFFFF)
    {
#if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
        // Make sure the usb string isn't in execute-only region.
        if (is_in_execute_only_region(propertyStore->configurationData.usbStringsPointer,
                                      sizeof(struct _USB_ALL_LANGUAGES)))
        {
            g_lang_ptr = &g_languages;
        }
        else
#endif // #if FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
        {
            g_lang_ptr = (struct _USB_ALL_LANGUAGES *)propertyStore->configurationData.usbStringsPointer;
        }
    }
    else
    {
        g_lang_ptr = &g_languages;
    }
    // Init HID class driver.
    uint8_t status = USB_Class_HID_Init(CONTROLLER_ID, usb_hid_app_callback, NULL, usb_hid_app_param_callback);

    return kStatus_Success;
}

void usb_hid_full_shutdown(const peripheral_descriptor_t *self)
{
    // Make sure we are clocking to the peripheral to ensure there
    // are no bus errors
    usb_set_clock_gate(self->instance, kPeripheralSetClock);

    // Disable the USB interrupt
    NVIC_DisableIRQ(usb_irq_ids[self->instance]);

    // Clear any pending interrupts on USB
    NVIC_ClearPendingIRQ(usb_irq_ids[self->instance]);

    // Shutdown class driver
    USB_Class_HID_DeInit(CONTROLLER_ID);

    usb_set_clock_gate(self->instance, kPeripheralClearClock);
}

static status_t usb_hid_packet_init(const peripheral_descriptor_t *self)
{
    sync_init(&s_hidInfo.receiveSync, false);
    sync_init(&s_hidInfo.sendSync, false);

    // Check for any received data that may be pending
    sync_signal(&s_hidInfo.receiveSync);

    return kStatus_Success;
}

static status_t usb_hid_packet_read(const peripheral_descriptor_t *self,
                                    uint8_t **packet,
                                    uint32_t *packetLength,
                                    packet_type_t packetType)
{
    if (!packet || !packetLength)
    {
        debug_printf("Error: invalid packet\r\n");
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
            debug_printf("usbhid: unsupported packet type %d\r\n", (int)packetType);
            return kStatus_Fail;
    };

    // The first receive data request was initiated after enumeration.
    // After that we wait until we are ready to read data before
    // we request more. This mechanism prevents data loss
    // by allowing the USB controller to hold off the host with NAKs
    // on the interrupt out pipe until we are ready.
    if (s_hidInfo.isReceiveDataRequestRequired)
    {
        // Initiate receive on interrupt out pipe.
        _usb_device_recv_data(CONTROLLER_ID, HID_OUT_ENDPOINT, (uint8_t *)&s_hidInfo.report.header,
                              sizeof(s_hidInfo.report));
    }
    s_hidInfo.isReceiveDataRequestRequired = true;

    // Wait until we have received a report.
    sync_wait(&s_hidInfo.receiveSync, kSyncWaitForever);

    // Check the report ID, the first byte of the report buffer.
    if (s_hidInfo.report.header.reportID != reportID)
    {
        // If waiting for a command but get data, this is a flush after a data abort.
        if ((reportID == kBootloaderReportID_CommandOut) &&
            (s_hidInfo.report.header.reportID == kBootloaderReportID_DataOut))
        {
            return kStatus_AbortDataPhase;
        }
        debug_printf("usbhid: received unexpected report=%x\r\n", s_hidInfo.report.header.reportID);
        return kStatus_Fail;
    }

    // Extract the packet length encoded as bytes 1 and 2 of the report. The packet length
    // is transferred in little endian byte order.
    uint16_t lengthOfPacket = s_hidInfo.report.header.packetLengthLsb | (s_hidInfo.report.header.packetLengthMsb << 8);

    // Make sure we got all of the packet. Some hosts (Windows) may send up to the maximum
    // report size, so there may be extra trailing bytes.
    if ((s_hidInfo.reportSize - sizeof(s_hidInfo.report.header)) < lengthOfPacket)
    {
        debug_printf("usbhid: received only %d bytes of packet with length %d\r\n", s_hidInfo.reportSize - 3,
                     lengthOfPacket);
        return kStatus_Fail;
    }

    // Return packet to caller.
    *packet = s_hidInfo.report.packet;
    *packetLength = lengthOfPacket;

    return kStatus_Success;
}

static status_t usb_hid_packet_write(const peripheral_descriptor_t *self,
                                     const uint8_t *packet,
                                     uint32_t byteCount,
                                     packet_type_t packetType)
{
    if (byteCount > kMinPacketBufferSize)
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
    if (s_hidInfo.didReceiveDataPhaseAbort)
    {
        s_hidInfo.didReceiveDataPhaseAbort = false;
        lock_release();
        return kStatus_AbortDataPhase;
    }
    lock_release();

    // Construct report contents.
    s_hidInfo.report.header.reportID = reportID;
    s_hidInfo.report.header._padding = 0;
    s_hidInfo.report.header.packetLengthLsb = byteCount & 0xff;
    s_hidInfo.report.header.packetLengthMsb = (byteCount >> 8) & 0xff;
    if (packet && byteCount > 0)
    {
        memcpy(&s_hidInfo.report.packet, packet, byteCount);
    }

    // Send the maximum report size since that's what the host expects.
    // There may be extra trailing bytes.
    USB_Class_HID_Send_Data(CONTROLLER_ID, HID_IN_ENDPOINT, (uint8_t *)&s_hidInfo.report.header,
                            sizeof(s_hidInfo.report));
    sync_wait(&s_hidInfo.sendSync, kSyncWaitForever);
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
    return kMinPacketBufferSize;
}

//! @}

#endif // BL_CONFIG_USB_HID
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
