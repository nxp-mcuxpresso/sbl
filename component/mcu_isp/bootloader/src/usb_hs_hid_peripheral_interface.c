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
#include "bootloader_core.h"
#include "usb_class_hid.h"
#include "usb_device_stack_interface.h"
#include "usb_hid.h"
#include "bootloader_hid_report_ids.h"
#include "usb_error.h"
#include "utilities/fsl_rtos_abstraction.h"
#include <string.h>
#include "property/property.h"
#include "usb_descriptor.h"
#include "usb_class_hid.h"

#if defined(K65F18_SERIES)
#include "MK65F18_INV.h"
#endif

#if (BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID)

//! @addtogroup usb_hid_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#if !defined USBHS_INSTANCE_COUNT
#define USBHS_INSTANCE_COUNT (0)
#endif

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
    hid_handle_t app_handle;
} usb_hid_packetizer_info_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
static void usb_hid_app_callback(uint8_t event_type, void *val, void *arg);
static uint8_t usb_hid_app_param_callback(
    uint8_t request, uint16_t value, uint8_t **data, USB_PACKET_SIZE *size, void *arg);

extern usb_desc_request_notify_struct_t g_desc_callback;

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
static void usb_hid_Periodic_Task(const peripheral_descriptor_t *self);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const peripheral_control_interface_t g_usbHidControlInterface = {.pollForActivity = usb_hid_poll_for_activity,
                                                                 .init = usb_hid_full_init,
                                                                 .shutdown = usb_hid_full_shutdown,
                                                                 .pump = usb_hid_Periodic_Task };

const peripheral_packet_interface_t g_usbHidPacketInterface = {.init = usb_hid_packet_init,
                                                               .readPacket = usb_hid_packet_read,
                                                               .writePacket = usb_hid_packet_write,
                                                               .abortDataPhase = usb_hid_packet_abort_data_phase,
                                                               .finalize = usb_hid_packet_finalize,
                                                               .getMaxPacketSize = usb_hid_packet_get_max_packet_size,
                                                               .byteReceivedCallback = 0 };

//! @brief Current state of the USB HID packetizer.
static usb_hid_packetizer_info_t s_hidInfo[USB_INSTANCE_COUNT + USBHS_INSTANCE_COUNT];
// static usb_hid_packetizer_info_t s_hidInfo;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

uint32_t usb_get_instance_via_ipsr(void)
{
#if (USBHS_INSTANCE_COUNT == 0)
    return USB_CONTROLLER_KHCI_0;
#else
    uint32_t vectorNum;

#if defined(__ICCARM__) // IAR assembler
    __asm(
        "mrs r0, ipsr\n"
        "mov %0, r0"
        : "=r"(vectorNum));
#elif defined(__CC_ARM) // ARM assembler
    __asm(
        "mrs r0, ipsr\n"
        "mov vectorNum, r0");
#elif defined(__GNUC__) // GNU assembler
    __asm("mrs %0, ipsr" : "=r"(vectorNum));
#else
#error "Unsupported assembler"
#endif

    return (vectorNum == INT_USB0) ? USB_CONTROLLER_KHCI_0 : USB_CONTROLLER_EHCI_0;
#endif
}

bool usb_hid_poll_for_activity(const peripheral_descriptor_t *self)
{
    uint32_t hidInfoIndex = self->instance / 2;
    return s_hidInfo[hidInfoIndex].isEnumerated && s_hidInfo[hidInfoIndex].didReceiveFirstReport;
}

/*!
 * @brief This function handles the callback
 *
 * This function is called from the class layer whenever reset occurs or enum
 * is complete. after the enum is complete this function sets a variable so
 * that the application can start
 *
 * @param event_type     value of the event
 * @param val            gives the configuration value
 * @param arg
 */
void usb_hid_app_callback(uint8_t event_type, void *val, void *arg)
{
    UNUSED_ARGUMENT(arg)
    UNUSED_ARGUMENT(val)

    uint32_t hidInfoIndex = usb_get_instance_via_ipsr() / 2;

    if ((event_type == USB_DEV_EVENT_BUS_RESET) || (event_type == USB_DEV_EVENT_CONFIG_CHANGED))
    {
        s_hidInfo[hidInfoIndex].isEnumerated = false;
        s_hidInfo[hidInfoIndex].didReceiveFirstReport = false;
    }
    else if (event_type == USB_DEV_EVENT_ENUM_COMPLETE)
    {
        // Enumeration is complete.
        s_hidInfo[hidInfoIndex].isEnumerated = true;
        USB_Class_HID_Recv_Data(s_hidInfo[hidInfoIndex].app_handle, HID_OUT_ENDPOINT,
                                (uint8_t *)&s_hidInfo[hidInfoIndex].report.header,
                                sizeof(s_hidInfo[hidInfoIndex].report));
    }
    else if (event_type == USB_DEV_EVENT_ERROR)
    {
        /* user may add code here for error handling
           NOTE : val has the value of error from h/w*/
        return;
    }

    return;
}

/******************************************************************************
 *
 *    @name        usb_hid_app_param_callback
 *
 *    @brief       This function handles the callback for Get/Set report req
 *
 * This function is called whenever a HID class request is received. This
 * function handles these class requests
 *    @param       request  :  request type
 *    @param       value    :  give report type and id
 *    @param       data     :  pointer to the data
 *    @param       size     :  size of the transfer
 *
 *    @return      status
 *                  USB_OK  :  if successful
 *                  else return error
 *
 *****************************************************************************/
uint8_t usb_hid_app_param_callback(uint8_t request, uint16_t value, uint8_t **data, USB_PACKET_SIZE *size, void *arg)
{
    UNUSED_ARGUMENT(arg)

    uint8_t status = USB_OK;
    uint8_t index = (uint8_t)((request - 2) & USB_HID_REQUEST_TYPE_MASK);
    USB_PACKET_SIZE incomingSize = *size;
    const bl_hid_header_t *header;

    uint32_t hidInfoIndex = usb_get_instance_via_ipsr() / 2;

    if (request == USB_DEV_EVENT_SEND_COMPLETE)
    {
        usb_event_struct_t *event = (usb_event_struct_t *)data;
        if ((event->len == 0xFFFFFFFF) || (event->len == 0x0))
        {
            // Transfer failure
            return status;
        }

        // Protect against extra complete due to data abort.
        if (s_hidInfo[hidInfoIndex].sendSync > 0)
        {
            // Signal that write has completed.
            sync_signal(&s_hidInfo[hidInfoIndex].sendSync);
        }
        return status;
    }
    else if (request == USB_DEV_EVENT_DATA_RECEIVED)
    {
        // Save the report size.
        usb_event_struct_t *event = (usb_event_struct_t *)data;
        if ((event->len == 0xFFFFFFFF) || (event->len == 0x0))
        {
            // Transfer failure
            return status;
        }

        s_hidInfo[hidInfoIndex].reportSize = event->len;

        s_hidInfo[hidInfoIndex].didReceiveFirstReport = true;

        // Wake up the read packet handler.
        sync_signal(&s_hidInfo[hidInfoIndex].receiveSync);
        return status;
    }

    // Set default return size of 0.
    *size = 0;

    // handle the class request
    switch (request)
    {
        case USB_HID_GET_REPORT_REQUEST:
            // Send an empty report. The host should only be reading reports via the interrupt pipe.
            memset(&s_hidInfo[hidInfoIndex].report, 0, sizeof(s_hidInfo[hidInfoIndex].report));
            *data = (uint8_t *)&s_hidInfo[hidInfoIndex].report.header; // point to the report to send
            // size is set to 0 above
            break;

        case USB_HID_SET_REPORT_REQUEST:
            // Check for data phase abort packet.
            header = (bl_hid_header_t *)*data;
            if ((header->packetLengthLsb == 0) && (header->packetLengthMsb == 0) &&
                (header->reportID == kBootloaderReportID_CommandOut))
            {
                s_hidInfo[hidInfoIndex].didReceiveDataPhaseAbort = true;
                break;
            }

            // Copy the report data into our local buffer.
            memcpy(&s_hidInfo[hidInfoIndex].report.header, *data, incomingSize);

            // Save the report size.
            s_hidInfo[hidInfoIndex].reportSize = incomingSize;

            // Remember that we received a report.
            s_hidInfo[hidInfoIndex].didReceiveFirstReport = true;

            // Wake up the read packet handler.
            sync_signal(&s_hidInfo[hidInfoIndex].receiveSync);
            break;

        case USB_HID_GET_IDLE_REQUEST:
            // point to the current idle rate
            *data = &s_hidInfo[hidInfoIndex].appRequestParams[index];
            *size = REQ_DATA_SIZE;
            break;

        case USB_HID_SET_IDLE_REQUEST:
            // set the idle rate sent by the host
            s_hidInfo[hidInfoIndex].appRequestParams[index] = (uint8_t)((value & MSB_MASK) >> HIGH_BYTE_SHIFT);
            break;

        case USB_HID_GET_PROTOCOL_REQUEST:
            // point to the current protocol code
            //  0 = Boot Protocol
            //  1 = Report Protocol
            *data = &s_hidInfo[hidInfoIndex].appRequestParams[index];
            *size = REQ_DATA_SIZE;
            break;

        case USB_HID_SET_PROTOCOL_REQUEST:
            // set the protocol sent by the host
            //      0 = Boot Protocol
            //      1 = Report Protocol
            s_hidInfo[hidInfoIndex].appRequestParams[index] = (uint8_t)(value);
            break;
        default:
            break;
    }

    return status;
}

status_t usb_hid_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    // Not used for USB
    (void)function;

    uint32_t hidInfoIndex = self->instance / 2;

    // Init the state info.
    memset(&s_hidInfo[hidInfoIndex], 0, sizeof(s_hidInfo[hidInfoIndex]));

    // Init the usb clock, if usb clock cannot be enabled, clear corresponding bit even it is enabled by user.
    if (!usb_clock_init())
    {
        g_bootloaderContext.propertyInterface->store->configurationData.enabledPeripherals &= ~kPeripheralType_USB_HID;
        g_bootloaderContext.propertyInterface->store->availablePeripherals &= ~kPeripheralType_USB_HID;
        return kStatus_Fail;
    }

    if (self->instance == USB_CONTROLLER_KHCI_0)
    {
        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USB0_IRQn);

        // Enable interrupts from USB module
        NVIC_EnableIRQ(USB0_IRQn);
    }
#if USBCFG_DEV_EHCI
    else if (self->instance == USB_CONTROLLER_EHCI_0)
    {
        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USBHS_IRQn);

        // Enable interrupts from USB module
        NVIC_EnableIRQ(USBHS_IRQn);
    }
#endif
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
    hid_config_struct_t hidConfig;
    hidConfig.hid_application_callback.callback = usb_hid_app_callback;
    hidConfig.hid_application_callback.arg = &s_hidInfo[hidInfoIndex].app_handle;
    hidConfig.class_specific_callback.callback = usb_hid_app_param_callback;
    hidConfig.class_specific_callback.arg = &s_hidInfo[hidInfoIndex].app_handle;
    hidConfig.desc_callback_ptr = &g_desc_callback;

    uint8_t status = USB_Class_HID_Init(self->instance, &hidConfig, &s_hidInfo[hidInfoIndex].app_handle);

    return kStatus_Success;
}

void usb_hid_full_shutdown(const peripheral_descriptor_t *self)
{
    uint32_t hidInfoIndex = self->instance / 2;

// Make sure we are clocking to the peripheral to ensure there
// are no bus errors
#if defined(PCC_BASE_ADDRS)
    if ((self->instance == USB_CONTROLLER_KHCI_0) && PCC_RD_CLKCFG_CGC(PCC0, (PCC_USB0FS_INDEX & PCC_PERIPHERAL_MASK)))
    {
        // Disable the USB interrupt
        NVIC_DisableIRQ(USB0_IRQn);

        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USB0_IRQn);

        // Shutdown class driver
        USB_Class_HID_Deinit(s_hidInfo[hidInfoIndex].app_handle);

        // Turn off clocking to USB
        PCC_WR_CLKCFG_CGC(PCC0, (PCC_USB0FS_INDEX & PCC_PERIPHERAL_MASK), 0);
    }
#else
    if ((self->instance == USB_CONTROLLER_KHCI_0) && (SIM_SCGC4 & SIM_SCGC4_USBOTG_MASK))
    {
        // Disable the USB interrupt
        NVIC_DisableIRQ(USB0_IRQn);

        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USB0_IRQn);

        // Shutdown class driver
        USB_Class_HID_Deinit(s_hidInfo[hidInfoIndex].app_handle);

        // Turn off clocking to USB
        SIM_SCGC4 &= ~SIM_SCGC4_USBOTG_MASK;
    }
#endif // defined(PCC_BASE_ADDRS)
#if USBCFG_DEV_EHCI
    else if ((self->instance == USB_CONTROLLER_EHCI_0) && (SIM_SCGC3 & SIM_SCGC3_USBHS_MASK))
    {
        // Disable the USB interrupt
        NVIC_DisableIRQ(USBHS_IRQn);

        // Clear any pending interrupts on USB
        NVIC_ClearPendingIRQ(USBHS_IRQn);

        // Shutdown class driver
        USB_Class_HID_Deinit(s_hidInfo[hidInfoIndex].app_handle);

        // Turn off HS USB PHY clock gate
        SIM_SCGC3 &= ~(SIM_SCGC3_USBHS_MASK | SIM_SCGC3_USBHSPHY_MASK);
    }
#endif
}

static status_t usb_hid_packet_init(const peripheral_descriptor_t *self)
{
    uint32_t hidInfoIndex = self->instance / 2;

    sync_init(&s_hidInfo[hidInfoIndex].receiveSync, false);
    sync_init(&s_hidInfo[hidInfoIndex].sendSync, false);

    // Check for any received data that may be pending
    sync_signal(&s_hidInfo[hidInfoIndex].receiveSync);

    return kStatus_Success;
}

static status_t usb_hid_packet_read(const peripheral_descriptor_t *self,
                                    uint8_t **packet,
                                    uint32_t *packetLength,
                                    packet_type_t packetType)
{
    uint32_t hidInfoIndex = self->instance / 2;

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
    if (s_hidInfo[hidInfoIndex].isReceiveDataRequestRequired)
    {
        // Initiate receive on interrupt out pipe.
        USB_Class_HID_Recv_Data(s_hidInfo[hidInfoIndex].app_handle, HID_OUT_ENDPOINT,
                                (uint8_t *)&s_hidInfo[hidInfoIndex].report.header,
                                sizeof(s_hidInfo[hidInfoIndex].report));
    }
    s_hidInfo[hidInfoIndex].isReceiveDataRequestRequired = true;

    // Wait until we have received a report.
    sync_wait(&s_hidInfo[hidInfoIndex].receiveSync, kSyncWaitForever);

    // Check the report ID, the first byte of the report buffer.
    if (s_hidInfo[hidInfoIndex].report.header.reportID != reportID)
    {
        // If waiting for a command but get data, this is a flush after a data abort.
        if ((reportID == kBootloaderReportID_CommandOut) &&
            (s_hidInfo[hidInfoIndex].report.header.reportID == kBootloaderReportID_DataOut))
        {
            return kStatus_AbortDataPhase;
        }
        debug_printf("usbhid: received unexpected report=%x\r\n", s_hidInfo[hidInfoIndex].report.header.reportID);
        return kStatus_Fail;
    }

    // Extract the packet length encoded as bytes 1 and 2 of the report. The packet length
    // is transferred in little endian byte order.
    uint16_t lengthOfPacket = s_hidInfo[hidInfoIndex].report.header.packetLengthLsb |
                              (s_hidInfo[hidInfoIndex].report.header.packetLengthMsb << 8);

    // Make sure we got all of the packet. Some hosts (Windows) may send up to the maximum
    // report size, so there may be extra trailing bytes.
    if ((s_hidInfo[hidInfoIndex].reportSize - sizeof(s_hidInfo[hidInfoIndex].report.header)) < lengthOfPacket)
    {
        debug_printf("usbhid: received only %d bytes of packet with length %d\r\n",
                     s_hidInfo[hidInfoIndex].reportSize - 3, lengthOfPacket);
        return kStatus_Fail;
    }

    // Return packet to caller.
    *packet = s_hidInfo[hidInfoIndex].report.packet;
    *packetLength = lengthOfPacket;

    return kStatus_Success;
}

static status_t usb_hid_packet_write(const peripheral_descriptor_t *self,
                                     const uint8_t *packet,
                                     uint32_t byteCount,
                                     packet_type_t packetType)
{
    uint32_t hidInfoIndex = self->instance / 2;

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
    if (s_hidInfo[hidInfoIndex].didReceiveDataPhaseAbort)
    {
        s_hidInfo[hidInfoIndex].didReceiveDataPhaseAbort = false;
        lock_release();
        return kStatus_AbortDataPhase;
    }
    lock_release();

    // Construct report contents.
    s_hidInfo[hidInfoIndex].report.header.reportID = reportID;
    s_hidInfo[hidInfoIndex].report.header._padding = 0;
    s_hidInfo[hidInfoIndex].report.header.packetLengthLsb = byteCount & 0xff;
    s_hidInfo[hidInfoIndex].report.header.packetLengthMsb = (byteCount >> 8) & 0xff;
    if (packet && byteCount > 0)
    {
        memcpy(&s_hidInfo[hidInfoIndex].report.packet, packet, byteCount);
    }

    // Send the maximum report size since that's what the host expects.
    // There may be extra trailing bytes.
    USB_Class_HID_Send_Data(s_hidInfo[hidInfoIndex].app_handle, HID_IN_ENDPOINT,
                            (uint8_t *)&s_hidInfo[hidInfoIndex].report.header, sizeof(s_hidInfo[hidInfoIndex].report));
    sync_wait(&s_hidInfo[hidInfoIndex].sendSync, kSyncWaitForever);
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

/**************************************************************************/ /*!
  *
  * @name   USB_Class_Periodic_Task
  *
  * @brief  The function calls for periodic tasks
  *
  * @param  None
  *
  * @return None
  ******************************************************************************
  * Called to check for any pending requests
  *****************************************************************************/
void usb_hid_Periodic_Task(const peripheral_descriptor_t *self)
{
    USB_HID_Periodic_Task();
}

//! @}

#endif // BL_CONFIG_USB_HID

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
