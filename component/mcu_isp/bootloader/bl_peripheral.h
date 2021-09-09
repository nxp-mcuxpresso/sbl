/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _peripheral_h
#define _peripheral_h

#include <stdint.h>
#include "bootloader_common.h"

//! @addtogroup peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

//! @brief Peripheral type bit mask definitions.
//!
//! These bit mask constants serve multiple purposes. They are each a unique value that identifies
//! a peripheral type. They are also the mask for the bits used in the bootloader configuration
//! flash region to list available peripherals and control which peripherals are enabled.
enum _peripheral_types
{
    kPeripheralType_UART = (1 << 0),
    kPeripheralType_I2CSlave = (1 << 1),
    kPeripheralType_SPISlave = (1 << 2),
    kPeripheralType_CAN = (1 << 3),
    kPeripheralType_USB_HID = (1 << 4),
    kPeripheralType_USB_CDC = (1 << 5),
    kPeripheralType_USB_DFU = (1 << 6),
    kPeripheralType_USB_MSC = (1 << 7),
    kPeripheralType_SDIO_SLAVE = (1 << 8),
    kPeripheralType_SAI_SLAVE = (1 << 9),
};

//! @brief Pinmux types.
typedef enum _pinmux_types
{
    kPinmuxType_Default = 0,
    kPinmuxType_PollForActivity = 1,
    kPinmuxType_Peripheral = 2,
    kPinmuxType_RestoreForActivity = 3
} pinmux_type_t;

// Forward declaration.
typedef struct PeripheralDescriptor peripheral_descriptor_t;

typedef void (*serial_byte_receive_func_t)(uint8_t);

//! @brief Peripheral control interface.
typedef struct _peripheral_control_interface
{
    bool (*pollForActivity)(const peripheral_descriptor_t *self);
    status_t (*init)(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
    void (*shutdown)(const peripheral_descriptor_t *self);
    void (*pump)(const peripheral_descriptor_t *self);
} peripheral_control_interface_t;

//! @brief Peripheral abstract byte interface.
typedef struct _peripheral_byte_inteface
{
    status_t (*init)(const peripheral_descriptor_t *self);
    status_t (*read)(const peripheral_descriptor_t *self, uint8_t *buffer, uint32_t requestedBytes, uint32_t timeoutMs);
    status_t (*write)(const peripheral_descriptor_t *self,
                      const uint8_t *buffer,
                      uint32_t byteCount,
                      uint32_t timeoutMs);
} peripheral_byte_inteface_t;

//! @brief Packet types.
typedef enum _packet_type
{
    kPacketType_Command = 0xf0f0, //!< Send or expect a command packet
    kPacketType_Data = 0xaaaa,    //!< Send or expect a data packet
    kPacketType_Invalid = 0x3c3c  // !< Invalid Packet
} packet_type_t;

//! @brief Peripheral Packet Interface.
typedef struct _peripheral_packet_interface
{
    status_t (*init)(const peripheral_descriptor_t *self);
    status_t (*readPacket)(const peripheral_descriptor_t *self,
                           uint8_t **packet,
                           uint32_t *packetLength,
                           packet_type_t packetType);
    status_t (*writePacket)(const peripheral_descriptor_t *self,
                            const uint8_t *packet,
                            uint32_t byteCount,
                            packet_type_t packetType);
    void (*abortDataPhase)(const peripheral_descriptor_t *self);
    status_t (*finalize)(const peripheral_descriptor_t *self);
    uint32_t (*getMaxPacketSize)(const peripheral_descriptor_t *self);
    void (*byteReceivedCallback)(uint8_t byte);
} peripheral_packet_interface_t;

//! @brief Peripheral descriptor.
//!
//! Instances of this struct describe a particular instance of a peripheral that is
//! available for bootloading.
struct PeripheralDescriptor
{
    //! @brief Bit mask identifying the peripheral type.
    //!
    //! See #_peripheral_types for a list of valid bits.
    uint32_t typeMask;

    //! @brief The instance number of the peripheral.
    uint32_t instance;

    //! @brief Configure pinmux setting for the peripheral.
    void (*pinmuxConfig)(uint32_t instance, pinmux_type_t pinmux);

    //! @brief Control interface for the peripheral.
    const peripheral_control_interface_t *controlInterface;

    //! @brief Byte-level interface for the peripheral.
    //!
    //! May be NULL since not all periperhals support this interface.
    const peripheral_byte_inteface_t *byteInterface;

    //! @brief Packet level interface for the peripheral.
    const peripheral_packet_interface_t *packetInterface;
};

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

//! @brief Array of all peripherals available in this device.
extern const peripheral_descriptor_t g_peripherals[];

//! @brief Array of all usb controllers available in this device.
extern const uint32_t k_usbControllerIds[];
////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
//!@brief Get Peripheral Name
const char *get_peripheral_name(uint32_t peripheralTypeMask);

//! @brief Determines the active peripheral.
//!
//! This function has several stages:
//! - Init enabled peripherals.
//! - Compute timeout.
//! - Wait for peripheral activity with timeout.
//! - Shutdown inactive peripherals.
//!
//! If peripheral detection times out, then this function will call jump_to_application() to
//! directly enter the user application.
//!
//! The timeout value comes from the BCA if set, or the #BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT
//! configuration macro. If the boot pin is asserted, or if there is not a valid user application
//! in flash, then the timeout is disabled and peripheral detection will continue infinitely.
peripheral_descriptor_t const *get_active_peripheral(uint32_t timeoutMs);

//! @}

#endif // _peripheral_h
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
