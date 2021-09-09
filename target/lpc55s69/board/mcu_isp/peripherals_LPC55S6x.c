/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "bootloader/bl_peripheral_interface.h"
#include "packet/serial_packet.h"
#include "usb.h"
extern void uart_pinmux_config(uint32_t instance, pinmux_type_t pinmux);
////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#if !BL_CONFIG_FLEXCOMM_USART && !BL_CONFIG_USB_HID
#error At least one peripheral must be enabled!
#endif

//! @brief Peripheral array for LPC5588.
const peripheral_descriptor_t g_peripherals[] = {
#if BL_CONFIG_FLEXCOMM_USART_0
    // FLEXCOMM USART0
    {.typeMask = kPeripheralType_UART,
     .instance = 0,
     .pinmuxConfig = uart_pinmux_config,
     .controlInterface = &g_flexcommUsartControlInterface,
     .byteInterface = &g_flexcommUsartByteInterface,
     .packetInterface = &g_framingPacketInterface },
#endif // BL_CONFIG_LPUART_0

#if BL_CONFIG_USB_HID || BL_CONFIG_HS_USB_HID
    // USB FS/HS HID
    {.typeMask = kPeripheralType_USB_HID,
     .instance = 4,
     .pinmuxConfig = NULL,
     .controlInterface = &g_usbHidControlInterface,
     .byteInterface = NULL,
     .packetInterface = &g_usbHidPacketInterface },
     
     {.typeMask = kPeripheralType_USB_HID,
     .instance = 6,
     .pinmuxConfig = NULL,
     .controlInterface = &g_usbHidControlInterface,
     .byteInterface = NULL,
     .packetInterface = &g_usbHidPacketInterface },
#endif

    { 0 } // Terminator
};
//! @brief USB controller id array for LPC5588.
const uint32_t k_usbControllerIds[] =
   { kUSB_ControllerLpcIp3511Fs0, kUSB_ControllerLpcIp3511Hs0};

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
