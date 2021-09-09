/*
* Copyright 2014-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2021 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*
*/

#include "bootloader/bl_context.h"
#include "bootloader/bl_peripheral_interface.h"
#include "packet/serial_packet.h"
#include "usb.h"

extern void uart_pinmux_config(uint32_t instance, pinmux_type_t pinmux);
extern void i2c_pinmux_config(uint32_t instance, pinmux_type_t pinmux);
extern void spi_pinmux_config(uint32_t instance, pinmux_type_t pinmux);
////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#if !BL_CONFIG_FLEXCOMM_USART && !BL_CONFIG_FLEXCOMM_I2C && !BL_CONFIG_FLEXCOMM_SPI && \
    !(BL_CONFIG_HS_USB_HID || BL_CONFIG_HS_USB_MSC || BL_CONFIG_HS_USB_DFU)
#error At least one peripheral must be enabled!
#endif

//! @brief Peripheral array for LPC6824.
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

#if BL_CONFIG_HS_USB_HID || BL_CONFIG_HS_USB_DFU
    // USB HS HID,DFU
    {.typeMask = kPeripheralType_USB_HID
#if BL_CONFIG_HS_USB_DFU
                 |
                 kPeripheralType_USB_DFU
#endif
     ,
     .instance = 6,
     .pinmuxConfig = NULL,
     .controlInterface = &g_usbHidControlInterface,
     .byteInterface = NULL,
     .packetInterface = &g_usbHidPacketInterface },
#endif

    { 0 } // Terminator
};

//! @brief USB controller id array for i.MXRT595.
const uint32_t k_usbControllerIds[] =
   { kUSB_ControllerLpcIp3511Hs0, (uint32_t)NULL};

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
