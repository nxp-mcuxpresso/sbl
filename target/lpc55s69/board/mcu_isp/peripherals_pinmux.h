/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! peripheral enable configurations
#define BL_ENABLE_PINMUX_UART0 (BL_CONFIG_FLEXCOMM_USART_0)

//! UART pinmux configurations
#define UART0_ENABLE_GINT (1)
#define UART0_ENABLE_PINT (0)
#if UART0_ENABLE_GINT
#define UART0_RX_GINT_BASE GINT0
#define UART0_RX_GINT_GROUP kGINT_Port0
#define UART0_RX_GPIO_IRQn GINT0_IRQn
#define UART0_RX_GPIO_IRQHandler GINT0_IRQHandler
#elif UART0_ENABLE_PINT
#define UART0_RX_PINT_BASE PINT
#define UART0_RX_PINT_INT_SRC kINPUTMUX_GpioPort0Pin29ToPintsel
#define UART0_RX_PINT_INT_TYPE kPINT_PinInt0
#define UART0_RX_GPIO_IRQn PIN_INT0_IRQn
#define UART0_RX_GPIO_IRQHandler PIN_INT0_IRQHandler
#endif
#define UART0_RX_IOCON_BASE IOCON
#define UART0_RX_GPIO_BASE GPIO
#define UART0_RX_GPIO_PIN_GROUP 0
#define UART0_RX_GPIO_PIN_NUM 29           // PIO0_29
#define UART0_RX_FUNC_ALT_MODE IOCON_FUNC1 // (FC0)FUNC mode for UART0 RX
#define UART0_RX_GPIO_ALT_MODE IOCON_FUNC0 // FUNC mode for GPIO
#define UART0_TX_IOCON_BASE IOCON
#define UART0_TX_GPIO_PIN_GROUP 0
#define UART0_TX_GPIO_PIN_NUM 30           // PIO0_30
#define UART0_TX_FUNC_ALT_MODE IOCON_FUNC1 // (FC0)FUNC mode for UART0 TX
#define UART0_IRQHandler FLEXCOMM0_IRQHandler

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
