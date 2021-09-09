/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2021 NXP
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
#define BL_ENABLE_PINMUX_I2C2 (BL_CONFIG_FLEXCOMM_I2C_2)
#define BL_ENABLE_PINMUX_SPI8 (BL_CONFIG_FLEXCOMM_SPI_8)

//! UART pinmux configurations
#define UART0_ENABLE_GINT (0)
#define UART0_ENABLE_PINT (1)
#if UART0_ENABLE_GINT
#define UART0_RX_GINT_BASE GINT0
#define UART0_RX_GINT_GROUP kGINT_Port0
#define UART0_RX_GPIO_IRQn GINT0_IRQn
#define UART0_RX_GPIO_IRQHandler GINT0_IRQHandler
#elif UART0_ENABLE_PINT
#define UART0_RX_PINT_BASE PINT
#define UART0_RX_PINT_INT_SRC kINPUTMUX_GpioPort0Pin2ToPintsel
#define UART0_RX_PINT_INT_TYPE kPINT_PinInt0
#define UART0_RX_GPIO_IRQn PIN_INT0_IRQn
#define UART0_RX_GPIO_IRQHandler PIN_INT0_IRQHandler
#endif
#define UART0_RX_IOPCTL_BASE IOPCTL
#define UART0_RX_GPIO_BASE GPIO
#define UART0_RX_GPIO_PIN_GROUP 0
#define UART0_RX_GPIO_PIN_NUM 02 // PIO0_02
#define UART0_RX_FUNC_ALT_MODE 1 // (FC0)FUNC mode for UART0 RX
#define UART0_RX_GPIO_ALT_MODE 0 // FUNC mode for GPIO
#define UART0_TX_IOPCTL_BASE IOPCTL
#define UART0_TX_GPIO_PIN_GROUP 0
#define UART0_TX_GPIO_PIN_NUM 01 // PIO0_01
#define UART0_TX_FUNC_ALT_MODE 1 // (FC0)FUNC mode for UART0 TX
#define UART0_IRQHandler FLEXCOMM0_IRQHandler

//! I2C pinmux configurations
#define I2C2_SCL_IOPCTL_BASE IOPCTL
#define I2C2_SCL_GPIO_PIN_GROUP 0
#define I2C2_SCL_GPIO_PIN_NUM 15 // PIO0_15
#define I2C2_SCL_FUNC_ALT_MODE 1 // (FC2)ALT mode for I2C2 SCL
#define I2C2_SDA_IOPCTL_BASE IOPCTL
#define I2C2_SDA_GPIO_PIN_GROUP 0
#define I2C2_SDA_GPIO_PIN_NUM 16 // PIO0_16
#define I2C2_SDA_FUNC_ALT_MODE 1 // (FC2)ALT mode for I2C2 SDA
#define I2C2_IRQHandler FLEXCOMM2_IRQHandler

//! SPI pinmux configurations
#define SPI8_SELECTED_SSEL 0
#define SPI8_SSEL_IOPCTL_BASE IOPCTL
#define SPI8_SSEL_GPIO_PIN_GROUP 1
#define SPI8_SSEL_GPIO_PIN_NUM 14 // PIO1_14
#define SPI8_SSEL_FUNC_ALT_MODE 1 // (FC14)ALT mode for SPI8 SSEL
#define SPI8_SCK_IOPCTL_BASE IOPCTL
#define SPI8_SCK_GPIO_PIN_GROUP 1
#define SPI8_SCK_GPIO_PIN_NUM 11 // PIO1_11
#define SPI8_SCK_FUNC_ALT_MODE 1 // (FC14)ALT mode for SPI8 SCK
#define SPI8_MISO_IOPCTL_BASE IOPCTL
#define SPI8_MISO_GPIO_PIN_GROUP 1
#define SPI8_MISO_GPIO_PIN_NUM 12 // PIO1_12
#define SPI8_MISO_FUNC_ALT_MODE 1 // (FC14)ALT mode for SPI8 MISO
#define SPI8_MOSI_IOPCTL_BASE IOPCTL
#define SPI8_MOSI_GPIO_PIN_GROUP 1
#define SPI8_MOSI_GPIO_PIN_NUM 13 // PIO1_13
#define SPI8_MOSI_FUNC_ALT_MODE 1 // (FC14)ALT mode for SPI8 MOSI
#define SPI8_IRQHandler FLEXCOMM14_IRQHandler

#if BL_CONFIG_SPI_ENABLE_DMA
#define SPI_DMA_INSTANCE DMA0
#define SPI_SLAVE_TX_DMA_CHANNEL 27
#define SPI_SLAVE_RX_DMA_CHANNEL 26
#endif

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
