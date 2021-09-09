/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2021NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "device_config.h"
#if FSL_FEATURE_SOC_IOMUXC_COUNT
#include "fsl_iomuxc.h"
#endif // #if FSL_FEATURE_SOC_IOMUXC_COUNT
#include "fsl_gpio.h"
#include "peripherals_pinmux.h"

#if BL_ENABLE_PINMUX_UART1
#define BL_ENABLED_MAX_UART_INSTANCE (1)
#endif

//! UART autobaud port irq configurations
#define GPIO_IRQC_INTERRUPT_ENABLED_PRIORITY 1
#define GPIO_IRQC_INTERRUPT_RESTORED_PRIORITY 0

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! this is to store the function pointer for calling back to the function that wants
//! the UART RX instance pin that triggered the interrupt. This only supports 1 pin
//! for UART0 because UART1 is on PORTC which does not support interrupts :(

static pin_irq_callback_t s_pin_irq_func[BL_ENABLED_MAX_UART_INSTANCE + 1] = { 0 };

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
static inline void IOMUXC_RestoreDefault(uint32_t muxRegister,
                                         uint32_t muxMode,
                                         uint32_t inputRegister,
                                         uint32_t inputDaisy,
                                         uint32_t configRegister,
                                         uint32_t configDefaultValue)
{
    *((volatile uint32_t *)muxRegister) = 0 /*Mux default value*/;
    if (inputRegister)
    {
        *((volatile uint32_t *)inputRegister) = 0 /*Daisy defualt value*/;
    }
    if (configRegister)
    {
        *((volatile uint32_t *)configRegister) = configDefaultValue;
    }
}

static inline void IOMUXC_SetUartAutoBaudPinMode(uint32_t muxRegister,
                                                 uint32_t muxMode,
                                                 uint32_t inputRegister,
                                                 uint32_t inputDaisy,
                                                 uint32_t configRegister,
                                                 GPIO_Type *gpioBase,
                                                 uint32_t pin)
{
    // Configure the UART RX pin to GPIO mode
    IOMUXC_SetPinMux(muxRegister, muxMode, inputRegister, inputDaisy, configRegister,
                     0 /*kIOMUXC_InputPathDeteminedByMux*/);
#if BL_FEATURE_UART_RX_PULLUP
    // Pull-up resistor enabled.
    *((volatile uint32_t *)configRegister) |= UART1_PULLUP_PAD_CTRL;
#else
    // Pull-up resistor disabled.
    *((volatile uint32_t *)configRegister) &= ~UART1_PULLUP_DISABLE_PAD_CTRL;
#endif // BL_FEATURE_UART_RX_PULLUP
    // Configure UART RX pin to digital input mode.
    gpioBase->GDIR &= (uint32_t) ~(1 << pin);
}

static inline void IOMUXC_SetUartPinMode(
    uint32_t muxRegister, uint32_t muxMode, uint32_t inputRegister, uint32_t inputDaisy, uint32_t configRegister)
{
    IOMUXC_SetPinMux(muxRegister, muxMode, inputRegister, inputDaisy, configRegister,
                     0 /*kIOMUXC_InputPathDeteminedByMux*/);
    *(volatile uint32_t *)configRegister = LPUART1_PAD_CTRL;
};

/*!
 * @brief Configure pinmux for uart module.
 *
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module)
 */
void uart_pinmux_config(uint32_t instance, pinmux_type_t pinmux)
{
#if BL_ENABLE_PINMUX_UART1
    switch (pinmux)
    {
        case kPinmuxType_Default:
            IOMUXC_RestoreDefault(UART1_TX_IOMUXC_MUX_FUNC, UART1_TX_IOMUXC_PAD_DEFAULT);
            IOMUXC_RestoreDefault(UART1_TX_IOMUXC_MUX_FUNC, UART1_TX_IOMUXC_PAD_DEFAULT);
            break;
        case kPinmuxType_PollForActivity:
            IOMUXC_SetUartAutoBaudPinMode(UART1_RX_IOMUXC_MUX_GPIO, UART1_RX_GPIO_BASE, UART1_RX_GPIO_PIN_NUM);
            break;
        case kPinmuxType_Peripheral:
            // Enable pins for UART1.
            IOMUXC_SetUartPinMode(UART1_RX_IOMUXC_MUX_FUNC);
            IOMUXC_SetUartPinMode(UART1_TX_IOMUXC_MUX_FUNC);
            break;
        default:
            break;
    }
#endif // #if BL_ENABLE_PINMUX_UART1
}

//! @brief this is going to be used for autobaud IRQ handling for UART1
#if BL_ENABLE_PINMUX_UART1
void UART1_RX_GPIO_IRQHandler(void)
{
    uint32_t interrupt_flag = (1U << UART1_RX_GPIO_PIN_NUM);
    // Check if the pin for UART1 is what triggered the RX PORT interrupt
    if ((UART1_RX_GPIO_BASE->ISR & interrupt_flag) && s_pin_irq_func[1])
    {
        UART1_RX_GPIO_BASE->ISR = interrupt_flag;
        __DSB();
        s_pin_irq_func[1](1);
    }
}
#endif // #if BL_ENABLE_PINMUX_UART1

void enable_autobaud_pin_irq(uint32_t instance, pin_irq_callback_t func)
{
#if BL_ENABLE_PINMUX_UART1
    s_pin_irq_func[1] = func;
    // Only look for a falling edge for our interrupts
    GPIO_SetPinInterruptConfig(UART1_RX_GPIO_BASE, UART1_RX_GPIO_PIN_NUM, kGPIO_IntFallingEdge);
    UART1_RX_GPIO_BASE->IMR |= (1U << UART1_RX_GPIO_PIN_NUM);
    NVIC_SetPriority(UART1_RX_GPIO_IRQn, GPIO_IRQC_INTERRUPT_ENABLED_PRIORITY);
    NVIC_EnableIRQ(UART1_RX_GPIO_IRQn);
#endif
}

void disable_autobaud_pin_irq(uint32_t instance)
{
#if BL_ENABLE_PINMUX_UART1
    NVIC_DisableIRQ(UART1_RX_GPIO_IRQn);
    NVIC_SetPriority(UART1_RX_GPIO_IRQn, GPIO_IRQC_INTERRUPT_RESTORED_PRIORITY);
    GPIO_SetPinInterruptConfig(UART1_RX_GPIO_BASE, UART1_RX_GPIO_PIN_NUM, kGPIO_NoIntmode);
    s_pin_irq_func[1] = 0;
#endif // #if BL_ENABLE_PINMUX_UART1
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
