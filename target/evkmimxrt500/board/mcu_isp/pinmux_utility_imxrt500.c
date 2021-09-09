/*
* Copyright 2014-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2021 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*
*/

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_inputmux.h"
#include "peripherals_pinmux.h"
#if UART0_ENABLE_GINT
#include "fsl_gint.h"
#elif UART0_ENABLE_PINT
#include "fsl_pint.h"
#endif

#if (BL_CONFIG_FLEXCOMM_USART)

#if BL_ENABLE_PINMUX_UART0
#define BL_ENABLED_MAX_UART_INSTANCE (0)
#endif

//! UART autobaud port irq configurations
#define GPIO_IRQC_INTERRUPT_ENABLED_PRIORITY 1
#define GPIO_IRQC_INTERRUPT_RESTORED_PRIORITY 0

enum
{
    kPullResistors_Disabled,
    kPullDownResistors_Enabled = 1,
    kPullUpResistors_Enabled = 2,
};

//! this is to store the function pointer for calling back to the function that wants
//! the UART RX instance pin that triggered the interrupt. This only supports 1 pin
//! for UART0 because UART1 is on PORTC which does not support interrupts :(

static pin_irq_callback_t s_pin_irq_func[BL_ENABLED_MAX_UART_INSTANCE + 1] = { 0 };

#endif // BL_CONFIG_FLEXCOMM_USART

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// Note: on LPCNEXT0, default values for all port registers are 0x00s.
static inline void IOPCTL_RestoreDefault(IOPCTL_Type *base, uint32_t port, uint32_t pin)
{
    base->PIO[port][pin] = 0;
}

//! @brief Configure the GPIO mode for auto baud detection.
static void IOPCTL_SetUartAutoBaudPinMode(IOPCTL_Type *base, GPIO_Type *gpioBase, uint32_t port, uint32_t pin)
{
    // Set PORT to GPIO mode, pull-up resistor enabled.
    base->PIO[port][pin] = IOPCTL_PIO_FSEL(0) | IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_PUPDSEL(1) | IOPCTL_PIO_IBENA(1);

    // Configure UART RX pin to digital input mode.
    gpioBase->DIR[port] &= ~(1U << pin);
}

static inline void IOPCTL_SetUartPinMode(IOPCTL_Type *base, uint32_t port, uint32_t pin, uint32_t mux)
{
    base->PIO[port][pin] = IOPCTL_PIO_FSEL(mux) | IOPCTL_PIO_IBENA(1);
}

#if BL_FEATURE_IRQ_NOTIFIER_PIN
static inline void IOPCTL_SetIrqNotifierPinMode(IOPCTL_Type *base, GPIO_Type *gpioBase, uint32_t port, uint32_t pin)
{
#if BL_FEATURE_IRQ_NOTIFIER_PIN_ACTIVE_POLARITY
    // Set nIRQ pin to GPIO mode, pull-down resistor enabled.
    base->PIO[port][pin] = IOPCTL_PIO_FSEL(0) | IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_PUPDSEL(0) | IOPCTL_PIO_IBENA(1) |
                           IOPCTL_PIO_SLEWRATE(0) | IOPCTL_PIO_FULLDRIVE(1);

    // Set the nIRQ pin to output low level(default state)
    gpioBase->CLR[port] |= (1U << pin);

    // Configure nIRQ pin to digital output mode.
    gpioBase->DIR[port] |= (1U << pin);

#else
    // Set nIRQ pin to GPIO mode, pull-up resistor enabled.
    base->PIO[port][pin] = IOPCTL_PIO_FSEL(0) | IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_PUPDSEL(1) | IOPCTL_PIO_IBENA(1) |
                           IOPCTL_PIO_SLEWRATE(0) | IOPCTL_PIO_FULLDRIVE(1);

    // Set the nIRQ pin to output high level(default state)
    gpioBase->SET[port] |= (1U << pin);

    // Configure nIRQ pin to digital output mode.
    gpioBase->DIR[port] |= (1U << pin);

#endif
}

static inline void IOPCTL_SetIrqNotifierPinOutput(GPIO_Type *gpioBase, uint32_t port, uint32_t pin, uint32_t value)
{
    value &= 0x1U;
    debug_printf("Bootloader: %s set gpio[%x][%x] output level to %x.", __func__, port, pin, value);
    if (value)
    {
        // Output high
        gpioBase->SET[port] |= (1U << pin);
    }
    else
    {
        // Output low
        gpioBase->CLR[port] |= (1U << pin);
    }
}

static inline void IOPCTL_RestoreIrqNotifierPinDefault(IOPCTL_Type *base,
                                                       GPIO_Type *gpioBase,
                                                       uint32_t port,
                                                       uint32_t pin)
{
    // Restore to default value.
    base->PIO[port][pin] = 0;
    gpioBase->DIR[port] = 0;
    gpioBase->CLR[port] |= (1U << pin);
}
#endif // #if BL_FEATURE_IRQ_NOTIFIER_PIN

/*!
 * @brief Configure pinmux for uart module.
 *
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module)
 */
void uart_pinmux_config(uint32_t instance, pinmux_type_t pinmux)
{
    (void)instance;

    switch (pinmux)
    {
        case kPinmuxType_Default:
            IOPCTL_RestoreDefault(UART0_TX_IOPCTL_BASE, UART0_RX_GPIO_PIN_GROUP, UART0_RX_GPIO_PIN_NUM);
            IOPCTL_RestoreDefault(UART0_RX_IOPCTL_BASE, UART0_TX_GPIO_PIN_GROUP, UART0_TX_GPIO_PIN_NUM);
            break;
        case kPinmuxType_PollForActivity:
            IOPCTL_SetUartAutoBaudPinMode(UART0_RX_IOPCTL_BASE, UART0_RX_GPIO_BASE, UART0_RX_GPIO_PIN_GROUP,
                                          UART0_RX_GPIO_PIN_NUM);
            break;
        case kPinmuxType_Peripheral:
            // Enable pins for UART0.
            IOPCTL_SetUartPinMode(UART0_RX_IOPCTL_BASE, UART0_RX_GPIO_PIN_GROUP, UART0_RX_GPIO_PIN_NUM,
                                  UART0_RX_FUNC_ALT_MODE); // Set UART0_RX pin to UART0_RX functionality
            IOPCTL_SetUartPinMode(UART0_TX_IOPCTL_BASE, UART0_TX_GPIO_PIN_GROUP, UART0_TX_GPIO_PIN_NUM,
                                  UART0_TX_FUNC_ALT_MODE); // Set UART0_TX pin to UART0_TX functionality
            break;
        default:
            break;
    }
}


//! @brief this is going to be used for autobaud IRQ handling for UART0
void UART0_RX_GPIO_IRQHandler(void)
{
// Clear interrupt before callback
#if UART0_ENABLE_GINT
    UART0_RX_GINT_BASE->CTRL |= GINT_CTRL_INT_MASK;
#elif UART0_ENABLE_PINT
    UART0_RX_PINT_BASE->FALL |= PINT_FALL_FDET_MASK;
#endif

    // Check if the pin for UART0 is what triggered the RX PORT interrupt
    if (s_pin_irq_func[0])
    {
        s_pin_irq_func[0](0);
    }
}

void enable_autobaud_pin_irq(uint32_t instance, pin_irq_callback_t func)
{
    (void)instance; // suppress compiler warnings

    NVIC_SetPriority(UART0_RX_GPIO_IRQn, GPIO_IRQC_INTERRUPT_ENABLED_PRIORITY);
    NVIC_EnableIRQ(UART0_RX_GPIO_IRQn);
// Connect trigger sources to PINT

#if UART0_ENABLE_GINT
    // Only look for a falling edge for our interrupts
    // Initialize GINT
    GINT_Init(UART0_RX_GINT_BASE);
    // Setup GINT for edge trigger, "OR" mode
    GINT_SetCtrl(UART0_RX_GINT_BASE, kGINT_CombineOr, kGINT_TrigEdge, NULL);
    // Select pins & polarity for GINT
    GINT_ConfigPins(UART0_RX_GINT_BASE, UART0_RX_GINT_GROUP, ~(1U << UART0_RX_GPIO_PIN_NUM),
                    (1U << UART0_RX_GPIO_PIN_NUM));
#elif UART0_ENABLE_PINT
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, UART0_RX_PINT_INT_TYPE, UART0_RX_PINT_INT_SRC);
    // Turnoff clock to inputmux to save power. Clock is only needed to make changes
    // INPUTMUX_Deinit(INPUTMUX);
    // Initialize PINT
    PINT_Init(UART0_RX_PINT_BASE);
    // Setup Pin Interrupt x for falling edge
    PINT_PinInterruptConfig(UART0_RX_PINT_BASE, UART0_RX_PINT_INT_TYPE, kPINT_PinIntEnableFallEdge, NULL);
#endif

    s_pin_irq_func[0] = func;
}

void disable_autobaud_pin_irq(uint32_t instance)
{
    (void)instance; // suppress compiler warnings

    NVIC_DisableIRQ(UART0_RX_GPIO_IRQn);
    NVIC_SetPriority(UART0_RX_GPIO_IRQn, GPIO_IRQC_INTERRUPT_RESTORED_PRIORITY);
#if UART0_ENABLE_GINT
    // De-initialize GINT
    GINT_Deinit(UART0_RX_GINT_BASE);
#elif UART0_ENABLE_PINT
    // De-initialize INPUTMUX
    INPUTMUX_Deinit(INPUTMUX);
    // De-initialize PINT
    PINT_Deinit(UART0_RX_PINT_BASE);
#endif
    s_pin_irq_func[0] = 0;
}

#if BL_FEATURE_IRQ_NOTIFIER_PIN
void irq_notifier_pinmux_config(uint32_t port, uint32_t pin, pinmux_type_t pinmux)
{
    switch (pinmux)
    {
        case kPinmuxType_Default:
            IOPCTL_RestoreIrqNotifierPinDefault(IOPCTL, GPIO, port, pin);
            break;
        case kPinmuxType_Peripheral:
            IOPCTL_SetIrqNotifierPinMode(IOPCTL, GPIO, port, pin);
            break;
        default:
            break;
    }
}

void irq_notifier_pinout_config(uint32_t port, uint32_t pin, bool active)
{
#if BL_FEATURE_IRQ_NOTIFIER_PIN_ACTIVE_POLARITY
    IOPCTL_SetIrqNotifierPinOutput(GPIO, port, pin, active ? 1 : 0);
#else
    IOPCTL_SetIrqNotifierPinOutput(GPIO, port, pin, active ? 0 : 1);
#endif
}
#endif // #if BL_FEATURE_IRQ_NOTIFIER_PIN

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
