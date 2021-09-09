/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "fsl_iocon.h"
#include "fsl_gpio.h"
#include "fsl_gint.h"
#include "fsl_inputmux.h"
#include "fsl_pint.h"
#include "peripherals_pinmux.h"

#if (BL_CONFIG_FLEXCOMM_USART)

#if BL_ENABLE_PINMUX_UART0
#define BL_ENABLED_MAX_UART_INSTANCE (0)
#endif

//! UART autobaud port irq configurations
#define GPIO_IRQC_INTERRUPT_ENABLED_PRIORITY 1
#define GPIO_IRQC_INTERRUPT_RESTORED_PRIORITY 0

//! this is to store the function pointer for calling back to the function that wants
//! the UART RX instance pin that triggered the interrupt. This only supports 1 pin
//! for UART0 because UART1 is on PORTC which does not support interrupts :(

static pin_irq_callback_t s_pin_irq_func[BL_ENABLED_MAX_UART_INSTANCE + 1] = { 0 };

#endif // BL_CONFIG_FLEXCOMM_USART

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

static inline void IOCON_RestoreDefault(IOCON_Type *base, uint32_t port, uint32_t pin)
{
    base->PIO[port][pin] = ((~IOCON_PIO_FUNC_MASK) & base->PIO[port][pin]) | IOCON_PIO_FUNC(IOCON_FUNC0);
}

#if BL_CONFIG_FLEXCOMM_USART
//! @brief Configure the GPIO mode for auto baud detection.
static inline void IOCON_SetUartAutoBaudPinMode(IOCON_Type *ioconBase, GPIO_Type *gpioBase, uint32_t port, uint32_t pin)
{
    // Enable Pull-up register, set to Digital mode, GPIO mode
    IOCON->PIO[port][pin] = (2 << 4) | (1 << 8);
    //    IOCON_PinMuxSet(ioconBase, port, pin, IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);

    // Configure UART RX pin to digital input mode.
    gpioBase->DIR[port] &= ~(1U << pin);
}

static inline void IOCON_SetUartPinMode(IOCON_Type *base, uint32_t port, uint32_t pin, uint32_t mux)
{
    // Set to GPIO mode, High Slew rate
    IOCON->PIO[port][pin] = mux | (1u << 8) | (1u << 6);
    //    IOCON_PinMuxSet(base, port, pin, (mux | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF));
}
#endif

#if BL_CONFIG_FLEXCOMM_I2C
#define IOCON_PIO_SLEW_STANDARD (0x0 << 6) /*!<@brief Standard mode, output slew rate control is enabled */
#define IOCON_PIO_INV_DI (0x0 << 7)        /*!<@brief Input function is not inverted */
#define IOCON_PIO_OPENDRAIN_DI (0x0 << 9)  /*!<@brief Open drain is disabled */
#define IOCON_PIO_SSEL_3V3 (0x0 << 10)     /*!<@brief 3V3 signaling in I2C mode */
#define IOCON_PIO_SSEL_1V8 (0x1 << 10)     /*!<@brief 1V8 signaling in I2C mode */
#define IOCON_PIO_INPFILT_OFF (0x1 << 11)  /*!<@brief Input filter disabled */
#define IOCON_PIO_ECS_EN (0x0 << 12)       /*!<@brief pull resistor is connected to IO */
#define IOCON_PIO_ECS_OFF (0x1 << 12)      /*!<@brief IO is in open drain */
#define IOCON_PIO_I2CSLEW_I2C (0x1 << 13)  /*!<@brief I2C mode */
#define IOCON_PIO_I2CFILTER_EN (0x0 << 14) /*!<@brief I2C 50 ns glitch filter enabled */
#warning "need to double check iocon driver "

static inline void IOCON_SetI2cPinMode(IOCON_Type *base, uint32_t port, uint32_t pin, uint32_t mux)
{
    /*
      IOCON_PinMuxSet(base, port, pin, mux |
                                       IOCON_PIO_I2CSLEW_I2C |
                                       IOCON_PIO_INV_DI |
                                       IOCON_DIGITAL_EN |
                                       IOCON_PIO_INPFILT_OFF |
                                       IOCON_PIO_I2CFILTER_EN);
   */
    IOCON_PinMuxSet(base, port, pin, mux | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_PIO_INPFILT_OFF);
}
#endif

#if BL_CONFIG_FLEXCOMM_SPI
static inline void IOCON_SetSpiPinMode(IOCON_Type *base, uint32_t port, uint32_t pin, uint32_t mux)
{
    // Pull-up, digital mode, fast slew rate
    IOCON->PIO[port][pin] = mux | (2 << 4) | (1 << 8) | (1 << 6);
    //    IOCON_PinMuxSet(base, port, pin, (mux | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));
}
#endif

#if BL_FEATURE_IRQ_NOTIFIER_PIN
static inline void IOPCTL_SetIrqNotifierPinMode(IOCON_Type *base, GPIO_Type *gpioBase, uint32_t port, uint32_t pin)
{
#if BL_FEATURE_IRQ_NOTIFIER_PIN_ACTIVE_POLARITY
    // Set nIRQ pin to GPIO mode, pull-down resistor enabled.
    IOCON->PIO[port][pin] = (1 << 4) | (1 << 8) | (1 << 6);
    //    IOCON_PinMuxSet(base, port, pin, (IOCON_FUNC0 | IOCON_MODE_PULLDOWN | IOCON_DIGITAL_EN));

    // Set the nIRQ pin to output low level(default state)
    gpioBase->CLR[port] |= (1U << pin);

    // Configure nIRQ pin to digital output mode.
    gpioBase->DIR[port] |= (1U << pin);
#else
    // Set nIRQ pin to GPIO mode, pull-up resistor enabled.
    IOCON->PIO[port][pin] = (2 << 4) | (1 << 8) | (1 << 6);
    //    IOCON_PinMuxSet(base, port, pin, (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));

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

static inline void IOPCTL_RestoreIrqNotifierPinDefault(IOCON_Type *base,
                                                       GPIO_Type *gpioBase,
                                                       uint32_t port,
                                                       uint32_t pin)
{
    // Restore to default value.
    IOCON_PinMuxSet(base, port, pin, 0);
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
    switch (instance)
    {
#if BL_ENABLE_PINMUX_UART0
        case 0:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(UART0_TX_IOCON_BASE, UART0_RX_GPIO_PIN_GROUP, UART0_RX_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(UART0_RX_IOCON_BASE, UART0_TX_GPIO_PIN_GROUP, UART0_TX_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_PollForActivity:
                    IOCON_SetUartAutoBaudPinMode(UART0_RX_IOCON_BASE, UART0_RX_GPIO_BASE, UART0_RX_GPIO_PIN_GROUP,
                                                 UART0_RX_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for UART0.
                    IOCON_SetUartPinMode(UART0_RX_IOCON_BASE, UART0_RX_GPIO_PIN_GROUP, UART0_RX_GPIO_PIN_NUM,
                                         UART0_RX_FUNC_ALT_MODE); // Set UART0_RX pin to UART0_RX functionality
                    IOCON_SetUartPinMode(UART0_TX_IOCON_BASE, UART0_TX_GPIO_PIN_GROUP, UART0_TX_GPIO_PIN_NUM,
                                         UART0_TX_FUNC_ALT_MODE); // Set UART0_TX pin to UART0_TX functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_UART0

        default:
            break;
    }
}

/*!
 * @brief Configure pinmux for i2c module.
 *
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module)
 */
void i2c_pinmux_config(uint32_t instance, pinmux_type_t pinmux)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_I2C1
        case 1:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(I2C1_SDA_IOCON_BASE, I2C1_SDA_GPIO_PIN_GROUP, I2C1_SDA_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(I2C1_SCL_IOCON_BASE, I2C1_SCL_GPIO_PIN_GROUP, I2C1_SCL_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C1.
                    IOCON_SetI2cPinMode(I2C1_SDA_IOCON_BASE, I2C1_SDA_GPIO_PIN_GROUP, I2C1_SDA_GPIO_PIN_NUM,
                                        I2C1_SDA_FUNC_ALT_MODE); // Set I2C1_SDA pin to I2C1_SDA functionality
                    IOCON_SetI2cPinMode(I2C1_SCL_IOCON_BASE, I2C1_SCL_GPIO_PIN_GROUP, I2C1_SCL_GPIO_PIN_NUM,
                                        I2C1_SCL_FUNC_ALT_MODE); // Set I2C1_SCL pin to I2C1_SCL functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_I2C1

#if BL_ENABLE_PINMUX_I2C2
        case 2:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(I2C2_SDA_IOCON_BASE, I2C2_SDA_GPIO_PIN_GROUP, I2C2_SDA_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(I2C2_SCL_IOCON_BASE, I2C2_SCL_GPIO_PIN_GROUP, I2C2_SCL_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C2.
                    IOCON_SetI2cPinMode(I2C2_SDA_IOCON_BASE, I2C2_SDA_GPIO_PIN_GROUP, I2C2_SDA_GPIO_PIN_NUM,
                                        I2C2_SDA_FUNC_ALT_MODE); // Set I2C2_SDA pin to I2C2_SDA functionality
                    IOCON_SetI2cPinMode(I2C2_SCL_IOCON_BASE, I2C2_SCL_GPIO_PIN_GROUP, I2C2_SCL_GPIO_PIN_NUM,
                                        I2C2_SCL_FUNC_ALT_MODE); // Set I2C2_SCL pin to I2C2_SCL functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_I2C2

        default:
            break;
    }
}

/*!
 * @brief Configure pinmux for SPI module.
 *
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module)
 */
void spi_pinmux_config(uint32_t instance, pinmux_type_t pinmux)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_SPI3
        case 3:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(SPI3_SSEL_IOCON_BASE, SPI3_SSEL_GPIO_PIN_GROUP, SPI3_SSEL_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI3_SCK_IOCON_BASE, SPI3_SCK_GPIO_PIN_GROUP, SPI3_SCK_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI3_MISO_IOCON_BASE, SPI3_MISO_GPIO_PIN_GROUP, SPI3_MISO_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI3_MOSI_IOCON_BASE, SPI3_MOSI_GPIO_PIN_GROUP, SPI3_MOSI_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for SPI3
                    IOCON_SetSpiPinMode(SPI3_MISO_IOCON_BASE, SPI3_MISO_GPIO_PIN_GROUP, SPI3_MISO_GPIO_PIN_NUM,
                                        SPI3_MISO_FUNC_ALT_MODE); // Set SPI3_MISO pin to SPI3_MISO functionality
                    IOCON_SetSpiPinMode(SPI3_SSEL_IOCON_BASE, SPI3_SSEL_GPIO_PIN_GROUP, SPI3_SSEL_GPIO_PIN_NUM,
                                        SPI3_SSEL_FUNC_ALT_MODE); // Set SPI3_SSEL pin to SPI3_SSEL functionality
                    IOCON_SetSpiPinMode(SPI3_SCK_IOCON_BASE, SPI3_SCK_GPIO_PIN_GROUP, SPI3_SCK_GPIO_PIN_NUM,
                                        SPI3_SCK_FUNC_ALT_MODE); // Set SPI3_SCK pin to SPI3_SCK functionality
                    IOCON_SetSpiPinMode(SPI3_MOSI_IOCON_BASE, SPI3_MOSI_GPIO_PIN_GROUP, SPI3_MOSI_GPIO_PIN_NUM,
                                        SPI3_MOSI_FUNC_ALT_MODE); // Set SPI3_MOSI pin to SPI3_MOSI functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_SPI3

#if BL_ENABLE_PINMUX_SPI8
        case 8:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(SPI8_SSEL_IOCON_BASE, SPI8_SSEL_GPIO_PIN_GROUP, SPI8_SSEL_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI8_SCK_IOCON_BASE, SPI8_SCK_GPIO_PIN_GROUP, SPI8_SCK_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI8_MISO_IOCON_BASE, SPI8_MISO_GPIO_PIN_GROUP, SPI8_MISO_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI8_MOSI_IOCON_BASE, SPI8_MOSI_GPIO_PIN_GROUP, SPI8_MOSI_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for SPI8
                    IOCON_SetSpiPinMode(SPI8_MISO_IOCON_BASE, SPI8_MISO_GPIO_PIN_GROUP, SPI8_MISO_GPIO_PIN_NUM,
                                        SPI8_MISO_FUNC_ALT_MODE); // Set SPI8_MISO pin to SPI8_MISO functionality
                    IOCON_SetSpiPinMode(SPI8_SSEL_IOCON_BASE, SPI8_SSEL_GPIO_PIN_GROUP, SPI8_SSEL_GPIO_PIN_NUM,
                                        SPI8_SSEL_FUNC_ALT_MODE); // Set SPI8_SSEL pin to SPI8_SSEL functionality
                    IOCON_SetSpiPinMode(SPI8_SCK_IOCON_BASE, SPI8_SCK_GPIO_PIN_GROUP, SPI8_SCK_GPIO_PIN_NUM,
                                        SPI8_SCK_FUNC_ALT_MODE); // Set SPI8_SCK pin to SPI8_SCK functionality
                    IOCON_SetSpiPinMode(SPI8_MOSI_IOCON_BASE, SPI8_MOSI_GPIO_PIN_GROUP, SPI8_MOSI_GPIO_PIN_NUM,
                                        SPI8_MOSI_FUNC_ALT_MODE); // Set SPI8_MOSI pin to SPI8_MOSI functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_SPI8

        default:
            break;
    }
}

//! @brief this is going to be used for autobaud IRQ handling for UART0
#if BL_ENABLE_PINMUX_UART0
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
#endif // #if BL_ENABLE_PINMUX_UART0

void enable_autobaud_pin_irq(uint32_t instance, pin_irq_callback_t func)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_UART0
        case 0:
            NVIC_SetPriority(UART0_RX_GPIO_IRQn, GPIO_IRQC_INTERRUPT_ENABLED_PRIORITY);
            NVIC_EnableIRQ(UART0_RX_GPIO_IRQn);
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
            // Connect trigger sources to PINT
            INPUTMUX_Init(INPUTMUX);
            INPUTMUX_AttachSignal(INPUTMUX, UART0_RX_PINT_INT_TYPE, UART0_RX_PINT_INT_SRC);
            // Turnoff clock to inputmux to save power. Clock is only needed to make changes
            INPUTMUX_Deinit(INPUTMUX);
            // Initialize PINT
            PINT_Init(UART0_RX_PINT_BASE);
            // Setup Pin Interrupt x for falling edge
            PINT_PinInterruptConfig(UART0_RX_PINT_BASE, UART0_RX_PINT_INT_TYPE, kPINT_PinIntEnableFallEdge, NULL);
#endif
            s_pin_irq_func[0] = func;
            break;
#endif // #if BL_ENABLE_PINMUX_UART0

        default:
            break;
    }
}

void disable_autobaud_pin_irq(uint32_t instance)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_UART0
        case 0:
            NVIC_DisableIRQ(UART0_RX_GPIO_IRQn);
            NVIC_SetPriority(UART0_RX_GPIO_IRQn, GPIO_IRQC_INTERRUPT_RESTORED_PRIORITY);
#if UART0_ENABLE_GINT
            // De-initialize GINT
            GINT_Deinit(UART0_RX_GINT_BASE);
#elif UART0_ENABLE_PINT
            // De-initialize PINT
            PINT_Deinit(UART0_RX_PINT_BASE);
#endif
            s_pin_irq_func[0] = 0;
            break;
#endif // #if BL_ENABLE_PINMUX_UART0

        default:
            break;
    }
}

#if BL_FEATURE_IRQ_NOTIFIER_PIN
void irq_notifier_pinmux_config(uint32_t port, uint32_t pin, pinmux_type_t pinmux)
{
    switch (pinmux)
    {
        case kPinmuxType_Default:
            IOPCTL_RestoreIrqNotifierPinDefault(IOCON, GPIO, port, pin);
            break;
        case kPinmuxType_Peripheral:
            IOPCTL_SetIrqNotifierPinMode(IOCON, GPIO, port, pin);
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
#endif
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
