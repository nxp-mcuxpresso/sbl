/*
 * Copyright (c) 2013-2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "bootloader_config.h"
#include "bootloader/bl_peripheral_interface.h"
#include "bootloader/bl_irq_common.h"
#include "autobaud/autobaud.h"
#include "packet/serial_packet.h"
#include "fsl_device_registers.h"
#include "fsl_lpuart.h"
#include "utilities/fsl_assert.h"

#if BL_CONFIG_LPUART

//! @addtogroup lpuart_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static bool lpuart_poll_for_activity(const peripheral_descriptor_t *self);
static status_t lpuart_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
static void lpuart_full_shutdown(const peripheral_descriptor_t *self);

static status_t lpuart_write(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount,
                                     uint32_t timeoutMs);

extern void LPUART_SetSystemIRQ(uint32_t instance, PeripheralSystemIRQSetting set);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const peripheral_control_interface_t g_lpuartControlInterface = {
    .pollForActivity = lpuart_poll_for_activity, .init = lpuart_full_init, .shutdown = lpuart_full_shutdown, .pump = 0
};

const peripheral_byte_inteface_t g_lpuartByteInterface = {.init = NULL, .write = lpuart_write };

static serial_byte_receive_func_t s_lpuart_byte_receive_callback;

static bool g_lpuartInitStatus[FSL_FEATURE_SOC_LPUART_COUNT] = { false }; // not initialized.

static const uint32_t g_lpuartBaseAddr[] = LPUART_BASE_ADDRS;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*!
 * This function will be called from the main peripheral detection loop to drive
 * the autobaud detect for this UART instance. If it is completed the UART is
 * fully initialized and ready to use.
 */
bool lpuart_poll_for_activity(const peripheral_descriptor_t *self)
{
    uint32_t instance;
#if USE_ONLY_UART(0)
    instance = 0;
#elif USE_ONLY_UART(1)
    instance = 1;
#else
    instance = self->instance;
#endif // USE_ONLY_UART(0)

    // Check for autobaud completion.
    uint32_t baud;
    status_t autoBaudCompleted = autobaud_get_rate(instance, &baud);

    // If autobaud is still running then exit immediately.
    if (autoBaudCompleted == kStatus_Success)
    {
        lpuart_config_t userConfig;
        uint32_t baseAddr = g_lpuartBaseAddr[instance];

        LPUART_GetDefaultConfig(&userConfig);
        userConfig.baudRate_Bps = baud;

        debug_printf("Auto detected baudrate =%d\n", baud);

        if (LPUART_Init((LPUART_Type *)baseAddr, &userConfig, get_uart_clock(instance)) == kStatus_Success)
        {
#if !BL_FEATURE_6PINS_PERIPHERAL
            // Configure selected pin as uart peripheral interface
            self->pinmuxConfig(instance, kPinmuxType_Peripheral);
#endif // !BL_FEATURE_6PINS_PERIPHERAL

            // Enable LPUART system interrupt
            LPUART_SetSystemIRQ(instance, kPeripheralEnableIRQ);

            // Enable LPUART peripheral interrupt
            LPUART_EnableInterrupts((LPUART_Type *)baseAddr, kLPUART_RxDataRegFullInterruptEnable);

            LPUART_EnableRx((LPUART_Type *)baseAddr, true);
            LPUART_EnableTx((LPUART_Type *)baseAddr, true);

            // This was the byte pattern identified in autobaud detection, inform the command layer
            s_lpuart_byte_receive_callback(kFramingPacketStartByte);
            s_lpuart_byte_receive_callback(kFramingPacketType_Ping);

            // update pheripheral interface init status
            g_lpuartInitStatus[instance] = true;

            // Return true to indicate autobaud is complete and the UART is active.
            return true;
        }
        else
        {
            autobaud_init(instance);
        }
    }
    return false;
}

//! Note that we don't ungate the LPUART clock gate here. That is done only after the
//! autobaud process has completed successfully.
status_t lpuart_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    s_lpuart_byte_receive_callback = function;

    // Since we are using autobaud once the detection is completed
    // it will call the UART initialization and remux the pins when it completes
    self->pinmuxConfig(self->instance, kPinmuxType_PollForActivity);

    // Init autobaud detector.
    autobaud_init(self->instance);

    return kStatus_Success;
}

#if BL_FEATURE_6PINS_PERIPHERAL
#if (defined(__GNUC__))
/* #pragma GCC push_options */
/* #pragma GCC optimize("O0") */
void __attribute__((optimize("O0"))) lpuart_full_shutdown(const peripheral_descriptor_t *self)
#else
#if (defined(__ICCARM__))
#pragma optimize = none
#endif
#if (defined(__CC_ARM))
#pragma push
#pragma O0
#endif
void lpuart_full_shutdown(const peripheral_descriptor_t *self)
#endif
#else
void lpuart_full_shutdown(const peripheral_descriptor_t *self)
#endif
{
    uint32_t instance;
#if USE_ONLY_UART(0)
    instance = 0;
#elif USE_ONLY_UART(1)
    instance = 1;
#else
    instance = self->instance;
#endif // USE_ONLY_UART(0)

    if (g_lpuartInitStatus[instance])
    {
        uint32_t baseAddr = g_lpuartBaseAddr[instance];

        // Disable LPUART interrupt
        LPUART_SetSystemIRQ(instance, kPeripheralDisableIRQ);

        // Reset LPUART registers
        LPUART_Deinit((LPUART_Type *)baseAddr);
    }

//! Note: if not deinit autobaud(IRQ method), user app may encounters hardfault
//! if it doesn't provide related pin interrupt service routine.
#if BL_FEATURE_UART_AUTOBAUD_IRQ
    // De-init autobaud detector.
    autobaud_deinit(instance);
#endif

#if BL_FEATURE_6PINS_PERIPHERAL
    // When the active peripheral is not UART, we should only restore
    //   those pin which we used to poll for activity.
    if (g_bootloaderContext.activePeripheral == NULL)
    {
        self->pinmuxConfig(self->instance, kPinmuxType_RestoreForActivity);
    }
    // When the active peripheral is UART, we should restore all
    //  the UART peripheral pin.
    else
#endif
    {
        // Restore selected pin to default state to reduce IDD.
        self->pinmuxConfig(self->instance, kPinmuxType_Default);
    }
}
#if BL_FEATURE_6PINS_PERIPHERAL
#if (defined(__CC_ARM))
#pragma pop
#endif
#if (defined(__GNUC__))
/* #pragma GCC pop_options */
#endif
#endif

status_t lpuart_write(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount, uint32_t timeoutMs)
{
    uint32_t instance;
#if USE_ONLY_UART(0)
    instance = 0;
#elif USE_ONLY_UART(1)
    instance = 1;
#else
    instance = self->instance;
#endif // USE_ONLY_UART(0)

    uint32_t baseAddr = g_lpuartBaseAddr[instance];

    LPUART_WriteBlocking((LPUART_Type *)baseAddr, buffer, byteCount);

    return kStatus_Success;
}

#if defined(BL_CONFIG_LPUART_0) && BL_CONFIG_LPUART_0
/********************************************************************/
/*
 * LPUART0 IRQ Handler
 *
 */
#ifdef PKE18F15_SERIES
void LPUART0_RxTx_IRQHandler(void)
#else
void LPUART0_IRQHandler(void)
#endif
{
    if (LPUART_GetStatusFlags(LPUART0) & kLPUART_RxDataRegFullFlag)
    {
        uint8_t byte;
        byte = LPUART0->DATA;
        s_lpuart_byte_receive_callback(byte);
    }
}
#endif // #if defined (BL_CONFIG_LPUART_0) && BL_CONFIG_LPUART_0

#if defined(BL_CONFIG_LPUART_1) && BL_CONFIG_LPUART_1
#ifdef PKE18F15_SERIES
void LPUART1_RxTx_IRQHandler(void)
#else
void LPUART1_IRQHandler(void)
#endif
{
    if (LPUART_GetStatusFlags(LPUART1) & kLPUART_RxDataRegFullFlag)
    {
        uint8_t byte;
        byte = LPUART1->DATA;
        s_lpuart_byte_receive_callback(byte);
    }
}
#endif // #if defined (BL_CONFIG_LPUART_1) && BL_CONFIG_LPUART_1

#if defined(BL_CONFIG_LPUART_2) && BL_CONFIG_LPUART_2
#ifdef PKE18F15_SERIES
void LPUART2_RxTx_IRQHandler(void)
#else
void LPUART2_IRQHandler(void)
#endif
{
    if (LPUART_GetStatusFlags(LPUART2) & kLPUART_RxDataRegFullFlag)
    {
        uint8_t byte;
        byte = LPUART2->DATA;
        s_lpuart_byte_receive_callback(byte);
    }
}
#endif // #if defined (BL_CONFIG_LPUART_2) && BL_CONFIG_LPUART_2

#if defined(BL_CONFIG_LPUART_3) && BL_CONFIG_LPUART_3
void LPUART3_IRQHandler(void)
{
    if (LPUART_GetStatusFlags(LPUART3) & kLPUART_RxDataRegFullFlag)
    {
        uint8_t byte;
        byte = LPUART3->DATA;
        s_lpuart_byte_receive_callback(byte);
    }
}
#endif // #if defined (BL_CONFIG_LPUART_3) && BL_CONFIG_LPUART_3

#if defined(BL_CONFIG_LPUART_4) && BL_CONFIG_LPUART_4
void LPUART4_IRQHandler(void)
{
    if (LPUART_GetStatusFlags(LPUART4) & kLPUART_RxDataRegFullFlag)
    {
        uint8_t byte;
        byte = LPUART4->DATA;
        s_lpuart_byte_receive_callback(byte);
    }
}

#endif // #if defined (BL_CONFIG_LPUART_4) && BL_CONFIG_LPUART_4

//! @}

#endif // BL_CONFIG_LPUART
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
