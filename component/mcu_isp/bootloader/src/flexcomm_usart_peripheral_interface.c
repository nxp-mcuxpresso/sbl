/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
#include "peripherals_pinmux.h"
#include "fsl_flexcomm.h"
#include "fsl_usart.h"
#include "utilities/fsl_assert.h"
#include "microseconds/microseconds.h"

#if BL_CONFIG_FLEXCOMM_USART

//! @addtogroup flexcomm_usart_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static bool flexcomm_usart_poll_for_activity(const peripheral_descriptor_t *self);
static status_t flexcomm_usart_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
static void flexcomm_usart_full_shutdown(const peripheral_descriptor_t *self);

static status_t flexcomm_usart_write(const peripheral_descriptor_t *self,
                                     const uint8_t *buffer,
                                     uint32_t byteCount,
                                     uint32_t timeoutMs);

extern void FLEXCOMM_USART_SetSystemIRQ(uint32_t instance, PeripheralSystemIRQSetting set);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const peripheral_control_interface_t g_flexcommUsartControlInterface = {.pollForActivity =
                                                                            flexcomm_usart_poll_for_activity,
                                                                        .init = flexcomm_usart_full_init,
                                                                        .shutdown = flexcomm_usart_full_shutdown,
                                                                        .pump = 0 };

const peripheral_byte_inteface_t g_flexcommUsartByteInterface = {.init = NULL, .write = flexcomm_usart_write };

static serial_byte_receive_func_t s_flexcomm_usart_byte_receive_callback;

static bool g_flexcommUsartInitStatus[FSL_FEATURE_SOC_USART_COUNT] = { false }; // not initialized.

static const uint32_t g_flexcommUsartBaseAddr[] = USART_BASE_ADDRS;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/*!
 * This function will be called from the main peripheral detection loop to drive
 * the autobaud detect for this UART instance. If it is completed the UART is
 * fully initialized and ready to use.
 */
bool flexcomm_usart_poll_for_activity(const peripheral_descriptor_t *self)
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
        usart_config_t userConfig;
        uint32_t baseAddr = g_flexcommUsartBaseAddr[instance];

        USART_GetDefaultConfig(&userConfig);
        userConfig.baudRate_Bps = baud;
        userConfig.enableTx = true;
        userConfig.enableRx = true;

        if (USART_Init((USART_Type *)baseAddr, &userConfig, get_flexcomm_clock(instance)) == kStatus_Success)
        {
#if !BL_FEATURE_6PINS_PERIPHERAL
            // Configure selected pin as uart peripheral interface
            self->pinmuxConfig(instance, kPinmuxType_Peripheral);
#endif // !BL_FEATURE_6PINS_PERIPHERAL

            // Enable FLEXCOMM USART system interrupt
            FLEXCOMM_USART_SetSystemIRQ(instance, kPeripheralEnableIRQ);

            // Enable FLEXCOMM USART peripheral interrupt
            USART_EnableInterrupts((USART_Type *)baseAddr, kUSART_RxLevelInterruptEnable);

            // This was the byte pattern identified in autobaud detection, inform the command layer
            s_flexcomm_usart_byte_receive_callback(kFramingPacketStartByte);
            s_flexcomm_usart_byte_receive_callback(kFramingPacketType_Ping);

            // update pheripheral interface init status
            g_flexcommUsartInitStatus[instance] = true;

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
status_t flexcomm_usart_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    s_flexcomm_usart_byte_receive_callback = function;

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
void __attribute__((optimize("O0"))) flexcomm_usart_full_shutdown(const peripheral_descriptor_t *self)
#else
#if (defined(__ICCARM__))
#pragma optimize = none
#endif
#if (defined(__CC_ARM))
#pragma push
#pragma O0
#endif
void flexcomm_usart_full_shutdown(const peripheral_descriptor_t *self)
#endif
#else
void flexcomm_usart_full_shutdown(const peripheral_descriptor_t *self)
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

    if (g_flexcommUsartInitStatus[instance])
    {
        uint32_t baseAddr = g_flexcommUsartBaseAddr[instance];

        // Disable FLEXCOMM USART interrupt
        FLEXCOMM_USART_SetSystemIRQ(instance, kPeripheralDisableIRQ);

        // Reset FLEXCOMM USART registers
        USART_Deinit((USART_Type *)baseAddr);
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

status_t flexcomm_usart_write(const peripheral_descriptor_t *self,
                              const uint8_t *buffer,
                              uint32_t byteCount,
                              uint32_t timeoutMs)
{
    uint32_t instance;
#if USE_ONLY_UART(0)
    instance = 0;
#elif USE_ONLY_UART(1)
    instance = 1;
#else
    instance = self->instance;
#endif // USE_ONLY_UART(0)

    USART_Type *baseAddr = (USART_Type *)g_flexcommUsartBaseAddr[instance];

    uint64_t startTicks = microseconds_get_ticks();
    uint64_t timeOutTicks = microseconds_convert_to_ticks(timeoutMs * 1000);
    uint64_t endTicks = startTicks;
    uint64_t deltaTicks = 0;

    // USART_WriteBlocking((USART_Type *)baseAddr, buffer, byteCount);

    /* Check whether txFIFO is enabled */
    if (!(baseAddr->FIFOCFG & USART_FIFOCFG_ENABLETX_MASK))
    {
        return kStatus_Fail;
    }
    for (; byteCount > 0; byteCount--)
    {
        /* Loop until txFIFO get some space for new data */
        while (!(baseAddr->FIFOSTAT & USART_FIFOSTAT_TXNOTFULL_MASK))
        {
            endTicks = microseconds_get_ticks();
            deltaTicks = endTicks - startTicks;

            // Check timer roll over
            if (endTicks < startTicks)
            {
                deltaTicks = endTicks + (~startTicks) + 1;
            }

            if (timeOutTicks && (deltaTicks >= timeOutTicks))
            {
                return kStatus_Timeout;
            }
        }
        baseAddr->FIFOWR = *buffer;
        buffer++;
    }

    /* Wait to finish transfer */
    while (!(baseAddr->STAT & USART_STAT_TXIDLE_MASK))
    {
        endTicks = microseconds_get_ticks();
        deltaTicks = endTicks - startTicks;

        // Check timer roll over
        if (endTicks < startTicks)
        {
            deltaTicks = endTicks + (~startTicks) + 1;
        }

        if (timeOutTicks && (deltaTicks >= timeOutTicks))
        {
            return kStatus_Timeout;
        }
    }

    return kStatus_Success;
}

#if defined(BL_CONFIG_FLEXCOMM_USART_0)
/********************************************************************/
/*
 * UART0 IRQ Handler
 *
 */
void UART0_IRQHandler(void)
{
    uint32_t baseAddr = g_flexcommUsartBaseAddr[0];
    if (USART_GetStatusFlags((USART_Type *)baseAddr) & kUSART_RxFifoNotEmptyFlag)
    {
        uint8_t byte;
        byte = USART_ReadByte((USART_Type *)baseAddr);
        s_flexcomm_usart_byte_receive_callback(byte);
    }
}
#endif // !defined(BL_CONFIG_FLEXCOMM_USART_0)

//! @}

#endif // BL_CONFIG_FLEXCOMM_USART
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
