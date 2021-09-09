/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*
 * @file microseconds_tstmr.c
 * @brief Microseconds tstmr timer driver source file
 *
 * Notes: The driver configure TSTMR as lifetime timer
 */
#include "microseconds/microseconds.h"
#include <stdarg.h>
#include "bootloader_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// Below MACROs are defined in order to keep this driver compabtile among all targets.
#if defined(TSTMR0_BASE)
#define TSTMR_BASE TSTMR0_BASE
#define TSTMRx TSTMR0
#elif defined(TSTMRA_BASE)
#define TSTMR_BASE TSTMRA_BASE
#define TSTMRx TSTMRA
#endif

#if defined(TSTMR_L_VALUE_MASK)
#define TSTMR_LOW_REG32 (TSTMRx->L)
#define TSTMR_HIGH_REG32 (TSTMRx->H)
#elif defined(TSTMR_LOW_VALUE_MASK)
#define TSTMR_LOW_REG32 (TSTMRx->LOW)
#define TSTMR_HIGH_REG32 (TSTMRx->HIGH)
#endif

enum
{
    kFrequency_1MHz = 1000000UL,
    kFrequency_8MHz = 8000000UL
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
static uint32_t s_tstmrModuleClock;
uint32_t s_tickPerMicrosecond; //!< This value equal to ticks per microseconds

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Initialize timer facilities.
//!
//! Initialize and start the timer facilities using the TSTMR.
void microseconds_init(void)
{
#if defined(FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_1MHZ) && FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_1MHZ
    s_tstmrModuleClock = kFrequency_1MHz;
#elif defined(FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_8MHZ) && FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_8MHZ
    s_tstmrModuleClock = kFrequency_8MHz;
#else
#error "Unknown TSTMR clock frequency"
#endif

    s_tickPerMicrosecond = s_tstmrModuleClock / kFrequency_1MHz;

    // Make sure this value is greater than 0
    if (!s_tickPerMicrosecond)
    {
        s_tickPerMicrosecond = 1;
    }
}

//! @brief Shutdown the microsecond timer
void microseconds_shutdown(void)
{
}

//! @brief Read back running tick count
uint64_t microseconds_get_ticks(void)
{
    // Note: the software must follow the read sequence (must read the TSTMR_L first,
    //  followed by a read to TSTMR_H to retrieve the complete value.) for correctly reading the TSTMR value.
    uint32_t low = TSTMR_LOW_REG32;
    uint64_t high = TSTMR_HIGH_REG32;

    return ((high << 32) | low);
    // return *(volatile uint64_t*)(TSTMR_BASE);
}

//! @brief Returns the conversion of ticks to actual microseconds
//!        This is used to seperate any calculations from getting a timer
//         value for speed critical scenarios
uint32_t microseconds_convert_to_microseconds(uint32_t ticks)
{
    // return the total ticks divided by the number of Mhz the system clock is at to give microseconds
    return (ticks / s_tickPerMicrosecond); //!< Assumes system clock will never be < 0.125 Mhz
}

//! @brief Returns the conversion of microseconds to ticks
uint64_t microseconds_convert_to_ticks(uint32_t microseconds)
{
    return ((uint64_t)microseconds * s_tickPerMicrosecond);
}

//! @brief Delay specified time
//!
//! @param us Delay time in microseconds unit
void microseconds_delay(uint32_t us)
{
    uint64_t currentTicks = microseconds_get_ticks();

    //! The clock value in Mhz = ticks/microsecond
    uint64_t ticksNeeded = ((uint64_t)us * s_tickPerMicrosecond) + currentTicks;
    while (microseconds_get_ticks() < ticksNeeded)
    {
        ;
    }
}

//! @brief Gets the clock value used for microseconds driver
uint32_t microseconds_get_clock(void)
{
    return s_tstmrModuleClock;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
