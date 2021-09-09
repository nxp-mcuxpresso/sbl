/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "autobaud/autobaud.h"
#include "microseconds/microseconds.h"
#include "bootloader_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum _autobaud_counts
{
    //! the number of falling edge transitions being counted
    //! for 0x5A
    kFirstByteRequiredFallingEdges = 4,
    //! the number of falling edge transitions being counted
    //! for 0xA6
    kSecondByteRequiredFallingEdges = 3,
    //! the number of bits being measured for the baud rate
    //! for 0x5A we have the start bit + 7 bits to the last falling edge = 8 bits
    kNumberOfBitsForFirstByteMeasured = 8,
    //! for 0xA6 we have the start bit + 6 bits to the last falling edge = 7 bits
    kNumberOfBitsForSecondByteMeasured = 7,
    //! Time in microseconds that we will wait in between toggles before restarting detection
    //! Make this value 8 bits at 100 baud worth of time = 80000 microseconds
    kMaximumTimeBetweenFallingEdges = 80000,
    //! Autobaud baud step size that our calculation will be rounded to, this is to ensure
    //! that we can use a valid multiplier in the UART configuration which runs into problems
    //! at higher baud rates with slightly off baud rates (e.g. if we measure 115458 vs 115200)
    //! a calculation of UartClock at 48MHz gives an SBR calculation of 48000000/ (16 * 115458) = 25
    //! giving a baud rate calculation of 48000000/(16 * 25) = 120000 baud
    //! which is out of spec and fails
    kAutobaudStepSize = 1200
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
void instance_transition_callback(uint32_t instance);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

static uint32_t s_transitionCount;
static uint64_t s_firstByteTotalTicks;
static uint64_t s_secondByteTotalTicks;
static uint64_t s_lastToggleTicks;
static uint64_t s_ticksBetweenFailure;
static uint32_t s_instanceMeasured;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void autobaud_init(uint32_t instance)
{
    s_transitionCount = 0;
    s_firstByteTotalTicks = 0;
    s_secondByteTotalTicks = 0;
    s_lastToggleTicks = 0;
    s_instanceMeasured = 0;
    s_ticksBetweenFailure = microseconds_convert_to_ticks(kMaximumTimeBetweenFallingEdges);
    enable_autobaud_pin_irq(instance, instance_transition_callback);
}

void autobaud_deinit(uint32_t instance)
{
    disable_autobaud_pin_irq(instance);
}

status_t autobaud_get_rate(uint32_t instance, uint32_t *rate)
{
    if ((s_transitionCount == (kFirstByteRequiredFallingEdges + kSecondByteRequiredFallingEdges)) &&
        (instance == s_instanceMeasured))
    {
        uint32_t calculatedBaud =
            (microseconds_get_clock() * (kNumberOfBitsForFirstByteMeasured + kNumberOfBitsForSecondByteMeasured)) /
            (uint32_t)(s_firstByteTotalTicks + s_secondByteTotalTicks);

        // Round the rate to the nearest step size
        // rounded = stepSize * (value/stepSize + .5)
        // multiplying by 10 since we can't work with floats
        *rate = ((((calculatedBaud * 10) / kAutobaudStepSize) + 5) / 10) * kAutobaudStepSize;

        return kStatus_Success;
    }
    else
    {
        // no baud rate yet/inactive
        return kStatus_Fail;
    }
}

void instance_transition_callback(uint32_t instance)
{
    uint64_t ticks = microseconds_get_ticks();
    s_transitionCount++;

    uint64_t delta = ticks - s_lastToggleTicks;

    // The last toggle was longer than we allow so treat this as the first one
    if (delta > s_ticksBetweenFailure)
    {
        s_transitionCount = 1;
    }

    switch (s_transitionCount)
    {
        case 1:
            // This is our first falling edge, store the initial ticks temporarily in firstByteTicks
            // and save the instance that we are measuring
            s_firstByteTotalTicks = ticks;
            s_instanceMeasured = instance;
            break;

        case kFirstByteRequiredFallingEdges:
            // We reached the end of our measurable first byte, subtract the current ticks from the initial
            // first byte ticks
            s_firstByteTotalTicks = ticks - s_firstByteTotalTicks;
            break;

        case (kFirstByteRequiredFallingEdges + 1):
            // We hit our first falling edge of the second byte, store the initial ticks temporarily in secondByteTicks
            s_secondByteTotalTicks = ticks;
            break;

        case (kFirstByteRequiredFallingEdges + kSecondByteRequiredFallingEdges):
            // We reached the end of our measurable second byte, subtract the current ticks from the initial
            // second byte ticks
            s_secondByteTotalTicks = ticks - s_secondByteTotalTicks;
            disable_autobaud_pin_irq(instance);
            break;
    }

    s_lastToggleTicks = ticks;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
