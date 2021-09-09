/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "bootloader/bl_peripheral.h"
#include "bootloader/bl_context.h"
#include "microseconds/microseconds.h"

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#if (defined(DEBUG) || defined(_DEBUG) || defined(FORCE_DEBUG)) && !defined(DEBUG_PRINT_DISABLE)
static const char *const kPeripheralNames[] = {
    "UART", // kPeripheralType_UART
    "I2C",  // kPeripheralType_I2CSlave
    "SPI",  // kPeripheralType_SPISlave
    "CAN",  // kPeripheralType_CAN
    "HID",  // kPeripheralType_USB_HID
    "CDC",  // kPeripheralType_USB_CDC
    "DFU",  // kPeripheralType_USB_DFU
    "MSD",  // kPeripheralType_USB_MSC
    "SDIO", // kPeripheralType_SDIO_SLAVE
    "SAI",  // kPeripheralType_SAI_SLAVE
};
#endif // DEBUG

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if (defined(DEBUG) || defined(_DEBUG) || defined(FORCE_DEBUG)) && !defined(DEBUG_PRINT_DISABLE)
// See bl_peripheral.h for documentation on this function.
const char *get_peripheral_name(uint32_t peripheralTypeMask)
{
    uint32_t i;
    for (i = 0; i < ARRAY_SIZE(kPeripheralNames); ++i)
    {
        if (peripheralTypeMask & (1 << i))
        {
            return kPeripheralNames[i];
        }
    }

    return "Unknown peripheral";
}
#endif // #if (defined(DEBUG) || defined(_DEBUG) || defined(FORCE_DEBUG)) && !defined(DEBUG_PRINT_DISABLE)

// See bl_peripheral.h for documentation on this function.
peripheral_descriptor_t const *get_active_peripheral(uint32_t timeoutMs)
{
    peripheral_descriptor_t const *peripheral;
    peripheral_descriptor_t const *activePeripheral = NULL;
    bootloader_configuration_data_t *configurationData =
        &g_bootloaderContext.propertyInterface->store->configurationData;

    // Bring up all the peripherals
    for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
    {
        // Check that the peripheral is enabled in the user configuration data
        if (configurationData->enabledPeripherals & peripheral->typeMask)
        {
            assert(peripheral->controlInterface->init);

            debug_printf("Initing %s\r\n", get_peripheral_name(peripheral->typeMask));
            peripheral->controlInterface->init(peripheral, peripheral->packetInterface->byteReceivedCallback);
        }
    }

    uint64_t timeoutTicks = timeoutMs * microseconds_convert_to_ticks(
                                            1000); // The number of ticks we will wait for timeout, 0 means no timeout
    uint64_t lastTicks = microseconds_get_ticks(); // Value of our last recorded ticks second marker

    // Wait for a peripheral to become active
    while (activePeripheral == NULL)
    {
#if !BL_FEATURE_TIMEOUT
        // If timeout is enabled, check to see if we've exceeded it.
        if (timeoutTicks)
        {
            // Note that we assume that the tick counter won't overflow and wrap back to 0.
            // The timeout value is only up to 65536 milliseconds, and the tick count starts
            // at zero when when inited the microseconds driver just a few moments ago.
            uint64_t elapsedTicks = microseconds_get_ticks() - lastTicks;

            // Check if the elapsed time is longer than the timeout.
            if (elapsedTicks >= timeoutTicks)
            {
                break;
            }
        }
#endif // !BL_FEATURE_TIMEOUT
        // Traverse through all the peripherals
        for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
        {
            // Check that the peripheral is enabled in the user configuration data
            if (configurationData->enabledPeripherals & peripheral->typeMask)
            {
                assert(peripheral->controlInterface->pollForActivity);

                if (peripheral->controlInterface->pollForActivity(peripheral))
                {
                    debug_printf("%s is active\r\n", get_peripheral_name(peripheral->typeMask));

                    activePeripheral = peripheral;
                    break;
                }
            }
        }
    }

    if (activePeripheral != NULL)
    {
        // Shut down all non active peripherals
        for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
        {
            // Check that the peripheral is enabled in the user configuration data
            if (configurationData->enabledPeripherals & peripheral->typeMask)
            {
                if (activePeripheral != peripheral)
                {
                    debug_printf("Shutting down %s\r\n", get_peripheral_name(peripheral->typeMask));

                    assert(peripheral->controlInterface->shutdown);
                    peripheral->controlInterface->shutdown(peripheral);
                }
            }
        }
    }

    return activePeripheral;
}
