/*
* Copyright 2016-2021 NXP
*
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "bootloader_config.h"
#include "bootloader/bl_irq_common.h"
#include "autobaud/autobaud.h"
#include "packet/serial_packet.h"
#include "fsl_device_registers.h"
#include "fsl_lpuart.h"
#include "utilities/fsl_assert.h"

void LPUART_SetSystemIRQ(uint32_t instance, PeripheralSystemIRQSetting set)
{
    if (set == kPeripheralEnableIRQ)
    {
        NVIC_EnableIRQ(LPUART1_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(LPUART1_IRQn);
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
