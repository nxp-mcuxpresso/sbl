/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* Copyright 2021 NXP
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
#include "fsl_flexcomm.h"
#include "fsl_usart.h"
#include "utilities/fsl_assert.h"

#if BL_CONFIG_FLEXCOMM_USART
static const IRQn_Type flexcomm_usart_irq_ids[FSL_FEATURE_SOC_USART_COUNT] = USART_IRQS;

void FLEXCOMM_USART_SetSystemIRQ(uint32_t instance, PeripheralSystemIRQSetting set)
{
    if (set == kPeripheralEnableIRQ)
    {
        NVIC_EnableIRQ(flexcomm_usart_irq_ids[instance]);
    }
    else
    {
        NVIC_DisableIRQ(flexcomm_usart_irq_ids[instance]);
    }
}
#endif // BL_CONFIG_FLEXCOMM_USART

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
