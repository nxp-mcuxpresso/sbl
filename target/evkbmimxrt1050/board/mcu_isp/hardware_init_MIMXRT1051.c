/*
 * The Clear BSD License
 * Copyright 2017-2021 NXP
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "fsl_lpuart.h"
#include "utilities/fsl_assert.h"
#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
#include "fsl_flexspi.h"
#include "flexspi_nor_flash.h"
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
#include "fusemap.h"
#include "peripherals_pinmux.h"
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Codes
 ******************************************************************************/

//! @brief Return uart clock frequency according to instance
uint32_t get_uart_clock(uint32_t instance)
{
    uint32_t periphDivider = ((CCM->CSCDR1 & CCM_CSCDR1_UART_CLK_PODF_MASK) >> CCM_CSCDR1_UART_CLK_PODF_SHIFT) + 1;  
  
    uint32_t lpuart_clock = 80000000UL / periphDivider;

    return lpuart_clock;
}

bool is_boot_pin_asserted(void)
{
    // Boot pin for Flash only target
    return false;
}

void dummy_byte_callback(uint8_t byte)
{
    (void)byte;
}

void debug_init(void)
{
}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__

void update_available_peripherals()
{
}

void update_specific_properties(void)
{
}

void init_hardware(void)
{
    //SCB_InvalidateDCache();
    SCB_DisableDCache();
    __ISB();
    __DSB();

    // Configure clocks.
    configure_clocks(kClockOption_EnterBootloader);

    CLOCK_EnableClock(kCLOCK_UsbOh3);
}

void deinit_hardware(void)
{
    CLOCK_DisableClock(kCLOCK_UsbOh3);

    SCB_EnableDCache();
    __ISB();
    __DSB();
}

//!@brief Get the hab status.
habstatus_option_t get_hab_status(void)
{
    if (ROM_OCOTP_SEC_CONFIG_VALUE() & 0x2)
    {
        return kHabStatus_Close;
    }
    else
    {
        return kHabStatus_Open;
    }
}

int memset_s(void *s, size_t smax, int c, size_t n)
{
    if (n > smax)
    {
        return 1;
    }
    memset(s, c, n);
    return 0;
}

bool isp_cleanup_exit(bool *isInfiniteIsp)
{
    uint32_t flag = IOMUXC_SNVS_GPR->GPR0;
    switch (flag)
    {
        case CLEANUP_SBL_TO_ISP:
            *isInfiniteIsp = true;
            flag = 0x0;
            break;
        case CLEANUP_ISP_TO_SBL:
        default:
            break;
    }
    IOMUXC_SNVS_GPR->GPR0 = 0x0;
    return flag;
}

void isp_cleanup_enter(uint32_t flag)
{
    IOMUXC_SNVS_GPR->GPR0 = flag;
    NVIC_SystemReset();
}


////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
