/*
 * The Clear BSD License
 * Copyright 2016-2021 NXP
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
#include "microseconds/microseconds.h"
#include "property/property.h"
#include "target_config.h"
#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
#include "fsl_flexspi.h"
#endif
#include "utilities/fsl_assert.h"
//#include "ocotp/fsl_ocotp.h"
#include "fusemap.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#ifndef FREQ_392MHz
#define FREQ_392MHz (392UL * 1000 * 1000)
#endif
#ifndef FREQ_528MHz
#define FREQ_528MHz (528UL * 1000 * 1000)
#endif
#ifndef FREQ_24MHz
#define FREQ_24MHz (24UL * 1000 * 1000)
#endif
#ifndef FREQ_480MHz
#define FREQ_480MHz (480UL * 1000 * 1000)
#endif

enum
{
    kMaxIpgClock = 144000000UL,
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
void clock_setup(void);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See bootloader_common for documentation on this function.
void configure_clocks(bootloader_clock_option_t option)
{
    if (option == kClockOption_EnterBootloader)
    {
        clock_setup();
    }
}

bool usb_clock_init(uint32_t controllerId)
{
    // Enable clock gate
    CCM->CCGR6 |= CCM_CCGR6_CG0_MASK;

    // Enable USB Clocks
    CCM_ANALOG->PLL_USB1_SET = CCM_ANALOG_PLL_USB1_EN_USB_CLKS_MASK;

    // Clear SFTRST
    USBPHY->CTRL_CLR = USBPHY_CTRL_SFTRST_MASK;

    // Clear Clock gate
    USBPHY->CTRL_CLR = USBPHY_CTRL_CLKGATE_MASK;

    // Clear power down register
    USBPHY->PWD = 0;

    // Disable Charger Detect
    USB_ANALOG->INSTANCE[0].CHRG_DETECT |= (USB_ANALOG_CHRG_DETECT_EN_B_MASK | USB_ANALOG_CHRG_DETECT_CHK_CHRG_B_MASK);

    USB->USBCMD &= (uint32_t)~USBHS_USBCMD_RS_MASK;

    return true;
}

void clock_setup(void)
{
    uint32_t clock_divider = 1;
    uint32_t fuse_div = 0;
    uint32_t core_clock = 0;

    CLOCK_SetXtal0Freq(CPU_XTAL_CLK_HZ);
    // Get the Boot Up CPU Clock Divider
    // 00b = LPB disabled
    // 01b = def frequency
    // 10b = divide by 2
    // 11b = divide by 4
    fuse_div = ROM_OCOTP_LPB_BOOT_VALUE();
    clock_divider = fuse_div ? (1 << (fuse_div - 1)) : 1;
    if (clock_divider)
    {
        core_clock = FREQ_392MHz / clock_divider;
    }

    // If clock is not configured, configure clock first, otherwise, just update SystemCoreClock
    //if (CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_BYPASS_MASK)
    {
        CCM_ANALOG->PLL_SYS_CLR = CCM_ANALOG_PLL_SYS_POWERDOWN_MASK;
        while ((CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_LOCK_MASK) == 0)
        {
        }
        CCM_ANALOG->PLL_SYS_SET = CCM_ANALOG_PLL_SYS_BYPASS_MASK;

        /*
         * PLL_SYS / PLL 2 / PLL 528
         * PFD0 = 396MHz
         * PFD1 = 396MHz
         * PFD2 = 500MHz
         * PFD3 = 396MHz
         */
        CCM_ANALOG->PFD_528 =
            (CCM_ANALOG->PFD_528 & (~(CCM_ANALOG_PFD_528_PFD0_FRAC_MASK | CCM_ANALOG_PFD_528_PFD1_FRAC_MASK |
                                      CCM_ANALOG_PFD_528_PFD2_FRAC_MASK | CCM_ANALOG_PFD_528_PFD3_FRAC_MASK))) |
            CCM_ANALOG_PFD_528_PFD0_FRAC(24) | CCM_ANALOG_PFD_528_PFD1_FRAC(24) | CCM_ANALOG_PFD_528_PFD2_FRAC(19) |
            CCM_ANALOG_PFD_528_PFD3_FRAC(24);

        CCM_ANALOG->PLL_USB1 =
            CCM_ANALOG_PLL_USB1_DIV_SELECT(0) | CCM_ANALOG_PLL_USB1_POWER(1) | CCM_ANALOG_PLL_USB1_ENABLE(1);
        while ((CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_LOCK_MASK) == 0)
        {
        }
        CCM_ANALOG->PLL_USB1_SET = CCM_ANALOG_PLL_USB1_BYPASS_MASK;

        /*
         * PLL_USB / PLL 3 / PLL 480
         * PFD0 = 247MHz
         * PFD1 = 247MHz  - LPSPI CLOCK Source
         * PFD2 = 332MHz
         * PFD3 = 392MHz
         */
        CCM_ANALOG->PFD_480 =
            (CCM_ANALOG->PFD_480 & (~(CCM_ANALOG_PFD_480_PFD0_FRAC_MASK | CCM_ANALOG_PFD_480_PFD1_FRAC_MASK |
                                      CCM_ANALOG_PFD_480_PFD2_FRAC_MASK | CCM_ANALOG_PFD_480_PFD3_FRAC_MASK))) |
            CCM_ANALOG_PFD_480_PFD0_FRAC(35) | CCM_ANALOG_PFD_480_PFD1_FRAC(35) | CCM_ANALOG_PFD_480_PFD2_FRAC(26) |
            CCM_ANALOG_PFD_480_PFD3_FRAC(22);

        uint32_t ipg_divider = (core_clock + kMaxIpgClock - 1) / kMaxIpgClock;

        CCM->CBCDR =
            (CCM->CBCDR & (~(CCM_CBCDR_PERIPH_CLK_SEL_MASK | CCM_CBCDR_AHB_PODF_MASK | CCM_CBCDR_IPG_PODF_MASK))) |
            CCM_CBCDR_PERIPH_CLK_SEL(0) | CCM_CBCDR_AHB_PODF(clock_divider - 1) | CCM_CBCDR_IPG_PODF(ipg_divider - 1);

        // LPUART clock configuration, peripheral clock 40MHz
        CCM->CSCDR1 = (CCM->CSCDR1 & (~(CCM_CSCDR1_UART_CLK_SEL_MASK | CCM_CSCDR1_UART_CLK_PODF_MASK))) |
                      CCM_CSCDR1_UART_CLK_PODF(1);

        // Configure the clock source for core modules
        CCM->CBCMR = ((CCM->CBCMR & ~(CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK)) | (CCM_CBCMR_PRE_PERIPH_CLK_SEL(1)));

        // Finally enable PLLs
        CCM_ANALOG->PLL_SYS &= ~CCM_ANALOG_PLL_SYS_BYPASS_MASK;
        CCM_ANALOG->PLL_USB1 &= ~CCM_ANALOG_PLL_USB1_BYPASS_MASK;
    }

    //SystemCoreClock = core_clock;
    
    SystemCoreClockUpdate();
}

// Get OCOTP clock
uint32_t get_ocotp_clock(void)
{
    uint32_t ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
    return SystemCoreClock / ahbBusDivider;
}

#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
//!@brief Configure clock for FlexSPI peripheral
/*
void flexspi_clock_config(uint32_t instance, flexspi_serial_clk_freq_t freq, uint32_t sampleClkMode)
{
    uint32_t pfd480 = 0;
    uint32_t cscmr1 = 0;
    uint32_t frac = 0;
    uint32_t podf = 0;

    typedef struct _flexspi_clock_param
    {
        uint8_t frac;
        uint8_t podf;
    } flexspi_clock_param_t;

    const flexspi_clock_param_t k_sdr_clock_config[kFlexSpiSerialClk_200MHz + 1] = {
        // Reserved, 30MHz     50MHz     60MHz        75MHz    80MHz       100MHz   133MHz       166MHz   200MHz
        { 0, 0 }, { 34, 8 }, { 22, 8 }, { 24, 6 }, { 30, 4 }, { 18, 6 }, { 14, 6 }, { 17, 4 }, { 26, 2 }, { 22, 2 }
    };
    const flexspi_clock_param_t k_ddr_clock_config[kFlexSpiSerialClk_200MHz + 1] = {
        // Reserved, 30MHz,  50MHz,       60MHz,      75MHz,   80Mhz,   100MHz,      133MHz,   166MHz,     200MHz
        { 0, 0 }, { 24, 6 }, { 22, 4 }, { 12, 6 }, { 30, 2 }, { 18, 3 }, { 22, 2 }, { 33, 1 }, { 26, 1 }, { 22, 1 }
    };

    do
    {
        if ((sampleClkMode != kFlexSpiClk_SDR) && (sampleClkMode != kFlexSpiClk_DDR))
        {
            break;
        }

        pfd480 = CCM_ANALOG->PFD_480 & (~CCM_ANALOG_PFD_480_PFD0_FRAC_MASK);
        cscmr1 = CCM->CSCMR1 & (~CCM_CSCMR1_FLEXSPI_PODF_MASK);

        // Note: Per ANALOG IP Owner's recommendation, FRAC should be even number,
        //       PODF should be even nubmer as well if the divider is greater than 1

        const flexspi_clock_param_t *flexspi_config_array = NULL;
        if (sampleClkMode == kFlexSpiClk_SDR)
        {
            flexspi_config_array = &k_sdr_clock_config[0];
        }
        else
        {
            flexspi_config_array = &k_ddr_clock_config[0];
        }

        if (freq >= kFlexSpiSerialClk_30MHz)
        {
            if (freq > kFlexSpiSerialClk_200MHz)
            {
                freq = kFlexSpiSerialClk_30MHz;
            }

            frac = flexspi_config_array[freq].frac;
            podf = flexspi_config_array[freq].podf;

            pfd480 |= CCM_ANALOG_PFD_480_PFD0_FRAC(frac);
            cscmr1 |= CCM_CSCMR1_FLEXSPI_PODF(podf - 1);

            FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
            flexspi_clock_gate_disable(instance);

            if (pfd480 != CCM_ANALOG->PFD_480)
            {
                CCM_ANALOG->PFD_480 = pfd480;
            }
            if (cscmr1 != CCM->CSCMR1)
            {
                CCM->CSCMR1 = cscmr1;
            }
            flexspi_clock_gate_enable(instance);
            FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
        }
        else
        {
            // Do nothing
        }
    } while (0);
}

//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable(uint32_t instance)
{
    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;
}

//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable(uint32_t instance)
{
    CCM->CCGR6 &= (uint32_t)~CCM_CCGR6_CG5_MASK;
}

//!@brief Get Clock for FlexSPI peripheral
#define flexspi_get_clock USE_ROMAPI_flexspi_get_clock
status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    uint32_t ahbBusDivider;
    uint32_t serialRootClkDivider;
    uint32_t arm_clock = SystemCoreClock;

    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = SystemCoreClock;
            break;
        // case kFlexSpiClock_AhbClock:
        // {
        //     // Note: In I.MXRT_512, actual AHB clock is IPG_CLOCK_ROOT
        //     ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
        //     clockFrequency = arm_clock / ahbBusDivider;
        // }
        // break;
        case kFlexSpiClock_SerialRootClock:
        {
            switch ((CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK) >> CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT)
            {
                case 0: // PLL_SYS
                    clockFrequency = FREQ_528MHz;
                    if (CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_BYPASS_MASK)
                    {
                        clockFrequency = FREQ_24MHz;
                    }
                    break;
                case 1: // PLL_USB1
                    clockFrequency = FREQ_480MHz;
                    if (CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_BYPASS_MASK)
                    {
                        clockFrequency = FREQ_24MHz;
                    }
                    break;
                case 2: // PFD_528_PFD2
                    if (CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_BYPASS_MASK)
                    {
                        clockFrequency = FREQ_24MHz;
                    }
                    else
                    {
                        uint32_t pfd = (CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD2_FRAC_MASK) >>
                                       CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT;
                        clockFrequency = FREQ_528MHz / pfd * 18;
                    }
                    break;
                case 3: // PFD_480_PFD0
                    if (CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_BYPASS_MASK)
                    {
                        clockFrequency = FREQ_24MHz;
                    }
                    else
                    {
                        uint32_t pfd = (CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >>
                                       CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT;
                        clockFrequency = FREQ_528MHz / pfd * 18;
                    }
                    break;
            }

            uint32_t flexspi_podf_divider =
                1 + ((CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_PODF_MASK) >> CCM_CSCMR1_FLEXSPI_PODF_SHIFT);

            clockFrequency /= flexspi_podf_divider;
        }
        break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }
    *freq = clockFrequency;

    return status;
}
#undef flexspi_get_clock

// Get max supported Frequency in this SoC
status_t flexspi_get_max_supported_freq(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if ((instance != 0) || (freq == NULL))
        {
            break;
        }

        *freq = (133UL * 1000 * 1000);
        status = kStatus_Success;

    } while (0);

    return status;
}
*/
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE

//! @brief Gets the clock value used for microseconds driver
uint32_t microseconds_get_clock(void)
{
    // Get PIT clock source
    uint32_t ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
    uint32_t periphDivider = ((CCM->CSCMR1 & CCM_CSCMR1_PERCLK_PODF_MASK) >> CCM_CSCMR1_PERCLK_PODF_SHIFT) + 1;
    return SystemCoreClock / ahbBusDivider / periphDivider;
}

//! @brief Get BUS clock value
uint32_t get_bus_clock(void)
{
    uint32_t ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
    return SystemCoreClock / ahbBusDivider;
}

void flexspi_sw_delay_us(uint64_t us)
{
    while (us--)
    {
        microseconds_delay(1);
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
