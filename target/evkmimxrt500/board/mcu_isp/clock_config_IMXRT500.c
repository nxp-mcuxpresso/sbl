/*
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "bootloader/bl_log.h"
#include "fsl_clock.h"
#include "fsl_device_registers.h"
#if BL_FEATURE_FLEXSPI_NOR_MODULE
#include "fsl_flexspi.h"
#endif
#include "fusemap.h"
#include "microseconds/microseconds.h"
#include "property/property.h"
#include "target_config.h"
#include "utilities/fsl_assert.h"
#include "bootloader/bootloader.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
typedef enum boot_power
{
    kBootPower_Normal, //!< Boot at 96MHz
    kBootPower_High    //!< Boot at 198MHz, high performance, high power consumption
} boot_power_t;

enum
{
    kFreq_1MHz = 1000000u,
    kFreq_4MHz = 4u * kFreq_1MHz,
    kFreq_24MHz = 24u * kFreq_1MHz,
    kFreq_48MHz = 48u * kFreq_1MHz,
    kFreq_44MHz = 44u * kFreq_1MHz,
    kFreq_192MHz = 192u * kFreq_1MHz,
    kFreq_198MHz = 198u * kFreq_1MHz,
    kFreq_264MHz = 264u * kFreq_1MHz,
    kFreq_16MHz = 16u * kFreq_1MHz,
    kFreq_96MHz = 96u * kFreq_1MHz,
    kFreq_240MHz = 240u * kFreq_1MHz,
    kFreq_133MHz = 133u * kFreq_1MHz,
    kFreq_132MHz = 132u * kFreq_1MHz,
    kFreq_144MHz = 144u * kFreq_1MHz,
    kFreq_166MHz = 166u * kFreq_1MHz,
    kFreq_200MHz = 200u * kFreq_1MHz,
    kDefaultUsbPllLockTimeUs = 200000u,

};

enum
{
    MAINCLKSELA_1M_LPOSC = 0,
    MAINCLKSELA_FRO_DIV, // Div from FRO192M (96M,48M,24M,12M)
    MAINCLKSELA_OSC_CLK,
    MAINCLKSELA_FRO_192M,
};

enum
{
    PLL528_CLKSRC_FRO_24M = 0,
    PLL528_CLKSRC_OSC,
};

enum
{
    MAINCLKSELB_SYSCLK = 0,
    MAINCLKSELB_MAIN_PLL_CLK, // MAIN PLL PFD0
    MAINCLKSELB_32K_CLK
};

enum
{
    SYSOSCSEL_XTAL_CLK = 0,
    SYSOSCSEL_CLKIN_CLK,
};

enum
{
    USBHSFCLKSEL_OSC_CLK = 0,
    USBHSFCLKSEL_MAIN_CLK,
    USBHSFCLKSEL_FRO_24M
};

enum
{
    kFlexSpiClockSrc_MainClk = 0,
    kFlexSpiClockSrc_MainPllClk,
    kFlexSpiClockSrc_Aux0PllClk,
    kFlexSpiClockSrc_FRO192M_Clk,
    kFlexSpiClockSrc_Aux1PllClk,
};

#if defined BL_TARGET_FPGA
uint32_t busClock = 1000000u; //! 6MHz default bus clock
#endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
static secure_bool_t isOscInited = kSecure_False;
static boot_power_t get_boot_power(void);
static void boot_set_clocks(boot_power_t boot_power);
static void init_syspll(uint32_t clk_src, uint32_t src_clk_freq);
static void init_sysosc(void);
static void delay_us_sw(uint32_t us);
extern void pmc_apply_cfg(void);
status_t flexspi_nor_set_clock_source(uint32_t clockSource);
void init_pll_use_xtal(void);
////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void delay_us_sw(uint32_t us)
{
    uint32_t ticksPerUs = SystemCoreClock / kFreq_1MHz;

    while (us--)
    {
        volatile uint32_t delayTicks = 1 + ticksPerUs / 4;
        while (--delayTicks)
        {
            __NOP();
        }
    }
}

enum
{
    kClockFreq_16MHz = 0,
    kClockFreq_19MHz,
    kClockFreq_24MHz,
    kClockFreq_26MHz,
    kClockFreq_30MHz,
} src_clk_freq;

void init_syspll(uint32_t clk_src, uint32_t src_clk_freq)
{
    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_SYSPLLLDO_PD_MASK | SYSCTL0_PDRUNCFG0_SYSPLLANA_PD_MASK;
    // Add a delay to make sure the osc_clk is ready before change the
    // syspll clock source to osc_clk or PLL will not lock up.
    delay_us_sw(100u);
    if (PLL528_CLKSRC_FRO_24M != clk_src)
    {
        // Reference clock change from  FRO24M to OSC CLK,needs an power down sequence
        SYSCTL0->PDRUNCFG0_SET = SYSCTL0_PDRUNCFG0_SYSPLLLDO_PD_MASK | SYSCTL0_PDRUNCFG0_SYSPLLANA_PD_MASK;
        CLKCTL0->SYSPLL0CLKSEL = clk_src;
        delay_us_sw(100u);
        SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_SYSPLLLDO_PD_MASK | SYSCTL0_PDRUNCFG0_SYSPLLANA_PD_MASK;
        delay_us_sw(100u);
    }
    else
    {
        CLKCTL0->SYSPLL0CLKSEL = clk_src;
    }

    if (kClockFreq_24MHz == src_clk_freq)
    {
        CLKCTL0->SYSPLL0NUM = 0x0u;
        CLKCTL0->SYSPLL0DENOM = 0x01u;
        // Configure to 528M, assert HOLDRINGOFF in the first half of lock time
        CLKCTL0->SYSPLL0CTL0 = CLKCTL0_SYSPLL0CTL0_MULT(22u) | CLKCTL0_SYSPLL0CTL0_HOLDRINGOFF_ENA_MASK;
        delay_us_sw(CLKCTL0->SYSPLL0LOCKTIMEDIV2 / 2);
        // De-assert HOLDRINGOFF in the second half of lock time
        CLKCTL0->SYSPLL0CTL0 &= (uint32_t)~CLKCTL0_SYSPLL0CTL0_HOLDRINGOFF_ENA_MASK;
        delay_us_sw(CLKCTL0->SYSPLL0LOCKTIMEDIV2 / 2);
    }
    else if (kClockFreq_16MHz == src_clk_freq)
    {
        CLKCTL0->SYSPLL0NUM = 0x0u;
        CLKCTL0->SYSPLL0DENOM = 0x01u;
        // Configure to 528M, assert HOLDRINGOFF in the first half of lock time
        CLKCTL0->SYSPLL0CTL0 = CLKCTL0_SYSPLL0CTL0_MULT(33u) | CLKCTL0_SYSPLL0CTL0_HOLDRINGOFF_ENA_MASK;
        delay_us_sw(CLKCTL0->SYSPLL0LOCKTIMEDIV2 / 2);
        // De-assert HOLDRINGOFF in the second half of lock time
        CLKCTL0->SYSPLL0CTL0 &= (uint32_t)~CLKCTL0_SYSPLL0CTL0_HOLDRINGOFF_ENA_MASK;
        delay_us_sw(CLKCTL0->SYSPLL0LOCKTIMEDIV2 / 2);
    }
}
void init_sysosc(void)
{
    if (isOscInited == kSecure_False)
    {
        SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_SYSXTAL_PD_MASK;
        __DSB();
        __ISB();
        CLKCTL0->SYSOSCCTL0 = CLKCTL0_SYSOSCCTL0_LP_ENABLE(1) | CLKCTL0_SYSOSCCTL0_BYPASS_ENABLE(0);

        CLKCTL0->SYSOSCBYPASS = CLKCTL0_SYSOSCBYPASS_SEL(SYSOSCSEL_XTAL_CLK);

        // Wait until OSC gets stable
//        uint32_t sysOscStableUsFuseIdx = 14;
        uint32_t sysOscStableUsFuseValue = 0;
        /*
        status_t status = otp_fuse_read(sysOscStableUsFuseIdx, &sysOscStableUsFuseValue);

        if (status != kStatus_Success)
        {
            go_fatal_mode();
        }
        */

        delay_us_sw(sysOscStableUsFuseValue);
        isOscInited = kSecure_True;
    }
}
boot_power_t get_boot_power(void)
{
    boot_power_t bootPower = kBootPower_Normal;
    if (OTP_BOOTSPEED_VALUE())
    {
        bootPower = kBootPower_High;
    }

    return bootPower;
}

void boot_set_clocks(boot_power_t boot_power)
{
  
#ifndef BL_TARGET_FPGA

    if (!OTP_BOOT_CLOCK_USE_XTAL_TO_PLL_VALUE())
    {
        if (boot_power == kBootPower_High)
        {
            debug_printf("%s, PLL will use FRO24M as input", __func__);

            debug_printf("%s, FFRO enabled", __func__);
            // Enable the FRO192M FRO48M FRO24M(Main PLL source)
            CLKCTL0->FRODIVOEN = CLKCTL0_FRODIVOEN_FRO_DIV1_O_EN_MASK | CLKCTL0_FRODIVOEN_FRO_DIV4_O_EN_MASK |
                                  CLKCTL0_FRODIVOEN_FRO_DIV8_O_EN_MASK;

            // Configure Clock to certain state before swithching to PLL
            CLKCTL0->MAINCLKSELB = MAINCLKSELB_SYSCLK;
            CLKCTL0->MAINCLKSELA = MAINCLKSELA_FRO_192M;

            // Increase the core voltage to 1.138v before setting the PLL
            PMC->RUNCTRL = PMC_RUNCTRL_CORELVL(0x32);
            pmc_apply_cfg();

            // PLL_OUTPUT = 24MHz * (22 + 0 / 1) = 528MHz
            SystemCoreClock = kFreq_48MHz;
            init_syspll(PLL528_CLKSRC_FRO_24M, kClockFreq_24MHz);
            // MAIN_PLL = 396MHz
            CLOCK_InitSysPfd(kCLOCK_Pfd0, 24u);

            // When in High power boot check whether this is RT3xx part, if yes, core clock max boot clock limit to
            // 132Mhz

            {
                debug_printf("%s,This RT5XX part", __func__);
                // 396MHz / 2 = 198MHz
                CLKCTL0->SYSCPUAHBCLKDIV = (2u - 1u);
                SystemCoreClock = kFreq_198MHz;
                // for glitch free clock swiching. Select the divider clock first then enable divider.
                CLKCTL0->SYSTICKFCLKSEL = 0u;
                CLKCTL0->SYSTICKFCLKDIV = (2u - 1u);
            }

            // Switch to MAIN PLL
            CLKCTL0->MAINCLKSELB = MAINCLKSELB_MAIN_PLL_CLK;
            __ISB();
            __DSB();
            // Configure FLEXSPI Clock to AUX0_PLL, so it is possible to achieve 100MHz DDR read when the core is
            // running at
            // 200MHz
            flexspi_nor_set_clock_source(kFlexSpiClockSrc_Aux0PllClk);
        }
        else
        {
            debug_printf("%s, FFRO enabled", __func__);
            // Enable the FRO192M FRO48M
            CLKCTL0->FRODIVOEN = CLKCTL0_FRODIVOEN_FRO_DIV1_O_EN_MASK | CLKCTL0_FRODIVOEN_FRO_DIV4_O_EN_MASK |
                                  CLKCTL0_FRODIVOEN_FRO_DIV8_O_EN_MASK;

            // Set FRO192M as clock source, core run at 192MHz / 4 = 48Mhz
            CLKCTL0->SYSCPUAHBCLKDIV = (4 - 1u);
            CLKCTL0->SYSTICKFCLKSEL = 0;
            CLKCTL0->SYSTICKFCLKDIV = (4 - 1u);
            CLKCTL0->MAINCLKSELA = MAINCLKSELA_FRO_192M;
            CLKCTL0->MAINCLKSELB = MAINCLKSELB_SYSCLK;

            // Intialize the PLL for uSDHC
            init_syspll(PLL528_CLKSRC_FRO_24M, kClockFreq_24MHz);
            // MAIN_PLL = 396MHz
            CLOCK_InitSysPfd(kCLOCK_Pfd0, 24u);

            SystemCoreClock = kFreq_48MHz;

            // Configure FLEXSPI Clock to AUX0_PLL, so it is possible to achieve 100MHz DDR read when the core is
            // running at
            // 200MHz
            flexspi_nor_set_clock_source(kFlexSpiClockSrc_Aux0PllClk);
        }

        // Select UART0 Clock source
        CLKCTL1->FLEXCOMM[0].FCFCLKSEL = 0x00u; // IRC48M
    }
    else
    {
        if (boot_power == kBootPower_High)
        {
            debug_printf("%s, PLL will use XTAL as input", __func__);

            init_pll_use_xtal();
            debug_printf("%s, FFRO enabled", __func__);
            // Enable the FRO192M FRO48M
            CLKCTL0->FRODIVOEN = CLKCTL0_FRODIVOEN_FRO_DIV1_O_EN_MASK | CLKCTL0_FRODIVOEN_FRO_DIV4_O_EN_MASK;
            
            // When in High power boot check whether this is RT3xx part, if yes, core clock max boot clock limit to
            // 132Mhz

            {
                debug_printf("%s,This RT5XX part", __func__);
                // Set FRO192M as clock source, core run at 192MHz
                CLKCTL0->SYSCPUAHBCLKDIV = (1 - 1u);
                CLKCTL0->SYSTICKFCLKSEL = 0;
                CLKCTL0->SYSTICKFCLKDIV = (1 - 1u);
                SystemCoreClock = kFreq_192MHz;
            }
            
            debug_printf("%s,Swicth to FRO192", __func__);

            CLKCTL0->MAINCLKSELA = MAINCLKSELA_FRO_192M;
            CLKCTL0->MAINCLKSELB = MAINCLKSELB_SYSCLK;

            __DSB();
            __ISB();

            // Configure FLEXSPI Clock to AUX0_PLL, so it is possible to achieve 100MHz DDR read when the core is
            // running at
            // 200MHz
            flexspi_nor_set_clock_source(kFlexSpiClockSrc_Aux0PllClk);
        }
        else
        {
            debug_printf("%s, PLL will use XTAL as input", __func__);

            init_pll_use_xtal();

            debug_printf("%s, FFRO enabled", __func__);

            // Enable the FRO192M FRO48M
            CLKCTL0->FRODIVOEN = CLKCTL0_FRODIVOEN_FRO_DIV1_O_EN_MASK | CLKCTL0_FRODIVOEN_FRO_DIV4_O_EN_MASK;
            
            // Set FRO192M as clock source, core run at 192MHz / 4 = 48Mhz
            CLKCTL0->SYSCPUAHBCLKDIV = (4 - 1u);
            CLKCTL0->SYSTICKFCLKSEL = 0;
            CLKCTL0->SYSTICKFCLKDIV = (4 - 1u);
            CLKCTL0->MAINCLKSELA = MAINCLKSELA_FRO_192M;
            CLKCTL0->MAINCLKSELB = MAINCLKSELB_SYSCLK;
            __ISB();
            __DSB();

            SystemCoreClock = kFreq_48MHz;

            // Configure FLEXSPI Clock to AUX0_PLL, so it is possible to achieve 100MHz DDR read when the core is
            // running at
            // 200MHz
            flexspi_nor_set_clock_source(kFlexSpiClockSrc_Aux0PllClk);
        }

        // Select UART0 Clock source
        CLKCTL1->FLEXCOMM[0].FCFCLKSEL = 0x00u; // IRC48M
    }
    
#else // FPGA build

    SystemCoreClock = kFreq_48MHz;

    // UART0
    CLKCTL1->FLEXCOMM[0].FCFCLKSEL = 0x04u; // Clock source : FRG clock
    CLKCTL1->FLEXCOMM[0].FRGCLKSEL = 0u;
#endif
    
    
}

// See bootloader_common for documentation on this function.
void configure_clocks(bootloader_clock_option_t option)
{
    debug_printf("%s\n", __func__);
    boot_power_t boot_power = get_boot_power();

    // Configure Clock in ROM code
    if (kClockOption_EnterBootloader == option)
    {
        boot_set_clocks(boot_power);
    }
    else // Restore clock settings before leaving ROM code
    {
        // Restore UART clock settings
        CLKCTL1->FLEXCOMM[0].FCFCLKSEL = 0x07u;

    }
}
// See bootloader_common.h for documentation on this function.
// Note: this function doesn't apply to FPGA build
uint32_t get_system_core_clock(void)
{
    return CLOCK_GetFreq(kCLOCK_CoreSysClk);
}

// See bootloader_common.h for documentation on this function.
uint32_t get_bus_clock(void)
{
    return get_system_core_clock();
}

status_t flexspi_nor_set_clock_source(uint32_t clockSource)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (clockSource > kFlexSpiClockSrc_Aux1PllClk)
        {
            break;
        }

        uint32_t currentClkSrc =
            (CLKCTL0->FLEXSPI0FCLKSEL & CLKCTL0_FLEXSPI0FCLKSEL_SEL_MASK) >> CLKCTL0_FLEXSPI0FCLKSEL_SEL_SHIFT;

        if (currentClkSrc != clockSource)
        {
            // Halt the clock before switching clock source to avoid potential glitch
            CLKCTL0->FLEXSPI0FCLKDIV |= (CLKCTL0_FLEXSPI0FCLKDIV_HALT_MASK);
            // Need some delay before changing OSPIFFCLKSEL
            __DSB();
            __ISB();
            // Switch clock source
            CLKCTL0->FLEXSPI0FCLKSEL = CLKCTL0_FLEXSPI0FCLKSEL_SEL(clockSource);
            // Need some delay before next step
            __DSB();
            __ISB();

            // Release the halt signal
            CLKCTL0->FLEXSPI0FCLKDIV &= ~CLKCTL0_FLEXSPI0FCLKDIV_HALT_MASK;
        }

        status = kStatus_Success;

    } while (0);

    return status;
}

#if BL_FEATURE_FLEXSPI_NOR_MODULE
/*
void flexspi_clock_config(uint32_t instance, uint32_t freqOption, uint32_t sampleClkMode)
{
#ifndef BL_TARGET_FPGA
    do
    {
        if ((freqOption < 1) || (freqOption > kFlexSpiSerialClk_200MHz) || (sampleClkMode > kFlexSpiClk_DDR))
        {
            break;
        }

        uint32_t clockSrc = (CLKCTL0->FLEXSPI0FCLKSEL & CLKCTL0_FLEXSPI0FCLKSEL_SEL_MASK) >> CLKCTL0_FLEXSPI0FCLKSEL_SEL_SHIFT;
        uint32_t clockDiv = 1;
        uint32_t currentDiv = (CLKCTL0->FLEXSPI0FCLKDIV & CLKCTL0_FLEXSPI0FCLKDIV_DIV_MASK) >> CLKCTL0_FLEXSPI0FCLKDIV_DIV_SHIFT;
        uint32_t needHaltClk = 0;

        if (clockSrc == kFlexSpiClockSrc_FRO192M_Clk)
        {
            if (sampleClkMode == kFlexSpiClk_SDR)
            {
                switch (freqOption)
                {
                    default:
                    case kFlexSpiSerialClk_32MHz:
                        clockDiv = 6; // 32Mhz
                        break;
                    case kFlexSpiSerialClk_48MHz:
                        clockDiv = 4; // 48Mhz
                        break;
                    case kFlexSpiSerialClk_64MHz:
                        clockDiv = 3; // 64Mhz
                        break;
                    case kFlexSpiSerialClk_96MHz:
                        clockDiv = 2; // 96Mhz
                        break;
                    case kFlexSpiSerialClk_192MHz:
                        clockDiv = 1; // 192Mhz
                        break;
                }
            }
            else // DDR mode
            {
                switch (freqOption)
                {
                    default:
                    case kFlexSpiSerialClk_32MHz:
                        clockDiv = 3; // 192 / 3 / 2 = 32Mhz
                        break;
                    case kFlexSpiSerialClk_48MHz:
                        clockDiv = 2; // 192 / 2 / 2 = 48Mhz
                        break;
                    case kFlexSpiSerialClk_96MHz:
                        clockDiv = 1; // 192 / 1 / 2 = 96Mhz
                        break;
                }
            }
        }

        if ((clockSrc == kFlexSpiClockSrc_MainClk) || (clockSrc == kFlexSpiClockSrc_MainPllClk))
        {
            uint32_t flexspiSrcClk = CLOCK_GetMainClkFreq();
            if (clockSrc == kFlexSpiClockSrc_MainPllClk)
            {
                flexspiSrcClk = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0);
            }

            uint32_t freqList[] = { 30, 30, 50, 60, 80, 100, 120, 133, 166, 200 };

            clockDiv = 1;
            uint32_t freqMHz = flexspiSrcClk / kFreq_1MHz;
            if (sampleClkMode == kFlexSpiClk_DDR)
            {
                freqMHz /= 2;
            }
            bool hasFoundDiv = false;

            while (!hasFoundDiv)
            {
                uint32_t divFreq = freqMHz / clockDiv;
                if (divFreq <= freqList[freqOption])
                {
                    hasFoundDiv = true;
                    break;
                }
                else
                {
                    ++clockDiv;
                }
            }
        }
        else if ((clockSrc == kFlexSpiClockSrc_Aux0PllClk) || (clockSrc == kFlexSpiClockSrc_Aux1PllClk))
        {
            uint32_t expectedPfd = 24; // Aux0_pll will be configured to 396MHz
            if (sampleClkMode == kFlexSpiClk_SDR)
            {
                switch (freqOption)
                {
                    default:
                    case kFlexSpiSerialClk_30MHz:
                        clockDiv = 13; // 528 * 18 / 24 / 13 = 30.46
                        break;
                    case kFlexSpiSerialClk_50MHz:
                        clockDiv = 8; // 528 * 18 / 24 / 8 = 49
                        break;
                    case kFlexSpiSerialClk_60MHz:
                        expectedPfd = 32;
                        clockDiv = 5; // 528 * 18 / 32 / 5 = 59.4
                        break;
                    case kFlexSpiSerialClk_80MHz:
                        expectedPfd = 20;
                        clockDiv = 6; // 528 * 18 / 20 / 6 = 79.2
                        break;
                    case kFlexSpiSerialClk_100MHz:
                        clockDiv = 4; // 528 * 18 / 24 / 6 = 99
                        break;
                    case kFlexSpiSerialClk_120MHz:
                        expectedPfd = 27; // 528 * 18 / 27 / 6 = 117
                        clockDiv = 3;
                        break;
                    case kFlexSpiSerialClk_133MHz:
                        clockDiv = 3; // 528 * 18 / 24 / 3 = 132
                        break;
                    case kFlexSpiSerialClk_166MHz:
                        expectedPfd = 29; // 528 * 18 / 29 / 2 = 163
                        clockDiv = 2;
                        break;
                    case kFlexSpiSerialClk_200MHz:
                        clockDiv = 2; // 528 * 18 / 24 / 2 = 198
                        break;
                }
            }
            else
            {
                expectedPfd = 24; // Aux0_pll will be configured to 396MHz
                switch (freqOption)
                {
                    default:
                    case kFlexSpiSerialClk_30MHz:
                        expectedPfd = 32;
                        clockDiv = 5; // 528 * 18 / 32 / 5 / 2 = 29.7
                        break;
                    case kFlexSpiSerialClk_50MHz:
                        clockDiv = 4; // 528 * 18 / 24 / 4 /2 = 49.5
                        break;
                    case kFlexSpiSerialClk_60MHz:
                        expectedPfd = 26;
                        clockDiv = 3; // 528 * 18 / 26 / 3 /2 = 60.8
                        break;
                    case kFlexSpiSerialClk_80MHz:
                        expectedPfd = 29;
                        clockDiv = 2; // 528 * 18 / 29 / 2 / 2 = 81.75
                        break;
                    case kFlexSpiSerialClk_100MHz:
                        clockDiv = 2; // 528 * 18 / 24 / 2 / 2  = 99
                        break;
                    case kFlexSpiSerialClk_120MHz:
                        expectedPfd = 20; // 528 * 18 / 20 / 2 / 2  = 118.75
                        clockDiv = 2;
                        break;
                    case kFlexSpiSerialClk_133MHz:
                        expectedPfd = 12; // 528 * 18 / 12 / 3 / 2  = 132
                        clockDiv = 3;
                        break;
                    case kFlexSpiSerialClk_166MHz:
                        expectedPfd = 29; // 528 * 18 / 29 / 1 / 2 = 163.8
                        clockDiv = 1;
                        break;
                    case kFlexSpiSerialClk_200MHz:
                        clockDiv = 1; // 528 * 18 / 24 / 1 / 2 = 198
                        break;
                }
            }
            uint32_t currentPfd = 0;

            if (clockSrc == kFlexSpiClockSrc_Aux0PllClk)
            {
                currentPfd = (CLKCTL0->SYSPLL0PFD & CLKCTL0_SYSPLL0PFD_PFD2_MASK) >> CLKCTL0_SYSPLL0PFD_PFD2_SHIFT;
            }
            else
            {
                currentPfd = (CLKCTL0->SYSPLL0PFD & CLKCTL0_SYSPLL0PFD_PFD3_MASK) >> CLKCTL0_SYSPLL0PFD_PFD3_SHIFT;
            }
            uint32_t currentClkSrc =
                (CLKCTL0->FLEXSPI0FCLKSEL & CLKCTL0_FLEXSPI0FCLKSEL_SEL_MASK) >> CLKCTL0_FLEXSPI0FCLKSEL_SEL_SHIFT;

            needHaltClk = (currentDiv != clockDiv) || (currentPfd != expectedPfd) || (clockSrc != currentClkSrc);

            if (needHaltClk)
            {
                // Halt the counter if PDF reconfiguration is required to avoid possible glitch
                CLKCTL0->FLEXSPI0FCLKDIV |= (CLKCTL0_FLEXSPI0FCLKDIV_HALT_MASK);
            }

            // Need some delay before changing OSPIFFCLKSEL
            __DSB();
            __ISB();
            // Switch clock source
            CLKCTL0->FLEXSPI0FCLKSEL = CLKCTL0_FLEXSPI0FCLKSEL_SEL(clockSrc);

            if (currentPfd != expectedPfd)
            {
                // Reconfigure PFD
                if (clockSrc == kFlexSpiClockSrc_Aux0PllClk)
                {
                    CLOCK_InitSysPfd(kCLOCK_Pfd2, expectedPfd);
                }
                else
                {
                    CLOCK_InitSysPfd(kCLOCK_Pfd3, expectedPfd);
                }
                __DSB();
                __ISB();
            }
        }

        // Update clock divider
        CLKCTL0->FLEXSPI0FCLKDIV = CLKCTL0_FLEXSPI0FCLKDIV_DIV(clockDiv - 1);
    } while (0);
#else
    // FPGA build
    CLKCTL0->OSPIFCLKDIV = 0u;                           // Clock divider = 1
    CLKCTL0->OSPIFFCLKSEL = CLKCTL0_OSPIFFCLKSEL_SEL(0); // Main clock
#endif
}

void flexspi_clock_disable(void)
{
    CLKCTL0->PSCCTL0_CLR = CLKCTL0_PSCCTL0_CLR_FLEXSPI0_OTFAD_CLK_MASK;
}

void flexspi_clock_enable(void)
{
    CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI0_OTFAD_CLK_MASK;
}
*/
#endif


bool usb_hs0_device_clock_enable(void)
{
    // USB boot will assume 24MHz for phase 1 ROM. In phase 2 auto-detection
    //  of XTAL frequency will be implemented using "Frequency measure" module.
    debug_printf("Bootloader: ----%s\n", __func__);
#ifndef BL_TARGET_FPGA
    //    /* disable usbhs device clock */
    //    CLOCK_DisableClock(kCLOCK_UsbhsDevice);

    /* Init system osc */
    init_sysosc();

    CLKCTL0->USBHSFCLKSEL = 0;
    CLKCTL0->USBHSFCLKDIV = 0;

    /* Enbale clock for usb1 phy */
    CLKCTL0->PSCCTL0_SET = (1u << 20);
    RSTCTL0->PRSTCTL0_CLR = (1u << 20);
    if (SystemCoreClock < (66 * 1000 * 1000))
    {
        CLKCTL0->PFCDIV[1] = 0;
    }
    else
    {
        CLKCTL0->PFCDIV[1] = 0x03;
    }
    __DSB();
    __ISB();

    // set up the PHY
    USBPHY->CTRL_CLR = USBPHY_CTRL_SFTRST_MASK;  // release PHY from reset
    USBPHY->CTRL_CLR = USBPHY_CTRL_CLKGATE_MASK; // Clear to 0 to run clocks

    /* Enable PLL regulator and wait a while for its stable */
    USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_REG_ENABLE_MASK;
    /* Design suggest 15us, we delay 50us for sure */
    microseconds_delay(50);

    // power up PLL, regulator enable
    uint32_t pll_div = OTP_USB_PLL_DIV_SEL_VALUE();
    if (pll_div == 0)
    {
        pll_div = 3; // Assume 24MHz clock by default
    }
    

    /* Select MULT for USB PLL */
    USBPHY->PLL_SIC = (USBPHY->PLL_SIC & ~(USBPHY_PLL_SIC_PLL_DIV_SEL_MASK)) | USBPHY_PLL_SIC_PLL_DIV_SEL(pll_div);
    microseconds_delay(65);
    /* Power on USB PLL */
    uint32_t retryCnt = 0;
    USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_SET_PLL_POWER(1);
	
    while (!(USBPHY->PLL_SIC & USBPHY_PLL_SIC_SET_PLL_POWER_MASK))
    {
        USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_SET_PLL_POWER(1);
        ++retryCnt;
    }

    debug_printf("Need %d time to set USBPHY_PLL_SIC_POWR bit", retryCnt);

    /* Polling PLLs lock bit */
    uint32_t usbPllLockTimeUs =
        SYSCTL0->USBPHYPLL0LOCKTIMEDIV2 ? SYSCTL0->USBPHYPLL0LOCKTIMEDIV2 : kDefaultUsbPllLockTimeUs;
    uint32_t timeoutTicks = SystemCoreClock / kFreq_1MHz * usbPllLockTimeUs / 4;
    do
    {
        --timeoutTicks;
        if (timeoutTicks < 1)
        {
            break;
        }
    }while (!(USBPHY->PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK_MASK));
    
    if (!(USBPHY->PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK_MASK))
    {
         return false;
    }
    

    /* Wait more time in case HW LOCK time is not enough */
    microseconds_delay(50);

    /* Clear USBPLL bypass bit */
    USBPHY->PLL_SIC_CLR = USBPHY_PLL_SIC_PLL_BYPASS_MASK;
    retryCnt = 0;
    while (USBPHY->PLL_SIC & USBPHY_PLL_SIC_PLL_BYPASS_MASK)
    {
        USBPHY->PLL_SIC_CLR = USBPHY_PLL_SIC_PLL_BYPASS_MASK;
        ++retryCnt;
    }
    debug_printf("Need %d time to clear USBPHY_PLL_SIC_BYPASS bit", retryCnt);

    /* Enable clock output from USB PLL and USB PHY */
    USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_ENABLE(1) | USBPHY_PLL_SIC_SET_PLL_EN_USB_CLKS(1);
    retryCnt = 0;
    
    while ((USBPHY->PLL_SIC & (USBPHY_PLL_SIC_PLL_ENABLE(1) | USBPHY_PLL_SIC_SET_PLL_EN_USB_CLKS(1))) !=
           (USBPHY_PLL_SIC_PLL_ENABLE(1) | USBPHY_PLL_SIC_SET_PLL_EN_USB_CLKS(1)))
    {
        USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_ENABLE(1) | USBPHY_PLL_SIC_SET_PLL_EN_USB_CLKS(1);
        ++retryCnt;
    }
    debug_printf("Need %d time to set USBPHY_PLL_SIC_SET_PLL_ENABLE and USBPHY_PLL_SIC_SET_PLL_EN_USB_CLKS bit", retryCnt);

    /*Clear power down register */
    USBPHY->PWD = 0;

    debug_printf("Enter 5 --- %s, ", __func__);

    // Power on USB
    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_CLR_HSPAD_FSPI0_VDET_LP_MASK;
    /* Power up all necessary modules */
    SYSCTL0->PDRUNCFG1_CLR = SYSCTL0_PDRUNCFG1_USBHS_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_USBHS_SRAM_PPD_MASK;
    /* Enable usbhs device and ram clock */
    CLOCK_EnableClock(kCLOCK_UsbhsDevice);
    CLOCK_EnableClock(kCLOCK_UsbhsSram);

#endif

    return true;
}

void init_pll_use_xtal()
{
    // init sysosc
    init_sysosc();
    // Increase the core voltage to 1.138v before setting the PLL
    PMC->RUNCTRL = PMC_RUNCTRL_CORELVL(0x32);
    pmc_apply_cfg();
    // init PLL using osc clk

    if (OTP_BOOT_CLOCK_XTAL_FREQ_VALUE())
    {
        // Enable PLL use XTAL 16Mhz
        init_syspll(PLL528_CLKSRC_OSC, kClockFreq_16MHz);
        debug_printf("%s, PLL enabled using XTAL 16Mhz", __func__);
    }
    else
    {
        // Enable PLL use XTAL 24Mhz
        init_syspll(PLL528_CLKSRC_OSC, kClockFreq_24MHz);
        debug_printf("%s, PLL enabled using XTAL 24Mhz", __func__);
    }

    // MAIN_PLL = 396MHz
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 24u);
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
