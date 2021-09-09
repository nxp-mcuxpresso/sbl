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
#include "flexspi_nor_flash.h"
#include "fusemap.h"
#include "microseconds/microseconds.h"
#if BL_FEATURE_OTP_MODULE
#include "otp/fsl_otp.h"
#endif
#include "property/property.h"
#include "target_config.h"
#include "utilities/fsl_assert.h"
#if BL_FEATURE_AUTHENTICATION
#include "authentication/secure_kboot.h"
#endif
#include "bootloader/bootloader.h"
#include "bootloader_hid_report_ids.h"
#include "composite.h"
#if ((defined FSL_FEATURE_SOC_USBPHY_COUNT) && (FSL_FEATURE_SOC_USBPHY_COUNT > 0U))
#include "usb_phy.h"
#endif
#include "fsl_power.h"
#include "board.h"

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
    kFreq_4MHz = 4 * kFreq_1MHz,
    kFreq_24MHz = 24u * kFreq_1MHz,
    kFreq_48MHz = 48u * kFreq_1MHz,
    kFreq_198MHz = 198u * kFreq_1MHz,
    kFreq_264MHz = 264u * kFreq_1MHz,
    kFreq_16MHz = 16u * kFreq_1MHz,
    kFreq_96MHz = 96u * kFreq_1MHz,
    kFreq_240MHz = 240u * kFreq_1MHz,
    kDefaultUsbPllLockTimeUs = 200000u,
};

enum
{
    MAINCLKSELA_12MHz_IRC = 0,
    MAINCLKSELA_OSC_CLK,
    MAINCLKSELA_1M_LPOSC,
    MAINCLKSELA_48M_60M_IRC,
};

enum
{
    PLL528_CLKSRC_16M_IRC = 0,
    PLL528_CLKSRC_OSC,
    PLL528_CLKSRC_48MHz_60MHz_Div2
};

enum
{
    MAINCLKSELB_SYSCLK,
    MAINCLKSELB_16M_IRC,
    MAINCLKSELB_MAIN_PLL_CLK,
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
};

enum
{
    kOspiClockSrc_MainClk = 0,
    kOspiClockSrc_MainPllClk,
    kOspiClockSrc_Aux0PllClk,
    kOspiClockSrc_48M_60M_Clk,
    kOspiClockSrc_Aux1PllClk,
};

#if defined BL_TARGET_FPGA
uint32_t busClock = 6000000u; //! 6MHz default bus clock
#endif

//!@brief FlexSPI clock configuration type
enum
{
    kFlexSpiClk_SDR, //!< Clock configure for SDR mode
    kFlexSpiClk_DDR, //!< Clock configurat for DDR mode
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
static boot_power_t get_boot_power(void);
static void boot_set_clocks(boot_power_t boot_power);
static void init_syspll(uint32_t clk_src, uint32_t src_clk_freq);
static void init_sysosc(void);
static void delay_us_sw(uint32_t us);

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

void init_syspll(uint32_t clk_src, uint32_t src_clk_freq)
{
    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_SYSPLLLDO_PD_MASK | SYSCTL0_PDRUNCFG0_SYSPLLANA_PD_MASK;

    CLKCTL0->SYSPLL0CLKSEL = clk_src;

    if (kFreq_24MHz == src_clk_freq)
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
}

void init_sysosc(void)
{
    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_SYSXTAL_PD_MASK;
    __DSB();
    __ISB();
    CLKCTL0->SYSOSCCTL0 = CLKCTL0_SYSOSCCTL0_LP_ENABLE(1) | CLKCTL0_SYSOSCCTL0_BYPASS_ENABLE(0);

    CLKCTL0->SYSOSCBYPASS = CLKCTL0_SYSOSCBYPASS_SEL(SYSOSCSEL_XTAL_CLK);

    // Wait until OSC gets stable
#if BL_FEATURE_OTP_MODULE
    uint32_t sysOscStableUsFuseIdx = 14;
    uint32_t sysOscStableUsFuseValue = 0;
    status_t status = otp_fuse_read(sysOscStableUsFuseIdx, &sysOscStableUsFuseValue);
    if (status != kStatus_Success)
    {
        go_fatal_mode();
    }
#else
    uint32_t sysOscStableUsFuseValue = 32;
#endif
    microseconds_delay(sysOscStableUsFuseValue);
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
    if (boot_power == kBootPower_High)
    {
        // Configure Clock to certain state before swithching to PLL
        CLKCTL0->MAINCLKSELB = MAINCLKSELB_SYSCLK;
        CLKCTL0->MAINCLKSELA = MAINCLKSELA_48M_60M_IRC;

        // PLL_OUTPUT = 24MHz * (22 + 0 / 1) = 528MHz
        SystemCoreClock = kFreq_48MHz;
        init_syspll(PLL528_CLKSRC_48MHz_60MHz_Div2, kFreq_24MHz);
        // MAIN_PLL = 396MHz
        CLOCK_InitSysPfd(kCLOCK_Pfd0, 24u);

        // 396MHz / 2 = 198MHz
        CLKCTL0->SYSCPUAHBCLKDIV = (2u - 1u);
        // for glitch free clock swiching. Select the divider clock first then enable divider.
        CLKCTL0->SYSTICKFCLKSEL = 0u;
        CLKCTL0->SYSTICKFCLKDIV = (2u - 1u);

        // Update OTP clock divider before switching to higer clock
        SystemCoreClock = kFreq_198MHz;
#if BL_FEATURE_OTP_MODULE
        otp_init(SystemCoreClock);
#endif

        // Switch to MAIN PLL
        CLKCTL0->MAINCLKSELB = MAINCLKSELB_MAIN_PLL_CLK;

        // Configure QuadSPI Clock to AUX_PLL, so it is possible to achieve 100MHz DDR read when the core is running at
        // 200MHz
        flexspi_nor_set_clock_source(kOspiClockSrc_Aux0PllClk);
    }
    else
    {
        // Set IRC48M as clock source, core run at 48MHz
        CLKCTL0->SYSCPUAHBCLKDIV = 0u;
        CLKCTL0->SYSTICKFCLKSEL = 0u;
        CLKCTL0->SYSTICKFCLKDIV = 0u;
        CLKCTL0->MAINCLKSELA = MAINCLKSELA_48M_60M_IRC;
        CLKCTL0->MAINCLKSELB = MAINCLKSELB_SYSCLK;

        // Intialize the PLL for uSDHC
        init_syspll(PLL528_CLKSRC_48MHz_60MHz_Div2, kFreq_24MHz);
        // MAIN_PLL = 396MHz
        CLOCK_InitSysPfd(kCLOCK_Pfd0, 24u);

        SystemCoreClock = kFreq_48MHz;

        // Configure QuadSPI Clock to FIRC48MHz, so it can use the same clock as the core to avoid potential TX buffer
        // underflow or RX buffer overflow issue due to setting the OSPI clock to a higher clock
        flexspi_nor_set_clock_source(kOspiClockSrc_48M_60M_Clk);
    }

    // Select UART0 Clock source
    CLKCTL1->FLEXCOMM[0].FCFCLKSEL = 0x01u; // IRC48M/60M
#else                                       // FPGA build

    SystemCoreClock = kFreq_16MHz;

    // UART0
    CLKCTL1->FLEXCOMM[0].FCFCLKSEL = 0x04u; // Clock source : FRG clock
    CLKCTL1->FLEXCOMM[0].FRGCLKSEL = 0u;

#endif
}

// See bootloader_common for documentation on this function.
void configure_clocks(bootloader_clock_option_t option)
{
    boot_power_t boot_power = get_boot_power();

    // Configure Clock in ROM code
    if (kClockOption_EnterBootloader == option)
    {
//        uint32_t logEntry = MAKE_LOG_ENTRY(kLog_HardwareInit, kSubState_ClockConfig, kLog_Status_Pass, 0);
//        LOG_ADD_ENTRY(logEntry);
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

        uint32_t clockSrc = (CLKCTL0->FLEXSPIFCLKSEL & CLKCTL0_FLEXSPIFCLKSEL_SEL_MASK) >> CLKCTL0_FLEXSPIFCLKSEL_SEL_SHIFT;
        uint32_t clockDiv = 1;
        uint32_t currentDiv = (CLKCTL0->FLEXSPIFCLKDIV & CLKCTL0_FLEXSPIFCLKDIV_DIV_MASK) >> CLKCTL0_FLEXSPIFCLKDIV_DIV_SHIFT;

        uint32_t needHaltClk = 0;
        if (clockSrc == kOspiClockSrc_48M_60M_Clk)
        {
            if (sampleClkMode == kFlexSpiClk_SDR)
            {
                if (freqOption < kFlexSpiSerialClk_50MHz)
                {
                    clockDiv = 2; // 24MHz
                }
                else
                {
                    clockDiv = 1; // 48MHz
                }
            }
            else // DDR mode
            {
                clockDiv = 1; // 48MHz / 4 = 12MHz
            }
        }
        if ((clockSrc == kOspiClockSrc_MainClk) || (clockSrc == kOspiClockSrc_MainPllClk))
        {
            uint32_t qspiSrcClk = CLOCK_GetMainClkFreq();
            if (clockSrc == kOspiClockSrc_MainPllClk)
            {
                qspiSrcClk = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0);
            }

            uint32_t freqList[] = { 30, 30, 50, 60, 80, 100, 120, 133, 166, 200 };

            clockDiv = 1;
            uint32_t freqMHz = qspiSrcClk / kFreq_1MHz;
            if (sampleClkMode == kFlexSpiClk_DDR)
            {
                freqMHz /= 4;
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
        else if ((clockSrc == kOspiClockSrc_Aux0PllClk) || (clockSrc == kOspiClockSrc_Aux1PllClk))
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
                        clockDiv = 5; // 528 * 18 / 20 / 6 = 79.2
                        break;
                    case kFlexSpiSerialClk_100MHz:
                        clockDiv = 4; // 528 * 18 / 24 / 6 = 99
                        break;
                    case kFlexSpiSerialClk_120MHz:
                        expectedPfd = 27;
                        clockDiv = 3; // 528 * 18 / 27 / 6 = 117
                        break;
                    case kFlexSpiSerialClk_133MHz:
                        clockDiv = 3; // 528 * 18 / 24 / 3 = 132
                        break;
                    case kFlexSpiSerialClk_166MHz:
                        expectedPfd = 29;
                        clockDiv = 2; // 528 * 18 / 29 / 2 = 163
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
            if (clockSrc == kOspiClockSrc_Aux0PllClk)
            {
                currentPfd = (CLKCTL0->SYSPLL0PFD & CLKCTL0_SYSPLL0PFD_PFD2_MASK) >> CLKCTL0_SYSPLL0PFD_PFD2_SHIFT;
            }
            else
            {
                currentPfd = (CLKCTL0->SYSPLL0PFD & CLKCTL0_SYSPLL0PFD_PFD3_MASK) >> CLKCTL0_SYSPLL0PFD_PFD3_SHIFT;
            }
            uint32_t currentClkSrc =
                (CLKCTL0->FLEXSPIFCLKSEL & CLKCTL0_FLEXSPIFCLKSEL_SEL_MASK) >> CLKCTL0_FLEXSPIFCLKSEL_SEL_SHIFT;

            needHaltClk = (currentDiv != clockDiv) || (currentPfd != expectedPfd) || (clockSrc != currentClkSrc);

            if (needHaltClk)
            {
                // Halt the counter if PDF reconfiguration is required to avoid possible glitch
                CLKCTL0->FLEXSPIFCLKDIV |= (CLKCTL0_FLEXSPIFCLKDIV_HALT_MASK);
            }

            // Need some delay before changing FLEXSPIFCLKSEL
            __DSB();
            __ISB();
            // Switch clock source
            CLKCTL0->FLEXSPIFCLKSEL = CLKCTL0_FLEXSPIFCLKSEL_SEL(clockSrc);

            if (currentPfd != expectedPfd)
            {
                // Reconfigure PFD
                if (clockSrc == kOspiClockSrc_Aux0PllClk)
                {
                    CLOCK_InitSysPfd(kCLOCK_Pfd2, expectedPfd);
                }
                else
                {
                    CLOCK_InitSysPfd(kCLOCK_Pfd3, expectedPfd);
                }
                // need some delay before next step
                __DSB();
                __ISB();
            }
        }

        // Update clock divider
        CLKCTL0->FLEXSPIFCLKDIV = CLKCTL0_FLEXSPIFCLKDIV_DIV(clockDiv - 1);

    } while (0);
#else
    // FPGA build
    CLKCTL0->FLEXSPIFCLKDIV = 0u;                           // Clock divider = 1
    CLKCTL0->FLEXSPIFCLKSEL = CLKCTL0_FLEXSPIFCLKSEL_SEL(0); // Main clock
#endif
}

void quadspi_clock_disable(void)
{
    CLKCTL0->PSCCTL0_CLR = CLKCTL0_PSCCTL0_FLEXSPI_OTFAD_CLK_MASK;
}

void quadspi_clock_enable(void)
{
    CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_FLEXSPI_OTFAD_CLK_MASK;
}
*/
    
void USB_DeviceClockInit(void)
{
    uint8_t usbClockDiv = 1;
    uint32_t usbClockFreq;
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    /* enable USB IP clock */
    CLOCK_SetClkDiv(kCLOCK_DivPfc1Clk, 5);
    CLOCK_AttachClk(kXTALIN_CLK_to_USB_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivUsbHsFclk, usbClockDiv);
    CLOCK_EnableUsbhsDeviceClock();
    RESET_PeripheralReset(kUSBHS_PHY_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSBHS_DEVICE_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSBHS_HOST_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSBHS_SRAM_RST_SHIFT_RSTn);
    /*Make sure USDHC ram buffer has power up*/
    POWER_DisablePD(kPDRUNCFG_APD_USBHS_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_USBHS_SRAM);
    POWER_ApplyPD();
    
    /* save usb ip clock freq*/
    usbClockFreq = g_xtalFreq / usbClockDiv;
    /* enable USB PHY PLL clock, the phy bus clock (480MHz) source is same with USB IP */
    CLOCK_EnableUsbHs0PhyPllClock(kXTALIN_CLK_to_USB_CLK, usbClockFreq);

#if defined(FSL_FEATURE_USBHSD_USB_RAM) && (FSL_FEATURE_USBHSD_USB_RAM)
    for (int i = 0; i < FSL_FEATURE_USBHSD_USB_RAM; i++)
    {
        ((uint8_t *)FSL_FEATURE_USBHSD_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
    }
#endif
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL_SYS_CLK_HZ, &phyConfig);

    /* the following code should run after phy initialization and should wait some microseconds to make sure utmi clock
     * valid */
    /* enable usb1 host clock */
    CLOCK_EnableClock(kCLOCK_UsbhsHost);
    /*  Wait until host_needclk de-asserts */
    while (SYSCTL0->USBCLKSTAT & SYSCTL0_USBCLKSTAT_HOST_NEED_CLKST_MASK)
    {
        __ASM("nop");
    }
    /*According to reference mannual, device mode setting has to be set by access usb host register */
    USBHSH->PORTMODE |= USBHSH_PORTMODE_DEV_ENABLE_MASK;
    /* disable usb1 host clock */
    CLOCK_DisableClock(kCLOCK_UsbhsHost);
}

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

    /* Power up usb1 phy */
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
    USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_POWER(1);
    while (!(USBPHY->PLL_SIC & USBPHY_PLL_SIC_PLL_POWER_MASK))
    {
        USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_POWER(1);
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
    } while (!(USBPHY->PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK_MASK));

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
    USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_ENABLE(1) | USBPHY_PLL_SIC_PLL_EN_USB_CLKS(1);
    retryCnt = 0;
    while ((USBPHY->PLL_SIC & (USBPHY_PLL_SIC_PLL_ENABLE(1) | USBPHY_PLL_SIC_PLL_EN_USB_CLKS(1))) !=
           (USBPHY_PLL_SIC_PLL_ENABLE(1) | USBPHY_PLL_SIC_PLL_EN_USB_CLKS(1)))
    {
        USBPHY->PLL_SIC_SET = USBPHY_PLL_SIC_PLL_ENABLE(1) | USBPHY_PLL_SIC_PLL_EN_USB_CLKS(1);
        ++retryCnt;
    }
    debug_printf("Need %d time to set USBPHY_PLL_SIC_PLL_ENABLE and USBPHY_PLL_SIC_PLL_EN_USB_CLKS bit", retryCnt);

    /*Clear power down register */
    USBPHY->PWD = 0;

    debug_printf("Enter 5 --- %s, ", __func__);

    // Power on USB
    uint32_t usbPdMask = 0x4000000;
    SYSCTL0->PDRUNCFG0_CLR = usbPdMask;
    /* Power up all necessary modules */
    SYSCTL0->PDRUNCFG1_CLR = SYSCTL0_PDRUNCFG1_USBHS_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_USBHS_SRAM_PPD_MASK;
    /* Enable usbhs device and ram clock */
    CLOCK_EnableClock(kCLOCK_UsbhsDevice);
    CLOCK_EnableClock(kCLOCK_UsbhsSram);

#endif

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
