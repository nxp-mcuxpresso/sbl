/*
 * Copyright 2018 - 2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#if BL_FEATURE_FLEXSPI_NOR_MODULE
#include "flexspi_nor_flash.h"
#endif
#include "fsl_device_registers.h"
#include "microseconds/microseconds.h"
#include "property/property.h"
#include "target_config.h"
#include "device_config.h"
#include "fusemap.h"
#if BL_FEATURE_OTFAD_MODULE
#include "ocotp/fsl_ocotp.h"
#endif
#include "utilities/fsl_assert.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#define FREQ_1MHz (1000000UL)
#define FREQ_396MHz (396UL * FREQ_1MHz)
#define FREQ_528MHz (528UL * FREQ_1MHz)
#define FREQ_24MHz (24UL * FREQ_1MHz)
#define FREQ_200MHz (200UL * FREQ_1MHz)
#define FREQ_240MHz (240UL * FREQ_1MHz)
#define FREQ_480MHz (480UL * FREQ_1MHz)
#define FREQ_400MHz (400UL * FREQ_1MHz)
#define FREQ_500MHz (500UL * FREQ_1MHz)
#define FREQ_696MHz (696UL * FREQ_1MHz)
#define FREQ_19P2MHz (19200000u) // 19.2MHz
#define FREQ_360MHz (360UL * FREQ_1MHz)
#define FREQ_594MHz (594UL * FREQ_1MHz)
#define FREQ_664MHz (664UL * FREQ_1MHz)

enum
{
    kMaxFreq_M7 = 700 * FREQ_1MHz,
    kMaxFreq_M4 = 240 * FREQ_1MHz,
    kMaxFreq_Bus = 200 * FREQ_1MHz,
    kMaxFreq_BusL = 120 * FREQ_1MHz,
    kMaxFreq_UART = 80 * FREQ_1MHz,
    kMaxFreq_FLEXSPI = 332 * FREQ_1MHz,
    kMaxFreq_SEMC = 400 * FREQ_1MHz,
    kMaxFreq_USDHC = 400 * FREQ_1MHz,
    kMaxFreq_SPI = 135 * FREQ_1MHz,
    kMaxFreq_FLEXRAM = 175 * FREQ_1MHz,
};

enum
{
    kCm7ClockSrc_RC400M = 0xaa,
    kCm7ClockSrc_PLL_ARM = 0x3c,
};

enum
{
    kCm4ClockSrc_RC400M = 0xaa,
    kCm4ClockSrc_PLL_480 = 0x3c,
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
extern bool is_cm4_boot(void);
void configure_m7_boot_clock(bootloader_clock_option_t option);
void configure_m4_boot_clock(bootloader_clock_option_t option);
void rc400m_trim(void);
void rc16m_trim(void);

uint32_t get_cm7_core_src(void);
uint32_t get_cm4_core_src(void);
uint32_t get_osc_freq(void);
status_t configure_plls(bool enableArmPll);
status_t configure_arm_pll(uint32_t oscFreq);
status_t configure_480_pll(uint32_t oscFreq);
status_t configure_528_pll(uint32_t oscFreq);
void configure_rc400m(void);
void sw_delay_us(uint32_t us);
void enable_pll_ldo_power(void);
extern uint32_t get_flexspinor_instance(void);
static inline uint32_t get_clock_div(uint32_t root, uint32_t maxFreq)
{
    return ((root + maxFreq - 1) / maxFreq);
}

void bypass_ldo_1p8(void);
void bypass_ldo_1p0(void);
void bypass_ldo_1p8_and_1p0(void);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
void rc400m_trim(void)
{
    uint32_t trimRegValue = ANADIG_OSC->OSC_OTP_TRIM_VALUE_200M;

    uint32_t trimEnable = (trimRegValue & (1u << 9)) ? 1 : 0;
    uint32_t trimBypass = (trimRegValue & (1u << 8)) ? 1 : 0;
    uint32_t trimValue = trimRegValue & 0xFF;

    if (trimEnable)
    {
        debug_printf("rc400m_trim_reg_value=%x\n", trimRegValue);
        ANADIG_MISC->VDDLPSR_AI400M_CTRL = 0x20;
        ANADIG_MISC->VDDLPSR_AI400M_WDATA = (trimBypass << 10) | (trimValue << 24);
        ANADIG_MISC->VDDLPSR_AI400M_CTRL |= 0x100;
        sw_delay_us(1);
        ANADIG_MISC->VDDLPSR_AI400M_CTRL &= ~0x100;
    }
}

void rc16m_trim(void)
{
    /*
    1,power up.
    2,RC16M power up without trim.
    3,Enable osc24M with register bit (bit8 of OSC24M_CTRL) write 1'b1
    4,wait OSC_24M_stable(bit 30 of OSC24M_CTRL) wait 1'b1
    5,Switch source(24M) of 16M with register bit (bit8 of OSC4M16M_CTRL) write 1'b1
    6,Enable TRIM_SW_FORCE_16M register bit(bit 2 of ANATOP_CTRL)
    7,Check OSC_OTP_TRIM_VALUE_16M bit 29(osc_16m_trim_en) is 1'b1.If 1'b1, TRIM_VALUE is available in trim bus and able
    to use.
    8,Switch source(Trimmed 16M) of 16M with register bit (bit8 of OSC4M16M_CTRL) write 1'b0
    */

    volatile uint32_t trimOption = 3;

    debug_printf("%s(), started\n", __func__);

    if (FUSE_OSC_4_16M_TRIM_EN_VAL)
    {
        if (trimOption == 0)
        {
            // Step 3
            // Enable OSC, wait until OSC is stable
            ANADIG_OSC->OSC_24M_CTRL = 0x14; // OSC_EN=1, LP_EN=1, GATE=0, BYPASS=0
            while (!(ANADIG_OSC->OSC_24M_CTRL & (1u << 30)))
            {
            }
            ANADIG_OSC->OSC_4M16M_CTRL |= (1u << 8);
            ANADIG_OSC->ANATOP_CTRL |= (1u << 2);
            if (ANADIG_OSC->OSC_OTP_TRIM_VALUE_16M & (1u << 29))
            {
            }
            ANADIG_OSC->OSC_4M16M_CTRL &= ~(uint32_t)(1u << 8);
        }
        else if (trimOption == 1)
        {
            ANADIG_OSC->OSC_4M16M_CTRL |= (1u << 8);
            ANADIG_OSC->ANATOP_CTRL |= (1u << 2);
            if (ANADIG_OSC->OSC_OTP_TRIM_VALUE_16M & (1u << 29))
            {
            }
            ANADIG_OSC->OSC_4M16M_CTRL &= ~(uint32_t)(1u << 8);
        }
        else
        {
            // Do not trim
        }
    }

    debug_printf("%s(), completed\n", __func__);
}

void bypass_ldo_1p8(void)
{
// 1. Bypass LPSPR_LDO-1P8 first
// Enter bypass sequence
/*
Before bypassing the LPSR_LDO_1P8, ensure the bit "pull_down_2ma_en"=1, "track_mode_en"=0, "bypass_mode_en"=0,
"reg_disable" =0, "reg_lp_en"=0. Then do the following steps to enter the bypass.
1. Set the "bypass_mode_cfg[1:0]"=10, "bypass_current_cfg[1:0]"=10 and "track_mode_en"=1 to ensure the LPSR_LDO_1P8’s
output start to follow the DCDC’s.
2. Waiting at least 125us to ensure the LDO’s output can follow the DCDC’s completely.
3. Set the "bypass_mode_en" to high to bypass the LPSR_LDO_1P8.
4. Waiting at least 50us.
5. Set the "reg_disable" =1, then the LPSR_LDO_1P8 enter the bypass mode completely.
*/
#define PULL_DOWN_2MA_EN_MASK (1u << 3)
#define TRACK_MODE_EN_MASK (1u << 19)
#define BYPASS_MODE_EN_MASK (1u << 5)
#define REG_DISABLE_MASK (1u << 2)
#define REG_LP_EN_MASK (1u << 0)
#define BYPASS_MODE_CFG_SHIFT (11)
#define BYPASS_MODE_CFG_MASK (0x3u << BYPASS_MODE_CFG_SHIFT)
#define BYPASS_MODE_CFG(x) (((x) << BYPASS_MODE_CFG_SHIFT) & BYPASS_MODE_CFG_MASK)
#define BYPASS_PRECHRG_CURRENT_CFG_SHIFT (9)
#define BYPASS_PRECHRG_CURRENT_CFG_MASK (0x3u << BYPASS_PRECHRG_CURRENT_CFG_SHIFT)
#define BYPASS_PRECHRG_CURRENT_CFG(x) (((x) << BYPASS_PRECHRG_CURRENT_CFG_SHIFT) & BYPASS_PRECHRG_CURRENT_CFG_MASK)
#define PMU_LDO_LPSR_1P8 (*(uint32_t *)(ANADIG_MISC_BASE + 0x510))

    debug_printf("%s(), started\n", __func__);

    // Preparation
    uint32_t tmpRegVal = PMU_LDO_LPSR_1P8;
    tmpRegVal |= (PULL_DOWN_2MA_EN_MASK);
    tmpRegVal &= (uint32_t) ~(TRACK_MODE_EN_MASK | BYPASS_MODE_EN_MASK | REG_DISABLE_MASK | REG_LP_EN_MASK);
    PMU_LDO_LPSR_1P8 = tmpRegVal;
    __ISB();

    // Step 1
    tmpRegVal = PMU_LDO_LPSR_1P8;
    tmpRegVal = (tmpRegVal & ~(BYPASS_MODE_CFG_MASK | BYPASS_PRECHRG_CURRENT_CFG_MASK)) | BYPASS_MODE_CFG(2) |
                BYPASS_PRECHRG_CURRENT_CFG(2);
    tmpRegVal |= TRACK_MODE_EN_MASK;
    PMU_LDO_LPSR_1P8 = tmpRegVal;
    __ISB();

    // Step 2, delay more time in case the clock is not accurate
    sw_delay_us(200);

    // Step 3
    PMU_LDO_LPSR_1P8 |= BYPASS_MODE_EN_MASK;
    __ISB();
    // Step 4
    sw_delay_us(100);

    // Step 5
    PMU_LDO_LPSR_1P8 |= REG_DISABLE_MASK;
    __ISB();

    debug_printf("%s(), completed\n", __func__);
}

void bypass_ldo_1p0(void)
{
/*
Enter bypass
1) before enter bypass mode, please make sure DCDC is enabled and LPSR_LDO_1P0 is enabled.
reg_en=1, reg_hp_en=0, tracking_mode=0, bypass_mode=0
2) Let LPSR_LDO_1P0 enter high power mode. At least wait 200us before next step operation.
reg_en=1, reg_hp_en=0->1, tracking_mode=0, bypass_mode=0
3) Let LPSR_LDO_1P0 enter track mode. At least wait 300us before next step operation.
reg_en=1, reg_hp_en=1, tracking_mode=0->1, bypass_mode=0
4) Let LPSR_LDO_1P0 enter the mode that DCDC and ldo_1p0 supply power simultaneously. At least wait 25us before next
step operation. reg_en=1, reg_hp_en=1, tracking_mode=1, bypass_mode=0->1
5) Disable LPSR_LDO_1P0, only DCDC provide power supply. The procedure of enter bypass mode is completed.
   reg_en=1->0, reg_hp_en=1, tracking_mode=1, bypass_mode=1

*/
#define PULL_DOWN_2MA_EN_MASK (1u << 3)
#define TRACKING_MODE_MASK (1u << 17)
#define BYPASS_MODE_MASK (1u << 18)
#define REG_EN_MASK (1u << 2)
#define REG_HP_EN_MASK (1u << 0)
#define BYPASS_MODE_CFG_SHIFT (11)
#define BYPASS_MODE_CFG_MASK (0x3u << BYPASS_MODE_CFG_SHIFT)
#define BYPASS_MODE_CFG(x) (((x) << BYPASS_MODE_CFG_SHIFT) & BYPASS_MODE_CFG_MASK)
#define BYPASS_PRECHRG_CURRENT_CFG_SHIFT (9)
#define BYPASS_PRECHRG_CURRENT_CFG_MASK (0x3u << BYPASS_PRECHRG_CURRENT_CFG_SHIFT)
#define BYPASS_PRECHRG_CURRENT_CFG(x) (((x) << BYPASS_PRECHRG_CURRENT_CFG_SHIFT) & BYPASS_PRECHRG_CURRENT_CFG_MASK)
#define PMU_LDO_LPSR_1P0 (*(uint32_t *)(ANADIG_MISC_BASE + 0x530))

    debug_printf("%s(), started\n", __func__);

    // Step 1
    uint32_t tmpRegVal = PMU_LDO_LPSR_1P0;
    tmpRegVal |= REG_EN_MASK;
    tmpRegVal &= (uint32_t) ~(REG_HP_EN_MASK | TRACKING_MODE_MASK | BYPASS_MODE_MASK);
    PMU_LDO_LPSR_1P0 = tmpRegVal;
    __ISB();

    // Step 2
    PMU_LDO_LPSR_1P0 |= REG_HP_EN_MASK;
    __ISB();
    sw_delay_us(300);

    // Step 3
    PMU_LDO_LPSR_1P0 |= TRACKING_MODE_MASK;
    __ISB();
    sw_delay_us(400);

    // Step 4
    PMU_LDO_LPSR_1P0 |= BYPASS_MODE_MASK;
    __ISB();
    sw_delay_us(50);

    // Step 5
    PMU_LDO_LPSR_1P0 &= (uint32_t)~REG_EN_MASK;
    __ISB();

    debug_printf("%s(), completed\n", __func__);
}

void bypass_ldo_1p8_and_1p0(void)
{
    /* Sequence from design team:
    (1)When we plan to do the bypass operation for the LDO_1P0, it's required to do the bypass operation for the
    LDO_1P8 first, after that, we can start to do the bypass for the LDO_1P0.
    (2)When we plan to exit the bypass mode
    of the LDO_1P0, it's required the LDO_1P0 exit first, after that, we start to do the exit for the LDO_1P8.
    */
    bypass_ldo_1p8();
    bypass_ldo_1p0();
}

uint32_t get_osc_freq(void)
{
    uint32_t oscFreq = FREQ_24MHz;
    if (FUSE_BOOT_OSC_REF_VALUE)
    {
        oscFreq = FREQ_19P2MHz;
    }

    return oscFreq;
}

void sw_delay_us(uint32_t us)
{
    if (us > 0)
    {
        SysTick->LOAD = (SystemCoreClock / FREQ_1MHz) * us;
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

        while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != SysTick_CTRL_COUNTFLAG_Msk)
        {
        }
        SysTick->CTRL = 0;
        SysTick->VAL = 0;
    }
}

void configure_rc400m(void)
{
    debug_printf("%s: Start enabling RC400MHz\n", __func__);
    // Enable RC400M
    ANADIG_OSC->OSC_200M_CTRL1 &= ~0x01; // Enable RC400M
    ANADIG_OSC->OSC_200M_CTRL2 |= 0x01;  // Enable CLK

    rc400m_trim();
}

status_t configure_arm_pll(uint32_t oscFreq)
{
    debug_printf("%s\n", __func__);

    uint32_t divSelect = 0;
    // Enable ARM PLL
    if (oscFreq == FREQ_24MHz)
    {
        divSelect = 116; // ARM_PLL = 24MHz * 116 / 2 / 2= 696MHz
    }
    else if (oscFreq == FREQ_19P2MHz)
    {
        divSelect = 145; // ARM_PLL = 19.2MHz * 145 / 2 / 2 = 696MHz
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    // 1. Configure PLL register
    // 2. Power up PLL, assert holdring_off
    // 3. At half lock time, de-assert hondring off
    // 4. Wait PLL lock
    // 5. Enable clock out
    // Note: lock time: ARM_PLL 50us, PLL_480 50us, PLL_528 450us
    uint32_t regValue = ANADIG_PLL_PLL_ARM_CTRL_DIV_SELECT(divSelect) | ANADIG_PLL_PLL_ARM_CTRL_POWERUP(1) |
                        ANADIG_PLL_PLL_ARM_CTRL_PLL_ARM_GATE(1);
    ANADIG_PLL->PLL_ARM_CTRL = regValue;
    __DSB();
    __ISB();
    sw_delay_us(30);
    //    regValue &= ~ANATOP_PLLARM_HOLD_RING_OFF(1);
    //    ANADIG_PLL->PLL_ARM_CTRL = regValue;
    uint32_t waitTicks = 0;
    uint32_t timeoutTicks = (SystemCoreClock / FREQ_1MHz) * 30;
    while ((ANADIG_PLL->PLL_ARM_CTRL & ANADIG_PLL_PLL_ARM_CTRL_PLL_ARM_STABLE_MASK) !=
           ANADIG_PLL_PLL_ARM_CTRL_PLL_ARM_STABLE_MASK)
    {
        waitTicks++;
        if (waitTicks > timeoutTicks)
        {
            debug_printf("ARM_PLL config timeout, ANADIG_PLL->PLL_ARM_CTRL=%x\n", ANADIG_PLL->PLL_ARM_CTRL);
            return kStatus_Timeout;
        }
    }

    // Delay 50us before enabling the clock out
    //    sw_delay_us(50);
    // Enable PLLARM Clock
    regValue |= ANADIG_PLL_PLL_ARM_CTRL_ENABLE_CLK(1);
    ANADIG_PLL->PLL_ARM_CTRL = regValue;

    // Enable Clock output
    regValue &= ~ANADIG_PLL_PLL_ARM_CTRL_PLL_ARM_GATE_MASK;
    ANADIG_PLL->PLL_ARM_CTRL = regValue;

    debug_printf("ARM_PLL config completed, ANADIG_PLL->PLL_ARM_CTRL=%x\n", ANADIG_PLL->PLL_ARM_CTRL);

    return kStatus_Success;
}

status_t configure_480_pll(uint32_t oscFreq)
{
    debug_printf("%s\n", __func__);

    uint32_t divSelect = 0;

    if (oscFreq == FREQ_24MHz)
    {
        divSelect = 3;
    }
    else if (oscFreq == FREQ_19P2MHz)
    {
        divSelect = 5;
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    // 1. Configure PLL register
    // 2. Enable interl LDO
    // 3. Wait LDO stable
    // 4. Power up PLL, assert holdring_off
    // 5. At half lock time, de-assert hondring off
    // 6. Wait PLL lock
    // 7. Enable clock out
    // Note: lock time: ARM_PLL 50us, PLL_480 50us, PLL_528 450us
    // Gate off all the PFDs first
    uint32_t pfdClockGateMask = ANADIG_PLL_PLL_480_PFD_PFD0_FRAC_MASK | ANADIG_PLL_PLL_480_PFD_PFD1_FRAC_MASK |
                                ANADIG_PLL_PLL_480_PFD_PFD2_FRAC_MASK | ANADIG_PLL_PLL_480_PFD_PFD3_FRAC_MASK;
    ANADIG_PLL->PLL_480_PFD |= pfdClockGateMask;
    // REG_EN=1, DIVSEL=3/5, GATE=1
    uint32_t regValue = ANADIG_PLL_PLL_480_CTRL_DIV_SELECT(divSelect) | ANADIG_PLL_PLL_480_CTRL_PLL_REG_EN(1) |
                        ANADIG_PLL_PLL_480_CTRL_PLL_480_GATE(1);
    ANADIG_PLL->PLL_480_CTRL = regValue;
    sw_delay_us(30); // Wait until the LDO is stable (controlled by REG_EN bit)
    // REG_EN=1, DIVSEL=3/5, GATE=1, POWERUP=1, HOLDRING_OFF=1
    regValue |= ANADIG_PLL_PLL_480_CTRL_POWERUP(1) | ANADIG_PLL_PLL_480_CTRL_HOLD_RING_OFF_MASK;
    ANADIG_PLL->PLL_480_CTRL = regValue;
    sw_delay_us(30);
    // REG_EN=1, DIVSEL=3/5, GATE=1, POWERUP=1 HOLDRING_OFF=0
    regValue &= ~ANADIG_PLL_PLL_480_CTRL_HOLD_RING_OFF_MASK;
    ANADIG_PLL->PLL_480_CTRL = regValue;
    uint32_t waitTicks = 0;
    uint32_t timeoutTicks = (SystemCoreClock / FREQ_1MHz) * 30;
    while ((ANADIG_PLL->PLL_480_CTRL & ANADIG_PLL_PLL_480_CTRL_PLL_480_STABLE_MASK) !=
           ANADIG_PLL_PLL_480_CTRL_PLL_480_STABLE_MASK)
    {
        waitTicks++;
        if (waitTicks > timeoutTicks)
        {
            debug_printf("480_PLL config timeout, ANADIG_PLL->PLL_480_CTRL=%x\n", ANADIG_PLL->PLL_480_CTRL);
            return kStatus_Timeout;
        }
    }

    // REG_EN=1, DIVSEL=3/5, GATE=1, POWERUP=1 HOLDRING_OFF=0 CLK=1, DIV2=1
    regValue |= ANADIG_PLL_PLL_480_CTRL_ENABLE_CLK(1) | ANADIG_PLL_PLL_480_CTRL_PLL_480_DIV2(1);
    ANADIG_PLL->PLL_480_CTRL = regValue;
    // REG_EN=1, DIVSEL=3/5, GATE=0, POWERUP=1 HOLDRING_OFF=0
    regValue &= ~ANADIG_PLL_PLL_480_CTRL_PLL_480_GATE_MASK;
    ANADIG_PLL->PLL_480_CTRL = regValue;

    debug_printf("480_PLL config completed, ANADIG_PLL->PLL_480_CTRL=%x\n", ANADIG_PLL->PLL_480_CTRL);

    // Enable PLL480_PFD, PFD0=13, PFD1=17, PFD2=32, PFD3=24
    // PFD0=664.6MHz, PFD1=508MHz, PFD2=270MHz, PFD3=360MHz
    ANADIG_PLL->PLL_480_PFD = ANADIG_PLL_PLL_480_PFD_PFD0_FRAC(13) | ANADIG_PLL_PLL_480_PFD_PFD1_FRAC(17) |
                              ANADIG_PLL_PLL_480_PFD_PFD2_FRAC(32) | ANADIG_PLL_PLL_480_PFD_PFD3_FRAC(24);
    debug_printf("ANADIG_PLL->PLL_480_PFD=%x\n", ANADIG_PLL->PLL_480_PFD);

    uint32_t stableMask =
        ANADIG_PLL->PLL_480_PFD & (ANADIG_PLL_PLL_480_PFD_PFD0_STABLE_MASK | ANADIG_PLL_PLL_480_PFD_PFD1_STABLE_MASK |
                                   ANADIG_PLL_PLL_480_PFD_PFD2_STABLE_MASK | ANADIG_PLL_PLL_480_PFD_PFD3_STABLE_MASK);

    /* Note:
     *      1. The PFD output will be updated automatically if the CLOCKGATE is turned on from the the gate-off state,
     *         No need to toggle the UPDATE bit.
     *      2. The stable signal will be ready quickly enough, so SW don't need to wait the stable signal, by the way,
     *         this signal will revert every time after the UPDATE completes
     */
    ANADIG_PLL->PLL_480_UPDATE ^= ANADIG_PLL_PLL_480_UPDATE_PFD0_UPDATE(1) | ANADIG_PLL_PLL_480_UPDATE_PFD1_UPDATE(1) |
                                  ANADIG_PLL_PLL_480_UPDATE_PFD2_UPDATE(1) | ANADIG_PLL_PLL_480_UPDATE_PFD3_UPDATE(1);

    // Wait until the stable bits get updated
    waitTicks = 0;
    timeoutTicks = (SystemCoreClock / FREQ_1MHz) * 30;
    while (ANADIG_PLL->PLL_480_PFD & stableMask)
    {
        waitTicks++;
        if (waitTicks > timeoutTicks)
        {
            debug_printf("480_PFD configuration timeout, ANADIG_PLL->PLL_480_PFD=%x\n", ANADIG_PLL->PLL_480_PFD);
            return kStatus_Timeout;
        }
    }

    return kStatus_Success;
}

status_t configure_528_pll(uint32_t oscFreq)
{
    debug_printf("%s\n", __func__);

    uint32_t mfn = 0;
    uint32_t mfi = 0;
    uint32_t mfd = 0xffffffffUL;
    if (oscFreq == FREQ_24MHz)
    {
        mfn = 0;
        mfi = 22;
    }
    else if (oscFreq == FREQ_19P2MHz)
    {
        mfn = 0x80000000UL;
        mfi = 27;
    }
    else
    {
        return kStatus_InvalidArgument;
    }
    // 1. Configure PLL register
    // 2. Enable interl LDO
    // 3. Wait LDO stable
    // 4. Power up PLL, assert holdring_off
    // 5. At half lock time, de-assert hondring off
    // 6. Wait PLL lock
    // 7. Enable clock out
    // Note: lock time: ARM_PLL 50us, PLL_480 50us, PLL_528 450us
    // Gate off all the PFDs first
    uint32_t pfdClockGateMask =
        ANADIG_PLL_PLL_528_PFD_PFD0_DIV1_CLKGATE(1) | ANADIG_PLL_PLL_528_PFD_PFD1_DIV1_CLKGATE(1) |
        ANADIG_PLL_PLL_528_PFD_PFD2_DIV1_CLKGATE(1) | ANADIG_PLL_PLL_528_PFD_PFD3_DIV1_CLKGATE(1);
    ANADIG_PLL->PLL_528_PFD |= pfdClockGateMask;

    // Freq = Fref * (MFI + MFN / MFD)
    ANADIG_PLL->PLL_528_MFD = mfd;
    ANADIG_PLL->PLL_528_MFN = mfn;
    ANADIG_PLL->PLL_528_MFI = mfi;

    // REG_EN=1, GATE=1 DIV_SEL = 0 POWERUP=0
    uint32_t regValue = ANADIG_PLL_PLL_528_CTRL_PLL_REG_EN(1) | ANADIG_PLL_PLL_528_CTRL_PLL_528_GATE(1);
    ANADIG_PLL->PLL_528_CTRL = regValue;
    sw_delay_us(30); // Wait until the LDO is stable (controlled by the REG_EN)
                     // REG_EN=1, GATE=1 DIV_SEL = 0 POWERUP=1 HOLDRING_OFF=1
    regValue |= ANADIG_PLL_PLL_528_CTRL_POWERUP(1) | ANADIG_PLL_PLL_528_CTRL_HOLD_RING_OFF_MASK;
    ANADIG_PLL->PLL_528_CTRL = regValue;
    sw_delay_us(250);
    // REG_EN=1, GATE=1 DIV_SEL = 0 POWERUP=1 HOLDRING_OFF = 0
    regValue &= ~ANADIG_PLL_PLL_528_CTRL_HOLD_RING_OFF_MASK;
    ANADIG_PLL->PLL_528_CTRL = regValue;
    uint32_t waitTicks = 0;
    uint32_t timeoutTicks = (SystemCoreClock / FREQ_1MHz) * 250;
    while ((ANADIG_PLL->PLL_528_CTRL & ANADIG_PLL_PLL_528_CTRL_PLL_528_STABLE_MASK) !=
           ANADIG_PLL_PLL_528_CTRL_PLL_528_STABLE_MASK)
    {
        waitTicks++;
        if (waitTicks > timeoutTicks)
        {
            debug_printf("528_PLL config timeout, ANADIG_PLL->PLL_528_CTRL=%x\n", ANADIG_PLL->PLL_528_CTRL);
            return kStatus_Timeout;
        }
    }

    // REG_EN=1, GATE=0 DIV_SEL = 0 POWERUP=1 HOLDRING_OFF=0 CLK=1
    regValue |= ANADIG_PLL_PLL_528_CTRL_ENABLE_CLK_MASK;
    // Enable PLL528 Clock
    ANADIG_PLL->PLL_528_CTRL = regValue;
    // REG_EN=1, GATE=0 DIV_SEL = 0 POWERUP=1 HOLDRING_OFF=0
    regValue &= ~ANADIG_PLL_PLL_528_CTRL_PLL_528_GATE_MASK;
    ANADIG_PLL->PLL_528_CTRL = regValue;
    debug_printf("528_PLL config completed, ANADIG_PLL->PLL_528_CTRL=%x\n", ANADIG_PLL->PLL_528_CTRL);

    // Enable PLL528_PFD PFD0=27, PFD1=16, PFD2=24, PFD3=32
    // PFD0=352MHz, PFD1=594MHz, PFD2=396MHz, PFD3=297MHz
    ANADIG_PLL->PLL_528_PFD = ANADIG_PLL_PLL_528_PFD_PFD0_FRAC(27) | ANADIG_PLL_PLL_528_PFD_PFD1_FRAC(16) |
                              ANADIG_PLL_PLL_528_PFD_PFD2_FRAC(24) | ANADIG_PLL_PLL_528_PFD_PFD3_FRAC(32);
    debug_printf("ANADIG_PLL->PLL_528_PFD=%x\n", ANADIG_PLL->PLL_528_PFD);
    /* Note:
     *      1. The PFD output will be updated automatically if the CLOCKGATE is turned on from the the gate-off state,
     *         No need to toggle the UPDATE bit.
     *      2. The stable signal will be ready quickly enough, so SW don't need to wait the stable signal, by the way,
     *        this signal will revert every time after the UPDATE completes
     */

    uint32_t stableMask =
        ANADIG_PLL->PLL_528_PFD & (ANADIG_PLL_PLL_528_PFD_PFD0_STABLE(1) | ANADIG_PLL_PLL_528_PFD_PFD1_STABLE(1) |
                                   ANADIG_PLL_PLL_528_PFD_PFD2_STABLE(1) | ANADIG_PLL_PLL_528_PFD_PFD3_STABLE(1));
    // Trigger PFD update
    ANADIG_PLL->PLL_528_UPDATE ^= (ANADIG_PLL_PLL_528_UPDATE_PFD0_UPDATE(1) | ANADIG_PLL_PLL_528_UPDATE_PFD1_UPDATE(1) |
                                   ANADIG_PLL_PLL_528_UPDATE_PFD2_UPDATE(1) | ANADIG_PLL_PLL_528_UPDATE_PFD3_UPDATE(1));

    // Wait until the stable bits get updated
    waitTicks = 0;
    timeoutTicks = (SystemCoreClock / FREQ_1MHz) * 250;
    while (ANADIG_PLL->PLL_528_PFD & stableMask)
    {
        waitTicks++;
        if (waitTicks > timeoutTicks)
        {
            debug_printf("528_PFD config timeout, ANADIG_PLL->PLL_528_PFD=%x\n", ANADIG_PLL->PLL_528_PFD);
            return kStatus_Timeout;
        }
    }

    return kStatus_Success;
}

status_t configure_plls(bool enableArmPll)
{
    status_t status = kStatus_Fail;
    // Enable OSC, wait until OSC is stable
    ANADIG_OSC->OSC_24M_CTRL = 0x14; // OSC_EN=1, LP_EN=1, GATE=0, BYPASS=0
    while (!(ANADIG_OSC->OSC_24M_CTRL & 0x40000000u))
    {
    }

    uint32_t oscFreq = get_osc_freq();

    if (enableArmPll)
    {
        status = configure_arm_pll(oscFreq);
        if (status != kStatus_Success)
        {
            return status;
        }
    }

    enable_pll_ldo_power();

    // Enable PLL480 and its PFDs
    status = configure_480_pll(oscFreq);
    if (status != kStatus_Success)
    {
        return status;
    }

    // Enable PLL528 and its PFDs
    status = configure_528_pll(oscFreq);
    if (status != kStatus_Success)
    {
        return status;
    }

    return kStatus_Success;
}

uint32_t get_cm7_core_src(void)
{
    if (FUSE_BOOT_FREQ_VALUE)
    {
        return kCm7ClockSrc_PLL_ARM;
    }
    else
    {
        return kCm7ClockSrc_RC400M;
    }
}

uint32_t get_cm4_core_src(void)
{
    if (FUSE_BOOT_FREQ_VALUE)
    {
        return kCm4ClockSrc_PLL_480;
    }
    else
    {
        return kCm4ClockSrc_RC400M;
    }
}

void configure_m7_boot_clock(bootloader_clock_option_t option)
{
    uint32_t postDivider = 1 << (FUSE_LPB_BOOT_VALUE);
    if (postDivider > 8)
    {
        postDivider = 1;
    }

    if (option == kClockOption_EnterBootloader)
    {
        // Switch the critical clock roots to the safe clock root before doing any clock updates
        CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM7].CONTROL = CCM_CONTROL_DIV(0) | CCM_CONTROL_MUX(0);
        CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM4].CONTROL = CCM_CONTROL_DIV(0) | CCM_CONTROL_MUX(0);
        // Configure Clock Divider, MUX
        CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUS].CONTROL = CCM_CONTROL_DIV(0) | CCM_CONTROL_MUX(0);
        CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUSL].CONTROL = CCM_CONTROL_DIV(0) | CCM_CONTROL_MUX(0);

        /*
            The M7 supports 2 typical clock frequencies:
                - 400MHz from RC400M
                - 800MHz from PLL_ARM with OSC24M/19.2M as reference clock
            To support 400MHz, ROM needs to perform RC400M TRIM routine first
         */
        // Always enable RC400M
        configure_rc400m();
        rc16m_trim();

        uint32_t coreSrc = get_cm7_core_src();
        if (coreSrc == kCm7ClockSrc_RC400M)
        {
            uint32_t clockToSwitch = FREQ_400MHz;

            uint32_t cm7ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M7) * postDivider;
            uint32_t cm4ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M4) * postDivider;
            uint32_t busClockDiv = get_clock_div(clockToSwitch, kMaxFreq_Bus) * postDivider;
            uint32_t lowpowerBusClock = get_clock_div(clockToSwitch, kMaxFreq_BusL) * postDivider;
            uint32_t flexramClkDiv = get_clock_div(clockToSwitch, kMaxFreq_FLEXRAM);

            // Configure FLEXRAM clock before switching to higher frequency
            CCM->CLOCK_GROUP[kCCM_ClockGroup_Flexram].CONTROL =
                CCM_CLOCK_GROUP_CTRL_DIV0(flexramClkDiv - 1) | CCM_CLOCK_GROUP_CTRL_RSTDIV(flexramClkDiv - 1);

            // Wait until switch done
            while (CCM->CLOCK_GROUP[kCCM_ClockGroup_Flexram].STATUS0 & 0x80000000u)
            {
            }

            // Configure Clock Source and divider, switch to RC400M
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM7].CONTROL =
                CCM_CONTROL_DIV(cm7ClockDiv - 1) | CCM_CONTROL_MUX(2);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM4].CONTROL =
                CCM_CONTROL_DIV(cm4ClockDiv - 1) | CCM_CONTROL_MUX(2);
            // Configure Clock Divider, MUX
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUS].CONTROL =
                CCM_CONTROL_DIV(busClockDiv - 1) | CCM_CONTROL_MUX(2);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUSL].CONTROL =
                CCM_CONTROL_DIV(lowpowerBusClock - 1) | CCM_CONTROL_MUX(2);

            // Enable PLL for FlexSPI, SEMC and other IP use.
            status_t status = configure_plls(false);
            if (status != kStatus_Success)
            {
                // TODO: do something here
            }
#ifndef BL_TARGET_FPGA
            // Select LPUART clock frequency: From RC400M / 5 = 80MHz
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_UART1].CONTROL = CCM_CONTROL_DIV(4) | CCM_CONTROL_MUX(2);
#endif
            SystemCoreClock = clockToSwitch / cm7ClockDiv;
        }
        else if (coreSrc == kCm7ClockSrc_PLL_ARM)
        {
            uint32_t cm7ClockDiv = get_clock_div(FREQ_696MHz, kMaxFreq_M7) * postDivider;
            uint32_t cm4ClockDiv = get_clock_div(FREQ_400MHz, kMaxFreq_M4) * postDivider;
            uint32_t busClockDiv = get_clock_div(FREQ_480MHz, kMaxFreq_Bus) * postDivider;
            uint32_t lowpowerBusClock = get_clock_div(FREQ_360MHz, kMaxFreq_BusL) * postDivider;
            uint32_t flexramClkDiv = get_clock_div(FREQ_696MHz, kMaxFreq_FLEXRAM) * postDivider;

            // Configure FLEXRAM clock before switch to higher frequency
            CCM->CLOCK_GROUP[kCCM_ClockGroup_Flexram].CONTROL =
                CCM_CLOCK_GROUP_CTRL_DIV0(flexramClkDiv - 1) | CCM_CLOCK_GROUP_CTRL_RSTDIV(flexramClkDiv - 1);
            // Wait until switch done
            while (CCM->CLOCK_GROUP[kCCM_ClockGroup_Flexram].STATUS0 & 0x80000000u)
            {
            }

            // Enable PLL for FlexSPI, SEMC and other IP use.
            status_t status = configure_plls(true);
            if (status != kStatus_Success)
            {
                // TODO: do something here
            }

            // Swtich clock soruce to PLL
            // Configure Clock Divider and MUX
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM7].CONTROL =
                CCM_CONTROL_DIV(cm7ClockDiv - 1) | CCM_CONTROL_MUX(4);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM4].CONTROL =
                CCM_CONTROL_DIV(cm4ClockDiv - 1) | CCM_CONTROL_MUX(2);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUS].CONTROL =
                CCM_CONTROL_DIV(busClockDiv - 1) | CCM_CONTROL_MUX(4);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUSL].CONTROL =
                CCM_CONTROL_DIV(lowpowerBusClock - 1) | CCM_CONTROL_MUX(4);

#ifndef BL_TARGET_FPGA
            // Select LPUART clock frequency - From PLL480_div / 3 = 240MHz / 3 = 80MHz
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_UART1].CONTROL = CCM_CONTROL_DIV(2) | CCM_CONTROL_MUX(4);
#endif

            SystemCoreClock = FREQ_696MHz;
        }

#if BL_FEATURE_FLEXSPI_NOR_MODULE
        // Set FlexSPI Boot clock source
//#warning sbl-tip3: Need to find out the failure cause of calling set_clock_source() api in sbl, note that it is ok in flashloader
        //flexspi_update_clock_source(get_flexspinor_instance(), kFlexSpiBootClkcSrc);
#endif

        debug_printf("Clock configuration done, SystemCoreClock = %dMHz\n", SystemCoreClock / FREQ_1MHz);
    }
    else if (option == kClockOption_ExitBootloader)
    {
    }
}

void configure_m4_boot_clock(bootloader_clock_option_t option)
{
    uint32_t postDivider = 1 << (FUSE_LPB_BOOT_VALUE);
    if (postDivider > 8)
    {
        postDivider = 1;
    }

    if (option == kClockOption_EnterBootloader)
    {
        /*
            The CM4 supports 1 typical clock frequency:
                - 200MHz from RC400M
            To support 200MHz, ROM needs to perform RC4800M TRIM routine first
         */
        configure_rc400m();
        rc16m_trim();

        // Bypass LD0 1P0 and 1P8 to achieve enough power
        bypass_ldo_1p8_and_1p0();

        uint32_t coreSrc = get_cm4_core_src();
        uint32_t clockToSwitch;
        uint32_t cm4ClockDiv;
        if (coreSrc == kCm4ClockSrc_RC400M)
        {
            uint32_t clockToSwitch = FREQ_400MHz;

            uint32_t cm7ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M7) * postDivider;
            uint32_t cm4ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M4) * postDivider;
            uint32_t busClockDiv = get_clock_div(clockToSwitch, kMaxFreq_Bus) * postDivider;
            uint32_t lowpowerBusClock = get_clock_div(clockToSwitch, kMaxFreq_BusL) * postDivider;
            uint32_t flexramClkDiv = get_clock_div(clockToSwitch, kMaxFreq_FLEXRAM) * postDivider;

            // Configure FLEXRAM clock before switching to higher frequency
            CCM->CLOCK_GROUP[kCCM_ClockGroup_Flexram].CONTROL =
                CCM_CLOCK_GROUP_CTRL_DIV0(flexramClkDiv - 1) | CCM_CLOCK_GROUP_CTRL_RSTDIV(flexramClkDiv - 1);

            // Wait until switch done
            while (CCM->CLOCK_GROUP[kCCM_ClockGroup_Flexram].STATUS0 & 0x80000000u)
            {
            }

            // Configure Clock Source and divider, switch to RC400M
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM7].CONTROL =
                CCM_CONTROL_DIV(cm7ClockDiv - 1) | CCM_CONTROL_MUX(2);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM4].CONTROL =
                CCM_CONTROL_DIV(cm4ClockDiv - 1) | CCM_CONTROL_MUX(2);
            // Configure Clock Divider, MUX
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUS].CONTROL =
                CCM_CONTROL_DIV(busClockDiv - 1) | CCM_CONTROL_MUX(2);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUSL].CONTROL =
                CCM_CONTROL_DIV(lowpowerBusClock - 1) | CCM_CONTROL_MUX(2);

            // Enable PLL for FlexSPI, SEMC and other IP use.
            status_t status = configure_plls(false);
            if (status != kStatus_Success)
            {
                // TODO: do something here
            }

#ifndef BL_TARGET_FPGA
            // Select LPUART clock frequency: From RC400M / 5 = 80MHz
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_UART1].CONTROL = CCM_CONTROL_DIV(4) | CCM_CONTROL_MUX(2);
#endif

            SystemCoreClock = clockToSwitch / cm7ClockDiv;
        }
        else
        {
            clockToSwitch = FREQ_480MHz;

            uint32_t cm7ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M7) * postDivider;
            cm4ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M4) * postDivider;
            uint32_t busClockDiv = get_clock_div(clockToSwitch, kMaxFreq_Bus) * postDivider;
            uint32_t lowpowerBusClock = get_clock_div(clockToSwitch, kMaxFreq_BusL) * postDivider;
            uint32_t flexramClkDiv = get_clock_div(clockToSwitch, kMaxFreq_FLEXRAM) * postDivider;

            // Configure FLEXRAM clock before switching to higher frequency
            CCM->CLOCK_GROUP[kCCM_ClockGroup_Flexram].CONTROL =
                CCM_CLOCK_GROUP_CTRL_DIV0(flexramClkDiv - 1) | CCM_CLOCK_GROUP_CTRL_RSTDIV(flexramClkDiv - 1);

            // Wait until switch done
            while (CCM->CLOCK_GROUP[kCCM_ClockGroup_Flexram].STATUS0 & 0x80000000u)
            {
            }

            // Enable PLL for FlexSPI, SEMC and other IP use.
            status_t status = configure_plls(false);
            if (status != kStatus_Success)
            {
                // This path should never be touched considering the use case of RT117x
            }

            // Configure Clock Source and divider, switch to Pll480 for CM7 and CM4 core
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM7].CONTROL =
                CCM_CONTROL_DIV(cm7ClockDiv - 1) | CCM_CONTROL_MUX(6);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM4].CONTROL =
                CCM_CONTROL_DIV(cm4ClockDiv - 1) | CCM_CONTROL_MUX(5);
            // Configure Clock Divider, MUX
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUS].CONTROL =
                CCM_CONTROL_DIV(busClockDiv - 1) | CCM_CONTROL_MUX(4);
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUSL].CONTROL =
                CCM_CONTROL_DIV(lowpowerBusClock - 1) | CCM_CONTROL_MUX(5);
#ifndef BL_TARGET_FPGA
            // Select LPUART clock frequency: From PLL480_Div2 / 3 = 240MHz / 3 = 80MHz
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_UART1].CONTROL =
                CCM_CONTROL_DIV(2) | CCM_CONTROL_MUX(4);
#endif
        }

#if BL_FEATURE_FLEXSPI_NOR_MODULE
        // Set FlexSPI Boot clock source
        //flexspi_update_clock_source(get_flexspinor_instance(), kFlexSpiBootClkcSrc);
#endif
    }
    else if (option == kClockOption_ExitBootloader)
    {
    }
}

void enable_pll_ldo_power(void)
{
    // Enable PLL LDO Power
    ANADIG_MISC->VDDSOC_AI_CTRL = 0;
    ANADIG_MISC->VDDSOC_AI_WDATA = 0x0105;
    ANADIG_PMU->PMU_LDO_PLL ^= 0x10000;
    sw_delay_us(100);
    ANADIG_MISC->VDDSOC_AI_CTRL = ANADIG_MISC_VDDSOC_AI_CTRL_VDDSOC_AIRWB_MASK;
    ANADIG_PMU->PMU_LDO_PLL ^= 0x10000;
    ANADIG_PMU->PMU_POWER_DETECT_CTRL = 0x100;
    sw_delay_us(1);
    ANADIG_PMU->PMU_REF_CTRL |= 0x10;
}

bool reuse_rom_clocks(void)
{
    if (!(CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM7].CONTROL && CCM->CLOCK_ROOT[kCCM_ClockRootIndex_CM7].CONTROL))
    {
        return false;
    }

    uint32_t postDivider = 1 << (FUSE_LPB_BOOT_VALUE);
    if (postDivider > 8)
    {
        postDivider = 1;
    }

    if (is_cm4_boot())
    {
        uint32_t clockToSwitch;
        uint32_t cm4ClockDiv;
        if (get_cm4_core_src() == kCm4ClockSrc_RC400M)
        {
            clockToSwitch = FREQ_400MHz;
            cm4ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M4) * postDivider;
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_UART1].CONTROL = CCM_CONTROL_DIV(4) | CCM_CONTROL_MUX(2);
        }
        else
        {
            clockToSwitch = FREQ_480MHz;
            cm4ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M4) * postDivider;
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_UART1].CONTROL = CCM_CONTROL_DIV(2) | CCM_CONTROL_MUX(4);
        }
        SystemCoreClock = clockToSwitch / cm4ClockDiv;
    }
    else
    {
        if (get_cm7_core_src() == kCm7ClockSrc_RC400M)
        {
            uint32_t clockToSwitch = FREQ_400MHz;
            uint32_t cm7ClockDiv = get_clock_div(clockToSwitch, kMaxFreq_M7) * postDivider;
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_UART1].CONTROL = CCM_CONTROL_DIV(4) | CCM_CONTROL_MUX(2);
            SystemCoreClock = clockToSwitch / cm7ClockDiv;
        }
        else /* get_cm7_core_src() == kCm7ClockSrc_PLL_ARM */
        {
            CCM->CLOCK_ROOT[kCCM_ClockRootIndex_UART1].CONTROL = CCM_CONTROL_DIV(2) | CCM_CONTROL_MUX(4);
            if (FUSE_SPEED_LIMIT_VALUE == 1)
            {
                SystemCoreClock = FREQ_500MHz / postDivider;
            }
            else
            {
                SystemCoreClock = FREQ_696MHz / postDivider;
            }
        }
    }

    return true;
}

// See bootloader_common for documentation on this function.
void configure_clocks(bootloader_clock_option_t option)
{
    debug_printf("configure_clocks()\n");
#ifndef BL_TARGET_FPGA
    if (option == kClockOption_EnterBootloader)
    {
        SystemCoreClock = FREQ_24MHz; // Default core clock before initializing the boot clock
        if (reuse_rom_clocks())
        {
            return;
        }
    }
    debug_printf("SystemCoreClock=%dMHz\n", SystemCoreClock / 1000000);

    if (is_cm4_boot())
    {
        configure_m4_boot_clock(option);
    }
    else
    {
        configure_m7_boot_clock(option);
    }
#else
    SystemCoreClock = 24000000u;
    debug_printf("SystemCoreClock=%dMHz\n", SystemCoreClock / 1000000);
#endif
}

#define USBPHY_PLL_DIV_SEL_20 3
#define USBPHY_PLL_DIV_SEL_25 5
bool usb_clock_init(uint32_t controllerId)
{
    debug_printf("Bootloader: usb_clock_init()---- Start.\n");

    USB1->USBCMD |= USBHS_USBCMD_RST_MASK;

    // Clear PHY from reset
    USBPHY1->CTRL_CLR = USBPHY_CTRL_SFTRST_MASK;
    // clear UTMI CLKGATE
    USBPHY1->CTRL &= ~USBPHY_CTRL_CLKGATE_MASK;
    USBPHY1->CTRL |= USBPHY_CTRL_ENUTMILEVEL2_MASK | USBPHY_CTRL_ENUTMILEVEL3_MASK;

    USBPHY1->CTRL |= USBPHY_CTRL_ENAUTOCLR_CLKGATE_MASK | USBPHY_CTRL_ENAUTOCLR_PHY_PWD_MASK;

    // Enable regulator
    USBPHY1->PLL_SIC |= USBPHY_PLL_SIC_PLL_REG_ENABLE_MASK;

    // FUSE_BOOT_OSC_REF_VALUE = 1'b0, 24 MHz input clock with default divid by 20
    // FUSE_BOOT_OSC_REF_VALUE = 1'b1, 19.2 MHz input clock with default divid by 25
    uint32_t pllSicVal = (USBPHY1->PLL_SIC & ~USBPHY_PLL_SIC_PLL_DIV_SEL_MASK);
    if (get_osc_freq() == FREQ_24MHz)
    {
        pllSicVal |= USBPHY_PLL_SIC_PLL_DIV_SEL(USBPHY_PLL_DIV_SEL_20);
    }
    else
    {
        pllSicVal |= USBPHY_PLL_SIC_PLL_DIV_SEL(USBPHY_PLL_DIV_SEL_25);
    }
    USBPHY1->PLL_SIC = pllSicVal;
    debug_printf("USBPHY1->PLL_SIC = %x\n", USBPHY1->PLL_SIC);

    // Enable the power
    USBPHY1->PLL_SIC |= USBPHY_PLL_SIC_PLL_POWER_MASK | USBPHY_PLL_SIC_PLL_ENABLE_MASK;
    // Wait for PLL lock
    microseconds_delay(100);

    // Enable usb clocks
    USBPHY1->PLL_SIC |= USBPHY_PLL_SIC_PLL_EN_USB_CLKS_MASK;
    // Clear bypass bit
    USBPHY1->PLL_SIC_CLR = USBPHY_PLL_SIC_PLL_BYPASS_MASK;

    // clear UTMI CLKGATE, to run the clocks
    USBPHY1->CTRL &= ~USBPHY_CTRL_CLKGATE_MASK;
    // Set to normal mode
    USBPHY1->PWD = 0;
    debug_printf("Bootloader: usb_clock_init()---- End.\n");

    USB1->USBCMD &= (uint32_t)~USBHS_USBCMD_RS_MASK;

    return true;
}

// Get OCOTP clock
uint32_t get_ocotp_clock(void)
{
//#warning need to implement later
    return 0;
}

uint32_t get_bus_clock(void)
{
#ifndef BL_TARGET_FPGA
    // PIT is from BUS clock, the maximum bus clock is 200MHz
    uint32_t clkRoot = CCM->CLOCK_ROOT[kCCM_ClockRootIndex_BUS].CONTROL;
    uint32_t div = (clkRoot & CCM_CLOCK_ROOT_CONTROL_DIV_MASK) >> CCM_CLOCK_ROOT_CONTROL_DIV_SHIFT;
    uint32_t mux = (clkRoot & CCM_CLOCK_ROOT_CONTROL_MUX_MASK) >> CCM_CLOCK_ROOT_CONTROL_MUX_SHIFT;
    uint32_t busFreq = 0;

    switch (mux)
    {
        default:
        case 0:
            busFreq = FREQ_1MHz * 24u;
            break;
        case 1:
            busFreq = FREQ_1MHz * 24u;
            break;
        case 2:
            busFreq = FREQ_1MHz * 400u;
            break;
        case 3:
            busFreq = FREQ_1MHz * 16u;
            break;
        case 4:
            busFreq = FREQ_1MHz * 480u;
            break;
        case 5:
            busFreq = FREQ_1MHz * 200u;
            break;
        case 6:
            busFreq = FREQ_1MHz * 528u;
            break;
        case 7:
            busFreq = FREQ_1MHz * 297u;
            break;
    }
    return busFreq / (div + 1);
#else
    return 12000000u;
#endif
}

//! @brief Gets the clock value used for microseconds driver
uint32_t microseconds_get_clock(void)
{
    return get_bus_clock();
}

//! @brief Return uart clock frequency according to instance
uint32_t get_uart_clock(uint32_t instance)
{
    // LPUART1 clock has been configured to 80MHz in clock_configure
    uint32_t lpuart_clock = 0;

#if !defined(BL_TARGET_FPGA)
    lpuart_clock = 80 * FREQ_1MHz; // 80MHz;
#else
    lpuart_clock = 12 * FREQ_1MHz; // 12MHz;
#endif //  #if !defined (BL_TARGET_ZEBU) && !defined (BL_TARGET_FPGA)

    return lpuart_clock;
}




////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
