/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "property/property.h"
#include "fsl_device_registers.h"
#include "utilities/fsl_assert.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "target_config.h"
#include "fsl_iap.h"
#include "fsl_iap_ffr.h"
#include "microseconds/microseconds.h"
//#include "authentication/secure_kboot.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#if defined BL_TARGET_FPGA
uint32_t busClock = 6000000u; //! 6MHz default bus clock
#endif

enum
{
    kFreq_1MHz = 1000000u,
    kFreq_12MHz = 12u * kFreq_1MHz,
    kFreq_24MHz = 24u * kFreq_1MHz,
    kFreq_48MHz = 48u * kFreq_1MHz,
    kFreq_96MHz = 96u * kFreq_1MHz
};

enum
{
    MAINCLKSELA_FRO_12MHz = 0,
    MAINCLKSELA_CLKIN,
    MAINCLKSELA_FRO_1MHz,
    MAINCLKSELA_FRO_HF_96MHz,
};

enum
{
    MAINCLKSELB_MAINCLKAOUT = 0,
    MAINCLKSELB_PLL0_CLK,
    MAINCLKSELB_PLL1_CLK,
    MAINCLKSELB_32K_OSC,
};

enum
{
    USBFSCLKSEL_MAIN_CLK = 0,
    USBFSCLKSEL_PLL0_CLK = 1,
    USBFSCLKSEL_FRO_HF_96MHz = 3,
    USBFSCLKSEL_PLL1_CLK = 5,
};

enum
{
    USBHSCLKSEL_MAIN_CLK = 0,
    USBHSCLKSEL_PLL0_CLK = 1,
    USBHSCLKSEL_CLKIN = 2,
    USBHSCLKSEL_PLL1_CLK = 5,
};

enum
{
    kBootSpeedSetting_Nmpa = 0,
    kBootSpeedSetting_96MHz = 1,
    kBootSpeedSetting_48MHz = 2,
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See bootloader_common for documentation on this function.
void configure_clocks(bootloader_clock_option_t option)
{
#ifndef BL_TARGET_FPGA
    if (kClockOption_EnterBootloader == option)
    {
        uint32_t sysClkDiv = 1;

        volatile uint32_t ffrVal = 0;
        cmpa_cfg_info_t *cmpaCfgInfo = (cmpa_cfg_info_t *)0;
        uint32_t bootCfgOffset = (uint8_t *)&cmpaCfgInfo->bootCfg - (uint8_t *)cmpaCfgInfo;
        status_t status =
            FFR_GetCustomerData(&g_bootloaderContext.allFlashState[0], (uint8_t *)&ffrVal, bootCfgOffset, 4);
        uint32_t bootSpeed = kBootSpeedSetting_48MHz;
        if (status == kStatus_Success)
        {
            bootSpeed = (ffrVal & 0x180) >> 7;
        }

        bool useHfoDivAsClockSource = false;
        bool needReConfigureClock = false;

        switch (bootSpeed)
        {
            case kBootSpeedSetting_Nmpa:
            {
                int32_t coreClock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
                if (coreClock > kFreq_12MHz)
                {
                    useHfoDivAsClockSource = true;
                    needReConfigureClock = true;
                    sysClkDiv = 1 + (SYSCON->AHBCLKDIV & SYSCON_AHBCLKDIV_DIV_MASK);
                }
            }
            break;
            case kBootSpeedSetting_96MHz:
                SystemCoreClock = kFreq_96MHz;
                sysClkDiv = 1;
                useHfoDivAsClockSource = true;
                needReConfigureClock = true;
                break;
            case kBootSpeedSetting_48MHz:
            default:
                SystemCoreClock = kFreq_48MHz;
                sysClkDiv = 2;
                useHfoDivAsClockSource = true;
                needReConfigureClock = true;
                break;
        }

        // Update available peripherals, if USB is enabled, override the default NMPA setting
        update_available_peripherals();
        bootloader_configuration_data_t *config = &g_bootloaderContext.propertyInterface->store->configurationData;
        if ((config->enabledPeripherals & kPeripheralType_USB_HID) && (!needReConfigureClock))
        {
            needReConfigureClock = true;
            useHfoDivAsClockSource = true;
            SystemCoreClock = kFreq_48MHz;
            sysClkDiv = 2;
        }

        debug_printf("bootSpeed=%d, needReConfigureClock=%s, useHfoDivAsClockSource=%d,sysClkDiv=%d\n", bootSpeed,
                     needReConfigureClock ? "true" : "false", useHfoDivAsClockSource, sysClkDiv);

        if (needReConfigureClock)
        {
            // Need to enable FRO_HF(96MHz) for USB or high speed boot
            if (!(ANACTRL->FRO192M_CTRL & ANACTRL_FRO192M_CTRL_ENA_96MHZCLK_MASK))
            {
                ANACTRL->FRO192M_CTRL |= ANACTRL_FRO192M_CTRL_ENA_96MHZCLK(1);
            }

            /* Configure CPU clock divider : divide by sysClkDiv */
            // Adjust Flash Controller and FMC before switching system clock (increase)
            FLASH_SetProperty(&g_bootloaderContext.allFlashState[kFlashIndex_Main], kFLASH_PropertyPflashSystemFreq,
                              (SystemCoreClock / kFreq_1MHz));
            FLASH_ConfigTiming(&g_bootloaderContext.allFlashState[kFlashIndex_Main]);

            if ((SYSCON->AHBCLKDIV & SYSCON_AHBCLKDIV_DIV_MASK) != (sysClkDiv - 1))
            {
                SYSCON->AHBCLKDIV = SYSCON_AHBCLKDIV_DIV(sysClkDiv - 1);
                __ISB();
                // Wait until divider is stable
                while (SYSCON->AHBCLKDIV & SYSCON_AHBCLKDIV_REQFLAG_MASK)
                {
                }
            }

            /* Switch Main clock to FRO192M 96 MHz output */
            SYSCON->MAINCLKSELA = MAINCLKSELA_FRO_HF_96MHz;
            SYSCON->MAINCLKSELB = MAINCLKSELB_MAINCLKAOUT;
        }

        // AHB clock has been configured in DD's startup
        SYSCON->SYSTICKCLKDIV0 = 0u;
        SYSCON->SYSTICKCLKSELX[0] = 0u;

        if (useHfoDivAsClockSource)
        {
            // Configure the FRO_HF_DIV clock to 48MHz
            SYSCON->FROHFDIV = 1;
            __ISB();
            while (SYSCON->FROHFDIV & SYSCON_FROHFDIV_REQFLAG_MASK)
            {
            }

            /* attach FROHF_DIV clock to FLEXCOMM0 (USART0) */
            CLOCK_AttachClk(kFRO_HF_DIV_to_FLEXCOMM0);

        }
        else
        {
            /* attach FRO12M clock to FLEXCOMM0 (USART0) */
            CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);
        }

        /* reset FLEXCOMM0 for USART0 */
        RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);
    }
    else
    {
        // Restore Clock related registers
        SYSCON->FROHFDIV = 0;
    }

#else
    if (kClockOption_EnterBootloader == option)
    {
        /* attach FRO12M clock to FLEXCOMM0 (USART0) */
        CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

        /* reset FLEXCOMM0 for USART0 */
        RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);
    }
    else
    {
    }

#endif
}
// See bootloader_common.h for documentation on this function.
// Note: this function doesn't apply to FPGA build
uint32_t get_system_core_clock(void)
{
    uint32_t systemCoreClock = SystemCoreClock;

    return systemCoreClock;
}

// See bootloader_common.h for documentation on this function.
uint32_t get_bus_clock(void)
{
    return 0;
}

status_t usb_fs0_device_clock_enable(void)
{
    /* disable usbfs device clock */
    CLOCK_DisableClock(kCLOCK_Usbd0);

#ifndef BL_TARGET_FPGA
    {
        uint32_t needHaltUsbClk;
        uint32_t clockSrc = USBFSCLKSEL_FRO_HF_96MHz;
        uint32_t clockDiv = 1;
        uint32_t curUsbClkSrc = (SYSCON->USB0CLKSEL & SYSCON_USB0CLKSEL_SEL_MASK) >> SYSCON_USB0CLKSEL_SEL_SHIFT;
        uint32_t curUsbDiv = (SYSCON->USB0CLKDIV & SYSCON_USB0CLKDIV_DIV_MASK) >> SYSCON_USB0CLKDIV_DIV_SHIFT;

        needHaltUsbClk = (curUsbDiv != clockDiv) || (curUsbClkSrc != clockSrc);
        if (needHaltUsbClk)
        {
            // Halt the counter if PDF reconfiguration is required to avoid possible glitch
            SYSCON->USB0CLKDIV |= (SYSCON_USB0CLKDIV_HALT_MASK);
        }
        /* Select osc as USB clock source*/
        if (clockSrc != curUsbClkSrc)
        {
            SYSCON->USB0CLKSEL = SYSCON_USB0CLKSEL_SEL(clockSrc);
        }
        if (clockDiv != curUsbDiv)
        {
            // Update USB clock divider
            SYSCON->USB0CLKDIV =
                SYSCON_USB0CLKDIV_DIV(clockDiv - 1) | SYSCON_USB0CLKDIV_RESET(1) | SYSCON_USB0CLKDIV_HALT(1);
        }
        if (SYSCON->USB0CLKDIV & SYSCON_USB0CLKDIV_HALT_MASK)
        {
            // Re-enable USB Clock
            SYSCON->USB0CLKDIV &= (uint32_t) ~(SYSCON_USB0CLKDIV_HALT_MASK | SYSCON_USB0CLKDIV_RESET_MASK);
        }
        debug_printf("Enter 3 --- %s, ", __func__);
        // Wait until clock change completes
        while (SYSCON->USB0CLKDIV & SYSCON_USB0CLKDIV_REQFLAG_MASK)
        {
        }
    }
#endif

    /* Enable usbfs device and ram clock */
    CLOCK_EnableClock(kCLOCK_Usbd0);
    CLOCK_EnableClock(kCLOCK_UsbRam1);
    /* Power up usb0 phy */
    PMC->PDRUNCFGCLR0 |= PMC_PDRUNCFG0_PDEN_USBFSPHY_MASK;

    return kStatus_Success;
}

void init_usbhs_phy_trim(void)
{
#define FLASH_NMPA_GPO2_0_ADDRS (0x9FC20)

    uint32_t usbhs_phy_trim_value;
    flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[kFlashIndex_Main];
    status_t status = FLASH_Read(flashConfig, FLASH_NMPA_GPO2_0_ADDRS, (uint8_t *)&usbhs_phy_trim_value,
                                 sizeof(usbhs_phy_trim_value));
    if (status != kStatus_FLASH_Success)
    {
        usbhs_phy_trim_value = 0;
    }
    /* Are USB Phy Trimming values in FLASH valid ? */
    if (usbhs_phy_trim_value & 0x01u)
    {
        //ANACTRL->USBHS_PHY_TRIM = (usbhs_phy_trim_value >> 1) & 0xFFFFFF;
        *(volatile uint32_t *)(ANACTRL_BASE + 0x104) = (usbhs_phy_trim_value >> 1) & 0xFFFFFF;
    }
}

/* Return values from Config (N-2) page of flash */
#define GET_16MXO_TRIM() (*(uint32_t *)0x9FCC8)
#define XO_SLAVE_EN (1)

uint8_t nio_osc_cap_convert(uint8_t u8OscCap, uint8_t u8CapBankDiscontinuity)
{
    /* Compensate for discontinuity in the capacitor banks */
    if (u8OscCap < 64)
    {
        if (u8OscCap >= u8CapBankDiscontinuity)
        {
            u8OscCap -= u8CapBankDiscontinuity;
        }
        else
        {
            u8OscCap = 0;
        }
    }
    else
    {
        if (u8OscCap <= (127 - u8CapBankDiscontinuity))
        {
            u8OscCap += u8CapBankDiscontinuity;
        }
        else
        {
            u8OscCap = 127;
        }
    }

    return u8OscCap;
}

void nio_set_xtal_16mhz_ldo(void)
{
    uint32_t temp;

    //const uint32_t u32Mask = (ANACTRL_LDO_XO32M_VOUT_MASK | ANACTRL_LDO_XO32M_IBIAS_MASK | ANACTRL_LDO_XO32M_STABMODE_MASK);
    const uint32_t u32Mask = (0x38U | 0xC0U | 0x300U);

    //const uint32_t u32Value = (ANACTRL_LDO_XO32M_VOUT(0x4) | ANACTRL_LDO_XO32M_IBIAS(0x2) | ANACTRL_LDO_XO32M_STABMODE(0x3));
    const uint32_t u32Value = 0x3A0;

    /* Enable & set-up XTAL 32 MHz clock LDO */
    //temp = ANACTRL->LDO_XO32M;
    temp = *(volatile uint32_t *)(ANACTRL_BASE + 0xB0);

    if ((temp & u32Mask) != u32Value)
    {
        temp &= ~u32Mask;

        /*
        * Enable the XTAL32M LDO
        * Adjust the output voltage level, 0x5 for 1.1V
        * Adjust the biasing current, 0x2 value
        * Stability configuration, 0x1 default mode
        */
        temp |= u32Value;

        //ANACTRL->LDO_XO32M = temp;
        *(volatile uint32_t *)(ANACTRL_BASE + 0xB0) = temp;
    }

    /* Enable LDO XO32M */
    PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_LDOXO32M_MASK;
}

void nio_set_xtal_16mhz_trim(uint32_t amp, uint32_t gm)
{
    uint32_t temp;
    //const uint32_t u32Mask = (ANACTRL_XO32M_CTRL_AMP_MASK | ANACTRL_XO32M_CTRL_GM_MASK);
    const uint32_t u32Mask = (0xE0U | 0xEU);
    //const uint32_t u32Value = (ANACTRL_XO32M_CTRL_AMP(amp) | ANACTRL_XO32M_CTRL_GM(gm));
    const uint32_t u32Value = ((((uint32_t)(((uint32_t)(amp)) << 5)) & 0xE0U) | (((uint32_t)(((uint32_t)(gm)) << 1)) & 0xEU));

    /* Set-up XTAL 16-MHz Trimmings */
    temp = ANACTRL->XO32M_CTRL;
    temp &= ~u32Mask;
    temp |= u32Value;
    ANACTRL->XO32M_CTRL = temp;
}

void nio_xtal_16mhz_capabank_trim(int32_t pi32_16MfXtalIecLoadpF_x100,
                                  int32_t pi32_16MfXtalPPcbParCappF_x100,
                                  int32_t pi32_16MfXtalNPcbParCappF_x100)
{
    uint32_t u32XOTrimValue;
    uint8_t u8IECXinCapCal6pF, u8IECXinCapCal8pF, u8IECXoutCapCal6pF, u8IECXoutCapCal8pF, u8XOSlave;
    int32_t iaXin_x4, ibXin, iaXout_x4, ibXout;
    int32_t iXOCapInpF_x100, iXOCapOutpF_x100;
    uint8_t u8XOCapInCtrl, u8XOCapOutCtrl;
    uint32_t u32RegVal;

    /* Enable and set LDO, if not already done */
    nio_set_xtal_16mhz_ldo();

    /* Get Cal values from Flash */
    u32XOTrimValue = GET_16MXO_TRIM();

    /* Check validity and apply */
    if ((u32XOTrimValue & 1) && ((u32XOTrimValue >> 15) & 1))
    {
        /* These fields are 7 bits, unsigned */
        u8IECXinCapCal6pF = (u32XOTrimValue >> 1) & 0x7f;
        u8IECXinCapCal8pF = (u32XOTrimValue >> 8) & 0x7f;
        u8IECXoutCapCal6pF = (u32XOTrimValue >> 16) & 0x7f;
        u8IECXoutCapCal8pF = (u32XOTrimValue >> 23) & 0x7f;

        /* This field is 1 bit */
        u8XOSlave = (u32XOTrimValue >> 30) & 0x1;

        /* Linear fit coefficients calculation */
        iaXin_x4 = (int)u8IECXinCapCal8pF - (int)u8IECXinCapCal6pF;
        ibXin = (int)u8IECXinCapCal6pF - iaXin_x4 * 3;
        iaXout_x4 = (int)u8IECXoutCapCal8pF - (int)u8IECXoutCapCal6pF;
        ibXout = (int)u8IECXoutCapCal6pF - iaXout_x4 * 3;
    }
    else
    {
        iaXin_x4 = 20;  // gain in LSB/pF
        ibXin = -9;     // offset in LSB
        iaXout_x4 = 20; // gain in LSB/pF
        ibXout = -13;   // offset in LSB
        u8XOSlave = 0;
    }

    /* In & out load cap calculation with derating */
    iXOCapInpF_x100 =
        2 * pi32_16MfXtalIecLoadpF_x100 - pi32_16MfXtalNPcbParCappF_x100 + 39 * (XO_SLAVE_EN - u8XOSlave) - 15;
    iXOCapOutpF_x100 = 2 * pi32_16MfXtalIecLoadpF_x100 - pi32_16MfXtalPPcbParCappF_x100 - 21;

    /* In & out XO_OSC_CAP_Code_CTRL calculation, with rounding */
    u8XOCapInCtrl = (uint8_t)((iXOCapInpF_x100 * iaXin_x4 + ibXin * 400) + 200) / 400;
    u8XOCapOutCtrl = (uint8_t)((iXOCapOutpF_x100 * iaXout_x4 + ibXout * 400) + 200) / 400;

    /* Read register and clear fields to be written */
    u32RegVal = ANACTRL->XO32M_CTRL;
    //u32RegVal &= ~(ANACTRL_XO32M_CTRL_OSC_CAP_IN_MASK | ANACTRL_XO32M_CTRL_OSC_CAP_OUT_MASK);
    u32RegVal &= ~(0x7F00U | 0x3F8000U);

/* Configuration of 32 MHz XO output buffers */
#if (XO_SLAVE_EN == 0)
    //u32RegVal &= ~(ANACTRL_XO32M_CTRL_SLAVE_MASK | ANACTRL_XO32M_CTRL_ACBUF_PASS_ENABLE_MASK);
    u32RegVal &= ~(0x10U | ANACTRL_XO32M_CTRL_ACBUF_PASS_ENABLE_MASK);
#else
    //u32RegVal |= ANACTRL_XO32M_CTRL_SLAVE_MASK | ANACTRL_XO32M_CTRL_ACBUF_PASS_ENABLE_MASK;
    u32RegVal |= 0x10U | ANACTRL_XO32M_CTRL_ACBUF_PASS_ENABLE_MASK;
#endif

    /* XO_OSC_CAP_Code_CTRL to XO_OSC_CAP_Code conversion */
    //u32RegVal |= nio_osc_cap_convert(u8XOCapInCtrl, 13) << ANACTRL_XO32M_CTRL_OSC_CAP_IN_SHIFT;
    u32RegVal |= nio_osc_cap_convert(u8XOCapInCtrl, 13) << 8U;
    //u32RegVal |= nio_osc_cap_convert(u8XOCapOutCtrl, 13) << ANACTRL_XO32M_CTRL_OSC_CAP_OUT_SHIFT;
    u32RegVal |= nio_osc_cap_convert(u8XOCapOutCtrl, 13) << 15U;

    /* Write back to register */
    ANACTRL->XO32M_CTRL = u32RegVal;
}

status_t usb_hs0_device_clock_enable(void)
{
#define FLASH_NMPA_USBCFG_ADDRS (0x9FC8C)
#define FLASH_NMPA_USBCFG_USB_USE_XO32M_CAPA_BANKS_MASK (0x10000U)

#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_ADDRS (0x9E434)
#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_VALID_MASK (0x1U)
#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_VALID_SHIFT (0U)
#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_XTAL_LOAD_CAP_IEC_PF_X100_MASK (0x000007FEU)
#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_XTAL_LOAD_CAP_IEC_PF_X100_SHIFT (1U)
#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_PCB_XIN_PARA_CAP_PF_X100_MASK (0x001FF800U)
#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_PCB_XIN_PARA_CAP_PF_X100_SHIFT (11U)
#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_PCB_XOUT_PARA_CAP_PF_X100_MASK (0x7FE00000U)
#define FLASH_CMPA_XTAL_16MHZ_CAPABANK_PCB_XOUT_PARA_CAP_PF_X100_SHIFT (21U)

    uint32_t usbcfg;
    uint32_t xtal_16mhz_capabank_cfg;
    int32_t xtal_iec_pf_x100;
    int32_t pcb_xin_pf_x100;
    int32_t pcb_xout_pf_x100;

    /* Enable usbhs device and ram clock */
    CLOCK_EnableClock(kCLOCK_Usbd1);
    CLOCK_EnableClock(kCLOCK_Usb1Clk);
    CLOCK_EnableClock(kCLOCK_UsbRam1);

#ifndef BL_TARGET_FPGA

    flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[kFlashIndex_Main];
    int32_t retryCnt = 4;
    status_t status;
    do
    {
        status = FLASH_Read(flashConfig, FLASH_NMPA_USBCFG_ADDRS, (uint8_t *)&usbcfg, sizeof(usbcfg));
    } while ((retryCnt-- > 0) && (status != kStatus_FLASH_Success));

    if (status != kStatus_FLASH_Success)
    {
        return status;
    }

    if (usbcfg & FLASH_NMPA_USBCFG_USB_USE_XO32M_CAPA_BANKS_MASK)
    {
        /* New configuration scheme : use internal XTAL 16MHz Capa Banks */

        flash_config_t *flashConfig = &g_bootloaderContext.allFlashState[kFlashIndex_Main];
        int32_t retryCnt = 4;
        status_t status;
        do
        {
            status = FLASH_Read(flashConfig, FLASH_CMPA_XTAL_16MHZ_CAPABANK_ADDRS, (uint8_t *)&xtal_16mhz_capabank_cfg,
                                sizeof(xtal_16mhz_capabank_cfg));
        } while ((retryCnt-- > 0) && (status != kStatus_FLASH_Success));

        if (status != kStatus_FLASH_Success)
        {
            return status;
        }

        /* Enable XO-16MHz LDO and configure XO-16MHz capa banks */
        if (xtal_16mhz_capabank_cfg & FLASH_CMPA_XTAL_16MHZ_CAPABANK_VALID_MASK)
        {
            /* Get parameters in CMPA */
            xtal_iec_pf_x100 = (int32_t)(
                (xtal_16mhz_capabank_cfg >> FLASH_CMPA_XTAL_16MHZ_CAPABANK_XTAL_LOAD_CAP_IEC_PF_X100_SHIFT) & 0x3FF);
            pcb_xin_pf_x100 = (int32_t)(
                (xtal_16mhz_capabank_cfg >> FLASH_CMPA_XTAL_16MHZ_CAPABANK_PCB_XIN_PARA_CAP_PF_X100_SHIFT) & 0x3FF);
            pcb_xout_pf_x100 = (int32_t)(
                (xtal_16mhz_capabank_cfg >> FLASH_CMPA_XTAL_16MHZ_CAPABANK_PCB_XOUT_PARA_CAP_PF_X100_SHIFT) & 0x3FF);
        }
        else
        {
            /*
            * Default generic parameters:
            * - 8 pF equivalent IEC capacitance
            * - 2 pF per pin parasitics capacitance on board
            */
            xtal_iec_pf_x100 = 800;
            pcb_xin_pf_x100 = 200;
            pcb_xout_pf_x100 = 200;
        }

        nio_xtal_16mhz_capabank_trim(xtal_iec_pf_x100, pcb_xin_pf_x100, pcb_xout_pf_x100);

        /* Set XO-16MHz Trims */
        nio_set_xtal_16mhz_trim(4, 5);

        /* Power On XO-16MHz */
        PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_XTAL32M_MASK;

        /* Enable and Select FRO32K (instead of XTAL-32K) */
        PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_FRO32K_MASK;
        PMC->RTCOSC32K &= ~PMC_RTCOSC32K_SEL_MASK;
    }
    else
    {
        /* Configuration as done in AO ROM code */

        /* Power on XTAL 16-MHZ, XTAL 16-MHZ LDO & FRO32K */
        PMC->PDRUNCFGCLR0 =
            PMC_PDRUNCFG0_PDEN_XTAL32M_MASK | PMC_PDRUNCFG0_PDEN_LDOXO32M_MASK | PMC_PDRUNCFG0_PDEN_FRO32K_MASK;

        // Select FRO32K
        PMC->RTCOSC32K &= ~PMC_RTCOSC32K_SEL_MASK;

        // Set up the USB HS PHY xtal_clk
        // GM=>[3:1], SLAVE=>[4], AMP=>[7:5], OSC_CAP_IN=>[14:8], OSC_CAP_OUT=>[21:15] values are made zero.
        ANACTRL->XO32M_CTRL &= 0xFFC00000;
        //ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_SLAVE_MASK;
        ANACTRL->XO32M_CTRL |= 0x10U;

        //ANACTRL->LDO_XO32M |= ANACTRL_LDO_XO32M_VOUT(0x7); // 0.8v to 1.21v
        *(volatile uint32_t *)(ANACTRL_BASE + 0xB0) |= 0x38; // 0.8v to 1.21v
    }

    /* Enable XO-16MHz output towards USB-HS */
    ANACTRL->XO32M_CTRL |= ANACTRL_XO32M_CTRL_ENABLE_PLL_USB_OUT_MASK;

    init_usbhs_phy_trim();
    //PMC->LDOUSBHS |= PMC_LDOUSBHS_VADJ(0x7); // 0.8v to 1.21v
    *(volatile uint32_t *)(PMC_BASE + 0x2C) |= 0x7; // 0.8v to 1.21v

    // wait to make sure XO32M is fully up
    debug_printf("Enter 2 --- %s, ", __func__);

    uint64_t lastTicks = microseconds_get_ticks();
    uint8_t xo32mReadyTimeoutInMs = 0;
    nmpa_cfg_info_t *nmpaCfgInfo = (nmpa_cfg_info_t *)0;
    uint32_t xo32mOffset = (uint8_t *)&nmpaCfgInfo->usbCfg.xo32mReadyTimeoutInMs - (uint8_t *)nmpaCfgInfo;
    FFR_GetManufactureData(&g_bootloaderContext.allFlashState[kFlashIndex_Main], &xo32mReadyTimeoutInMs, xo32mOffset,
                           1);
    if (!xo32mReadyTimeoutInMs)
    {
        xo32mReadyTimeoutInMs = 1;
    }
    while (!(ANACTRL->XO32M_STATUS & ANACTRL_XO32M_STATUS_XO_READY_MASK))
    {
        uint64_t ticks = microseconds_get_ticks();
        uint64_t delta = ticks - lastTicks;
        uint32_t timeoutInUs = microseconds_convert_to_microseconds((uint32_t)delta);
        if (timeoutInUs > (xo32mReadyTimeoutInMs * 1000))
        {
            return kStatus_Timeout;
        }
    }

//    usb_peripheral_reset();

    debug_printf("Enter 3 --- %s, ", __func__);
    USBPHY->CTRL &= ~USBPHY_CTRL_SFTRST_MASK;  // release PHY from reset
    USBPHY->CTRL &= ~USBPHY_CTRL_CLKGATE_MASK; // Clear to 0 to run clocks
    USBPHY->CTRL |= USBPHY_CTRL_ENUTMILEVEL2_MASK | USBPHY_CTRL_ENUTMILEVEL3_MASK;

    // Power on the USB-HS PHY
    PMC->PDRUNCFGCLR0 = PMC_PDRUNCFG0_PDEN_USBHSPHY_MASK | PMC_PDRUNCFG0_PDEN_LDOUSBHS_MASK;

    microseconds_delay(100);

    // power up PLL, regulator enable
    //USBPHY->PLL_SIC |= (USBPHY_PLL_SIC_POWER_MASK | USBPHY_PLL_SIC_PLL_REG_ENABLE_MASK);
    USBPHY->PLL_SIC |= (0x1000U | 0x200000U);
    // 16 mhz input clock with default divid by 30
    //USBPHY->PLL_SIC &= ~USBPHY_PLL_SIC_DIV_SELECT_MASK;
    USBPHY->PLL_SIC &= ~0x1C00000U;
    //USBPHY->PLL_SIC |= USBPHY_PLL_SIC_DIV_SELECT(6);
    USBPHY->PLL_SIC |= 0x1800000U;
    // clear bypass bit
    //USBPHY->PLL_SIC &= ~USBPHY_PLL_SIC_BYPASS_MASK;
    USBPHY->PLL_SIC &= ~0x10000U;
    // enable USB clock output from USB PHY PLL, enables auto power down of PHY PLL during suspend
    //USBPHY->PLL_SIC |= (USBPHY_PLL_SIC_EN_USB_CLKS_MASK); // | USBPHY_PLL_SIC_MISC2_CONTROL0_MASK);
    USBPHY->PLL_SIC |= (0x40U); // | USBPHY_PLL_SIC_MISC2_CONTROL0_MASK);

    USBPHY->CTRL &= ~USBPHY_CTRL_CLKGATE_MASK; // Clear to 0 to run clocks
    USBPHY->PWD = 0;                           // for normal operation

    debug_printf("Enter 4b --- %s, ", __func__);
    //  pCfg->regs->PLL_SIC_SET = PLL_MISC2_CONTROL0; // enables auto power down of PHY PLL during suspend
    while (!(USBPHY->PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK_MASK))
    {
    }
    debug_printf("Enter 5 --- %s, ", __func__);
    USBPHY->PWD = 0; // for normal operation
#endif

    debug_printf("Enter 6a --- %s, ", __func__);

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
