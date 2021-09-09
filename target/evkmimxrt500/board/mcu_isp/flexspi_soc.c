/*
 * Copyright 2018 - 2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "fsl_device_registers.h"
//#include "fsl_flexspi.h"
#include "flexspi_nor_flash.h"
#include "microseconds/microseconds.h"
#include "peripherals_pinmux.h"
#include "fusemap.h"
#if BL_FEATURE_MASTER_BOOT
#include "masterboot/masterboot.h"
#include "masterboot_config.h"
#endif
//#include "api_tree_root.h"

///*******************************************************************************
// * Definitions
// ******************************************************************************/
#define FREQ_396MHz (396000000U)
#define FREQ_480MHz (480000000U)
#define FREQ_528MHz (528000000U)
#define FREQ_24MHz (24000000U)
#define FREQ_1MHz (1000000U)

//!@brief OTFAD related bitmask
#define IOMUXC_GPR34_OTFAD1_EN_MASK (1u << 1)
#define IOMUXC_GPR35_OTFAD2_EN_MASK (1u << 1)

#define FLEXSPI_REMAP_ADDR_UNIT_IN_BYTES (4096ul)

#define FLEXSPI_BOOT_IMAGE0_INDEX (0u)
#define FLEXSPI_BOOT_IMAGE1_INDEX (1u)
#define FLEXSPI_BOOT_IMAGE_INDEX_MAX (1u)

#define FLEXSPI_DUAL_IMAGE_VERSION_OFFSET (0x600u)

#define IMAGE_ANTI_VERSION_MASK (0xFFFF0000u)
#define IMAGE_VERSION_MASK (0x0000FFFFu)
#define GET_IMAGE_ANTI_VERSION(version) ((version & IMAGE_ANTI_VERSION_MASK) >> 16u)
#define NON_PROGRAMMED_IMAGE_VERSION (0xFFFFFFFFu)
//!@brief Address remap related defintions
typedef struct
{
    __IO uint32_t START;
    __IO uint32_t END;
    __IO uint32_t OFFSET;
} FLEXSPI_ADDR_REMAP_Type;

enum
{
    kImageSize_SecImageOffset = 0, // Image Size = Secondary Image Offset
    kImageSize_1MB = 1,
    kImageSize_2MB = 2,
    kImageSize_3MB = 3,
    kImageSize_4MB = 4,
    kImageSiz_5MB = 5,
    kImageSiz_6MB = 6,
    kImageSiz_7MB = 7,
    kImageSiz_8MB = 8,
    kImageSiz_9MB = 9,
    kImageSiz_10MB = 10,
    kImageSiz_11MB = 11,
    kImageSiz_12MB = 12,
    kImageSize_256KB = 13,
    kImageSize_512KB = 14,
    kImageSize_768KB = 15,
};

enum
{
    kFlashSize_1KB = 1024u,
    kFlashSize_256KB = 256 * kFlashSize_1KB,
    kFlashSize_1MB = (1024 * kFlashSize_1KB),
};

enum
{
    kHoldTime_500US = 0,
    kHoldTime_1MS = 1,
    kHoldTime_3MS = 2,
    kHoldTime_10MS = 3,
};

//!@brief FLEXSPI ROOT CLOCK soruce related definitions
enum
{
    kFlexSpiClkSrc_MAIN_CLK = 0,
    kFlexSpiClkSrc_MAIN_PLL = 1,
    kFlexSpiClkSrc_AUX0_PLL = 2,
    kFlexSpiClkSrc_FRO48_60M = 3,
    kFlexSpiClkSrc_AUX1_PLL = 4,
};

enum
{
    kFlexSpiClkFreqSrc_Normal = 0,
    kFlexSpiClkFreqSrc_RC400M = 1,
    kFlexSpiClkFreqSrc_PLL480M = 2,
};

typedef struct
{
    uint8_t muxPadIndex;       // Index in IOMUXC_MUX_CTL and IOMUXC_PAD_CTL , 0 means this feature is not supported
    uint8_t muxValue;          // MUX value of Pin
    uint8_t selectInputIndex;  // index in IOMUXC_SELECT_INPUT, 0 means this feature is not supported
    uint8_t selecctInputValue; // Selece Input value
} flexspi_pad_attribute_t;

typedef struct
{
    flexspi_pad_attribute_t A_SS0;
    flexspi_pad_attribute_t A_SS1;
    flexspi_pad_attribute_t A_SCLK;
    flexspi_pad_attribute_t A_SCLKB;
    flexspi_pad_attribute_t A_DQS;
    flexspi_pad_attribute_t A_DATA[8];
    flexspi_pad_attribute_t B_SS0;
    flexspi_pad_attribute_t B_SS1;
    flexspi_pad_attribute_t B_SCLK;
    flexspi_pad_attribute_t B_SCLKB;
    flexspi_pad_attribute_t B_DQS;
    flexspi_pad_attribute_t B_DATA[8];
} flexspi_pad_data_t;

enum
{
    kFlexspiPadDataIndex_FlexSpi1_Primary = 0,
    kFlexspiPadDataIndex_FlexSpi1_Secondary = 1,
    kFlexspiPadDataIndex_FlexSpi2_Primary = 2,
    kFlexspiPadDataIndex_FlexSpi2_Secondary = 3,
    kFlexspiPadDataIndex_Max,
};

typedef struct
{
    uint32_t muxPadIndex;
    uint32_t muxValue;
    GPIO_Type *gpio;
    uint32_t pinIndex;
} gpio_pad_data_t;

typedef struct
{
    gpio_pad_data_t SS0;
    gpio_pad_data_t SCLK;
    gpio_pad_data_t SIN;
} flash_jedec_hw_pin_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static bool is_flexspi_2nd_bootpin(void);
bool is_flexspi_clock_enabled(uint32_t instance);
static void otfad_enable(uint32_t instance);
static uint32_t image0_version = 0;
bool g_image0_version_valid_flag = true;

static uint32_t image1_version = 0;
bool g_image1_version_valid_flag = true;

//!@brief Get the FlexSPI PORT
extern uint32_t masterboot_flexspinor_get_port(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Codes
 ******************************************************************************/

void flexspi_sw_delay_us(uint64_t us)
{
    uint32_t ticks_per_us = CLOCK_GetFreq(kCLOCK_CoreSysClk) / FREQ_1MHz;
    while (us--)
    {
        // Measured on RTL testbench, the below loop needs 5 ticks
        register uint32_t ticks = ticks_per_us / 5;
        while (ticks--)
        {
            __NOP();
        }
    }
}

//!@brief Get Clock for FlexSPI peripheral
status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    if ((freq == NULL) || (type > kFlexSpiClock_IpgClock))
    {
        return kStatus_InvalidArgument;
    }

    if ((instance != kFlexspiInstance_0) && (instance != kFlexspiInstance_1))
    {
        return kStatus_InvalidArgument;
    }

    uint32_t seralRootClkDivider;

    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = CLOCK_GetFreq(kCLOCK_CoreSysClk);
            break;
        case kFlexSpiClock_AhbClock:
            clockFrequency = CLOCK_GetFreq(kCLOCK_CoreSysClk);
            break;
        case kFlexSpiClock_SerialRootClock:
        {
            uint32_t srcRootClk;
            uint32_t flexspiClkSrc = 0;
            uint32_t flexspiClkDiv = 0;
            volatile uint32_t *clkDivRegister = NULL;
            if (instance == kFlexspiInstance_0)
            {
                clkDivRegister = &CLKCTL0->FLEXSPIFCLKDIV;
            }
            else if (instance == kFlexspiInstance_1)
            {
            }
            // FLEXPI CLK SEL
            flexspiClkSrc = CLKCTL0->FLEXSPIFCLKSEL;
            // FLEXSPI CLK DIV
            flexspiClkDiv = 1 + CLKCTL0->FLEXSPIFCLKDIV;

            switch (flexspiClkSrc)
            {
                case kFlexSpiClkSrc_MAIN_CLK:
                    srcRootClk = CLOCK_GetMainClkFreq();
                    break;
                case kFlexSpiClkSrc_MAIN_PLL:
                    srcRootClk = CLOCK_GetSysPfdFreq(kCLOCK_Pfd0);
                    break;
                case kFlexSpiClkSrc_AUX0_PLL:
                    srcRootClk = CLOCK_GetSysPfdFreq(kCLOCK_Pfd2);
                    break;
                case kFlexSpiClkSrc_FRO48_60M:
                    srcRootClk = 48000000u;
                    break;
                case kFlexSpiClkSrc_AUX1_PLL:
                    srcRootClk = CLOCK_GetSysPfdFreq(kCLOCK_Pfd3);
                    break;
                default:
                    status = kStatus_OutOfRange;
                    break;
            }

            if (status != kStatus_Success)
            {
                break;
            }

            clockFrequency = srcRootClk / flexspiClkDiv;
            debug_printf("BootROM: FlexSPI Serial Root Clock = %dMHz\n", clockFrequency / FREQ_1MHz);
        }
        break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }

    if (status == kStatus_Success)
    {
        *freq = clockFrequency;
    }

    return status;
}

//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable(uint32_t instance)
{
    if (instance == kFlexspiInstance_0)
    {
        CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_FLEXSPI_OTFAD_CLK_MASK;
    }
    else if (instance == kFlexspiInstance_1)
    {
        uint32_t flexspi1OtfadClkMask = 0x40000;
        CLKCTL0->PSCCTL0_SET = flexspi1OtfadClkMask;
    }
}

//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable(uint32_t instance)
{
    if (instance == kFlexspiInstance_0)
    {
        CLKCTL0->PSCCTL0_CLR = CLKCTL0_PSCCTL0_FLEXSPI_OTFAD_CLK_MASK;
    }
    else if (instance == kFlexspiInstance_1)
    {
        uint32_t flexspi1OtfadClkMask = 0x40000;
        CLKCTL0->PSCCTL0_CLR = flexspi1OtfadClkMask;
    }
}

bool is_flexspi_clock_enabled(uint32_t instance)
{
    if (instance == kFlexspiInstance_0)
    {
        if (CLKCTL0->PSCCTL0 & CLKCTL0_PSCCTL0_FLEXSPI_OTFAD_CLK_MASK)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    if (instance == kFlexspiInstance_1)
    {
        uint32_t flexspi1OtfadClkMask = 0x40000;
        if (CLKCTL0->PSCCTL0 & flexspi1OtfadClkMask)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    return false;
}

void flexspi_update_padsetting(flexspi_mem_config_t *config, uint32_t driveStrength)
{
}

//!@brief Get maximum frequency supported by FlexSPI
status_t flexspi_get_max_supported_freq(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (freq == NULL)
        {
            break;
        }

        *freq = (166UL * 1000 * 1000);
        status = kStatus_Success;

    } while (0);

    return status;
}

//!@brief Set FlexSPI failsafe setting
status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    (void)config;

    return kStatus_Success;
}

void flexspinor_wait_powerup(void)
{
    uint32_t delay_pwr_us = 0;

    debug_printf("BootROM: %s\n", __func__);
    if (RSTCTL0->SYSRSTSTAT & RSTCTL0_SYSRSTSTAT_VDD_POR_MASK)
    {
        const uint32_t k_delayPowerUs[16] = {
            0,      // 0us
            100,    // 100us
            500,    // 500us
            1000,   // 1ms
            10000,  // 10ms
            20000,  // 20ms
            40000,  // 40ms
            60000,  // 60ms
            80000,  // 80ms
            100000, // 100ms
            120000, // 120ms
            140000, // 140ms
            160000, // 160ms
            180000, // 180ms
            200000, // 200ms
            220000, // 220ms
        };
        uint32_t powerHoldTimeIndex = OTP_FLEXSPI_PWR_HOLD_TIME_VALUE();
        if (powerHoldTimeIndex >= (sizeof(k_delayPowerUs) / sizeof(k_delayPowerUs[0])))
        {
            powerHoldTimeIndex = 0;
        }
        delay_pwr_us = k_delayPowerUs[powerHoldTimeIndex];

        debug_printf("BootROM: VDD1V8 POR reset, Flash power up time=%d us\n", delay_pwr_us);
        microseconds_delay(delay_pwr_us);
    }
}

uint32_t get_flexspinor_instance(void)
{
    uint32_t instance = 0;
    return instance;
}

uint32_t get_flexspinand_instance(void)
{
    uint32_t instance = 0;
    return instance;
}

uint32_t flexspinor_get_phy_addr(uint32_t offset)
{
    return (FlexSPI_AMBA_BASE + offset);
}

