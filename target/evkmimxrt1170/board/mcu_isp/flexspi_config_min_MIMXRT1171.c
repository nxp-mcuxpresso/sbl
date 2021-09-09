/*
 * Copyright 2018-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "fsl_device_registers.h"
#include "flexspi_nor_flash.h"
#include "microseconds/microseconds.h"
#include "peripherals_pinmux.h"
#include "fusemap.h"
#include "device_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FREQ_396MHz (396000000U)
#define FREQ_480MHz (480000000U)
#define FREQ_528MHz (528000000U)
#define FREQ_24MHz (24000000U)
#define FREQ_1MHz (1000000U)

//!@brief OTFAD related bitmask
#define IOMUXC_GPR34_OTFAD1_EN_MASK (1u << 1)
#define IOMUXC_GPR35_OTFAD2_EN_MASK (1u << 1)

#define FLEXSPI_REMAP_ADDR_UNIT_IN_BYTES (4096ul)

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
    kFlexSpiClkSrc_RC48M_Div2 = 0,
    kFlexSpiClkSrc_OSC = 1,
    kFlexSpiClkSrc_RC400M = 2,
    kFlexSpiClkSrc_RC4M_16M = 3,
    kFlexSpiClkSrc_PLL_480_PFD0 = 4,
    kFlexSpiClkSrc_PLL_528 = 5,
    kFlexSpiClkSrc_PLL_528_PFD2 = 6,
    kFlexSpiClkSrc_PLL_480 = 7,
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
    uint32_t csPadSetting;
    uint32_t sclkPadSetting;
    uint32_t dataPadSetting;
    uint32_t dqsPadSetting;
} flexspi_pad_data_t;

enum
{
    kFlexspiPadDataIndex_FlexSpi1_Primary = 0,
    kFlexspiPadDataIndex_FlexSpi1_Secondary = 1,
    kFlexspiPadDataIndex_FlexSpi2_Primary = 2,
    kFlexspiPadDataIndex_FlexSpi2_Secondary = 3,
    kFlexspiPadDataIndex_Max,
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/


/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Codes
 ******************************************************************************/

uint32_t get_flexspinor_instance(void)
{
    uint32_t instance = 1 + FUSE_FLEXSPI_INSTANCE_VALUE;
    return instance;
}

bool flexspi_is_parallel_mode(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_ParallelEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}
