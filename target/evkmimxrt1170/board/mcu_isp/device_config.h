/*
 * Copyright 2018 - 2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum
{
    kCCM_ClockRootIndex_CM7 = 0,
    kCCM_ClockRootIndex_CM4 = 1,
    kCCM_ClockRootIndex_BUS = 2,
    kCCM_ClockRootIndex_BUSL = 3,
    kCCM_ClockRootIndex_SEMC = 4,
    kCCM_ClockRootIndex_DBG = 5,
    kCCM_ClockRootIndex_M4TICK = 7,
    kCCM_ClockRootIndex_M7TICK = 8,

    kCCM_ClockRootIndex_GPT1 = 14,
    kCCM_ClockRootIndex_GPT2,
    kCCM_ClockRootIndex_GPT3,
    kCCM_ClockRootIndex_GPT4,
    kCCM_ClockRootIndex_GPT5,
    kCCM_ClockRootIndex_GPT6,

    kCCM_ClockRootIndex_FLEXSPI1 = 20,
    kCCM_ClockRootIndex_FLEXSPI2 = 21,

    kCCM_ClockRootIndex_UART1 = 25,
    kCCM_ClockRootIndex_UART2 = 26,
    kCCM_ClockRootIndex_UART3 = 27,
    kCCM_ClockRootIndex_UART4 = 28,
    kCCM_ClockRootIndex_UART5 = 29,
    kCCM_ClockRootIndex_SPI1 = 43,
    kCCM_ClockRootIndex_SPI2,
    kCCM_ClockRootIndex_SPI3,
    kCCM_ClockRootIndex_SPI4,

    kCCM_ClockRootIndex_USDHC1 = 58,
    kCCM_ClockRootIndex_USDHC2 = 59,
    kCCM_ClockRootIndex_SAI1 = 64,
    kCCM_ClockRootIndex_SAI2,
    kCCM_ClockRootIndex_SAI3,
};

enum
{
    kCCM_ClockGroup_Flexram,
    kCCM_ClockGroup_Display
};

enum
{
    kCCM_ClockGateIndex_FLEXSPI1 = 28,
    kCCM_ClockGateIndex_FLEXSPI2 = 29,
    kCCM_ClockGateIndex_SEMC = 33,
    kCCM_ClockGateIndex_XECC = 34,
    kCCM_ClockGateIndex_IEE = 35,
    kCCM_ClockGateIndex_KEYM = 36,
    kCCM_ClockGateIndex_GPIO1 = 51,
    kCCM_ClockGateIndex_PIT1 = 62,
    kCCM_ClockGateIndex_UART1 = 86,
    kCCM_ClockGateIndex_SPI1 = 104,
    kCCM_ClockGateIndex_SPI2,
    kCCM_ClockGateIndex_SPI3,
    kCCM_ClockGateIndex_SPI4,
    kCCM_ClockGateIndex_USB = 115,
    kCCM_ClockGateIndex_SDIOSLV = 116,
    kCCM_ClockGateIndex_USDHC1 = 117,
    kCCM_ClockGateIndex_USDHC2 = 118,
    kCCM_ClockGateIndex_SAI1 = 123,
};

enum
{
    kCCM_OscPllIndex_RC16M = 0,
    kCCM_OscPllIndex_RC48M = 1,
    kCCM_OscPllIndex_RC48M_Div2 = 2,
    kCCM_OscPllIndex_RC400M = 3,
    kCCM_OscPllIndex_OSC24M = 4,
    kCCM_OscPllIndex_OSC24M_Out = 5,
    kCCM_OscPllIndex_PLL_ARM = 6,
    kCCM_OscPllIndex_PLL_ARM_OUT = 7,
    kCCM_OscPllIndex_PLL_528M = 8,
    kCCM_OscPllIndex_PLL_528M_OUT = 9,
    kCCM_OscPllIndex_PLL_528_PFD0 = 10,
    kCCM_OscPllIndex_PLL_528_PFD1 = 11,
    kCCM_OscPllIndex_PLL_528_PFD2 = 12,
    kCCM_OscPllIndex_PLL_528_PFD3 = 13,
    kCCM_OscPllIndex_PLL_480 = 14,
    kCCM_OscPllIndex_PLL_480_OUT = 15,
    kCCM_OscPllIndex_PLL_480_Div2 = 16,
    kCCM_OscPllIndex_PLL_480_PFD0 = 17,
    kCCM_OscPllIndex_PLL_480_PFD1 = 18,
    kCCM_OscPllIndex_PLL_480_PFD2 = 19,
    kCCM_OscPllIndex_PLL_480_PFD3 = 20,
};

#define CCM_CONTROL_DIV_SHIFT (0)
#define CCM_CONTROL_DIV_MASK (0xFF << CCM_CONTROL_DIV_SHIFT)
#define CCM_CONTROL_DIV(x) (((x) << CCM_CONTROL_DIV_SHIFT) & CCM_CONTROL_DIV_MASK)
#define CCM_CONTROL_MUX_SHIFT (8)
#define CCM_CONTROL_MUX_MASK (0x7 << CCM_CONTROL_MUX_SHIFT)
#define CCM_CONTROL_MUX(x) (((x) << CCM_CONTROL_MUX_SHIFT) & CCM_CONTROL_MUX_MASK)

#define CCM_CLOCK_GROUP_CTRL_DIV0_SHIFT (0)
#define CCM_CLOCK_GROUP_CTRL_DIV0_MASK (0xF << CCM_CLOCK_GROUP_CTRL_DIV0_SHIFT)
#define CCM_CLOCK_GROUP_CTRL_DIV0(x) (((x) << CCM_CLOCK_GROUP_CTRL_DIV0_SHIFT) & CCM_CLOCK_GROUP_CTRL_DIV0_MASK)
#define CCM_CLOCK_GROUP_CTRL_DIV1_SHIFT (4)
#define CCM_CLOCK_GROUP_CTRL_DIV1_MASK (0xF << CCM_CLOCK_GROUP_CTRL_DIV1_SHIFT)
#define CCM_CLOCK_GROUP_CTRL_DIV1(x) (((x) << CCM_CLOCK_GROUP_CTRL_DIV1_SHIFT) & CCM_CLOCK_GROUP_CTRL_DIV1_MASK)
#define CCM_CLOCK_GROUP_CTRL_DIV2_SHIFT (8)
#define CCM_CLOCK_GROUP_CTRL_DIV2_MASK (0xF << CCM_CLOCK_GROUP_CTRL_DIV2_SHIFT)
#define CCM_CLOCK_GROUP_CTRL_DIV2(x) (((x) << CCM_CLOCK_GROUP_CTRL_DIV2_SHIFT) & CCM_CLOCK_GROUP_CTRL_DIV2_MASK)
#define CCM_CLOCK_GROUP_CTRL_DIV3_SHIFT (12)
#define CCM_CLOCK_GROUP_CTRL_DIV3_MASK (0xF << CCM_CLOCK_GROUP_CTRL_DIV3_SHIFT)
#define CCM_CLOCK_GROUP_CTRL_DIV3(x) (((x) << CCM_CLOCK_GROUP_CTRL_DIV3_SHIFT) & CCM_CLOCK_GROUP_CTRL_DIV3_MASK)
#define CCM_CLOCK_GROUP_CTRL_RSTDIV_SHIFT (16)
#define CCM_CLOCK_GROUP_CTRL_RSTDIV_MASK (0xFF << CCM_CLOCK_GROUP_CTRL_RSTDIV_SHIFT)
#define CCM_CLOCK_GROUP_CTRL_RSTDIV(x) (((x) << CCM_CLOCK_GROUP_CTRL_RSTDIV_SHIFT) & CCM_CLOCK_GROUP_CTRL_RSTDIV_MASK)

/*!
 * @brief Enumeration for the IOMUXC select input
 *
 * Defines the enumeration for the IOMUXC select input collections.
 */
enum 
{
    kIOMUXC_FLEXSPI1A_DQS_SELECT_INPUT = 46U,   /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1A_DATA0_SELECT_INPUT = 47U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1A_DATA1_SELECT_INPUT = 48U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1A_DATA2_SELECT_INPUT = 49U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1A_DATA3_SELECT_INPUT = 50U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1B_DATA0_SELECT_INPUT = 51U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1B_DATA1_SELECT_INPUT = 52U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1B_DATA2_SELECT_INPUT = 53U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1B_DATA3_SELECT_INPUT = 54U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1A_SCK_SELECT_INPUT = 55U,   /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI1B_SCK_SELECT_INPUT = 56U,   /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI2A_DATA0_SELECT_INPUT = 57U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI2A_DATA1_SELECT_INPUT = 58U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI2A_DATA2_SELECT_INPUT = 59U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI2A_DATA3_SELECT_INPUT = 60U, /**< IOMUXC select input index */
    kIOMUXC_FLEXSPI2A_SCK_SELECT_INPUT = 61U,   /**< IOMUXC select input index */

    kIOMUXC_LPSPI1_PCS0_SELECT_INPUT = 77U, /**< IOMUXC select input index */
    kIOMUXC_LPSPI1_SCK_SELECT_INPUT = 78U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI1_SDI_SELECT_INPUT = 79U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI1_SDO_SELECT_INPUT = 80U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI2_PCS0_SELECT_INPUT = 81U, /**< IOMUXC select input index */
    kIOMUXC_LPSPI2_PCS1_SELECT_INPUT = 82U, /**< IOMUXC select input index */
    kIOMUXC_LPSPI2_SCK_SELECT_INPUT = 83U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI2_SDI_SELECT_INPUT = 84U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI2_SDO_SELECT_INPUT = 85U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI3_PCS0_SELECT_INPUT = 86U, /**< IOMUXC select input index */
    kIOMUXC_LPSPI3_PCS1_SELECT_INPUT = 87U, /**< IOMUXC select input index */
    kIOMUXC_LPSPI3_PCS2_SELECT_INPUT = 88U, /**< IOMUXC select input index */
    kIOMUXC_LPSPI3_PCS3_SELECT_INPUT = 89U, /**< IOMUXC select input index */
    kIOMUXC_LPSPI3_SCK_SELECT_INPUT = 90U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI3_SDI_SELECT_INPUT = 91U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI3_SDO_SELECT_INPUT = 92U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI4_PCS0_SELECT_INPUT = 93U, /**< IOMUXC select input index */
    kIOMUXC_LPSPI4_SCK_SELECT_INPUT = 94U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI4_SDI_SELECT_INPUT = 95U,  /**< IOMUXC select input index */
    kIOMUXC_LPSPI4_SDO_SELECT_INPUT = 96U,  /**< IOMUXC select input index */
    kIOMUXC_LPUART1_RX_SELECT_INPUT = 97U,  /**< IOMUXC select input index */
    kIOMUXC_LPUART1_TX_SELECT_INPUT = 98U,  /**< IOMUXC select input index */

    kIOMUXC_SAI1_MCLK_SELECT_INPUT = 117U,     /**< IOMUXC select input index */
    kIOMUXC_SAI1_RX_BCLK_SELECT_INPUT = 118U,  /**< IOMUXC select input index */
    kIOMUXC_SAI1_RX_DATA0_SELECT_INPUT = 119U, /**< IOMUXC select input index */
    kIOMUXC_SAI1_RX_SYNC_SELECT_INPUT = 120U,  /**< IOMUXC select input index */
    kIOMUXC_SAI1_TX_BCLK_SELECT_INPUT = 121U,  /**< IOMUXC select input index */
    kIOMUXC_SAI1_TX_SYNC_SELECT_INPUT = 122U,  /**< IOMUXC select input index */
    kIOMUXC_SDIOSLV_CLK_CD_SELECT_INPUT = 123U,
    kIOMUXC_SDIOSLV_CMD_DI_SELECT_INPUT = 124U,
    kIOMUXC_SDIOSLV_DAT0_DO_SELECT_INPUT = 125U,
    kIOMUXC_SDIOSLV_DAT1_IRQ_SELECT_INPUT = 126U,
    kIOMUXC_SDIOSLV_DAT2_RW_SELECT_INPUT = 127U,
    kIOMUXC_SDIOSLV_DAT3_CS_SELECT_INPUT = 128U,
    //kIOMUXC_USB_OTG2_OC_SELECT_INPUT = 136U, /**< IOMUXC select input index */
    kIOMUXC_USB_OTG1_OC_SELECT_INPUT = 137U, /**< IOMUXC select input index */
    kIOMUXC_USB_PHY1_ID_SELECT_INPUT = 138U, /**< IOMUXC select input index */
    kIOMUXC_USB_PHY2_ID_SELECT_INPUT = 139U, /**< IOMUXC select input index */
    kIOMUXC_USDHC1_CD_B_SELECT_INPUT = 140U, /**< IOMUXC select input index */
    kIOMUXC_USDHC1_WP_SELECT_INPUT = 141U,   /**< IOMUXC select input index */
    kIOMUXC_USDHC2_CD_B_SELECT_INPUT = 142U, /**< IOMUXC select input index */
    kIOMUXC_USDHC2_WP_SELECT_INPUT = 143U,   /**< IOMUXC select input index */
};


/*!
 * @addtogroup IOMUXC_Register_Masks IOMUXC Register Masks
 * @{
 */

/*! @name SW_MUX_CTL_PAD - SW_MUX_CTL_PAD_GPIO_EMC_00 SW MUX Control Register..SW_MUX_CTL_PAD_GPIO_SD_B1_11 SW MUX
 * Control Register */
/*! @{ */
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_MASK (0xFU)
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_SHIFT (0U)
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_SHIFT)) & IOMUXC_SW_MUX_CTL_PAD_MUX_MODE_MASK)
#define IOMUXC_SW_MUX_CTL_PAD_SION_MASK (0x10U)
#define IOMUXC_SW_MUX_CTL_PAD_SION_SHIFT (4U)
#define IOMUXC_SW_MUX_CTL_PAD_SION(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_MUX_CTL_PAD_SION_SHIFT)) & IOMUXC_SW_MUX_CTL_PAD_SION_MASK)
/*! @} */

/* The count of IOMUXC_SW_MUX_CTL_PAD */
#define IOMUXC_SW_MUX_CTL_PAD_COUNT (145U)

/*! @name SW_PAD_CTL_PAD - SW_PAD_CTL_PAD_GPIO_EMC_00 SW PAD Control Register..SW_PAD_CTL_PAD_GPIO_SD_B1_11 SW PAD
 * Control Register */
/*! @{ */
#define IOMUXC_SW_PAD_CTL_PAD_SD_PDRV_MASK (0x2U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_PDRV_SHIFT (1U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_PDRV(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_SD_PDRV_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_SD_PDRV_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_SD_PULL_MASK (0xCU)
#define IOMUXC_SW_PAD_CTL_PAD_SD_PULL_SHIFT (2U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_PULL(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_SD_PULL_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_SD_PULL_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE_MASK (0x10U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE_SHIFT (4U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_SD_ODE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_SD_ODE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE_LPSR_MASK (0x20U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE_LPSR_SHIFT (5U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE_LPSR(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_SD_ODE_LPSR_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_SD_ODE_LPSR_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE_SNVS_MASK (0x40U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE_SNVS_SHIFT (6U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_ODE_SNVS(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_SD_ODE_SNVS_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_SD_ODE_SNVS_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_SD_APC_MASK (0xF0000000U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_APC_SHIFT (28U)
#define IOMUXC_SW_PAD_CTL_PAD_SD_APC(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_SD_APC_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_SD_APC_MASK)

#define IOMUXC_SW_PAD_CTL_PAD_EMC_PDRV_MASK (0x2U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_PDRV_SHIFT (1U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_PDRV(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_EMC_PDRV_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_EMC_PDRV_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_PULL_MASK (0xCU)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_PULL_SHIFT (2U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_PULL(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_EMC_PULL_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_EMC_PULL_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_MASK (0x10U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_SHIFT (4U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_LPSR_MASK (0x20U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_LPSR_SHIFT (5U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_LPSR(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_LPSR_SHIFT)) & \
     IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_LPSR_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_SNVS_MASK (0x40U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_SNVS_SHIFT (6U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_SNVS(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_SNVS_SHIFT)) & \
     IOMUXC_SW_PAD_CTL_PAD_EMC_ODE_SNVS_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_APC_MASK (0xF0000000U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_APC_SHIFT (28U)
#define IOMUXC_SW_PAD_CTL_PAD_EMC_APC(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_EMC_APC_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_EMC_APC_MASK)

#define IOMUXC_SW_PAD_CTL_PAD_AD_SRE_MASK (0x1U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_SRE_SHIFT (0U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_SRE(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_AD_SRE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_AD_SRE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_AD_DSE_MASK (0x2U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_DSE_SHIFT (1U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_DSE(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_AD_DSE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_AD_DSE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_AD_PUE_MASK (0x4U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_PUE_SHIFT (2U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_PUE(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_AD_PUE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_AD_PUE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_AD_PUS_MASK (0x8U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_PUS_SHIFT (3U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_PUS(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_AD_PUS_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_AD_PUS_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE_MASK (0x10U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE_SHIFT (4U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_AD_ODE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_AD_ODE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE_LPSR_MASK (0x20U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE_LPSR_SHIFT (5U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE_LPSR(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_AD_ODE_LPSR_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_AD_ODE_LPSR_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE_SNVS_MASK (0x40U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE_SNVS_SHIFT (6U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_ODE_SNVS(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_AD_ODE_SNVS_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_AD_ODE_SNVS_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_AD_APC_MASK (0xF0000000U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_APC_SHIFT (28U)
#define IOMUXC_SW_PAD_CTL_PAD_AD_APC(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_AD_APC_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_AD_APC_MASK)

#define IOMUXC_SW_PAD_CTL_PAD_DISP_PDRV_MASK (0x2U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_PDRV_SHIFT (1U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_PDRV(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_DISP_PDRV_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_DISP_PDRV_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_PULL_MASK (0xCU)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_PULL_SHIFT (2U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_PULL(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_DISP_PULL_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_DISP_PULL_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_MASK (0x10U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_SHIFT (4U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_LPSR_MASK (0x20U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_LPSR_SHIFT (5U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_LPSR(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_LPSR_SHIFT)) & \
     IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_LPSR_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_SNVS_MASK (0x40U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_SNVS_SHIFT (6U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_SNVS(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_SNVS_SHIFT)) & \
     IOMUXC_SW_PAD_CTL_PAD_DISP_ODE_SNVS_MASK)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_APC_MASK (0xF0000000U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_APC_SHIFT (28U)
#define IOMUXC_SW_PAD_CTL_PAD_DISP_APC(x) \
    (((uint32_t)(((uint32_t)(x)) << IOMUXC_SW_PAD_CTL_PAD_DISP_APC_SHIFT)) & IOMUXC_SW_PAD_CTL_PAD_DISP_APC_MASK)
/*! @} */


      
