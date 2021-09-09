/*
 * Copyright 2017-2021 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "fsl_lpuart.h"
#include "utilities/fsl_assert.h"
#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
#include "flexspi_nor_flash.h"
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
#include "fusemap.h"
#include "peripherals_pinmux.h"
#include "bl_api.h"
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FREQ_396MHz (396000000U)
#define FREQ_480MHz (480000000U)
#define FREQ_528MHz (528000000U)
#define FREQ_24MHz (24000000U)

typedef struct
{
    uint8_t RESERVED0[0x65C];
    __IO uint32_t SPI_B0_SW_MUX_CTL_PAD[14];
    __IO uint32_t SPI_B1_SW_MUX_CTL_PAD[8];
    __IO uint32_t SPI_B0_SW_PAD_CTL_PAD[14];
    __IO uint32_t SPI_B1_SW_PAD_CTL_PAD[8];
    uint8_t RESERVED1[0x072c - 0x070C];
    __IO uint32_t FLEXSPI2_SELECT_INPUT[11];

} IOMUXC2_Type;

enum __flexspi2_select_input_idx
{
    kFlexSpi2_DQS_Idx,
    kFlexSpi2_FA_DATA0_Idx,
    kFlexSpi2_FA_DATA1_Idx,
    kFlexSpi2_FA_DATA2_Idx,
    kFlexSpi2_FA_DATA3_Idx,
    kFlexSpi2_FB_DATA0_Idx,
    kFlexSpi2_FB_DATA1_Idx,
    kFlexSpi2_FB_DATA2_Idx,
    kFlexSpi2_FB_DATA3_Idx,
    kFlexSpi2_FA_SCLK_Idx,
    kFlexSpi2_FB_SCLK_Idx,
};

#define IOMUXC2 ((volatile IOMUXC2_Type *)(IOMUXC_BASE))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Codes
 ******************************************************************************/

#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue = config->csPadSettingOverride ? config->csPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue =
        config->dqsPadSettingOverride ? config->dqsPadSettingOverride : FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = config->sclkPadSettingOverride ? config->sclkPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = config->dataPadSettingOverride ? config->dataPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;

    if (instance == 0)
    {
        if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondPinMux))
        {
            // The secondary FlexSPI Pinmux, supports only 1 Flash
            if (config->sflashA1Size > 0)
            {
                // FLEXSPIA_SS0_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX] = FLEXSPIA_SEC_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX] = csPadCtlValue;
                // FLEXSPIA_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX] =
                    FLEXSPIA_SEC_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX] = sclkPadCtlValue;
                IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_SCLK_IDX] = 0x01;

                // FLEXSPIA_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX] = FLEXSPIA_SEC_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX] = dataPadCtlValue;
                IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA0_IDX] = 0x01;

                // FLEXSPIA_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX] = FLEXSPIA_SEC_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX] = dataPadCtlValue;
                IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA1_IDX] = 0x01;

                // FLEXSPIA_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX] = FLEXSPIA_SEC_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX] = dataPadCtlValue;
                IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA2_IDX] = 0x01;

                // FLEXSPIA_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX] = FLEXSPIA_SEC_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX] = dataPadCtlValue;
                IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA3_IDX] = 0x01;

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    // FLEXSPIA_DQS
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DQS_IDX] =
                        FLEXSPIA_SEC_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DQS_IDX] = dqsPadCtlValue;
                    IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DQS_IDX] = 0x01;
                }
            }
        }
        else
        {
            // Pinmux configuration for FLEXSPI PortA
            if (config->sflashA1Size || config->sflashA2Size)
            {
                if (config->sflashA2Size)
                {
                    // FLEXSPIA_SS1_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SS1_B_IDX] = FLEXSPIA_SS1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SS1_B_IDX] = csPadCtlValue;
                }

                // Basic pinmux configuration for FLEXSPI
                if (config->sflashA1Size)
                {
                    // FLEXSPIA_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SS0_B_IDX] = FLEXSPIA_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPIA_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SCLK_IDX] =
                    FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPIA_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA0_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA1_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA2_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA3_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA3_IDX] = dataPadCtlValue;

                if ((config->sflashPadType == kSerialFlash_8Pads))
                {
                    // FLEXSPIA_DATA4 / FLEXSPIB_DATA0
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX] = FLEXSPIA_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX] = dataPadCtlValue;

                    // FLEXSPIA_DATA5 / FLEXSPIB_DATA1
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX] = FLEXSPIA_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX] = dataPadCtlValue;

                    // FLEXSPIA_DATA6 / FLEXSPIB_DATA2
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX] = FLEXSPIA_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX] = dataPadCtlValue;

                    // FLEXSPIA_DATA7 / FLEXSPIB_DATA3
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX] = FLEXSPIA_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX] = dataPadCtlValue;
                }

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    // FLEXSPIA_DQS
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DQS_IDX] =
                        FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DQS_IDX] = dqsPadCtlValue;
                }

                // Configure Differential Clock pin
                if (flexspi_is_differential_clock_enable(config))
                {
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SCLK_B_IDX] = FLEXSPIA_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SCLK_B_IDX] = sclkPadCtlValue;
                }
            }

            // Pinmux configuration for FLEXSPI PortB
            if (config->sflashB1Size || config->sflashB2Size)
            {
                if (config->sflashB2Size)
                {
                    // FLEXSPIB_SS1_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SS1_B_IDX] = FLEXSPIB_SS1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SS1_B_IDX] = csPadCtlValue;
                }

                // Basic pinmux configuration for FLEXSPI
                if (config->sflashB1Size)
                {
                    // FLEXSPIB_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SS0_B_IDX] = FLEXSPIB_SS0_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPIB_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SCLK_IDX] =
                    FLEXSPIB_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPIB_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX] = FLEXSPIB_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPIB_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX] = FLEXSPIB_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPIB_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX] = FLEXSPIB_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPIB_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX] = FLEXSPIB_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX] = dataPadCtlValue;

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    // FLEXSPIB_DQS
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DQS_IDX] =
                        FLEXSPIB_DQS_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DQS_IDX] = dqsPadCtlValue;
                }
            }
        }
    }
    else if (instance == 2)
    {
        if (config->sflashA1Size)
        {
            // SS0
            IOMUXC2->SPI_B1_SW_MUX_CTL_PAD[6] = 0;
            IOMUXC2->SPI_B1_SW_PAD_CTL_PAD[6] = csPadCtlValue;

            // SCLK
            IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[8] = 0 | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[8] = sclkPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FA_SCLK_Idx] = 1;

            // DATA0
            IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[2] = 0;
            IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[2] = dataPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FA_DATA0_Idx] = 2;

            // DATA1
            IOMUXC2->SPI_B1_SW_MUX_CTL_PAD[3] = 0;
            IOMUXC2->SPI_B1_SW_PAD_CTL_PAD[3] = dataPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FA_DATA1_Idx] = 0;

            // DATA2
            IOMUXC2->SPI_B1_SW_MUX_CTL_PAD[2] = 0;
            IOMUXC2->SPI_B1_SW_PAD_CTL_PAD[2] = dataPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FA_DATA2_Idx] = 0;

            // DATA3
            IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[10] = 0;
            IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[10] = dataPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FA_DATA3_Idx] = 2;

            // Configure DQS pad
            if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
            {
                IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[9] = 0 | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[9] = dqsPadCtlValue;
                IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_DQS_Idx] = 2;
            }
        }

        if (config->sflashB1Size || (config->sflashA1Size && (config->sflashPadType == kSerialFlash_8Pads)))
        {
            // DATA4/0
            IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[11] = 0;
            IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[11] = dataPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FB_DATA0_Idx] = 1;
            // DATA5/1
            IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[7] = 0;
            IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[7] = dataPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FB_DATA1_Idx] = 1;
            // DATA6/2
            IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[3] = 0;
            IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[3] = dataPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FB_DATA2_Idx] = 1;
            // DATA7/3
            IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[4] = 0;
            IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[4] = dataPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FB_DATA3_Idx] = 1;
        }

        // Configure Differential Clock pin or PORTB is enabled
        if (flexspi_is_differential_clock_enable(config) || config->sflashB1Size)
        {
            IOMUXC2->SPI_B0_SW_MUX_CTL_PAD[1] = 0 | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC2->SPI_B0_SW_PAD_CTL_PAD[1] = sclkPadCtlValue;
            IOMUXC2->FLEXSPI2_SELECT_INPUT[kFlexSpi2_FB_SCLK_Idx] = 0;
        }
    }
}
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE

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

void init_system(void)
{
    SCB_DisableDCache();
    __ISB();
    __DSB();
}

void deinit_system(void)
{
    SCB_EnableDCache();
    __ISB();
    __DSB();
}

void init_hardware(void)
{
    init_system();

    //bl_api_init();

    // Configure clocks.
    configure_clocks(kClockOption_EnterBootloader);

    CLOCK_EnableClock(kCLOCK_UsbOh3);

    // Restore secondary image related settings
    IOMUXC_GPR->GPR30 = 0;
    IOMUXC_GPR->GPR31 = 0;
    IOMUXC_GPR->GPR32 = 0;
}

void deinit_hardware(void)
{
    CLOCK_DisableClock(kCLOCK_UsbOh3);

    deinit_system();
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

void flexspi_update_padsetting(flexspi_mem_config_t *config, uint32_t driveStrength)
{
#define IOMUXC_PAD_SETTING_DSE_SHIFT (3)
#define IOMUXC_PAD_SETTING_DSE_MASK (0x07 << IOMUXC_PAD_SETTING_DSE_SHIFT)
#define IOMUXC_PAD_SETTING_DSE(x) (((x) << IOMUXC_PAD_SETTING_DSE_SHIFT) & IOMUXC_PAD_SETTING_DSE_MASK)
    if (driveStrength)
    {
        config->dqsPadSettingOverride =
            (FLEXSPI_DQS_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
        config->sclkPadSettingOverride =
            (FLEXSPI_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
        config->dataPadSettingOverride =
            (FLEXSPI_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);

        config->csPadSettingOverride =
            (FLEXSPI_DQS_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
    }
}

uint32_t get_flexspinor_instance(void)
{
    return (ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_VALUE() ? 2 : 0);
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

bool flexspi_is_differential_clock_enable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_DiffClkEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

uint32_t get_flexspinor_amba_base(void)
{
    if (get_flexspinor_instance())
    {
        return FlexSPI2_AMBA_BASE;
    }
    else
    {
        return FlexSPI_AMBA_BASE;
    }
}

void normal_mem_init(void)
{
    typedef struct
    {
        uint32_t dtcmSizeKB;
        uint32_t itcmSizeKB;
        uint32_t ocramSizeKB;
    } flexram_cfg_t;

    const flexram_cfg_t k_flexramCfgList[] = {
        { 128, 128, 256 }, { 128, 64, 320 }, { 128, 256, 128 }, { 128, 32, 352 }, { 64, 128, 320 },
        { 64, 256, 192 },  { 0, 442, 64 },   { 256, 128, 128 }, { 256, 64, 192 }, { 192, 256, 64 },
        { 448, 0, 64 },    { 0, 128, 384 },  { 32, 32, 448 },   { 0, 256, 256 },  { 0, 0, 512 },
    };

    uint32_t fixedOcramSize = 512u * 1024ul;

    uint32_t ramCfgIndex = ROM_OCOTP_FLEXRAM_CFG_VALUE();

    uint32_t itcmSize = k_flexramCfgList[ramCfgIndex].itcmSizeKB * 1024u;
    uint32_t dtcmSize = k_flexramCfgList[ramCfgIndex].dtcmSizeKB * 1024u;
    uint32_t ocramSize = k_flexramCfgList[ramCfgIndex].ocramSizeKB * 1024u + fixedOcramSize;

    if (itcmSize < 1)
    {
        itcmSize = 1;
    }
    if (dtcmSize < 1)
    {
        dtcmSize = 1;
    }

    g_memoryMap[0].startAddress = 0;
    g_memoryMap[0].endAddress = g_memoryMap[0].startAddress + itcmSize - 1;
    g_memoryMap[1].startAddress = 0x20000000;
    g_memoryMap[1].endAddress = g_memoryMap[1].startAddress + dtcmSize - 1;
    g_memoryMap[2].startAddress = 0x20200000;
    g_memoryMap[2].endAddress = g_memoryMap[2].startAddress + ocramSize - 1;
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
