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
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
#include "fusemap.h"
#include "peripherals_pinmux.h"
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifndef FREQ_396MHz
    #define FREQ_396MHz (396000000U)
#endif
#ifndef FREQ_480MHz
    #define FREQ_480MHz (480000000U)
#endif
#ifndef FREQ_528MHz
    #define FREQ_528MHz (528000000U)
#endif
#ifndef FREQ_24MHz
    #define FREQ_24MHz (24000000U)
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
// static bool is_flexspi_2nd_bootpin(void);

/*******************************************************************************
 * Codes
 ******************************************************************************/

// bool is_flexspi_2nd_bootpin(void)
// {
//     bool is_2nd_bootpin_selected = false;
//     if ((ROM_OCOTP_FLASH_TYPE_VALUE() == 0x07) || ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_VALUE())
//     {
//         is_2nd_bootpin_selected = true;
//     }
//     else
//     {
//         is_2nd_bootpin_selected = false;
//     }
//
//     return is_2nd_bootpin_selected;
// }

#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
//!@brief Configure IOMUX for FlexSPI Peripheral
/*
void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue = config->csPadSettingOverride ? config->csPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue = config->dqsPadSettingOverride ? config->dqsPadSettingOverride : FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = config->sclkPadSettingOverride ? config->sclkPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = config->dataPadSettingOverride ? config->dataPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;

    if (instance == 0)
    {
        // The primary FlexSPI pinmux, support octal Flash and up to 4 QuadSPI NOR Flash
        {
            // Pinmux configuration for FLEXSPI1 PortA
            if (config->sflashA1Size || config->sflashA2Size)
            {
                if (config->sflashA2Size)
                {
                    // FLEXSPI1A_SS1_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SS1_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SS1_B_IDX] = csPadCtlValue;
                }

                // Basic pinmux configuration for FLEXSPI1
                if (config->sflashA1Size)
                {
                    // FLEXSPI1A_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SS0_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPI1A_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX] = \
                    FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPI1A_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA3_IDX] = dataPadCtlValue;

                if (config->sflashPadType == kSerialFlash_8Pads)
                {
                    // FLEXSPI1A_DATA4 / FLEXSPI1B_DATA0
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA5 / FLEXSPI1B_DATA1
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA6 / FLEXSPI1B_DATA2
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA7 / FLEXSPI1B_DATA3
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX] = dataPadCtlValue;
                }

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondDqsPinMux))
                    {
                        // FLEXSPI1A_SEC_DQS
                        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX] = \
                            FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX] = dqsPadCtlValue;
                        IOMUXC->SELECT_INPUT[SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX] = kFLEXSPI1A_DQS_SRC_GPIO_SD_14;
                    }
                    else
                    {
                        // FLEXSPI1A_DQS
                        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DQS_IDX] = \
                            FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DQS_IDX] = dqsPadCtlValue;
                        IOMUXC->SELECT_INPUT[SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX] = kFLEXSPI1A_DQS_SRC_GPIO_SD_12;
                    }
                }

                // Configure Differential Clock pin
                if (flexspi_is_differential_clock_enable(config))
                {
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX] = sclkPadCtlValue;
                }
            }

            // Pinmux configuration for FLEXSPI1 PortB
            if (config->sflashB1Size || config->sflashB2Size)
            {
                // Basic pinmux configuration for FLEXSPI1
                if (config->sflashB1Size)
                {
                    // FLEXSPI1B_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPI1B_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX] = \
                    FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPI1B_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX] = dataPadCtlValue;

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    // FLEXSPI1B_DQS
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DQS_IDX] =
                        FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DQS_IDX] = dqsPadCtlValue;
                }
            }
        }
    }
}
*/
#endif // BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE

//! @brief Return uart clock frequency according to instance
uint32_t get_uart_clock(uint32_t instance)
{
    uint32_t periphDivider = ((CCM->CSCDR1 & CCM_CSCDR1_UART_CLK_PODF_MASK) >> CCM_CSCDR1_UART_CLK_PODF_SHIFT) + 1;  
  
    uint32_t lpuart_clock = 80000000UL / periphDivider;

    return lpuart_clock;
}

/*
uint32_t read_autobaud_pin( uint32_t instance )
{
    switch(instance)
    {
        case 0:
            return (GPIO_RD_PDIR(GPIOB) >> UART0_RX_GPIO_PIN_NUM) & 1;
        case 1:
            return (GPIO_RD_PDIR(GPIOC) >> UART1_RX_GPIO_PIN_NUM) & 1;
        case 2:
            return (GPIO_RD_PDIR(GPIOD) >> UART2_RX_GPIO_PIN_NUM) & 1;
        default:
            return 0;
    }
}
*/
bool is_boot_pin_asserted(void)
{
    // Boot pin for Flash only target
    return false;
}

#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
// Set failsafe settings
status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
// This is an example that shows how to override the default pad setting in ROM, for now, the pad setting in ROM is
// idential to below values
// So, below codes are not required.
#if 0
        // See IOMUXC pad setting definitions for more details.
        config->controllerMiscOption |= (1<<kFlexSpiMiscOffset_PadSettingOverrideEnable);
        config->dqsPadSettingOverride = 0x130f1;
        config->sclkPadSettingOverride = 0x10f1;
        config->csPadSettingOverride = 0x10f1;
        config->dataPadSettingOverride = 0x10f1;
#endif
        if (config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad)
        {
            if (config->controllerMiscOption & (1 << kFlexSpiMiscOffset_DdrModeEnable))
            {
                config->dataValidTime[0].time_100ps = 15; // 1.5 ns // 1/4 * cycle of 166MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 30; // 3 ns // 1/2 * cycle of 166MHz DDR
                }
            }
        }
        status = kStatus_Success;

    } while (0);

    return status;
}
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE

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
habstatus_option_t get_hab_status()
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
    // SNVS_GPR0,1,2 is removed in design per system's request
    // SNVS_GPR3 bit[15:0] can be accessed, [31:16] is read only
    uint32_t flag = IOMUXC_SNVS_GPR->GPR3 & 0xFFFFu;
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
    IOMUXC_SNVS_GPR->GPR3 &= 0xFFFF0000u;
    return flag;
}

void isp_cleanup_enter(uint32_t flag)
{
    IOMUXC_SNVS_GPR->GPR3 = (IOMUXC_SNVS_GPR->GPR3 & 0xFFFF0000u) | (uint16_t)flag;
    NVIC_SystemReset();
}



////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
