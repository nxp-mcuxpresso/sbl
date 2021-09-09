/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "sdmmc_config.h"
#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
#if defined(SDIO_ENABLED) || defined(SD_ENABLED)
void BOARD_SDCardIoVoltageControl(sdmmc_operation_voltage_t voltage);
#endif
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*!brief sdmmc dma buffer */
AT_NONCACHEABLE_SECTION_ALIGN(static uint32_t s_sdmmcHostDmaBuffer[BOARD_SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE],
                              SDMMCHOST_DMA_DESCRIPTOR_BUFFER_ALIGN_SIZE);
#if defined(SDIO_ENABLED) || defined(SD_ENABLED)
static sd_detect_card_t s_cd;
static sd_io_voltage_t s_ioVoltage = {
    .type = BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_TYPE,
    .func = BOARD_SDCardIoVoltageControl,
};
#endif
static sdmmchost_t s_host;

#ifdef SDIO_ENABLED
static sdio_card_int_t s_sdioInt;
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/
uint32_t BOARD_USDHC0ClockConfiguration(void)
{
    /*Make sure USDHC ram buffer has power up*/
    POWER_DisablePD(kPDRUNCFG_APD_USDHC0_SRAM);
    POWER_DisablePD(kPDRUNCFG_PPD_USDHC0_SRAM);
    POWER_DisablePD(kPDRUNCFG_PD_LPOSC);
    POWER_ApplyPD();

    /* SDIO0 */
    /* usdhc depend on 32K clock also */
    CLOCK_AttachClk(kLPOSC_DIV32_to_32KHZWAKE_CLK);
    CLOCK_AttachClk(kAUX0_PLL_to_SDIO0_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivSdio0Clk, 1);

    return CLOCK_GetSdioClkFreq(0);
}
#if defined(SDIO_ENABLED) || defined(SD_ENABLED)
void BOARD_SDCardDetectInit(sd_cd_t cd, void *userData)
{
    /* install card detect callback */
    s_cd.cdDebounce_ms = BOARD_SDMMC_SD_CARD_DETECT_DEBOUNCE_DELAY_MS;
    s_cd.type          = BOARD_SDMMC_SD_CD_TYPE;
    s_cd.callback      = cd;
    s_cd.userData      = userData;
}
#endif
void BOARD_SDCardIoVoltageControlInit(void)
{
    /* Define the init structure for the SD0 power switch pin*/
    gpio_pin_config_t config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* workaround for calling GPIO_PortInit may reset the configuration already done for the port */
    CLOCK_EnableClock(BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_CLOCK_NAME);
    RESET_ClearPeripheralReset(BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_RESET_SOURCE);

    GPIO_PinInit(BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_BASE, BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_PORT,
                 BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_PIN, &config);
}

void BOARD_SDCardIoVoltageControl(sdmmc_operation_voltage_t voltage)
{
    if (voltage == kSDMMC_OperationVoltage330V)
    {
        GPIO_PinWrite(BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_BASE, BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_PORT,
                      BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_PIN, 0U);
    }
    else if (voltage == kSDMMC_OperationVoltage180V)
    {
        GPIO_PinWrite(BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_BASE, BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_PORT,
                      BOARD_SDMMC_SD_IO_VOLTAGE_CONTROL_GPIO_PIN, 1U);
    }
}

void BOARD_SDCardPowerResetInit(void)
{
    /* workaround for calling GPIO_PortInit may reset the configuration already done for the port */
    CLOCK_EnableClock(BOARD_SDMMC_SD_POWER_RESET_GPIO_CLOCK_NAME);
    RESET_ClearPeripheralReset(BOARD_SDMMC_SD_POWER_RESET_GPIO_RESET_SOURCE);

    GPIO_PinInit(BOARD_SDMMC_SD_POWER_RESET_GPIO_BASE, BOARD_SDMMC_SD_POWER_RESET_GPIO_PORT,
                 BOARD_SDMMC_SD_POWER_RESET_GPIO_PIN, &(gpio_pin_config_t){kGPIO_DigitalOutput, 0});
}

void BOARD_SDCardPowerControl(bool enable)
{
    if (enable)
    {
        GPIO_PortSet(BOARD_SDMMC_SD_POWER_RESET_GPIO_BASE, BOARD_SDMMC_SD_POWER_RESET_GPIO_PORT,
                     1U << BOARD_SDMMC_SD_POWER_RESET_GPIO_PIN);
    }
    else
    {
        GPIO_PortClear(BOARD_SDMMC_SD_POWER_RESET_GPIO_BASE, BOARD_SDMMC_SD_POWER_RESET_GPIO_PORT,
                       1U << BOARD_SDMMC_SD_POWER_RESET_GPIO_PIN);
    }
}

#ifdef SD_ENABLED
void BOARD_SD_Config(void *card, sd_cd_t cd, uint32_t hostIRQPriority, void *userData)
{
    assert(card);

    s_host.dmaDesBuffer                                      = s_sdmmcHostDmaBuffer;
    s_host.dmaDesBufferWordsNum                              = BOARD_SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE;
    ((sd_card_t *)card)->host                                = &s_host;
    ((sd_card_t *)card)->host->hostController.base           = BOARD_SDMMC_SD_HOST_BASEADDR;
    ((sd_card_t *)card)->host->hostController.sourceClock_Hz = BOARD_USDHC0ClockConfiguration();

    ((sd_card_t *)card)->host->tuningType = BOARD_SDMMC_SD_TUNING_TYPE;

    ((sd_card_t *)card)->usrParam.cd        = &s_cd;
    ((sd_card_t *)card)->usrParam.pwr       = BOARD_SDCardPowerControl;
    ((sd_card_t *)card)->usrParam.ioVoltage = &s_ioVoltage;

    BOARD_SDCardPowerResetInit();

    BOARD_SDCardIoVoltageControlInit();

    BOARD_SDCardDetectInit(cd, userData);

    NVIC_SetPriority(BOARD_SDMMC_SD_HOST_IRQ, hostIRQPriority);
}
#endif

#ifdef SDIO_ENABLED
void BOARD_SDIO_Config(void *card, sd_cd_t cd, uint32_t hostIRQPriority, sdio_int_t cardInt)
{
    assert(card);

    s_host.dmaDesBuffer                                        = s_sdmmcHostDmaBuffer;
    s_host.dmaDesBufferWordsNum                                = BOARD_SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE;
    ((sdio_card_t *)card)->host                                = &s_host;
    ((sdio_card_t *)card)->host->hostController.base           = BOARD_SDMMC_SDIO_HOST_BASEADDR;
    ((sdio_card_t *)card)->host->hostController.sourceClock_Hz = BOARD_USDHC0ClockConfiguration();
    ((sdio_card_t *)card)->host->tuningType                    = BOARD_SDMMC_SD_TUNING_TYPE;

    ((sdio_card_t *)card)->usrParam.cd        = &s_cd;
    ((sdio_card_t *)card)->usrParam.pwr       = BOARD_SDCardPowerControl;
    ((sdio_card_t *)card)->usrParam.ioVoltage = &s_ioVoltage;
    if (cardInt != NULL)
    {
        s_sdioInt.cardInterrupt                 = cardInt;
        ((sdio_card_t *)card)->usrParam.sdioInt = &s_sdioInt;
    }
    BOARD_SDCardPowerResetInit();

    BOARD_SDCardIoVoltageControlInit();

    BOARD_SDCardDetectInit(cd, NULL);

    NVIC_SetPriority(BOARD_SDMMC_SDIO_HOST_IRQ, hostIRQPriority);
}
#endif

#ifdef MMC_ENABLED
void BOARD_MMC_Config(void *card, uint32_t hostIRQPriority)
{
    assert(card);

    s_host.dmaDesBuffer                                       = s_sdmmcHostDmaBuffer;
    s_host.dmaDesBufferWordsNum                               = BOARD_SDMMC_HOST_DMA_DESCRIPTOR_BUFFER_SIZE;
    ((mmc_card_t *)card)->host                                = &s_host;
    ((mmc_card_t *)card)->host->hostController.base           = BOARD_SDMMC_MMC_HOST_BASEADDR;
    ((mmc_card_t *)card)->host->hostController.sourceClock_Hz = BOARD_USDHC0ClockConfiguration();
    ((mmc_card_t *)card)->host->tuningType                    = BOARD_SDMMC_MMC_TUNING_TYPE;
    ((mmc_card_t *)card)->hostVoltageWindowVCC                = BOARD_SDMMC_MMC_VCC_SUPPLY;
    ((mmc_card_t *)card)->hostVoltageWindowVCCQ               = BOARD_SDMMC_MMC_VCCQ_SUPPLY;

    ((mmc_card_t *)card)->usrParam.capability |= BOARD_SDMMC_MMC_SUPPORT_8_BIT_DATA_WIDTH;

    BOARD_SDCardIoVoltageControlInit();
    BOARD_SDCardIoVoltageControl(kSDMMC_OperationVoltage180V);

    NVIC_SetPriority(BOARD_SDMMC_MMC_HOST_IRQ, hostIRQPriority);
}
#endif
