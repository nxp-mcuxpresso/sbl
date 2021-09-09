/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v7.0
processor: MIMXRT1176xxxxx
package_id: MIMXRT1176DVMAA
mcu_data: ksdk2_0
processor_version: 0.0.0
pin_labels:
- {pin_num: M13, pin_signal: GPIO_AD_04, label: USER_LED, identifier: USER_LED}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
}

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm7, enableClock: 'true'}
- pin_list:
  - {pin_num: M15, peripheral: LPUART1, signal: RXD, pin_signal: GPIO_AD_25, software_input_on: Disable, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper,
    open_drain: Disable, drive_strength: High, slew_rate: Slow}
  - {pin_num: L13, peripheral: LPUART1, signal: TXD, pin_signal: GPIO_AD_24, software_input_on: Disable, pull_up_down_config: Pull_Down, pull_keeper_select: Keeper,
    open_drain: Disable, drive_strength: High, slew_rate: Slow}
  - {pin_num: F15, peripheral: FLEXSPI1, signal: 'flexspi_a_data, 00', pin_signal: GPIO_SD_B2_08, software_input_on: Enable, pull_down_pull_up_config: Pull_Down,
    pdrv_config: Normal_Driver, open_drain: Disable}
  - {pin_num: H15, peripheral: FLEXSPI1, signal: 'flexspi_a_data, 01', pin_signal: GPIO_SD_B2_09, software_input_on: Enable, pull_down_pull_up_config: Pull_Down,
    pdrv_config: Normal_Driver, open_drain: Disable}
  - {pin_num: H14, peripheral: FLEXSPI1, signal: 'flexspi_a_data, 02', pin_signal: GPIO_SD_B2_10, software_input_on: Enable, pull_down_pull_up_config: Pull_Down,
    pdrv_config: Normal_Driver, open_drain: Disable}
  - {pin_num: F16, peripheral: FLEXSPI1, signal: 'flexspi_a_data, 03', pin_signal: GPIO_SD_B2_11, software_input_on: Enable, pull_down_pull_up_config: Pull_Down,
    pdrv_config: Normal_Driver, open_drain: Disable}
  - {pin_num: E14, peripheral: FLEXSPI1, signal: FLEXSPI_A_DQS, pin_signal: GPIO_SD_B2_05, software_input_on: Enable, pull_down_pull_up_config: Pull_Down, pdrv_config: Normal_Driver,
    open_drain: Disable}
  - {pin_num: G14, peripheral: FLEXSPI1, signal: FLEXSPI_A_SCLK, pin_signal: GPIO_SD_B2_07, software_input_on: Enable, pull_down_pull_up_config: Pull_Down, pdrv_config: Normal_Driver,
    open_drain: Disable}
  - {pin_num: F17, peripheral: FLEXSPI1, signal: flexspi_a_ss_b, pin_signal: GPIO_SD_B2_06, software_input_on: Enable, pull_down_pull_up_config: Pull_Down, pdrv_config: Normal_Driver,
    open_drain: Disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iomuxc);           /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,          /* GPIO_AD_24 is configured as LPUART1_TXD */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,          /* GPIO_AD_25 is configured as LPUART1_RXD */
      0U);                                    /* Software Input On Field: Input Path is determined by functionality */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B2_05_FLEXSPI1_A_DQS,    /* GPIO_SD_B2_05 is configured as FLEXSPI1_A_DQS */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_05 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B2_06_FLEXSPI1_A_SS0_B,  /* GPIO_SD_B2_06 is configured as FLEXSPI1_A_SS0_B */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_06 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B2_07_FLEXSPI1_A_SCLK,   /* GPIO_SD_B2_07 is configured as FLEXSPI1_A_SCLK */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_07 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B2_08_FLEXSPI1_A_DATA00,  /* GPIO_SD_B2_08 is configured as FLEXSPI1_A_DATA00 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_08 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B2_09_FLEXSPI1_A_DATA01,  /* GPIO_SD_B2_09 is configured as FLEXSPI1_A_DATA01 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_09 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B2_10_FLEXSPI1_A_DATA02,  /* GPIO_SD_B2_10 is configured as FLEXSPI1_A_DATA02 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_10 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_SD_B2_11_FLEXSPI1_A_DATA03,  /* GPIO_SD_B2_11 is configured as FLEXSPI1_A_DATA03 */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_SD_B2_11 */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_24_LPUART1_TXD,          /* GPIO_AD_24 PAD functional properties : */
      0x02U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_AD_25_LPUART1_RXD,          /* GPIO_AD_25 PAD functional properties : */
      0x02U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: high driver
                                                 Pull / Keep Select Field: Pull Disable, Highz
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B2_05_FLEXSPI1_A_DQS,    /* GPIO_SD_B2_05 PAD functional properties : */
      0x0AU);                                 /* PDRV Field: normal driver
                                                 Pull Down Pull Up Field: PD
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B2_06_FLEXSPI1_A_SS0_B,  /* GPIO_SD_B2_06 PAD functional properties : */
      0x0AU);                                 /* PDRV Field: normal driver
                                                 Pull Down Pull Up Field: PD
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B2_07_FLEXSPI1_A_SCLK,   /* GPIO_SD_B2_07 PAD functional properties : */
      0x0AU);                                 /* PDRV Field: normal driver
                                                 Pull Down Pull Up Field: PD
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B2_08_FLEXSPI1_A_DATA00,  /* GPIO_SD_B2_08 PAD functional properties : */
      0x0AU);                                 /* PDRV Field: normal driver
                                                 Pull Down Pull Up Field: PD
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B2_09_FLEXSPI1_A_DATA01,  /* GPIO_SD_B2_09 PAD functional properties : */
      0x0AU);                                 /* PDRV Field: normal driver
                                                 Pull Down Pull Up Field: PD
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B2_10_FLEXSPI1_A_DATA02,  /* GPIO_SD_B2_10 PAD functional properties : */
      0x0AU);                                 /* PDRV Field: normal driver
                                                 Pull Down Pull Up Field: PD
                                                 Open Drain Field: Disabled */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_SD_B2_11_FLEXSPI1_A_DATA03,  /* GPIO_SD_B2_11 PAD functional properties : */
      0x0AU);                                 /* PDRV Field: normal driver
                                                 Pull Down Pull Up Field: PD
                                                 Open Drain Field: Disabled */
}

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
