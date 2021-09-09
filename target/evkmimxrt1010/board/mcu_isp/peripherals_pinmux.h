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

#include "fsl_device_registers.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! peripheral enable configurations
/*====================== LPUART IOMUXC Definitions ===========================*/
#define BL_ENABLE_PINMUX_UART1 (BL_CONFIG_LPUART_1)
#define UART1_RX_IOMUXC_MUX_FUNC    IOMUXC_GPIO_09_LPUART1_RXD
#define UART1_RX_IOMUXC_MUX_GPIO    IOMUXC_GPIO_09_GPIOMUX_IO09
#define UART1_RX_GPIO_BASE       GPIO1
#define UART1_RX_GPIO_PIN_NUM    9 // gpiomux.io[9], sel gpio1/gpio2 via iomuxc_gpr26
#define UART1_RX_GPIO_IRQn       GPIO1_Combined_0_15_IRQn
#define UART1_RX_GPIO_IRQHandler GPIO1_Combined_0_15_IRQHandler

#define UART1_TX_IOMUXC_MUX_FUNC  IOMUXC_GPIO_10_LPUART1_TXD
#define UART1_TX_IOMUXC_PAD_DEFAULT 0x000010B0
#define UART1_TX_GPIO_PIN_NUM   10 // gpiomux.io[10], sel gpio1/gpio2 via iomuxc_gpr26

#define LPUART1_PAD_CTRL                                                                            \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(0) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(2) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1))
#define UART1_PULLUP_PAD_CTRL (IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(2))

// /*====================== LPSPI IOMUXC Definitions ===========================*/
// /* LPSPI1 PINMUX Info  */
// #define SW_MUX_CTL_PAD_LPSPI1_PCS0_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01
// #define SW_MUX_CTL_PAD_LPSPI1_SCK_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00
// #define SW_MUX_CTL_PAD_LPSPI1_SIN_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_03
// #define SW_MUX_CTL_PAD_LPSPI1_SOUT_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_02
//
// #define SW_PAD_CTL_PAD_LPSPI1_PCS0_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_01
// #define SW_PAD_CTL_PAD_LPSPI1_SCK_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_00
// #define SW_PAD_CTL_PAD_LPSPI1_SIN_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_03
// #define SW_PAD_CTL_PAD_LPSPI1_SOUT_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B0_02
//
// #define SELECT_INPUT_LPSPI1_SDI_IDX kIOMUXC_LPSPI1_SDI_SELECT_INPUT
//
// #define LPSPI1_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(4)
// #define GPIO_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5)
// #define LPSPI1_PCS_GPIO GPIO3
// #define LPSPI1_PCS_GPIO_NUM 13
//
// /* LPSPI2 PINMUX Info */
// #define SW_MUX_CTL_PAD_LPSPI2_PCS0_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_06
// #define SW_MUX_CTL_PAD_LPSPI2_SCK_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_07
// #define SW_MUX_CTL_PAD_LPSPI2_SIN_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_09
// #define SW_MUX_CTL_PAD_LPSPI2_SOUT_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_08
//
// #define SW_PAD_CTL_PAD_LPSPI2_PCS0_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_06
// #define SW_PAD_CTL_PAD_LPSPI2_SCK_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_07
// #define SW_PAD_CTL_PAD_LPSPI2_SIN_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_09
// #define SW_PAD_CTL_PAD_LPSPI2_SOUT_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_B1_08
//
// #define SELECT_INPUT_LPSPI2_SDI_IDX kIOMUXC_LPSPI2_SDI_SELECT_INPUT
//
// #define LPSPI2_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(4)
// #define LPSPI2_PCS_GPIO GPIO3
// #define LPSPI2_PCS_GPIO_NUM 6
//
// /* LPSPI3 PINMUX Info */
// #define SW_MUX_CTL_PAD_LPSPI3_PCS0_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_03
// #define SW_MUX_CTL_PAD_LPSPI3_SCK_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_00
// #define SW_MUX_CTL_PAD_LPSPI3_SIN_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02
// #define SW_MUX_CTL_PAD_LPSPI3_SOUT_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_01
//
// #define SW_PAD_CTL_PAD_LPSPI3_PCS0_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_03
// #define SW_PAD_CTL_PAD_LPSPI3_SCK_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_00
// #define SW_PAD_CTL_PAD_LPSPI3_SIN_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_02
// #define SW_PAD_CTL_PAD_LPSPI3_SOUT_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_01
//
// #define SELECT_INPUT_LPSPI3_SDI_IDX kIOMUXC_LPSPI3_SDI_SELECT_INPUT
//
// #define LPSPI3_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(7)
// #define LPSPI3_PCS_GPIO GPIO1
// #define LPSPI3_PCS_GPIO_NUM 3
//
// /* LPSPI4 PINMUX Info */
// #define SW_MUX_CTL_PAD_LPSPI4_PCS0_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00
// #define SW_MUX_CTL_PAD_LPSPI4_SCK_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03
// #define SW_MUX_CTL_PAD_LPSPI4_SIN_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01
// #define SW_MUX_CTL_PAD_LPSPI4_SOUT_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02
//
// #define SW_PAD_CTL_PAD_LPSPI4_PCS0_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00
// #define SW_PAD_CTL_PAD_LPSPI4_SCK_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03
// #define SW_PAD_CTL_PAD_LPSPI4_SIN_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01
// #define SW_PAD_CTL_PAD_LPSPI4_SOUT_IDX kIOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02
//
// #define SELECT_INPUT_LPSPI4_SDI_IDX kIOMUXC_LPSPI4_SDI_SELECT_INPUT
//
// #define LPSPI4_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(3)
// #define LPSPI4_PCS_GPIO GPIO2
// #define LPSPI4_PCS_GPIO_NUM 0
//
// // Fast Slew Rate
// // Driver Strength: 260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR
// // Max Speed : 200MHz
// // Pull enabled
// // Pull
// // 47K Ohm Pull up.
// #define LPSPI_SW_PAD_CTL_VAL                                                                       \
//     IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(0) | \
//         IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1)
// #define GPIO_SW_PAD_CTL_VAL                                                                        \
//     IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(0) | \
//         IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1)
// #define LPSPI1_SDI_SELECT_INPUT_VAL IOMUXC_SELECT_INPUT_DAISY(1) // SELECT_GPIO_SD_B0_03_ALT4
// #define LPSPI2_SDI_SELECT_INPUT_VAL IOMUXC_SELECT_INPUT_DAISY(0) // SELECT_GPIO_SD_B1_09_ALT4
// #define LPSPI3_SDI_SELECT_INPUT_VAL IOMUXC_SELECT_INPUT_DAISY(0) // SELECT_GPIO_AD_B0_02_ALT7
// #define LPSPI4_SDI_SELECT_INPUT_VAL IOMUXC_SELECT_INPUT_DAISY(0) // SELECT_GPIO_SD_B0_01_ALT3

/*====================== FLEXSPI IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPI1B_DQS_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_00
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_04
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_02
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_01
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_03
#define SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_00
#define SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_13

#define SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_14
#define SW_MUX_CTL_PAD_FLEXSPI1A_DQS_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_12
#define SW_MUX_CTL_PAD_FLEXSPI1A_SS0_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_06
#define SW_MUX_CTL_PAD_FLEXSPI1A_SS1_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_05
#define SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_10
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA0_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_09
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA1_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_07
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA2_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_08
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA3_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_11

#define SW_PAD_CTL_PAD_FLEXSPI1B_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_00
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_04
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_02
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_01
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_03
#define SW_PAD_CTL_PAD_FLEXSPI1B_SS0_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_00
#define SW_PAD_CTL_PAD_FLEXSPI1B_SCLK_IDX   kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_13

#define SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_14
#define SW_PAD_CTL_PAD_FLEXSPI1A_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_12
#define SW_PAD_CTL_PAD_FLEXSPI1A_SS0_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_06
#define SW_PAD_CTL_PAD_FLEXSPI1A_SS1_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_05
#define SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX   kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_10
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA0_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_09
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA1_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_07
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA2_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_08
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA3_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_11

#define SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX   kIOMUXC_FLEXSPI_DQS_FA_SELECT_INPUT
#define kFLEXSPI1A_DQS_SRC_GPIO_SD_14 (0u)
#define kFLEXSPI1A_DQS_SRC_GPIO_SD_12 (1u)

#define FLEXSPI1_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0)

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Actual R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Keeper
#define FLEXSPI_SW_PAD_CTL_VAL                                                                      \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(0) | IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Acutal R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Pull
// 100k ohm pull down resistor
#define FLEXSPI_DQS_SW_PAD_CTL_VAL                                                                  \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |   \
     IOMUXC_SW_PAD_CTL_PAD_HYS(1))

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
