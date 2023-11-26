/****************************************************************************
 * boards/arm/imxrt/edgepro1/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_IMXRT_EDGEPRO1_INCLUDE_BOARD_H
#define __BOARDS_ARM_IMXRT_EDGEPRO1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include i.MXRT header files here. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Set VDD_SOC to 1.25V */

#define IMXRT_VDD_SOC (0x12)

/* Set Arm PLL (PLL1) to  fOut    = (24Mhz * ARM_PLL_DIV_SELECT/2) /
 *                                  ARM_PODF_DIVISOR
 *                        600Mhz  = (24Mhz * ARM_PLL_DIV_SELECT/2) /
 *                                  ARM_PODF_DIVISOR
 *                        ARM_PLL_DIV_SELECT = 100
 *                        ARM_PODF_DIVISOR   = 2
 *                        600Mhz  = (24Mhz * 100/2) / 2
 *
 *     AHB_CLOCK_ROOT             = PLL1fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 600 MHz             = 600Mhz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        600Mhz  = 600Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL1
 *     PERIPH_CLK_SEL             = 1 (0 select PERIPH_CLK2_PODF,
 *                                     1 select PRE_PERIPH_CLK_SEL_PLL1)
 *     PERIPH_CLK                 = 600Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       150Mhz = 600Mhz / 4
 *
 *     PRECLK_CLOCK_ROOT          = IPG_CLOCK_ROOT /
 *                                  IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 9
 *                       16.6Mhz  = 150Mhz / 9
 *
 *     SEMC_CLK_ROOT              = 600Mhz / IMXRT_SEMC_PODF_DIVIDER
 *                                  (labeled AIX_PODF in 18.2)
 *                       IMXRT_SEMC_PODF_DIVIDER = 8
 *                       75Mhz    = 600Mhz / 8
 *
 * Set Sys PLL (PLL2) to  fOut    = (24Mhz * (20+(2*(DIV_SELECT)))
 *                        528Mhz  = (24Mhz * (20+(2*(1)))
 *
 * Set USB1 PLL (PLL3) to fOut    = (24Mhz * 20)
 *                         480Mhz = (24Mhz * 20)
 *
 * Set LPSPI PLL3 PFD0 to fOut    = (480Mhz / 12 * 18)
 *                        720Mhz  = (480Mhz / 12 * 18)
 *                         90Mhz  = (720Mhz / LSPI_PODF_DIVIDER)
 *
 * Set LPI2C PLL3 / 8 to   fOut   = (480Mhz / 8)
 *                         60Mhz  = (480Mhz / 8)
 *                         12Mhz  = (60Mhz / LSPI_PODF_DIVIDER)
 *
 * These clock frequencies can be verified via the CCM_CLKO1 pin and sending
 * the appropriate clock to it with something like;
 *
 *   putreg32( <Clk number> | CCM_CCOSR_CLKO1_EN ,   IMXRT_CCM_CCOSR);
 *   imxrt_config_gpio(GPIO_CCM_CLKO1);
 */

#define BOARD_XTAL_FREQUENCY       24000000
#define IMXRT_PRE_PERIPH_CLK_SEL   CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1
#define IMXRT_PERIPH_CLK_SEL       CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PLL_DIV_SELECT   100
#define IMXRT_ARM_PODF_DIVIDER     2
#define IMXRT_AHB_PODF_DIVIDER     1
#define IMXRT_IPG_PODF_DIVIDER     4
#define IMXRT_PERCLK_CLK_SEL       CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER  9
#define IMXRT_SEMC_PODF_DIVIDER    8

#define IMXRT_LPSPI_CLK_SELECT     CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER    8

#define IMXRT_LPI2C_CLK_SELECT     CCM_CSCDR2_LPI2C_CLK_SEL_PLL3_60M
#define IMXRT_LSI2C_PODF_DIVIDER   5

#define IMXRT_CAN_CLK_SELECT       CCM_CSCMR2_CAN_CLK_SEL_PLL3_SW_80
#define IMXRT_CAN_PODF_DIVIDER     1

#define IMXRT_SYS_PLL_SELECT       CCM_ANALOG_PLL_SYS_DIV_SELECT_22

#define IMXRT_USB1_PLL_DIV_SELECT  CCM_ANALOG_PLL_USB1_DIV_SELECT_20

#define BOARD_CPU_FREQUENCY \
  (BOARD_XTAL_FREQUENCY * (IMXRT_ARM_PLL_DIV_SELECT / 2)) / \
  IMXRT_ARM_PODF_DIVIDER

/* LED definitions **********************************************************/

/* RGB LED
 *
 * R = PWM2 on B0_11 | G = PWM1 on B1_00 | B = PWM1 on B1_01
 */
#define GPIO_FLEXPWM2_MOD3_A  (0) /* DUMMY */
#define GPIO_FLEXPWM2_MOD3_B  (GPIO_FLEXPWM2_PWMB02_2|IOMUX_PWM_DEFAULT) /* R LED */
#define GPIO_FLEXPWM1_MOD4_A  (GPIO_FLEXPWM1_PWMA03_5|IOMUX_PWM_DEFAULT) /* G LED */
#define GPIO_FLEXPWM1_MOD4_B  (GPIO_FLEXPWM1_PWMB03_5|IOMUX_PWM_DEFAULT) /* B LED */

#define RGBLED_PWM1         1
#define RGBLED_PWM2         2
#define RGBLED_RPWMCHANNEL  6
#define RGBLED_GPWMCHANNEL  7
#define RGBLED_BPWMCHANNEL  8

/* PIO Disambiguation *******************************************************/

/* LPUART1
 */
#define GPIO_LPUART1_RX      (GPIO_LPUART1_RX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B0_13 */
#define GPIO_LPUART1_TX      (GPIO_LPUART1_TX_1|IOMUX_UART_DEFAULT) /* GPIO_AD_B0_12 */

/* NINA UART
 */
#define GPIO_LPUART2_RX     (GPIO_LPUART2_RX_1|IOMUX_UART_DEFAULT)
#define GPIO_LPUART2_TX     (GPIO_LPUART2_TX_1|IOMUX_UART_DEFAULT)
#define GPIO_LPUART2_RTS    (GPIO_LPUART2_RTS_1|IOMUX_UART_DEFAULT)
#define GPIO_LPUART2_CTS    (GPIO_LPUART2_CTS_1|IOMUX_UART_DEFAULT)

/* IO UART
 */
#define GPIO_LPUART3_RX     (GPIO_LPUART3_RX_2|IOMUX_UART_DEFAULT)
#define GPIO_LPUART3_TX     (GPIO_LPUART3_TX_2|IOMUX_UART_DEFAULT)
#define GPIO_LPUART3_RTS    (GPIO_LPUART3_RTS_2|IOMUX_UART_DEFAULT)
#define GPIO_LPUART3_CTS    (GPIO_LPUART3_CTS_2|IOMUX_UART_DEFAULT)

/* IO I2C
 */
#define GPIO_LPI2C3_SDA     (GPIO_LPI2C3_SDA_1|IOMUX_LPI2C_DEFAULT)
#define GPIO_LPI2C3_SCL     (GPIO_LPI2C3_SCL_1|IOMUX_LPI2C_DEFAULT)

/* NINA SPI
 */
#define GPIO_LPSPI1_SCK     (GPIO_LPSPI1_SCK_2|IOMUX_LPSPI_DEFAULT)
#define GPIO_LPSPI1_MISO    (GPIO_LPSPI1_SDI_2|IOMUX_LPSPI_DEFAULT)
#define GPIO_LPSPI1_MOSI    (GPIO_LPSPI1_SDO_2|IOMUX_LPSPI_DEFAULT)

/* IO SPI
 */
#define GPIO_LPSPI4_SCK     (GPIO_LPSPI4_SCK_2|IOMUX_LPSPI_DEFAULT)
#define GPIO_LPSPI4_MISO    (GPIO_LPSPI4_SDI_2|IOMUX_LPSPI_DEFAULT)
#define GPIO_LPSPI4_MOSI    (GPIO_LPSPI4_SDO_2|IOMUX_LPSPI_DEFAULT)

/* IO SAI
 */
#define GPIO_SAI2_TX_SYNC   (GPIO_SAI2_TX_SYNC_2|IOMUX_SAI_DEFAULT)
#define GPIO_SAI2_TX_BCLK   (GPIO_SAI2_TX_BCLK_2|IOMUX_SAI_DEFAULT)
#define GPIO_SAI2_RX_BCLK   (GPIO_SAI2_RX_BCLK_2|IOMUX_SAI_DEFAULT)
#define GPIO_SAI2_RX_SYNC   (GPIO_SAI2_RX_SYNC_2|IOMUX_SAI_DEFAULT)
#define GPIO_SAI2_RX_DATA   (GPIO_SAI2_RX_DATA_2|IOMUX_SAI_DEFAULT)
#define GPIO_SAI2_TX_DATA   (GPIO_SAI2_TX_DATA_2|IOMUX_SAI_DEFAULT)
#define GPIO_SAI2_MCLK      (GPIO_SAI2_MCLK_2|IOMUX_SAI_DEFAULT)

/* DATA FLASH
 */
#define GPIO_FLEXSPI_DQS    (GPIO_FLEXSPIA_DQS_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_CS     (GPIO_FLEXSPIA_SS0_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_IO0    (GPIO_FLEXSPIA_DATA00_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_IO1    (GPIO_FLEXSPIA_DATA01_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_IO2    (GPIO_FLEXSPIA_DATA02_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_IO3    (GPIO_FLEXSPIA_DATA03_2|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI_SCK    (GPIO_FLEXSPIA_SCLK_2|IOMUX_FLEXSPI_DEFAULT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_EDGEPRO1_INCLUDE_BOARD_H */
