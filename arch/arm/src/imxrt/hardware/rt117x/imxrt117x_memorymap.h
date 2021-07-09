/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_memorymap.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT117X_MEMORYMAP_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT117X_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System memory map */

#define IMXRT_ITCM_BASE           0x00000000  /* 512KB ITCM */

                               /* 0x00080000     512KB ITCM Reserved */

                               /* 0x00100000     1MB ITCM Reserved */

                               /* 0x00200000     256KB Reserved */

                               /* 0x00240000     256KB Reserved */

#define IMXRT_CAAM_SECURE_BASE    0x00280000  /* 64KB CAAM_SECURE RAM */

                               /* 0x00290000     1472KB Reserved */

                               /* 0x00400000     124MB Reserved */

                               /* 0x08000000     128MB Reserved */

                               /* 0x10000000     256MB Reserved */

#define IMXRT_DTCM_BASE           0x20000000  /* 512KB DTCM */

                               /* 0x20080000     512KB DTCM Reserved */

                               /* 0x20100000     1MB Reserved */

#define IMXRT_OCRAM_M4_BASE       0x20200000  /* 256KB OCRAM M4 */
#define IMXRT_OCRAM1_BASE         0x20240000  /* 512KB OCRAM1 */
#define IMXRT_OCRAM2_BASE         0x202C0000  /* 512KB OCRAM2 */
#define IMXRT_OCRAM1_ECC_BASE     0x20340000  /* 64KB OCRAM1 ECC */
#define IMXRT_OCRAM2_ECC_BASE     0x20350000  /* 64KB OCRAM2 ECC */
#define IMXRT_OCRAM_M7_ECC_BASE   0x20360000  /* 128KB OCRAM M7 (FlexRAM ECC) */
#define IMXRT_OCRAM_M7_BASE       0x20380000  /* 512KB OCRAM M7 (FlexRAM) */

#define IMXRT_FLEXSPI1TX_BASE     0x2F800000/* 4MB FlexSPI1 TX FIFO */
#define IMXRT_FLEXSPI1RX_BASE     0x2FC00000/* 4MB FlexSPI1 RX FIFO */
#define IMXRT_FLEXSPI1_BASE       0x30000000  /* FlexSPI1 */                   

#define IMXRT_AIPS1_BASE          0x40000000  /* 4MB AIPS-1 */
#define IMXRT_AIPS2_BASE          0x40400000  /* 4MB AIPS-2 */
#define IMXRT_AIPS3_BASE          0x40800000  /* 4MB AIPS-3 */
#define IMXRT_AIPS4_BASE          0x40C00000  /* 4MB AIPS-4 */

#define IMXRT_SIM_DISP_BASE       0x41000000  /* 1MB SIM_DISP configuration port */
#define IMXRT_SIM_M_BASE          0x41100000  /* 1MB SIM_M configuration port */

                               /* 0x41200000     1MB Reserved */
                               /* 0x41300000     1MB Reserved */

#define IMXRT_SIM_M7_BASE         0x41400000  /* 1MB SIM_M7 configuration port */

                               /* 0x41500000     1MB Reserved */
                               /* 0x41600000     1MB Reserved */
                               /* 0x41700000     1MB Reserved */

#define IMXRT_GPU2D_BASE          0x41800000  /* 1MB GPU2D (Peripheral, AHB) */
#define IMXRT_CDOG_BASE           0x41900000  /* 1MB CDOG (Peripheral, AHB) */

                               /* 0x41A00000     6MB Reserved */

#define IMXRT_AIPS_M7_BASE        0x42000000  /* 1MB AIPS M7 (Peripheral, Platform) */

                               /* 0x42100000     7MB Reserved */
                               /* 0x42800000     472MB Reserved */

#define IMXRT_FLEXSPI2_BASE       0x60000000  /* FlexSPI2 */
#define IMXRT_FLEXSPI2TX_BASE     0x7F800000/* 4MB FlexSPI2 TX FIFO */
#define IMXRT_FLEXSPI2RX_BASE     0x7FC00000/* 4MB FlexSPI2 RX FIFO */

#define IMXRT_SEMC0_BASE          0x80000000  /* 256MB SEMC0 */
#define IMXRT_SEMC1_BASE          0x90000000  /* 256MB SEMC1 */
#define IMXRT_SEMC2_BASE          0xA0000000  /* 512MB SEMC2 */
#define IMXRT_SEMC3_BASE          0xC0000000  /* 512MB SEMC3 */


#define IMXRT_CM7_BASE            0xE0000000  /* 1MB CM7 PPB */

                               /* 0xE0100000     511MB Reserved */

/* AIPS-1 memory map */

                               /* 0x40000000     80KB Reserved */

#define IMXRT_MECC1_BASE          0x40014000  /* 16KB MECC1 */
#define IMXRT_MECC2_BASE          0x40018000  /* 16KB MECC1 */
#define IMXRT_XECC_FLEXSPI1_BASE  0x4001C000  /* 16KB XECC_FLEXSPI1 */
#define IMXRT_XECC_FLEXSPI2_BASE  0x40020000  /* 16KB XECC_FLEXSPI2 */
#define IMXRT_XECC_SEMC_BASE      0x40024000  /* 16KB XECC_SEMC */
#define IMXRT_CM7_BASE            0x40028000  /* 16KB CM7 FLEXRAM */
#define IMXRT_EWM_BASE            0x4002C000  /* 16KB EWM */
#define IMXRT_WDOG1_BASE          0x40030000  /* 16KB WDOG1 */
#define IMXRT_WDOG2_BASE          0x40034000  /* 16KB WDOG2 */
#define IMXRT_WDOG3_BASE          0x40038000  /* 16KB WDOG3 */
#define IMXRT_XBAR1_BASE          0x4003C000  /* 16KB XBAR1 */
#define IMXRT_XBAR2_BASE          0x40040000  /* 16KB XBAR2 */
#define IMXRT_XBAR3_BASE          0x40044000  /* 16KB XBAR3 */
#define IMXRT_ADC_ETC_BASE        0x40048000  /* 16KB ADC_ETC */
                               /* 0x4004C000     16KB Reserved */
#define IMXRT_LPADC1_BASE         0x40050000  /* 16KB LPADC1 */
#define IMXRT_LPADC2_BASE         0x40054000  /* 16KB LPADC2 */
                               /* 0x40058000     16KB Reserved */
                               /* 0x4005C000     16KB Reserved */
                               /* 0x40060000     16KB Reserved */
#define IMXRT_DAC_BASE            0x40064000  /* 16KB DAC */
#define IMXRT_IEE_APC_BASE        0x40068000  /* 16KB IEE_APC */
#define IMXRT_IEE_BASE            0x4006C000  /* 16KB IEE */
#define IMXRT_EDMA_BASE           0x40070000  /* 16KB EDMA */
#define IMXRT_DMAMUX0_BASE        0x40074000  /* 16KB DMAMUX0 */
                               /* 0x40078000     16KB Reserved */
#define IMXRT_LPUART1_BASE        0x4007C000  /* 16KB LPUART1 */
#define IMXRT_LPUART2_BASE        0x40080000  /* 16KB LPUART2 */
#define IMXRT_LPUART3_BASE        0x40084000  /* 16KB LPUART3 */
#define IMXRT_LPUART4_BASE        0x40088000  /* 16KB LPUART4 */
#define IMXRT_LPUART5_BASE        0x4008C000  /* 16KB LPUART5 */
#define IMXRT_LPUART6_BASE        0x40090000  /* 16KB LPUART6 */
#define IMXRT_LPUART7_BASE        0x40094000  /* 16KB LPUART7 */
#define IMXRT_LPUART8_BASE        0x40098000  /* 16KB LPUART8 */
#define IMXRT_LPUART9_BASE        0x4009c000  /* 16KB LPUART9 */
#define IMXRT_LPUART10_BASE       0x400A0000  /* 16KB LPUART10 */
                               /* 0x400A4000     16KB Reserved */
                               /* 0x400A8000     16KB Reserved */
#define IMXRT_FLEXIO1_BASE        0x400AC000  /* 16KB FlexIO1 */
#define IMXRT_FLEXIO2_BASE        0x400B0000  /* 16KB FlexIO2 */
                               /* 0x400B4000     16KB Reserved */
#define IMXRT_AOI1_BASE           0x400B8000  /* 16KB AOI1 */
#define IMXRT_AOI2_BASE           0x400BC000  /* 16KB AOI2 */
                               /* 0x400C0000     16KB Reserved */
#define IMXRT_CAN1_BASE           0x400C4000  /* 16KB CAN1 */
#define IMXRT_CAN2_BASE           0x400C8000  /* 16KB CAN2 */
#define IMXRT_FLEXSPI1_BASE       0x400CC000  /* 16KB FLEXSPI1 */
#define IMXRT_FLEXSPI2_BASE       0x400D0000  /* 16KB FLEXSPI2 */
#define IMXRT_SEMC_BASE           0x400D4000  /* 16KB SEMC */
#define IMXRT_PIT1_BASE           0x400D8000  /* 16KB PIT1 */
                               /* 0x400DC000     16KB Reserved */
#define IMXRT_KPP_BASE            0x400E0000  /* 16KB KPP */
#define IMXRT_IOMUXC_GPR_BASE     0x400E4000  /* 16KB IOMUXC_GPR */
#define IMXRT_IOMUXC_BASE         0x400E8000  /* 16KB IOMUXC */
#define IMXRT_GPT1_BASE           0x400EC000  /* 16KB GPT1 */
#define IMXRT_GPT2_BASE           0x400F0000  /* 16KB GPT2 */
#define IMXRT_GPT3_BASE           0x400F4000  /* 16KB GPT3 */
#define IMXRT_GPT4_BASE           0x400F8000  /* 16KB GPT4 */
#define IMXRT_GPT5_BASE           0x400FC000  /* 16KB GPT5 */
#define IMXRT_GPT6_BASE           0x40100000  /* 16KB GPT6 */
#define IMXRT_LPI2C1_BASE         0x40104000  /* 16KB LPI2C1 */
#define IMXRT_LPI2C2_BASE         0x40108000  /* 16KB LPI2C2 */
#define IMXRT_LPI2C3_BASE         0x4010C000  /* 16KB LPI2C3 */
#define IMXRT_LPI2C4_BASE         0x40110000  /* 16KB LPI2C4 */
#define IMXRT_LPSPI1_BASE         0x40114000  /* 16KB LPSPI1 */
#define IMXRT_LPSPI2_BASE         0x40118000  /* 16KB LPSPI2 */
#define IMXRT_LPSPI3_BASE         0x4011C000  /* 16KB LPSPI3 */
#define IMXRT_LPSPI4_BASE         0x40120000  /* 16KB LPSPI4 */
                               /* 0x40124000     16KB Reserved */
                               /* 0x40128000     16KB Reserved */
#define IMXRT_GPIO1_BASE          0x4012C000  /* 16KB GPIO1 */
#define IMXRT_GPIO2_BASE          0x40130000  /* 16KB GPIO2 */
#define IMXRT_GPIO3_BASE          0x40134000  /* 16KB GPIO3 */
#define IMXRT_GPIO4_BASE          0x40138000  /* 16KB GPIO4 */
#define IMXRT_GPIO5_BASE          0x4013C000  /* 16KB GPIO5 */
#define IMXRT_GPIO6_BASE          0x40140000  /* 16KB GPIO6 */
                               /* 0x40144000     16KB Reserved */
                               /* 0x40148000     16KB Reserved */
                               /* 0x4014C000     16KB Reserved */
                               /* 0x40150000     16KB Reserved */
#define IMXRT_EMVSIM1_BASE        0x40154000  /* 16KB EMVSIM1 */
#define IMXRT_EMVSIM2_BASE        0x40158000  /* 16KB EMVSIM2 */
#define IMXRT_TMR1_BASE           0x4015C000  /* 16KB TMR1 */
#define IMXRT_TMR2_BASE           0x40160000  /* 16KB TMR2 */
#define IMXRT_TMR3_BASE           0x40164000  /* 16KB TMR3 */
#define IMXRT_TMR4_BASE           0x40168000  /* 16KB TMR4 */
                               /* 0x4016C000     16KB Reserved */
                               /* 0x40170000     16KB Reserved */
#define IMXRT_QDC1_BASE           0x40174000  /* 16KB QDC1 */
#define IMXRT_QDC2_BASE           0x40178000  /* 16KB QDC2 */
#define IMXRT_QDC3_BASE           0x4017C000  /* 16KB QDC3 */
#define IMXRT_QDC4_BASE           0x40180000  /* 16KB QDC4 */
                               /* 0x40184000     16KB Reserved */
                               /* 0x40188000     16KB Reserved */
#define IMXRT_FLEXPWM1_BASE       0x4018C000  /* 16KB FLEXPWM1 */
#define IMXRT_FLEXPWM2_BASE       0x40190000  /* 16KB FLEXPWM2 */
#define IMXRT_FLEXPWM3_BASE       0x40194000  /* 16KB FLEXPWM3 */
#define IMXRT_FLEXPWM4_BASE       0x40198000  /* 16KB FLEXPWM4 */
                               /* 0x4019C000     16KB Reserved */
                               /* 0x401A0000     16KB Reserved */
#define IMXRT_ACMP1_BASE          0x401A4000  /* 16KB ACMP1 */
#define IMXRT_ACMP2_BASE          0x401A8000  /* 16KB ACMP2 */
#define IMXRT_ACMP3_BASE          0x401AC000  /* 16KB ACMP3 */
#define IMXRT_ACMP4_BASE          0x401B0000  /* 16KB ACMP4 */
                               /* 0x401B4000     16KB Reserved */
                               /* 0x401B8000     16KB Reserved */
                               /* 0x401BC000     16KB Reserved */
                               /* 0x401C0000     16KB Reserved */
                               /* 0x401C4000     16KB Reserved */
                               /* 0x401C8000     16KB Reserved */
                               /* 0x401CC000     16KB Reserved */
                               /* 0x401D0000     16KB Reserved */
                               /* 0x401D4000     16KB Reserved */
                               /* 0x401D8000     16KB Reserved */
                               /* 0x401DC000     16KB Reserved */
                               /* 0x401E0000     16KB Reserved */
                               /* 0x401E4000     16KB Reserved */
                               /* 0x401E8000     16KB Reserved */
                               /* 0x401EC000     16KB Reserved */
                               /* 0x401F0000     16KB Reserved */
                               /* 0x401F4000     16KB Reserved */
                               /* 0x401F8000     16KB Reserved */
                               /* 0x401FC000     16KB Reserved */
                               /* 0x40200000     1MB Reserved */
                               /* 0x40300000     1MB Reserved */

/* AIPS-2 memory map */

#define IMXRT_SPDIF_BASE          0x40400000  /* 16KB SPDIF */
#define IMXRT_SAI1_BASE           0x40404000  /* 16KB SAI1 */
#define IMXRT_SAI2_BASE           0x40408000  /* 16KB SAI2 */
#define IMXRT_SAI3_BASE           0x4040C000  /* 16KB SAI3 */
                               /* 0x40410000     16KB Reserved */
#define IMXRT_ASRC_BASE           0x40414000  /* 16KB ASRC */
#define IMXRT_USDHC1_BASE         0x40418000  /* 16KB USDHC1 */
#define IMXRT_USDHC2_BASE         0x4041C000  /* 16KB USDHC2 */
#define IMXRT_ENET_1G_BASE        0x40420000  /* 16KB ENET_1G */
#define IMXRT_ENET_BASE           0x40424000  /* 16KB ENET */
                               /* 0x40428000     16KB Reserved */
#define IMXRT_USBOTG1_BASE        0x4042C000  /* 16KB USBOTG1 */
#define IMXRT_USBOTG2_BASE        0x40430000  /* 16KB USBOTG2 */
#define IMXRT_USBPHY1_BASE        0x40434000  /* 16KB USBPHY1 */
#define IMXRT_USBPHY2_BASE        0x40438000  /* 16KB USBPHY2 */
#define IMXRT_ENET_QOS_BASE       0x4043C000  /* 16KB ENET_QOS */
#define IMXRT_CAAM1_BASE          0x40440000  /* 16KB CAAM */
#define IMXRT_CAAM2_BASE          0x40444000  /* 16KB CAAM */
#define IMXRT_CAAM3_BASE          0x40448000  /* 16KB CAAM */
#define IMXRT_CAAM4_BASE          0x4044C000  /* 16KB CAAM */
#define IMXRT_CAAM5_BASE          0x40450000  /* 16KB CAAM */
#define IMXRT_CAAM6_BASE          0x40454000  /* 16KB CAAM */
#define IMXRT_CAAM7_BASE          0x40458000  /* 16KB CAAM */
#define IMXRT_CAAM8_BASE          0x4045C000  /* 16KB CAAM */
#define IMXRT_CAAM9_BASE          0x40460000  /* 16KB CAAM */
#define IMXRT_CAAM10_BASE         0x40464000  /* 16KB CAAM */
#define IMXRT_CAAM11_BASE         0x40468000  /* 16KB CAAM */
#define IMXRT_CAAM12_BASE         0x4046C000  /* 16KB CAAM */
#define IMXRT_CAAM13_BASE         0x40470000  /* 16KB CAAM */
#define IMXRT_CAAM14_BASE         0x40474000  /* 16KB CAAM */
#define IMXRT_CAAM15_BASE         0x40478000  /* 16KB CAAM */
#define IMXRT_CAAM16_BASE         0x4047C000  /* 16KB CAAM */
#define IMXRT_CAAM17_BASE         0x40480000  /* 16KB CAAM */
#define IMXRT_CAAM18_BASE         0x40484000  /* 16KB CAAM */
#define IMXRT_CAAM19_BASE         0x40488000  /* 16KB CAAM */
#define IMXRT_CAAM20_BASE         0x4048C000  /* 16KB CAAM */
#define IMXRT_CAAM21_BASE         0x40490000  /* 16KB CAAM */
#define IMXRT_CAAM22_BASE         0x40494000  /* 16KB CAAM */
#define IMXRT_CAAM23_BASE         0x40498000  /* 16KB CAAM */
#define IMXRT_CAAM24_BASE         0x4049C000  /* 16KB CAAM */
#define IMXRT_CAAM25_BASE         0x404A0000  /* 16KB CAAM */
#define IMXRT_CAAM26_BASE         0x404A4000  /* 16KB CAAM */
#define IMXRT_CAAM27_BASE         0x404A8000  /* 16KB CAAM */
#define IMXRT_CAAM28_BASE         0x404AC000  /* 16KB CAAM */
#define IMXRT_CAAM29_BASE         0x404B0000  /* 16KB CAAM */
#define IMXRT_CAAM30_BASE         0x404B4000  /* 16KB CAAM */
#define IMXRT_CAAM31_BASE         0x404B8000  /* 16KB CAAM */
#define IMXRT_CAAM32_BASE         0x404BC000  /* 16KB CAAM */
#define IMXRT_CAAM33_BASE         0x404C0000  /* 16KB CAAM */
#define IMXRT_CAAM34_BASE         0x404C4000  /* 16KB CAAM */
#define IMXRT_CAAM35_BASE         0x404C8000  /* 16KB CAAM */
#define IMXRT_CAAM36_BASE         0x404CC000  /* 16KB CAAM */
#define IMXRT_CAAM37_BASE         0x404D0000  /* 448KB CAAM */
                               /* 0x40540000     768KB Reserved */
                               /* 0x40600000     1MB Reserved */
                               /* 0x40700000     1MB Reserved */

/* AIPS-3 memory map */

#define IMXRT_CSI_BASE            0x40800000  /* 16KB CSI */
#define IMXRT_LCDIF_BASE          0x40804000  /* 16KB LCDIF */
#define IMXRT_LCDIF2_BASE         0x40808000  /* 16KB LCDIFv2 */
#define IMXRT_MIPIDSI_BASE        0x4080C000  /* 16KB MIPI DSI */
#define IMXRT_MIPICSI_BASE        0x40810000  /* 16KB MIPI CSI */
#define IMXRT_PXP_BASE            0x40814000  /* 16KB PXP */
#define IMXRT_VIDEOMUX_BASE       0x40818000  /* 16KB VIDEOMUX */
                               /* 0x4081C000     16KB Reserved */
                               /* 0x40A00000     1MB Reserved */
                               /* 0x40B00000     1MB Reserved */

/* AIPS-4 memory map */

#define IMXRT_GPC_BASE            0x40C00000  /* 16KB GPC */
#define IMXRT_SRC_BASE            0x40C04000  /* 16KB SRC */
#define IMXRT_IOMUXCLPSR_BASE     0x40C08000  /* 16KB IOMUXC LPSR */
#define IMXRT_IOMUXCLPSR_GPR_BASE 0x40C0C000  /* 16KB IOMUXC LPSR GPR */
#define IMXRT_WDOG4_BASE          0x40C10000  /* 16KB WDOG4 */
#define IMXRT_EDMALPSR_BASE       0x40C14000  /* 16KB EDMA LPSR */
#define IMXRT_DMAMUX1_BASE        0x40C18000  /* 16KB DMAMUX1 LPSR */
                               /* 0x40C1C000     16KB Reserved */
#define IMXRT_PDM_BASE            0x40C20000  /* 16KB PDM */
#define IMXRT_LPUART11_BASE       0x40C24000  /* 16KB LPUART11 */
#define IMXRT_LPUART12_BASE       0x40C28000  /* 16KB LPUART12 */
#define IMXRT_LPSPI5_BASE         0x40C2C000  /* 16KB LPSPI5 */
#define IMXRT_LPSPI6_BASE         0x40C30000  /* 16KB LPSPI6 */
#define IMXRT_LPI2C5_BASE         0x40C34000  /* 16KB LPI2C5 */
#define IMXRT_LPI2C6_BASE         0x40C38000  /* 16KB LPI2C6 */
#define IMXRT_CAN3_BASE           0x40C3C000  /* 16KB CAN3 */
#define IMXRT_SAI4_BASE           0x40C40000  /* 16KB SAI4 */
#define IMXRT_RDC_SEM1_BASE       0x40C44000  /* 16KB RDC_SEMAPHORE1 */
#define IMXRT_MUA_BASE            0x40C48000  /* 16KB MU-A */
#define IMXRT_MUB_BASE            0x40C4C000  /* 16KB MU-B */
                               /* 0x40C50000     16KB Reserved */
                               /* 0x40C54000     16KB Reserved */
                               /* 0x40C58000     16KB Reserved */
#define IMXRT_GPIO7_BASE          0x40C5C000  /* 16KB GPIO7 */
#define IMXRT_GPIO8_BASE          0x40C60000  /* 16KB GPIO8 */
#define IMXRT_GPIO9_BASE          0x40C64000  /* 16KB GPIO9 */
#define IMXRT_GPIO10_BASE         0x40C68000  /* 16KB GPIO10 */
#define IMXRT_GPIO11_BASE         0x40C6C000  /* 16KB GPIO11 */
#define IMXRT_GPIO12_BASE         0x40C70000  /* 16KB GPIO12 */
                               /* 0x40C74000     16KB Reserved */
#define IMXRT_RDC_BASE            0x40C78000  /* 16KB RDC */
                               /* 0x40C7C000     16KB Reserved */
#define IMXRT_KEYMGR_BASE         0x40C80000  /* 16KB KEYMGR */
#define IMXRT_ANALOG_BASE         0x40C84000  /* 16KB ANALOG/ANADIG */
#define IMXRT_PGMC_BASE           0x40C88000  /* 16KB PGMC */
                               /* 0x40C8C000     16KB Reserved */
#define IMXRT_SNVS_BASE           0x40C90000  /* 16KB SNVS */
#define IMXRT_IOMUXCSNVS_BASE     0x40C94000  /* 16KB IOMUXC SNVS */
#define IMXRT_IOMUXCSNVS_GPR_BASE 0x40C98000  /* 16KB IOMUXC SNVS GPR */
#define IMXRT_SNVS_SRAM_BASE      0x40C9C000  /* 16KB SNVS SRAM */
#define IMXRT_GPIO13_BASE         0x40CA0000  /* 16KB GPIO13 */
#define IMXRT_ROMCP_BASE          0x40CA4000  /* 16KB ROMCP */
#define IMXRT_DCDC_BASE           0x40CA8000  /* 16KB DCDC */
#define IMXRT_OCOTP_BASE          0x40CAC000  /* 16KB OCOTP */
#define IMXRT_PIT2_BASE           0x40CB0000  /* 16KB PIT2 */
#define IMXRT_SSARCHP_BASE        0x40CB4000  /* 16KB SSARC HP */
#define IMXRT_SSARCLP_BASE        0x40CB8000  /* 16KB SSARC LP */
                               /* 0x40CBC000     16KB Reserved */
#define IMXRT_CCM_BASE            0x40CC0000  /* 32KB CCM */
#define IMXRT_SEMA4_BASE          0x40CC8000  /* 16KB SEMA4 */
#define IMXRT_RDC_SEM2_BASE       0x40CCC000  /* 16KB RDC_SEMAPHORE2 */
#define IMXRT_XRDC2MGR1_M4_BASE   0x40CD0000  /* 16KB XRDC2MGR_M4 */
#define IMXRT_XRDC2MGR2_M4_BASE   0x40CD4000  /* 16KB XRDC2MGR_M4 */
#define IMXRT_XRDC2MGR3_M4_BASE   0x40CD8000  /* 16KB XRDC2MGR_M4 */
#define IMXRT_XRDC2MGR4_M4_BASE   0x40CDC000  /* 16KB XRDC2MGR_M4 */
#define IMXRT_XRDC2MGR1_M7_BASE   0x40CE0000  /* 16KB XRDC2MGR_M7 */
#define IMXRT_XRDC2MGR2_M7_BASE   0x40CE4000  /* 16KB XRDC2MGR_M7 */
#define IMXRT_XRDC2MGR3_M7_BASE   0x40CE8000  /* 16KB XRDC2MGR_M7 */
#define IMXRT_XRDC2MGR4_M7_BASE   0x40CEC000  /* 16KB XRDC2MGR_M7 */
                               /* 0x40CF0000     1088KB Reserved */
                               /* 0x40E00000     1MB Reserved */
                               /* 0x40F00000     1MB Reserved */

/* AIPS-M7 memory map */

                               /* 0x42000000     32KB Reserved */
#define IMXRT_GPIO_M7_2_BASE      0x42008000  /* 16KB GPIO_M7_2 */
#define IMXRT_GPIO_M7_3_BASE      0x4200C000  /* 16KB GPIO_M7_3 */
                               /* 0x42010000     960KB Reserved */

/* PPB M7 memory map */

#define IMXRT_INTERNAL_BASE       0xE0000000  /* 256KB Internal use */
                               /* 0xE0040000     4KB PPB Reserved */
#define IMXRT_ETM_BASE            0xE0041000  /* 4KB ETM */
#define IMXRT_CTI_BASE            0xE0042000  /* 4KB CTI */
#define IMXRT_ATB_BASE            0xE0043000  /* 4KB ATB Funnel */
#define IMXRT_CSSYS_CTI_BASE      0xE0044000  /* 4KB CSSYS CTI */
#define IMXRT_CSSYS_ATB_BASE      0xE0045000  /* 4KB CSSYS ATB Funnel */
#define IMXRT_CSSYS_TPIU_BASE     0xE0046000  /* 4KB CSSYS TPIU */
#define IMXRT_CSSYS_TSGEN_BASE    0xE0047000  /* 4KB CSSYS TSGEN */
#define IMXRT_CSSYS_SWO_BASE      0xE0048000  /* 4KB CSSYS SWO */
                               /* 0xE0049000     220KB PPB Reserved */
#define IMXRT_MCM_BASE            0xE0080000  /* 4KB MCM */
                               /* 0xE0081000     4KB PPB Reserved */

/* PPB M4 memory map */

                               /* 0xE0040000     4KB PPB Reserved */
#define IMXRT_M4_ETM_BASE         0xE0041000  /* 4KB ETM */
#define IMXRT_M4_CTI_BASE         0xE0042000  /* 4KB CTI */
#define IMXRT_M4_ATB_BASE         0xE0043000  /* 4KB ATB Funnel */
#define IMXRT_M4_CSSYS_CTI_BASE   0xE0044000  /* 4KB CSSYS CTI */
#define IMXRT_M4_CSSYS_ATB_BASE   0xE0045000  /* 4KB CSSYS ATB Funnel */
#define IMXRT_M4_CSSYS_TPIU_BASE  0xE0046000  /* 4KB CSSYS TPIU */
#define IMXRT_M4_CSSYS_TSGEN_BASE 0xE0047000  /* 4KB CSSYS TSGEN */
#define IMXRT_M4_CSSYS_SWO_BASE   0xE0048000  /* 4KB CSSYS SWO */
                               /* 0xE0049000     220KB PPB Reserved */
#define IMXRT_M4_MCM_BASE         0xE0080000  /* 4KB MCM */
#define IMXRT_M4_MMCAU_BASE       0xE0081000  /* 4KB MMCAU */
#define IMXRT_M4_AHB_LMEM_BASE    0xE0082000  /* 4KB AHB LMEM */
                               /* 0xE0083000     488KB PPB Reserved */
                               /* 0xE00FD000     4KB PPB Reserved */
                               /* 0xE00FE000     4KB PPB Reserved */
#define IMXRT_M4_PPB_ROM_BASE     0xE00FF000  /* 4KB PPB ROM */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT117X_MEMORYMAP_H */
