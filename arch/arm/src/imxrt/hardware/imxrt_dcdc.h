/****************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_dcdc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_DCDC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_DCDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/
#if defined(CONFIG_ARCH_FAMILY_IMXRT117x)

#define IMXRT_DCDC_CTRL0_OFFSET             0x0000  /* DCDC Control Register 0 */
#define IMXRT_DCDC_CTRL1_OFFSET             0x0004  /* DCDC Control Register 1 */
#define IMXRT_DCDC_REG0_OFFSET              0x0008  /* DCDC Register 0 */
#define IMXRT_DCDC_REG1_OFFSET              0x000C  /* DCDC Register 1 */
#define IMXRT_DCDC_REG2_OFFSET              0x0010  /* DCDC Register 2 */
#define IMXRT_DCDC_REG3_OFFSET              0x0014  /* DCDC Register 3 */
#define IMXRT_DCDC_REG4_OFFSET              0x0018  /* DCDC Register 4 */
#define IMXRT_DCDC_REG5_OFFSET              0x001C  /* DCDC Register 5 */
#define IMXRT_DCDC_REG6_OFFSET              0x0020  /* DCDC Register 6 */
#define IMXRT_DCDC_REG7_OFFSET              0x0024  /* DCDC Register 7 */
#define IMXRT_DCDC_REG7P_OFFSET             0x0028  /* DCDC Register 7 Plus */
#define IMXRT_DCDC_REG8_OFFSET              0x002C  /* DCDC Register 8 */
#define IMXRT_DCDC_REG9_OFFSET              0x0030  /* DCDC Register 9 */
#define IMXRT_DCDC_REG10_OFFSET             0x0034  /* DCDC Register 10 */
#define IMXRT_DCDC_REG11_OFFSET             0x0038  /* DCDC Register 11 */
#define IMXRT_DCDC_REG12_OFFSET             0x003C  /* DCDC Register 12 */
#define IMXRT_DCDC_REG13_OFFSET             0x0040  /* DCDC Register 13 */
#define IMXRT_DCDC_REG14_OFFSET             0x0044  /* DCDC Register 14 */
#define IMXRT_DCDC_REG15_OFFSET             0x0048  /* DCDC Register 15 */
#define IMXRT_DCDC_REG16_OFFSET             0x004C  /* DCDC Register 16 */
#define IMXRT_DCDC_REG17_OFFSET             0x0050  /* DCDC Register 17 */
#define IMXRT_DCDC_REG18_OFFSET             0x0054  /* DCDC Register 18 */
#define IMXRT_DCDC_REG19_OFFSET             0x0058  /* DCDC Register 19 */
#define IMXRT_DCDC_REG20_OFFSET             0x005C  /* DCDC Register 20 */
#define IMXRT_DCDC_REG21_OFFSET             0x0060  /* DCDC Register 21 */
#define IMXRT_DCDC_REG22_OFFSET             0x0064  /* DCDC Register 22 */
#define IMXRT_DCDC_REG23_OFFSET             0x0068  /* DCDC Register 23 */
#define IMXRT_DCDC_REG24_OFFSET             0x006C  /* DCDC Register 24 */

/* Register addresses *******************************************************/

#define IMXRT_DCDC_CTRL0                    (IMXRT_DCDC_BASE + IMXRT_DCDC_CTRL0_OFFSET)
#define IMXRT_DCDC_CTRL1                    (IMXRT_DCDC_BASE + IMXRT_DCDC_CTRL1_OFFSET)
#define IMXRT_DCDC_REG0                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG0_OFFSET)
#define IMXRT_DCDC_REG1                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG1_OFFSET)
#define IMXRT_DCDC_REG2                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG2_OFFSET)
#define IMXRT_DCDC_REG3                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG3_OFFSET)
#define IMXRT_DCDC_REG4                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG4_OFFSET)
#define IMXRT_DCDC_REG5                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG5_OFFSET)
#define IMXRT_DCDC_REG6                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG6_OFFSET)
#define IMXRT_DCDC_REG7                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG7_OFFSET)
#define IMXRT_DCDC_REG7P                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG7P_OFFSET)
#define IMXRT_DCDC_REG8                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG8_OFFSET)
#define IMXRT_DCDC_REG9                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG9_OFFSET)
#define IMXRT_DCDC_REG10                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG10_OFFSET)
#define IMXRT_DCDC_REG11                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG11_OFFSET)
#define IMXRT_DCDC_REG12                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG12_OFFSET)
#define IMXRT_DCDC_REG13                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG13_OFFSET)
#define IMXRT_DCDC_REG14                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG14_OFFSET)
#define IMXRT_DCDC_REG15                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG15_OFFSET)
#define IMXRT_DCDC_REG16                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG16_OFFSET)
#define IMXRT_DCDC_REG17                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG17_OFFSET)
#define IMXRT_DCDC_REG18                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG18_OFFSET)
#define IMXRT_DCDC_REG19                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG19_OFFSET)
#define IMXRT_DCDC_REG20                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG20_OFFSET)
#define IMXRT_DCDC_REG21                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG21_OFFSET)
#define IMXRT_DCDC_REG22                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG22_OFFSET)
#define IMXRT_DCDC_REG23                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG23_OFFSET)
#define IMXRT_DCDC_REG24                    (IMXRT_DCDC_BASE + IMXRT_DCDC_REG24_OFFSET)

#else

#define IMXRT_DCDC_REG0_OFFSET              0x0000  /* DCDC Register 0 */
#define IMXRT_DCDC_REG1_OFFSET              0x0004  /* DCDC Register 1 */
#define IMXRT_DCDC_REG2_OFFSET              0x0008  /* DCDC Register 2 */
#define IMXRT_DCDC_REG3_OFFSET              0x000c  /* DCDC Register 3 */

/* Register addresses *******************************************************/

#define IMXRT_DCDC_REG0                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG0_OFFSET)
#define IMXRT_DCDC_REG1                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG1_OFFSET)
#define IMXRT_DCDC_REG2                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG2_OFFSET)
#define IMXRT_DCDC_REG3                     (IMXRT_DCDC_BASE + IMXRT_DCDC_REG3_OFFSET)

#endif
/* Register bit definitions *************************************************/

#if defined(CONFIG_ARCH_FAMILY_IMXRT117x)

/* Control Register 0 */

#define DCDC_CTRL0_ENABLE                   (1 << 0)  /* Bit 0:  DCDC enable */
#define DCDC_CTRL0_DIG_EN                   (1 << 1)  /* Bit 1:  Enable the DCDC_DIG switching converter output */
#define DCDC_CTRL0_STBY_EN                  (1 << 2)  /* Bit 2:  DCDC Standby mode enable */
#define DCDC_CTRL0_LP_MODE_EN               (1 << 3)  /* Bit 3:  DCDC Low-power mode enable */
#define DCDC_CTRL0_STBY_LP_MODE_EN          (1 << 4)  /* Bit 4:  DCDC Low-power mode enable by GPC standby request */
#define DCDC_CTRL0_ENABLE_DCDC_CNT          (1 << 5)  /* Bit 5:  Enable internal count for DCDC_OK timeout */
#define DCDC_CTRL0_TRIM_HOLD                (1 << 6)  /* Bit 6:  Hold trim input */
#define DCDC_CTRL0_CONTROL_MODE             (1 << 31) /* Bit 31: Control mode */

/* Control Register 1 */

#define DCDC_CTRL1_VDD1P8CTRL_TRG_SHIFT     (0)       /* Bits 0-4: Target value of VDD1P8 in buck mode */
#define DCDC_CTRL1_VDD1P8CTRL_TRG_MASK      (0x1f << DCDC_CTRL1_VDD1P8CTRL_TRG_SHIFT)
#  define DCDC_CTRL1_VDD1P8CTRL_TRG_1V5     ((uint32_t)(0) << DCDC_CTRL1_VDD1P8CTRL_TRG_SHIFT)
#  define DCDC_CTRL1_VDD1P8CTRL_TRG_1V8     ((uint32_t)(0x10) << DCDC_CTRL1_VDD1P8CTRL_TRG_SHIFT)
#  define DCDC_CTRL1_VDD1P8CTRL_TRG_2V275   ((uint32_t)(0x1F) << DCDC_CTRL1_VDD1P8CTRL_TRG_SHIFT)

#define DCDC_CTRL1_VDD1P0CTRL_TRG_SHIFT     (8)       /* Bits 8-12: Target value of VDD1P0 in buck mode */
#define DCDC_CTRL1_VDD1P0CTRL_TRG_MASK      (0x1f << DCDC_CTRL1_VDD1P0CTRL_TRG_SHIFT)
#  define DCDC_CTRL1_VDD1P0CTRL_TRG_0V6     ((uint32_t)(0) << DCDC_CTRL1_VDD1P0CTRL_TRG_SHIFT)
#  define DCDC_CTRL1_VDD1P0CTRL_TRG_1V0     ((uint32_t)(0x0C) << DCDC_CTRL1_VDD1P0CTRL_TRG_SHIFT)
#  define DCDC_CTRL1_VDD1P0CTRL_TRG_1V375   ((uint32_t)(0x1F) << DCDC_CTRL1_VDD1P0CTRL_TRG_SHIFT)

#define DCDC_CTRL1_VDD1P8CTRL_STBY_SHIFT    (16)       /* Bits 16-20: Target value of VDD1P8 in standby mode */
#define DCDC_CTRL1_VDD1P8CTRL_STBY_MASK     (0x1f << DCDC_CTRL1_VDD1P8CTRL_STBY_SHIFT)
#  define DCDC_CTRL1_VDD1P8CTRL_STBY_1V525  ((uint32_t)(0) << DCDC_CTRL1_VDD1P8CTRL_STBY_SHIFT)
#  define DCDC_CTRL1_VDD1P8CTRL_STBY_1V8    ((uint32_t)(0x10) << DCDC_CTRL1_VDD1P8CTRL_STBY_SHIFT)
#  define DCDC_CTRL1_VDD1P8CTRL_STBY_2V3    ((uint32_t)(0x1F) << DCDC_CTRL1_VDD1P8CTRL_STBY_SHIFT)

#define DCDC_CTRL1_VDD1P0CTRL_STBY_SHIFT    (24)       /* Bits 24-28: Target value of VDD1P0 in standby mode */
#define DCDC_CTRL1_VDD1P0CTRL_STBY_MASK     (0x1f << DCDC_CTRL1_VDD1P0CTRL_STBY_SHIFT)
#  define DCDC_CTRL1_VDD1P0CTRL_STBY_0V625  ((uint32_t)(0) << DCDC_CTRL1_VDD1P0CTRL_STBY_SHIFT)
#  define DCDC_CTRL1_VDD1P0CTRL_STBY_1V0    ((uint32_t)(0x0C) << DCDC_CTRL1_VDD1P0CTRL_STBY_SHIFT)
#  define DCDC_CTRL1_VDD1P0CTRL_STBY_1V4    ((uint32_t)(0x1F) << DCDC_CTRL1_VDD1P0CTRL_STBY_SHIFT)

#endif

/* Register 0 */

#define DCDC_REG0_PWD_ZCD                   (1 << 0)  /* Bit 0:  Power down the zero cross detection */
#define DCDC_REG0_DISABLE_AUTO_CLK_SWITCH   (1 << 1)  /* Bit 1:  Disable automatic clock switch */
#define DCDC_REG0_SEL_CLK                   (1 << 2)  /* Bit 2:  Select 24 MHz Crystal clock */
#define DCDC_REG0_PWD_OSC_INT               (1 << 3)  /* Bit 3:  Power down internal osc */
#define DCDC_REG0_PWD_CUR_SNS_CMP           (1 << 4)  /* Bit 4:  The power down signal of the current detector */
#define DCDC_REG0_CUR_SNS_THRSH_SHIFT       (5)       /* Bits 5-7: threshold of current detector */
#define DCDC_REG0_CUR_SNS_THRSH_MASK        (0x7 << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_150MA     ((uint32_t)(0) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_250MA     ((uint32_t)(1) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_350MA     ((uint32_t)(2) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_450MA     ((uint32_t)(3) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_550MA     ((uint32_t)(4) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#  define DCDC_REG0_CUR_SNS_THRSH_650MA     ((uint32_t)(5) << DCDC_REG0_CUR_SNS_THRSH_SHIFT)
#define DCDC_REG0_PWD_OVERCUR_DET           (1 << 8)  /* Bit 8:  Power down overcurrent detection comparator */
#define DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT    (9)       /* Bits 9-10:  The threshold of over current detection */
#define DCDC_REG0_OVERCUR_TIRG_ADJ_MASK     (0x3 << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#  define DCDC_REG0_OVERCUR_TIRG_ADJ_1A_025 ((uint32_t)(0) << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#  define DCDC_REG0_OVERCUR_TIRG_ADJ_2A_025 ((uint32_t)(1) << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#  define DCDC_REG0_OVERCUR_TIRG_ADJ_1A_02  ((uint32_t)(2) << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#  define DCDC_REG0_OVERCUR_TIRG_ADJ_2A_02  ((uint32_t)(3) << DCDC_REG0_OVERCUR_TIRG_ADJ_SHIFT)
#define DCDC_REG0_PWD_CMP_BATT_DET          (1 << 11) /* Bit 11: Power down the low voltage detection comparator */
#define DCDC_REG0_ADJ_POSLIMIT_BUCK_SHIFT   (12)      /* Bits 12-15: Adjust value to poslimit_buck register */
#define DCDC_REG0_ADJ_POSLIMIT_BUCK_MASK    (0xf << DCDC_REG0_ADJ_POSLIMIT_BUCK_SHIFT)
#  define DCDC_REG0_ADJ_POSLIMIT_BUCK(n)    ((uint32_t)(n) << DCDC_REG0_ADJ_POSLIMIT_BUCK_SHIFT)
#define DCDC_REG0_EN_LP_OVERLOAD_SNS        (1 << 16) /* Bit 16: Enable the overload detection in power save mode */
#define DCDC_REG0_PWD_HIGH_VOLT_DET         (1 << 17) /* Bit 17: Power down overvoltage detection comparator */
#define DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT   (18)      /* Bits 18-19: the threshold of the counting number */
#define DCDC_REG0_LP_OVERLOAD_THRSH_MASK    (0x3 << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#  define DCDC_REG0_LP_OVERLOAD_THRSH_32    ((uint32_t)(0) << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#  define DCDC_REG0_LP_OVERLOAD_THRSH_64    ((uint32_t)(1) << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#  define DCDC_REG0_LP_OVERLOAD_THRSH_16    ((uint32_t)(2) << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#  define DCDC_REG0_LP_OVERLOAD_THRSH_8     ((uint32_t)(3) << DCDC_REG0_LP_OVERLOAD_THRSH_SHIFT)
#define DCDC_REG0_LP_OVERLOAD_FREQ_SEL      (1 << 20) /* Bit 20: The period of counting the charging times in power save mode */
#define DCDC_REG0_LP_OVERLOAD_FREQ_SEL_8    (0 << 20) /* Bit 20: The period of counting the charging times in power save mode */
#define DCDC_REG0_LP_OVERLOAD_FREQ_SEL_16   (1 << 20) /* Bit 20: Fhe period of counting the charging times in power save mode */
#define DCDC_REG0_LP_HIGH_HYS               (1 << 21) /* Bit 21: Adjust hysteretic value in low power from 12.5mV to 25mV */
                                                      /* Bits 22-26  Reserved */
#define DCDC_REG0_XTALOK_DISABLE            (1 << 27) /* Bit 27: Disable xtalok detection circuit */
#define DCDC_REG0_CURRENT_ALERT_RESET       (1 << 28) /* Bit 28: Reset current alert signal */
#define DCDC_REG0_XTAL_24M_OK               (1 << 29) /* Bit 29: Set to 1 to switch internal ring osc to xtal 24M */
                                                      /* Bit 30: Reserved */
#define DCDC_REG0_STS_DC_OK                 (1 << 31) /* Bit 31: Status register to indicate DCDC status */

#if defined(CONFIG_ARCH_FAMILY_IMXRT117x)

/* Register 1 */

#define DCDC_REG1_DM_CTRL                   (1 << 3)  /* Bit 3: DM Control */
#define DCDC_REG1_RLOAD_REG_EN_LPSR         (1 << 3)  /* Bit 4: Load resistor enable */
#define DCDC_REG1_VBG_TRIM_SHIFT            (6)       /* Bits 6-10: Trim bandgap voltage */
#define DCDC_REG1_VBG_TRIM_MASK             (0x1f << DCDC_REG1_VBG_TRIM_SHIFT)
#  define DCDC_REG1_VBG_TRIM_0V452          ((uint32_t)(0) << DCDC_REG1_VBG_TRIM_SHIFT)
#  define DCDC_REG1_VBG_TRIM_0V5            ((uint32_t)(0x10) << DCDC_REG1_VBG_TRIM_SHIFT)
#  define DCDC_REG1_VBG_TRIM_0V545          ((uint32_t)(0x1F) << DCDC_REG1_VBG_TRIM_SHIFT)
#define DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT     (11)       /* Bits 11-12: Low power comparator current bias */
#define DCDC_REG1_LP_CMP_ISRC_SEL_MASK      (0x3 << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#  define DCDC_REG1_LP_CMP_ISRC_SEL_50NA    ((uint32_t)(0) << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#  define DCDC_REG1_LP_CMP_ISRC_SEL_100NA   ((uint32_t)(1) << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#  define DCDC_REG1_LP_CMP_ISRC_SEL_200NA   ((uint32_t)(2) << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#  define DCDC_REG1_LP_CMP_ISRC_SEL_400NA   ((uint32_t)(3) << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#define DCDC_REG1_LOOPCTRL_CM_HST_THRESH    (1 << 27)  /* Bit 27: Increase threshold detection */
#define DCDC_REG1_LOOPCTRL_DF_HST_THRESH    (1 << 28)  /* Bit 28: Increase threshold detection */
#define DCDC_REG1_LOOPCTRL_EN_CM_HST        (1 << 29)  /* Bit 29: Enable hysteresis */
#define DCDC_REG1_LOOPCTRL_EN_DM_HST        (1 << 30)  /* Bit 30: Enable hysteresis */

/* Register 2 */


/* Register 3 */

#define DCDC_REG3_IN_BROWNOUT               (1 << 14)  /* Bit 14: Voltage lower than 2.6V */
#define DCDC_REG3_OVERVOLT_VDD1P8_DET_OUT   (1 << 15)  /* Bit 15: Overvoltage on the VDD1P8 output */
#define DCDC_REG3_OVERVOLT_VDD1P0_DET_OUT   (1 << 16)  /* Bit 16: Overvoltage on the VDD1P0 output */
#define DCDC_REG3_OVERCUR_DETECT_OUT        (1 << 17)  /* Bit 17: Overcurrent */
#define DCDC_REG3_ENABLE_FF                 (1 << 18)  /* Bit 18: Enable feed-forward */
#define DCDC_REG3_DISABLE_PULSE_SKIP        (1 << 19)  /* Bit 19: Disable pulse skip */
#define DCDC_REG3_DISABLE_IDLE_SKIP         (1 << 20)  /* Bit 20: Enable idle skip function */
#define DCDC_REG3_DOUBLE_IBIAS_CMP_LP_LPSR  (1 << 21)  /* Bit 21: Double the bias current for the comparator in LP */
#define DCDC_REG3_REG_FBK_SEL_SHIFT         (22)       /* Bits 22-23: Select the feedback point of the internal regulator */
#define DCDC_REG3_REG_FBK_SEL_MASK          (0x3 << DCDC_REG3_REG_FBK_SEL_SHIFT)
#define DCDC_REG3_MINPWR_DC_HALFCLK         (1 << 24)  /* Bit 24: Set DCDC clock to half frequency for continuous mode */
#define DCDC_REG3_MINPWR_HALF_FETS          (1 << 26)  /* Bit 26: Use half switch FET */
#define DCDC_REG3_MISC_DELAY_TIMING         (1 << 27)  /* Bit 27: Adjust delay to reduce ground noise */
#define DCDC_REG3_VDD1P0CTRL_DISABLE_STEP   (1 << 29)  /* Bit 29: Disable step for VDD1P0 */
#define DCDC_REG3_VDD1P8CTRL_DISABLE_STEP   (1 << 30)  /* Bit 30: Disable step for VDD1P8 */

#else

/* Register 1 */

#define DCDC_REG1_POSLIMIT_BUCK_IN_SHIFT    (0)       /* Bits 0-6: Upper limit duty cycle limit in DC-DC converter */
#define DCDC_REG1_POSLIMIT_BUCK_IN_MASK     (0x7f << DCDC_REG1_POSLIMIT_BUCK_IN_SHIFT)
#  define DCDC_REG1_POSLIMIT_BUCK_IN(n)     ((uint32_t)(n) << DCDC_REG1_POSLIMIT_BUCK_IN_SHIFT)
#define DCDC_REG1_REG_FBK_SEL_SHIFT         (7)       /* Bits 7-8: Select the feedback point of the internal regulator */
#define DCDC_REG1_REG_FBK_SEL_MASK          (0x3 << DCDC_REG1_REG_FBK_SEL_SHIFT)
#  define DCDC_REG1_REG_FBK_SEL(n)          ((uint32_t)(n) << DCDC_REG1_REG_FBK_SEL_SHIFT)
                                                      /* Bits 9-11: Reserved */
#define DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT     (12)      /* Bits 12-13: Set the current bias of low power comparator */
#define DCDC_REG1_LP_CMP_ISRC_SEL_MASK      (0x3 << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#  define DCDC_REG1_LP_CMP_ISRC_SEL(n)      ((uint32_t)(n) << DCDC_REG1_LP_CMP_ISRC_SEL_SHIFT)
#define DCDC_REG1_NEGLIMIT_IN_SHIFT         (14)      /* Bits 14-20: Set the current bias of low power comparator */
#define DCDC_REG1_NEGLIMIT_IN_MASK          (0x3f << DCDC_REG1_NEGLIMIT_IN_SHIFT)
#  define DCDC_REG1_NEGLIMIT_IN(n)          ((uint32_t)(n) << DCDC_REG1_NEGLIMIT_IN_SHIFT)
#define DCDC_REG1_LOOPCTRL_HST_THRESH       (1 << 21) /* Bit 21: Increase the threshold detection for common mode analog comparator */
                                                      /* Bit 22: Reserved */
#define DCDC_REG1_LOOPCTRL_EN_HYST          (1 << 23) /* Bit 23: Enable hysteresis in switching converter */
#define DCDC_REG1_VBG_TRIM_SHIFT            (24)      /* Bits 24-28: Trim bandgap voltage */
#define DCDC_REG1_VBG_TRIM_MASK             (0x1f << DCDC_REG1_VBG_TRIM_SHIFT)
#  define DCDC_REG1_VBG_TRIM(n)             ((uint32_t)(n) << DCDC_REG1_VBG_TRIM_SHIFT)
                                                      /* Bit 29-31: Reserved */

/* Register 3 */

#define DCDC_REG3_TRG_SHIFT                 (0)       /* Bits 0-4: Target value of VDD_SOC, 25 mV each step */
#define DCDC_REG3_TRG_MASK                  (0x1f << DCDC_REG3_TRG_SHIFT)
#  define DCDC_REG3_TRG(n)                  ((uint32_t)(n) << DCDC_REG3_TRG_SHIFT)
                                                      /* Bit 5-7: Reserved */
#define DCDC_REG3_TARGET_LP_SHIFT           (8)       /* Bits 8-10: Target value of standby (low power) mode */
#define DCDC_REG3_TARGET_LP_MASK            (0x7 << DCDC_REG3_TARGET_LP_SHIFT)
#  define DCDC_REG3_TARGET_LP_(n)           ((uint32_t)(n) << DCDC_REG3_TARGET_LP_SHIFT)
                                                      /* Bit 11-23: Reserved */
#define DCDC_REG3_MINPWR_DC_HALFCLK         (1 << 24) /* Bit 24: Set DCDC clock to half frequency for continuous mode */
                                                      /* Bit 25-26: Reserved */
#define DCDC_REG3_MISC_DELAY_TIMING         (1 << 27) /* Bit 27: Adjust delay to reduce ground noise */
#define DCDC_REG3_MISC_DISABLE_FET_LOGIC    (1 << 28) /* Bit 28: Datasheet: reserved? */
                                                      /* Bit 29: Reserved */
#define DCDC_REG3_DISABLE_STEP              (1 << 30) /* Bit 30: Disable stepping */
                                                      /* Bit 31: Reserved */
#endif

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_DCDC_H */
