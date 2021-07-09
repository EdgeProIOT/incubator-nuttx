/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_pmu.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT117X_PMU_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT117X_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMXRT_PMU_LDO_LPSR_ANA_OFFSET     0x510
#define IMXRT_PMU_LDO_LPSR_DIG_2_OFFSET   0x520
#define IMXRT_PMU_LDO_LPSR_DIG_OFFSET     0x530
#define IMXRT_PMU_CTRL_OFFSET             0x550  /* PMU CTRL register */
#define IMXRT_PMU_CTRL2_OFFSET            0x560  /* PMU CTRL2 register */

#define IMXRT_PMU_LDO_LPSR_ANA            (IMXRT_ANALOG_BASE + IMXRT_PMU_LDO_LPSR_ANA_OFFSET)
#define IMXRT_PMU_LDO_LPSR_DIG_2          (IMXRT_ANALOG_BASE + IMXRT_PMU_LDO_LPSR_DIG_2_OFFSET)
#define IMXRT_PMU_LDO_LPSR_DIG            (IMXRT_ANALOG_BASE + IMXRT_PMU_LDO_LPSR_DIG_OFFSET)
#define IMXRT_PMU_CTRL                    (IMXRT_ANALOG_BASE + IMXRT_PMU_CTRL_OFFSET)
#define IMXRT_PMU_CTRL2                   (IMXRT_ANALOG_BASE + IMXRT_PMU_CTRL2_OFFSET)


#define PMU_LDO_LPSR_ANA_REG_LP_EN_MASK   (0x1)
#define PMU_LDO_LPSR_ANA_REG_LP_EN_SHIFT  (0)
#define PMU_LDO_LPSR_ANA_REG_LP_EN(x)                                               \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_REG_LP_EN_SHIFT)) & \
    PMU_LDO_LPSR_ANA_REG_LP_EN_MASK)

#define PMU_LDO_LPSR_ANA_REG_DISABLE_MASK  (0x4)
#define PMU_LDO_LPSR_ANA_REG_DISABLE_SHIFT (2)
#define PMU_LDO_LPSR_ANA_REG_DISABLE(x)                                             \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_REG_DISABLE_SHIFT)) & \
    PMU_LDO_LPSR_ANA_REG_DISABLE_MASK)

#define PMU_LDO_LPSR_ANA_PULL_DOWN_2MA_EN_MASK  (0x8)
#define PMU_LDO_LPSR_ANA_PULL_DOWN_2MA_EN_SHIFT (3)
#define PMU_LDO_LPSR_ANA_PULL_DOWN_2MA_EN(x)                                        \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_PULL_DOWN_2MA_EN_SHIFT)) & \
    PMU_LDO_LPSR_ANA_PULL_DOWN_2MA_EN_MASK)

#define PMU_LDO_LPSR_ANA_LPSR_ANA_CONTROL_MODE_MASK   (0x10)
#define PMU_LDO_LPSR_ANA_LPSR_ANA_CONTROL_MODE_SHIFT  (4)
#define PMU_LDO_LPSR_ANA_LPSR_ANA_CONTROL_MODE(x)                                   \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_LPSR_ANA_CONTROL_MODE_SHIFT)) & \
    PMU_LDO_LPSR_ANA_LPSR_ANA_CONTROL_MODE_MASK)

#define PMU_LDO_LPSR_ANA_BYPASS_MODE_EN_MASK  (0x20)
#define PMU_LDO_LPSR_ANA_BYPASS_MODE_EN_SHIFT (5)
#define PMU_LDO_LPSR_ANA_BYPASS_MODE_EN(x)                                          \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_BYPASS_MODE_EN_SHIFT)) & \
    PMU_LDO_LPSR_ANA_BYPASS_MODE_EN_MASK)

#define PMU_LDO_LPSR_ANA_STANDBY_EN_MASK  (0x40)
#define PMU_LDO_LPSR_ANA_STANDBY_EN_SHIFT (6)
#define PMU_LDO_LPSR_ANA_STANDBY_EN(x)                                              \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_STANDBY_EN_SHIFT)) & \
    PMU_LDO_LPSR_ANA_STANDBY_EN_MASK)

#define PMU_LDO_LPSR_ANA_ALWAYS_4MA_PULLDOWN_EN_MASK  (0x100)
#define PMU_LDO_LPSR_ANA_ALWAYS_4MA_PULLDOWN_EN_SHIFT (8)
#define PMU_LDO_LPSR_ANA_ALWAYS_4MA_PULLDOWN_EN(x)                                  \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_ALWAYS_4MA_PULLDOWN_EN_SHIFT)) & \
    PMU_LDO_LPSR_ANA_ALWAYS_4MA_PULLDOWN_EN_MASK)

#define PMU_LDO_LPSR_ANA_TRACK_MODE_EN_MASK   (0x80000)
#define PMU_LDO_LPSR_ANA_TRACK_MODE_EN_SHIFT  (19)
#define PMU_LDO_LPSR_ANA_TRACK_MODE_EN(x)                                           \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_TRACK_MODE_EN_SHIFT)) & \
    PMU_LDO_LPSR_ANA_TRACK_MODE_EN_MASK)

#define PMU_LDO_LPSR_ANA_PULL_DOWN_20UA_EN_MASK   (0x100000)
#define PMU_LDO_LPSR_ANA_PULL_DOWN_20UA_EN_SHIFT  (20)
#define PMU_LDO_LPSR_ANA_PULL_DOWN_20UA_EN(x)                                       \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_ANA_PULL_DOWN_20UA_EN_SHIFT)) & \
    PMU_LDO_LPSR_ANA_PULL_DOWN_20UA_EN_MASK)

#define PMU_LDO_LPSR_DIG_2_VOLTAGE_STEP_INC_MASK  (0x3)
#define PMU_LDO_LPSR_DIG_2_VOLTAGE_STEP_INC_SHIFT (0)
#define PMU_LDO_LPSR_DIG_2_VOLTAGE_STEP_INC(x)                                      \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_DIG_2_VOLTAGE_STEP_INC_SHIFT)) & \
    PMU_LDO_LPSR_DIG_2_VOLTAGE_STEP_INC_MASK)

#define PMU_LDO_LPSR_DIG_REG_EN_MASK  (0x4)
#define PMU_LDO_LPSR_DIG_REG_EN_SHIFT (2)
#define PMU_LDO_LPSR_DIG_REG_EN(x)                                                  \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_DIG_REG_EN_SHIFT)) & \
    PMU_LDO_LPSR_DIG_REG_EN_MASK)

#define PMU_LDO_LPSR_DIG_LPSR_DIG_CONTROL_MODE_MASK   (0x20)
#define PMU_LDO_LPSR_DIG_LPSR_DIG_CONTROL_MODE_SHIFT  (5)
#define PMU_LDO_LPSR_DIG_LPSR_DIG_CONTROL_MODE(x)                                   \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_DIG_LPSR_DIG_CONTROL_MODE_SHIFT)) & \
    PMU_LDO_LPSR_DIG_LPSR_DIG_CONTROL_MODE_MASK)

#define PMU_LDO_LPSR_DIG_STANDBY_EN_MASK  (0x40)
#define PMU_LDO_LPSR_DIG_STANDBY_EN_SHIFT (6)
#define PMU_LDO_LPSR_DIG_STANDBY_EN(x)                                              \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_DIG_STANDBY_EN_SHIFT)) & \
    PMU_LDO_LPSR_DIG_STANDBY_EN_MASK)

#define PMU_LDO_LPSR_DIG_TRACKING_MODE_MASK   (0x20000)
#define PMU_LDO_LPSR_DIG_TRACKING_MODE_SHIFT  (17)
#define PMU_LDO_LPSR_DIG_TRACKING_MODE(x)                                           \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_DIG_TRACKING_MODE_SHIFT)) & \
    PMU_LDO_LPSR_DIG_TRACKING_MODE_MASK)

#define PMU_LDO_LPSR_DIG_BYPASS_MODE_MASK   (0x40000)
#define PMU_LDO_LPSR_DIG_BYPASS_MODE_SHIFT  (18)
#define PMU_LDO_LPSR_DIG_BYPASS_MODE(x)                                             \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_DIG_BYPASS_MODE_SHIFT)) & \
    PMU_LDO_LPSR_DIG_BYPASS_MODE_MASK)

#define PMU_LDO_LPSR_DIG_VOLTAGE_SELECT_MASK  (0x1F00000)
#define PMU_LDO_LPSR_DIG_VOLTAGE_SELECT_SHIFT (20)
#define PMU_LDO_LPSR_DIG_VOLTAGE_SELECT(x)                                          \
    (((uint32_t)(((uint32_t)(x)) << PMU_LDO_LPSR_DIG_VOLTAGE_SELECT_SHIFT)) & \
    PMU_LDO_LPSR_DIG_VOLTAGE_SELECT_MASK)

/* CTRL */

#define PMU_CTRL_WB_CFG_1P8_WELL_SELECT_MASK         (0x1)
#define PMU_CTRL_WB_CFG_1P8_VOLTAGE_THRESHOLD_MASK   (0x2)
#define PMU_CTRL_WB_CFG_1P8_VOLTAGE_THRESHOLD_SHIFT  (1)
#define PMU_CTRL_WB_CFG_1P8_VOLTAGE_THRESHOLD(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << PMU_CTRL_WB_CFG_1P8_VOLTAGE_THRESHOLD_SHIFT)) & \
    PMU_CTRL_WB_CFG_1P8_VOLTAGE_THRESHOLD_MASK)
#define PMU_CTRL_WB_CFG_1P8_DRIVE_STRENGTH_MASK      (0x1C)
#define PMU_CTRL_WB_CFG_1P8_DRIVE_STRENGTH_SHIFT     (2)
#define PMU_CTRL_WB_CFG_1P8_DRIVE_STRENGTH(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << PMU_CTRL_WB_CFG_1P8_DRIVE_STRENGTH_SHIFT)) & \
    PMU_CTRL_WB_CFG_1P8_DRIVE_STRENGTH_MASK)

#define PMU_CTRL_WB_CFG_1P8_OSCILLATOR_FREQ_MASK     (0x1E0)
#define PMU_CTRL_WB_CFG_1P8_OSCILLATOR_FREQ_SHIFT    (5)
#define PMU_CTRL_WB_CFG_1P8_OSCILLATOR_FREQ(x)                                    \
    (((uint32_t)(((uint32_t)(x)) << PMU_CTRL_WB_CFG_1P8_OSCILLATOR_FREQ_SHIFT)) & \
    PMU_CTRL_WB_CFG_1P8_OSCILLATOR_FREQ_MASK)

/* CTRL2 */

#define PMU_CTRL2_WB_PWR_SW_EN_1P8_MASK               (0x1C00U)
#define PMU_CTRL2_WB_PWR_SW_EN_1P8_SHIFT              (10U)
#define PMU_CTRL2_WB_PWR_SW_EN_1P8(x)                                             \
    (((uint32_t)(((uint32_t)(x)) << PMU_CTRL2_WB_PWR_SW_EN_1P8_SHIFT)) & \
    PMU_CTRL2_WB_PWR_SW_EN_1P8_MASK)

#define PMU_CTRL2_WB_EN_MASK                          (0x1000000U)
#define PMU_CTRL2_WB_EN_SHIFT                         (24U)
#define PMU_CTRL2_WB_EN(x)                                                        \
    (((uint32_t)(((uint32_t)(x)) << PMU_CTRL2_WB_EN_SHIFT)) & \
    PMU_CTRL2_WB_EN_MASK)

#define PMU_CTRL2_WB_OK_MASK                          (0x4000000U)
#define PMU_CTRL2_WB_OK_SHIFT                         (26U)
#define PMU_CTRL2_WB_OK(x)                                                        \
    (((uint32_t)(((uint32_t)(x)) << PMU_CTRL2_WB_OK_SHIFT)) & \
    PMU_CTRL2_WB_OK_MASK)



#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT117X_PMU_H */
