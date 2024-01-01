/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_boot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>
#include <nuttx/mm/mm.h>
#include <arch/board/board.h>

#include "imxrt_start.h"
#include "edgepro1.h"
#include "arm_internal.h"
#include "imxrt_flexspi_nor_boot.h"

#define CONFIG_ITCM_USED 0
#if defined(CONFIG_IMXRT_ITCM)
#  if (CONFIG_IMXRT_ITCM % 32) != 0
#    error IMXRT_ITCM must be divisible by 32
#  endif
#  undef CONFIG_ITCM_USED
#  define CONFIG_ITCM_USED (CONFIG_IMXRT_ITCM * 1024)
#else
#  define CONFIG_IMXRT_ITCM 0
#endif

#define CONFIG_DTCM_USED 0
#if defined(CONFIG_IMXRT_DTCM)
#  if (CONFIG_IMXRT_DTCM % 32) != 0
#    error CONFIG_IMXRT_DTCM must be divisible by 32
#  endif
#  undef CONFIG_DTCM_USED
#  define CONFIG_DTCM_USED (CONFIG_IMXRT_DTCM * 1024)
#else
#  define IMXRT_DTCM 0
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct mm_heap_s *itcm_heap;
struct mm_heap_s *ocram_heap;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t map_tcm_size(uint8_t tcm_bank_num)
{
  uint8_t tcm_size_config;
  uint32_t total_tcm_size;

  tcm_size_config = 0U;
  total_tcm_size = 0U;

  /* if bank number is a odd value, use a new bank number which bigger
   * than target
   */

  do
    {
      if ((tcm_bank_num & (tcm_bank_num - 1U)) == 0U)
        {
          break;
        }
    }
  while (++tcm_bank_num < 16);

  total_tcm_size = tcm_bank_num * (32768 >> 10U);

  /* get bit '1' position */

  while (total_tcm_size)
    {
      if ((total_tcm_size & 1U) == 0U)
        {
          tcm_size_config++;
        }
      else
        {
          break;
        }

      total_tcm_size >>= 1U;
    }

  return tcm_size_config + 1U;
}

static void set_tcm_size(uint8_t itcm_bank_num, uint8_t dtcm_bank_num)
{
  uint32_t regval;

  /* dtcm configuration */

  if (dtcm_bank_num != 0U)
    {
      regval = getreg32(IMXRT_IOMUXC_GPR_GPR14);
      regval &= ~GPR_GPR14_CM7_CFGDTCMSZ_MASK;
      putreg32(regval |
        map_tcm_size(dtcm_bank_num) << GPR_GPR14_CM7_CFGDTCMSZ_SHIFT,
        IMXRT_IOMUXC_GPR_GPR14);
      regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
      putreg32(regval | GPR_GPR16_INIT_DTCM_EN, IMXRT_IOMUXC_GPR_GPR16);
    }
  else
    {
      regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
      putreg32(regval & ~GPR_GPR16_INIT_DTCM_EN, IMXRT_IOMUXC_GPR_GPR16);
    }

  /* itcm configuration */

  if (itcm_bank_num != 0U)
    {
      regval = getreg32(IMXRT_IOMUXC_GPR_GPR14);
      regval &= ~GPR_GPR14_CM7_CFGITCMSZ_MASK;
      putreg32(regval |
        map_tcm_size(itcm_bank_num) << GPR_GPR14_CM7_CFGITCMSZ_SHIFT,
        IMXRT_IOMUXC_GPR_GPR14);
      regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
      putreg32(regval | GPR_GPR16_INIT_ITCM_EN, IMXRT_IOMUXC_GPR_GPR16);
    }
  else
    {
      regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
      putreg32(regval & ~GPR_GPR16_INIT_ITCM_EN, IMXRT_IOMUXC_GPR_GPR16);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_ocram_initialize
 *
 * Description:
 *   Called off reset vector to reconfigure the flexRAM
 *   and finish the FLASH to RAM Copy.
 *
 ****************************************************************************/

void imxrt_ocram_initialize(void)
{
  uint32_t regval;
  uint8_t dtcm_bank_num;
  uint8_t itcm_bank_num;
  uint8_t ocram_bank_num;
  uint32_t bank_cfg;
  uint32_t i;
#ifdef CONFIG_BOOT_RUNFROMISRAM
  const uint32_t *src;
  uint32_t *dest;
#endif

  dtcm_bank_num = (CONFIG_IMXRT_DTCM / 32);
  itcm_bank_num = (CONFIG_IMXRT_ITCM / 32);
  ocram_bank_num = ((512 - CONFIG_IMXRT_DTCM - CONFIG_IMXRT_ITCM) / 32);
  bank_cfg = 0U;

  /* check the arguments */

  if (16 < (dtcm_bank_num + itcm_bank_num + ocram_bank_num))
    {
      return;
    }

  /* flexram bank config value */

  for (i = 0U; i < 16; i++)
    {
      if (i < ocram_bank_num)
        {
          bank_cfg |= ((uint32_t)1U) << (i * 2);
          continue;
        }

      if (i < (dtcm_bank_num + ocram_bank_num))
        {
          bank_cfg |= ((uint32_t)2U) << (i * 2);
          continue;
        }

      if (i < (dtcm_bank_num + ocram_bank_num + itcm_bank_num))
        {
          bank_cfg |= ((uint32_t)3U) << (i * 2);
          continue;
        }
    }

  putreg32(bank_cfg, IMXRT_IOMUXC_GPR_GPR17);
  set_tcm_size(itcm_bank_num, dtcm_bank_num);
  regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
  putreg32(regval | GPR_GPR16_FLEXRAM_BANK_CFG_SEL, IMXRT_IOMUXC_GPR_GPR16);

#ifdef CONFIG_BOOT_RUNFROMISRAM
  src = (uint32_t *) (LOCATE_IN_SRC(g_boot_data.start) + g_boot_data.size);
  dest = (uint32_t *) (g_boot_data.start + g_boot_data.size);

  while (dest < (uint32_t *) &_etext)
    {
      *dest++ = *src++;
    }
#endif
}

/****************************************************************************
 * Name: imxrt_boardinitialize
 *
 * Description:
 *   All i.MX RT architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocking and
 *   memory have been configured but before caches have been enabled and
 *   before any devices have been initialized.
 *
 ****************************************************************************/

extern uint32_t _sitcm;
extern uint32_t _socram;

void imxrt_boardinitialize(void)
{
  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  imxrt_autoled_initialize();
#endif

  itcm_heap = mm_initialize("itcm", (FAR void *)&_sitcm,
    CONFIG_IMXRT_ITCM * 1024);

  ocram_heap = mm_initialize("ocram2", (FAR void *)&_socram,
    (0x00100000 - CONFIG_IMXRT_ITCM * 1024 - CONFIG_IMXRT_DTCM * 1024));
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_intitialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board initialization */

  imxrt_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
