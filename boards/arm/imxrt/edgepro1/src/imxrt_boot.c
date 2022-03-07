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
#include "arm_arch.h"
#include "imxrt_flexspi_nor_boot.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/
struct mm_heap_s *itcm_heap;
struct mm_heap_s *ocram_heap;

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
  const uint32_t *src;
  uint32_t *dest;
  uint32_t regval;

  /* 
   * Final Configuration is
   *    128 ITCM    (0000:0000-0001:ffff)
   *    256 DTCM    (2000:0000-2005:ffff)
   *    640 OCRAM   (2020:0000-2027:ffff)
   * */
  putreg32(0x5aaffaa5, IMXRT_IOMUXC_GPR_GPR17);
  regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
  putreg32(regval | GPR_GPR16_FLEXRAM_BANK_CFG_SELF, IMXRT_IOMUXC_GPR_GPR16);

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

  itcm_heap = mm_initialize("itcm", (FAR void *)&_sitcm, 128*1024);

  ocram_heap = mm_initialize("ocram", (FAR void *)&_socram, 640*1024);
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
