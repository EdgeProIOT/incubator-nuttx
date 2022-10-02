/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"

#include "imxrt_config.h"
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "edgepro1.h"

#if defined(CONFIG_IMXRT_LPSPI1) || defined(CONFIG_IMXRT_LPSPI2) || \
    defined(CONFIG_IMXRT_LPSPI3) || defined(CONFIG_IMXRT_LPSPI4)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the imxrt1060-evk
 *   board.
 *
 ****************************************************************************/

void weak_function imxrt_spidev_initialize(void)
{
#ifdef CONFIG_IMXRT_LPSPI1
  imxrt_config_gpio(GPIO_LPSPI1_CS); /* LPSPI1 chip select */
#endif
#ifdef CONFIG_IMXRT_LPSPI2
  imxrt_config_gpio(GPIO_LPSPI2_CS); /* LPSPI2 chip select */
#endif
#ifdef CONFIG_IMXRT_LPSPI3
  imxrt_config_gpio(GPIO_LPSPI3_CS); /* LPSPI3 chip select */
#endif
#ifdef CONFIG_IMXRT_LPSPI4
  imxrt_config_gpio(GPIO_LPSPI4_CS); /* LPSPI4 chip select */
#endif
}

/****************************************************************************
 * Name:  imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status
 *
 * Description:
 *  The external functions, imxrt_lpspi1/2/3select and imxrt_lpspi1/2/3status
 *  must be provided by board-specific logic.
 *  They are implementations of the select and status methods of the SPI
 *  interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *  All other methods (including imxrt_lpspibus_initialize())
 *  are provided by common STM32 logic.
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in imxrt_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide imxrt_lpspi1/2/3select() and imxrt_lpspi1/2/3status()
 *      functions in your board-specific logic.
 *      These functions will perform chip selection and status operations
 *      using GPIOs in the way your board is configured.
 *   3. Add a calls to imxrt_lpspibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by imxrt_lpspibus_initialize() may then be used
 *      to bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPSPI1
void imxrt_lpspi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI1_CS, !selected);
}

uint8_t imxrt_lpspi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI2
void imxrt_lpspi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI2_CS, !selected);
}

uint8_t imxrt_lpspi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI3
void imxrt_lpspi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI3_CS, !selected);
}

uint8_t imxrt_lpspi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_IMXRT_LPSPI4
void imxrt_lpspi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

  imxrt_gpio_write(GPIO_LPSPI4_CS, selected);
}

uint8_t imxrt_lpspi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#endif /* CONFIG_IMXRT_LPSPI1 || CONFIG_IMXRT_LPSPI2 */
