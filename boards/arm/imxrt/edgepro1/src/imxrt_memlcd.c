/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_memlcd.c
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

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/memlcd.h>

#include "imxrt_lpspi.h"
#include "edgepro1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_SPI_PORTNO 4

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcd = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct memlcd_priv_s memlcd_priv =
{
  .attachirq   = NULL,
  .dispcontrol = NULL,
#ifndef CONFIG_MEMLCD_EXTCOMIN_MODE_HW
  .setpolarity = NULL,
#endif
  .setvcomfreq = NULL,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  g_spidev = imxrt_lpspibus_initialize(LCD_SPI_PORTNO);
  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", LCD_SPI_PORTNO);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  g_lcd = memlcd_initialize(g_spidev, &memlcd_priv, 0);
  if (!g_lcd)
    {
      lcderr("ERROR: Failed to bind SPI port 4 to LCD %d\n", devno);
    }
  else
    {
      lcdinfo("SPI port 4 bound to LCD %d\n", devno);
      return g_lcd;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* Turn the display off */

  g_lcd->setpower(g_lcd, 0);
}
