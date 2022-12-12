/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_bringup.c
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

#include <sys/types.h>
#include <debug.h>

#include <syslog.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/lcd/lcd_dev.h>
#include <imxrt_lpi2c.h>
#include <imxrt_lpspi.h>
#include "edgepro1.h"

#include <arch/board/board.h>  /* Must always be included last */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C)
static void imxrt_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = imxrt_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      serr("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          serr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          imxrt_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

#if defined(CONFIG_SPI_DRIVER) && (defined(CONFIG_IMXRT_LPSPI4) || \
    defined(CONFIG_IMXRT_LPSPI1))
static void imxrt_spi_register(int bus)
{
  FAR struct spi_dev_s *spi;
  int ret;

  spi = imxrt_lpspibus_initialize(bus);
  if (spi == NULL)
    {
      serr("ERROR: Failed to get SPI%d interface\n", bus);
    }
  else
    {
      ret = spi_register(spi, bus);
      if (ret < 0)
        {
          serr("ERROR: Failed to register spi%d: %d\n", bus, ret);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int imxrt_bringup(void)
{
  int ret;

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C3)
  imxrt_i2c_register(3);
#endif

#if defined(CONFIG_SPI_DRIVER) && defined(CONFIG_IMXRT_LPSPI4)
  imxrt_spidev_initialize();
  imxrt_spi_register(4);
#endif

#ifdef CONFIG_DEV_GPIO
  /* Initialize the GPIO driver */

  ret = imxrt_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_IMXRT_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = imxrt_adc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imxrt_adc_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_IMXRT_FLEXSPI
  ret = imxrt_flexspi_nor_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
              "ERROR: imxrt_flexspi_nor_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_IMXRT_FLEXSPI */

#ifdef CONFIG_IEEE80211_ESP_HOSTED_FG
  ret = board_esp_hosted_fg_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
              "ERROR: Failed to initialize esp_hosted_fg.\n");
    }
#endif

#ifdef CONFIG_RGBLED
  /* Configure and initialize the RGB LED. */

  ret = imxrt_rgbled_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imxrt_rgbled_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LCD_DEV
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_lcd_initialize() failed: %d\n", ret);
    }

  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_MXKBD
  ret = mxkbd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mxkbd_initialize() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
