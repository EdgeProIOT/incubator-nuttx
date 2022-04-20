/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_gpio.c
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

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"

#include <imxrt_gpio.h>
#include "edgepro1.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrtgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gpio_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpio_write(FAR struct gpio_dev_s *dev, bool value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpio_operations_s gpio_ops =
{
  .go_read   = gpio_read,
  .go_write  = gpio_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins */

static const uint32_t g_gpiopins[BOARD_NGPIO] =
{
  GPIO_NINA_NRST,
  GPIO_NINA_BOOT,
  GPIO_NINA_GPIO
};

static struct imxrtgpio_dev_s g_gpio[BOARD_NGPIO];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int gpio_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct imxrtgpio_dev_s *imxrtgpio = (FAR struct imxrtgpio_dev_s *)dev;

  DEBUGASSERT(imxrtgpio != NULL && value != NULL);
  DEBUGASSERT(imxrtgpio->id < BOARD_NGPIO);
  gpioinfo("Reading...\n");

  *value = imxrt_gpio_read(g_gpiopins[imxrtgpio->id]);
  return OK;
}

static int gpio_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct imxrtgpio_dev_s *imxrtgpio = (FAR struct imxrtgpio_dev_s *)dev;

  DEBUGASSERT(imxrtgpio != NULL);
  DEBUGASSERT(imxrtgpio->id < BOARD_NGPIO);
  gpioinfo("Writing %d\n", (int)value);

  imxrt_gpio_write(g_gpiopins[imxrtgpio->id], value);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int imxrt_gpio_initialize(void)
{
  int pincount = 0;
  int i;

  for (i = 0; i < BOARD_NGPIO; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpio[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpio[i].gpio.gp_ops     = &gpio_ops;
      g_gpio[i].id              = i;

      gpio_pin_register(&g_gpio[i].gpio, pincount);

      /* Configure the pin that will be used as output */

      imxrt_gpio_write(g_gpiopins[i], 0);
      imxrt_config_gpio(g_gpiopins[i]);

      pincount++;
    }

  return 0;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
