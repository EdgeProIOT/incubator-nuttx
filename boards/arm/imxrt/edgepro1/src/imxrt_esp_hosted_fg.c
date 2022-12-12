/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_esp_hosted_fg.c
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

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/esp.h>

#include "imxrt_config.h"
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "edgepro1.h"


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  esp_hosted_fg_irq_attach(int, xcpt_t, void *);
static void esp_hosted_fg_irq_enable(int);
static void esp_hosted_fg_irq_disable(int);
static void esp_hosted_fg_reset(bool);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct esp_hosted_fg_lower_s g_esp_hosted_fg_lower =
{
  .attach  = esp_hosted_fg_irq_attach,
  .enable  = esp_hosted_fg_irq_enable,
  .disable = esp_hosted_fg_irq_disable,
  .read    = esp_hosted_fg_read,
  .reset   = esp_hosted_fg_reset
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int esp_hosted_fg_irq_attach(int irq, xcpt_t handler, void *arg)
{
  switch (irq)
    {
    case ESP_IRQ_HANDSHAKE:
      imxrt_config_gpio(GPIO_HANDSHAKE_INT);
      irq_attach(GPIO_HANDSHAKE_IRQ, handler, arg);
      break;
    case ESP_IRQ_DATA_READY:
      imxrt_config_gpio(GPIO_DATAREADY_INT);
      irq_attach(GPIO_DATAREADY_IRQ, handler, arg);
      break;
    default:
      break;
    }

  return 0;
}

static void esp_hosted_fg_irq_enable(int irq)
{
  irqstate_t flags = enter_critical_section();

  switch (irq)
    {
    case ESP_IRQ_HANDSHAKE:
      up_enable_irq(GPIO_HANDSHAKE_IRQ);
      break;
    case ESP_IRQ_DATA_READY:
      up_enable_irq(GPIO_DATAREADY_IRQ);
      break;
    default:
      break;
    }

  leave_critical_section(flags);
}

static void esp_hosted_fg_irq_disable(int irq)
{
  irqstate_t flags = enter_critical_section();

  switch (irq)
    {
    case ESP_IRQ_HANDSHAKE:
      up_disable_irq(GPIO_HANDSHAKE_IRQ);
      break;
    case ESP_IRQ_DATA_READY:
      up_disable_irq(GPIO_DATAREADY_IRQ);
      break;
    default:
      break;
    }

  leave_critical_section(flags);
}

static int esp_hosted_fg_read(int irq)
{
  int ret;
  irqstate_t flags = enter_critical_section();

  switch (irq)
    {
    case ESP_IRQ_HANDSHAKE:
      ret = imxrt_gpio_read(GPIO_HANDSHAKE_INT);
      break;
    case ESP_IRQ_DATA_READY:
      ret = imxrt_gpio_read(GPIO_DATAREADY_INT);
      break;
    default:
      break;
    }

  leave_critical_section(flags);

  return ret;
}

static void esp_hosted_fg_reset(bool reset)
{
  imxrt_gpio_write(GPIO_NINA_NRST, !reset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_esp_hosted_fg_initialize(void)
{
  struct spi_dev_s *spi;
  int ret;

  wlinfo("Initializing esp_hosted_fg\n");

  imxrt_config_gpio(GPIO_NINA_NRST);
  imxrt_config_gpio(GPIO_NINA_BOOT);
  imxrt_config_gpio(GPIO_NINA_GPIO);

  /* Initialize spi device */
  
  spi = imxrt_lpspibus_initialize(1);
  if (spi == NULL)
    {
      serr("ERROR: Failed to initialize spi%d\n", bus);
    }

  ret = esp_hosted_fg_register(spi, &g_esp_hosted_fg_lower);

  if (!ret)
    {
      wlerr("ERROR: Failed to register esp_hosted_fg driver.\n");
      return -ENODEV;
    }

  return OK;
}
