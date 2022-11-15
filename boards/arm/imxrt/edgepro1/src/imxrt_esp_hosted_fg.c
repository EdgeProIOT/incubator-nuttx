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

#include <debug.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/esp.h>

#include <arch/chip/pin.h>

#include "arm_internal.h"

#include "imxrt_config.h"
#include "imxrt_lpspi.h"
#include "imxrt_gpio.h"
#include "edgepro1.h"


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  esp_hosted_fg_irq_attach(xcpt_t, void *);
static void esp_hosted_fg_irq_enable(void);
static void esp_hosted_fg_irq_disable(void);
static void esp_hosted_fg_reset(bool);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct esp_hosted_fg_lower_s g_esp_hosted_fg_lower =
{
  .attach  = esp_hosted_fg_irq_attach,
  .enable  = esp_hosted_fg_irq_enable,
  .disable = esp_hosted_fg_irq_disable,
  .reset   = esp_hosted_fg_reset
};

static void *g_devhandle = NULL;
static volatile int32_t  _enable_count = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gesp_hosted_fg_irq_attach
 ****************************************************************************/

static int esp_hosted_fg_irq_attach(xcpt_t handler, void *arg)
{
  
  return 0;
}

/****************************************************************************
 * Name: gesp_hosted_fg_irq_enable
 ****************************************************************************/

static void esp_hosted_fg_irq_enable(void)
{
  irqstate_t flags = enter_critical_section();

  if (0 == _enable_count)
    {
      
    }

  _enable_count++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp_hosted_fg_irq_disable
 ****************************************************************************/

static void esp_hosted_fg_irq_disable(void)
{
  irqstate_t flags = enter_critical_section();

  _enable_count--;

  if (0 == _enable_count)
    {
      
    }

  leave_critical_section(flags);
}


/****************************************************************************
 * Name: esp_hosted_fg_reset
 ****************************************************************************/

static void esp_hosted_fg_reset(bool reset)
{
  cxd56_gpio_write(GPIO_NINA_NRST, !reset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_esp_hosted_fg_initialize
 ****************************************************************************/

int board_esp_hosted_fg_initialize(const char *devpath, int bus)
{
  struct spi_dev_s *spi;

  wlinfo("Initializing esp_hosted_fg\n");

  if (!g_devhandle)
    {
      imxrt_config_gpio(GPIO_NINA_NRST);
      imxrt_config_gpio(GPIO_NINA_BOOT);
      imxrt_config_gpio(GPIO_NINA_GPIO);

      /* Initialize spi device */
      
      spi = imxrt_lpspibus_initialize(bus);
      if (spi == NULL)
        {
          serr("ERROR: Failed to initialize spi%d\n", bus);
        }

      g_devhandle = esp_hosted_fg_register(devpath, spi, &g_esp_hosted_fg_lower);

      if (!g_devhandle)
        {
          wlerr("ERROR: Failed to register esp_hosted_fg driver.\n");
          return -ENODEV;
        }
    }

  return OK;
}
