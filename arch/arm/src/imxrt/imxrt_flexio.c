/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexio.c
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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "barriers.h"

#include "imxrt_gpio.h"
#include "imxrt_periphclks.h"
#include "imxrt_flexio.h"
#include "hardware/imxrt_flexio.h"

#ifdef CONFIG_IMXRT_FLEXIO

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the FlexIO controller. */

struct imxrt_flexiodev_s
{
  struct flexio_dev_s     flexio;       /* Externally visible part of the FlexIO interface */
  struct flexio_type_s    *base;        /* FlexIO controller register base address */
  bool                    initialized;  /* TRUE: Controller has been initialized */
  mutex_t                 lock;         /* Assures mutually exclusive access to FlexIO */
};

struct flexio_config_s
{
  bool enable_flexio;                   /* Enable/disable FlexIO module */
  bool enable_indoze;                   /* Enable/disable FlexIO operation in doze mode */
  bool enable_indebug;                  /* Enable/disable FlexIO operation in debug mode */
  bool enable_fast_access;              /* Enable/disable fast access to FlexIO registers, fast access requires
                                         * the FlexIO clock to be at least twice the frequency of the bus clock. */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_IMXRT_FLEXIO1
static struct imxrt_flexiodev_s g_flexio1_dev =
{
  .base = (struct flexio_type_s *)IMXRT_FLEXIO1_BASE,
  .lock = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_IMXRT_FLEXIO2
static struct imxrt_flexiodev_s g_flexio2_dev =
{
  .base = (struct flexio_type_s *)IMXRT_FLEXIO2_BASE,
  .lock = NXMUTEX_INITIALIZER,
};
#endif

#ifdef CONFIG_IMXRT_FLEXIO3
static struct imxrt_flexiodev_s g_flexio3_dev =
{
  .base = (struct flexio_type_s *)IMXRT_FLEXIO3_BASE,
  .lock = NXMUTEX_INITIALIZER,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Resets the FlexIO module.
 *
 * param base FlexIO peripheral base address
 */

void imxrt_flexio_reset(struct flexio_type_s *base)
{
  /* do software reset, software reset operation affect all other FLEXIO
   * registers except CTRL
   */

  base->CTRL |= FLEXIO_CTRL_SWRST_MASK;
  base->CTRL = 0;
}

/* Configures the FlexIO with a FlexIO configuration. The configuration
 * structure can be filled by the user or be set with default values by
 * imxrt_flexio_get_default_config().
 *
 * Example
 * code
 * struct flexio_config_s config = {
 * .enable_flexio = true,
 * .enable_indoze = false,
 * .enable_indebug = true,
 * .enable_fast_access = false
 * };
 * imxrt_flexio_init(base, &config);
 * endcode
 *
 * param base FlexIO peripheral base address
 * param user_config pointer to flexio_config_s structure
 */

void imxrt_flexio_init(struct flexio_type_s *base,
                       const struct flexio_config_s *user_config)
{
  uint32_t ctrl_reg = 0;

  imxrt_flexio_reset(base);

  ctrl_reg = base->CTRL;
  ctrl_reg &= ~(FLEXIO_CTRL_DOZEN_MASK |
                FLEXIO_CTRL_DBGE_MASK |
                FLEXIO_CTRL_FASTACC_MASK |
                FLEXIO_CTRL_FLEXEN_MASK);
  ctrl_reg |= (FLEXIO_CTRL_DBGE(user_config->enable_indebug) |
               FLEXIO_CTRL_FASTACC(user_config->enable_fast_access) |
               FLEXIO_CTRL_FLEXEN(user_config->enable_flexio));
  if (!user_config->enable_indoze)
    {
      ctrl_reg |= FLEXIO_CTRL_DOZEN_MASK;
    }

  base->CTRL = ctrl_reg;
}

/* Gets the default configuration to configure the FlexIO module.
 *
 * Example:
 * code
 * struct flexio_config_s config;
 * imxrt_flexio_get_default_config(&config);
 * endcode
 *
 * param user_config pointer to flexio_config_s structure
 */

void imxrt_flexio_get_default_config(struct flexio_config_s *user_config)
{
  assert(user_config != NULL);

  (void)memset(user_config, 0, sizeof(*user_config));

  user_config->enable_flexio      = true;
  user_config->enable_indoze      = false;
  user_config->enable_indebug     = true;
  user_config->enable_fast_access = false;
}

/* Gets the shifter buffer address for the DMA transfer usage.
 *
 * param base FlexIO peripheral base address
 * param type Shifter type of enum flexio_shifter_buffer_type_e
 * param index Shifter index
 * return Corresponding shifter buffer index
 */

uint32_t imxrt_flexio_get_shifter_buffer_address(
  struct flexio_type_s *base,
  enum flexio_shifter_buffer_type_e type,
  uint8_t index)
{
  assert(index < FLEXIO_SHIFTBUF_COUNT);

  uint32_t address = 0;

  switch (type)
    {
    case FLEXIO_SHIFTER_BUFFER:
      address = (uint32_t) & (base->SHIFTBUF[index]);
      break;

    case FLEXIO_SHIFTER_BUFFER_BIT_SWAPPED:
      address = (uint32_t) & (base->SHIFTBUFBIS[index]);
      break;

    case FLEXIO_SHIFTER_BUFFER_BYTE_SWAPPED:
      address = (uint32_t) & (base->SHIFTBUFBYS[index]);
      break;

    case FLEXIO_SHIFTER_BUFFER_BIT_BYTE_SWAPPED:
      address = (uint32_t) & (base->SHIFTBUFBBS[index]);
      break;

    case FLEXIO_SHIFTER_BUFFER_NIBBLE_BYTE_SWAPPED:
      address = (uint32_t) & (base->SHIFTBUFNBS[index]);
      break;

    case FLEXIO_SHIFTER_BUFFER_HALF_WORD_SWAPPED:
      address = (uint32_t) & (base->SHIFTBUFHWS[index]);
      break;

    case FLEXIO_SHIFTER_BUFFER_NIBBLE_SWAPPED:
      address = (uint32_t) & (base->SHIFTBUFNIS[index]);
      break;

    default:
      address = (uint32_t) & (base->SHIFTBUF[index]);
      break;
    }

  return address;
}

/* Configures the shifter with the shifter configuration. The configuration
 * structure covers both the SHIFTCTL and SHIFTCFG registers. To configure
 * the shifter to the proper mode, select which timer controls the shifter
 * to shift, whether to generate start bit/stop bit, and the polarity of
 * start bit and stop bit.
 *
 * Example
 * code
 * struct flexio_shifter_config_s config = {
 * .timer_select = 0,
 * .timer_polarity = FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE,
 * .pin_config = FLEXIO_PIN_CONFIG_OPEN_DRAIN_OR_BIDIRECTION,
 * .pin_polarity = FLEXIO_PIN_ACTIVE_LOW,
 * .shifter_mode = FLEXIO_SHIFTER_MODE_TRANSMIT,
 * .input_source = FLEXIO_SHIFTER_INPUT_FROM_PIN,
 * .shifter_stop = FLEXIO_SHIFTER_STOP_BIT_HIGH,
 * .shifter_start = FLEXIO_SHIFTER_START_BIT_LOW
 * };
 * imxrt_flexio_set_shifter_config(base, &config);
 * endcode
 *
 * param base FlexIO peripheral base address
 * param index Shifter index
 * param shifter_config Pointer to flexio_shifter_config_s structure
 */

void imxrt_flexio_set_shifter_config(
  struct flexio_type_s *base,
  uint8_t index,
  const struct flexio_shifter_config_s *shifter_config)
{
  base->SHIFTCFG[index] =
    FLEXIO_SHIFTCFG_INSRC(shifter_config->input_source)
    | FLEXIO_SHIFTCFG_PWIDTH(shifter_config->parallel_width)
    | FLEXIO_SHIFTCFG_SSTOP(shifter_config->shifter_stop)
    | FLEXIO_SHIFTCFG_SSTART(shifter_config->shifter_start);

  base->SHIFTCTL[index] =
    FLEXIO_SHIFTCTL_TIMSEL(shifter_config->timer_select) |
    FLEXIO_SHIFTCTL_TIMPOL(shifter_config->timer_polarity) |
    FLEXIO_SHIFTCTL_PINCFG(shifter_config->pin_config) |
    FLEXIO_SHIFTCTL_PINSEL(shifter_config->pin_select) |
    FLEXIO_SHIFTCTL_PINPOL(shifter_config->pin_polarity) |
    FLEXIO_SHIFTCTL_SMOD(shifter_config->shifter_mode);
}

/* Configures the timer with the timer configuration. The configuration
 * structure covers both the TIMCTL and TIMCFG registers. To configure the
 * timer to the proper mode, select trigger source for timer and the timer
 * pin output and the timing for timer.
 *
 * Example
 * code
 * struct flexio_timer_config_s config = {
 * .trigger_select = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(0),
 * .trigger_polarity = FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW,
 * .trigger_source = FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL,
 * .pin_config = FLEXIO_PIN_CONFIG_OPEN_DRAIN_OR_BIDIRECTION,
 * .pin_select = 0,
 * .pin_polarity = FLEXIO_PIN_ACTIVE_HIGH,
 * .timer_mode = FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT,
 * .timer_output = FLEXIO_TIMER_OUTPUT_ZERO_NOT_AFFECTED_BY_RESET,
 * .timer_decrement =
 *    FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT,
 * .timer_reset = FLEXIO_TIMER_RESET_ON_TIMER_PIN_EQUAL_TO_TIMER_OUTPUT,
 * .timer_disable = FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE,
 * .timer_enable = FLEXIO_TIMER_ENABLE_ON_TRIGGER_HIGH,
 * .timer_stop = FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE,
 * .timer_start = FLEXIO_TIMER_START_BIT_ENABLED
 * };
 * imxrt_flexio_set_timer_config(base, &config);
 * endcode
 *
 * param base FlexIO peripheral base address
 * param index Timer index
 * param timer_config Pointer to the flexio_timer_config_s structure
 */

void imxrt_flexio_set_timer_config(
  struct flexio_type_s *base,
  uint8_t index,
  const struct flexio_timer_config_s *timer_config)
{
  base->TIMCFG[index] =
    FLEXIO_TIMCFG_TIMOUT(timer_config->timer_output) |
    FLEXIO_TIMCFG_TIMDEC(timer_config->timer_decrement) |
    FLEXIO_TIMCFG_TIMRST(timer_config->timer_reset) |
    FLEXIO_TIMCFG_TIMDIS(timer_config->timer_disable) |
    FLEXIO_TIMCFG_TIMENA(timer_config->timer_enable) |
    FLEXIO_TIMCFG_TSTOP(timer_config->timer_stop) |
    FLEXIO_TIMCFG_TSTART(timer_config->timer_start);

  base->TIMCMP[index] = FLEXIO_TIMCMP_CMP(timer_config->timer_compare);

  base->TIMCTL[index] = FLEXIO_TIMCTL_TRGSEL(timer_config->trigger_select) |
    FLEXIO_TIMCTL_TRGPOL(timer_config->trigger_polarity) |
    FLEXIO_TIMCTL_TRGSRC(timer_config->trigger_source) |
    FLEXIO_TIMCTL_PINCFG(timer_config->pin_config) |
    FLEXIO_TIMCTL_PINSEL(timer_config->pin_select) |
    FLEXIO_TIMCTL_PINPOL(timer_config->pin_polarity) |
    FLEXIO_TIMCTL_TIMOD(timer_config->timer_mode);
}

/****************************************************************************
 * Name: imxrt_flexio_initialize
 *
 * Description:
 *   Initialize the selected FlexIO port in master mode
 *
 * Input Parameters:
 *   intf - Interface number(must be zero)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int imxrt_flexio_initialize(int intf)
{
  struct imxrt_flexiodev_s *priv;
  struct flexio_config_s flexio_config;

  switch (intf)
    {
#ifdef CONFIG_IMXRT_FLEXIO1
    case 1:
      priv = &g_flexio1_dev;
      imxrt_clockall_flexio1();
      break;
#endif
#ifdef CONFIG_IMXRT_FLEXIO2
    case 2:
      priv = &g_flexio2_dev;
      imxrt_clockall_flexio2();
      break;
#endif
#ifdef CONFIG_IMXRT_FLEXIO3
    case 3:
      priv = &g_flexio3_dev;
      imxrt_clockall_flexio2();
      break;
#endif
    default:
      return -ENODEV;
    }

  /* Has the FlexSPI hardware been initialized? */

  if (!priv->initialized)
    {
      /* Now perform one time initialization */

      /* Perform hardware initialization. Puts the FlexIO into an active
       * state.
       */

      imxrt_flexio_get_default_config(&flexio_config);
      imxrt_flexio_init(priv->base, &flexio_config);

      /* Enable interrupts at the NVIC */

      priv->initialized = true;
    }

  return 0;
}

#endif /* CONFIG_IMXRT_FLEXIO */
