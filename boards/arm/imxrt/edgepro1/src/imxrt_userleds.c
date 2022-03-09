/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_userleds.c
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

/* TThere is RGB LED indicator located on the EdegPro1 Board.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include <arch/board/board.h>
#include "edgepro1.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED GPIO for output */

  imxrt_config_gpio(GPIO_LED_RED);
  imxrt_config_gpio(GPIO_LED_GREEN);
  imxrt_config_gpio(GPIO_LED_BLUE);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  switch (led)
    {
    case GPIO_LED_RED:
      imxrt_gpio_write(GPIO_LED_RED, !ledon);  /* Low illuminates */
      break;

    case GPIO_LED_GREEN:
      imxrt_gpio_write(GPIO_LED_GREEN, !ledon);  /* Low illuminates */
      break;

    case GPIO_LED_BLUE:
      imxrt_gpio_write(GPIO_LED_BLUE, !ledon);  /* Low illuminates */
      break;
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  /* Low illuminates */
  imxrt_gpio_write(GPIO_LED_RED, (ledset & BOARD_USERLED_RED_BIT) == 0);
  imxrt_gpio_write(GPIO_LED_GREEN, (ledset & BOARD_USERLED_GREEN_BIT) == 0);
  imxrt_gpio_write(GPIO_LED_BLUE, (ledset & BOARD_USERLED_BLUE_BIT) == 0);
}

#endif /* !CONFIG_ARCH_LEDS */
