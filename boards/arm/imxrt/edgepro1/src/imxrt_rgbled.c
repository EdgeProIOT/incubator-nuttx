/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_rgbled.c
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

#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/leds/rgbled.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "imxrt_flexpwm.h"
#include "edgepro1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_RGBLED 1

#ifndef CONFIG_PWM
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_IMXRT_FLEXPWM1
#  undef HAVE_RGBLED
#endif

#ifndef CONFIG_IMXRT_FLEXPWM2
#  undef HAVE_RGBLED
#endif

#ifdef HAVE_RGBLED

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_rgbled_setup
 *
 * Description:
 *   Initial for support of a connected RGB LED using PWM.
 *
 ****************************************************************************/

int imxrt_rgbled_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s    *ledr;
  struct pwm_lowerhalf_s    *ledg;
  struct pwm_lowerhalf_s    *ledb;
  struct file file;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call imxrt_pwminitialize() to get an instance of the PWM interface */

      ledr = imxrt_pwminitialize(RGBLED_PWM2);
      if (!ledr)
        {
          lederr("ERROR: Failed to get the IMXRT PWM lower half to LEDR\n");
          return -ENODEV;
        }

      ledr->ops->setup(ledr);

      /* Call imxrt_pwminitialize() to get an instance of the PWM interface */

      ledg = imxrt_pwminitialize(RGBLED_PWM1);
      if (!ledg)
        {
          lederr("ERROR: Failed to get the IMXRT PWM lower half to LEDG\n");
          return -ENODEV;
        }

      ledg->ops->setup(ledg);

      /* Call imxrt_pwminitialize() to get an instance of the PWM interface */

      ledb = ledg;

      /* Register the RGB LED diver at "/dev/rgbled0" */

#ifdef CONFIG_PWM_MULTICHAN
      ret = rgbled_register("/dev/rgbled0", ledr, ledg, ledb,
                            RGBLED_RPWMCHANNEL, RGBLED_GPWMCHANNEL,
                            RGBLED_BPWMCHANNEL);
#else
      ret = rgbled_register("/dev/rgbled0", ledr, ledg, ledb);
#endif
      if (ret < 0)
        {
          lederr("ERROR: rgbled_register failed: %d\n", ret);
          return ret;
        }

      ret = file_open(&file, "/dev/rgbled0", O_WRONLY);
      if (ret < 0)
        {
          lederr("ERROR: open failed: %d\n", ret);
          return ret;
        }

      /* Initialize led off */

      file_write(&file, "#000000", 8);
      file_close(&file);

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#else
#  error "HAVE_RGBLED is undefined"
#endif /* HAVE_RGBLED */
