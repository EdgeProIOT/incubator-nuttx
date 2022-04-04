/****************************************************************************
 * include/nuttx/input/edgepromx.h
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

/* The EdgeProMX keyboard driver exports a standard character driver 
 * interface. By convention, the keyboard driver is exposed as /dev/kbd[n] 
 * device driver path.
 */

#ifndef __INCLUDE_NUTTX_INPUT_EDGEPROMX_H
#define __INCLUDE_NUTTX_INPUT_EDGEPROMX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdbool.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the
 * EdgeProMX Keyboard driver.  This structure provides information
 * about the configuration and provides some board-specific hooks.
 */

struct edgepromxkbd_config_s
{
  /* Device characterization */

  uint32_t frequency;  /* I2C frequency */
  uint8_t  address;    /* I2C 7-bit device address */

  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the EdgeProMX Keyboard driver from differences in 
   * GPIO interrupt handling by varying boards and MCUs.
   *
   * attach  - Attach the EdgeProMX kbd interrupt handler to the GPIO 
   *           interrupt
   * enable  - Enable or disable the GPIO interrupt
   * clear   - Acknowledge/clear any pending GPIO interrupt
   */

  int  (*attach)(FAR const struct edgepromxkbd_config_s *state, xcpt_t isr,
                 FAR void *arg);
  void (*enable)(FAR const struct edgepromxkbd_config_s *state, bool enable);
  void (*clear)(FAR const struct edgepromxkbd_config_s *state);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: edgepromxkbd_register
 *
 * Description:
 *   Configure the EdgeProMX Keyboard to use the provided I2C device
 *   instance.  This will register the driver as /dev/kbdN where N is the
 *   minor device number
 *
 * Input Parameters:
 *   i2c         - An I2C driver instance
 *   config      - Persistent board configuration data
 *   kbdminor    - The keyboard input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int edgepromxkbd_register(FAR struct i2c_master_s *i2c,
                      FAR const struct edgepromxkbd_config_s *config,
                      char kbdminor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_INPUT_EDGEPROMX_H */
