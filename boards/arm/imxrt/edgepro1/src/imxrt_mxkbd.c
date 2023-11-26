/****************************************************************************
 * boards/arm/imxrt/edgepro1/src/imxrt_mxkbd.c
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
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/ioexpander/mcp23x17.h>
#include <nuttx/input/mxkbd.h>
#include <imxrt_lpi2c.h>
#include "edgepro1.h"

#ifdef CONFIG_INPUT_MXKBD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCP23X17_BUS        3
#define MCP23X17_ADDRESS    0x20
#define MCP23X17_FREQUENCY  400000 

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the MCP23x17
 * driver.  This structure provides information about the configuration
 * of the MCP23x17 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

static struct mcp23x17_config_s g_mcp23x17_info =
{
  .address        = MCP23X17_ADDRESS,
  .frequency      = MCP23X17_FREQUENCY,
  .set_nreset_pin = NULL
};

 /****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mxkbd_initialize
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   keyboard device. This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mxkbd_initialize(void)
{
  struct i2c_master_s *i2c;
  struct ioexpander_dev_s *ioe;
  int ret;

  i2c = imxrt_i2cbus_initialize(MCP23X17_BUS);
  if (i2c == NULL)
    {
      ierr("ERROR: Failed to get I2C interface\n");
      return -ENODEV;
    }

  /* Get an instance of the I/O expander */
  ioe = mcp23x17_initialize(i2c, &g_mcp23x17_info);
  if (ioe == NULL)
    {
      ierr("ERROR: mcp23x17_initialize failed\n");
      return -ENODEV;
    }

  ret = mxkbd_register(ioe, 0);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register input device minor=%d\n", 0);
      return -ENODEV;
    }

  return OK;
}


#endif /* CONFIG_INPUT_MXKBD */
