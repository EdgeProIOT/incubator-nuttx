/****************************************************************************
 * boards/arm/imxrt/imxrt1064-evk/src/imxrt_flexspi_nor.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/flexspi.h>

#include "imxrt_flexspi.h"
#include "imxrt1064-evk.h"

#ifdef CONFIG_IMXRT_FLEXSPI

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_flexspi_nor_setup
 *
 * Description:
 *  This function is called by board-bringup logic to configure the
 *  flash device.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int imxrt_flexspi_nor_setup(void)
{
  FAR struct flexspi_dev_s *flexspi_dev ;
  FAR struct mtd_dev_s *mtd_dev;
  int ret = -1;

  flexspi_dev = imxrt_flexspi_initialize(0);
  if (!flexspi_dev)
    {
      _err("ERROR: Failed to initialize FlexSPI minor %d: %d\n",
           0, ret);
      return -1;
    }

  mtd_dev = flexspi_nor_initialize(flexspi_dev, false);
  if (!mtd_dev)
    {
      _err("ERROR: flexspi_nor_initialize() failed!\n");
      return -1;
    }

#ifdef CONFIG_FS_LITTLEFS
  /* Register the MTD driver so that it can be accessed from the
   * VFS.
   */

  ret = register_mtddriver("/dev/nor", mtd_dev, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n",
             ret);
    }

  /* mtd_dev->ioctl(mtd_dev, MTDIOC_BULKERASE, 0); */

  /* Mount the LittleFS file system */

  ret = nx_mount("/dev/nor", "/mnt/lfs", "littlefs", 0,
                 "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
             ret);
    }
#endif

  return 0;
}

#endif /* CONFIG_IMXRT_FLEXSPI */
