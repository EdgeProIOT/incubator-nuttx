/****************************************************************************
 * drivers/wireless/esp-hosted/esp_hosted_spi.h
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

#ifndef __DRIVERS_WIRELESS_ESP_HOSTED_ESP_HOSTED_SPI_H
#define __DRIVERS_WIRELESS_ESP_HOSTED_ESP_HOSTED_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>
#include <nuttx/mm/iob.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/esp_hosted.h>

#include "esp_hosted_adapter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

struct esp_hosted_spi_s
{
  FAR struct spi_dev_s *spi;
  struct iob_queue_s    tx_q[MAX_PRIORITY_QUEUES];
  struct iob_queue_s    rx_q[MAX_PRIORITY_QUEUES];
  struct work_s         work;
  FAR const struct esp_hosted_lower_s *lower;
  mutex_t               lock;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif



#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_WIRELESS_ESP_HOSTED_ESP_HOSTED_SPI_H */
