/****************************************************************************
 * drivers/wireless/ieee80211/esp_hosted_fg/esp_spi.h
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

#ifndef _ESP_SPI_H_
#define _ESP_SPI_H_

#include "esp.h"

#define SPI_BUF_SIZE            1600

struct esp_spi_context
{
  struct esp_adapter            *adapter;
  struct spi_dev_s              *esp_spi_dev;
  struct sk_buff_head           tx_q[MAX_PRIORITY_QUEUES];
  struct sk_buff_head           rx_q[MAX_PRIORITY_QUEUES];
  struct work_s                 *spi_workqueue;
  worker_t                      spi_work;
};

enum
{
  CLOSE_DATAPATH,
  OPEN_DATAPATH,
};


#endif
