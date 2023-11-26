/****************************************************************************
 * drivers/wireless/ieee80211/esp_hosted_fg/esp_if.h
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

#ifndef __ESP_IF__H_
#define __ESP_IF__H_

#include "esp.h"

struct esp_if_ops
{
  int (*init)(struct esp_adapter *adapter);
  struct sk_buff* (*read)(struct esp_adapter *adapter);
  int (*write)(struct esp_adapter *adapter, struct sk_buff *skb);
  int (*deinit)(struct esp_adapter *adapter);
};

int esp_init_interface_layer(struct spi_dev_s *spi,
                             struct esp_adapter *adapter);
void esp_deinit_interface_layer(void);

#endif
