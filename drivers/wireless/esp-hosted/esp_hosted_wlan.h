/****************************************************************************
 * drivers/wireless/esp-hosted/esp_hosted_wlan.h
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

#ifndef __DRIVERS_WIRELESS_ESP_HOSTED_ESP_HOSTED_WLAN_H
#define __DRIVERS_WIRELESS_ESP_HOSTED_ESP_HOSTED_WLAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_ESP_HOSTED

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef ESP_HOSTED_WLAN_HAS_STA

/****************************************************************************
 * Name: esp_hosted_wlan_sta_set_linkstatus
 *
 * Description:
 *   Set Wi-Fi station link status
 *
 * Parameters:
 *   linkstatus - true Notifies the networking layer about an available
 *                carrier, false Notifies the networking layer about an
 *                disappeared carrier.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_hosted_wlan_sta_set_linkstatus(bool linkstatus);

/****************************************************************************
 * Name: esp_hosted_wlan_sta_initialize
 *
 * Description:
 *   Initialize the ESP-Hosted WLAN Station netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp_hosted_wlan_sta_initialize(void);
#endif /* ESP_HOSTED_WLAN_HAS_STA */

/****************************************************************************
 * Name: esp_hosted_wlan_softap_initialize
 *
 * Description:
 *   Initialize the ESP-Hosted WLAN SoftAP netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP_HOSTED_WLAN_HAS_SOFTAP
int esp_hosted_wlan_softap_initialize(void);
#endif /* ESP_HOSTED_WLAN_HAS_SOFTAP */

#endif /* CONFIG_ESP_HOSTED */
#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __DRIVERS_WIRELESS_ESP_HOSTED_ESP_HOSTED_WLAN_H */
