/****************************************************************************
 * drivers/wireless/esp-hosted/esp_hosted_control_api.c
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

#include "esp_hosted_control_api.h"
#include "esp_hosted_control_core.h"

#define CTRL_SEND_REQ(msGiD) do {                                     \
    req.msg_id = msGiD;                                               \
    if(SUCCESS != ctrl_app_send_req(&req)) {                          \
        printf("Failed to send control req %u\n", req.msg_id);        \
        return NULL;                                                  \
    }                                                                 \
} while(0);

#define CTRL_DECODE_RESP_IF_NOT_ASYNC() do {                          \
  if (CALLBACK_AVAILABLE == is_async_resp_callback_registered(req))   \
    return NULL;                                                      \
  return ctrl_wait_and_parse_sync_resp(&req);                         \
} while(0);

extern int init_hosted_control_lib_internal(void);
extern int deinit_hosted_control_lib_internal(void);


int init_hosted_control_lib(void)
{
  return init_hosted_control_lib_internal();
}

int deinit_hosted_control_lib(void)
{
  return deinit_hosted_control_lib_internal();
}

/** Control Req->Resp APIs **/
ctrl_cmd_t * wifi_get_mac(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_GET_MAC_ADDR);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_mac(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_SET_MAC_ADDR);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_mode(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_GET_WIFI_MODE);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_mode(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_SET_WIFI_MODE);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_power_save_mode(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_SET_PS_MODE);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_power_save_mode(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_GET_PS_MODE);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_ap_scan_list(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_GET_AP_SCAN_LIST);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_ap_config(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_GET_AP_CONFIG);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_connect_ap(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_CONNECT_AP);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_disconnect_ap(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_DISCONNECT_AP);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_start_softap(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_START_SOFTAP);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_softap_config(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_GET_SOFTAP_CONFIG);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_stop_softap(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_STOP_SOFTAP);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_softap_connected_station_list(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_GET_SOFTAP_CONN_STA_LIST);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_vendor_specific_ie(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_SET_SOFTAP_VND_IE);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_set_max_tx_power(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_SET_WIFI_MAX_TX_POWER);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * wifi_get_curr_tx_power(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_GET_WIFI_CURR_TX_POWER);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * config_heartbeat(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_CONFIG_HEARTBEAT);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_begin(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_OTA_BEGIN);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_write(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_OTA_WRITE);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}

ctrl_cmd_t * ota_end(ctrl_cmd_t req)
{
  CTRL_SEND_REQ(CTRL_REQ_OTA_END);
  CTRL_DECODE_RESP_IF_NOT_ASYNC();
}