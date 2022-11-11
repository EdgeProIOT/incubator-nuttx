/*
 * Espressif Systems Wireless LAN device driver
 *
 * Copyright (C) 2015-2021 Espressif Systems (Shanghai) PTE LTD
 *
 * This software file (the "File") is distributed by Espressif Systems (Shanghai)
 * PTE LTD under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#ifndef __esp__h_
#define __esp__h_

#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/irq.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/bluetooth.h>
#include "adapter.h"

#define ESP_IF_TYPE_SDIO        1
#define ESP_IF_TYPE_SPI         2

/* Network link status */
#define ESP_LINK_DOWN           0
#define ESP_LINK_UP             1

#define ESP_MAX_INTERFACE       2

#define ESP_PAYLOAD_HEADER      8
struct esp_private;
struct esp_adapter;

#define ACQUIRE_LOCK            1
#define LOCK_ALREADY_ACQUIRED   0

#define SKB_DATA_ADDR_ALIGNMENT 4
#define INTERFACE_HEADER_PADDING (SKB_DATA_ADDR_ALIGNMENT*3)

struct esp_adapter
{
  uint8_t                 if_type;
  uint32_t                capabilities;

  /* Possible types:
   * struct esp_sdio_context */
  void                    *if_context;

  struct esp_if_ops       *if_ops;

  /* Private for each interface */
  struct esp_private      *priv[ESP_MAX_INTERFACE];
  struct hci_dev_t        *hcidev;

  struct work_s           *if_rx_workqueue;
  worker_t                 if_rx_work;

  /* Process TX work */
  struct work_s           *tx_workqueue;
  worker_t                 tx_work;
};


struct esp_private
{
  struct esp_adapter      *adapter;
  struct net_driver_s     *ndev;
  //struct net_device_stats stats;
  uint8_t                 link_state;
  uint8_t                 mac_address[6];
  uint8_t                 if_type;
  uint8_t                 if_num;
};

struct esp_skb_cb
{
  struct esp_private      *priv;
};
#endif
