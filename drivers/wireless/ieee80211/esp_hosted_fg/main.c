/****************************************************************************
 * drivers/wireless/ieee80211/esp_hosted_fg/main.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/ascii.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/esp.h>
#include <nuttx/net/netdev.h>

#include "esp_if.h"
#ifdef CONFIG_SUPPORT_ESP_SERIAL
#include "esp_serial.h"
#endif
#include "esp_bt_api.h"
#include "esp_api.h"
#include "esp_kernel_port.h"
#include "esp_stats.h"


struct esp_adapter adapter;
volatile uint8_t stop_data = 0;

#define ACTION_DROP 1

/**
 * ether_addr_copy - Copy an Ethernet address
 * @dst: Pointer to a six-byte array Ethernet address destination
 * @src: Pointer to a six-byte array Ethernet address source
 *
 * Please note: dst & src must both be aligned to uint16_t.
 */
static inline void ether_addr_copy(uint8_t *dst, const uint8_t *src)
{
  *(uint32_t *)dst = *(const uint32_t *)src;
  *(uint16_t *)(dst + 4) = *(const uint16_t *)(src + 4);
}

static int esp_open(struct net_driver_s *ndev);
static int esp_stop(struct net_driver_s *ndev);
static int esp_hard_start_xmit(struct sk_buff *skb, struct net_driver_s *ndev);
static int esp_set_mac_address(struct net_driver_s *ndev, void *addr);
static struct net_driver_s_stats* esp_get_stats(struct net_driver_s *ndev);
static void esp_set_rx_mode(struct net_driver_s *ndev);
static int process_tx_packet (struct sk_buff *skb);
static NDO_TX_TIMEOUT_PROTOTYPE();
int esp_send_packet(struct esp_adapter *adapter, struct sk_buff *skb);
struct sk_buff * esp_alloc_skb(uint32_t len);

static const struct net_driver_s_ops esp_netdev_ops =
{
  .ndo_open = esp_open,
  .ndo_stop = esp_stop,
  .ndo_start_xmit = esp_hard_start_xmit,
  .ndo_set_mac_address = esp_set_mac_address,
  .ndo_validate_addr = eth_validate_addr,
  .ndo_tx_timeout = esp_tx_timeout,
  .ndo_get_stats = esp_get_stats,
  .ndo_set_rx_mode = esp_set_rx_mode,
};

struct esp_adapter * esp_get_adapter(void)
{
  return &adapter;
}

static int esp_open(struct net_driver_s *ndev)
{
  netif_start_queue(ndev);
  return 0;
}

static int esp_stop(struct net_driver_s *ndev)
{
  netif_stop_queue(ndev);
  return 0;
}

static struct net_driver_s_stats* esp_get_stats(struct net_driver_s *ndev)
{
  struct esp_private *priv = netdev_priv(ndev);
  return &priv->stats;
}

static int esp_set_mac_address(struct net_driver_s *ndev, void *data)
{
  struct esp_private *priv = netdev_priv(ndev);
  struct sockaddr *mac_addr = data;

  if (!priv)
    {
      return -EINVAL;
    }

  ether_addr_copy(priv->mac_address, mac_addr->sa_data);
  ether_addr_copy(ndev->dev_addr, mac_addr->sa_data);
  return 0;
}

static NDO_TX_TIMEOUT_PROTOTYPE()
{
}

static void esp_set_rx_mode(struct net_driver_s *ndev)
{
}

static int esp_hard_start_xmit(struct sk_buff *skb, struct net_driver_s *ndev)
{
  struct esp_private *priv = netdev_priv(ndev);
  struct esp_skb_cb *cb = NULL;

  if (!priv)
    {
      dev_kfree_skb(skb);
      return NETDEV_TX_OK;
    }

  if (!skb->len || (skb->len > ETH_FRAME_LEN))
    {
      wlerr( "%s: Bad len %d\n", __func__, skb->len);
      priv->stats.tx_dropped++;
      dev_kfree_skb(skb);
      return NETDEV_TX_OK;
    }

  cb = (struct esp_skb_cb *) skb->cb;
  cb->priv = priv;

  return process_tx_packet(skb);
}

uint8_t esp_is_bt_supported_over_sdio(uint32_t cap)
{
  return (cap & ESP_BT_SDIO_SUPPORT);
}

static struct esp_private * get_priv_from_payload_header(struct esp_payload_header *header)
{
  struct esp_private *priv = NULL;
  uint8_t i = 0;

  if (!header)
    {
      return NULL;
    }

  for (i = 0; i < ESP_MAX_INTERFACE; i++)
    {
      priv = adapter.priv[i];

      if (!priv)
        {
          continue;
        }

      if (priv->if_type == header->if_type &&
          priv->if_num == header->if_num)
        {
          return priv;
        }
    }

  return NULL;
}

void esp_process_new_packet_intr(struct esp_adapter *adapter)
{
  if (adapter)
    {
      queue_work(adapter->if_rx_workqueue, &adapter->if_rx_work);
    }
}

static int process_tx_packet(struct sk_buff *skb)
{
  struct esp_private *priv = NULL;
  struct esp_skb_cb *cb = NULL;
  struct esp_payload_header *payload_header = NULL;
  struct sk_buff *new_skb = NULL;
  int ret = 0;
  uint8_t pad_len = 0, realloc_skb = 0;
  uint16_t len = 0;
  uint16_t total_len = 0;
  static uint8_t c = 0;
  uint8_t *pos = NULL;

  c++;

  /* Get the priv */

  cb = (struct esp_skb_cb *) skb->cb;
  priv = cb->priv;

  if (!priv)
    {
      dev_kfree_skb(skb);
      return NETDEV_TX_OK;
    }

  if (netif_queue_stopped((const struct net_driver_s *) adapter.priv[0]->ndev) ||
      netif_queue_stopped((const struct net_driver_s *) adapter.priv[1]->ndev))
    {
      return NETDEV_TX_BUSY;
    }

  len = skb->len;

  /* Create space for payload header */

  pad_len = sizeof(struct esp_payload_header);

  total_len = len + pad_len;

  /* Align buffer length */

  pad_len += SKB_DATA_ADDR_ALIGNMENT - (total_len % SKB_DATA_ADDR_ALIGNMENT);

  if (skb_headroom(skb) < pad_len)
    {
      /* Headroom is not sufficient */

      realloc_skb = 1;
    }

  if (realloc_skb || !IS_ALIGNED((unsigned long) skb->data, SKB_DATA_ADDR_ALIGNMENT))
    {
      /* Realloc SKB */

      if (skb_linearize(skb))
        {
          priv->stats.tx_errors++;
          dev_kfree_skb(skb);
          return NETDEV_TX_OK;
        }

      new_skb = esp_alloc_skb(skb->len + pad_len);

      if (!new_skb)
        {
          wlerr( "%s: Failed to allocate SKB", __func__);
          priv->stats.tx_errors++;
          dev_kfree_skb(skb);
          return NETDEV_TX_OK;
        }

      pos = new_skb->data;
      pos += pad_len;

      /* Populate new SKB */

      skb_copy_from_linear_data(skb, pos, skb->len);
      skb_put(new_skb, skb->len + pad_len);

      /* Replace old SKB */

      dev_kfree_skb_any(skb);
      skb = new_skb;
    }
  else
    {
      /* Realloc is not needed, Make space for interface header */

      skb_push(skb, pad_len);
    }

  /* Set payload header */

  payload_header = (struct esp_payload_header *) skb->data;
  memset(payload_header, 0, pad_len);

  payload_header->if_type = priv->if_type;
  payload_header->if_num = priv->if_num;
  payload_header->len = cpu_to_le16(len);
  payload_header->offset = cpu_to_le16(pad_len);

  if (adapter.capabilities & ESP_CHECKSUM_ENABLED)
    {
      payload_header->checksum = cpu_to_le16(compute_checksum(skb->data, (len + pad_len)));
    }

  if (!stop_data)
    {
      ret = esp_send_packet(priv->adapter, skb);

      if (ret)
        {
          priv->stats.tx_errors++;
        }
      else
        {
          priv->stats.tx_packets++;
          priv->stats.tx_bytes += skb->len;
        }
    }
  else
    {
      dev_kfree_skb_any(skb);
      priv->stats.tx_dropped++;
    }

  return 0;
}

void process_capabilities(uint8_t cap)
{
  struct esp_adapter *adapter = esp_get_adapter();
  wlinfo( "ESP peripheral capabilities: 0x%x\n", cap);
  adapter->capabilities = cap;

  /* Reset BT */

  esp_deinit_bt(esp_get_adapter());

  if ((cap & ESP_BT_SPI_SUPPORT) || (cap & ESP_BT_SDIO_SUPPORT))
    {
      usleep(200000);
      esp_init_bt(esp_get_adapter());
    }
}

static void process_event(uint8_t *evt_buf, uint16_t len)
{
  int ret = 0;
  struct esp_priv_event *event;

  if (!evt_buf || !len)
    {
      return;
    }

  event = (struct esp_priv_event *) evt_buf;

  if (event->event_type == ESP_PRIV_EVENT_INIT)
    {

      wlinfo( "\nReceived INIT event from ESP32 peripheral");

      ret = process_init_event(event->event_data, event->event_len);

#ifdef CONFIG_SUPPORT_ESP_SERIAL
      if (!ret)
        {
          esp_serial_reinit(esp_get_adapter());
        }
#endif

    }
  else
    {
      wlwarn( "Drop unknown event\n");
    }
}

static void process_priv_communication(struct sk_buff *skb)
{
  struct esp_payload_header *header;
  uint8_t *payload;
  uint16_t len;

  if (!skb || !skb->data)
    {
      return;
    }

  header = (struct esp_payload_header *) skb->data;

  payload = skb->data + le16_to_cpu(header->offset);
  len = le16_to_cpu(header->len);

  if (header->priv_pkt_type == ESP_PACKET_TYPE_EVENT)
    {
      process_event(payload, len);
    }

  dev_kfree_skb(skb);
}

static void process_rx_packet(struct sk_buff *skb)
{
  struct esp_private *priv = NULL;
  struct esp_payload_header *payload_header = NULL;
  uint16_t len = 0, offset = 0;
  uint16_t rx_checksum = 0, checksum = 0;
  struct hci_dev *hdev = adapter.hcidev;
  uint8_t *type = NULL;
  int ret = 0, ret_len = 0;
  struct esp_adapter *adapter = esp_get_adapter();

  if (!skb)
    {
      return;
    }

  /* get the paload header */

  payload_header = (struct esp_payload_header *) skb->data;

  len = le16_to_cpu(payload_header->len);
  offset = le16_to_cpu(payload_header->offset);

  if (adapter->capabilities & ESP_CHECKSUM_ENABLED)
    {
      rx_checksum = le16_to_cpu(payload_header->checksum);
      payload_header->checksum = 0;

      checksum = compute_checksum(skb->data, (len + offset));

      if (checksum != rx_checksum)
        {
          dev_kfree_skb_any(skb);
          return;
        }
    }

  if (payload_header->if_type == ESP_SERIAL_IF)
    {
#ifdef CONFIG_SUPPORT_ESP_SERIAL
      /* print_hex_dump(KERN_INFO, "esp_serial_rx: ",
       * DUMP_PREFIX_ADDRESS, 16, 1, skb->data + offset, len, 1  ); */

      do
        {
          ret = esp_serial_data_received(payload_header->if_num,
                                         (skb->data + offset + ret_len), (len - ret_len));
          if (ret < 0)
            {
              wlerr( "%s, Failed to process data for iface type %d\n",
                     __func__, payload_header->if_num);
              break;
            }
          ret_len += ret;
        }
      while (ret_len < len);
#else
      wlerr( "%s, Dropping unsupported serial frame\n", __func__);
#endif
      dev_kfree_skb_any(skb);
    }
  else if (payload_header->if_type == ESP_STA_IF ||
           payload_header->if_type == ESP_AP_IF)
    {
      /* chop off the header from skb */

      skb_pull(skb, offset);

      /* retrieve priv based on payload header contents */

      priv = get_priv_from_payload_header(payload_header);

      if (!priv)
        {
          wlerr( "%s: empty priv\n", __func__);
          dev_kfree_skb_any(skb);
          return;
        }

      skb->dev = priv->ndev;
      skb->protocol = eth_type_trans(skb, priv->ndev);
      skb->ip_summed = CHECKSUM_NONE;

      /* Forward skb to kernel */

      netif_rx_ni(skb);

      priv->stats.rx_bytes += skb->len;
      priv->stats.rx_packets++;
    }
  else if (payload_header->if_type == ESP_HCI_IF)
    {
      if (hdev)
        {
          /* chop off the header from skb */

          skb_pull(skb, offset);

          type = skb->data;

          /* print_hex_dump(KERN_INFO, "bt_rx: ",
           * DUMP_PREFIX_ADDRESS, 16, 1, skb->data, len, 1);*/

          hci_skb_pkt_type(skb) = *type;
          skb_pull(skb, 1);

          if (hci_recv_frame(hdev, skb))
            {
              hdev->stat.err_rx++;
            }
          else
            {
              esp_hci_update_rx_counter(hdev, *type, skb->len);
            }
        }
    }
  else if (payload_header->if_type == ESP_PRIV_IF)
    {
      process_priv_communication(skb);
    }
  else if (payload_header->if_type == ESP_TEST_IF)
    {
      dev_kfree_skb_any(skb);
    }
}

int esp_is_tx_queue_paused(void)
{
  if ((adapter.priv[0]->ndev &&
       !netif_queue_stopped((const struct net_driver_s *)
                            adapter.priv[0]->ndev)) ||
      (adapter.priv[1]->ndev &&
       !netif_queue_stopped((const struct net_driver_s *)
                            adapter.priv[1]->ndev)))
    {
      return 1;
    }
  return 0;
}

void esp_tx_pause(void)
{
  if (adapter.priv[0]->ndev &&
      !netif_queue_stopped((const struct net_driver_s *)
                           adapter.priv[0]->ndev))
    {
      netif_stop_queue(adapter.priv[0]->ndev);
    }

  if (adapter.priv[1]->ndev &&
      !netif_queue_stopped((const struct net_driver_s *)
                           adapter.priv[1]->ndev))
    {
      netif_stop_queue(adapter.priv[1]->ndev);
    }
}

void esp_tx_resume(void)
{
  if (adapter.priv[0]->ndev &&
      netif_queue_stopped((const struct net_driver_s *)
                          adapter.priv[0]->ndev))
    {
      netif_wake_queue(adapter.priv[0]->ndev);
    }

  if (adapter.priv[1]->ndev &&
      netif_queue_stopped((const struct net_driver_s *)
                          adapter.priv[1]->ndev))
    {
      netif_wake_queue(adapter.priv[1]->ndev);
    }
}

struct sk_buff * esp_alloc_skb(uint32_t len)
{
  struct sk_buff *skb = NULL;

  uint8_t offset;

  skb = netdev_alloc_skb(NULL, len + INTERFACE_HEADER_PADDING);

  if (skb)
    {
      /* Align SKB data pointer */

      offset = ((unsigned long)skb->data) & (SKB_DATA_ADDR_ALIGNMENT - 1);

      if (offset)
        {
          skb_reserve(skb, INTERFACE_HEADER_PADDING - offset);
        }
    }

  return skb;
}


static int esp_get_packets(struct esp_adapter *adapter)
{
  struct sk_buff *skb = NULL;

  if (!adapter || !adapter->if_ops || !adapter->if_ops->read)
    {
      return -EINVAL;
    }

  skb = adapter->if_ops->read(adapter);

  if (!skb)
    {
      return -EFAULT;
    }

  process_rx_packet(skb);

  return 0;
}

int esp_send_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
  if (!adapter || !adapter->if_ops || !adapter->if_ops->write)
    {
      return -EINVAL;
    }

  return adapter->if_ops->write(adapter, skb);
}

static int insert_priv_to_adapter(struct esp_private *priv)
{
  int i = 0;

  for (i = 0; i < ESP_MAX_INTERFACE; i++)
    {
      /* Check if priv can be added */

      if (adapter.priv[i] == NULL)
        {
          adapter.priv[i] = priv;
          return 0;
        }
    }

  return -1;
}

static int esp_init_priv(struct esp_private *priv, struct net_driver_s *dev,
                         uint8_t if_type, uint8_t if_num)
{
  int ret = 0;

  if (!priv || !dev)
    {
      return -EINVAL;
    }

  ret = insert_priv_to_adapter(priv);
  if (ret)
    {
      return ret;
    }

  priv->ndev = dev;
  priv->if_type = if_type;
  priv->if_num = if_num;
  priv->link_state = ESP_LINK_DOWN;
  priv->adapter = &adapter;
  memset(&priv->stats, 0, sizeof(priv->stats));

  return 0;
}

static int esp_init_net_dev(struct net_driver_s *ndev, struct esp_private *priv)
{
  int ret = 0;
  /* Set netdev */
  /*  SET_NETDEV_DEV(ndev, &adapter->context.func->dev);*/

  /* set net dev ops */

  ndev->netdev_ops = &esp_netdev_ops;

  ether_addr_copy(ndev->dev_addr, priv->mac_address);

  /* set ethtool ops */

  /* update features supported */

  /* min mtu */

  /* register netdev */

  ret = register_netdev(ndev);

  /*  netif_start_queue(ndev);*/
  /* ndev->needs_free_netdev = true; */

  /* set watchdog timeout */

  return ret;
}

static int esp_add_interface(struct esp_adapter *adapter, uint8_t if_type, uint8_t if_num, char *name)
{
  struct net_driver_s *ndev = NULL;
  struct esp_private *priv = NULL;
  int ret = 0;

  ndev = alloc_netdev_mqs(sizeof(struct esp_private), name,
                          NET_NAME_ENUM, ether_setup, 1, 1);

  if (!ndev)
    {
      wlerr( "%s: alloc failed\n", __func__);
      return -ENOMEM;
    }

  priv = netdev_priv(ndev);

  /* Init priv */

  ret = esp_init_priv(priv, ndev, if_type, if_num);
  if (ret)
    {
      wlerr( "%s: Init priv failed\n", __func__);
      goto error_exit;
    }

  ret = esp_init_net_dev(ndev, priv);
  if (ret)
    {
      wlerr( "%s: Init netdev failed\n", __func__);
      goto error_exit;
    }

  return ret;

error_exit:
  free_netdev(ndev);
  return ret;
}

static void esp_remove_network_interfaces(struct esp_adapter *adapter)
{
  if (adapter->priv[0]->ndev)
    {
      netif_stop_queue(adapter->priv[0]->ndev);
      unregister_netdev(adapter->priv[0]->ndev);
      free_netdev(adapter->priv[0]->ndev);
    }

  if (adapter->priv[1]->ndev)
    {
      netif_stop_queue(adapter->priv[1]->ndev);
      unregister_netdev(adapter->priv[1]->ndev);
      free_netdev(adapter->priv[1]->ndev);
    }
}

int esp_add_card(struct esp_adapter *adapter)
{
  int ret = 0;

  if (!adapter)
    {
      wlerr( "%s: Invalid args\n", __func__);
      return -EINVAL;
    }

  stop_data = 0;

  /* Add interface STA and AP */

  ret = esp_add_interface(adapter, ESP_STA_IF, 0, "ethsta%d");
  if (ret)
    {
      wlerr( "%s: Failed to add STA\n", __func__);
      return ret;
    }

  ret = esp_add_interface(adapter, ESP_AP_IF, 0, "ethap%d");
  if (ret)
    {
      wlerr( "%s: Failed to add AP\n", __func__);
      esp_remove_network_interfaces(adapter);
    }

  return ret;
}

int esp_remove_card(struct esp_adapter *adapter)
{
  stop_data = 1;

  if (!adapter)
    {
      return 0;
    }

  /* Flush workqueues */

  if (adapter->if_rx_workqueue)
    {
      work_cancel(LPWORK, adapter->if_rx_workqueue);
    }

  if (adapter->tx_workqueue)
    {
      work_cancel(LPWORK, adapter->tx_workqueue);
    }

  esp_remove_network_interfaces(adapter);

  adapter->priv[0] = NULL;
  adapter->priv[1] = NULL;

  return 0;
}

static void esp_if_rx_work(void *arg)
{
  /* read inbound packet and forward it to network/serial interface */

  esp_get_packets(&adapter);
}

static void deinit_adapter(void)
{
  if (adapter.if_rx_workqueue)
    {
      work_cancel(LPWORK, adapter.if_rx_workqueue);
    }

  if (adapter.tx_workqueue)
    {
      work_cancel(LPWORK, adapter.tx_workqueue);
    }
}

static void esp_reset(void)
{
  adapter.lower->reset(TRUE);
  udelay(1000);
  adapter.lower->reset(FALSE);
  udelay(1000);
}

static struct esp_adapter * init_adapter(struct esp_hosted_fg_lower_s *lower)
{
  memset(&adapter, 0, sizeof(adapter));

  adapter.lower = lower;

  /* Prepare interface RX work */

  if (work_available(adapter.if_rx_workqueue))
    {
      work_queue(LPWORK, adapter.if_rx_workqueue, esp_if_rx_work, NULL, 0);
    }

  adapter.if_rx_work = esp_if_rx_work;

  return &adapter;
}

FAR int esp_hosted_fg_register(FAR struct spi_dev_s *spi,
                               FAR const struct esp_hosted_fg_lower_s *lower)
{
  int ret = 0;
  struct esp_adapter *adapter = NULL;

  /* Init adapter */

  adapter = init_adapter(lower);
  if (!adapter)
    {
      return -EFAULT;
    }

  /* Reset ESP, Clean start ESP */

  esp_reset();

  /* Init transport layer */

  ret = esp_init_interface_layer(spi, adapter);

  if (ret != 0)
    {
      deinit_adapter();
    }

  return ret;
}
