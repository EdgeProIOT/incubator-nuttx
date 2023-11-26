/****************************************************************************
 * drivers/wireless/ieee80211/esp_hosted_fg/esp_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/spi/spi.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/timer.h>
#include "esp_spi.h"
#include "esp_if.h"
#include "esp_api.h"
#include "esp_bt_api.h"
#ifdef CONFIG_SUPPORT_ESP_SERIAL
#include "esp_serial.h"
#endif
#include "esp_kernel_port.h"
#include "esp_stats.h"

#define SPI_INITIAL_CLK_MHZ     10
#define NUMBER_1M               1000000
#define TX_MAX_PENDING_COUNT    100
#define TX_RESUME_THRESHOLD     (TX_MAX_PENDING_COUNT/5)

/* ESP in sdkconfig has CONFIG_IDF_FIRMWARE_CHIP_ID entry.
 * supported values of CONFIG_IDF_FIRMWARE_CHIP_ID are - */
#define ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED (0xff)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32        (0x0)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S2      (0x2)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C3      (0x5)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S3      (0x9)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C2      (0xC)

static struct sk_buff * read_packet(struct esp_adapter *adapter);
static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb);
static void spi_exit(void);
static void adjust_spi_clock(uint8_t spi_clk_mhz);

volatile uint8_t data_path = 0;
static struct esp_spi_context spi_context;
static char hardware_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
static atomic_t tx_pending;

static struct esp_if_ops if_ops = {
  .read  = read_packet,
  .write = write_packet,
};

static mutex_t spi_lock = NXMUTEX_INITIALIZER;

static void open_data_path(void)
{
  atomic_set(&tx_pending, 0);
  usleep(200000);
  data_path = OPEN_DATAPATH;
}

static void close_data_path(void)
{
  data_path = CLOSE_DATAPATH;
  usleep(200000);
}

static int spi_data_ready_interrupt_handler(int irq, FAR void *context, FAR void *arg)
{
  /* ESP peripheral has queued buffer for transmission */
  if (work_available(spi_context.spi_workqueue))
    {
      work_queue(LPWORK, spi_context.spi_workqueue, &spi_context.spi_work, NULL, 0);
    }

  return IRQ_HANDLED;
 }

static int spi_interrupt_handler(int irq, FAR void *context, FAR void *arg)
{
  /* ESP peripheral is ready for next SPI transaction */
  if (work_available(spi_context.spi_workqueue))
    {
      work_queue(LPWORK, spi_context.spi_workqueue, &spi_context.spi_work, NULL, 0);
    }

  return IRQ_HANDLED;
}

static struct sk_buff * read_packet(struct esp_adapter *adapter)
{
  struct esp_spi_context *context;
  struct sk_buff *skb = NULL;

  if (!data_path)
    {
      return NULL;
    }

  if (!adapter || !adapter->if_context)
    {
      wlerr("%s: Invalid args\n", __func__);
      return NULL;
    }

  context = adapter->if_context;

  if (context->esp_spi_dev)
    {
      skb = skb_dequeue(&(context->rx_q[PRIO_Q_SERIAL]));
      if (!skb)
        {
          skb = skb_dequeue(&(context->rx_q[PRIO_Q_BT]));
        }
      if (!skb)
        {
          skb = skb_dequeue(&(context->rx_q[PRIO_Q_OTHERS]));
        }
    }
  else
    {
      wlerr("%s: Invalid args\n", __func__);
      return NULL;
    }

  return skb;
}

static int write_packet(struct esp_adapter *adapter, struct sk_buff *skb)
{
  uint32_t max_pkt_size = SPI_BUF_SIZE;
  struct esp_payload_header *payload_header = (struct esp_payload_header *) skb->data;

  if (!adapter || !adapter->if_context || !skb || !skb->data || !skb->len)
    {
      wlerr("%s: Invalid args\n", __func__);
      dev_kfree_skb(skb);
      return -EINVAL;
    }

  if (skb->len > max_pkt_size)
    {
      wlerr("%s: Drop pkt of len[%u] > max spi transport len[%u]\n",
          __func__, skb->len, max_pkt_size);
      dev_kfree_skb(skb);
      return -EPERM;
    }

  if (!data_path)
    {
      dev_kfree_skb(skb);
      return -EPERM;
    }


  /* Enqueue SKB in tx_q */
  if (payload_header->if_type == ESP_SERIAL_IF)
    {
      skb_queue_tail(&spi_context.tx_q[PRIO_Q_SERIAL], skb);
    }
  else if (payload_header->if_type == ESP_HCI_IF)
    {
      skb_queue_tail(&spi_context.tx_q[PRIO_Q_BT], skb);
    }
  else
    {
      if (atomic_read(&tx_pending) >= TX_MAX_PENDING_COUNT)
        {
          esp_tx_pause();
          dev_kfree_skb(skb);
          if (work_available(spi_context.spi_workqueue))
            {
              work_queue(LPWORK, spi_context.spi_workqueue, &spi_context.spi_work, NULL, 0);
            }
          return -EBUSY;
        }
      skb_queue_tail(&spi_context.tx_q[PRIO_Q_OTHERS], skb);
      atomic_inc(&tx_pending);
    }

  if (work_available(spi_context.spi_workqueue))
    {
      work_queue(LPWORK, spi_context.spi_workqueue, &spi_context.spi_work, NULL, 0);
    }

  return 0;
}


int process_init_event(uint8_t *evt_buf, uint8_t len)
{
  uint8_t len_left = len, tag_len;
  uint8_t *pos;

  if (!evt_buf)
    {
      return -1;
    }

  pos = evt_buf;

  while (len_left)
    {
      tag_len = *(pos + 1);
      wlinfo("EVENT: %d\n", *pos);
      if (*pos == ESP_PRIV_CAPABILITY)
        {
          process_capabilities(*(pos + 2));
        }
      else if (*pos == ESP_PRIV_SPI_CLK_MHZ)
        {
          adjust_spi_clock(*(pos + 2));
        }
      else if (*pos == ESP_PRIV_FIRMWARE_CHIP_ID)
        {
          hardware_type = *(pos+2);
        }
      else
        {
          wlwarn("Unsupported tag in event");
        }
      pos += (tag_len+2);
      len_left -= (tag_len+2);
    }
  if ((hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S2) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C2) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C3) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S3))
    {
      wlinfo("ESP board type is not mentioned, ignoring [%d]\n", hardware_type);
      hardware_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
      return -1;
    }

  return 0;
}


static int process_rx_buf(struct sk_buff *skb)
{
  struct esp_payload_header *header;
  uint16_t len = 0;
  uint16_t offset = 0;

  if (!skb)
    {
      return -EINVAL;
    }

  header = (struct esp_payload_header *) skb->data;

  if (header->if_type >= ESP_MAX_IF)
    {
      return -EINVAL;
    }

  offset = header->offset;

  /* Validate received SKB. Check len and offset fields */
  
  if (offset != sizeof(struct esp_payload_header))
    {
      return -EINVAL;
    }

  len = header->len;
  if (!len)
    {
      return -EINVAL;
    }

  len += sizeof(struct esp_payload_header);

  if (len > SPI_BUF_SIZE)
    {
      return -EINVAL;
    }

  /* Trim SKB to actual size */
  
  skb_trim(skb, len);

  if (!data_path)
    {
      return -EPERM;
    }

  /* enqueue skb for read_packet to pick it */
  
  if (header->if_type == ESP_SERIAL_IF)
    {
      skb_queue_tail(&spi_context.rx_q[PRIO_Q_SERIAL], skb);
    }
  else if (header->if_type == ESP_HCI_IF)
    {
      skb_queue_tail(&spi_context.rx_q[PRIO_Q_BT], skb);
    }
  else
    {
      skb_queue_tail(&spi_context.rx_q[PRIO_Q_OTHERS], skb);
    }

  /* indicate reception of new packet */
  
  esp_process_new_packet_intr(spi_context.adapter);

  return 0;
}

static void esp_spi_work(void *arg)
{
  uint8_t *tx_buf;
  uint8_t *rx_buf;
  struct sk_buff *tx_skb = NULL, *rx_skb = NULL;
  int ret = 0;
  volatile int trans_ready, rx_pending;

  nxmutex_lock(&spi_lock);

  trans_ready = spi_context.adapter->lower->read(ESP_IRQ_HANDSHAKE);
  rx_pending = spi_context.adapter->lower->read(ESP_IRQ_DATA_READY);

  if (trans_ready)
    {
      if (data_path)
        {
          tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_SERIAL]);
          if (!tx_skb)
            {
              tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_BT]);
            }
      
          if (!tx_skb)
            {
              tx_skb = skb_dequeue(&spi_context.tx_q[PRIO_Q_OTHERS]);
            }
          
          if (tx_skb)
            {
              if (atomic_read(&tx_pending))
                {
                  atomic_dec(&tx_pending);
                }

              if (atomic_read(&tx_pending) < TX_RESUME_THRESHOLD)
                {
                  esp_tx_resume();
                }
            }
        }

      if (rx_pending || tx_skb)
        {
          /* Setup and execute SPI transaction
           *   Tx_buf: Check if tx_q has valid buffer for transmission,
           *     else keep it blank
           *
           *   Rx_buf: Allocate memory for incoming data. This will be freed
           *    immediately if received buffer is invalid.
           *    If it is a valid buffer, upper layer will free it.
           * */

          /* Configure TX buffer if available */

          if (tx_skb)
            {
              tx_buf = tx_skb->data;
            } 
            else
            {
              tx_skb = esp_alloc_skb(SPI_BUF_SIZE);
              tx_buf = skb_put(tx_skb, SPI_BUF_SIZE);
              memset((void*)tx_buf, 0, SPI_BUF_SIZE);
            }

          /* Configure RX buffer */

          rx_skb = esp_alloc_skb(SPI_BUF_SIZE);
          rx_buf = skb_put(rx_skb, SPI_BUF_SIZE);

          memset(rx_buf, 0, SPI_BUF_SIZE);

          SPI_SELECT(spi_context.esp_spi_dev, SPIDEV_WIRELESS(0), TRUE);
          SPI_EXCHANGE(spi_context.esp_spi_dev, reg, val, 2);
          SPI_SELECT(spi_context.esp_spi_dev, SPIDEV_WIRELESS(0), FALSE);

          /* Free rx_skb if received data is not valid */
          if (process_rx_buf(rx_skb))
            {
              dev_kfree_skb(rx_skb);
            }

          if (tx_skb)
            {
              dev_kfree_skb(tx_skb);
            }
        }
    }

  nxmutex_unlock(&spi_lock);
}

static int spi_dev_init(int spi_clk_mhz)
{
  SPI_LOCK(spi_context.esp_spi_dev, true);
  SPI_SETMODE(spi_context.esp_spi_dev, SPIDEV_MODE2);
  SPI_SETBITS(spi_context.esp_spi_dev, 8);
  SPI_SETFREQUENCY(spi_context.esp_spi_dev, spi_clk_mhz * NUMBER_1M);
  SPI_LOCK(spi_context.esp_spi_dev, false);

  spi_context.adapter->lower->attach(ESP_IRQ_HANDSHAKE, spi_interrupt_handler, NULL);
  spi_context.adapter->lower->attach(ESP_IRQ_DATA_READY, spi_data_ready_interrupt_handler, NULL);
  spi_context.adapter->lower->enable(ESP_IRQ_HANDSHAKE);
  spi_context.adapter->lower->enable(ESP_IRQ_DATA_READY);

  open_data_path();

  return 0;
}

static int spi_reinit_spidev(int spi_clk_mhz)
{
  spi_context.adapter->lower->disable(ESP_IRQ_HANDSHAKE);
  spi_context.adapter->lower->disable(ESP_IRQ_DATA_READY);
  close_data_path();

  return spi_dev_init(spi_clk_mhz);
}

static int spi_init(void)
{
  int status = 0;
  uint8_t prio_q_idx = 0;

  spi_context.spi_work = esp_spi_work;

  for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++)
    {
      skb_queue_head_init(&spi_context.tx_q[prio_q_idx]);
      skb_queue_head_init(&spi_context.rx_q[prio_q_idx]);
    }

  status = spi_dev_init(SPI_INITIAL_CLK_MHZ);
  if (status)
    {
      spi_exit();
      wlerr("Failed Init SPI device\n");
      return status;
    }

#ifdef CONFIG_SUPPORT_ESP_SERIAL
  status = esp_serial_init((void *) spi_context.adapter);
  if (status != 0)
    {
      spi_exit();
      wlerr("Error initialising serial interface\n");
      return status;
    }
#endif

  status = esp_add_card(spi_context.adapter);
  if (status)
    {
      spi_exit();
      wlerr("Failed to add card\n");
      return status;
    }

    usleep(200000);

    return status;
}

static void spi_exit(void)
{
  uint8_t prio_q_idx = 0;

  spi_context.adapter->lower->disable(ESP_IRQ_HANDSHAKE);
  spi_context.adapter->lower->disable(ESP_IRQ_DATA_READY);
  close_data_path();
  usleep(200000);

  for (prio_q_idx=0; prio_q_idx<MAX_PRIORITY_QUEUES; prio_q_idx++)
    {
      skb_queue_purge(&spi_context.tx_q[prio_q_idx]);
      skb_queue_purge(&spi_context.rx_q[prio_q_idx]);
    }

  if (work_available(spi_context.spi_workqueue))
    {
      work_cancel(LPWORK, spi_context.spi_workqueue);
      spi_context.spi_workqueue = NULL;
    }

#ifdef CONFIG_SUPPORT_ESP_SERIAL
  esp_serial_cleanup();
#endif

  esp_remove_card(spi_context.adapter);

  if (spi_context.adapter->hcidev)
    {
      esp_deinit_bt(spi_context.adapter);
    }

  memset(&spi_context, 0, sizeof(spi_context));
}

static void adjust_spi_clock(uint8_t spi_clk_mhz)
{
  if ((spi_clk_mhz) && (spi_clk_mhz != SPI_INITIAL_CLK_MHZ))
    {
      wlinfo("ESP Reconfigure SPI CLK to %u MHz\n",spi_clk_mhz);

      if (spi_reinit_spidev(spi_clk_mhz))
        {
          wlerr("Failed to reinit SPI device\n");
          spi_exit();
          return;
        }
    }
}

int esp_init_interface_layer(struct spi_dev_s *spi,
                             struct esp_adapter *adapter)
{
  if (!spi || !adapter)
    {
      return -EINVAL;
    }

  memset(&spi_context, 0, sizeof(spi_context));

  adapter->if_context = &spi_context;
  adapter->if_ops = &if_ops;
  adapter->if_type = ESP_IF_TYPE_SPI;
  
  spi_context.esp_spi_dev = spi;
  spi_context.adapter = adapter;

  return spi_init();
}

void esp_deinit_interface_layer(void)
{
  spi_exit();
}
