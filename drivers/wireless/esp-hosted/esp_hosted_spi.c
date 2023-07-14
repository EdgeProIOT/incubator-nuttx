/****************************************************************************
 * drivers/wireless/esp-hosted/esp_hosted_spi.c
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

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>

#include "esp_hosted_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ESP_HOSTED_SPIFREQUENCY
#  define CONFIG_ESP_HOSTED_SPIFREQUENCY    (10000000)
#endif

/* ESP in sdkconfig has CONFIG_IDF_FIRMWARE_CHIP_ID entry.
 * supported values of CONFIG_IDF_FIRMWARE_CHIP_ID are - */
#define ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED (0xff)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32        (0x0)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S2      (0x2)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C3      (0x5)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32S3      (0x9)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C2      (0xC)
#define ESP_PRIV_FIRMWARE_CHIP_ESP32C6      (0xD)


static struct esp_hosted_spi_s spi_priv;
static char hardware_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int handshake_interrupt_handler(int irq, FAR void *context,
                                       FAR void *arg)
{
  FAR struct esp_hosted_spi_s *priv = (FAR struct esp_hosted_spi_s *)arg;
  int ret;

  if (work_available(&priv->work))
    {
      ret = work_queue(LPWORK, &priv->work, spi_work, priv, 0);
      if (ret != 0)
        {
          nerr("ERROR: Failed to queue work: %d\n", ret);
        }
    }
}

static int data_ready_interrupt_handler(int irq, FAR void *context,
                                        FAR void *arg)
{
  FAR struct esp_hosted_spi_s *priv = (FAR struct esp_hosted_spi_s *)arg;
  int ret;

  if (work_available(&priv->work))
    {
      ret = work_queue(LPWORK, &priv->work, spi_work, priv, 0);
      if (ret != 0)
        {
          nerr("ERROR: Failed to queue work: %d\n", ret);
        }
    }
}

static struct iob_s *read_packet(struct esp_hosted_spi_s *priv)
{

}

static int write_packet(struct esp_hosted_spi_s *priv, struct iob_s *iob)
{

}

void process_capabilities(uint8_t cap)
{

}

static int process_init_event(uint8_t *evt_buf, uint8_t len)
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
      ninfo("EVENT: %d\n", *pos);
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
          hardware_type = *(pos + 2);
        }
      else
        {
          nwarn("Unsupported tag in event\n");
        }
      
      pos += (tag_len + 2);
      len_left -= (tag_len + 2);
    }
  
  if ((hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S2) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C2) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C3) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32C6) &&
      (hardware_type != ESP_PRIV_FIRMWARE_CHIP_ESP32S3))
    {
      ninfo("ESP board type is not mentioned, ignoring [%d]\n",
            hardware_type);
      hardware_type = ESP_PRIV_FIRMWARE_CHIP_UNRECOGNIZED;
      return -1;
    }

  return 0;
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
      ninfo("Received INIT event from ESP32 peripheral\n");
      ret = process_init_event(event->event_data, event->event_len);
    }
  else
    {
      nwarn("Drop unknown event\n");
    }
}

static int process_rx_buf(void)
{

}

static void spi_work(void *arg)
{

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp_hosted_spi_init(FAR struct spi_dev_s *spi)
{
  int ret = 0;
  uint8_t prio_q_idx = 0;

  for (prio_q_idx = 0; prio_q_idx < MAX_PRIORITY_QUEUES; prio_q_idx++)
    {
      IOB_QINIT(&spi_priv.tx_q[prio_q_idx]);
      IOB_QINIT(&spi_priv.rx_q[prio_q_idx]);
    }

  spi_priv.spi = spi;

  nxmutex_init(&spi_priv.lock);

  SPI_LOCK(spi, true);

  SPI_SETMODE(spi, SPIDEV_MODE2);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_ESP_HOSTED_SPIFREQUENCY);

  SPI_LOCK(spi, false);

  lower->attach_dready(data_ready_interrupt_handler, &spi_priv);
  lower->attach_handshake(handshake_interrupt_handler, &spi_priv);

  /* Reset and Unreset ESP module */

  lower->reset(true);
  up_mdelay(1);
  lower->reset(false);
  up_mdelay(100);

  return ret;
}