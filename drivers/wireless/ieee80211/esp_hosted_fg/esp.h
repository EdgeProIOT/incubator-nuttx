/****************************************************************************
 * drivers/wireless/ieee80211/esp_hosted_fg/esp.h
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

enum esp_irq
{
  ESP_IRQ_HANDSHAKE = 0,
  ESP_IRQ_DATA_READY
};

struct esp_hosted_fg_lower_s
{
  int  (*attach)(int irq, xcpt_t handler, FAR void *arg);
  void (*enable)(int irq);
  void (*disable)(int irq);
  int  (*read)(int irq);
  void (*reset)(bool);
};

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

  struct esp_hosted_fg_lower_s *lower;
};


struct esp_private
{
  struct esp_adapter      *adapter;
  struct net_driver_s     *ndev;
  uint8_t                 link_state;
  uint8_t                 mac_address[6];
  uint8_t                 if_type;
  uint8_t                 if_num;
};

typedef struct
{
  volatile int counter;
} atomic_t;

/* stdio.h Wrapper End */

static int uxcriticalnesting = 0;

/* Critical Operation Start */

inline void save_and_cli(void)
{
  enter_critical_section();
  uxcriticalnesting++;
}

inline void restore_flags(void)
{
  ASSERT(uxcriticalnesting);
  uxcriticalnesting--;
  if (uxcriticalnesting == 0)
    {
      leave_critical_section(0);
    }
}

inline void atomic_set(atomic_t *v, int i)
{
  v->counter = i;
}

inline int atomic_read(atomic_t *v)
{
  return v->counter;
}

inline void atomic_add(atomic_t *v, int i)
{
  save_and_cli();
  v->counter += i;
  restore_flags();
}

inline void atomic_sub(atomic_t *v, int i)
{
  save_and_cli();
  v->counter -= i;
  restore_flags();
}

inline void atomic_inc(atomic_t *v)
{
  atomic_add(v, 1);
}

inline void atomic_dec(atomic_t *v)
{
  atomic_sub(v, 1);
}

inline int atomic_add_return(atomic_t *v, int i)
{
  int temp;
  save_and_cli();
  temp = v->counter;
  temp += i;
  v->counter = temp;
  restore_flags();
  return temp;
}

inline int atomic_sub_return(atomic_t *v, int i)
{
  int temp;
  save_and_cli();
  temp = v->counter;
  temp -= i;
  v->counter = temp;
  restore_flags();
  return temp;
}

inline int atomic_inc_return(atomic_t *v)
{
  return atomic_add_return(v, 1);
}

inline int atomic_dec_return(atomic_t *v)
{
  return atomic_sub_return(v, 1);
}

inline int atomic_dec_and_test(atomic_t *v)
{
  return atomic_dec_return(v) == 0;
}

struct esp_skb_cb
{
  struct esp_private      *priv;
};

struct sk_buff_head
{
  struct list_head        *next;
  struct list_head        *prev;
  unsigned int            qlen;
};

struct sk_buff
{
  struct sk_buff          *next;
  struct sk_buff          *prev;
  struct sk_buff_head     *list;
  unsigned char           *head;
  unsigned char           *data;
  unsigned char           *tail;
  unsigned char           *end;
  void                    *dev;
  unsigned int            len;
  int                     dyalloc_flag;
};

unsigned char *skb_put(struct sk_buff *skb, unsigned int len);
unsigned char *skb_pull(struct sk_buff *skb, unsigned int len);

FAR int esp_hosted_fg_register(FAR struct spi_dev_s *dev,
                        FAR const struct esp_hosted_fg_lower_s *lower);
#endif
