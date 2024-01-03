/****************************************************************************
 * drivers/wireless/esp-hosted/esp_hosted_queue.h
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

#include <stdio.h>
#include <stdlib.h>
#include "esp_hosted_queue.h"

/* create new node */

static struct q_node_s * new_q_node(void *data)
{
  struct q_node_s* new_node = (struct q_node_s*)malloc(sizeof(struct q_node_s));
  if (!new_node)
    {
      return NULL;
    }
  new_node->data = data;
  new_node->next = NULL;
  return new_node;
}

/* Create app queue */

struct esp_hosted_queue_s* esp_hosted_queue_create(void)
{
  struct esp_hosted_queue_s* q = (struct esp_hosted_queue_s*)malloc(sizeof(struct esp_hosted_queue_s));
  if (!q)
    {
      return NULL;
    }

  q->front = q->rear = NULL;
  return q;
}

/* Put element in app queue */

int esp_hosted_queue_put(struct esp_hosted_queue_s* q, void *data)
{
  struct q_node_s* new_node = NULL;

  if (!q)
    {
      printf("q undefined\n");
      return ESP_QUEUE_ERR_UNINITALISED;
    }

  new_node = new_q_node(data);
  if (!new_node)
    {
      printf("malloc failed in qpp_q_put\n");
      return ESP_QUEUE_ERR_MEMORY;
    }

  /* queue empty condition */

  if (q->rear == NULL)
    {
      q->front = q->rear = new_node;
      return ESP_QUEUE_SUCCESS;
    }

  q->rear->next = new_node;
  q->rear = new_node;
  return ESP_QUEUE_SUCCESS;
}

/* Get element in app queue */

void *esp_hosted_queue_get(struct esp_hosted_queue_s* q)
{
  void * data = NULL;
  struct q_node_s* temp = NULL;

  if (!q || q->front == NULL)
    {
      return NULL;
    }

  /* move front one node ahead */

  temp = q->front;

  if (!temp)
    {
      return NULL;
    }

  q->front = q->front->next;

  data = temp->data;

  free(temp);
  temp = NULL;

  /* If front is NULL, change rear also as NULL */

  if (q->front == NULL)
    {
      q->rear = NULL;
    }

  return data;
}

void esp_hosted_queue_destroy(struct esp_hosted_queue_s** q)
{
  struct q_node_s* temp = NULL;

  if (!q || !*q)
    {
      return;
    }

  while ((*q)->front)
    {

      temp = (*q)->front;
      (*q)->front = (*q)->front->next;

      free(temp);
      temp = NULL;
    }

  free(*q);
  *q = NULL;
}