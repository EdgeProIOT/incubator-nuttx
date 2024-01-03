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

#ifndef __ESP_HOSTED_QUEUE_H__
#define __ESP_HOSTED_QUEUE_H__

#define ESP_HOSTED_QUEUE_SUCCESS         	0
#define ESP_HOSTED_QUEUE_ERR_UNINITALISED -1
#define ESP_HOSTED_QUEUE_ERR_MEMORY       -2

struct q_element_s
{
	void *buf;
	int buf_len;
};

/* Queue based on Linked List */

struct q_node_s
{
	void *data;
	struct q_node_s* next;
};

struct esp_hosted_queue_s
{
	struct q_node_s *front, *rear;
};

struct esp_hosted_queue_s* esp_hosted_queue_create(void);
void *esp_hosted_queue_get(struct esp_hosted_queue_s* q);
int esp_hosted_queue_put(struct esp_hosted_queue_s* q, void *data);
void esp_hosted_queue_destroy(struct esp_hosted_queue_s** q);

#endif /*__ESP_HOSTED_QUEUE_H__*/
