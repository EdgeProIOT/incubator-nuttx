/****************************************************************************
 * drivers/wireless/esp-hosted/esp_hosted_serial_if.h
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

#ifndef __ESP_HOSTED_SERIAL_IF_H
#define __ESP_HOSTED_SERIAL_IF_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "esp_hosted_adapter.h"

#define SIZE_OF_TYPE                1
#define SIZE_OF_LENGTH              2

/*
 * The data written on serial driver file, `SERIAL_IF_FILE` from adapter.h
 * In TLV i.e. Type Length Value format, to transfer data between host and ESP32
 *  | type | length | value |
 * Types are 0x01 : for endpoint name
 *           0x02 : for data
 * length is respective value field's data length in 16 bits
 * value is actual data to be transferred
 */
uint16_t compose_tlv(uint8_t* buf, uint8_t* data, uint16_t data_length);

/* Parse the protobuf encoded data in format of tag, length and value
 * Thi will help application to decode protobuf payload and payload length
 **/
uint8_t parse_tlv(uint8_t* data, uint32_t* pro_len);

/* Open the serial driver for serial operations
 **/
int transport_pserial_open(void);

/* Close the serial driver for serial operations
 **/
int transport_pserial_close(void);

/* Send buffer with length as argument on transport as serial interface type
 **/
int transport_pserial_send(uint8_t* data, uint16_t data_length);

/* Read and return number of bytes and buffer from serial interface
 **/
uint8_t * transport_pserial_read(uint32_t *out_nbyte);
#endif