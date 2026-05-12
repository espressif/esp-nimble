/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "os/os.h"
#include "sdkconfig.h"
#include "host/ble_hs.h"
#include "host/ble_hs_log.h"

struct log ble_hs_log;

#define BLE_HS_LOG_HEX_BYTES_PER_LINE 10

static void
ble_hs_log_debug_hex_chunk(const uint8_t *bytes, int len)
{
    static const char hex_chars[] = "0123456789abcdef";
    char hex_str[31] = {0}; // 10 bytes * 3 chars ("ff ") + 1 null = 31
    int offset = 0;

    if (len <= 0) {
        return;
    }

    if (len > BLE_HS_LOG_HEX_BYTES_PER_LINE) {
        len = BLE_HS_LOG_HEX_BYTES_PER_LINE;
    }

    for (int i = 0; i < len; i++) {
        hex_str[offset++] = hex_chars[bytes[i] >> 4];
        hex_str[offset++] = hex_chars[bytes[i] & 0x0f];
        hex_str[offset++] = ' ';
    }

    BLE_HS_LOG(DEBUG, "%s", hex_str);
}

void
ble_hs_log_mbuf(const struct os_mbuf *om)
{
    uint8_t buf[BLE_HS_LOG_HEX_BYTES_PER_LINE];
    int total_len;
    int offset;
    int chunk_len;

    if (om == NULL) {
        return;
    }

    total_len = OS_MBUF_PKTLEN(om);

    for (offset = 0; offset < total_len; offset += BLE_HS_LOG_HEX_BYTES_PER_LINE) {
        chunk_len = total_len - offset;
        if (chunk_len > BLE_HS_LOG_HEX_BYTES_PER_LINE) {
            chunk_len = BLE_HS_LOG_HEX_BYTES_PER_LINE;
        }

        os_mbuf_copydata(om, offset, chunk_len, buf);
        ble_hs_log_debug_hex_chunk(buf, chunk_len);
    }
}

void
ble_hs_log_flat_buf(const void *data, int len)
{
    int offset;
    int chunk_len;
    const uint8_t *u8ptr;

    if (data == NULL) {
        return;
    }

    u8ptr = data;
    for (offset = 0; offset < len; offset += BLE_HS_LOG_HEX_BYTES_PER_LINE) {
        chunk_len = len - offset;
        if (chunk_len > BLE_HS_LOG_HEX_BYTES_PER_LINE) {
            chunk_len = BLE_HS_LOG_HEX_BYTES_PER_LINE;
        }

        ble_hs_log_debug_hex_chunk(u8ptr + offset, chunk_len);
    }
}
