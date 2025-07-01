/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef H_BLE_HS_ISO_
#define H_BLE_HS_ISO_

#include <stdint.h>
#include "os/os.h"
#include "mem/mem.h"
#include "syscfg/syscfg.h"

#ifdef __cplusplus
extern "C" {
#endif

int ble_hs_hci_set_iso_buf_sz(uint16_t pktlen, uint16_t max_pkts);

void ble_hs_hci_add_iso_avail_pkts(uint16_t conn_handle, uint16_t delta);

int ble_hs_hci_iso_tx(uint16_t conn_handle, const uint8_t *sdu, uint16_t sdu_len,
                      bool ts_flag, uint32_t time_stamp, uint16_t pkt_seq_num);

typedef int (*ble_hs_iso_pkt_rx_fn)(const uint8_t *data, uint16_t len, void *arg);

int ble_hs_iso_pkt_rx_cb_set(void *cb);

int ble_hs_rx_iso_data(const uint8_t *data, uint16_t len, void *arg);

#ifdef __cplusplus
}
#endif

#endif /* H_BLE_HS_ISO_ */
