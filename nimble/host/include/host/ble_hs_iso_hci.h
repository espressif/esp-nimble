/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef H_BLE_HS_ISO_HCI_
#define H_BLE_HS_ISO_HCI_

#include <stdint.h>
#include "os/os.h"
#include "mem/mem.h"
#include "syscfg/syscfg.h"
#include "nimble/hci_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#if MYNEWT_VAL(BLE_ISO)
int ble_hs_hci_iso_disconnect(uint16_t cis_handle, uint8_t reason);

int ble_hs_hci_read_local_supp_codec(uint8_t *rsp_buf, uint8_t rsp_len);

int ble_hs_hci_read_local_supp_codec_caps(uint8_t coding_fmt, uint16_t company_id,
                                          uint16_t vs_codec_id, uint8_t logical_transport_type,
                                          uint8_t direction, uint8_t *rsp_buf, uint8_t rsp_len);

int ble_hs_hci_read_local_supp_controller_delay(uint8_t coding_fmt, uint16_t company_id,
                                                uint16_t vs_codec_id, uint8_t logical_transport_type,
                                                uint8_t direction,
                                                uint8_t codec_cfg_len, uint8_t *codec_cfg,
                                                uint32_t *min_controller_delay,
                                                uint32_t *max_controller_delay);

int ble_hs_hci_cfg_data_path(uint8_t data_path_direction, uint8_t data_path_id,
                             uint8_t vs_cfg_len, uint8_t *vs_cfg);

int ble_hs_hci_read_buf_sz_v2(uint16_t *data_len, uint8_t *data_packets,
                              uint16_t *iso_data_len, uint8_t *iso_data_packets);

int ble_hs_hci_read_iso_tx_sync(uint16_t conn_handle, uint16_t *packet_seq_num,
                                uint32_t *timestamp, uint32_t *timeoffset);

int ble_hs_hci_set_cig_params(uint8_t cig_id, uint32_t sdu_interval_c_to_p, uint32_t sdu_interval_p_to_c,
                              uint8_t slaves_clock_accuracy, uint8_t packing, uint8_t framing,
                              uint16_t max_transport_latency_c_to_p, uint16_t max_transport_latency_p_to_c,
                              uint8_t cis_cnt, struct ble_hci_le_cis_params *cis_params,
                              uint8_t *rsp_buf, uint8_t rsp_len);

#if MYNEWT_VAL(BLE_ISO_TEST)
int ble_hs_hci_set_cig_params_test(uint8_t cig_id, uint32_t sdu_interval_c_to_p, uint32_t sdu_interval_p_to_c,
                                   uint8_t ft_c_to_p, uint8_t ft_p_to_c, uint16_t iso_interval,
                                   uint8_t slaves_clock_accuracy, uint8_t packing, uint8_t framing,
                                   uint8_t cis_cnt, struct ble_hci_le_cis_params_test *cis_params,
                                   uint8_t *rsp_buf, uint8_t rsp_len);
#endif /* MYNEWT_VAL(BLE_ISO_TEST) */

int ble_hs_hci_create_cis(uint8_t cis_cnt, const struct ble_hci_le_create_cis_params *params);

int ble_hs_hci_remove_cig(uint8_t cig_id);

int ble_hs_hci_accept_cis_request(uint16_t cis_handle);

int ble_hs_hci_reject_cis_request(uint16_t cis_handle, uint8_t reason);

int ble_hs_hci_create_big(uint8_t big_handle, uint8_t adv_handle, uint8_t num_bis,
                          uint32_t sdu_interval, uint16_t max_sdu,
                          uint16_t max_transport_latency, uint8_t rtn, uint8_t phy,
                          uint8_t packing, uint8_t framing, uint8_t encryption,
                          uint8_t broadcast_code[16]);

#if MYNEWT_VAL(BLE_ISO_TEST)
int ble_hs_hci_create_big_test(uint8_t big_handle, uint8_t adv_handle, uint8_t num_bis,
                               uint32_t sdu_interval, uint16_t iso_interval, uint8_t nse,
                               uint16_t max_sdu, uint16_t max_pdu, uint8_t phy,
                               uint8_t packing, uint8_t framing, uint8_t bn, uint8_t irc,
                               uint8_t pto, uint8_t encryption, uint8_t broadcast_code[16]);
#endif /* MYNEWT_VAL(BLE_ISO_TEST) */

int ble_hs_hci_terminate_big(uint8_t big_handle, uint8_t reason);

int ble_hs_hci_big_create_sync(uint8_t big_handle, uint16_t sync_handle,
                               uint8_t encryption, uint8_t broadcast_code[16],
                               uint8_t mse, uint16_t sync_timeout,
                               uint8_t num_bis, uint8_t *bis_index);

int ble_hs_hci_big_terminate_sync(uint8_t big_handle);

int ble_hs_hci_setup_iso_data_path(uint16_t conn_handle, uint8_t data_path_direction,
                                   uint8_t data_path_id, uint8_t coding_fmt,
                                   uint16_t company_id, uint16_t vs_codec_id,
                                   uint32_t controller_delay, uint8_t codec_cfg_len,
                                   uint8_t *codec_cfg);

int ble_hs_hci_remove_iso_data_path(uint16_t conn_handle, uint8_t data_path_direction);

#if MYNEWT_VAL(BLE_ISO_TEST)
int ble_hs_hci_iso_transmit_test(uint16_t conn_handle, uint8_t payload_type);

int ble_hs_hci_iso_receive_test(uint16_t conn_handle, uint8_t payload_type);

int ble_hs_hci_iso_read_test_counters(uint16_t conn_handle, uint32_t *received_sdu_count,
                                      uint32_t *missed_sdu_count, uint32_t *failed_sdu_count);

int ble_hs_hci_iso_test_end(uint16_t conn_handle, uint32_t *received_sdu_count,
                            uint32_t *missed_sdu_count, uint32_t *failed_sdu_count);
#endif /* MYNEWT_VAL(BLE_ISO_TEST) */

int ble_hs_hci_read_iso_link_quality(uint16_t conn_handle, uint32_t *tx_unacked_pkts,
                                     uint32_t *tx_flushed_pkts, uint32_t *tx_last_subev_pkts,
                                     uint32_t *retransmitted_pkts, uint32_t *crc_error_pkts,
                                     uint32_t *rx_unreceived_pkts, uint32_t *duplicate_pkts);
#endif /* MYNEWT_VAL(BLE_ISO) */

#ifdef __cplusplus
}
#endif

#endif /* H_BLE_HS_ISO_HCI_ */
