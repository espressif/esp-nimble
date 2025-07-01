/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include "os/os.h"
#include "mem/mem.h"
#include "nimble/hci_common.h"
#include "host/ble_hs_hci.h"
#include "ble_hs_priv.h"

#if MYNEWT_VAL(BLE_ISO)
int
ble_hs_hci_iso_disconnect(uint16_t cis_handle, uint8_t reason)
{
    struct ble_hci_lc_disconnect_cp cmd;

    cmd.conn_handle = htole16(cis_handle);
    cmd.reason = reason;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LINK_CTRL,
                                        BLE_HCI_OCF_DISCONNECT_CMD),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_read_local_supp_codec(uint8_t *rsp_buf, uint8_t rsp_len)
{
    if (rsp_buf == NULL || rsp_len < 2) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_INFO_PARAMS,
                                        BLE_HCI_OCF_IP_RD_LOCAL_SUPP_CODEC),
                             NULL, 0, rsp_buf, rsp_len);
}

int
ble_hs_hci_read_local_supp_codec_caps(uint8_t coding_fmt, uint16_t company_id,
                                      uint16_t vs_codec_id, uint8_t logical_transport_type,
                                      uint8_t direction, uint8_t *rsp_buf, uint8_t rsp_len)
{
    struct ble_hci_ip_rd_local_supp_codec_caps_cp cmd;

    if (rsp_buf == NULL || rsp_len == 0) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd.coding_fmt = coding_fmt;
    cmd.company_id = company_id;
    cmd.vs_codec_id = vs_codec_id;
    cmd.logical_tpt_type = logical_transport_type;
    cmd.direction = direction;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_INFO_PARAMS,
                                        BLE_HCI_OCF_IP_RD_LOCAL_SUPP_CODEC_CAPS),
                             &cmd, sizeof(cmd), rsp_buf, rsp_len);
}

int
ble_hs_hci_read_local_supp_controller_delay(uint8_t coding_fmt, uint16_t company_id,
                                            uint16_t vs_codec_id, uint8_t logical_transport_type,
                                            uint8_t direction,
                                            uint8_t codec_cfg_len, uint8_t *codec_cfg,
                                            uint32_t *min_controller_delay,
                                            uint32_t *max_controller_delay)
{
    struct ble_hci_ip_rd_local_supp_controller_delay_cp cmd;
    struct ble_hci_ip_rd_local_supp_controller_delay_rp rsp;
    int rc;

    if ((codec_cfg_len && codec_cfg == NULL) ||
        min_controller_delay == NULL ||
        max_controller_delay == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd.coding_fmt = coding_fmt;
    cmd.company_id = company_id;
    cmd.vs_codec_id = vs_codec_id;
    cmd.logical_tpt_type = logical_transport_type;
    cmd.direction = direction;
    cmd.codec_cfg_len = codec_cfg_len;
    /* TODO: Use Codec Configuration */

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_INFO_PARAMS,
                                      BLE_HCI_OCF_IP_RD_LOCAL_SUPP_CONTROLLER_DELAY),
                           &cmd, sizeof(cmd), &rsp, sizeof(rsp));
    if (rc) {
        return rc;
    }

    memcpy(min_controller_delay, rsp.min_controller_delay, sizeof(rsp.min_controller_delay));
    memcpy(max_controller_delay, rsp.max_controller_delay, sizeof(rsp.max_controller_delay));

    return 0;
}

int
ble_hs_hci_cfg_data_path(uint8_t data_path_direction, uint8_t data_path_id,
                         uint8_t vs_cfg_len, uint8_t *vs_cfg)
{
    struct ble_hci_cb_cfg_data_path_cp cmd;

    if (vs_cfg_len && vs_cfg == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd.data_path_dir = data_path_direction;
    cmd.data_path_id = data_path_id;
    cmd.vs_cfg_len = vs_cfg_len;
    /* TODO: Use Vendor-Specific Configuration */

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_CTLR_BASEBAND,
                                        BLE_HCI_OCF_CB_CFG_DATA_PATH),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_read_buf_sz_v2(uint16_t *data_len, uint8_t *data_packets,
                          uint16_t *iso_data_len, uint8_t *iso_data_packets)
{
    struct ble_hci_le_rd_buf_size_v2_rp rsp;
    int rc;

    if (data_len == NULL || data_packets == NULL ||
        iso_data_len == NULL || iso_data_packets == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_RD_BUF_SIZE_V2),
                           NULL, 0, &rsp, sizeof(rsp));
    if (rc) {
        return rc;
    }

    *data_len = rsp.data_len;
    *data_packets = rsp.data_packets;
    *iso_data_len = rsp.iso_data_len;
    *iso_data_packets = rsp.iso_data_packets;

    return 0;
}

int
ble_hs_hci_read_iso_tx_sync(uint16_t conn_handle, uint16_t *packet_seq_num,
                            uint32_t *timestamp, uint32_t *timeoffset)
{
    struct ble_hci_le_read_iso_tx_sync_cp cmd;
    struct ble_hci_le_read_iso_tx_sync_rp rsp;
    int rc;

    if (packet_seq_num == NULL || timestamp == NULL || timeoffset == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd.conn_handle = htole16(conn_handle);

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_READ_ISO_TX_SYNC),
                           &cmd, sizeof(cmd), &rsp, sizeof(rsp));
    if (rc) {
        return rc;
    }

    if (le16toh(rsp.conn_handle) != conn_handle) {
        return BLE_HS_ECONTROLLER;
    }

    *packet_seq_num = rsp.packet_seq_num;
    *timestamp = rsp.tx_timestamp;
    memcpy(timeoffset, rsp.time_offset, sizeof(rsp.time_offset));

    return 0;
}

int
ble_hs_hci_set_cig_params(uint8_t cig_id, uint32_t sdu_interval_c_to_p, uint32_t sdu_interval_p_to_c,
                          uint8_t slaves_clock_accuracy, uint8_t packing, uint8_t framing,
                          uint16_t max_transport_latency_c_to_p, uint16_t max_transport_latency_p_to_c,
                          uint8_t cis_cnt, struct ble_hci_le_cis_params *cis_params,
                          uint8_t *rsp_buf, uint8_t rsp_len)
{
    struct ble_hci_le_set_cig_params_cp *cmd;
    struct ble_hci_le_cis_params *cis_param;
    uint8_t cmd_buf[sizeof(*cmd) + cis_cnt * sizeof(*cis_params)];
    uint8_t cmd_len = sizeof(*cmd);

    if (cis_cnt == 0 || cis_params == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    if (rsp_buf == NULL ||
        rsp_len < sizeof(struct ble_hci_le_set_cig_params_rp) + cis_cnt * 2) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd = (void *)cmd_buf;
    cis_param = cmd->cis;

    cmd->cig_id = cig_id;
    memcpy(cmd->sdu_interval_c_to_p, &sdu_interval_c_to_p, sizeof(cmd->sdu_interval_c_to_p));
    memcpy(cmd->sdu_interval_p_to_c, &sdu_interval_p_to_c, sizeof(cmd->sdu_interval_p_to_c));
    cmd->worst_sca = slaves_clock_accuracy;
    cmd->packing = packing;
    cmd->framing = framing;
    cmd->max_latency_c_to_p = max_transport_latency_c_to_p;
    cmd->max_latency_p_to_c = max_transport_latency_p_to_c;
    cmd->cis_count = cis_cnt;

    for (size_t i = 0; i < cis_cnt; i++) {
        cis_param->cis_id = cis_params[i].cis_id;
        cis_param->max_sdu_c_to_p = cis_params[i].max_sdu_c_to_p;
        cis_param->max_sdu_p_to_c = cis_params[i].max_sdu_p_to_c;
        cis_param->phy_c_to_p = cis_params[i].phy_c_to_p;
        cis_param->phy_p_to_c = cis_params[i].phy_p_to_c;
        cis_param->rtn_c_to_p = cis_params[i].rtn_c_to_p;
        cis_param->rtn_p_to_c = cis_params[i].rtn_p_to_c;

        cmd_len += sizeof(*cis_param);
        cis_param++;
    }

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_SET_CIG_PARAMS),
                             cmd, cmd_len, rsp_buf, rsp_len);
}

#if MYNEWT_VAL(BLE_ISO_TEST)
int
ble_hs_hci_set_cig_params_test(uint8_t cig_id, uint32_t sdu_interval_c_to_p, uint32_t sdu_interval_p_to_c,
                               uint8_t ft_c_to_p, uint8_t ft_p_to_c, uint16_t iso_interval,
                               uint8_t slaves_clock_accuracy, uint8_t packing, uint8_t framing,
                               uint8_t cis_cnt, struct ble_hci_le_cis_params_test *cis_params,
                               uint8_t *rsp_buf, uint8_t rsp_len)
{
    struct ble_hci_le_set_cig_params_test_cp *cmd;
    struct ble_hci_le_cis_params_test *cis_param;
    uint8_t cmd_buf[sizeof(*cmd) + cis_cnt * sizeof(*cis_params)];
    uint8_t cmd_len = sizeof(*cmd);

    if (cis_cnt == 0 || cis_params == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    if (rsp_buf == NULL||
        rsp_len < sizeof(struct ble_hci_le_set_cig_params_rp) + cis_cnt * 2) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd = (void *)cmd_buf;
    cis_param = cmd->cis;

    cmd->cig_id = cig_id;
    memcpy(cmd->sdu_interval_c_to_p, &sdu_interval_c_to_p, sizeof(cmd->sdu_interval_c_to_p));
    memcpy(cmd->sdu_interval_p_to_c, &sdu_interval_p_to_c, sizeof(cmd->sdu_interval_p_to_c));
    cmd->ft_c_to_p = ft_c_to_p;
    cmd->ft_p_to_c = ft_p_to_c;
    cmd->iso_interval = iso_interval;
    cmd->worst_sca = slaves_clock_accuracy;
    cmd->packing = packing;
    cmd->framing = framing;
    cmd->cis_count = cis_cnt;

    for (size_t i = 0; i < cis_cnt; i++) {
        cis_param->cis_id = cis_params[i].cis_id;
        cis_param->nse = cis_params[i].nse;
        cis_param->max_sdu_c_to_p = cis_params[i].max_sdu_c_to_p;
        cis_param->max_sdu_p_to_c = cis_params[i].max_sdu_p_to_c;
        cis_param->max_pdu_c_to_p = cis_params[i].max_pdu_c_to_p;
        cis_param->max_pdu_p_to_c = cis_params[i].max_pdu_p_to_c;
        cis_param->phy_c_to_p = cis_params[i].phy_c_to_p;
        cis_param->phy_p_to_c = cis_params[i].phy_p_to_c;
        cis_param->bn_c_to_p = cis_params[i].bn_c_to_p;
        cis_param->bn_p_to_c = cis_params[i].bn_p_to_c;

        cmd_len += sizeof(*cis_param);
        cis_param++;
    }

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_SET_CIG_PARAMS_TEST),
                             cmd_buf, cmd_len, rsp_buf, rsp_len);
}
#endif /* MYNEWT_VAL(BLE_ISO_TEST) */

int
ble_hs_hci_create_cis(uint8_t cis_cnt, const struct ble_hci_le_create_cis_params *params)
{
    struct ble_hci_le_create_cis_cp *cmd;
    struct ble_hci_le_create_cis_params *param;
    uint8_t cmd_buf[sizeof(*cmd) + cis_cnt * sizeof(*params)];
    uint8_t cmd_len = sizeof(*cmd);

    if (cis_cnt == 0 || params == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd = (void *)cmd_buf;
    param = cmd->cis;

    cmd->cis_count = cis_cnt;

    for (size_t i = 0; i < cis_cnt; i++) {
        param->cis_handle = params[i].cis_handle;
        param->conn_handle = params[i].conn_handle;

        cmd_len += sizeof(*param);
        param++;
    }

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CREATE_CIS),
                             cmd_buf, cmd_len, NULL, 0);
}

int
ble_hs_hci_remove_cig(uint8_t cig_id)
{
    struct ble_hci_le_remove_cig_cp cmd;
    struct ble_hci_le_remove_cig_rp rsp;

    cmd.cig_id = cig_id;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_REMOVE_CIG),
                             &cmd, sizeof(cmd), &rsp, sizeof(rsp));
}

int
ble_hs_hci_accept_cis_request(uint16_t cis_handle)
{
    struct ble_hci_le_accept_cis_request_cp cmd;

    cmd.conn_handle = cis_handle;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_ACCEPT_CIS_REQ),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_reject_cis_request(uint16_t cis_handle, uint8_t reason)
{
    struct ble_hci_le_reject_cis_request_cp cmd;

    cmd.conn_handle = cis_handle;
    cmd.reason = reason;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_REJECT_CIS_REQ),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_create_big(uint8_t big_handle, uint8_t adv_handle, uint8_t num_bis,
                      uint32_t sdu_interval, uint16_t max_sdu,
                      uint16_t max_transport_latency, uint8_t rtn, uint8_t phy,
                      uint8_t packing, uint8_t framing, uint8_t encryption,
                      uint8_t broadcast_code[16])
{
    struct ble_hci_le_create_big_cp cmd;

    if (num_bis == 0 || (encryption && broadcast_code == NULL)) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd.big_handle = big_handle;
    cmd.adv_handle = adv_handle;
    cmd.num_bis = num_bis;
    memcpy(cmd.sdu_interval, &sdu_interval, sizeof(cmd.sdu_interval));
    cmd.max_sdu = max_sdu;
    cmd.max_transport_latency = max_transport_latency;
    cmd.rtn = rtn;
    cmd.phy = phy;
    cmd.packing = packing;
    cmd.framing = framing;
    cmd.encryption = encryption;
    if (encryption) {
        memcpy(cmd.broadcast_code, broadcast_code, sizeof(cmd.broadcast_code));
    }

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CREATE_BIG),
                             &cmd, sizeof(cmd), NULL, 0);
}

#if MYNEWT_VAL(BLE_ISO_TEST)
int
ble_hs_hci_create_big_test(uint8_t big_handle, uint8_t adv_handle, uint8_t num_bis,
                           uint32_t sdu_interval, uint16_t iso_interval, uint8_t nse,
                           uint16_t max_sdu, uint16_t max_pdu, uint8_t phy,
                           uint8_t packing, uint8_t framing, uint8_t bn, uint8_t irc,
                           uint8_t pto, uint8_t encryption, uint8_t broadcast_code[16])
{
    struct ble_hci_le_create_big_test_cp cmd;

    if (num_bis == 0 || (encryption && broadcast_code == NULL)) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd.big_handle = big_handle;
    cmd.adv_handle = adv_handle;
    cmd.num_bis = num_bis;
    memcpy(cmd.sdu_interval, &sdu_interval, sizeof(cmd.sdu_interval));
    cmd.iso_interval = iso_interval;
    cmd.nse = nse;
    cmd.max_sdu = max_sdu;
    cmd.max_pdu = max_pdu;
    cmd.phy = phy;
    cmd.packing = packing;
    cmd.framing = framing;
    cmd.bn = bn;
    cmd.irc = irc;
    cmd.pto = pto;
    cmd.encryption = encryption;
    if (encryption) {
        memcpy(cmd.broadcast_code, broadcast_code, sizeof(cmd.broadcast_code));
    }

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CREATE_BIG_TEST),
                             &cmd, sizeof(cmd), NULL, 0);
}
#endif /* MYNEWT_VAL(BLE_ISO_TEST) */

int
ble_hs_hci_terminate_big(uint8_t big_handle, uint8_t reason)
{
    struct ble_hci_le_terminate_big_cp cmd;

    cmd.big_handle = big_handle;
    cmd.reason = reason;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_TERMINATE_BIG),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_big_create_sync(uint8_t big_handle, uint16_t sync_handle,
                           uint8_t encryption, uint8_t broadcast_code[16],
                           uint8_t mse, uint16_t sync_timeout,
                           uint8_t num_bis, uint8_t *bis_index)
{
    struct ble_hci_le_big_create_sync_cp *cmd;
    uint8_t cmd_buf[sizeof(*cmd) + MYNEWT_VAL(BLE_ISO_BIS_PER_BIG)];

    if ((encryption && broadcast_code == NULL) || num_bis == 0 || bis_index == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd = (void *)cmd_buf;

    cmd->big_handle = big_handle;
    cmd->sync_handle = sync_handle;
    cmd->encryption = encryption;
    if (encryption) {
        memcpy(cmd->broadcast_code, broadcast_code, 16);
    }
    cmd->mse = mse;
    cmd->sync_timeout = sync_timeout;
    cmd->num_bis = num_bis;
    memcpy(cmd->bis, bis_index, num_bis);

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_BIG_CREATE_SYNC),
                             cmd, sizeof(*cmd) + num_bis, NULL, 0);
}

int
ble_hs_hci_big_terminate_sync(uint8_t big_handle)
{
    struct ble_hci_le_big_terminate_sync_cp cmd;

    cmd.big_handle = big_handle;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_BIG_TERMINATE_SYNC),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_setup_iso_data_path(uint16_t conn_handle, uint8_t data_path_direction,
                               uint8_t data_path_id, uint8_t coding_fmt,
                               uint16_t company_id, uint16_t vs_codec_id,
                               uint32_t controller_delay, uint8_t codec_cfg_len,
                               uint8_t *codec_cfg)
{
    struct ble_hci_le_setup_iso_data_path_cp cmd;

    if (codec_cfg_len && codec_cfg == NULL) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd.conn_handle = conn_handle;
    cmd.data_path_dir = data_path_direction;
    cmd.data_path_id = data_path_id;
    cmd.codec_id[0] = coding_fmt;
    put_le16(cmd.codec_id + 1, company_id);
    put_le16(cmd.codec_id + 3, vs_codec_id);
    memcpy(cmd.controller_delay, &controller_delay, sizeof(cmd.controller_delay));
    cmd.codec_config_len = codec_cfg_len;
    /* TODO: Codec Configuration */

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_SETUP_ISO_DATA_PATH),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_remove_iso_data_path(uint16_t conn_handle, uint8_t data_path_direction)
{
    struct ble_hci_le_remove_iso_data_path_cp cmd;

    cmd.conn_handle = conn_handle;
    cmd.data_path_dir = data_path_direction;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_REMOVE_ISO_DATA_PATH),
                             &cmd, sizeof(cmd), NULL, 0);
}

#if MYNEWT_VAL(BLE_ISO_TEST)
int
ble_hs_hci_iso_transmit_test(uint16_t conn_handle, uint8_t payload_type)
{
    struct ble_hci_le_iso_transmit_test_cp cmd;

    cmd.conn_handle = conn_handle;
    cmd.payload_type = payload_type;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_ISO_TRANSMIT_TEST),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_iso_receive_test(uint16_t conn_handle, uint8_t payload_type)
{
    struct ble_hci_le_iso_receive_test_cp cmd;

    cmd.conn_handle = conn_handle;
    cmd.payload_type = payload_type;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_ISO_RECEIVE_TEST),
                             &cmd, sizeof(cmd), NULL, 0);
}

int
ble_hs_hci_iso_read_test_counters(uint16_t conn_handle, uint32_t *received_sdu_count,
                                  uint32_t *missed_sdu_count, uint32_t *failed_sdu_count)
{
    struct ble_hci_le_iso_read_test_counters_cp cmd;
    struct ble_hci_le_iso_read_test_counters_rp rsp;
    int rc;

    cmd.conn_handle = conn_handle;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_ISO_READ_TEST_COUNTERS),
                           &cmd, sizeof(cmd), &rsp, sizeof(rsp));
    if (rc) {
        return rc;
    }

    *received_sdu_count = rsp.received_sdu_count;
    *missed_sdu_count = rsp.missed_sdu_count;
    *failed_sdu_count = rsp.failed_sdu_count;

    return 0;
}

int
ble_hs_hci_iso_test_end(uint16_t conn_handle, uint32_t *received_sdu_count,
                        uint32_t *missed_sdu_count, uint32_t *failed_sdu_count)
{
    struct ble_hci_le_iso_test_end_cp cmd;
    struct ble_hci_le_iso_test_end_rp rsp;
    int rc;

    cmd.conn_handle = conn_handle;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_ISO_TEST_END),
                           &cmd, sizeof(cmd), &rsp, sizeof(rsp));
    if (rc) {
        return rc;
    }

    *received_sdu_count = rsp.received_sdu_count;
    *missed_sdu_count = rsp.missed_sdu_count;
    *failed_sdu_count = rsp.failed_sdu_count;

    return 0;
}
#endif /* MYNEWT_VAL(BLE_ISO_TEST) */

int
ble_hs_hci_read_iso_link_quality(uint16_t conn_handle, uint32_t *tx_unacked_pkts,
                                 uint32_t *tx_flushed_pkts, uint32_t *tx_last_subev_pkts,
                                 uint32_t *retransmitted_pkts, uint32_t *crc_error_pkts,
                                 uint32_t *rx_unreceived_pkts, uint32_t *duplicate_pkts)
{
    struct ble_hci_le_read_iso_link_quality_cp cmd;
    struct ble_hci_le_read_iso_link_quality_rp rsp;
    int rc;

    if (!tx_unacked_pkts || !tx_flushed_pkts ||
        !tx_last_subev_pkts || !retransmitted_pkts ||
        !crc_error_pkts || !rx_unreceived_pkts ||
        !duplicate_pkts) {
        return BLE_ERR_INV_HCI_CMD_PARMS;
    }

    cmd.conn_handle = conn_handle;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_READ_ISO_LINK_QUALITY),
                           &cmd, sizeof(cmd), &rsp, sizeof(rsp));
    if (rc) {
        return rc;
    }

    *tx_unacked_pkts = rsp.tx_unacked_pkts;
    *tx_flushed_pkts = rsp.tx_flushed_pkts;
    *tx_last_subev_pkts = rsp.tx_last_subevent_pkts;
    *retransmitted_pkts = rsp.retransmitted_pkts;
    *crc_error_pkts = rsp.crc_error_pkts;
    *rx_unreceived_pkts = rsp.rx_unreceived_pkts;
    *duplicate_pkts = rsp.duplicate_pkts;

    return 0;
}
#endif /* MYNEWT_VAL(BLE_ISO) */
