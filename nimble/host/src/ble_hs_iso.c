/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <string.h>
#include <assert.h>
#include "os/os.h"
#include "mem/mem.h"
#include "nimble/hci_common.h"
#include "host/ble_hs_hci.h"
#include "ble_hs_priv.h"
#include "bt_osi_mem.h"
#include "host/ble_hs_iso.h"

#if MYNEWT_VAL(BLE_ISO)

_Static_assert((MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL) &&
                MYNEWT_VAL(BLE_ISO_NON_STD_FLOW_CTRL)) == 0,
                "Only one of standard or non-standard can be enabled");

#define BLE_HCI_ISO_PB_FIRST_FRAG           0
#define BLE_HCI_ISO_PB_CONT_FRAG            1
#define BLE_HCI_ISO_PB_COMP_SDU             2
#define BLE_HCI_ISO_PB_LAST_FRAG            3

#define BLE_HCI_ISO_DATA_HDR_SZ             4
#define BLE_HCI_ISO_DATA_LOAD_TS_SZ         4
#define BLE_HCI_ISO_DATA_LOAD_HDR_SZ        4

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

static uint16_t ble_hs_iso_buf_sz;
static uint8_t ble_hs_iso_max_pkts;

#if MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL)
/* Number of available ISO transmit buffers on the controller.
 * It must only be accessed while the host mutex is locked.
 */
static uint16_t ble_hs_iso_avail_pkts;
#endif /* MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL) */

int
ble_hs_hci_set_iso_buf_sz(uint16_t pktlen, uint16_t max_pkts)
{
    if (pktlen == 0 || max_pkts == 0) {
        return BLE_HS_EINVAL;
    }

    ble_hs_iso_buf_sz = pktlen;
    ble_hs_iso_max_pkts = max_pkts;

#if MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL)
    ble_hs_iso_avail_pkts = max_pkts;
#endif /* MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL) */

    BLE_HS_LOG(INFO, "ISO Flow Control:");
    BLE_HS_LOG(INFO, "          Length: %u", pktlen);
    BLE_HS_LOG(INFO, "           Count: %u", max_pkts);
    BLE_HS_LOG(INFO, "          Status: ");
    if (MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL)) {
        BLE_HS_LOG(INFO, "%s", "Standard");
    } else if (MYNEWT_VAL(BLE_ISO_NON_STD_FLOW_CTRL)) {
        BLE_HS_LOG(INFO, "%s", "Non-standard");
    } else {
        BLE_HS_LOG(INFO, "%s", "Not support");
    }

    return 0;
}

#if MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL)
void
ble_hs_hci_add_iso_avail_pkts(uint16_t conn_handle, uint16_t delta)
{
    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    if (ble_hs_iso_avail_pkts + delta > ble_hs_iso_max_pkts) {
        BLE_HS_LOG(ERROR, "ISO_HS_RESET %u %u %u", ble_hs_iso_avail_pkts, delta, ble_hs_iso_max_pkts);
        ble_hs_sched_reset(BLE_HS_ECONTROLLER);
    } else {
        ble_hs_iso_avail_pkts += delta;
    }
}
#endif /* MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL) */

#if MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL)
static uint8_t
ble_hs_hci_iso_buf_needed(uint16_t sdu_len, bool ts_flag)
{
    uint16_t sdu_offset;
    uint16_t dl_len;
    uint8_t dlh_len;
    uint8_t count;

    dlh_len = (ts_flag ? BLE_HCI_ISO_DATA_LOAD_TS_SZ : 0) + BLE_HCI_ISO_DATA_LOAD_HDR_SZ;
    sdu_offset = 0;
    count = 1;  /* 1 extra since framed pdu may be used */

    while (1) {
        dl_len = min(dlh_len + sdu_len - sdu_offset, ble_hs_iso_buf_sz);

        count += 1;

        sdu_offset += dl_len - dlh_len;
        assert(sdu_offset <= sdu_len);

        if (sdu_offset == sdu_len) {
            break;
        }

        /* No data load header for continuation/last segment */
        dlh_len = 0;
    }

    return count;
}
#endif /* MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL) */

static void
ble_hs_hci_iso_hdr_append(uint16_t conn_handle, uint16_t sdu_len,
                          bool ts_flag, uint32_t time_stamp,
                          uint16_t pkt_seq_num, uint8_t *frag,
                          uint8_t pb_flag, uint16_t dl_len)
{
    uint32_t pkt_hdr;
    uint32_t dl_hdr;

    pkt_hdr = ((pb_flag << 12) | conn_handle);
    if (pb_flag == BLE_HCI_ISO_PB_FIRST_FRAG ||
        pb_flag == BLE_HCI_ISO_PB_COMP_SDU) {
        pkt_hdr |= (ts_flag << 14);
    }
    pkt_hdr |= (dl_len << 16);

    memcpy(frag, &pkt_hdr, BLE_HCI_ISO_DATA_HDR_SZ);

    /* No data load header for continuation/last segment */
    if (pb_flag == BLE_HCI_ISO_PB_CONT_FRAG ||
        pb_flag == BLE_HCI_ISO_PB_LAST_FRAG) {
        return;
    }

    if (ts_flag) {
        memcpy(frag + BLE_HCI_ISO_DATA_HDR_SZ, &time_stamp, BLE_HCI_ISO_DATA_LOAD_TS_SZ);
    }

    dl_hdr = (sdu_len << 16) | pkt_seq_num;

    memcpy(frag + BLE_HCI_ISO_DATA_HDR_SZ + (ts_flag ? BLE_HCI_ISO_DATA_LOAD_TS_SZ : 0),
           &dl_hdr, BLE_HCI_ISO_DATA_LOAD_HDR_SZ);
}

static int
ble_hs_hci_iso_tx_now(uint16_t conn_handle, const uint8_t *sdu, uint16_t sdu_len,
                      bool ts_flag, uint32_t time_stamp, uint16_t pkt_seq_num)
{
    uint8_t dlh_len;
    uint8_t *frag;
    int rc;

#if MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL)
    /* Get the Controller ISO buffer needed for the SDU */
    uint8_t count = ble_hs_hci_iso_buf_needed(sdu_len, ts_flag);

    /* Make sure the Controller ISO buffer can accommodate the SDU completely */
    if (count > ble_hs_iso_avail_pkts) {
        BLE_HS_LOG(WARN, "ISO flow control(%u/%u)!", count, ble_hs_iso_avail_pkts);
        return BLE_HS_EAGAIN;
    }
#elif MYNEWT_VAL(BLE_ISO_NON_STD_FLOW_CTRL)
    extern uint16_t ble_ll_iso_free_buf_num_get(uint16_t conn_handle);
    if (ble_ll_iso_free_buf_num_get(conn_handle) == 0) {
        BLE_HS_LOG(WARN, "ISO flow control!");
        return BLE_HS_EAGAIN;
    }
#endif

    dlh_len = (ts_flag ? BLE_HCI_ISO_DATA_LOAD_TS_SZ : 0) + BLE_HCI_ISO_DATA_LOAD_HDR_SZ;

    frag = nimble_platform_mem_malloc(BLE_HCI_ISO_DATA_HDR_SZ + dlh_len + sdu_len);
    if (frag == NULL) {
        return BLE_HS_ENOMEM;
    }

    ble_hs_hci_iso_hdr_append(conn_handle, sdu_len, ts_flag, time_stamp, pkt_seq_num,
                              frag, BLE_HCI_ISO_PB_COMP_SDU, dlh_len + sdu_len);

    memcpy(frag + BLE_HCI_ISO_DATA_HDR_SZ + dlh_len, sdu, sdu_len);

    extern int ble_hci_trans_hs_iso_tx(const uint8_t *data, uint16_t length, void *arg);
    rc = ble_hci_trans_hs_iso_tx(frag, BLE_HCI_ISO_DATA_HDR_SZ + dlh_len + sdu_len, NULL);
    if (rc) {
        return BLE_HS_EDONE;
    }

#if MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL)
    /* If an ISO SDU is fragmented into fragments, flow control is not supported.
     *
     * Currently even if an SDU is larger than the ISO buffer size, fragmentation
     * will not happen here, the SDU will be posted to Controller completely.
     */
    ble_hs_iso_avail_pkts -= 1;
#endif /* MYNEWT_VAL(BLE_ISO_STD_FLOW_CTRL) */

    return 0;
}

int
ble_hs_hci_iso_tx(uint16_t conn_handle, const uint8_t *sdu, uint16_t sdu_len,
                  bool ts_flag, uint32_t time_stamp, uint16_t pkt_seq_num)
{
    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    return ble_hs_hci_iso_tx_now(conn_handle, sdu, sdu_len, ts_flag, time_stamp, pkt_seq_num);
}

static ble_hs_iso_pkt_rx_fn ble_hs_iso_pkt_rx_cb;

int
ble_hs_iso_pkt_rx_cb_set(void *cb)
{
    if (cb == NULL) {
        return -BLE_HS_EINVAL;
    }

    if (ble_hs_iso_pkt_rx_cb) {
        return -BLE_HS_EALREADY;
    }

    ble_hs_iso_pkt_rx_cb = cb;

    return 0;
}

int
ble_hs_rx_iso_data(const uint8_t *data, uint16_t len, void *arg)
{
    if (ble_hs_iso_pkt_rx_cb) {
        ble_hs_iso_pkt_rx_cb(data, len, arg);
    }

    /* The `data` is dynamically allocated by Controller, free it here */
    bt_osi_mem_free((void *)data);

    return 0;
}

#endif /* MYNEWT_VAL(BLE_ISO) */
