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

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <syscfg/syscfg.h>
#include <os/os.h>
#include <os/os_mbuf.h>
#include <nimble/hci_common.h>
#include <nimble/transport.h>
#include <nimble/transport/hci_h4.h>
#if MYNEWT_VAL(BLE_HOST)
#include "host/ble_hs.h"
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#define HCI_H4_SM_W4_PKT_TYPE   0
#define HCI_H4_SM_W4_HEADER     1
#define HCI_H4_SM_W4_PAYLOAD    2
#define HCI_H4_SM_COMPLETED     3
/* Controller->host sync loss. Bytes are not parsed until HCI_Reset's CC. */
#define HCI_H4_SM_RESYNC        4

/*
 * Host reset reason for H4 sync loss (Core v6.3 Vol 4 Part A §4).
 * Same value as a controller Hardware Error event with Hardware_Code 0x01.
 * reset_cb cannot tell those two apart. Both still reset the host.
 */
#if MYNEWT_VAL(BLE_HOST)
#define HCI_H4_RESET_REASON_SYNC    BLE_HS_HW_ERR(0x01)
#endif

const struct hci_h4_allocators hci_h4_allocs_from_ll = {
    .acl = ble_transport_alloc_acl_from_ll,
    .evt = ble_transport_alloc_evt,
};
const struct hci_h4_allocators hci_h4_allocs_from_hs = {
    .cmd = ble_transport_alloc_cmd,
    .acl = ble_transport_alloc_acl_from_hs,
    .iso = ble_transport_alloc_iso_from_hs,
};

struct hci_h4_input_buffer {
    const uint8_t *buf;
    uint16_t len;
};

static int
hci_h4_has_alloc(const struct hci_h4_sm *h4sm, uint8_t pkt_type)
{
    if (h4sm->allocs == NULL) {
        return 0;
    }

    switch (pkt_type) {
    case HCI_H4_CMD:
        return h4sm->allocs->cmd != NULL;
    case HCI_H4_ACL:
        return h4sm->allocs->acl != NULL;
    case HCI_H4_EVT:
        return h4sm->allocs->evt != NULL;
    case HCI_H4_ISO:
        return h4sm->allocs->iso != NULL;
    default:
        return 0;
    }
}

/* exp_len is the arm flag during RESYNC. Host task writes it, UART reads it. */
static void
hci_h4_arm_set(struct hci_h4_sm *h4sm, uint16_t armed)
{
    *(volatile uint16_t *)&h4sm->exp_len = armed;
}

static uint16_t
hci_h4_arm_get(const struct hci_h4_sm *h4sm)
{
    return *(volatile uint16_t *)&h4sm->exp_len;
}

static void
hci_h4_match_set(struct hci_h4_sm *h4sm, uint16_t idx)
{
    *(volatile uint16_t *)&h4sm->len = idx;
}

static uint16_t
hci_h4_match_get(const struct hci_h4_sm *h4sm)
{
    return *(volatile uint16_t *)&h4sm->len;
}

static uint8_t
hci_h4_state_get(const struct hci_h4_sm *h4sm)
{
    return *(volatile uint8_t *)&h4sm->state;
}

static void
hci_h4_state_set(struct hci_h4_sm *h4sm, uint8_t state)
{
    *(volatile uint8_t *)&h4sm->state = state;
}

static int
hci_h4_rx_from_ll(const struct hci_h4_sm *h4sm)
{
    return h4sm->allocs != NULL &&
           h4sm->allocs->evt != NULL &&
           h4sm->allocs->cmd == NULL;
}

static void
hci_h4_free_partial(struct hci_h4_sm *h4sm)
{
    if (h4sm->pkt_type == HCI_H4_ACL || h4sm->pkt_type == HCI_H4_ISO) {
        if (h4sm->om != NULL) {
            os_mbuf_free_chain(h4sm->om);
            h4sm->om = NULL;
        }
        return;
    }

    if (h4sm->buf != NULL) {
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
        ble_transport_free(h4sm->pkt_type, h4sm->buf);
#else
        ble_transport_free(h4sm->buf);
#endif
        h4sm->buf = NULL;
    }
}

/*
 * Controller->host loss of sync. Queues a host reset (drops every connection)
 * and stops delivering frames. Returns 1 if this SM is now resyncing.
 * Host->controller parsers are left to drop the octet and keep scanning.
 */
static int
hci_h4_sync_lost(struct hci_h4_sm *h4sm)
{
#if MYNEWT_VAL(BLE_HOST)
    if (!hci_h4_rx_from_ll(h4sm)) {
        return 0;
    }

    if (h4sm->state == HCI_H4_SM_RESYNC) {
        return 1;
    }

    hci_h4_free_partial(h4sm);
    hci_h4_state_set(h4sm, HCI_H4_SM_RESYNC);
    h4sm->pkt_type = 0;
    h4sm->min_len = 0;
    h4sm->buf = NULL;
    memset(h4sm->hdr, 0, sizeof(h4sm->hdr));
    hci_h4_match_set(h4sm, 0);
    /*
     * Arm before the host reset: every controller->host framer must match the
     * HCI_Reset Command Complete (Core v6.3 Vol 4 Part A 4) even when the
     * transport does not report the queued Reset command.
     */
    hci_h4_arm_set(h4sm, 1);
    ble_hs_sched_reset(HCI_H4_RESET_REASON_SYNC);
    return 1;
#else
    (void)h4sm;
    return 0;
#endif
}

static void
hci_h4_resync_deliver(struct hci_h4_sm *h4sm)
{
    uint8_t *buf;
    uint16_t opcode;
    int rc;

    hci_h4_match_set(h4sm, 0);

    if (h4sm->allocs == NULL || h4sm->allocs->evt == NULL ||
        h4sm->frame_cb == NULL) {
        return;
    }

    /* Event code + length + Command Complete body. */
    if (MYNEWT_VAL(BLE_TRANSPORT_EVT_SIZE) <
        (2 + (int)sizeof(struct ble_hci_ev_command_complete))) {
        return;
    }

    buf = h4sm->allocs->evt(0);
    if (buf == NULL) {
        /* Startup HCI command times out and sends Reset again. */
        return;
    }

    opcode = BLE_HCI_OP(BLE_HCI_OGF_CTLR_BASEBAND, BLE_HCI_OCF_CB_RESET);
    buf[0] = BLE_HCI_EVCODE_COMMAND_COMPLETE;
    buf[1] = sizeof(struct ble_hci_ev_command_complete);
    buf[2] = h4sm->hdr[0];
    buf[3] = (uint8_t)opcode;
    buf[4] = (uint8_t)(opcode >> 8);
    buf[5] = h4sm->hdr[1];

    /*
     * frame_cb -> ble_hs_hci_rx_evt takes the buffer. On error it frees
     * the buffer before returning. Do not free here.
     */
    rc = h4sm->frame_cb(HCI_H4_EVT, buf);
    (void)rc;

    hci_h4_state_set(h4sm, HCI_H4_SM_W4_PKT_TYPE);
    h4sm->pkt_type = 0;
    h4sm->min_len = 0;
    h4sm->buf = NULL;
    memset(h4sm->hdr, 0, sizeof(h4sm->hdr));
    hci_h4_match_set(h4sm, 0);
    hci_h4_arm_set(h4sm, 0);
}

static void
hci_h4_resync_feed(struct hci_h4_sm *h4sm, uint8_t b)
{
    uint16_t idx;
    uint16_t opcode;
    uint8_t expect;

    if (hci_h4_arm_get(h4sm) == 0) {
        hci_h4_match_set(h4sm, 0);
        return;
    }

    idx = hci_h4_match_get(h4sm);
    opcode = BLE_HCI_OP(BLE_HCI_OGF_CTLR_BASEBAND, BLE_HCI_OCF_CB_RESET);

    switch (idx) {
    case 0:
        expect = HCI_H4_EVT;
        break;
    case 1:
        expect = BLE_HCI_EVCODE_COMMAND_COMPLETE;
        break;
    case 2:
        expect = sizeof(struct ble_hci_ev_command_complete);
        break;
    case 3:
        h4sm->hdr[0] = b;
        hci_h4_match_set(h4sm, 4);
        return;
    case 4:
        expect = (uint8_t)opcode;
        break;
    case 5:
        expect = (uint8_t)(opcode >> 8);
        break;
    case 6:
        h4sm->hdr[1] = b;
        hci_h4_resync_deliver(h4sm);
        return;
    default:
        hci_h4_match_set(h4sm, (b == HCI_H4_EVT) ? 1 : 0);
        return;
    }

    if (b == expect) {
        hci_h4_match_set(h4sm, idx + 1);
        return;
    }

    hci_h4_match_set(h4sm, (b == HCI_H4_EVT) ? 1 : 0);
}

void
hci_h4_sm_note_host_reset_cmd(struct hci_h4_sm *h4sm)
{
    if (h4sm == NULL || hci_h4_state_get(h4sm) != HCI_H4_SM_RESYNC) {
        return;
    }

    /* Clear any partial match, then publish armed. */
    hci_h4_match_set(h4sm, 0);
    hci_h4_arm_set(h4sm, 1);

    /* UART may have left RESYNC while the command was being queued. */
    if (hci_h4_state_get(h4sm) != HCI_H4_SM_RESYNC) {
        hci_h4_arm_set(h4sm, 0);
    }
}

static void
hci_h4_frame_start(struct hci_h4_sm *rxs, uint8_t pkt_type)
{
    rxs->pkt_type = pkt_type;
    rxs->len = 0;
    rxs->exp_len = 0;
    rxs->min_len = 0;
    memset(rxs->hdr, 0, sizeof(rxs->hdr));

    switch (rxs->pkt_type) {
    case HCI_H4_CMD:
        rxs->min_len = 3;
        break;
    case HCI_H4_ACL:
    case HCI_H4_ISO:
        rxs->min_len = 4;
        break;
    case HCI_H4_EVT:
        rxs->min_len = 2;
        break;
    default:
        /* XXX sync loss */
        assert(0);
        break;
    }
}

static int
hci_h4_ib_consume(struct hci_h4_input_buffer *ib, uint16_t len)
{
    assert(ib->len >= len);

    ib->buf += len;
    ib->len -= len;

    return len;
}

static int
hci_h4_ib_pull_min_len(struct hci_h4_sm *rxs,
                       struct hci_h4_input_buffer *ib)
{
    uint16_t len;

    len = min(ib->len, rxs->min_len - rxs->len);
    memcpy(&rxs->hdr[rxs->len], ib->buf, len);

    rxs->len += len;
    hci_h4_ib_consume(ib, len);

    return rxs->len != rxs->min_len;
}

static int
hci_h4_sm_w4_header(struct hci_h4_sm *h4sm, struct hci_h4_input_buffer *ib)
{
    int rc;

    rc = hci_h4_ib_pull_min_len(h4sm, ib);
    if (rc) {
        /* need more data */
        return 1;
    }

    switch (h4sm->pkt_type) {
    case HCI_H4_CMD:
        assert(h4sm->allocs && h4sm->allocs->cmd);
        h4sm->buf = h4sm->allocs->cmd();
        if (!h4sm->buf) {
            return -1;
        }

        memcpy(h4sm->buf, h4sm->hdr, h4sm->len);
        h4sm->exp_len = h4sm->hdr[2] + 3;
        break;
    case HCI_H4_ACL:
        assert(h4sm->allocs && h4sm->allocs->acl);
        h4sm->om = h4sm->allocs->acl();
        if (!h4sm->om) {
            return -1;
        }

        os_mbuf_append(h4sm->om, h4sm->hdr, h4sm->len);
        h4sm->exp_len = get_le16(&h4sm->hdr[2]) + 4;
        break;
    case HCI_H4_EVT:
        if (h4sm->hdr[0] == BLE_HCI_EVCODE_LE_META) {
            /* For LE Meta event we need 3 bytes to parse header */
            h4sm->min_len = 3;
            rc = hci_h4_ib_pull_min_len(h4sm, ib);
            if (rc) {
                /* need more data */
                return 1;
            }
        }

        assert(h4sm->allocs && h4sm->allocs->evt);

        h4sm->exp_len = h4sm->hdr[1] + 2;
        /*
         * A declared frame shorter than the header already taken cannot be
         * framed at all: that is loss of sync.
         */
        if (h4sm->exp_len < h4sm->len) {
            if (hci_h4_sync_lost(h4sm)) {
                return 1;
            }
            h4sm->state = HCI_H4_SM_W4_PKT_TYPE;
            return 1;
        }

        /*
         * A length octet that is inside the HCI event range but larger than
         * this transport's event buffer is a frame we cannot store, not loss
         * of synchronization (Core v6.3 Vol 4 Part A 4). Skip it by its own
         * length: the payload is discarded and the next frame stays aligned.
         */
        if (h4sm->exp_len > MYNEWT_VAL(BLE_TRANSPORT_EVT_SIZE)) {
            return 0;
        }

        /* Legacy advertising reports may come from the discardable pool.
         * hdr[2] is the subevent code, and it is valid only for LE Meta.
         */
        if ((h4sm->hdr[0] == BLE_HCI_EVCODE_LE_META) &&
            (h4sm->hdr[2] == BLE_HCI_LE_SUBEV_ADV_RPT)) {
            h4sm->buf = h4sm->allocs->evt(1);
        } else {
            h4sm->buf = h4sm->allocs->evt(0);
            if (!h4sm->buf) {
                return -1;
            }
        }

        if (h4sm->buf) {
            memcpy(h4sm->buf, h4sm->hdr, h4sm->len);
        }
        break;
    case HCI_H4_ISO:
        assert(h4sm->allocs && h4sm->allocs->iso);
        h4sm->om = h4sm->allocs->iso();
        if (!h4sm->om) {
            return -1;
        }

        os_mbuf_append(h4sm->om, h4sm->hdr, h4sm->len);
        h4sm->exp_len = (get_le16(&h4sm->hdr[2]) & 0x7fff) + 4;
        break;
    default:
        assert(0);
        break;
    }

    return 0;
}

static int
hci_h4_sm_w4_payload(struct hci_h4_sm *h4sm,
                     struct hci_h4_input_buffer *ib)
{
    uint16_t mbuf_len;
    uint16_t len;
    int rc;

    if (h4sm->exp_len < h4sm->len) {
        return 0;
    }

    len = min(ib->len, h4sm->exp_len - h4sm->len);

    switch (h4sm->pkt_type) {
    case HCI_H4_CMD:
    case HCI_H4_EVT:
        if (h4sm->buf) {
            memcpy(&h4sm->buf[h4sm->len], ib->buf, len);
        }
        break;
    case HCI_H4_ACL:
    case HCI_H4_ISO:
        assert(h4sm->om);

        mbuf_len = OS_MBUF_PKTLEN(h4sm->om);
        rc = os_mbuf_append(h4sm->om, ib->buf, len);
        if (rc) {
            /* Some data may already be appended so need to adjust h4sm only by
             * the size of appended data.
             */
            len = OS_MBUF_PKTLEN(h4sm->om) - mbuf_len;
            h4sm->len += len;
            hci_h4_ib_consume(ib, len);

            return -1;
        }
        break;
    default:
        assert(0);
        break;
    }

    h4sm->len += len;
    hci_h4_ib_consume(ib, len);

    /* return 1 if need more data */
    return h4sm->len != h4sm->exp_len;
}

static void
hci_h4_sm_completed(struct hci_h4_sm *h4sm)
{
    int rc;

    switch (h4sm->pkt_type) {
    case HCI_H4_CMD:
    case HCI_H4_EVT:
        if (h4sm->buf) {
            assert(h4sm->frame_cb);
            rc = h4sm->frame_cb(h4sm->pkt_type, h4sm->buf);
#if MYNEWT_VAL(BLE_HOST)
            /*
             * The BLE host frees CMD/EVT buffers before returning an error
             * (see ble_hs_hci_rx_evt). Do not free here on failure.
             */
            (void)rc;
#else
            if (rc != 0) {
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
                ble_transport_free(h4sm->pkt_type, h4sm->buf);
#else
                ble_transport_free(h4sm->buf);
#endif
            }
#endif
            h4sm->buf = NULL;
        }
        break;
    case HCI_H4_ACL:
    case HCI_H4_ISO:
        if (h4sm->om) {
            assert(h4sm->frame_cb);
            rc = h4sm->frame_cb(h4sm->pkt_type, h4sm->om);
#if MYNEWT_VAL(BLE_HOST)
            /* ble_hs_rx_data always consumes the mbuf. */
            (void)rc;
#else
            if (rc != 0) {
                os_mbuf_free_chain(h4sm->om);
            }
#endif
            h4sm->om = NULL;
        }
        break;
    default:
        assert(0);
        break;
    }
}

int
hci_h4_sm_rx(struct hci_h4_sm *h4sm, const uint8_t *buf, uint16_t len)
{
    struct hci_h4_input_buffer ib = {
        .buf = buf,
        .len = len,
    };

    int rc = 0;
    while (ib.len && (rc >= 0)) {
        rc = 0;
        switch (h4sm->state) {
        case HCI_H4_SM_W4_PKT_TYPE:
            if (!hci_h4_has_alloc(h4sm, ib.buf[0])) {
                /*
                 * Not an indicator this direction can frame. Command bytes
                 * are host->controller only. SCO/ISO with no allocator
                 * cannot be skipped by length, so the stream is no longer
                 * aligned. Controller->host: HCI_Reset, then scan for its
                 * Command Complete. Other direction: drop the octet.
                 */
                hci_h4_ib_consume(&ib, 1);
                hci_h4_sync_lost(h4sm);
                break;
            }
            hci_h4_frame_start(h4sm, ib.buf[0]);
            hci_h4_ib_consume(&ib, 1);
            h4sm->state = HCI_H4_SM_W4_HEADER;
        /* no break */
        case HCI_H4_SM_W4_HEADER:
            rc = hci_h4_sm_w4_header(h4sm, &ib);
            assert(rc >= 0);
            if (rc) {
                break;
            }
            h4sm->state = HCI_H4_SM_W4_PAYLOAD;
        /* no break */
        case HCI_H4_SM_W4_PAYLOAD:
            rc = hci_h4_sm_w4_payload(h4sm, &ib);
            assert(rc >= 0);
            if (rc) {
                break;
            }
            h4sm->state = HCI_H4_SM_COMPLETED;
        /* no break */
        case HCI_H4_SM_COMPLETED:
            hci_h4_sm_completed(h4sm);
            h4sm->state = HCI_H4_SM_W4_PKT_TYPE;
            break;
        case HCI_H4_SM_RESYNC:
#if MYNEWT_VAL(BLE_HOST)
            /* The reset this state waits for may have been dropped by a full
             * host queue; re-post it while it is still pending. */
            if (hci_h4_rx_from_ll(h4sm)) {
                ble_hs_sched_reset_retry();
            }
#endif
            hci_h4_resync_feed(h4sm, ib.buf[0]);
            hci_h4_ib_consume(&ib, 1);
            break;
        default:
            assert(0);
            break;
        }
    }

    /* Calculate consumed bytes
     *
     * Note: we should always consume some bytes unless there is an oom error.
     * It's also possible that we have an oom error but already consumed some
     * data, in such case just return success and error will be returned on next
     * pass.
     */
    len = len - ib.len;
    if (len == 0) {
        assert(rc < 0);
        return -1;
    }

    return len;
}

void
hci_h4_sm_init(struct hci_h4_sm *h4sm, const struct hci_h4_allocators *allocs,
               hci_h4_frame_cb *frame_cb)
{
    memset(h4sm, 0, sizeof(*h4sm));
    h4sm->allocs = allocs;
    h4sm->frame_cb = frame_cb;
}

void
hci_h4_sm_deinit(struct hci_h4_sm *h4sm)
{
    if (h4sm == NULL) {
        return;
    }

    hci_h4_free_partial(h4sm);
    memset(h4sm, 0, sizeof(*h4sm));
}

void
hci_h4_sm_sync_lost(struct hci_h4_sm *h4sm)
{
    if (h4sm == NULL) {
        return;
    }

    (void)hci_h4_sync_lost(h4sm);
}
