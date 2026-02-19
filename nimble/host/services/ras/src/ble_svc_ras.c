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

#include "syscfg/syscfg.h"

/* BLE_RAS_SERVICE is declared in this package's syscfg.yml alongside
 * BLE_SVC_RAS_SYSINIT_STAGE, so MYNEWT_VAL(BLE_RAS_SERVICE) always resolves.
 *
 * ESP-IDF does not run the syscfg flow: it mirrors every MYNEWT_VAL by hand
 * into components/bt/host/nimble/port/include/esp_nimble_cfg.h, where
 * MYNEWT_VAL_BLE_RAS_SERVICE follows MYNEWT_VAL_BLE_CHANNEL_SOUNDING and takes
 * its value from the Kconfig option BT_NIMBLE_RAS_SERVICE (Kconfig.in: bool,
 * depends on BT_NIMBLE_CHANNEL_SOUNDING, default y).  On that build the port
 * header is the source of truth and is where a change to the setting belongs;
 * reading the guard needs the esp-idf side of the tree at a revision matching
 * this submodule. */
#if MYNEWT_VAL(BLE_CHANNEL_SOUNDING) && MYNEWT_VAL(BLE_RAS_SERVICE)

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "../src/ble_gatt_priv.h"
#include "../../../src/ble_hs_priv.h"
#include "services/ras/ble_svc_ras.h"
#include "sysinit/sysinit.h"
#include "host/ble_cs.h"
#include "host/ble_hs_log.h"
#include "nimble/hci_common.h"
#include "nimble/nimble_port.h"
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#include "esp_nimble_mem.h"
#endif

#if MYNEWT_VAL(BLE_GATTS) && MYNEWT_VAL(BLE_RAS_SERVICE)
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/* RRSP-internal constants (not part of public API) */
#define BLE_SVC_RAS_CP_CMD_OPCODE_OFFSET     0
#define BLE_SVC_RAS_CP_CMD_PARAMS_OFFSET     BLE_SVC_RAS_CP_CMD_OPCODE_LEN
#define BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS   5000

/* RAS pool sizing.  One served connection: ble_cs.c tracks a single CS
 * procedure at a time, so a second struct ble_svc_ras_rrsp would never be
 * fed any ranging data while its RD buffers cost BLE_RAS_PROCEDURE_MEM
 * each.  Raise this together with the CS state in ble_cs.c if concurrent
 * ranging is ever added.  With BLE_STATIC_TO_DYNAMIC enabled the pools are
 * also lazy-initialized on first use instead of at sysinit, so an application
 * that never serves RAS data never pays the RAM cost. */
#define RAS_RRSP_MAX_CONN           1
/* Two buffers, because a fetch outlives a procedure: the client holds a buffer
 * from GET_RD until ACK_RD, which typically takes longer than the interval
 * between procedures.  With a single buffer the next procedure would find it
 * claimed (refcount > 0) and be dropped; the second buffer catches it. */
#define RAS_RD_BUFFERS_PER_CONN     2
#define RAS_TOTAL_RD_BUFFERS        (RAS_RRSP_MAX_CONN * RAS_RD_BUFFERS_PER_CONN)

#define BLE_HCI_LE_CS_SUBEVENT_DONE_STATUS_COMPLETE  0x0
#define BLE_HCI_LE_CS_SUBEVENT_DONE_STATUS_PARTIAL   0x1
#define BLE_HCI_LE_CS_SUBEVENT_DONE_STATUS_ABORTED   0xF

/* Procedure_Done_Status shares the same encoding as Subevent_Done_Status per
 * BT Core Spec 7.7.65.44, but is a distinct field.  Use dedicated macros to
 * keep procedure-level and subevent-level checks semantically separate. */
#define BLE_HCI_LE_CS_PROCEDURE_DONE_STATUS_COMPLETE 0x0
#define BLE_HCI_LE_CS_PROCEDURE_DONE_STATUS_PARTIAL  0x1
#define BLE_HCI_LE_CS_PROCEDURE_DONE_STATUS_ABORTED  0xF

#define BLE_HCI_LE_CS_ABORT_REASON_UNSPECIFIED       0xF

/* Delay between segment notifications (ms).  Real-time mode has no
 * retransmission, so the stream for one procedure should drain before the next
 * procedure's data is ready.  The reflector does not choose the CS procedure
 * interval - the initiator does - so this cannot be derived from the interval,
 * and sending faster than the link carries only queues data up until the mbuf
 * pool is empty (see RAS_SEG_MAX_STALL_RETRIES).  Keep it slow enough that a
 * stream drains while the controller is also spending airtime on the CS
 * procedure itself. */
#define RAS_SEG_DELAY_MS 50

/* Inter-segment delay for real-time ranging data (ms).  Real-time data is only
 * useful to a client that still holds its own step data for the same procedure,
 * and a client discards that as soon as the next procedure starts, so a record
 * that takes longer than one procedure interval to stream arrives after the
 * client has already moved on and is dropped as a counter mismatch every time.
 * The whole record therefore has to be on the wire within one interval, not
 * just each segment: at the shortest interval the channel sounding examples
 * negotiate that leaves well under RAS_SEG_DELAY_MS per segment.  On-demand
 * data keeps the slower cadence above because the client asks for a specific
 * counter and holds it until the transfer finishes. */
#define RAS_RT_SEG_DELAY_MS 15

/* Real-time segments allowed to be outstanding at the controller at once.  A
 * controller only reports its ACL buffer count, which can be as low as three,
 * and it returns them a connection event at a time; filling them all leaves the
 * stream with nothing to do but drop segments until credits come back. */
#define RAS_RT_MAX_INFLIGHT_PKTS 1

/* A segment notification is queued on the connection and its mbuf is only
 * returned once the controller reports the packet complete, while the notify
 * call reports success as soon as the packet is queued.  There is therefore no
 * completion signal to pace against, and a client that asks for procedures
 * faster than the link drains them fills the mbuf pool.  Once that happens the
 * segment cannot be built at all, so retry, but give up after this many tries.
 * 50 tries at RAS_SEG_RETRY_MS is a ~500 ms budget, half the inter-segment gap
 * after which RAP 4.4.1.1 lets the client drop the stream - real-time data is
 * not retransmitted, so give up before the client does and let the next
 * procedure start clean instead of retrying forever and never sending anything
 * again. */
#define RAS_SEG_MAX_STALL_RETRIES 50

/* Retry period for a segment that could not be queued (ms) */
#define RAS_SEG_RETRY_MS 10

/* Bound on this connection's notifications parked in the host's outbound ACL
 * queue.  ble_l2cap_tx() reports success for a packet it could only park on the
 * unbounded conn->bhc_tx_q, so a controller that stops returning ACL credits
 * would let this fixed-cadence streamer drain msys.  Ranging data is
 * perishable: drop the segment instead and let the stall retry abandon it.
 *
 * This has to stay above the number of segments one record needs, or the last
 * segment of a record leaves the queue exactly at the bound and the next
 * record's first segment is refused before the link has had a chance to drain
 * any of it.  That turns a guard against a stalled controller into a limit on
 * normal streaming.  A record is ranging-data-length / (ATT_MTU - 4) segments,
 * four at the MTU the channel sounding examples negotiate. */
#define RAS_MAX_QUEUED_SEGS 8

/* Free mbufs to leave for the rest of the host.  Ranging data is only one user
 * of the shared pool, and the segment stream is the one user with a fixed
 * cadence that does not wait for completions, so it is the one that has to
 * yield.  Below this threshold a segment allocation would very likely fail
 * anyway, so bail early rather than building a partial segment that has to
 * be discarded. */
#define RAS_MIN_FREE_MBUFS 4

/* Forward Declarations */
static int ble_svc_ras_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_svc_ras_ack_timer_cb(struct ble_npl_event *ev);
static void ble_svc_ras_seg_timer_cb(struct ble_npl_event *ev);
static int ble_svc_ras_gap_listener_cb(struct ble_gap_event *event, void *arg);

/* GATT Handle Storage */
static uint16_t ble_svc_ras_feat_val_handle;
static uint16_t ble_svc_ras_rt_rd_val_handle;
static uint16_t ble_svc_ras_od_rd_val_handle;
static uint16_t ble_svc_ras_cp_val_handle;
static uint16_t ble_svc_ras_rd_val_handle;
static uint16_t ble_svc_ras_rd_ov_val_handle;

static uint32_t ble_svc_ras_feat_val;
static struct ble_gap_event_listener ras_gap_listener;

/* GATT Service / Characteristic Definitions */
static const ble_uuid16_t uuid_svc_ras = BLE_UUID16_INIT(BLE_SVC_RAS_RANGING_SERVICE_VAL);
static const ble_uuid16_t uuid_chr_feat_value = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_FEATURES_VAL);
static const ble_uuid16_t uuid_chr_demand_rd =
    BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_ONDEMAND_RD_VAL);
static const ble_uuid16_t uuid_chr_real_rd = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_REALTIME_RD_VAL);
static const ble_uuid16_t uuid_chr_ctrl_pt = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_CP_VAL);
static const ble_uuid16_t uuid_chr_data_rdy = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_RD_READY_VAL);
static const ble_uuid16_t uuid_chr_data_ow =
    BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_RD_OVERWRITTEN_VAL);

/* RAS 1.0, Table 3.1 requires every characteristic of this service to be
 * protected by encryption: value reads and writes take BLE_GATT_CHR_F_READ_ENC
 * and BLE_GATT_CHR_F_WRITE_ENC, and the client configuration descriptors take
 * BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC - without the latter an unencrypted peer
 * can write a CCCD and, in real-time mode, be handed raw ranging data. */
static const struct ble_gatt_chr_def ras_characteristics[] = {
    {
        .uuid = &uuid_chr_feat_value.u,
        .access_cb = ble_svc_ras_chr_access,
        .val_handle = &ble_svc_ras_feat_val_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
    },
    {
        .uuid = &uuid_chr_demand_rd.u,
        .access_cb = ble_svc_ras_chr_access,
        .val_handle = &ble_svc_ras_od_rd_val_handle,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE |
                 BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
    },
    {
        .uuid = &uuid_chr_real_rd.u,
        .access_cb = ble_svc_ras_chr_access,
        .val_handle = &ble_svc_ras_rt_rd_val_handle,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE |
                 BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
    },
    {
        .uuid = &uuid_chr_ctrl_pt.u,
        .access_cb = ble_svc_ras_chr_access,
        .val_handle = &ble_svc_ras_cp_val_handle,
        .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC |
                 BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
    },
    {
        .uuid = &uuid_chr_data_rdy.u,
        .access_cb = ble_svc_ras_chr_access,
        .val_handle = &ble_svc_ras_rd_val_handle,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE |
                 BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC |
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = &uuid_chr_data_ow.u,
        .access_cb = ble_svc_ras_chr_access,
        .val_handle = &ble_svc_ras_rd_ov_val_handle,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE |
                 BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC |
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ,
    },
    { 0 },
};

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_ras.u,
        .characteristics = ras_characteristics,
    },
    { 0 },
};

/* Ranging Buffer Pool */
struct ranging_buffer {
    uint16_t conn_handle;
    uint16_t ranging_counter;
    uint16_t subevent_cursor;
    uint16_t subev_hdr_offset;
    /* Monotonic allocation sequence used to pick the oldest buffer during
     * eviction.  The 12-bit RAS ranging_counter wraps every 4096
     * procedures, so a raw counter compare is unreliable; a signed delta
     * over this 16-bit sequence is. */
    uint16_t alloc_seq;
    /* Flags packed into one byte.  refcount caps at 7 (3-bit field),
     * which is more than enough: refcount only increments during active
     * CP streaming (one pending indication at a time per RAS spec). */
    uint8_t  refcount : 3;
    uint8_t  ready    : 1;
    uint8_t  busy     : 1;
    /* An RD Ready indication for this counter actually reached the client.
     * Only then does RD Overwritten mean anything to it - see the eviction
     * paths in ble_svc_ras_buf_alloc(). */
    uint8_t  announced : 1;
    /* The body did not fit BLE_RAS_PROCEDURE_MEM, so it was finalised early
     * while the controller still had subevents of this procedure to report.
     * Those later subevents must be dropped rather than allowed to
     * re-initialise the buffer - see ble_svc_ras_buf_alloc(). */
    uint8_t  truncated : 1;
    /* A step of the subevent currently being assembled did not fit and was
     * dropped.  Latched here because the loss has to outlive the event that
     * caused it: step sizes differ within a subevent, so a later
     * SUBEVENT_RESULT_CONTINUE carrying smaller steps can still fit, and that
     * event would otherwise write its own Complete status over the subevent
     * header a dropped step had marked aborted.  Reset when the next
     * SUBEVENT_RESULT opens a header of its own. */
    uint8_t  steps_lost : 1;
    union {
        uint8_t buf[BLE_RAS_PROCEDURE_MEM];
        struct {
            struct ranging_header ranging_header;
            uint8_t subevents[0];
        } __attribute__((packed));
    } ranging_data;
};

/* Pool of per-procedure ranging-data buffers.  Static array in .bss when
 * BLE_STATIC_TO_DYNAMIC is disabled; otherwise heap-allocated on first use
 * so the RAM (BLE_RAS_PROCEDURE_MEM per entry) is only occupied while the
 * RAS service is in use. */
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
static struct ranging_buffer *ranging_buffers = NULL;
#else
static struct ranging_buffer ranging_buffers[RAS_TOTAL_RD_BUFFERS];
#endif
static uint16_t ranging_buffer_alloc_seq;
static bool ranging_buffers_inited = false;

static int
ble_svc_ras_buf_pool_init(void)
{
    if (ranging_buffers_inited) {
        return 0;
    }
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ranging_buffers == NULL) {
        ranging_buffers = nimble_platform_mem_calloc(
            RAS_TOTAL_RD_BUFFERS, sizeof(*ranging_buffers));
        if (ranging_buffers == NULL) {
            MODLOG_DFLT(ERROR, "[RAS] ranging_buffers alloc failed (%d x %u B)\n",
                    RAS_TOTAL_RD_BUFFERS,
                    (unsigned)sizeof(*ranging_buffers));
            return BLE_HS_ENOMEM;
        }
    }
#endif
    for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
        ranging_buffers[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ranging_buffers[i].ranging_counter = 0;
        ranging_buffers[i].subevent_cursor = 0;
        ranging_buffers[i].refcount = 0;
        ranging_buffers[i].ready = false;
        ranging_buffers[i].busy = false;
        ranging_buffers[i].announced = false;
        ranging_buffers[i].truncated = false;
        ranging_buffers[i].steps_lost = false;
    }
    ranging_buffers_inited = true;
    return 0;
}

static int
ble_svc_ras_buf_count(uint16_t conn_handle)
{
    int count = 0;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ranging_buffers == NULL) {
        return 0;
    }
#endif
    for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
        if (ranging_buffers[i].conn_handle == conn_handle) {
            count++;
        }
    }

    return count;
}

static struct ranging_buffer *
ble_svc_ras_buf_find(uint16_t conn_handle, uint16_t ranging_counter)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    /* Pool not allocated yet; nothing stored, so nothing to find. */
    if (ranging_buffers == NULL) {
        return NULL;
    }
#endif
    for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
        if (ranging_buffers[i].conn_handle == conn_handle &&
            BLE_SVC_RAS_RANGING_COUNTER(ranging_buffers[i].ranging_counter) ==
            BLE_SVC_RAS_RANGING_COUNTER(ranging_counter)) {
            return &ranging_buffers[i];
        }
    }
    return NULL;
}

static struct ranging_buffer *
ble_svc_ras_buf_find_ready(uint16_t conn_handle, uint16_t ranging_counter)
{
    struct ranging_buffer *buf = ble_svc_ras_buf_find(conn_handle, ranging_counter);
    if (buf && buf->ready) {
        return buf;
    }
    return NULL;
}

static void
ble_svc_ras_buf_free(struct ranging_buffer *buf)
{
    MODLOG_DFLT(DEBUG, "[RAS] buffer free conn=%d counter=%d\n",
            buf->conn_handle, buf->ranging_counter);
    buf->conn_handle = BLE_HS_CONN_HANDLE_NONE;
    buf->refcount = 0;
    buf->ready = false;
    buf->busy = false;
    buf->announced = false;
    buf->truncated = false;
    buf->steps_lost = false;
    buf->subevent_cursor = 0;
}

static void
ble_svc_ras_bufs_flush(uint16_t conn_handle)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ranging_buffers == NULL) {
        return;
    }
#endif
    for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
        if (ranging_buffers[i].conn_handle == conn_handle) {
            ble_svc_ras_buf_free(&ranging_buffers[i]);
        }
    }
}

static void ble_svc_ras_indicate_rd_overwritten(uint16_t conn_handle, uint16_t counter);
static void ble_svc_ras_rt_push(uint16_t conn_handle, struct ranging_buffer *buf);

/* Called once a new procedure is known to have started on this connection,
 * which proves any procedure still marked busy will never be continued - its
 * last result carried Partial and the rest was lost (CS disabled or the config
 * removed mid-procedure, or a dropped controller event).  Such a buffer has to
 * go now: the Subevent Result Continue event carries no procedure counter, so
 * the handler picks the first busy buffer of the connection and would append
 * the new procedure's steps to the stale one.  Leaving it busy would also make
 * it unreclaimable, since the eviction passes skip busy buffers.  "keep" is the
 * buffer the new procedure is about to occupy, if one has already been chosen. */
static void
ble_svc_ras_buf_discard_busy(uint16_t conn_handle, const struct ranging_buffer *keep)
{
    for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
        if (&ranging_buffers[i] != keep &&
            ranging_buffers[i].conn_handle == conn_handle &&
            ranging_buffers[i].busy) {
            MODLOG_DFLT(WARN, "[RAS] incomplete procedure discarded conn=%d counter=%d\n",
                    conn_handle, ranging_buffers[i].ranging_counter);
            ble_svc_ras_buf_free(&ranging_buffers[i]);
        }
    }
}

static struct ranging_buffer *
ble_svc_ras_buf_alloc(uint16_t conn_handle, uint16_t ranging_counter)
{
    struct ranging_buffer *buf;

    if (ble_svc_ras_buf_pool_init() != 0) {
        return NULL;
    }

    buf = ble_svc_ras_buf_find(conn_handle, ranging_counter);
    if (buf) {
        if (buf->busy) {
            /* Legitimate multi-subevent continuation: same procedure still in
             * progress, return the buffer as-is so the caller appends data. */
            MODLOG_DFLT(DEBUG, "[RAS] buffer reuse conn=%d counter=%d\n", conn_handle,
                    BLE_SVC_RAS_RANGING_COUNTER(ranging_counter));
            return buf;
        }
        /* This procedure was finalised early because its body outgrew
         * BLE_RAS_PROCEDURE_MEM, and the controller is still reporting its
         * remaining subevents.  Drop them: re-initialising here would discard
         * the truncated body the client has already been told about and
         * announce the tail again under the same ranging counter, which the
         * client would decode as a complete procedure.  A counter that wraps
         * back to this one 4096 procedures later allocates in between, so the
         * sequence check keeps the drop to this procedure only. */
        if (buf->truncated && buf->alloc_seq == ranging_buffer_alloc_seq) {
            MODLOG_DFLT(DEBUG, "[RAS] truncated procedure, dropping subevent conn=%d counter=%d\n",
                    conn_handle, BLE_SVC_RAS_RANGING_COUNTER(ranging_counter));
            return NULL;
        }
        /* Stale completed buffer matched (likely 12-bit counter wrapped after
         * 4096 procedures, or a duplicate event from the controller).  Evict
         * and reinitialize it so the new procedure starts with clean state. */
        if (buf->refcount > 0) {
            MODLOG_DFLT(WARN, "[RAS] stale buffer in active stream conn=%d counter=%d, rejecting\n",
                    conn_handle, BLE_SVC_RAS_RANGING_COUNTER(ranging_counter));
            return NULL;
        }
        MODLOG_DFLT(WARN, "[RAS] stale buffer reuse (counter wrap?) conn=%d counter=%d, evicting\n",
                conn_handle, BLE_SVC_RAS_RANGING_COUNTER(ranging_counter));
        if (buf->announced) {
            ble_svc_ras_indicate_rd_overwritten(conn_handle, buf->ranging_counter);
        }
        ble_svc_ras_buf_discard_busy(conn_handle, buf);
        goto init_buf;
    }

    ble_svc_ras_buf_discard_busy(conn_handle, NULL);

    /* RAS_RD_BUFFERS_PER_CONN is a quota, not just a pool multiplier: the
     * eviction pass below only reclaims buffers of the requesting
     * connection, so with RAS_RRSP_MAX_CONN raised a connection that took
     * more than its share would starve the others instead of overwriting
     * its own oldest data. */
    if (ble_svc_ras_buf_count(conn_handle) < RAS_RD_BUFFERS_PER_CONN) {
        for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
            if (ranging_buffers[i].conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                buf = &ranging_buffers[i];
                goto init_buf;
            }
        }
    }

    {
        struct ranging_buffer *oldest = NULL;
        for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
            if (ranging_buffers[i].conn_handle == conn_handle &&
                ranging_buffers[i].refcount == 0 &&
                !ranging_buffers[i].busy) {
                /* Oldest-first via signed-delta compare on alloc_seq; the
                 * 12-bit ranging_counter wraps too often to compare raw. */
                if (oldest == NULL ||
                    (int16_t)(ranging_buffers[i].alloc_seq -
                              oldest->alloc_seq) < 0) {
                    oldest = &ranging_buffers[i];
                }
            }
        }
        if (oldest) {
            MODLOG_DFLT(INFO, "[RAS] evicting buffer conn=%d counter=%d for counter=%d\n",
                    conn_handle, oldest->ranging_counter,
                    BLE_SVC_RAS_RANGING_COUNTER(ranging_counter));
            /* Announcing the loss of data the client was never told about would
             * be noise, and worse: with a single ATT indication slot per
             * connection that RD Overwritten takes the slot that the incoming
             * procedure's RD Ready needs.  The client then never hears about
             * the new data, never fetches it, never frees the buffer - so the
             * next procedure evicts again and the session livelocks. */
            if (oldest->announced) {
                ble_svc_ras_indicate_rd_overwritten(conn_handle, oldest->ranging_counter);
            }
            buf = oldest;
            goto init_buf;
        }
    }

    /* Expected while the client is fetching: every buffer is claimed, so the
     * new procedure is dropped rather than announced.  Not an error path. */
    MODLOG_DFLT(DEBUG, "[RAS] no buffer available conn=%d counter=%d\n",
            conn_handle, BLE_SVC_RAS_RANGING_COUNTER(ranging_counter));
    return NULL;

init_buf:
    buf->conn_handle = conn_handle;
    buf->ranging_counter = BLE_SVC_RAS_RANGING_COUNTER(ranging_counter);
    buf->subevent_cursor = 0;
    buf->subev_hdr_offset = 0;
    buf->alloc_seq = ++ranging_buffer_alloc_seq;
    buf->refcount = 0;
    buf->ready = false;
    buf->busy = true;
    buf->announced = false;
    buf->truncated = false;
    buf->steps_lost = false;
    memset(buf->ranging_data.buf, 0, BLE_RAS_RANGING_HEADER_LEN);
    MODLOG_DFLT(DEBUG, "[RAS] buffer alloc conn=%d counter=%d\n", conn_handle,
            BLE_SVC_RAS_RANGING_COUNTER(ranging_counter));
    return buf;
}

/* Per-Connection RRSP State */
struct ble_svc_ras_rrsp {
    uint16_t conn_handle;

    /* Active buffer and streaming state */
    struct ranging_buffer *active_buf;
    uint16_t active_buf_read_cursor;
    uint16_t streaming_end_offset;
    uint8_t segment_counter;
    uint8_t seg_stall_retries;

    /* Flag bits packed into one byte. */
    uint8_t streaming           : 1;
    uint8_t timers_inited       : 1;
    uint8_t od_rd_subscribed    : 1;
    uint8_t rt_rd_subscribed    : 1;
    uint8_t cp_subscribed       : 1;
    uint8_t rd_ready_subscribed : 1;
    uint8_t rd_ow_subscribed    : 1;
    /* The client's CCCD choice for the two announcement characteristics.  Both
     * advertise NOTIFY and INDICATE, so the bit the client actually set is
     * what may be put on the wire (Core 6.0 Vol 3, Part G, Section 3.3.3.3). */
    uint8_t rd_ready_indicate   : 1;
    uint8_t rd_ow_indicate      : 1;
    uint8_t rt_mode_streaming   : 1;
    /* Indication flow control (only one indication outstanding at a time)
     * packed into a second byte. */
    uint8_t rd_ready_pending    : 1;
    uint8_t rd_ow_pending       : 1;
    uint8_t cp_pending          : 1;
    /* A ranging data segment indication is in flight.  Only ever set when the
     * client subscribed to the RD characteristic with indications alone; a
     * notification-mode stream never takes the ATT indication slot. */
    uint8_t seg_pending         : 1;
    uint8_t complete_rd_deferred: 1;
    uint8_t rsp_code_deferred   : 1;
    /* The notify bit of the client's RD CCCD writes.  RAS 2.7 requires
     * segments to go out as notifications whenever the client enabled them
     * and as indications when it subscribed with indications alone. */
    uint8_t od_rd_notify        : 1;
    uint8_t rt_rd_notify        : 1;
    /* Set while ble_svc_ras_send_segment() is on the stack.  An indication
     * that fails before it reaches the air is reported through a synchronous
     * NOTIFY_TX event *and* as the return value, so without this the segment
     * failure handler and ble_svc_ras_send_one_segment()'s caller would both
     * tear the stream down - double PROCEDURE_NOT_COMPLETED and a refcount
     * decremented twice.  The caller's paths win: they also cover the mbuf
     * allocation failures that never produce a NOTIFY_TX at all, and they are
     * the ones that know ENOMEM means retry rather than abort. */
    uint8_t seg_tx_in_call      : 1;

    uint8_t deferred_rsp_code;

    /* Signed, like the ranging header field it is copied into and like the CS
     * event it comes from.  0x7F is the RAS "not available" value. */
    int8_t selected_tx_power;

    uint16_t last_rd_ready_counter;
    uint16_t last_rd_overwritten_counter;

    struct ble_npl_callout ack_timer;
    struct ble_npl_callout seg_timer;
};

/* Per-connection RRSP state.  Same static/dynamic split as ranging_buffers. */
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
static struct ble_svc_ras_rrsp *rrsp_pool = NULL;
#else
static struct ble_svc_ras_rrsp rrsp_pool[RAS_RRSP_MAX_CONN];
#endif
static bool rrsp_pool_inited = false;

static int ble_svc_ras_send_complete_rd_rsp(struct ble_svc_ras_rrsp *rrsp);

static void
ble_svc_ras_rrsp_reset_state(struct ble_svc_ras_rrsp *rrsp)
{
    rrsp->active_buf = NULL;
    rrsp->active_buf_read_cursor = 0;
    rrsp->streaming_end_offset = 0;
    rrsp->segment_counter = 0;
    rrsp->seg_stall_retries = 0;
    rrsp->streaming = false;

    rrsp->od_rd_subscribed = false;
    rrsp->rt_rd_subscribed = false;
    rrsp->cp_subscribed = false;
    rrsp->rd_ready_subscribed = false;
    rrsp->rd_ow_subscribed = false;
    rrsp->rd_ready_indicate = false;
    rrsp->rd_ow_indicate = false;

    rrsp->rt_mode_streaming = false;
    rrsp->rd_ready_pending = false;
    rrsp->rd_ow_pending = false;
    rrsp->cp_pending = false;
    rrsp->seg_pending = false;
    rrsp->od_rd_notify = false;
    rrsp->rt_rd_notify = false;
    rrsp->seg_tx_in_call = false;
    rrsp->complete_rd_deferred = false;
    rrsp->rsp_code_deferred = false;
    rrsp->deferred_rsp_code = 0;
    rrsp->selected_tx_power = 0x7F;
    rrsp->last_rd_ready_counter = 0;
    rrsp->last_rd_overwritten_counter = 0;
}

static int
ble_svc_ras_rrsp_pool_init(void)
{
    if (rrsp_pool_inited) {
        return 0;
    }
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (rrsp_pool == NULL) {
        rrsp_pool = nimble_platform_mem_calloc(
            RAS_RRSP_MAX_CONN, sizeof(*rrsp_pool));
        if (rrsp_pool == NULL) {
            MODLOG_DFLT(ERROR, "[RAS] rrsp_pool alloc failed (%d x %u B)\n",
                    RAS_RRSP_MAX_CONN, (unsigned)sizeof(*rrsp_pool));
            return BLE_HS_ENOMEM;
        }
    }
#endif
    for (int i = 0; i < RAS_RRSP_MAX_CONN; i++) {
        rrsp_pool[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        rrsp_pool[i].timers_inited = false;
        ble_svc_ras_rrsp_reset_state(&rrsp_pool[i]);
    }
    rrsp_pool_inited = true;
    return 0;
}

static struct ble_svc_ras_rrsp *
ble_svc_ras_rrsp_find(uint16_t conn_handle)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (rrsp_pool == NULL) {
        return NULL;
    }
#endif
    for (int i = 0; i < RAS_RRSP_MAX_CONN; i++) {
        if (rrsp_pool[i].conn_handle == conn_handle) {
            return &rrsp_pool[i];
        }
    }
    return NULL;
}

static struct ble_svc_ras_rrsp *
ble_svc_ras_rrsp_alloc(uint16_t conn_handle)
{
    struct ble_svc_ras_rrsp *rrsp;

    if (ble_svc_ras_rrsp_pool_init() != 0) {
        return NULL;
    }

    rrsp = ble_svc_ras_rrsp_find(conn_handle);
    if (rrsp) {
        return rrsp;
    }

    for (int i = 0; i < RAS_RRSP_MAX_CONN; i++) {
        if (rrsp_pool[i].conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            rrsp = &rrsp_pool[i];
            rrsp->conn_handle = conn_handle;
            ble_svc_ras_rrsp_reset_state(rrsp);

            if (!rrsp->timers_inited) {
                struct ble_npl_eventq *evq = nimble_port_get_dflt_eventq();
                /* Both timers have to exist before the slot is handed out.
                 * The ACK_RD timeout is what bounds the pending-ACK state and
                 * releases the held buffer, and the segment timer is what
                 * recovers a stalled stream; a slot latched as inited with a
                 * timer that was never created would drop both silently for
                 * every connection that later lands on it.  Leave
                 * timers_inited clear on failure so the next alloc retries. */
                if (ble_npl_callout_init(&rrsp->ack_timer, evq,
                                         ble_svc_ras_ack_timer_cb, rrsp) != 0 ||
                    ble_npl_callout_init(&rrsp->seg_timer, evq,
                                         ble_svc_ras_seg_timer_cb, rrsp) != 0) {
                    ble_npl_callout_deinit(&rrsp->ack_timer);
                    ble_npl_callout_deinit(&rrsp->seg_timer);
                    rrsp->conn_handle = BLE_HS_CONN_HANDLE_NONE;
                    MODLOG_DFLT(ERROR, "[RAS] RRSP timer init failed conn=%d\n",
                            conn_handle);
                    return NULL;
                }
                rrsp->timers_inited = true;
            }

            MODLOG_DFLT(INFO, "[RAS] RRSP alloc conn=%d\n", conn_handle);
            return rrsp;
        }
    }

    MODLOG_DFLT(ERROR, "[RAS] RRSP pool full, cannot alloc conn=%d\n", conn_handle);
    return NULL;
}

/* Indication / Notification Helpers */

/* The mbuf allocation below is the only failure the callers have to undo
 * themselves, which is why they clear their pending flag on BLE_HS_ENOMEM
 * alone.  Every error returned once ble_gatts_indicate_custom is entered is
 * also reported through a NOTIFY_TX event carrying that status, and the RAS
 * GAP handler clears the flag from there, so clearing it at the call site too
 * would be redundant. */
static int
ble_svc_ras_indicate_custom(uint16_t conn_handle, uint16_t attr_handle,
                            const void *data, uint16_t len)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        MODLOG_DFLT(ERROR, "[RAS] indicate mbuf alloc failed conn=%d\n", conn_handle);
        return BLE_HS_ENOMEM;
    }
    return ble_gatts_indicate_custom(conn_handle, attr_handle, om);
}

/* Notification counterpart of the above, for the announcement characteristics
 * when the client enabled NOTIFY.  A notification is delivered once and never
 * confirmed, so nothing here occupies the ATT indication slot and the callers
 * settle the announcement as soon as the PDU is handed to the stack. */
static int
ble_svc_ras_notify_custom(uint16_t conn_handle, uint16_t attr_handle,
                          const void *data, uint16_t len)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        MODLOG_DFLT(ERROR, "[RAS] notify mbuf alloc failed conn=%d\n", conn_handle);
        return BLE_HS_ENOMEM;
    }
    return ble_gatts_notify_custom(conn_handle, attr_handle, om);
}

/* Wrap up a stream whose last segment has gone out: real-time releases the
 * buffer straight away, while on-demand hands over to the Complete_RD_RSP /
 * ACK_RD exchange. */
static void
ble_svc_ras_stream_finish(struct ble_svc_ras_rrsp *rrsp)
{
    rrsp->streaming = false;

    if (!rrsp->rt_mode_streaming) {
        int crd_rc = ble_svc_ras_send_complete_rd_rsp(rrsp);
        if (crd_rc == BLE_HS_EBUSY) {
            rrsp->complete_rd_deferred = true;
        } else {
            ble_npl_callout_reset(&rrsp->ack_timer,
                ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
        }
        return;
    }

    /* Real-time: no Complete_RD_RSP / ACK - release the buffer directly */
    MODLOG_DFLT(INFO, "[RAS] RT stream done conn=%d counter=%d total=%d\n",
            rrsp->conn_handle,
            BLE_SVC_RAS_RANGING_COUNTER(rrsp->active_buf->ranging_counter),
            rrsp->active_buf_read_cursor);
    if (rrsp->active_buf->refcount > 0) {
        rrsp->active_buf->refcount--;
    }
    ble_svc_ras_buf_free(rrsp->active_buf);
    rrsp->active_buf = NULL;
    rrsp->active_buf_read_cursor = 0;
    rrsp->rt_mode_streaming = false;

    /* A buffer that became ready while this stream was draining is left for
     * the allocator to reclaim rather than sent now.  Real-time data is only
     * worth anything while it is current, and by the time a stream ends the
     * procedure behind it has already been superseded; the next procedure to
     * complete pushes itself, so the client always gets the freshest set. */
}

/* Sends one segment: the one-octet segment header followed by the payload
 * slice, appended into one mbuf.  RAS 2.7 picks the sub-procedure: segments
 * go out as notifications whenever the client enabled them and as
 * indications when it subscribed with indications alone; an indicated
 * stream is then paced by the ATT confirmations (see seg_pending). */
static int
ble_svc_ras_send_segment(uint16_t conn_handle, uint16_t attr_handle,
                         bool indicate, const struct segment_header *hdr,
                         const void *data, uint16_t len)
{
    struct os_mbuf *om = ble_hs_mbuf_att_pkt();
    if (om == NULL) {
        MODLOG_DFLT(ERROR, "[RAS] segment mbuf alloc failed conn=%d\n", conn_handle);
        return BLE_HS_ENOMEM;
    }
    if (os_mbuf_append(om, hdr, sizeof(*hdr)) != 0 ||
        os_mbuf_append(om, data, len) != 0) {
        MODLOG_DFLT(ERROR, "[RAS] segment mbuf append failed conn=%d len=%d\n",
                conn_handle, len);
        os_mbuf_free_chain(om);
        return BLE_HS_ENOMEM;
    }
    if (indicate) {
        return ble_gatts_indicate_custom(conn_handle, attr_handle, om);
    }
    return ble_gatts_notify_custom(conn_handle, attr_handle, om);
}

static void
ble_svc_ras_indicate_rd_ready(uint16_t conn_handle, uint16_t ranging_counter)
{
    struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
    if (rrsp == NULL || !rrsp->rd_ready_subscribed) {
        return;
    }
    /* RAS 2.7.1 / 3.4: the server shall not indicate Ranging Data Ready in
     * Real-time Ranging Data mode.  The mode is enforced here, at the one place
     * every announcement passes through, rather than at each caller - the CCCD
     * filter only stops On-demand and Real-time being enabled together, so a
     * client is free to hold the RD Ready CCCD open while it streams, and a
     * buffer that finishes during a stream is left unannounced for the
     * allocator and would otherwise be offered by the deferred scan. */
    if (rrsp->rt_rd_subscribed) {
        return;
    }
    /* Announcing while a fetch is in flight invites a GET_RD that the handler
     * below is guaranteed to answer with Server Busy - segment notifications
     * do not occupy the ATT indication slot, so RD Ready escapes mid-transfer.
     * Worse, the buffer is then marked announced and evicted by the following
     * procedure, so the client's retry gets No Records Found.  The counter is
     * not lost: ble_svc_ras_announce_newest_ready() picks it up when the transfer
     * ends.  Same condition as the Server Busy check in handle_get_rd(). */
    if (rrsp->streaming || rrsp->active_buf != NULL) {
        MODLOG_DFLT(DEBUG, "[RAS] RD Ready deferred until transfer ends conn=%d counter=%d\n",
                conn_handle, BLE_SVC_RAS_RANGING_COUNTER(ranging_counter));
        return;
    }
    /* If an indication is already in flight the counter is not announced now:
     * ATT allows only one outstanding indication (Core Spec Vol 3, Part F,
     * Section 3.3.2) and there is no indication queue.  The buffer stays in the
     * pool and ble_svc_ras_announce_newest_ready() offers it once that
     * indication completes. */
    if (rrsp->rd_ready_pending || rrsp->rd_ow_pending || rrsp->cp_pending ||
        rrsp->seg_pending) {
        MODLOG_DFLT(WARN, "[RAS] RD Ready skipped, indication already pending conn=%d\n",
                conn_handle);
        return;
    }

    uint16_t counter = BLE_SVC_RAS_RANGING_COUNTER(ranging_counter);
    rrsp->last_rd_ready_counter = counter;

    /* The characteristic carries both properties, so the sub-procedure follows
     * the bit the client set in the CCCD (Core Spec Vol 3, Part G, Section
     * 3.3.3.3).  An indication is preferred and is what the announcement
     * protocol is built around - the confirmation releases rd_ready_pending and
     * is what marks the buffer announced, which in turn decides whether a later
     * eviction is worth an RD Overwritten.  A notify-only client gets the same
     * counter without that round trip, so the buffer is marked announced as
     * soon as the PDU is accepted and no pending flag is armed. */
    if (!rrsp->rd_ready_indicate) {
        MODLOG_DFLT(INFO, "[RAS] RD Ready notify conn=%d counter=%d\n", conn_handle, counter);
        if (ble_svc_ras_notify_custom(conn_handle, ble_svc_ras_rd_val_handle,
                                      &counter, sizeof(counter)) == 0) {
            struct ranging_buffer *buf = ble_svc_ras_buf_find(conn_handle, counter);
            if (buf) {
                buf->announced = true;
            }
        }
        return;
    }

    MODLOG_DFLT(INFO, "[RAS] RD Ready indicate conn=%d counter=%d\n", conn_handle, counter);
    rrsp->rd_ready_pending = true;
    int rc = ble_svc_ras_indicate_custom(conn_handle, ble_svc_ras_rd_val_handle,
                                         &counter, sizeof(counter));
    if (rc == BLE_HS_ENOMEM) {
        MODLOG_DFLT(ERROR, "[RAS] RD Ready indicate failed rc=%d conn=%d\n", rc, conn_handle);
        rrsp->rd_ready_pending = false;
        return;
    }
    if (rc == 0) {
        struct ranging_buffer *buf = ble_svc_ras_buf_find(conn_handle, counter);
        if (buf) {
            buf->announced = true;
        }
    }
}

/* Called when an indication completes: announces the freshest procedure the
 * client has not been told about yet, which is how counters deferred by the
 * transfer-in-flight gate above get back on the wire.  An announcement the
 * client never acted on must not suppress the next one: a client that pairs
 * its own step data with one specific ranging counter ignores every other
 * counter, so withholding the one it is waiting for stalls the session until
 * the link drops.  Freshest rather than oldest because that is the procedure
 * a client following the live stream is holding. */
static void
ble_svc_ras_announce_newest_ready(struct ble_svc_ras_rrsp *rrsp)
{
    struct ranging_buffer *newest = NULL;

    if (rrsp->streaming || rrsp->active_buf != NULL ||
        rrsp->rd_ready_pending || rrsp->rd_ow_pending || rrsp->cp_pending ||
        rrsp->seg_pending ||
        rrsp->complete_rd_deferred || rrsp->rsp_code_deferred) {
        return;
    }

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ranging_buffers == NULL) {
        return;
    }
#endif
    for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
        struct ranging_buffer *buf = &ranging_buffers[i];

        if (buf->conn_handle != rrsp->conn_handle || !buf->ready ||
            buf->announced) {
            continue;
        }
        /* Signed-delta compare on alloc_seq, as in the eviction pass: the
         * 12-bit ranging_counter wraps too often to compare raw. */
        if (newest == NULL || (int16_t)(buf->alloc_seq - newest->alloc_seq) > 0) {
            newest = buf;
        }
    }

    /* RD Ready belongs to the on-demand flow: a real-time client leaves the
     * RD Ready CCCD unsubscribed (RAS 2.7.1 / 3.4 - the server shall not
     * indicate RD Ready in Real-time Ranging Data mode), so the
     * rd_ready_subscribed gate inside ble_svc_ras_indicate_rd_ready() is the
     * mode check and the scan above stays mode-agnostic. */
    if (newest) {
        ble_svc_ras_indicate_rd_ready(rrsp->conn_handle, newest->ranging_counter);
    }
}

static void
ble_svc_ras_indicate_rd_overwritten(uint16_t conn_handle, uint16_t counter)
{
    struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
    if (rrsp == NULL || !rrsp->rd_ow_subscribed) {
        return;
    }
    /* BLE ATT allows only one indication in flight per connection (Core Spec
     * Vol 3, Part F, Section 3.3.2).  Sending while another indication is pending
     * would overwrite the GATT server's indicate_val_handle, causing the
     * prior indication's completion event to be lost and permanently blocking
     * future indications on this connection. */
    if (rrsp->rd_ready_pending || rrsp->rd_ow_pending || rrsp->cp_pending ||
        rrsp->seg_pending) {
        MODLOG_DFLT(WARN, "[RAS] RD Overwritten skipped, indication pending conn=%d\n",
                conn_handle);
        return;
    }
    uint16_t c = BLE_SVC_RAS_RANGING_COUNTER(counter);
    rrsp->last_rd_overwritten_counter = c;

    /* Same rule as RD Ready: the characteristic offers both properties and the
     * CCCD bit the client set picks the sub-procedure (Core Spec Vol 3, Part G,
     * Section 3.3.3.3).  The single 2-byte counter tells the client a procedure
     * it may still be fetching has been evicted, and an indication is preferred
     * so the loss is acknowledged; a notify-only client is told once. */
    if (!rrsp->rd_ow_indicate) {
        MODLOG_DFLT(INFO, "[RAS] RD Overwritten notify conn=%d counter=%d\n", conn_handle, c);
        ble_svc_ras_notify_custom(conn_handle, ble_svc_ras_rd_ov_val_handle,
                                  &c, sizeof(c));
        return;
    }

    MODLOG_DFLT(INFO, "[RAS] RD Overwritten indicate conn=%d counter=%d\n", conn_handle, c);
    rrsp->rd_ow_pending = true;
    int rc = ble_svc_ras_indicate_custom(conn_handle, ble_svc_ras_rd_ov_val_handle,
                                         &c, sizeof(c));
    if (rc == BLE_HS_ENOMEM) {
        MODLOG_DFLT(ERROR, "[RAS] RD Overwritten indicate failed rc=%d conn=%d\n", rc, conn_handle);
        rrsp->rd_ow_pending = false;
    }
}

static void
ble_svc_ras_send_cp_rsp_code(uint16_t conn_handle, uint8_t rsp_code)
{
    struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
    uint8_t buf[2];
    buf[0] = BLE_SVC_RAS_CP_RSP_OPCODE_RSP_CODE;
    buf[1] = rsp_code;
    MODLOG_DFLT(DEBUG, "[RAS] CP Response Code conn=%d code=0x%02x\n", conn_handle, rsp_code);

    /* The CCCD decides, and it is checked here rather than only where a
     * command arrives: the timers and the NOTIFY_TX completion can reach this
     * helper long after the write that started the exchange, and the client is
     * free to clear the CP CCCD in between.  Indicating to a client that has
     * not enabled indications is what Core 6.0 Vol 3, Part G, Section 4.11.1
     * rules out, so every producer of a response code funnels through this one
     * test instead of repeating it. */
    if (rrsp == NULL || !rrsp->cp_subscribed) {
        MODLOG_DFLT(DEBUG, "[RAS] CP RSP 0x%02x dropped, CP indications not enabled "
                    "conn=%d\n", rsp_code, conn_handle);
        return;
    }

    if (rrsp->rd_ready_pending || rrsp->rd_ow_pending ||
        rrsp->cp_pending || rrsp->seg_pending) {
        MODLOG_DFLT(INFO, "[RAS] CP RSP deferred, indication pending conn=%d code=0x%02x\n",
                    conn_handle, rsp_code);
        rrsp->rsp_code_deferred = true;
        rrsp->deferred_rsp_code = rsp_code;
        return;
    }
    rrsp->cp_pending = true;
    int rc = ble_svc_ras_indicate_custom(conn_handle, ble_svc_ras_cp_val_handle,
                                buf, sizeof(buf));
    if (rc == BLE_HS_ENOMEM) {
        rrsp->cp_pending = false;
    }
}

static int
ble_svc_ras_send_complete_rd_rsp(struct ble_svc_ras_rrsp *rrsp)
{
    if (rrsp->active_buf == NULL) {
        return BLE_HS_EINVAL;
    }
    /* Same CCCD gate as ble_svc_ras_send_cp_rsp_code(), for the same reason:
     * the segment timer and the NOTIFY_TX completion reach this long after the
     * Get Ranging Data write, and a client may clear the CP CCCD in between.
     * Core 6.0 Vol 3, Part G, Section 4.11.1 rules out indicating to a client
     * that has not enabled indications. */
    if (!rrsp->cp_subscribed) {
        MODLOG_DFLT(DEBUG, "[RAS] Complete_RD_RSP dropped, CP indications not enabled "
                    "conn=%d\n", rrsp->conn_handle);
        return BLE_HS_EINVAL;
    }
    uint8_t buf[3];
    uint16_t counter = BLE_SVC_RAS_RANGING_COUNTER(rrsp->active_buf->ranging_counter);
    buf[0] = BLE_SVC_RAS_CP_RSP_OPCODE_COMPLETE_RD_RSP;
    memcpy(&buf[1], &counter, sizeof(counter));

    if (rrsp->rd_ready_pending || rrsp->rd_ow_pending || rrsp->cp_pending ||
        rrsp->seg_pending) {
        MODLOG_DFLT(WARN, "[RAS] Complete_RD_RSP skipped, indication pending conn=%d\n",
                rrsp->conn_handle);
        return BLE_HS_EBUSY;
    }
    MODLOG_DFLT(INFO, "[RAS] Complete_RD_RSP conn=%d counter=%d, "
            "all %d segments (%d bytes) sent to client, waiting for ACK\n",
            rrsp->conn_handle, counter,
            rrsp->segment_counter,
            rrsp->active_buf_read_cursor);
    rrsp->cp_pending = true;
    int rc = ble_svc_ras_indicate_custom(rrsp->conn_handle, ble_svc_ras_cp_val_handle,
                                buf, sizeof(buf));
    if (rc == BLE_HS_ENOMEM) {
        rrsp->cp_pending = false;
    }
    return rc;
}

/* True once RAS_MAX_QUEUED_SEGS or more packets are waiting on controller
 * credits, the shared mbuf pool is nearly empty, or a real-time stream already
 * has RAS_RT_MAX_INFLIGHT_PKTS packets the controller has not finished sending.
 * The queue walk stops at the bound. */
static bool
ble_svc_ras_conn_tx_backlogged(const struct ble_svc_ras_rrsp *rrsp)
{
    uint16_t conn_handle = rrsp->conn_handle;
    struct os_mbuf_pkthdr *omp;
    struct ble_hs_conn *conn;
    int outstanding = 0;
    int depth = 0;

    /* RAS_MAX_QUEUED_SEGS bounds what this connection has handed to the
     * controller, which says nothing about the rest of the host drawing on the
     * same pool - notifications, ATT responses and the CS event traffic all
     * allocate from msys, so the pool can be near empty while the segment
     * stream still looks idle.  Checking the pool directly reaches the same
     * stall-retry path an allocation failure would, only before a partly built
     * segment has to be discarded. */
    if (os_msys_num_free() <= RAS_MIN_FREE_MBUFS) {
        return true;
    }

    ble_hs_lock();
    conn = ble_hs_conn_find(conn_handle);
    if (conn != NULL) {
        STAILQ_FOREACH(omp, &conn->bhc_tx_q, omp_next) {
            if (++depth >= RAS_MAX_QUEUED_SEGS) {
                break;
            }
        }
        outstanding = conn->bhc_outstanding_pkts;
    }
    ble_hs_unlock();

    /* Real-time segments are paced against what the controller has actually
     * finished sending, not just against what the host has managed to hand
     * over.  A notification reports success once it is queued, so without this
     * the stream keeps producing while the controller still owns every one of
     * its few ACL buffers, and the burst that follows has nowhere to go.  The
     * indication variant of this stream gets the same pacing for free from the
     * one-outstanding-transaction rule in Core Vol 3 Part F 3.3.2, and is the
     * mode that survives a long ranging session, so match its behaviour rather
     * than relying on a timer that cannot know the controller's depth. */
    if (rrsp->rt_mode_streaming && outstanding >= RAS_RT_MAX_INFLIGHT_PKTS) {
        return true;
    }

    return depth >= RAS_MAX_QUEUED_SEGS;
}

/* Segment Send (Step 3: send RD segments; when done, Step 4: Complete_RD_RSP + ack timer) */
static int
ble_svc_ras_send_one_segment(struct ble_svc_ras_rrsp *rrsp)
{
    if (rrsp->active_buf == NULL) {
        return BLE_HS_EINVAL;
    }

    /* Drop the segment before allocating for it if the link is not draining.
     * ble_svc_ras_seg_timer_cb already treats ENOMEM as a stall, so this reuses the
     * retry-then-abandon path. */
    if (ble_svc_ras_conn_tx_backlogged(rrsp)) {
        if (rrsp->seg_stall_retries == 0) {
            MODLOG_DFLT(WARN, "[RAS] tx queue backed up, pausing stream conn=%d seg=%d\n",
                    rrsp->conn_handle, rrsp->segment_counter);
        }
        return BLE_HS_ENOMEM;
    }

    uint16_t mtu = ble_att_mtu(rrsp->conn_handle);
    if (mtu < BLE_ATT_MTU_DFLT) {
        mtu = BLE_ATT_MTU_DFLT;
    }
    /* RAS 3.2.2: a segment carries at most ATT_MTU - 4 octets of ranging data,
     * that is the notification payload less the one-octet segment header.  The
     * ATT MTU is therefore what sizes a segment, so it has to be chosen so the
     * notification still fits one LL data PDU; a stream of segments that each
     * need ACL fragmentation backs the transmit queue up faster than it drains.
     * See CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU in the channel sounding examples. */
    uint16_t max_data = mtu - 3 - sizeof(struct segment_header);

    uint16_t total_len = BLE_RAS_RANGING_HEADER_LEN + rrsp->active_buf->subevent_cursor;
    uint16_t end_offset = rrsp->streaming_end_offset;
    if (end_offset == 0 || end_offset > total_len) {
        end_offset = total_len;
    }

    /* All segments sent */
    if (rrsp->active_buf_read_cursor >= end_offset) {
        ble_svc_ras_stream_finish(rrsp);
        return 0;
    }

    uint16_t remaining = end_offset - rrsp->active_buf_read_cursor;
    bool is_first = (rrsp->active_buf_read_cursor == 0);
    uint16_t pull_bytes = MIN(max_data, remaining);
    /* Use end_offset (stream limit), not total_len, so capped stream and last segment get last=1 */
    bool is_last = (rrsp->active_buf_read_cursor + pull_bytes >= end_offset);

    /* One byte of segment header, then the payload slice.  The counter is 6 bits
     * wide and wraps, which is what the client uses to detect a missing segment.
     */
    struct segment_header hdr;
    hdr.first_seg = is_first;
    hdr.last_seg = is_last;
    hdr.seg_counter = rrsp->segment_counter & BLE_SVC_RAS_SEG_COUNTER_MASK;

    MODLOG_DFLT(DEBUG, "[RAS] segment send conn=%d seg=%d first=%d last=%d bytes=%d\n",
            rrsp->conn_handle, hdr.seg_counter, is_first, is_last, pull_bytes);

    uint16_t attr_handle;
    bool use_ind;

    if (rrsp->rt_mode_streaming) {
        attr_handle = ble_svc_ras_rt_rd_val_handle;
        use_ind = !rrsp->rt_rd_notify;
    } else {
        attr_handle = ble_svc_ras_od_rd_val_handle;
        use_ind = !rrsp->od_rd_notify;
    }

    /* An indicated segment has to wait for the single ATT indication slot
     * like every other indication.  ENOMEM reuses the stall/retry path, and
     * the completion handler of whichever indication holds the slot kicks
     * seg_timer the moment it frees up. */
    if (use_ind &&
        (rrsp->rd_ready_pending || rrsp->rd_ow_pending || rrsp->cp_pending ||
         rrsp->seg_pending)) {
        return BLE_HS_ENOMEM;
    }

    const uint8_t *seg_data =
        &rrsp->active_buf->ranging_data.buf[rrsp->active_buf_read_cursor];
    rrsp->seg_tx_in_call = true;
    int rc = ble_svc_ras_send_segment(rrsp->conn_handle, attr_handle, use_ind,
                                      &hdr, seg_data, pull_bytes);
    rrsp->seg_tx_in_call = false;
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "[RAS] segment send failed rc=%d conn=%d seg=%d\n",
                rc, rrsp->conn_handle, hdr.seg_counter);
        return rc;
    }

    if (use_ind) {
        rrsp->seg_pending = true;
    }
    rrsp->active_buf_read_cursor += pull_bytes;
    rrsp->segment_counter = (rrsp->segment_counter + 1) & BLE_SVC_RAS_SEG_COUNTER_MASK;

    /* Wrap the stream up here rather than waiting for one more seg_timer tick
     * just to discover there is nothing left to send.  That tick costs a whole
     * RAS_SEG_DELAY_MS on every procedure, which real-time mode cannot spare. */
    if (rrsp->active_buf_read_cursor >= end_offset) {
        ble_svc_ras_stream_finish(rrsp);
    }

    return 0;
}

/* Timer Callbacks */
static void
ble_svc_ras_ack_timer_cb(struct ble_npl_event *ev)
{
    struct ble_svc_ras_rrsp *rrsp = ble_npl_event_get_arg(ev);

    /* ble_npl_callout_stop() drains an expiry that is already queued on the
     * eventq, but it cannot cancel one that is in flight, so a stale event can
     * still be dequeued after the stop.  Two guards reject it: the slot may
     * have been freed by ble_svc_ras_on_disconnect and reused for a new
     * connection (conn_handle NONE), or the callout may have been re-armed for
     * a newer ACK wait, which supersedes this expiry.  A genuine one-shot
     * expiry leaves the callout inactive, so nothing real is suppressed. */
    if (rrsp->conn_handle == BLE_HS_CONN_HANDLE_NONE ||
        ble_npl_callout_is_active(&rrsp->ack_timer)) {
        return;
    }

    /* Stale-event guard: if ABORT or ACK_RD already cleaned up (active_buf
     * set to NULL), this is a queued event that raced with the cleanup.
     * Bail out to avoid sending a spurious PROCEDURE_NOT_COMPLETED after
     * the SUCCESS indication was already sent. */
    if (rrsp->active_buf == NULL) {
        return;
    }

    MODLOG_DFLT(WARN, "[RAS] ACK timeout fired conn=%d\n", rrsp->conn_handle);

    if (rrsp->active_buf->refcount > 0) {
        rrsp->active_buf->refcount--;
    }
    rrsp->active_buf = NULL;
    rrsp->active_buf_read_cursor = 0;
    rrsp->streaming = false;
    rrsp->complete_rd_deferred = false;

    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle,
                                  BLE_SVC_RAS_CP_RSP_PROCEDURE_NOT_COMPLETED);
}

static uint32_t
ble_svc_ras_seg_delay_ticks(const struct ble_svc_ras_rrsp *rrsp)
{
    return ble_npl_time_ms_to_ticks32(rrsp->rt_mode_streaming ?
                                      RAS_RT_SEG_DELAY_MS : RAS_SEG_DELAY_MS);
}

static void
ble_svc_ras_seg_timer_cb(struct ble_npl_event *ev)
{
    struct ble_svc_ras_rrsp *rrsp = ble_npl_event_get_arg(ev);

    /* Same freed-slot guard as ble_svc_ras_ack_timer_cb.  No re-arm guard is needed here:
     * this callout only paces the next segment, so an expiry that slipped past
     * a stop simply sends it a few ticks early, and the arm that superseded it
     * sends the following one. */
    if (rrsp->conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    if (!rrsp->streaming || rrsp->active_buf == NULL) {
        return;
    }

    /* An indicated segment is waiting for its ATT confirmation; the
     * NOTIFY_TX completion handler kicks this callout the moment the slot
     * frees, so a cadence tick landing in that window would only burn
     * stall retries. */
    if (rrsp->seg_pending) {
        return;
    }

    int rc = ble_svc_ras_send_one_segment(rrsp);
    if (rc == BLE_HS_ENOMEM &&
        ++rrsp->seg_stall_retries < RAS_SEG_MAX_STALL_RETRIES) {
        ble_npl_callout_reset(&rrsp->seg_timer,
                              ble_npl_time_ms_to_ticks32(RAS_SEG_RETRY_MS));
    } else if (rc != 0) {
        MODLOG_DFLT(ERROR, "[RAS] seg send %s rc=%d, stopping stream conn=%d\n",
                rc == BLE_HS_ENOMEM ? "stalled" : "fatal error",
                rc, rrsp->conn_handle);
        bool was_rt = rrsp->rt_mode_streaming;
        rrsp->streaming = false;
        rrsp->rt_mode_streaming = false;
        if (rrsp->active_buf) {
            if (rrsp->active_buf->refcount > 0) {
                rrsp->active_buf->refcount--;
            }
            if (was_rt) {
                ble_svc_ras_buf_free(rrsp->active_buf);
            }
            rrsp->active_buf = NULL;
        }
        rrsp->active_buf_read_cursor = 0;
        rrsp->complete_rd_deferred = false;
        if (!was_rt) {
            ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle,
                                          BLE_SVC_RAS_CP_RSP_PROCEDURE_NOT_COMPLETED);
        }
    } else {
        rrsp->seg_stall_retries = 0;
        if (rrsp->streaming) {
            /* This reset can overwrite the shorter retry armed by an rt_push
             * that ran from inside send_one_segment.  Harmless: under memory
             * pressure that retry just happens a little later. */
            ble_npl_callout_reset(&rrsp->seg_timer,
                                  ble_svc_ras_seg_delay_ticks(rrsp));
        }
    }
}

/*
 * Real-time Ranging Data: auto-push segments when buffer becomes ready
 * and the client has subscribed to Real-time RD (RAP 4.4.1).
 * No RD Ready indication, no GET_RD, no ACK_RD.
 */
static void
ble_svc_ras_rt_push(uint16_t conn_handle, struct ranging_buffer *buf)
{
    struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
    if (rrsp == NULL) {
        return;
    }

    if (rrsp->streaming || rrsp->active_buf != NULL) {
        /* A transfer is already in flight, so leave it alone.  Segments are
         * paced well inside the 1 s per-segment tolerance of RAP 4.4.1.1, so
         * an in-flight stream is still healthy however long it has been
         * running; cutting it short would restart the rolling segment counter
         * mid-set and a client that only accepts in-order segments discards
         * the whole procedure.  The stream-done path picks this buffer up. */
        MODLOG_DFLT(DEBUG, "[RAS] RT push deferred, %s conn=%d counter=%d\n",
                rrsp->streaming ? "already streaming" : "active_buf pending ACK",
                conn_handle, BLE_SVC_RAS_RANGING_COUNTER(buf->ranging_counter));
        return;
    }

    buf->refcount++;
    rrsp->active_buf = buf;
    rrsp->active_buf_read_cursor = 0;
    rrsp->segment_counter = 0;
    rrsp->seg_stall_retries = 0;
    rrsp->streaming = true;
    rrsp->rt_mode_streaming = true;
    rrsp->streaming_end_offset = BLE_RAS_RANGING_HEADER_LEN + buf->subevent_cursor;

    MODLOG_DFLT(INFO, "[RAS] RT push start conn=%d counter=%d len=%d\n",
            conn_handle,
            BLE_SVC_RAS_RANGING_COUNTER(buf->ranging_counter),
            rrsp->streaming_end_offset);

    int rc = ble_svc_ras_send_one_segment(rrsp);
    if (rc == BLE_HS_ENOMEM) {
        ble_npl_callout_reset(&rrsp->seg_timer,
                              ble_npl_time_ms_to_ticks32(RAS_SEG_RETRY_MS));
    } else if (rc != 0) {
        MODLOG_DFLT(ERROR, "[RAS] RT push first segment failed rc=%d conn=%d\n", rc, conn_handle);
        rrsp->streaming = false;
        rrsp->rt_mode_streaming = false;
        buf->refcount--;
        ble_svc_ras_buf_free(buf);
        rrsp->active_buf = NULL;
    } else if (rrsp->streaming) {
        ble_npl_callout_reset(&rrsp->seg_timer,
                              ble_svc_ras_seg_delay_ticks(rrsp));
    }
}

static int ble_svc_ras_gap_event(struct ble_gap_event *event);

static int
ble_svc_ras_gap_listener_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    return ble_svc_ras_gap_event(event);
}

/*
 * CP Command Processing (reflector side)
 *
 * RD transfer flow: Step 1 RD Ready (we send) -> Step 2 GET_RD (client)
 * -> Step 3 we send segments -> Step 4 we send Complete_RD_RSP, wait for ACK_RD
 * -> Step 5 we receive ACK_RD -> Step 6 we send RSP_CODE success.
 */
static void
ble_svc_ras_handle_get_rd(struct ble_svc_ras_rrsp *rrsp,
                          const uint8_t *params, uint16_t params_len)
{
    /* Step 2: Client sent GET_RD; validate and start sending segments (step 3).
     * The only parameter is the 2-octet Ranging Counter (RAS Spec Table 3.10),
     * so any other length is an invalid command. */
    if (params_len != 2) {
        MODLOG_DFLT(ERROR, "[RAS] GET_RD invalid param len=%d conn=%d\n",
                params_len, rrsp->conn_handle);
        ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_INVALID_PARAMETER);
        return;
    }

    if (!rrsp->od_rd_subscribed) {
        MODLOG_DFLT(ERROR, "[RAS] GET_RD rejected: client not subscribed to On-demand RD conn=%d\n",
                rrsp->conn_handle);
        ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_INVALID_PARAMETER);
        return;
    }

    if (rrsp->streaming || rrsp->active_buf != NULL) {
        MODLOG_DFLT(ERROR, "[RAS] GET_RD rejected: server busy conn=%d\n", rrsp->conn_handle);
        ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_SERVER_BUSY);
        return;
    }

    uint16_t req_counter;
    memcpy(&req_counter, params, sizeof(req_counter));
    req_counter = BLE_SVC_RAS_RANGING_COUNTER(req_counter);

    MODLOG_DFLT(INFO, "[RAS] GET_RD conn=%d counter=%d\n", rrsp->conn_handle, req_counter);

    struct ranging_buffer *buf = ble_svc_ras_buf_find_ready(rrsp->conn_handle, req_counter);
    if (buf == NULL) {
        MODLOG_DFLT(ERROR, "[RAS] GET_RD no records found conn=%d counter=%d\n",
                rrsp->conn_handle, req_counter);
        ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_NO_RECORDS_FOUND);
        return;
    }

    /* No RSP_CODE on this path: a successful GET_RD is answered by the segment
     * stream followed by Complete_RD_RSP.  Sending SUCCESS first would also
     * consume the single ATT indication slot that Complete_RD_RSP needs. */
    buf->refcount++;
    rrsp->active_buf = buf;
    rrsp->active_buf_read_cursor = 0;
    rrsp->segment_counter = 0;
    rrsp->seg_stall_retries = 0;
    rrsp->streaming = true;
    rrsp->rt_mode_streaming = false;
    rrsp->complete_rd_deferred = false;
    rrsp->streaming_end_offset = BLE_RAS_RANGING_HEADER_LEN + buf->subevent_cursor;

    if (rrsp->rd_ready_pending || rrsp->rd_ow_pending || rrsp->cp_pending) {
        /* Indication not yet confirmed - the first segment will
         * be sent once the RD Ready EDONE fires (see GAP event handler). */
        MODLOG_DFLT(DEBUG, "[RAS] GET_RD: deferring first segment until RD Ready EDONE conn=%d\n",
                rrsp->conn_handle);
        return;
    }

    int rc = ble_svc_ras_send_one_segment(rrsp);
    if (rc == BLE_HS_ENOMEM) {
        ble_npl_callout_reset(&rrsp->seg_timer,
                              ble_npl_time_ms_to_ticks32(RAS_SEG_RETRY_MS));
    } else if (rc != 0) {
        MODLOG_DFLT(ERROR, "[RAS] GET_RD first segment failed rc=%d conn=%d\n",
                rc, rrsp->conn_handle);
        rrsp->streaming = false;
        buf->refcount--;
        rrsp->active_buf = NULL;
        rrsp->active_buf_read_cursor = 0;
        ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_PROCEDURE_NOT_COMPLETED);
    } else if (rrsp->streaming) {
        ble_npl_callout_reset(&rrsp->seg_timer,
                              ble_npl_time_ms_to_ticks32(RAS_SEG_DELAY_MS));
    }
}

/* Step 5: Client sent ACK_RD; stop ack timer, free buffer, Step 6: send RSP_CODE success. */
static void
ble_svc_ras_handle_ack_rd(struct ble_svc_ras_rrsp *rrsp,
                          const uint8_t *params, uint16_t params_len)
{
    /* RAS 3.3.1.2: the only parameter is the 2-octet Ranging Counter, so the
     * length must be exactly 2 - trailing bytes are an Invalid Parameter. */
    if (params_len != 2) {
        MODLOG_DFLT(ERROR, "[RAS] ACK_RD invalid param len=%d conn=%d\n",
                params_len, rrsp->conn_handle);
        ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_INVALID_PARAMETER);
        return;
    }

    uint16_t ack_counter;
    memcpy(&ack_counter, params, sizeof(ack_counter));
    ack_counter = BLE_SVC_RAS_RANGING_COUNTER(ack_counter);

    MODLOG_DFLT(INFO, "[RAS] ACK_RD conn=%d counter=%d, "
            "client confirmed receipt, transfer SUCCESS\n",
            rrsp->conn_handle, ack_counter);

    ble_npl_callout_stop(&rrsp->ack_timer);
    ble_npl_callout_stop(&rrsp->seg_timer);

    if (rrsp->active_buf &&
        BLE_SVC_RAS_RANGING_COUNTER(rrsp->active_buf->ranging_counter) == ack_counter) {
        if (rrsp->active_buf->refcount > 0) {
            rrsp->active_buf->refcount--;
        }
        ble_svc_ras_buf_free(rrsp->active_buf);
        rrsp->active_buf = NULL;
    } else {
        /* Counter mismatch: release the active streaming reference before
         * freeing the acknowledged buffer so neither leaks a refcount. */
        if (rrsp->active_buf) {
            if (rrsp->active_buf->refcount > 0) {
                rrsp->active_buf->refcount--;
            }
            rrsp->active_buf = NULL;
        }
        /* ble_svc_ras_buf_find (not _find_ready) is intentional: ACK_RD
         * acknowledges data the client already received, so a buffer that has
         * been served and is no longer ready still has to be cleaned up.  A
         * busy buffer is different - it is still being filled and was never
         * served, so acknowledging it is bogus and freeing it would truncate
         * the procedure in flight. */
        struct ranging_buffer *buf = ble_svc_ras_buf_find(rrsp->conn_handle, ack_counter);
        if (buf && !buf->busy) {
            ble_svc_ras_buf_free(buf);
        } else {
            /* RAS Spec: ACK_RD with unknown counter -> No Records Found */
            rrsp->active_buf_read_cursor = 0;
            rrsp->streaming = false;
            rrsp->complete_rd_deferred = false;
            ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle,
                                         BLE_SVC_RAS_CP_RSP_NO_RECORDS_FOUND);
            return;
        }
    }

    rrsp->active_buf_read_cursor = 0;
    rrsp->streaming = false;
    rrsp->complete_rd_deferred = false;

    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_SUCCESS);
}

static void
ble_svc_ras_handle_abort(struct ble_svc_ras_rrsp *rrsp, uint16_t params_len)
{
    /* Abort Operation takes no parameters (RAS Spec Table 3.10), so extra
     * octets make the command invalid.  Report it and leave the transfer in
     * progress alone: an invalid command must have no side effects. */
    if (params_len != 0) {
        MODLOG_DFLT(ERROR, "[RAS] ABORT invalid param len=%d conn=%d\n",
                params_len, rrsp->conn_handle);
        ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_INVALID_PARAMETER);
        return;
    }

    MODLOG_DFLT(INFO, "[RAS] ABORT conn=%d\n", rrsp->conn_handle);

    ble_npl_callout_stop(&rrsp->seg_timer);
    ble_npl_callout_stop(&rrsp->ack_timer);

    if (rrsp->active_buf) {
        if (rrsp->active_buf->refcount > 0) {
            rrsp->active_buf->refcount--;
        }
        rrsp->active_buf = NULL;
    }
    rrsp->active_buf_read_cursor = 0;
    rrsp->streaming = false;
    rrsp->complete_rd_deferred = false;

    /* If a Complete_RD_RSP indication is still in flight (cp_pending), the
     * guard in ble_svc_ras_send_cp_rsp_code defers this SUCCESS indication
     * until that one completes.  ABORT is write-no-rsp, so the client does
     * not block on the response, and server state is already cleaned up
     * above. */
    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, BLE_SVC_RAS_CP_RSP_SUCCESS);
}


static int
ble_svc_ras_process_cp_write(uint16_t conn_handle, const uint8_t *data, uint16_t len)
{
    /* Resolve (or lazily allocate) connection state first: it is what makes a
     * CP response indication possible for the validation failures below, the
     * way ble_svc_ras_handle_get_rd and the other handlers report errors by
     * indication.  Where it cannot be obtained there is nothing to answer
     * with, so that one case is dropped instead - see below. */
    struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
    if (rrsp == NULL) {
        rrsp = ble_svc_ras_rrsp_alloc(conn_handle);
        if (rrsp == NULL) {
            MODLOG_DFLT(ERROR, "[RAS] CP write: no RRSP for conn=%d\n", conn_handle);
            /* Drop the command rather than answer it.  Every indication this
             * server sends is gated on per-connection state: cp_subscribed
             * records the CCCD the client wrote, and cp_pending serialises the
             * one outstanding indication ATT permits (Core 6.0 Vol 3, Part F,
             * Section 3.3.2).  With no rrsp neither is knowable, so a response
             * from here could reach a client that never enabled the CP CCCD,
             * which Core 6.0 Vol 3, Part G, Section 4.11.1 forbids; and since
             * the CP is write-without-response, a peer writing repeatedly
             * would stack untracked indications that overwrite each other's
             * completion.  Silence matches the cp_subscribed gate below and
             * keeps this path allocation-free, which matters because the way
             * to reach it is an exhausted pool or a failed pool init. */
            return 0;
        }
    }

    if (len < BLE_SVC_RAS_CP_CMD_OPCODE_LEN) {
        MODLOG_DFLT(ERROR, "[RAS] CP write too short len=%d conn=%d\n", len, conn_handle);
        /* Notify the client if it has already subscribed to CP indications;
         * otherwise the write is silently ignored (WRITE_NO_RSP semantics). */
        if (rrsp->cp_subscribed) {
            ble_svc_ras_send_cp_rsp_code(conn_handle, BLE_SVC_RAS_CP_RSP_INVALID_PARAMETER);
        }
        return 0;
    }

    if (!rrsp->cp_subscribed) {
        MODLOG_DFLT(ERROR, "[RAS] CP write rejected: CP indications not subscribed conn=%d\n",
                conn_handle);
        /* The CP characteristic uses WRITE_NO_RSP (ATT Write Command).  Per BT Core
         * Spec Vol 3 Part F Section 3.4.5.3, the server MUST NOT send ATT_ERROR_RSP for a
         * Write Command, so any non-zero return here is silently dropped by the ATT
         * layer - the client receives no feedback.  Return 0 (silent ignore) as
         * required by the spec; the client must subscribe to CP indications before
         * issuing any CP command. */
        return 0;
    }

    uint8_t opcode = data[BLE_SVC_RAS_CP_CMD_OPCODE_OFFSET];
    const uint8_t *params = data + BLE_SVC_RAS_CP_CMD_PARAMS_OFFSET;
    uint16_t params_len = len - BLE_SVC_RAS_CP_CMD_OPCODE_LEN;

    MODLOG_DFLT(DEBUG, "[RAS] CP write conn=%d opcode=0x%02x params_len=%d\n",
            conn_handle, opcode, params_len);

    /* RAS 3.2.3.1 / RAP 4.4.1: In real-time mode the CP is "limited to Ranging
     * Data filtering configuration only."  Every other Op Code - including
     * GET_RD, ACK_RD, ABORT and RETRIEVE_LOST_RD_SEGMENTS - is outside that
     * scope and shall be rejected with Op Code Not Supported.
     * Checked before the streaming-busy guard so that RT-disallowed opcodes
     * get the semantically correct NOT_SUPPORTED (permanent) rather than
     * SERVER_BUSY (transient / retry-later). */
    if (rrsp->rt_rd_subscribed &&
        (opcode == BLE_SVC_RAS_CP_OPCODE_GET_RD ||
         opcode == BLE_SVC_RAS_CP_OPCODE_ACK_RD ||
         opcode == BLE_SVC_RAS_CP_OPCODE_ABORT_OP ||
         opcode == BLE_SVC_RAS_CP_OPCODE_RETRIEVE_LOST_RD_SEGMENTS)) {
        MODLOG_DFLT(WARN, "[RAS] CP opcode 0x%02x rejected: real-time mode active conn=%d\n",
                opcode, conn_handle);
        ble_svc_ras_send_cp_rsp_code(conn_handle, BLE_SVC_RAS_CP_RSP_OP_CODE_NOT_SUPPORTED);
        return 0;
    }

    /* RAS 3.3.3: an opcode that cannot be served while a transfer is streaming
     * is answered Server Busy, which is the correct code for this server - the
     * condition really is transient, and it is the response that tells the
     * client to retry once the transfer ends.  The list here is the set of
     * exemptions, not a ranking of opcodes:
     * ABORT_OP and ACK_RD act on the transfer in progress, so they have to be
     * let through; the client may ACK before Complete_RD_RSP goes out.
     * SET_FILTER is exempt because it is the one opcode whose rejection is
     * unconditional - it is refused identically whether or not a transfer is
     * running, so routing it through the busy check would offer a retry that
     * can never succeed.  RETRIEVE_LOST_RD_SEGMENTS is deliberately not in
     * this list: it is meaningful only against a transfer, so while one is
     * streaming Server Busy is the accurate answer, and outside a transfer it
     * falls through to Op Code Not Supported below. */
    if (opcode != BLE_SVC_RAS_CP_OPCODE_ABORT_OP && opcode != BLE_SVC_RAS_CP_OPCODE_ACK_RD &&
        opcode != BLE_SVC_RAS_CP_OPCODE_SET_FILTER && rrsp->streaming) {
        MODLOG_DFLT(ERROR, "[RAS] CP busy: opcode=0x%02x during streaming conn=%d\n",
                opcode, conn_handle);
        ble_svc_ras_send_cp_rsp_code(conn_handle, BLE_SVC_RAS_CP_RSP_SERVER_BUSY);
        return 0;
    }

    /* ACK_RD is exempt from the busy guard above because the client may ACK
     * before Complete_RD_RSP goes out, but acknowledging data that has not
     * been sent yet is premature.  The read cursor is the discriminator: it
     * reaches streaming_end_offset as the last segment is handed to the ATT
     * layer, while 'streaming' stays set until the next seg_timer tick runs
     * the completion bookkeeping.  So cursor < end_offset means segments are
     * still outstanding, and only that case is Server Busy. */
    if (opcode == BLE_SVC_RAS_CP_OPCODE_ACK_RD && rrsp->streaming && rrsp->active_buf &&
        rrsp->active_buf_read_cursor < rrsp->streaming_end_offset) {
        MODLOG_DFLT(ERROR, "[RAS] CP busy: ACK_RD mid-transfer (cursor=%d end=%d) conn=%d\n",
                rrsp->active_buf_read_cursor, rrsp->streaming_end_offset, conn_handle);
        ble_svc_ras_send_cp_rsp_code(conn_handle, BLE_SVC_RAS_CP_RSP_SERVER_BUSY);
        return 0;
    }

    switch (opcode) {
    case BLE_SVC_RAS_CP_OPCODE_GET_RD:
        ble_svc_ras_handle_get_rd(rrsp, params, params_len);
        break;
    case BLE_SVC_RAS_CP_OPCODE_ACK_RD:
        ble_svc_ras_handle_ack_rd(rrsp, params, params_len);
        break;
    case BLE_SVC_RAS_CP_OPCODE_ABORT_OP:
        ble_svc_ras_handle_abort(rrsp, params_len);
        break;
    /* Both are optional features that this server does not advertise in RAS
     * Features, so RAS 3.3.3 requires Op Code Not Supported rather than a
     * Success that would imply the request had taken effect.
     */
    case BLE_SVC_RAS_CP_OPCODE_RETRIEVE_LOST_RD_SEGMENTS:
    case BLE_SVC_RAS_CP_OPCODE_SET_FILTER:
        MODLOG_DFLT(WARN, "[RAS] CP opcode 0x%02x not supported conn=%d\n",
                    opcode, conn_handle);
        ble_svc_ras_send_cp_rsp_code(conn_handle, BLE_SVC_RAS_CP_RSP_OP_CODE_NOT_SUPPORTED);
        break;
    default:
        MODLOG_DFLT(ERROR, "[RAS] CP unsupported opcode=0x%02x conn=%d\n", opcode, conn_handle);
        ble_svc_ras_send_cp_rsp_code(conn_handle, BLE_SVC_RAS_CP_RSP_OP_CODE_NOT_SUPPORTED);
        break;
    }
    return 0;
}

/* GATT Access Callback */
static int
ble_svc_ras_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (attr_handle == ble_svc_ras_feat_val_handle) {
            MODLOG_DFLT(DEBUG, "[RAS] read Features conn=%d val=0x%08" PRIx32 "\n",
                    conn_handle, ble_svc_ras_feat_val);
            rc = os_mbuf_append(ctxt->om, &ble_svc_ras_feat_val,
                                sizeof(ble_svc_ras_feat_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == ble_svc_ras_rd_val_handle) {
            struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
            uint16_t val = rrsp ? rrsp->last_rd_ready_counter : 0;
            rc = os_mbuf_append(ctxt->om, &val, sizeof(val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == ble_svc_ras_rd_ov_val_handle) {
            struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
            uint16_t val = rrsp ? rrsp->last_rd_overwritten_counter : 0;
            rc = os_mbuf_append(ctxt->om, &val, sizeof(val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (attr_handle == ble_svc_ras_cp_val_handle) {
            uint8_t cp_buf[BLE_SVC_RAS_CP_WRITE_MAX_LEN];
            uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);

            /* A write longer than the characteristic's maximum is an ATT-layer
             * length violation, so it is answered at that layer rather than
             * with a RAS Response Code: RAS Response Codes report parameter
             * errors inside a well-formed command, and the per-opcode
             * validators below own that case. */
            if (om_len > sizeof(cp_buf)) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            rc = ble_hs_mbuf_to_flat(ctxt->om, cp_buf, om_len, NULL);
            if (rc != 0) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            return ble_svc_ras_process_cp_write(conn_handle, cp_buf, om_len);
        }
        return BLE_ATT_ERR_UNLIKELY;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/*
 * GAP Event Handler (CCCD Subscription Tracking + indication-driven
 * segment pipelining)
 */
static int
ble_svc_ras_gap_event(struct ble_gap_event *event)
{
    if (event->type == BLE_GAP_EVENT_NOTIFY_TX &&
        event->notify_tx.indication &&
        event->notify_tx.attr_handle == ble_svc_ras_rd_val_handle) {

        struct ble_svc_ras_rrsp *rrsp =
            ble_svc_ras_rrsp_find(event->notify_tx.conn_handle);
        if (rrsp) {
            if (event->notify_tx.status == BLE_HS_EDONE) {
                /* ACK received - indication cycle complete. */
                rrsp->rd_ready_pending = false;
                if (rrsp->streaming) {
                    ble_npl_callout_reset(&rrsp->seg_timer, 1);
                }
                if (rrsp->complete_rd_deferred && rrsp->active_buf) {
                    rrsp->complete_rd_deferred = false;
                    int crd_rc = ble_svc_ras_send_complete_rd_rsp(rrsp);
                    if (crd_rc == BLE_HS_EBUSY) {
                        rrsp->complete_rd_deferred = true;
                    } else {
                        ble_npl_callout_reset(&rrsp->ack_timer,
                            ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
                    }
                } else if (rrsp->rsp_code_deferred) {
                    uint8_t code = rrsp->deferred_rsp_code;
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, code);
                }
                ble_svc_ras_announce_newest_ready(rrsp);
            } else if (event->notify_tx.status == 0) {
                /* ble_gatts_indicate_custom fires a NOTIFY_TX with status=0
                 * when the indication PDU is sent.  The ACK will arrive later
                 * as EDONE.  Do NOT clear rd_ready_pending here - the
                 * indication is still in flight. */
            } else {
                rrsp->rd_ready_pending = false;
                MODLOG_DFLT(WARN, "[RAS] RD Ready indication failed status=%d conn=%d\n",
                        event->notify_tx.status, event->notify_tx.conn_handle);
                if (rrsp->streaming && rrsp->active_buf &&
                    rrsp->active_buf_read_cursor == 0 &&
                    !rrsp->rt_mode_streaming) {
                    ble_npl_callout_stop(&rrsp->seg_timer);
                    ble_npl_callout_stop(&rrsp->ack_timer);
                    rrsp->streaming = false;
                    rrsp->complete_rd_deferred = false;
                    if (rrsp->active_buf->refcount > 0) {
                        rrsp->active_buf->refcount--;
                    }
                    rrsp->active_buf = NULL;
                    rrsp->active_buf_read_cursor = 0;
                    /* PROCEDURE_NOT_COMPLETED supersedes a Response Code that
                     * was deferred behind this indication, otherwise its flush
                     * at the CP completion below would put a second Response
                     * Code on the wire for one client command. */
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle,
                                                  BLE_SVC_RAS_CP_RSP_PROCEDURE_NOT_COMPLETED);
                } else if (rrsp->complete_rd_deferred && rrsp->active_buf) {
                    rrsp->complete_rd_deferred = false;
                    int crd_rc = ble_svc_ras_send_complete_rd_rsp(rrsp);
                    if (crd_rc == BLE_HS_EBUSY) {
                        rrsp->complete_rd_deferred = true;
                    } else {
                        ble_npl_callout_reset(&rrsp->ack_timer,
                            ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
                    }
                    /* A deferred Response Code is kept here on purpose: the
                     * Complete_RD_RSP just queued is a different Op Code, so
                     * flushing the Response Code after it answers the client's
                     * command rather than duplicating a response. */
                } else if (rrsp->rsp_code_deferred) {
                    /* Nothing else is queued, so flush now instead of leaving
                     * the client waiting for the next indication to carry it. */
                    uint8_t code = rrsp->deferred_rsp_code;
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, code);
                }
            }
        }
        return 0;
    }

    /* RD Overwritten indication completion - mirrors the RD Ready handler
     * above: clear rd_ow_pending and resume any deferred GET_RD stream. */
    if (event->type == BLE_GAP_EVENT_NOTIFY_TX &&
        event->notify_tx.indication &&
        event->notify_tx.attr_handle == ble_svc_ras_rd_ov_val_handle) {

        struct ble_svc_ras_rrsp *rrsp =
            ble_svc_ras_rrsp_find(event->notify_tx.conn_handle);
        if (rrsp) {
            if (event->notify_tx.status == BLE_HS_EDONE) {
                rrsp->rd_ow_pending = false;
                if (rrsp->streaming) {
                    ble_npl_callout_reset(&rrsp->seg_timer, 1);
                }
                if (rrsp->complete_rd_deferred && rrsp->active_buf) {
                    rrsp->complete_rd_deferred = false;
                    int crd_rc = ble_svc_ras_send_complete_rd_rsp(rrsp);
                    if (crd_rc == BLE_HS_EBUSY) {
                        rrsp->complete_rd_deferred = true;
                    } else {
                        ble_npl_callout_reset(&rrsp->ack_timer,
                            ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
                    }
                } else if (rrsp->rsp_code_deferred) {
                    uint8_t code = rrsp->deferred_rsp_code;
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, code);
                }
                ble_svc_ras_announce_newest_ready(rrsp);
            } else if (event->notify_tx.status != 0) {
                rrsp->rd_ow_pending = false;
                if (rrsp->streaming && rrsp->active_buf &&
                    rrsp->active_buf_read_cursor == 0 &&
                    !rrsp->rt_mode_streaming) {
                    ble_npl_callout_stop(&rrsp->seg_timer);
                    ble_npl_callout_stop(&rrsp->ack_timer);
                    rrsp->streaming = false;
                    rrsp->complete_rd_deferred = false;
                    if (rrsp->active_buf->refcount > 0) {
                        rrsp->active_buf->refcount--;
                    }
                    rrsp->active_buf = NULL;
                    rrsp->active_buf_read_cursor = 0;
                    /* Supersedes a deferred Response Code, see the RD Ready
                     * failure handler above. */
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle,
                                                  BLE_SVC_RAS_CP_RSP_PROCEDURE_NOT_COMPLETED);
                } else if (rrsp->complete_rd_deferred && rrsp->active_buf) {
                    rrsp->complete_rd_deferred = false;
                    int crd_rc = ble_svc_ras_send_complete_rd_rsp(rrsp);
                    if (crd_rc == BLE_HS_EBUSY) {
                        rrsp->complete_rd_deferred = true;
                    } else {
                        ble_npl_callout_reset(&rrsp->ack_timer,
                            ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
                    }
                } else if (rrsp->rsp_code_deferred) {
                    uint8_t code = rrsp->deferred_rsp_code;
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, code);
                }
            }
        }
        return 0;
    }

    /* CP indication completion - clear cp_pending and resume any
     * deferred stream or Complete_RD_RSP (mirrors RD Ready handler). */
    if (event->type == BLE_GAP_EVENT_NOTIFY_TX &&
        event->notify_tx.indication &&
        event->notify_tx.attr_handle == ble_svc_ras_cp_val_handle) {

        struct ble_svc_ras_rrsp *rrsp =
            ble_svc_ras_rrsp_find(event->notify_tx.conn_handle);
        if (rrsp) {
            if (event->notify_tx.status == BLE_HS_EDONE) {
                rrsp->cp_pending = false;
                if (rrsp->streaming) {
                    ble_npl_callout_reset(&rrsp->seg_timer, 1);
                }
                if (rrsp->complete_rd_deferred && rrsp->active_buf) {
                    rrsp->complete_rd_deferred = false;
                    int crd_rc = ble_svc_ras_send_complete_rd_rsp(rrsp);
                    if (crd_rc == BLE_HS_EBUSY) {
                        rrsp->complete_rd_deferred = true;
                    } else {
                        ble_npl_callout_reset(&rrsp->ack_timer,
                            ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
                    }
                } else if (rrsp->rsp_code_deferred) {
                    uint8_t code = rrsp->deferred_rsp_code;
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, code);
                }
                ble_svc_ras_announce_newest_ready(rrsp);
            } else if (event->notify_tx.status != 0) {
                rrsp->cp_pending = false;
                /* Dropped rather than flushed as in the RD Ready and RD
                 * Overwritten failure handlers above: the CP indication itself
                 * failed, so another Response Code on the same handle would
                 * only fail the same way. */
                rrsp->rsp_code_deferred = false;
                rrsp->deferred_rsp_code = 0;
                if (rrsp->streaming && rrsp->active_buf &&
                    rrsp->active_buf_read_cursor == 0 &&
                    !rrsp->rt_mode_streaming) {
                    ble_npl_callout_stop(&rrsp->seg_timer);
                    ble_npl_callout_stop(&rrsp->ack_timer);
                    rrsp->streaming = false;
                    rrsp->complete_rd_deferred = false;
                    if (rrsp->active_buf->refcount > 0) {
                        rrsp->active_buf->refcount--;
                    }
                    rrsp->active_buf = NULL;
                    rrsp->active_buf_read_cursor = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle,
                                                  BLE_SVC_RAS_CP_RSP_PROCEDURE_NOT_COMPLETED);
                } else if (rrsp->complete_rd_deferred && rrsp->active_buf) {
                    rrsp->complete_rd_deferred = false;
                    int crd_rc = ble_svc_ras_send_complete_rd_rsp(rrsp);
                    if (crd_rc == BLE_HS_EBUSY) {
                        rrsp->complete_rd_deferred = true;
                    } else {
                        ble_npl_callout_reset(&rrsp->ack_timer,
                            ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
                    }
                }
            }
        }
        return 0;
    }

    /* Ranging data segment indication completion.  Only reached when the
     * client subscribed to the RD characteristic with indications alone
     * (see ble_svc_ras_send_one_segment); the confirmations are what pace
     * such a stream, one segment per ATT round trip. */
    if (event->type == BLE_GAP_EVENT_NOTIFY_TX &&
        event->notify_tx.indication &&
        (event->notify_tx.attr_handle == ble_svc_ras_od_rd_val_handle ||
         event->notify_tx.attr_handle == ble_svc_ras_rt_rd_val_handle)) {

        struct ble_svc_ras_rrsp *rrsp =
            ble_svc_ras_rrsp_find(event->notify_tx.conn_handle);
        if (rrsp && rrsp->seg_tx_in_call) {
            /* Delivered from inside ble_gatts_indicate_custom(), which returns
             * the same status to ble_svc_ras_send_one_segment() - see
             * seg_tx_in_call.  Let the caller own it, so ENOMEM keeps reaching
             * the stall/retry path and a fatal error is reported once. */
            return 0;
        }
        if (rrsp) {
            if (event->notify_tx.status == BLE_HS_EDONE) {
                rrsp->seg_pending = false;
                if (rrsp->streaming) {
                    ble_npl_callout_reset(&rrsp->seg_timer, 1);
                }
                if (rrsp->complete_rd_deferred && rrsp->active_buf) {
                    rrsp->complete_rd_deferred = false;
                    int crd_rc = ble_svc_ras_send_complete_rd_rsp(rrsp);
                    if (crd_rc == BLE_HS_EBUSY) {
                        rrsp->complete_rd_deferred = true;
                    } else {
                        ble_npl_callout_reset(&rrsp->ack_timer,
                            ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
                    }
                } else if (rrsp->rsp_code_deferred) {
                    uint8_t code = rrsp->deferred_rsp_code;
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, code);
                }
                ble_svc_ras_announce_newest_ready(rrsp);
            } else if (event->notify_tx.status != 0) {
                /* The segment never reached the client (or the ATT
                 * transaction timed out), so the transfer cannot complete;
                 * tear the stream down the same way as the seg_timer fatal
                 * path.  The attribute handle tells the mode apart - the
                 * rt_mode_streaming flag is already false when the failure
                 * hits the trailing segment of a finished stream. */
                bool was_rt = (event->notify_tx.attr_handle ==
                               ble_svc_ras_rt_rd_val_handle);
                bool had_buf = (rrsp->active_buf != NULL);

                MODLOG_DFLT(ERROR, "[RAS] segment indication failed status=%d conn=%d\n",
                        event->notify_tx.status, event->notify_tx.conn_handle);
                rrsp->seg_pending = false;
                ble_npl_callout_stop(&rrsp->seg_timer);
                ble_npl_callout_stop(&rrsp->ack_timer);
                rrsp->streaming = false;
                rrsp->rt_mode_streaming = false;
                rrsp->complete_rd_deferred = false;
                if (rrsp->active_buf) {
                    if (rrsp->active_buf->refcount > 0) {
                        rrsp->active_buf->refcount--;
                    }
                    if (was_rt) {
                        ble_svc_ras_buf_free(rrsp->active_buf);
                    }
                    rrsp->active_buf = NULL;
                }
                rrsp->active_buf_read_cursor = 0;
                if (!was_rt && had_buf) {
                    /* Supersedes a deferred Response Code, see the RD Ready
                     * failure handler above. */
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle,
                                                  BLE_SVC_RAS_CP_RSP_PROCEDURE_NOT_COMPLETED);
                } else if (rrsp->rsp_code_deferred) {
                    uint8_t code = rrsp->deferred_rsp_code;
                    rrsp->rsp_code_deferred = false;
                    rrsp->deferred_rsp_code = 0;
                    ble_svc_ras_send_cp_rsp_code(rrsp->conn_handle, code);
                }
            }
        }
        return 0;
    }

    if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        ble_svc_ras_on_disconnect(event->disconnect.conn.conn_handle);
        return 0;
    }

    if (event->type != BLE_GAP_EVENT_SUBSCRIBE) {
        return 0;
    }

    uint16_t conn_handle = event->subscribe.conn_handle;
    uint16_t attr_handle = event->subscribe.attr_handle;
    bool subscribed = event->subscribe.cur_notify || event->subscribe.cur_indicate;

    if (attr_handle != ble_svc_ras_od_rd_val_handle &&
        attr_handle != ble_svc_ras_rt_rd_val_handle &&
        attr_handle != ble_svc_ras_cp_val_handle &&
        attr_handle != ble_svc_ras_rd_val_handle &&
        attr_handle != ble_svc_ras_rd_ov_val_handle) {
        return 0;
    }

    struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
    if (rrsp == NULL) {
        rrsp = ble_svc_ras_rrsp_alloc(conn_handle);
        if (rrsp == NULL) {
            return 0;
        }
    }

    if (attr_handle == ble_svc_ras_od_rd_val_handle) {
        if (subscribed && rrsp->rt_rd_subscribed) {
            MODLOG_DFLT(WARN, "[RAS] CCCD On-demand RD while Real-time active conn=%d\n",
                    conn_handle);
        }
        rrsp->od_rd_subscribed = subscribed;
        rrsp->od_rd_notify = event->subscribe.cur_notify;
        MODLOG_DFLT(INFO, "[RAS] CCCD On-demand RD %s conn=%d notify=%d indicate=%d\n",
                subscribed ? "enabled" : "disabled", conn_handle,
                event->subscribe.cur_notify, event->subscribe.cur_indicate);
        if (!subscribed) {
            /* Check active_buf (not just streaming) to also cover the
             * pending-ACK state where streaming is false but a buffer
             * is still held awaiting the ACK_RD response. */
            if (!rrsp->rt_mode_streaming && rrsp->active_buf) {
                ble_npl_callout_stop(&rrsp->seg_timer);
                ble_npl_callout_stop(&rrsp->ack_timer);
                rrsp->streaming = false;
                rrsp->complete_rd_deferred = false;
                if (rrsp->active_buf->refcount > 0) {
                    rrsp->active_buf->refcount--;
                }
                rrsp->active_buf = NULL;
                rrsp->active_buf_read_cursor = 0;
                MODLOG_DFLT(WARN, "[RAS] OD unsubscribe during active transfer, "
                        "teardown conn=%d\n", conn_handle);
            }
        }
    } else if (attr_handle == ble_svc_ras_rt_rd_val_handle) {
        if (subscribed && rrsp->od_rd_subscribed) {
            MODLOG_DFLT(WARN, "[RAS] CCCD Real-time RD while On-demand active conn=%d\n",
                    conn_handle);
        }
        rrsp->rt_rd_subscribed = subscribed;
        rrsp->rt_rd_notify = event->subscribe.cur_notify;
        MODLOG_DFLT(INFO, "[RAS] CCCD Real-time RD %s conn=%d notify=%d indicate=%d\n",
                subscribed ? "enabled" : "disabled", conn_handle,
                event->subscribe.cur_notify, event->subscribe.cur_indicate);
        if (!subscribed) {
            if (rrsp->streaming && rrsp->rt_mode_streaming) {
                ble_npl_callout_stop(&rrsp->seg_timer);
                rrsp->streaming = false;
                rrsp->rt_mode_streaming = false;
                rrsp->complete_rd_deferred = false;
                if (rrsp->active_buf) {
                    if (rrsp->active_buf->refcount > 0) {
                        rrsp->active_buf->refcount--;
                    }
                    ble_svc_ras_buf_free(rrsp->active_buf);
                    rrsp->active_buf = NULL;
                }
                rrsp->active_buf_read_cursor = 0;
                MODLOG_DFLT(WARN, "[RAS] RT unsubscribe during active stream, "
                        "teardown conn=%d\n", conn_handle);
            }
        }
    } else if (attr_handle == ble_svc_ras_cp_val_handle) {
        rrsp->cp_subscribed = subscribed;
        if (!subscribed) {
            /* A response held back for a busy indication slot has nowhere to go
             * once indications are off, and flushing it later would put an
             * indication on the wire the client no longer accepts. */
            rrsp->rsp_code_deferred = false;
            rrsp->deferred_rsp_code = 0;

            /* Dropping a deferred Complete_RD_RSP also drops the event that
             * would have started the ACK_RD timeout, and that timeout is the
             * only thing that releases a buffer waiting to be acknowledged -
             * while it is held, RD Ready stays silent and GET_RD answers
             * Server Busy.  Start it here so the pending-ACK state stays
             * bounded, exactly as ble_svc_ras_stream_finish() does when it
             * finds the Control Point already unsubscribed. */
            if (rrsp->complete_rd_deferred && rrsp->active_buf &&
                !rrsp->rt_mode_streaming) {
                ble_npl_callout_reset(&rrsp->ack_timer,
                    ble_npl_time_ms_to_ticks32(BLE_SVC_RAS_CP_ACK_DATA_TIMEOUT_MS));
            }
            rrsp->complete_rd_deferred = false;
        }
        MODLOG_DFLT(INFO, "[RAS] CCCD Control Point %s conn=%d\n",
                subscribed ? "enabled" : "disabled", conn_handle);
    } else if (attr_handle == ble_svc_ras_rd_val_handle) {
        rrsp->rd_ready_subscribed = subscribed;
        /* Which of the two the client enabled decides how the announcement is
         * sent; see ble_svc_ras_indicate_rd_ready(). */
        rrsp->rd_ready_indicate = event->subscribe.cur_indicate;
        MODLOG_DFLT(INFO, "[RAS] CCCD RD Ready %s conn=%d\n",
                subscribed ? "enabled" : "disabled", conn_handle);
    } else if (attr_handle == ble_svc_ras_rd_ov_val_handle) {
        rrsp->rd_ow_subscribed = subscribed;
        rrsp->rd_ow_indicate = event->subscribe.cur_indicate;
        MODLOG_DFLT(INFO, "[RAS] CCCD RD Overwritten %s conn=%d\n",
                subscribed ? "enabled" : "disabled", conn_handle);
    }

    return 0;
}

/* Mark a body that did not fit BLE_RAS_PROCEDURE_MEM.  The done statuses are
 * otherwise copied straight from the controller, so a truncated body would
 * still claim the procedure completed and the client would silently decode
 * fewer steps than the header promises.  subevent_truncated tells whether
 * steps of this subevent were dropped, or only later subevents. */
static void
ble_svc_ras_mark_truncated(struct subevent_header *sub_hdr, bool subevent_truncated)
{
    sub_hdr->ranging_done_status = BLE_HCI_LE_CS_PROCEDURE_DONE_STATUS_ABORTED;
    sub_hdr->ranging_abort_reason = BLE_HCI_LE_CS_ABORT_REASON_UNSPECIFIED;

    if (subevent_truncated) {
        sub_hdr->subevent_done_status = BLE_HCI_LE_CS_SUBEVENT_DONE_STATUS_ABORTED;
        sub_hdr->subevent_abort_reason = BLE_HCI_LE_CS_ABORT_REASON_UNSPECIFIED;
    }
}

/* Auto-Store CS Event Handler (RRSP side) */
int
ble_svc_ras_cs_event_handler(struct ble_cs_event *event, void *arg)
{
    struct ranging_buffer *buf;
    uint16_t conn_handle;

    switch (event->type) {
    case BLE_CS_EVENT_SUBEVENT_RESULT: {
        conn_handle = event->subev_result.conn_handle;

        /* Only look up existing RRSP state; allocation happens on subscribe.
         * A missing RRSP is not an error - the CS buffer is still filled
         * so it can be served when the client subscribes later. */
        (void)ble_svc_ras_rrsp_find(conn_handle);

        uint16_t proc_counter = BLE_SVC_RAS_RANGING_COUNTER(event->subev_result.procedure_counter);

        MODLOG_DFLT(DEBUG, "[RAS] CS subevent_result conn=%d proc=%d steps=%d "
                "done_status=%d sub_done=%d\n",
                conn_handle, proc_counter,
                event->subev_result.num_steps_reported,
                event->subev_result.procedure_done_status,
                event->subev_result.subevent_done_status);

        buf = ble_svc_ras_buf_alloc(conn_handle, proc_counter);
        if (buf == NULL) {
            MODLOG_DFLT(DEBUG, "[RAS] cs_event: no buffer for conn=%d proc=%d\n",
                    conn_handle, proc_counter);
            return 0;
        }

        /* Ranging header (written once per procedure, on first subevent) */
        if (buf->subevent_cursor == 0) {
            buf->ranging_data.ranging_header.ranging_counter = proc_counter;
            buf->ranging_data.ranging_header.config_id =
                event->subev_result.config_id;
            {
                struct ble_svc_ras_rrsp *rrsp_tx = ble_svc_ras_rrsp_find(conn_handle);
                buf->ranging_data.ranging_header.selected_tx_power =
                    rrsp_tx ? rrsp_tx->selected_tx_power : 0x7F;
            }
            {
                /* num_antenna_paths comes straight off the transport, so clamp
                 * before shifting: 4 is both the maximum the spec allows and the
                 * width of the mask field, and a larger value would shift by at
                 * least the width of an int. */
                uint8_t n_ap = event->subev_result.num_antenna_paths;

                if (n_ap > BLE_RAS_MAX_ANTENNA_PATHS) {
                    MODLOG_DFLT(ERROR, "[RAS] cs_event: num_antenna_paths=%d conn=%d\n",
                            n_ap, conn_handle);
                    n_ap = BLE_RAS_MAX_ANTENNA_PATHS;
                }
                buf->ranging_data.ranging_header.antenna_paths_mask =
                    (1u << n_ap) - 1u;
            }
            buf->ranging_data.ranging_header.rfu = 0;
        }

        /* Subevent header */
        if (buf->subevent_cursor + sizeof(struct subevent_header) >
            BLE_RAS_PROCEDURE_MEM - BLE_RAS_RANGING_HEADER_LEN) {
            MODLOG_DFLT(ERROR, "[RAS] cs_event: buffer overflow (subevent header) conn=%d, "
                    "forcing buffer done\n", conn_handle);
            /* This subevent and every later one are dropped; the subevents
             * already stored are intact, so only the procedure is aborted. */
            ble_svc_ras_mark_truncated((struct subevent_header *)
                               (buf->ranging_data.subevents +
                                buf->subev_hdr_offset), false);
            buf->ready = true;
            buf->busy = false;
            buf->truncated = true;
            {
                struct ble_svc_ras_rrsp *rrsp_tmp = ble_svc_ras_rrsp_find(conn_handle);
                if (rrsp_tmp && rrsp_tmp->rt_rd_subscribed) {
                    ble_svc_ras_rt_push(conn_handle, buf);
                } else {
                    ble_svc_ras_indicate_rd_ready(conn_handle, proc_counter);
                }
            }
            break;
        }

        buf->subev_hdr_offset = buf->subevent_cursor;
        /* A new header owns its own record of dropped steps; a loss in the
         * previous subevent has already been marked on the header it belongs
         * to and must not follow this one. */
        buf->steps_lost = false;
        struct subevent_header *sub_hdr =
            (struct subevent_header *)(buf->ranging_data.subevents + buf->subevent_cursor);
        buf->subevent_cursor += sizeof(struct subevent_header);

        sub_hdr->start_acl_conn_event =
            event->subev_result.start_acl_conn_event_counter;
        sub_hdr->freq_compensation = event->subev_result.frequency_compensation;
        sub_hdr->ranging_done_status = event->subev_result.procedure_done_status;
        sub_hdr->subevent_done_status = event->subev_result.subevent_done_status;
        /* HCI packs Procedure_Abort_Reason in the low nibble and
         * Subevent_Abort_Reason in the high nibble of abort_reason. */
        sub_hdr->ranging_abort_reason =
            event->subev_result.abort_reason & 0x0F;
        sub_hdr->subevent_abort_reason =
            (event->subev_result.abort_reason >> 4) & 0x0F;
        sub_hdr->ref_power_level = event->subev_result.reference_power_level;
        /* Step data: walk packed variable-length entries via pointer arithmetic;
         * steps[i] indexing is invalid because sizeof(cs_steps_data) excludes
         * the flexible-array data[] member.  Track steps_written separately so
         * the subevent header reflects only what was actually stored. */
        int steps_written = 0;
        const uint8_t *step_ptr = (const uint8_t *)event->subev_result.steps;
        const uint8_t *steps_end = step_ptr + event->subev_result.steps_len;
        for (int i = 0; i < event->subev_result.num_steps_reported; i++) {
            const struct cs_steps_data *step =
                (const struct cs_steps_data *)step_ptr;
            uint16_t needed;

            /* Bound the walk on the event's own step data rather than relying on
             * the producer having validated data_len. */
            if (step_ptr + sizeof(*step) > steps_end ||
                step_ptr + sizeof(*step) + step->data_len > steps_end) {
                MODLOG_DFLT(ERROR, "[RAS] cs_event: step %d out of bounds conn=%d\n",
                        i, conn_handle);
                break;
            }

            needed = BLE_RAS_STEP_MODE_LEN + step->data_len;

            if (buf->subevent_cursor + needed >
                BLE_RAS_PROCEDURE_MEM - BLE_RAS_RANGING_HEADER_LEN) {
                MODLOG_DFLT(ERROR, "[RAS] cs_event: buffer overflow (step %d) conn=%d\n",
                        i, conn_handle);
                break;
            }

            buf->ranging_data.subevents[buf->subevent_cursor] = step->mode;
            buf->subevent_cursor += BLE_RAS_STEP_MODE_LEN;
            memcpy(&buf->ranging_data.subevents[buf->subevent_cursor],
                   step->data, step->data_len);
            buf->subevent_cursor += step->data_len;
            step_ptr += sizeof(struct cs_steps_data) + step->data_len;
            steps_written++;
        }
        sub_hdr->num_steps_reported = (uint8_t)steps_written;
        if (steps_written < event->subev_result.num_steps_reported) {
            buf->steps_lost = true;
            ble_svc_ras_mark_truncated(sub_hdr, true);
        }

        if (event->subev_result.procedure_done_status ==
            BLE_HCI_LE_CS_PROCEDURE_DONE_STATUS_COMPLETE ||
            event->subev_result.procedure_done_status ==
            BLE_HCI_LE_CS_PROCEDURE_DONE_STATUS_ABORTED) {
            buf->ready = true;
            buf->busy = false;
            MODLOG_DFLT(DEBUG, "[RAS] buffer ready conn=%d counter=%d total_len=%d done=%d\n",
                    conn_handle, proc_counter,
                    BLE_RAS_RANGING_HEADER_LEN + buf->subevent_cursor,
                    event->subev_result.procedure_done_status);
            {
                struct ble_svc_ras_rrsp *rrsp_tmp = ble_svc_ras_rrsp_find(conn_handle);
                if (rrsp_tmp && rrsp_tmp->rt_rd_subscribed) {
                    ble_svc_ras_rt_push(conn_handle, buf);
                } else {
                    ble_svc_ras_indicate_rd_ready(conn_handle, proc_counter);
                }
            }
        }
        break;
    }

    case BLE_CS_EVENT_SUBEVENT_RESULT_CONTINUE: {
        conn_handle = event->subev_result_continue.conn_handle;

        MODLOG_DFLT(DEBUG, "[RAS] CS subevent_result_continue conn=%d steps=%d done_status=%d\n",
                conn_handle,
                event->subev_result_continue.num_steps_reported,
                event->subev_result_continue.procedure_done_status);

        /* Find the active buffer for this connection. */
        buf = NULL;
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        if (ranging_buffers == NULL) {
            MODLOG_DFLT(ERROR, "[RAS] cs_continue: pool not initialized conn=%d\n", conn_handle);
            return 0;
        }
#endif
        for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
            if (ranging_buffers[i].conn_handle == conn_handle &&
                ranging_buffers[i].busy) {
                buf = &ranging_buffers[i];
                break;
            }
        }
        if (buf == NULL) {
            MODLOG_DFLT(ERROR, "[RAS] cs_continue: no active buffer for conn=%d\n", conn_handle);
            return 0;
        }

        /* Append step data from continue event (pointer arithmetic, see above).
         * Track steps_written so the subevent header only counts stored steps. */
        int steps_written = 0;
        int steps_reported;
        bool steps_truncated = false;
        const uint8_t *step_ptr = (const uint8_t *)event->subev_result_continue.steps;
        const uint8_t *steps_end = step_ptr + event->subev_result_continue.steps_len;
        for (int i = 0; i < event->subev_result_continue.num_steps_reported; i++) {
            const struct cs_steps_data *step =
                (const struct cs_steps_data *)step_ptr;
            uint16_t needed;

            /* See the bounds check in the RESULT handler. */
            if (step_ptr + sizeof(*step) > steps_end ||
                step_ptr + sizeof(*step) + step->data_len > steps_end) {
                MODLOG_DFLT(ERROR, "[RAS] cs_continue: step %d out of bounds conn=%d\n",
                        i, conn_handle);
                break;
            }

            needed = BLE_RAS_STEP_MODE_LEN + step->data_len;

            if (buf->subevent_cursor + needed >
                BLE_RAS_PROCEDURE_MEM - BLE_RAS_RANGING_HEADER_LEN) {
                MODLOG_DFLT(ERROR, "[RAS] cs_continue: buffer overflow (step %d) conn=%d\n",
                        i, conn_handle);
                break;
            }

            buf->ranging_data.subevents[buf->subevent_cursor] = step->mode;
            buf->subevent_cursor += BLE_RAS_STEP_MODE_LEN;
            memcpy(&buf->ranging_data.subevents[buf->subevent_cursor],
                   step->data, step->data_len);
            buf->subevent_cursor += step->data_len;
            step_ptr += sizeof(struct cs_steps_data) + step->data_len;
            steps_written++;
        }

        /* Update the subevent header: accumulate only actually-stored steps and
         * propagate the latest done/abort status from the CONTINUE event. */
        struct subevent_header *sub_hdr =
            (struct subevent_header *)(buf->ranging_data.subevents +
                                       buf->subev_hdr_offset);
        /* Saturate instead of wrapping: the count is an 8-bit wire field, so a
         * controller reporting more steps for one subevent than it can express
         * would otherwise silently restart the count from zero. */
        steps_reported = sub_hdr->num_steps_reported + steps_written;
        if (steps_reported > UINT8_MAX) {
            MODLOG_DFLT(ERROR, "[RAS] cs_continue: step count overflow (%d) conn=%d\n",
                    steps_reported, conn_handle);
            steps_reported = UINT8_MAX;
            steps_truncated = true;
        }
        sub_hdr->num_steps_reported = (uint8_t)steps_reported;
        sub_hdr->ranging_done_status =
            event->subev_result_continue.procedure_done_status;
        sub_hdr->subevent_done_status =
            event->subev_result_continue.subevent_done_status;
        /* Unpack the two 4-bit abort reasons (see RESULT handler). */
        sub_hdr->ranging_abort_reason =
            event->subev_result_continue.abort_reason & 0x0F;
        sub_hdr->subevent_abort_reason =
            (event->subev_result_continue.abort_reason >> 4) & 0x0F;
        /* After the status copy above, so truncation is not overwritten.
         *
         * The marking is driven by the latch rather than by this event alone.
         * Step sizes vary within a subevent - a Mode 3 step is far larger than
         * a Mode 1 step - so a CONTINUE carrying only small steps can still
         * fit after a larger one did not.  Judged on its own evidence such an
         * event reports no loss, and the status copy above would then replace
         * the aborted marking an earlier event had set with the controller's
         * Complete, leaving a body that is missing steps in the middle while
         * claiming it finished normally.  steps_lost keeps the earlier loss
         * visible for the rest of this subevent, so the header stays aborted
         * once anything has been dropped from it. */
        if (steps_truncated ||
            steps_written < event->subev_result_continue.num_steps_reported) {
            buf->steps_lost = true;
        }
        if (buf->steps_lost) {
            ble_svc_ras_mark_truncated(sub_hdr, true);
        }

        if (event->subev_result_continue.procedure_done_status ==
            BLE_HCI_LE_CS_PROCEDURE_DONE_STATUS_COMPLETE ||
            event->subev_result_continue.procedure_done_status ==
            BLE_HCI_LE_CS_PROCEDURE_DONE_STATUS_ABORTED) {
            buf->ready = true;
            buf->busy = false;
            MODLOG_DFLT(DEBUG, "[RAS] buffer ready conn=%d counter=%d total_len=%d done=%d\n",
                    conn_handle, buf->ranging_counter,
                    BLE_RAS_RANGING_HEADER_LEN + buf->subevent_cursor,
                    event->subev_result_continue.procedure_done_status);
            {
                struct ble_svc_ras_rrsp *rrsp_tmp = ble_svc_ras_rrsp_find(conn_handle);
                if (rrsp_tmp && rrsp_tmp->rt_rd_subscribed) {
                    ble_svc_ras_rt_push(conn_handle, buf);
                } else {
                    ble_svc_ras_indicate_rd_ready(conn_handle, buf->ranging_counter);
                }
            }
        }
        break;
    }

    case BLE_CS_EVENT_CS_PROCEDURE_ENABLE_COMPLETE: {
        conn_handle = event->procedure_enable_complete.conn_handle;
        struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
        if (rrsp) {
            rrsp->selected_tx_power =
                event->procedure_enable_complete.selected_tx_power;
        }
        MODLOG_DFLT(INFO, "[RAS] CS procedure enable complete conn=%d "
                "tx_power=%d\n", conn_handle,
                event->procedure_enable_complete.selected_tx_power);
        break;
    }

    case BLE_CS_EVENT_CS_PROCEDURE_COMPLETE:
        MODLOG_DFLT(INFO, "[RAS] CS procedure complete conn=%d status=%d\n",
                event->procedure_complete.conn_handle,
                event->procedure_complete.status);
        break;

    default:
        break;
    }

    return 0;
}

/* Disconnect Cleanup */
void
ble_svc_ras_on_disconnect(uint16_t conn_handle)
{
    MODLOG_DFLT(INFO, "[RAS] disconnect cleanup conn=%d\n", conn_handle);

    struct ble_svc_ras_rrsp *rrsp = ble_svc_ras_rrsp_find(conn_handle);
    if (rrsp) {
        ble_npl_callout_stop(&rrsp->ack_timer);
        ble_npl_callout_stop(&rrsp->seg_timer);
        if (rrsp->active_buf && rrsp->active_buf->refcount > 0) {
            rrsp->active_buf->refcount--;
        }
        ble_svc_ras_rrsp_reset_state(rrsp);
        rrsp->conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }

    ble_svc_ras_bufs_flush(conn_handle);
}

/* GATT Registration Callback */
void
ble_svc_ras_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "[RAS] registered service %s handle=%d\n",
                ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "[RAS] registered chr %s def_handle=%d val_handle=%d\n",
                ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                ctxt->chr.def_handle,
                ctxt->chr.val_handle);
        break;
    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "[RAS] registered dsc %s handle=%d\n",
                ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                ctxt->dsc.handle);
        break;
    default:
        break;
    }
}

/* RAP 4.4: a client shall enable notifications on only ONE of On-Demand /
 * Real-Time Ranging Data.  Enabling the second one is rejected at the ATT
 * layer with 0xFD (CCCD Improperly Configured) via the GATT server CCCD
 * filter hook.  Runs with the GATT server lock held, so it only reads the
 * RAS-local subscription flags (exact mirrors of the CCCD state) and never
 * calls back into locked GATT APIs. */
static int
ble_svc_ras_cccd_filter(uint16_t conn_handle, uint16_t chr_val_handle,
                        uint16_t new_flags)
{
    struct ble_svc_ras_rrsp *rrsp;

    /* Disabling (or any write that doesn't enable) is always allowed. */
    if (new_flags == 0) {
        return 0;
    }

    if (chr_val_handle != ble_svc_ras_od_rd_val_handle &&
        chr_val_handle != ble_svc_ras_rt_rd_val_handle) {
        return 0;
    }

    rrsp = ble_svc_ras_rrsp_find(conn_handle);
    if (rrsp == NULL) {
        return 0;
    }

    if (chr_val_handle == ble_svc_ras_od_rd_val_handle &&
        rrsp->rt_rd_subscribed) {
        MODLOG_DFLT(ERROR, "[RAS] OD RD CCCD enable rejected: RT already enabled conn=%d\n",
                    conn_handle);
        return BLE_ATT_ERR_CCCD_IMPROPER_CONF;
    }

    if (chr_val_handle == ble_svc_ras_rt_rd_val_handle &&
        rrsp->od_rd_subscribed) {
        MODLOG_DFLT(ERROR, "[RAS] RT RD CCCD enable rejected: OD already enabled conn=%d\n",
                    conn_handle);
        return BLE_ATT_ERR_CCCD_IMPROPER_CONF;
    }

    return 0;
}

/* Init */

/* Drops everything this module cached in an earlier host incarnation.
 *
 * nimble_port_deinit() takes the NPL callout pool and the host's connection
 * state down without telling the services, so on the next nimble_port_init()
 * the entries here would still carry the conn_handles, subscriptions and
 * buffers of a host that no longer exists, and their callout handles would
 * point into memory that has been given back.  Every entry is therefore put
 * back into its never-used state here, and the two "inited" latches are cleared
 * so the pool setup rebuilds the rest from scratch.
 *
 * Clearing conn_handle is what makes the order safe: the pools are allocated on
 * first use, so the window between this function and the first
 * ble_svc_ras_rrsp_alloc() is reachable by the find() paths, and an entry left
 * carrying an old conn_handle would be handed to a caller that then arms its
 * zeroed callout.  With the handle cleared, find() reports no match and the
 * caller goes through alloc(), which is where timers are set up again.
 *
 * The callout handles are zeroed rather than deinited because the block behind
 * each one belongs to a pool that is gone: it cannot be returned, and
 * ble_npl_callout_init() adopts a non-NULL handle instead of taking a fresh
 * block, which is exactly the reuse that has to be avoided.  Zeroing puts each
 * callout back in the state a never-initialised entry has, so the next
 * ble_svc_ras_rrsp_alloc() gives it a block from the live pool.
 *
 * On a first init every latch is already clear and both pools are untouched,
 * so this does nothing.  Calling ble_svc_ras_init() a second time within one
 * host lifetime is not a supported sequence - it would register the GATT
 * service twice as well - so a live callout cannot be reached from here. */
static void
ble_svc_ras_init_reset_state(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (rrsp_pool != NULL)
#endif
    {
        for (int i = 0; i < RAS_RRSP_MAX_CONN; i++) {
            if (rrsp_pool[i].timers_inited) {
                memset(&rrsp_pool[i].ack_timer, 0, sizeof(rrsp_pool[i].ack_timer));
                memset(&rrsp_pool[i].seg_timer, 0, sizeof(rrsp_pool[i].seg_timer));
                rrsp_pool[i].timers_inited = false;
            }
            rrsp_pool[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ble_svc_ras_rrsp_reset_state(&rrsp_pool[i]);
        }
    }

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ranging_buffers != NULL)
#endif
    {
        for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
            ble_svc_ras_buf_free(&ranging_buffers[i]);
        }
    }

    rrsp_pool_inited = false;
    ranging_buffers_inited = false;
    ranging_buffer_alloc_seq = 0;
}

void
ble_svc_ras_init(void)
{
    int rc;

    SYSINIT_ASSERT_ACTIVE();

    ble_svc_ras_init_reset_state();
    ble_svc_ras_rreq_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    /* RAS 3.1.2: Features describe static server capabilities across both modes.
     * This server supports both RT and OD; the mode is selected at runtime by
     * the client's CCCD subscription.  Only the optional features the CP
     * actually carries out are advertised, because a client picks its
     * behaviour from these bits (RAP 4.3.2) and the CP answers anything else
     * with Op Code Not Supported.
     */
    ble_svc_ras_feat_val = BLE_SVC_RAS_FEAT_REALTIME_RD_BIT | BLE_SVC_RAS_FEAT_ABORT_OP_BIT;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    /* Pools are allocated on first use instead; see ble_svc_ras_buf_alloc()
     * and ble_svc_ras_rrsp_alloc(). */
#else
    /* Reset every pool entry to BLE_HS_CONN_HANDLE_NONE so a zero-filled
     * .bss entry cannot alias conn_handle 0 in find()/flush(). */
    rc = ble_svc_ras_buf_pool_init();
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_svc_ras_rrsp_pool_init();
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    rc = ble_gap_event_listener_register(&ras_gap_listener,
                                         ble_svc_ras_gap_listener_cb, NULL);
    SYSINIT_PANIC_ASSERT(rc == 0);

    ble_gatts_cccd_filter_register(ble_svc_ras_cccd_filter);

    MODLOG_DFLT(INFO, "[RAS] RAS service initialized (features=0x%08" PRIx32
            " buffers=%d max_conn=%d)\n",
            ble_svc_ras_feat_val, RAS_TOTAL_RD_BUFFERS, RAS_RRSP_MAX_CONN);
}

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
static void
ble_svc_ras_buf_pool_deinit(void)
{
    if (ranging_buffers != NULL) {
        nimble_platform_mem_free(ranging_buffers);
        ranging_buffers = NULL;
    }
    ranging_buffers_inited = false;
    ranging_buffer_alloc_seq = 0;
}

static void
ble_svc_ras_rrsp_pool_deinit(void)
{
    if (rrsp_pool != NULL) {
        for (int i = 0; i < RAS_RRSP_MAX_CONN; i++) {
            struct ble_svc_ras_rrsp *rrsp = &rrsp_pool[i];
            if (rrsp->timers_inited) {
                ble_npl_callout_stop(&rrsp->ack_timer);
                ble_npl_callout_stop(&rrsp->seg_timer);
                ble_npl_callout_deinit(&rrsp->ack_timer);
                ble_npl_callout_deinit(&rrsp->seg_timer);
                rrsp->timers_inited = false;
            }
        }
        nimble_platform_mem_free(rrsp_pool);
        rrsp_pool = NULL;
    }
    rrsp_pool_inited = false;
}
#else
static void
ble_svc_ras_buf_pool_deinit(void)
{
    ranging_buffers_inited = false;
    ranging_buffer_alloc_seq = 0;
    for (int i = 0; i < RAS_TOTAL_RD_BUFFERS; i++) {
        ranging_buffers[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
}

static void
ble_svc_ras_rrsp_pool_deinit(void)
{
    for (int i = 0; i < RAS_RRSP_MAX_CONN; i++) {
        struct ble_svc_ras_rrsp *rrsp = &rrsp_pool[i];
        if (rrsp->timers_inited) {
            ble_npl_callout_stop(&rrsp->ack_timer);
            ble_npl_callout_stop(&rrsp->seg_timer);
            ble_npl_callout_deinit(&rrsp->ack_timer);
            ble_npl_callout_deinit(&rrsp->seg_timer);
            rrsp->timers_inited = false;
        }
        rrsp->conn_handle = BLE_HS_CONN_HANDLE_NONE;
    }
    rrsp_pool_inited = false;
}
#endif

/* Teardown for ble_svc_ras_init().  Timers are stopped and deinited before the
 * pool memory is freed, so this runs while the host is still up - after
 * ble_svc_ras_init() and before nimble_port_deinit(), which releases the NPL
 * callout pool the timers came from.  An application that brings the host down
 * first needs no call at all: ble_svc_ras_init_reset_state() discards what the
 * previous incarnation left behind.  Repeat calls inside one host lifetime, and
 * calls made without a preceding init, do nothing - the timers_inited latch and
 * the NULL pool pointers make both a no-op.
 *
 * Deliberately has no in-tree caller: the host never tears services down on
 * its own, so this exists for an application that wants to reclaim the RAS
 * pools after it is done with ranging.  Nothing leaks if it is never called -
 * without BLE_STATIC_TO_DYNAMIC the pools are .bss, and with it they are
 * allocated once and reused by the next ble_svc_ras_init(), which resets them
 * through ble_svc_ras_init_reset_state().
 *
 * The GATT service entry is left in place on purpose.  It is owned by the GATT
 * server, which drops the whole table in ble_gatts_stop() when the host goes
 * down, and removing one service from a running server would renumber the
 * handles of every service after it.  The call therefore marks the point where
 * the server side stops being used: the per-connection cleanup runs from the
 * GAP listener unregistered above, so this belongs at the end of a session
 * rather than between two of them. */
void
ble_svc_ras_deinit(void)
{
    ble_gatts_cccd_filter_register(NULL);
    ble_gap_event_listener_unregister(&ras_gap_listener);
    ble_svc_ras_rrsp_pool_deinit();
    ble_svc_ras_buf_pool_deinit();
}
#endif /* BLE_GATTS && BLE_RAS_SERVICE */

#endif /* BLE_CHANNEL_SOUNDING && BLE_RAS_SERVICE */
