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
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/ras/ble_svc_ras.h"
#include "sysinit/sysinit.h"
#include "host/ble_cs.h"
#include "host/ble_hs_log.h"
#include "nimble/hci_common.h"
#include "esp_nimble_mem.h"

#if MYNEWT_VAL(BLE_GATTS) && CONFIG_BT_NIMBLE_RAS_SERVICE
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
typedef struct {
    uint32_t _ble_svc_ras_feat_val;
    uint16_t _ble_svc_ras_rd_val;
    uint16_t _ble_svc_ras_rd_ov_val;
    uint8_t  _ble_svc_ras_cp_val[RASCP_CMD_OPCODE_LEN + sizeof(uint16_t)] ;
    struct segment *_ble_svc_ras_od_rd_val;
    uint16_t _ble_svc_ras_od_rd_seg_len;
    uint16_t _ble_svc_ras_rt_rd_val;
    struct ranging_buffer _ranging_buffers[BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE];
} ble_svc_ras_ctx_t;

static ble_svc_ras_ctx_t *ble_svc_ras_ctx;

#define ble_svc_ras_feat_val (ble_svc_ras_ctx->_ble_svc_ras_feat_val)
#define ble_svc_ras_rd_val (ble_svc_ras_ctx->_ble_svc_ras_rd_val)
#define ble_svc_ras_rd_ov_val (ble_svc_ras_ctx->_ble_svc_ras_rd_ov_val)
#define ble_svc_ras_cp_val (ble_svc_ras_ctx->_ble_svc_ras_cp_val)
#define ble_svc_ras_od_rd_val (ble_svc_ras_ctx->_ble_svc_ras_od_rd_val)
#define ble_svc_ras_od_rd_seg_len (ble_svc_ras_ctx->_ble_svc_ras_od_rd_seg_len)
#define ble_svc_ras_rt_rd_val (ble_svc_ras_ctx->_ble_svc_ras_rt_rd_val)
#define ranging_buffers (ble_svc_ras_ctx->_ranging_buffers)

#else /* MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) */

static uint32_t ble_svc_ras_feat_val;

static uint16_t ble_svc_ras_rd_val;

static uint16_t ble_svc_ras_rd_ov_val;

static uint8_t  ble_svc_ras_cp_val[RASCP_CMD_OPCODE_LEN + sizeof(uint16_t)] ;


static struct segment *ble_svc_ras_od_rd_val;
static uint16_t ble_svc_ras_od_rd_seg_len;

static uint16_t ble_svc_ras_rt_rd_val;

static struct ranging_buffer ranging_buffers[BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE];

#endif /* MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) */

static uint16_t ble_svc_ras_feat_val_handle;
static uint16_t ble_svc_ras_rd_val_handle;
static uint16_t ble_svc_ras_rd_ov_val_handle;
static uint16_t ble_svc_ras_cp_val_handle;
static uint16_t ble_svc_ras_od_rd_val_handle;
static uint16_t ble_svc_ras_rt_rd_val_handle;

static void ranging_buffer_init(uint16_t conn_handle, struct ranging_buffer *buf, uint16_t ranging_counter)
{
    buf->conn = conn_handle;
    buf->ranging_counter = ranging_counter;
    buf->isready = false;
    buf->isbusy = true;
    buf->isacked = false;
    buf->subevent_cursor = 0;
    buf->last_subevent_hdr_offset = 0;
    buf->last_subevent_hdr_valid = false;
    memset(&buf->ranging_data, 0, sizeof(buf->ranging_data));
}

static void reset_ranging_buffer(void)
{
   /* reset whole array of ranging buffers */
   for (int i = 0; i < BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE; i++) {
        ranging_buffers[i].conn = -1;  //No connection
        ranging_buffers[i].ranging_counter = 0;
        ranging_buffers[i].isready = false;
        ranging_buffers[i].isbusy = false;
        ranging_buffers[i].isacked = false;
        ranging_buffers[i].subevent_cursor = 0;
        ranging_buffers[i].last_subevent_hdr_offset = 0;
        ranging_buffers[i].last_subevent_hdr_valid = false;
    }
}

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
void
ble_svc_ras_ensure_ctx_init()
{
    if (ble_svc_ras_ctx == NULL) {
        ble_svc_ras_ctx = nimble_platform_mem_calloc(1, sizeof(ble_svc_ras_ctx_t));
        if (ble_svc_ras_ctx != NULL) {
            reset_ranging_buffer();
        }
    }
}

void
ble_svc_ras_ctx_deinit()
{
    if (ble_svc_ras_ctx) {
        if (ble_svc_ras_ctx->_ble_svc_ras_od_rd_val) {
            /* Ensure that heap memory is not lost after deiniting ras. */
            nimble_platform_mem_free(ble_svc_ras_ctx->_ble_svc_ras_od_rd_val);
            ble_svc_ras_ctx->_ble_svc_ras_od_rd_val = NULL;
        }
        nimble_platform_mem_free(ble_svc_ras_ctx);
        ble_svc_ras_ctx = NULL;
    }
}
#endif

void ble_gatts_indicate_ranging_data_ready(uint16_t ranging_counter)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    ble_svc_ras_ensure_ctx_init();
    if (ble_svc_ras_ctx == NULL) {
        return;
    }
#endif

    MODLOG_DFLT(INFO, "Indicate ranging data ready for counter %d\n", ranging_counter);
    /* Indicate that the ranging data is ready for the client */
    ble_svc_ras_rd_val = ranging_counter;
    ble_gatts_chr_updated(ble_svc_ras_rd_val_handle);
}

void ble_gatts_indicate_control_point_response(uint16_t attr_handle , uint16_t ranging_counter)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    ble_svc_ras_ensure_ctx_init();
    if (ble_svc_ras_ctx == NULL) {
        return;
    }
#endif

    /* Indication control point response only when all the indication for on_demand_rd is sent for all segments */
    if (attr_handle == ble_svc_ras_od_rd_val_handle) {
        MODLOG_DFLT(INFO, "Indicate control point response\n");
        ble_svc_ras_cp_val[0] = RASCP_RSP_OPCODE_COMPLETE_RD_RSP;
        memcpy(&ble_svc_ras_cp_val[RASCP_RSP_OPCODE_COMPLETE_RD_RSP_LEN], &ranging_counter, sizeof(uint16_t));
        ble_gatts_chr_updated(ble_svc_ras_cp_val_handle);
    } else {
        return ;
    }
}

struct ranging_buffer *ranging_buffer_alloc(uint16_t conn_handle, uint16_t ranging_counter)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    ble_svc_ras_ensure_ctx_init();
    if (ble_svc_ras_ctx == NULL) {
        return NULL;
    }
#endif

    for (uint8_t i = 0; i < sizeof(ranging_buffers)/sizeof(ranging_buffers[0]); i++) {
        if (ranging_buffers[i].conn == -1) {
            ranging_buffer_init(conn_handle, &ranging_buffers[i], ranging_counter);
            return &ranging_buffers[i];
        }
    }

    /* No buffer available */
    return NULL;
}

static int gatt_svr_chr_access_ras_val(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg);

static const ble_uuid16_t uuid_svc_ras = BLE_UUID16_INIT(BLE_SVC_RAS_RANGING_SERVICE_VAL);
static const ble_uuid16_t uuid_chr_feat_value = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_FEATURES_VAL);
static const ble_uuid16_t uuid_chr_demand_rd = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_ONDEMAND_RD_VAL);
static const ble_uuid16_t uuid_chr_real_rd = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_REALTIME_RD_VAL);
static const ble_uuid16_t uuid_chr_ctrl_pt = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_CP_VAL);
static const ble_uuid16_t uuid_chr_data_rdy = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_RD_READY_VAL);
static const ble_uuid16_t uuid_chr_data_ow = BLE_UUID16_INIT(BLE_SVC_RAS_CHR_UUID_RD_OVERWRITTEN_VAL);

static const struct ble_gatt_chr_def ras_characteristics[] = {
            {
                /* Characteristic: Feature Value */
                .uuid = &uuid_chr_feat_value.u,
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_feat_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
            },
            {
                /* Characteristic: On demand ranging data */
                .uuid = &uuid_chr_demand_rd.u,
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_od_rd_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE  ,
            },
            {
                 /* Characteristic: On realtime ranging data */
                 .uuid = &uuid_chr_real_rd.u,
                 .access_cb = gatt_svr_chr_access_ras_val,
                 .val_handle = &ble_svc_ras_rt_rd_val_handle,
                 .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE  ,
             },

            {
                /* Characteristic: RAS Control Point */
                .uuid = &uuid_chr_ctrl_pt.u,
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_cp_val_handle,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_INDICATE,
            },
            {
                /* Characteristic: RAS Ranging Data Ready */
                .uuid = &uuid_chr_data_rdy.u,
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_rd_val_handle,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ ,
            },
            {
                /* Characteristic: RAS  Ranging data overwritten */
                .uuid = &uuid_chr_data_ow.u,
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_rd_ov_val_handle,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ,
            },
            {
                0, /* No more characteristics in this service */
            },
};

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Ranging Data Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_ras.u,
        .characteristics = ras_characteristics,
    },
    {
        0, /* No more services */
    },
};


static int
gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);

    if (om_len < min_len || om_len > max_len) {
        MODLOG_DFLT(INFO, "Invalid attr len");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}


static int gatt_svr_chr_access_ras_val(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_svc_ras_ctx == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }
#endif

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                MODLOG_DFLT(INFO, "Characteristic read; conn_handle=%d attr_handle=%d\n",
                            conn_handle, attr_handle);
            } else {
                MODLOG_DFLT(INFO, "Characteristic read by NimBLE stack; attr_handle=%d\n",
                            attr_handle);
            }

            uuid = ble_uuid_u16(ctxt->chr->uuid);
            if (uuid == BLE_SVC_RAS_CHR_UUID_FEATURES_VAL) {
                ble_svc_ras_feat_val |= RETRIEVE_LST_SEG_BIT | ABORT_OP_BIT | FLTR_RANGING_DATA_BIT;
                MODLOG_DFLT(INFO, "ble_svc_ras_feat_val = %02x\n",ble_svc_ras_feat_val);
                if (attr_handle == ble_svc_ras_feat_val_handle) {
                    rc = os_mbuf_append(ctxt->om,
                                        &ble_svc_ras_feat_val,
                                        sizeof(ble_svc_ras_feat_val));
                    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }
            } else if (uuid == BLE_SVC_RAS_CHR_UUID_RD_READY_VAL) {
                MODLOG_DFLT(INFO, "ble_svc_ras_rd_val = %d\n", ble_svc_ras_rd_val);
                if (attr_handle == ble_svc_ras_rd_val_handle) {
                    rc = os_mbuf_append(ctxt->om,
                                        &ble_svc_ras_rd_val,
                                        sizeof(ble_svc_ras_rd_val));
                    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }
            }
            else if (uuid == BLE_SVC_RAS_CHR_UUID_ONDEMAND_RD_VAL) {
                /* print the segment data */
                if (ble_svc_ras_od_rd_val == NULL) {
                    return BLE_ATT_ERR_UNLIKELY;
                }
                MODLOG_DFLT(INFO, "ble_svc_ras_od_rd_val\n");
                if (attr_handle == ble_svc_ras_od_rd_val_handle) {
                    rc = os_mbuf_append(ctxt->om,
                                        ble_svc_ras_od_rd_val,
                                        ble_svc_ras_od_rd_seg_len);
                    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }
            } else if (uuid == BLE_SVC_RAS_CHR_UUID_REALTIME_RD_VAL) {
                MODLOG_DFLT(INFO, "ble_svc_ras_rt_rd_val = %d\n", ble_svc_ras_rt_rd_val);
                if (attr_handle == ble_svc_ras_rt_rd_val_handle) {
                    rc = os_mbuf_append(ctxt->om,
                                        &ble_svc_ras_rt_rd_val,
                                        sizeof(ble_svc_ras_rt_rd_val));
                    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }
            } else if (uuid == BLE_SVC_RAS_CHR_UUID_CP_VAL) {
                MODLOG_DFLT(INFO,"ble_svc_ras_cp_val read  = %02x %02x %02x \n",ble_svc_ras_cp_val[0],ble_svc_ras_cp_val[1],ble_svc_ras_cp_val[2]);
                    rc = os_mbuf_append(ctxt->om,
                                        &ble_svc_ras_cp_val,
                                        sizeof(ble_svc_ras_cp_val));
                    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }

             else if (uuid == BLE_SVC_RAS_CHR_UUID_RD_OVERWRITTEN_VAL) {
                MODLOG_DFLT(INFO, "ble_svc_ras_rd_ov_val = %d\n", ble_svc_ras_rd_ov_val);
                if (attr_handle == ble_svc_ras_rd_ov_val_handle) {
                    rc = os_mbuf_append(ctxt->om,
                                        &ble_svc_ras_rd_ov_val,
                                        sizeof(ble_svc_ras_rd_ov_val));
                    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }
            }
            return 0;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
                MODLOG_DFLT(INFO, "Characteristic write; conn_handle=%d attr_handle=%d",
                            conn_handle, attr_handle);
            } else {
                MODLOG_DFLT(INFO, "Characteristic write by NimBLE stack; attr_handle=%d",
                            attr_handle);
            }
            if (attr_handle == ble_svc_ras_rt_rd_val_handle) {
                rc = gatt_svr_write(ctxt->om,
                        sizeof(ble_svc_ras_rt_rd_val),
                        sizeof(ble_svc_ras_rt_rd_val),
                        &ble_svc_ras_rt_rd_val, NULL);
                if (rc == 0) {
                    ble_gatts_chr_updated(attr_handle);
                    MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                    "all subscribed peers.\n");
                }
                return rc;
            } else if (attr_handle == ble_svc_ras_od_rd_val_handle) {
                /* Ensure the buffer is allocated before writing to it */
                if (ble_svc_ras_od_rd_val == NULL) {
                    return BLE_ATT_ERR_UNLIKELY;
                }

                rc = gatt_svr_write(ctxt->om,
                                    0,
                                    ble_svc_ras_od_rd_seg_len,
                                    ble_svc_ras_od_rd_val, NULL);
                if (rc == 0) {
                    ble_gatts_chr_updated(attr_handle);
                    MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                    "all subscribed peers.\n");
                }
                return rc;
            } else if (attr_handle == ble_svc_ras_cp_val_handle) {
                MODLOG_DFLT(INFO, "write for control point val ");

                rc = gatt_svr_write(ctxt->om,
                        RASCP_CMD_OPCODE_LEN,
                        sizeof(ble_svc_ras_cp_val),
                        &ble_svc_ras_cp_val, NULL);
                if (rc != 0) {
                    return rc;
                }
                 MODLOG_DFLT(INFO, "ble_svc_gap_cp_val = %02x %02x %02x \n",ble_svc_ras_cp_val[0],ble_svc_ras_cp_val[1],ble_svc_ras_cp_val[2]);
                if (ble_svc_ras_cp_val[0] ==  RASCP_OPCODE_GET_RD) {
                    // n = size of (rangeing_buffer[i]) / mut -4);
                    /* Run a loop to send n segmet . for now sending only 1 segment*/
                    ble_gatts_chr_updated(ble_svc_ras_od_rd_val_handle);
                } else if (ble_svc_ras_cp_val[0] == RASCP_OPCODE_ACK_RD) {
                    MODLOG_DFLT(INFO, "ack received\n");
                    /* Free the acknowledged segment */
                    if (ble_svc_ras_od_rd_val != NULL) {
                        nimble_platform_mem_free(ble_svc_ras_od_rd_val);
                        ble_svc_ras_od_rd_val = NULL;
                        ble_svc_ras_od_rd_seg_len = 0;
                    }
                    /* Reset the ranging buffer for this connection */
                    for (int i = 0; i < BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE; i++) {
                        if (ranging_buffers[i].conn == (int)conn_handle) {
                            ranging_buffers[i].conn = -1;
                            ranging_buffers[i].ranging_counter = 0;
                            ranging_buffers[i].isready = false;
                            ranging_buffers[i].isbusy = false;
                            ranging_buffers[i].isacked = false;
                            ranging_buffers[i].subevent_cursor = 0;
                            ranging_buffers[i].last_subevent_hdr_offset = 0;
                            ranging_buffers[i].last_subevent_hdr_valid = false;
                        }
                    }
                    ble_svc_ras_cp_val[0]= 0x02;
                    /*Table 3.12. Response Code Values associated with Op Code 0x02*/
                    ble_svc_ras_cp_val[1]=0x01; // Success
                    ble_svc_ras_cp_val[2]=0x01; // Response value: Success
                    ble_gatts_chr_updated(ble_svc_ras_cp_val_handle);
                    MODLOG_DFLT(INFO, "Successfully completed the Ranging procedure\n");

                    // vTaskDelay(4000 / portTICK_PERIOD_MS);
                    //extern void bt_hci_log_hci_data_show(void);
                    //bt_hci_log_hci_data_show();

                }
                MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                "all subscribed peers.\n");
                return rc;
            } else if (attr_handle == ble_svc_ras_rd_val_handle) {
                rc = gatt_svr_write(ctxt->om,
                        sizeof(ble_svc_ras_rd_val),
                        sizeof(ble_svc_ras_rd_val),
                        &ble_svc_ras_rd_val, NULL);
                if (rc == 0) {
                    ble_gatts_chr_updated(attr_handle);
                    MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                    "all subscribed peers.\n");
                }
                return rc;
            } else if (attr_handle == ble_svc_ras_rd_ov_val_handle) {
                rc = gatt_svr_write(ctxt->om,
                        sizeof(ble_svc_ras_rd_ov_val),
                        sizeof(ble_svc_ras_rd_ov_val),
                        &ble_svc_ras_rd_ov_val, NULL);
                if (rc == 0) {
                    ble_gatts_chr_updated(attr_handle);
                    MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                    "all subscribed peers.\n");
                }
                return rc;
            }

            return 0;

        default:
            goto unknown;
    }

unknown:
    /* Unknown characteristic/descriptor;
     * The NimBLE host should not have called this function;
     */
    // assert(0);

    return BLE_ATT_ERR_UNLIKELY;
}

void ble_gatts_store_ranging_data(struct ble_cs_event ranging_subevent) {
    struct ranging_buffer *buf = NULL;
    uint16_t conn_handle;
    uint16_t procedure_counter;
    uint8_t config_id;
    uint8_t procedure_done_status;
    uint8_t subevent_done_status;
    uint8_t abort_reason;
    uint8_t num_antenna_paths;
    uint8_t num_steps_reported;
    const struct cs_steps_data *steps;
    uint16_t start_acl_conn_event_counter = 0;
    uint16_t frequency_compensation = 0;
    uint8_t reference_power_level = 0;

    /* Extract fields from the correct union member based on event type */
    if (ranging_subevent.type == BLE_CS_EVENT_SUBEVET_RESULT) {
        conn_handle = ranging_subevent.subev_result.conn_handle;
        procedure_counter = ranging_subevent.subev_result.procedure_counter;
        config_id = ranging_subevent.subev_result.config_id;
        procedure_done_status = ranging_subevent.subev_result.procedure_done_status;
        subevent_done_status = ranging_subevent.subev_result.subevent_done_status;
        abort_reason = ranging_subevent.subev_result.abort_reason;
        num_antenna_paths = ranging_subevent.subev_result.num_antenna_paths;
        num_steps_reported = ranging_subevent.subev_result.num_steps_reported;
        steps = ranging_subevent.subev_result.steps;
        start_acl_conn_event_counter = ranging_subevent.subev_result.start_acl_conn_event_counter;
        frequency_compensation = ranging_subevent.subev_result.frequency_compensation;
        reference_power_level = ranging_subevent.subev_result.reference_power_level;
    } else if (ranging_subevent.type == BLE_CS_EVENT_SUBEVET_RESULT_CONTINUE) {
        conn_handle = ranging_subevent.subev_result_continue.conn_handle;
        /* subev_result_continue has no procedure_counter; reuse conn to find buf */
        procedure_counter = 0;
        config_id = ranging_subevent.subev_result_continue.config_id;
        procedure_done_status = ranging_subevent.subev_result_continue.procedure_done_status;
        subevent_done_status = ranging_subevent.subev_result_continue.subevent_done_status;
        abort_reason = ranging_subevent.subev_result_continue.abort_reason;
        num_antenna_paths = ranging_subevent.subev_result_continue.num_antenna_paths;
        num_steps_reported = ranging_subevent.subev_result_continue.num_steps_reported;
        steps = ranging_subevent.subev_result_continue.steps;
    } else {
        MODLOG_DFLT(ERROR, "Unknown CS event type %d\n", ranging_subevent.type);
        return;
    }

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    ble_svc_ras_ensure_ctx_init();
    if (ble_svc_ras_ctx == NULL) {
        return;
    }
#endif

    /* Find the buffer for this event.
     * RESULT:          match on conn_handle + ranging_counter (unique per procedure).
     * RESULT_CONTINUE: procedure_counter not available per BT Core Spec §7.7.65.45;
     *                  match on conn_handle + config_id. config_id is only 4 bits
     *                  (0-3), so successive procedures on the same connection share
     *                  the same config_id. Among all matching buffers pick the one
     *                  with the highest ranging_counter — that is the most recently
     *                  started procedure and the correct target for RESULT_CONTINUE. */
    if (ranging_subevent.type == BLE_CS_EVENT_SUBEVET_RESULT_CONTINUE) {
        uint16_t best_counter = 0;
        bool found = false;
        for (int i = 0; i < BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE; i++) {
            if (ranging_buffers[i].conn != (int)conn_handle) {
                continue;
            }
            if (ranging_buffers[i].ranging_data.ranging_header.config_id == config_id) {
                if (!found || ranging_buffers[i].ranging_counter > best_counter) {
                    buf = &ranging_buffers[i];
                    best_counter = ranging_buffers[i].ranging_counter;
                    found = true;
                }
            }
        }
    } else {
        for (int i = 0; i < BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE; i++) {
            if (ranging_buffers[i].conn != (int)conn_handle) {
                continue;
            }
            if (ranging_buffers[i].ranging_counter == procedure_counter) {
                buf = &ranging_buffers[i];
                break;
            }
        }
    }

    if (buf == NULL) {
        /* Per BT spec §7.7.65.45, RESULT_CONTINUE shall only follow a prior
         * RESULT event. Receiving RESULT_CONTINUE without an existing buffer
         * is a protocol error — reject to avoid producing corrupt data. */
        if (ranging_subevent.type == BLE_CS_EVENT_SUBEVET_RESULT_CONTINUE) {
            MODLOG_DFLT(ERROR, "RESULT_CONTINUE without prior RESULT for conn 0x%04x\n",
                        conn_handle);
            return;
        }
        buf = ranging_buffer_alloc(conn_handle, procedure_counter);
        if (buf == NULL) {
            MODLOG_DFLT(ERROR,"No available buffer for storing ranging data\n");
            return;
        }
    }

    /* Only set header fields for initial RESULT event; RESULT_CONTINUE appends
     * to an existing buffer and must not overwrite the valid ranging_counter. */
    if (ranging_subevent.type == BLE_CS_EVENT_SUBEVET_RESULT) {
        buf->ranging_data.ranging_header.config_id = config_id;
        buf->ranging_data.ranging_header.ranging_counter = procedure_counter;
        /* convert antenna path mask using bitmask */
        buf->ranging_data.ranging_header.antenna_paths_mask = num_antenna_paths;
    }

    uint16_t max_subevent_data = BLE_RAS_PROCEDURE_MEM - sizeof(struct ranging_header);

    /* RESULT_CONTINUE appends step data to the same logical subevent; no new
     * subevent_header is written. Only RESULT events open a new subevent. */
    if (ranging_subevent.type == BLE_CS_EVENT_SUBEVET_RESULT) {
        /* Invalidate the prior subevent's header pointer before the overflow
         * check. If this RESULT overflows and returns early, any subsequent
         * RESULT_CONTINUE must not corrupt the previous subevent's header. */
        buf->last_subevent_hdr_valid = false;

        if (buf->subevent_cursor + sizeof(struct subevent_header) > max_subevent_data) {
            MODLOG_DFLT(ERROR, "Ranging buffer overflow on subevent header\n");
            return;
        }

        /* Save offset so RESULT_CONTINUE can locate and update this header. */
        buf->last_subevent_hdr_offset = buf->subevent_cursor;
        buf->last_subevent_hdr_valid = true;

        struct subevent_header *subevent_hdr = (struct subevent_header *)(buf->ranging_data.subevents + buf->subevent_cursor);
        buf->subevent_cursor += sizeof(struct subevent_header);
        subevent_hdr->start_acl_conn_event = start_acl_conn_event_counter;
        subevent_hdr->freq_compensation = frequency_compensation;
        subevent_hdr->ranging_done_status = procedure_done_status;
        subevent_hdr->subevent_done_status = subevent_done_status;
        /* Abort_Reason per BT Core Spec §7.7.65.44: bits 0-3 = procedure abort,
         * bits 4-7 = subevent abort. Extract each nibble separately. */
        subevent_hdr->ranging_abort_reason  = abort_reason & 0x0F;
        subevent_hdr->subevent_abort_reason = (abort_reason >> 4) & 0x0F;
        subevent_hdr->ref_power_level = reference_power_level;
        /* num_steps_reported set after the step loop to reflect actual written count. */
    }

    /* For RESULT_CONTINUE: if the corresponding RESULT event failed (overflow),
     * last_subevent_hdr_valid is false. Writing step data into the buffer with
     * no subevent header to account for it produces a corrupt RAS segment.
     * Reject early, before the loop advances subevent_cursor. */
    if (ranging_subevent.type == BLE_CS_EVENT_SUBEVET_RESULT_CONTINUE &&
            !buf->last_subevent_hdr_valid) {
        MODLOG_DFLT(ERROR, "RESULT_CONTINUE with no valid subevent header; dropping\n");
        return;
    }

    /* Add step data; track actual steps written so the header reflects reality
     * even if the buffer overflows mid-loop and we return early. */
    uint8_t steps_written = 0;
    const uint8_t *step_ptr = (const uint8_t *)steps;
    for (int i = 0; i < num_steps_reported; i++) {
        const struct cs_steps_data *step = (const struct cs_steps_data *)step_ptr;

        if (buf->subevent_cursor + BLE_RAS_STEP_MODE_LEN + step->data_len > max_subevent_data) {
            MODLOG_DFLT(ERROR, "Ranging buffer overflow on step data\n");
            break;
        }

        buf->ranging_data.subevents[buf->subevent_cursor] = step->mode;
        buf->subevent_cursor += BLE_RAS_STEP_MODE_LEN;
        memcpy(&buf->ranging_data.subevents[buf->subevent_cursor], step->data, step->data_len);
        buf->subevent_cursor += step->data_len;
        step_ptr += sizeof(struct cs_steps_data) + step->data_len;
        steps_written++;
    }

    /* Update the subevent header with the count of steps actually written.
     * For RESULT: correct any pre-set count in case of overflow.
     * For RESULT_CONTINUE: accumulate only the steps that fit, and overwrite
     * the status/abort fields with the final values from this last event
     * (BT Core Spec v6.2 §7.7.65.44 — the Continue event carries the terminal
     * done/abort status that supersedes the partial status set by RESULT).
     * Guard with last_subevent_hdr_valid: a RESULT_CONTINUE arriving after an
     * overflowed RESULT must not corrupt the previous subevent's header. */
    if (buf->last_subevent_hdr_valid) {
        struct subevent_header *subevent_hdr =
            (struct subevent_header *)(buf->ranging_data.subevents +
                                       buf->last_subevent_hdr_offset);
        if (ranging_subevent.type == BLE_CS_EVENT_SUBEVET_RESULT) {
            subevent_hdr->num_steps_reported = steps_written;
        } else {
            subevent_hdr->num_steps_reported += steps_written;
            subevent_hdr->ranging_done_status  = procedure_done_status;
            subevent_hdr->subevent_done_status = subevent_done_status;
            subevent_hdr->ranging_abort_reason  = abort_reason & 0x0F;
            subevent_hdr->subevent_abort_reason = (abort_reason >> 4) & 0x0F;
        }
    }
    /* Create RAS segment*/
    struct segment *ras_segment;

    uint16_t att_mtu = ble_att_mtu(conn_handle);
    uint16_t overhead = (uint16_t)(sizeof(struct segment_header) + 4);
    if (att_mtu == 0 || att_mtu <= overhead) {
        MODLOG_DFLT(INFO, "MTU (%d) too small for RAS segment overhead (%d)\n",
                    att_mtu, overhead);
        return;
    }
    uint16_t max_data_len = att_mtu - overhead;
    MODLOG_DFLT(INFO, "Max data len : %d\n", max_data_len);
    ras_segment= nimble_platform_mem_calloc(1,sizeof(struct segment)+ max_data_len);
    if (ras_segment == NULL) {
        MODLOG_DFLT(INFO, "Failed to allocate memory for RAS segment\n");
        return;
    }
    ras_segment->header.first_seg = true;
    ras_segment->header.seg_counter = 0; /* First segment */
    uint16_t buf_len = sizeof(struct ranging_header) + buf->subevent_cursor;
    uint16_t pull_bytes = MIN(max_data_len, buf_len);

    ras_segment->header.last_seg = (buf_len <= max_data_len);
    memcpy(ras_segment->data, &buf->ranging_data.buf[0], pull_bytes);

    /* Free previous segment if any */
    if (ble_svc_ras_od_rd_val != NULL) {
        nimble_platform_mem_free(ble_svc_ras_od_rd_val);
    }
    ble_svc_ras_od_rd_val = ras_segment;
    ble_svc_ras_od_rd_seg_len = sizeof(struct segment_header) + pull_bytes;
}

void custom_gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                        ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                        ctxt->svc.handle);
            break;

        case BLE_GATT_REGISTER_OP_CHR:
            MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                        "def_handle=%d val_handle=%d\n",
                        ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                        ctxt->chr.def_handle,
                        ctxt->chr.val_handle);
            break;

        case BLE_GATT_REGISTER_OP_DSC:
            MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                        ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                        ctxt->dsc.handle);
            break;

        default:
            assert(0);
            break;
    }
}

void
ble_svc_ras_init(void) {
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    ble_svc_ras_ensure_ctx_init();
    if (ble_svc_ras_ctx == NULL) {
        SYSINIT_PANIC_ASSERT(0);
        return;
    }
#endif

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    reset_ranging_buffer();

}

void
ble_svc_ras_deinit(void)
{
#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_svc_ras_od_rd_val != NULL) {
        nimble_platform_mem_free(ble_svc_ras_od_rd_val);
        ble_svc_ras_od_rd_val = NULL;
    }
    ble_svc_ras_od_rd_seg_len = 0;
#else
    /* ble_svc_ras_ctx_deinit handles NULL check, frees inner buffer,
     * frees ctx itself, and sets ble_svc_ras_ctx = NULL. */
    ble_svc_ras_ctx_deinit();
#endif
}
#endif
