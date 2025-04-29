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
#include "nimble/hci_common.h"
#include "esp_nimble_mem.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static uint16_t ble_svc_ras_feat_val_handle;
static uint32_t ble_svc_ras_feat_val;

static uint16_t ble_svc_ras_rd_val_handle;
static uint16_t ble_svc_ras_rd_val;

static uint16_t ble_svc_ras_rd_ov_val_handle;
static uint16_t ble_svc_ras_rd_ov_val;

static uint16_t ble_svc_ras_cp_val_handle;
static uint8_t  ble_svc_ras_cp_val[RASCP_CMD_OPCODE_LEN + sizeof(uint16_t)] ;


static uint16_t ble_svc_ras_od_rd_val_handle;
static struct segment *ble_svc_ras_od_rd_val;

static uint16_t ble_svc_ras_rt_rd_val_handle;
static uint16_t ble_svc_ras_rt_rd_val;

static struct ranging_buffer ranging_buffers[BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE];

void ble_gatts_indicate_ranging_data_ready(uint16_t ranging_counter)
{
    MODLOG_DFLT(INFO, "Indicate ranging data ready for counter %d\n", ranging_counter);
    /* Indicate that the ranging data is ready for the client */
    ble_svc_ras_rd_val = ranging_counter;
    ble_gatts_chr_updated(ble_svc_ras_rd_val_handle);
}

void ble_gatts_indicate_control_point_response(uint16_t attr_handle , uint16_t ranging_counter)
{
    /* Indication control point reponse only when all the idication for on_demad_rd is sent for all segments;*/
    if (attr_handle == ble_svc_ras_od_rd_val_handle) {
        MODLOG_DFLT(INFO, "Indicate control point response\n");
        ble_svc_ras_cp_val[0] = RASCP_RSP_OPCODE_COMPLETE_RD_RSP;
        memcpy(&ble_svc_ras_cp_val[RASCP_RSP_OPCODE_COMPLETE_RD_RSP_LEN], &ranging_counter, sizeof(uint16_t));
        ble_gatts_chr_updated(ble_svc_ras_cp_val_handle);
    } else {
        return ;
    }
}

static void ranging_buffer_init(uint16_t conn_handle, struct ranging_buffer *buf, uint16_t ranging_counter)
{
    buf->conn = conn_handle;
    buf->ranging_counter = ranging_counter;
    buf->isready = false;
    buf->isbusy = true;
    buf->isacked = false;
    buf->subevent_cursor = 0;
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
    }
}
struct ranging_buffer *ranging_buffer_alloc(uint16_t conn_handle , uint16_t ranging_counter)
{
    uint16_t conn_buffer_count = 0;

    for (uint8_t i = 0; i < sizeof(ranging_buffers)/sizeof(ranging_buffers[0]) ; i++) {
        if (ranging_buffers[i].conn == -1) {
            conn_buffer_count++;
            ranging_buffer_init(conn_handle, &ranging_buffers[i] , ranging_counter);
            return &ranging_buffers[i];
        }
    }

    /* No buffer available */
    return NULL;
}

static int gatt_svr_chr_access_ras_val(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Ranging Data Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_RANGING_SERVICE_VAL),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Characteristic: Feature Value */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_FEATURES_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_feat_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
            },
            {
                /* Characteristic: On demand ranging data */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_ONDEMAND_RD_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_od_rd_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE  ,
            },
            {
                 /* Characteristic: On realtime ranging data */
                 .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_REALTIME_RD_VAL),
                 .access_cb = gatt_svr_chr_access_ras_val,
                 .val_handle = &ble_svc_ras_rt_rd_val_handle,
                 .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE  ,
             },

            {
                /* Characteristic: RAS Control Point */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_CP_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_cp_val_handle,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_INDICATE,
            },
            {
                /* Characteristic: RAS Ranging Data Ready */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_RD_READY_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_rd_val_handle,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ ,
            },
            {
                /* Characteristic: RAS  Ranging data overwritten */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_RD_OVERWRITTEN_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_rd_ov_val_handle,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ,
            },
            {
                0, /* No more characteristics in this service */
            },
        },
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
                // print the segament data
                MODLOG_DFLT(INFO,"ble_svc_ras_od_rd_val\n");
                MODLOG_DFLT(INFO, "ble_svc_ras_od_rd_val.data = %2x %02x %02x \n",ble_svc_ras_od_rd_val->header.first_seg,ble_svc_ras_od_rd_val->header.last_seg,ble_svc_ras_od_rd_val->header.seg_counter);
                if (attr_handle == ble_svc_ras_od_rd_val_handle) {
                    rc = os_mbuf_append(ctxt->om,
                                        ble_svc_ras_od_rd_val,
                                        sizeof(struct segment )+100);
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
                ble_gatts_chr_updated(attr_handle);
                MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                "all subscribed peers.\n");
                return rc;
            } else if (attr_handle == ble_svc_ras_od_rd_val_handle) {
                rc = gatt_svr_write(ctxt->om,
                        sizeof(ble_svc_ras_od_rd_val),
                        sizeof(ble_svc_ras_od_rd_val),
                        ble_svc_ras_od_rd_val, NULL);
                ble_gatts_chr_updated(attr_handle);
                MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                "all subscribed peers.\n");
                return rc;
            } else if (attr_handle == ble_svc_ras_cp_val_handle) {
                MODLOG_DFLT(INFO, "write for control point val ");

                rc = gatt_svr_write(ctxt->om,
                        sizeof(ble_svc_ras_cp_val),
                        sizeof(ble_svc_ras_cp_val),
                        &ble_svc_ras_cp_val, NULL);
                 MODLOG_DFLT(INFO, "ble_svc_gap_cp_val = %02x %02x %02x \n",ble_svc_ras_cp_val[0],ble_svc_ras_cp_val[1],ble_svc_ras_cp_val[2]);
                if (ble_svc_ras_cp_val[0] ==  RASCP_OPCODE_GET_RD) {
                    // n = size of (rangeing_buffer[i]) / mut -4);
                    /* Run a loop to send n segmet . for now sending only 1 segment*/
                    ble_gatts_chr_updated(ble_svc_ras_od_rd_val_handle);
                } else if (ble_svc_ras_cp_val[0] == RASCP_OPCODE_ACK_RD) {
                    MODLOG_DFLT(INFO, "ack revied\n");
                    /* Reset the ranging buffers */
                    ble_svc_ras_cp_val[0]= 0x02;
                    /*Table 3.12. Response Code Values associated with Op Code 0x02*/
                    ble_svc_ras_cp_val[1]=0x01; // Success
                    ble_gatts_chr_updated(ble_svc_ras_cp_val_handle);
                    MODLOG_DFLT(INFO, "Sucssefully completed the Ranging procedure\n");

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
                ble_gatts_chr_updated(attr_handle);
                MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                "all subscribed peers.\n");
                return rc;
            } else if (attr_handle == ble_svc_ras_rd_ov_val_handle) {
                rc = gatt_svr_write(ctxt->om,
                        sizeof(ble_svc_ras_rd_ov_val),
                        sizeof(ble_svc_ras_rd_ov_val),
                        &ble_svc_ras_rd_ov_val, NULL);
                ble_gatts_chr_updated(attr_handle);
                MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                "all subscribed peers.\n");
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

    /* Check if the subevent is already stored */
    for (int i = 0; i < BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE; i++) {
        if (ranging_buffers[i].conn == ranging_subevent.subev_result.conn_handle &&
            ranging_buffers[i].ranging_counter == ranging_subevent.subev_result.procedure_counter) {
            buf = &ranging_buffers[i];
            break;
        }
    }

    if (buf == NULL) {
        /* Allocate a new buffer if not found */
        buf = ranging_buffer_alloc(ranging_subevent.subev_result.conn_handle, ranging_subevent.subev_result.procedure_counter);
        if (buf == NULL) {
            MODLOG_DFLT(ERROR,"No available buffer for storing ranging data\n");
            return;
        }
    }

    buf->ranging_data.ranging_header.config_id = ranging_subevent.subev_result.config_id;
    buf->ranging_data.ranging_header.ranging_counter = ranging_subevent.subev_result.procedure_counter;
  //  buf->ranging_data.ranging_header.selected_tx_power = ranging_subevent.subev_result.selected_tx_power;
  /* convert antena path mask using bitmask*/
    buf->ranging_data.ranging_header.antenna_paths_mask = ranging_subevent.subev_result.num_antenna_paths;

    struct subevent_header *subevent_hdr = (struct subevent_header *)(buf->ranging_data.subevents + buf->subevent_cursor);
    buf->subevent_cursor += sizeof(struct subevent_header);
    subevent_hdr->start_acl_conn_event = ranging_subevent.subev_result.start_acl_conn_event_counter;
    subevent_hdr->freq_compensation = ranging_subevent.subev_result.frequency_compensation;;
    subevent_hdr->ranging_done_status = ranging_subevent.subev_result_continue.procedure_done_status;
    subevent_hdr->subevent_done_status = ranging_subevent.subev_result_continue.subevent_done_status;
    subevent_hdr->ranging_abort_reason = ranging_subevent.subev_result.abort_reason;
    subevent_hdr->subevent_abort_reason = ranging_subevent.subev_result.abort_reason;
    subevent_hdr->ref_power_level = ranging_subevent.subev_result.reference_power_level;
    subevent_hdr->num_steps_reported = ranging_subevent.subev_result.num_steps_reported + ranging_subevent.subev_result_continue.num_steps_reported;


    /* Add step data to buf*/
    for (int i = 0; i < ranging_subevent.subev_result.num_steps_reported; i++) {
        const struct cs_steps_data *step=&ranging_subevent.subev_result.steps[i];

        buf->ranging_data.subevents[buf->subevent_cursor] = step->mode;
        buf->subevent_cursor += BLE_RAS_STEP_MODE_LEN;
        memcpy(&buf->ranging_data.subevents[buf->subevent_cursor], step->data, step->data_len);
        buf->subevent_cursor += step->data_len;

    }
    /* Create RAS segment*/
    struct segment *ras_segment;

    uint16_t max_data_len = ble_att_mtu(ranging_subevent.subev_result.conn_handle) - sizeof(struct segment_header) - 4;
    MODLOG_DFLT(INFO, "Max data len : %d\n", max_data_len);
    ras_segment= nimble_platform_mem_malloc(sizeof(struct segment)+ max_data_len);
    if (ras_segment == NULL) {
        MODLOG_DFLT(INFO, "Failed to allocate memory for RAS segment\n");
        return;
    }
    ras_segment->header.first_seg = true;
    ras_segment->header.last_seg = true;
    ras_segment->header.seg_counter = 0; // First segment
    uint16_t buf_len = sizeof(struct ranging_header) + buf->subevent_cursor;
    uint16_t remaining = buf_len - 0;
     uint16_t pull_bytes = MIN(max_data_len, remaining);
    if (max_data_len < remaining) {
        // More segments to come
        pull_bytes = max_data_len;
    }else {
        pull_bytes = remaining;
    }

    memcpy( ras_segment->data, &buf->ranging_data.buf[0], pull_bytes);

    ble_svc_ras_od_rd_val= ras_segment; // Store the segment in the global variable
    // ble_gatts_chr_updated(ble_svc_ras_od_rd_val_handle);

    // 4 bytes for header and CRC
    // now max data len is att mtu -4
    // count number  of egment required based on actudal  rd_buffer len
    // noe  set buf to  data []  of segment ensuring only one segment is created
    // if more than one segment is required then set first_seg and last_seg accordingly
    // and set seg_counter to 0 for first segment and increment it for each segment
    // and set the data len to actual data len

    // now once cp control point command is recived we will append the segment to os_mbuf buffer and indicate it to the client
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

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    reset_ranging_buffer();

}
