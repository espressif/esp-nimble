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
 #include <string.h>
 #include "host/ble_hs.h"
 #include "host/ble_uuid.h"
 #include "services/ras/ble_svc_ras.h"

 /* Char values */
static uint32_t ble_svc_ras_feat_val;
static uint16_t ble_svc_ras_rd_val;
static uint16_t ble_svc_ras_rd_ov_val;
static uint16_t ble_svc_ras_cp_val;

static uint16_t ble_svc_ras_feat_val_handle;
static uint16_t ble_svc_ras_od_val_handle;
static uint16_t ble_svc_ras_rd_val_handle;
static uint16_t ble_svc_ras_rd_ov_val_handle;
static uint16_t ble_svc_ras_cp_val_handle;

static int
gatt_svr_chr_access_ras_val(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);

static int
gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
{
        /* Service: Ranging Data Service */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_RANGING_SERVICE_VAL),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: Feature Value */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_FEATURES_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_feat_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
          },{
                /* Characteristic: On demand ranging data */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_ONDEMAND_RD_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_od_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
          },{
                /* Characteristic: RAS Control Point */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_CP_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_cp_val_handle,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_INDICATE,
          },{
                /* Characteristic: RAS Data Ready */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_RD_READY_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_rd_val_handle,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
          },{
                /* Characteristic: RAS data overwritten */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_RAS_CHR_UUID_RD_OVERWRITTEN_VAL),
                .access_cb = gatt_svr_chr_access_ras_val,
                .val_handle = &ble_svc_ras_rd_ov_val_handle,
                .flags = BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
          },{
                 0, /* No more characteristics in this service */
          },
       }
     },
     {
         0, /* No more services */
     },
};


static int
gatt_svr_chr_access_ras_val(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
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
       if (attr_handle == ble_svc_ras_feat_val_handle) {
           rc = os_mbuf_append(ctxt->om,
                               &ble_svc_ras_feat_val,
                               sizeof(ble_svc_ras_feat_val));
           return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
       } else if (attr_handle == ble_svc_ras_rd_val_handle) {
           rc = os_mbuf_append(ctxt->om,
                               &ble_svc_ras_rd_val,
                               sizeof(ble_svc_ras_rd_val));
           return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
       } else if (attr_handle == ble_svc_ras_rd_ov_val_handle) {
           rc = os_mbuf_append(ctxt->om,
                               &ble_svc_ras_rd_ov_val,
                               sizeof(ble_svc_ras_rd_ov_val));
           return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
       }
       return BLE_ATT_ERR_UNLIKELY;

   case BLE_GATT_ACCESS_OP_WRITE_CHR:
       if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
           MODLOG_DFLT(INFO, "Characteristic write; conn_handle=%d attr_handle=%d",
                       conn_handle, attr_handle);
       } else {
           MODLOG_DFLT(INFO, "Characteristic write by NimBLE stack; attr_handle=%d",
                       attr_handle);
       }
       if (attr_handle == ble_svc_ras_cp_val_handle) {
           rc = gatt_svr_write(ctxt->om,
                               RASCP_CMD_OPCODE_LEN,
                               sizeof(ble_svc_ras_cp_val),
                               &ble_svc_ras_cp_val, NULL);
           return rc;
       }
       return BLE_ATT_ERR_UNLIKELY;

   default:
       break;
   }

   /* Unknown characteristic/descriptor */
   return BLE_ATT_ERR_UNLIKELY;
 }

void
custom_gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
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
        break;
    }
}

int
ble_svc_ras_rrsp_init(void)
{
    int rc;

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
