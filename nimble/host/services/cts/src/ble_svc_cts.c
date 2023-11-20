/**
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

#include "sysinit/sysinit.h"
#include "syscfg/syscfg.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/cts/ble_svc_cts.h"
#include "time.h"
#include <sys/time.h>


struct ble_svc_cts_cfg cts_cfg = {0};

/* characteristic values */
struct ble_svc_cts_curr_time current_local_time_val;
struct ble_svc_cts_local_time_info local_time_info_val;
struct ble_svc_cts_reference_time_info ref_time_info_val;

/* Characteristic value handles */
uint16_t ble_svc_cts_curr_time_handle;
uint16_t ble_svc_cts_local_time_info_handle;
uint16_t ble_svc_cts_ref_time_handle;

/* Access function */
static int
ble_svc_cts_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def ble_svc_cts_defs[] = {
    {
        /*** Current Time Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_CTS_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) { {
	    /*** Current Time characteristic */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_CTS_CHR_UUID16_CURRENT_TIME),
            .access_cb = ble_svc_cts_access,
	        .val_handle = &ble_svc_cts_curr_time_handle,
            .flags = BLE_GATT_CHR_F_READ |
                     BLE_GATT_CHR_F_WRITE | /* optional */
	                 BLE_GATT_CHR_F_NOTIFY
	    }, {
        /*** Local info characteristic */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_CTS_CHR_UUID16_LOCAL_TIME_INFO),
            .access_cb = ble_svc_cts_access,
            .val_handle = &ble_svc_cts_local_time_info_handle,
            .flags = BLE_GATT_CHR_F_READ |
                     BLE_GATT_CHR_F_WRITE
	    }, {
        /*** Reference time info Characteristic */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_CTS_CHR_UUID16_REF_TIME_INFO),
            .access_cb = ble_svc_cts_access,
            .val_handle = &ble_svc_cts_ref_time_handle,
            .flags = BLE_GATT_CHR_F_READ
	    }, {
            0, /* No more characteristics in this service. */
        } },
    },

    {
        0, /* No more services. */
    },
};

static int
ble_svc_cts_chr_write(struct os_mbuf *om, uint16_t min_len,
                      uint16_t max_len, void *dst,
                      uint16_t *len)
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

int ble_svc_cts_curr_time_validate(struct ble_svc_cts_curr_time curr_time) {
    if(curr_time.et_256.d_d_t.day_of_week > 7  ||
       (curr_time.et_256.d_d_t.d_t.year < 1582)||
       curr_time.et_256.d_d_t.d_t.year > 9999  ||
       curr_time.et_256.d_d_t.d_t.month > 12   ||
       curr_time.et_256.d_d_t.d_t.day > 31     ||
       curr_time.et_256.d_d_t.d_t.hours > 23   ||
       curr_time.et_256.d_d_t.d_t.minutes > 59 ||
       curr_time.et_256.d_d_t.d_t.seconds > 59 ||
       curr_time.adjust_reason >> 4 > 0
       ) {
        return BLE_SVC_CTS_ERR_DATA_FIELD_IGNORED;
    }
    return 0;
}

int ble_svc_cts_local_time_info_validate(struct ble_svc_cts_local_time_info local_time_info) {
    if((local_time_info.timezone < -48    &&
        local_time_info.timezone != -128) ||
        local_time_info.timezone >  56    ||
        (local_time_info.dst_offset != 0  &&
        local_time_info.dst_offset != 2   &&
        local_time_info.dst_offset != 4   &&
        local_time_info.dst_offset != 8   &&
        local_time_info.dst_offset != 255)
      ) {
        return BLE_SVC_CTS_ERR_DATA_FIELD_IGNORED;
    }
    return 0;
}
/**
 * CTS access function
 */
static int
ble_svc_cts_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg)
{
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    struct ble_svc_cts_local_time_info local_time_info;
    struct ble_svc_cts_curr_time curr_time;
    int rc = 0;

    switch (uuid16) {
    case BLE_SVC_CTS_CHR_UUID16_CURRENT_TIME:
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR);
        switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            if(cts_cfg.fetch_time_cb != NULL) {
                rc = cts_cfg.fetch_time_cb(&current_local_time_val);
                if(rc != 0) {
                   return BLE_ATT_ERR_UNLIKELY;
                }
            }
            else {
                memset(&current_local_time_val, 0, sizeof(current_local_time_val));
            }
            rc = os_mbuf_append(ctxt->om, &current_local_time_val,
                                sizeof current_local_time_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_hs_mbuf_to_flat(ctxt->om, &curr_time, sizeof curr_time, NULL);
            assert(rc == 0);
            rc = ble_svc_cts_curr_time_validate(curr_time);
            if(rc != 0) {
                return rc;
            }
            rc = ble_svc_cts_chr_write(ctxt->om, 0, sizeof current_local_time_val, &current_local_time_val, NULL);
            if(rc != 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }

            if(cts_cfg.set_time_cb == NULL) {
                return BLE_ATT_ERR_UNLIKELY;
            }

            rc = cts_cfg.set_time_cb(curr_time);
            if(rc != 0) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            /* schedule notifications for subscribed peers */
            if(rc == 0) {
                ble_gatts_chr_updated(attr_handle);
            }
            return 0;
        }

    case BLE_SVC_CTS_CHR_UUID16_LOCAL_TIME_INFO:
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR);
        switch(ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            if(cts_cfg.local_time_info_cb != NULL) {
                rc = cts_cfg.local_time_info_cb(&local_time_info_val);
                if(rc != 0) {
                   return BLE_ATT_ERR_UNLIKELY;
                }
            }
            else {
                memset(&local_time_info_val, 0, sizeof(local_time_info_val)); /* 0 is unknown */
            }
            rc = os_mbuf_append(ctxt->om, &local_time_info_val,
                                sizeof local_time_info_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_hs_mbuf_to_flat(ctxt->om, &local_time_info, 2, NULL);
            assert(rc == 0);
            rc = ble_svc_cts_local_time_info_validate(local_time_info);
            if(rc != 0) {
                return rc;
            }
            rc = ble_svc_cts_chr_write(ctxt->om, 0, sizeof local_time_info_val, &local_time_info_val, NULL);
            if(rc != 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }

            if(cts_cfg.set_local_time_info_cb == NULL) {
              return BLE_ATT_ERR_UNLIKELY;
            }

            rc = cts_cfg.set_local_time_info_cb(local_time_info);
            if(rc != 0) {
               return BLE_ATT_ERR_UNLIKELY;
            }
            if(rc == 0) {
                /* set the adjust reason mask */
                /* notify the connected clients about the change in timezone and time */
                ble_gatts_chr_updated(ble_svc_cts_curr_time_handle);
            }
            return 0;
        }
    case BLE_SVC_CTS_CHR_UUID16_REF_TIME_INFO:
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
        if(cts_cfg.ref_time_info_cb != NULL) {
            rc = cts_cfg.ref_time_info_cb(&ref_time_info_val);
            if(rc != 0) {
               return BLE_ATT_ERR_UNLIKELY;
            }
        }
        rc = os_mbuf_append(ctxt->om, &ref_time_info_val,
                            sizeof ref_time_info_val);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/* this function is invoked to make the service aware that the time is updated */
void ble_svc_cts_time_updated() {
    /* notify the bonded clients */
    ble_gatts_chr_updated(ble_svc_cts_curr_time_handle);
}

/**
 * Initialize the Current Time Service.
 */
void
ble_svc_cts_init(struct ble_svc_cts_cfg cfg)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();
    memcpy(&cts_cfg, &cfg, sizeof cfg);

    rc = ble_gatts_count_cfg(ble_svc_cts_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_cts_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);
}
