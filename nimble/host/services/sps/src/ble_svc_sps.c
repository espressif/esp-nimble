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
#include "host/ble_hs.h"
#include "services/sps/ble_svc_sps.h"

static uint16_t ble_scan_itvl;
static uint16_t ble_scan_window;
static uint8_t ble_scan_refresh;
static uint16_t ble_scan_itvl_handle;
static uint16_t ble_scan_refresh_handle;


/* Access function */
static int
ble_svc_sps_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def ble_svc_sps_defs[] = {
    { /*** Service: Device Information Service (SPS). */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPS_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) { {
	    /*** Characteristic: Scan Interval */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPS_CHR_UUID16_SCAN_ITVL_WINDOW),
            .access_cb = ble_svc_sps_access,
            .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
            .val_handle = &ble_scan_itvl_handle,
            },  {
	    /*** Characteristic: Scan Refresh */
            .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPS_CHR_UUID16_SCAN_REFRESH),
            .access_cb = ble_svc_sps_access,
            .flags = BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &ble_scan_refresh_handle,
            },  {
            0, /* No more characteristics in this service */
        }, }
    },

    {
        0, /* No more services. */
    },
};

static int
ble_svc_sps_chr_write(struct os_mbuf *om, uint16_t min_len,
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

void ble_svc_sps_scan_refresh() {
    /* spec allows only value 0 to send */
    ble_scan_refresh = 0;
    ble_gatts_chr_updated(ble_scan_refresh_handle);
}

static int
ble_svc_sps_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid = ble_uuid_u16(ctxt->chr->uuid);
    uint32_t write_val;
    int rc;

    switch(uuid) {
    case BLE_SVC_SPS_CHR_UUID16_SCAN_ITVL_WINDOW:
        assert(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR);
        rc = ble_svc_sps_chr_write(ctxt->om, 0, sizeof(ble_scan_itvl) + sizeof(ble_scan_window), &write_val, NULL);
        if(rc != 0) {
            ble_scan_itvl = (write_val & 0xffff0000) >> 16;
            ble_scan_window = (write_val && 0x0000ffff);
        }
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    case BLE_SVC_SPS_CHR_UUID16_SCAN_REFRESH:
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR && conn_handle == BLE_HS_CONN_HANDLE_NONE);
        rc = os_mbuf_append(ctxt->om, &ble_scan_refresh,
                sizeof ble_scan_refresh);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}


/**
 * Initialize the SPS package.
 */
void
ble_svc_sps_init(uint16_t scan_itvl, uint16_t scan_window)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    ble_scan_itvl = scan_itvl;
    ble_scan_window = scan_window;
    rc = ble_gatts_count_cfg(ble_svc_sps_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_sps_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);
}
