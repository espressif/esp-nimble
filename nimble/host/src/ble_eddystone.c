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

#include <string.h>
#include "os/endian.h"
#include "nimble/nimble_opt.h"
#include "host/ble_eddystone.h"
#include "host/ble_hs_adv.h"
#include "ble_hs_priv.h"
#include "host/ble_hs_log.h"
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#include "esp_nimble_mem.h"
#endif

#define BLE_EDDYSTONE_MAX_SVC_DATA_LEN  22
#define BLE_EDDYSTONE_SVC_DATA_BASE_SZ  3

#define BLE_EDDYSTONE_SERVICE_UUID      0xfeaa

#define BLE_EDDYSTONE_FRAME_TYPE_UID    0x00
#define BLE_EDDYSTONE_FRAME_TYPE_URL    0x10

#define BLE_EDDYSTONE_UID_LEN           16
#define BLE_EDDYSTONE_UID_RFU_LEN       2
#define BLE_EDDYSTONE_UID_SVC_DATA_LEN  (1 + BLE_EDDYSTONE_UID_LEN + BLE_EDDYSTONE_UID_RFU_LEN)

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
typedef struct {
    ble_uuid16_t _ble_eddystone_uuids16[BLE_EDDYSTONE_MAX_UUIDS16 + 1];
    uint8_t _ble_eddystone_svc_data[BLE_EDDYSTONE_MAX_SVC_DATA_LEN];
} ble_eddystone_ctx_t;

static ble_eddystone_ctx_t *ble_eddystone_ctx;

#define ble_eddystone_uuids16 (ble_eddystone_ctx->_ble_eddystone_uuids16)
#define ble_eddystone_svc_data (ble_eddystone_ctx->_ble_eddystone_svc_data)

#else

static ble_uuid16_t ble_eddystone_uuids16[BLE_EDDYSTONE_MAX_UUIDS16 + 1];
static uint8_t ble_eddystone_svc_data[BLE_EDDYSTONE_MAX_SVC_DATA_LEN];

#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
int
ble_eddystone_ensure_ctx_init()
{
    if (ble_eddystone_ctx == NULL) {
        ble_eddystone_ctx = nimble_platform_mem_calloc(1, sizeof(ble_eddystone_ctx_t));
        if (ble_eddystone_ctx == NULL) {
            return BLE_HS_ENOMEM;
        }
    }
    return 0;
}

void
ble_eddystone_ctx_deinit()
{
    if (ble_eddystone_ctx) {
        nimble_platform_mem_free(ble_eddystone_ctx);
        ble_eddystone_ctx = NULL;
    }
}
#endif

/**
 * Writes an eddystone header to the global service data buffer.
 *
 * @param frame_type                The eddystone frame type; one of the
 *                                      BLE_EDDYSTONE_FRAME_TYPE_[...] values.
 *
 * @return                          A pointer to where the service data payload
 *                                      should be written.
 */
static void *
ble_eddystone_set_svc_data_base(uint8_t frame_type)
{
    put_le16(ble_eddystone_svc_data, BLE_EDDYSTONE_SERVICE_UUID);
    ble_eddystone_svc_data[2] = frame_type;

    return ble_eddystone_svc_data + BLE_EDDYSTONE_SVC_DATA_BASE_SZ;
}

/**
 * Populates the supplied advertisement fields struct to represent an eddystone
 * advertisement.  Prior to calling this function, you must write the service
 * data header and payload using the ble_eddystone_set_svc_data_base()
 * function.
 *
 * @param adv_fields            The base advertisement fields to transform into
 *                                  an eddystone beacon.  All configured fields
 *                                  are preserved; you probably want to clear
 *                                  this struct before calling this function.
 * @param svc_data_len          The amount of data written to the global
 *                                  service data buffer.
 *
 * @return                      0 on success; BLE_HS_E... on failure.
 */
static int
ble_eddystone_set_adv_data_gen(struct ble_hs_adv_fields *adv_fields,
                               uint8_t svc_data_len)
{
    int rc;

    if (adv_fields->num_uuids16 > BLE_EDDYSTONE_MAX_UUIDS16) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (svc_data_len > (BLE_EDDYSTONE_MAX_SVC_DATA_LEN - BLE_EDDYSTONE_SVC_DATA_BASE_SZ)) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (adv_fields->num_uuids16 > 0 && !adv_fields->uuids16_is_complete) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (adv_fields->svc_data_uuid16_len != 0) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    ble_eddystone_uuids16[0] =
        (ble_uuid16_t) BLE_UUID16_INIT(BLE_EDDYSTONE_SERVICE_UUID);

    /* Only copy if there are UUIDs and the pointer is valid */
    if (adv_fields->num_uuids16 > 0) {
        if (adv_fields->uuids16 == NULL) {
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
            return BLE_HS_EINVAL;
        }
        memcpy(ble_eddystone_uuids16 + 1, adv_fields->uuids16,
               adv_fields->num_uuids16 * sizeof(ble_uuid16_t));
    }
    adv_fields->uuids16 = ble_eddystone_uuids16;
    adv_fields->num_uuids16++;
    adv_fields->uuids16_is_complete = 1;

    adv_fields->svc_data_uuid16 = ble_eddystone_svc_data;
    adv_fields->svc_data_uuid16_len = svc_data_len +
                                      BLE_EDDYSTONE_SVC_DATA_BASE_SZ;

    rc = ble_gap_adv_set_fields(adv_fields);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

int
ble_eddystone_set_adv_data_uid(struct ble_hs_adv_fields *adv_fields,
                               const void *uid, int8_t measured_power)
{
    uint8_t *svc_data;
    int rc;

    /* Validate inputs before modifying global state */
    if (adv_fields == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (uid == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (measured_power < -127 || measured_power > 20) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    rc = ble_eddystone_ensure_ctx_init();
    if (rc != 0) {
        return rc;
    }
#endif

    /* Eddystone UUID and frame type (0). */
    svc_data = ble_eddystone_set_svc_data_base(BLE_EDDYSTONE_FRAME_TYPE_UID);

    /* Measured Power ranging data (Calibrated tx power at 0 meters). */
    svc_data[0] = measured_power;

    /* UID. */
    memcpy(svc_data + 1, uid, BLE_EDDYSTONE_UID_LEN);

    /* Reserved. */
    svc_data[1 + BLE_EDDYSTONE_UID_LEN] = 0x00;
    svc_data[1 + BLE_EDDYSTONE_UID_LEN + 1] = 0x00;

    rc = ble_eddystone_set_adv_data_gen(adv_fields, BLE_EDDYSTONE_UID_SVC_DATA_LEN);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

int
ble_eddystone_set_adv_data_url(struct ble_hs_adv_fields *adv_fields,
                               uint8_t url_scheme, const char *url_body,
                               uint8_t url_body_len, uint8_t url_suffix,
                               int8_t measured_power)
{
    uint8_t *svc_data;
    int url_len;
    int rc;

    /* Validate all inputs before modifying global state */
    if (adv_fields == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (url_body == NULL && url_body_len > 0) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (measured_power < -127 || measured_power > 20) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (url_scheme > BLE_EDDYSTONE_URL_SCHEME_HTTPS) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }
    if (url_suffix != BLE_EDDYSTONE_URL_SUFFIX_NONE && url_suffix > BLE_EDDYSTONE_URL_SUFFIX_GOV) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    rc = ble_eddystone_ensure_ctx_init();
    if (rc != 0) {
        return rc;
    }
#endif

    url_len = url_body_len;
    if (url_suffix != BLE_EDDYSTONE_URL_SUFFIX_NONE) {
        url_len++;
    }
    if (url_len < 1 || url_len > BLE_EDDYSTONE_URL_MAX_LEN) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    svc_data = ble_eddystone_set_svc_data_base(BLE_EDDYSTONE_FRAME_TYPE_URL);

    /* Measured Power ranging data (Calibrated tx power at 0 meters). */
    svc_data[0] = measured_power;

    svc_data[1] = url_scheme;
    if (url_body_len > 0) {
        memcpy(svc_data + 2, url_body, url_body_len);
    }
    if (url_suffix != BLE_EDDYSTONE_URL_SUFFIX_NONE) {
        svc_data[2 + url_body_len] = url_suffix;
    }

    rc = ble_eddystone_set_adv_data_gen(adv_fields, url_len + 2);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
