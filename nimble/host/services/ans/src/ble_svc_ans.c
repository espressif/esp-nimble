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
#include "../../../src/ble_hs_priv.h"
#include "host/ble_gap.h"
#include "services/ans/ble_svc_ans.h"
#include "host/ble_hs_log.h"

#if MYNEWT_VAL(BLE_GATTS) && CONFIG_BT_NIMBLE_ANS_SERVICE
/* Max length of new alert info string */
#define BLE_SVC_ANS_INFO_STR_MAX_LEN        18
/* Max length of a new alert notification, max string length + 2 bytes
 * for category ID and count. */
#define BLE_SVC_ANS_NEW_ALERT_MAX_LEN   (BLE_SVC_ANS_INFO_STR_MAX_LEN + 2)

/* Category ID for "all categories" */
#define BLE_SVC_ANS_CAT_ID_ALL  0xFF

/* Supported categories bitmasks */
static uint8_t ble_svc_ans_new_alert_cat;
static uint8_t ble_svc_ans_unr_alert_cat;

/* Notification state bitmasks */
static uint8_t ble_svc_ans_new_alert_notif_state;
static uint8_t ble_svc_ans_unr_alert_notif_state;

/* Characteristic values */
static uint8_t ble_svc_ans_new_alert_val[BLE_SVC_ANS_NEW_ALERT_MAX_LEN];
static uint16_t ble_svc_ans_new_alert_val_len;
static uint8_t ble_svc_ans_unr_alert_stat[2];
static uint8_t ble_svc_ans_alert_not_ctrl_pt[2];

/* Alert counts, one value for each category */
static uint8_t ble_svc_ans_new_alert_cnt[BLE_SVC_ANS_CAT_NUM];
static uint8_t ble_svc_ans_unr_alert_cnt[BLE_SVC_ANS_CAT_NUM];

/* Characteristic value handles */
static uint16_t ble_svc_ans_new_alert_val_handle;
static uint16_t ble_svc_ans_unr_alert_val_handle;

/* Connection handle
 *
 * TODO: In order to support multiple connections we would need to save
 *       the handles for every connection, not just the most recent. Then
 *       we would need to notify each connection when needed.
 * */
static uint16_t ble_svc_ans_conn_handle;

/* Access function */
static int
ble_svc_ans_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Notify new alert */
static int
ble_svc_ans_new_alert_notify(uint8_t cat_id, const char * info_str);

/* Notify unread alert */
static int
ble_svc_ans_unr_alert_notify(uint8_t cat_id);

/* Save written value to local characteristic value */
static int
ble_svc_ans_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                      void *dst, uint16_t *len);

static const ble_uuid16_t uuid_svc_ans = BLE_UUID16_INIT(BLE_SVC_ANS_UUID16);
static const ble_uuid16_t uuid_chr_new_alert_cat = BLE_UUID16_INIT(BLE_SVC_ANS_CHR_UUID16_SUP_NEW_ALERT_CAT);
static const ble_uuid16_t uuid_chr_new_alert = BLE_UUID16_INIT(BLE_SVC_ANS_CHR_UUID16_NEW_ALERT);
static const ble_uuid16_t uuid_chr_unr_alert_cat = BLE_UUID16_INIT(BLE_SVC_ANS_CHR_UUID16_SUP_UNR_ALERT_CAT);
static const ble_uuid16_t uuid_chr_unr_alert_stat = BLE_UUID16_INIT(BLE_SVC_ANS_CHR_UUID16_UNR_ALERT_STAT);
static const ble_uuid16_t uuid_chr_alert_notif_ctrl_pt = BLE_UUID16_INIT(BLE_SVC_ANS_CHR_UUID16_ALERT_NOT_CTRL_PT);

static const struct ble_gatt_chr_def ans_characteristics[] = {
        {
            /** Supported New Alert Category
             *
             * This characteristic exposes what categories of new
             * alert are supported in the server.
             */
            .uuid = &uuid_chr_new_alert_cat.u,
            .access_cb = ble_svc_ans_access,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            /** New Alert
             *
             * This characteristic exposes information about
             * the count of new alerts (for a given category).
             */
            .uuid = &uuid_chr_new_alert.u,
            .access_cb = ble_svc_ans_access,
            .val_handle = &ble_svc_ans_new_alert_val_handle,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, {
            /** Supported Unread Alert Category
             *
             * This characteristic exposes what categories of
             * unread alert are supported in the server.
             */
            .uuid = &uuid_chr_unr_alert_cat.u,
            .access_cb = ble_svc_ans_access,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            /** Unread Alert Status
             *
             * This characteristic exposes the count of unread
             * alert events existing in the server.
             */
            .uuid = &uuid_chr_unr_alert_stat.u,
            .access_cb = ble_svc_ans_access,
            .val_handle = &ble_svc_ans_unr_alert_val_handle,
            .flags = BLE_GATT_CHR_F_NOTIFY,
        }, {
            /** Alert Notification Control Point
             *
             * This characteristic allows the peer device to
             * enable/disable the alert notification of new alert
             * and unread event more selectively than can be done
             * by setting or clearing the notification bit in the
             * Client Characteristic Configuration for each alert
             * characteristic.
             */
            .uuid = &uuid_chr_alert_notif_ctrl_pt.u,
            .access_cb = ble_svc_ans_access,
            .flags = BLE_GATT_CHR_F_WRITE,
        }, {
            0, /* No more characteristics in this service. */
        }
};

static const struct ble_gatt_svc_def ble_svc_ans_defs[] = {
    {
        /*** Alert Notification Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_ans.u,
        .characteristics = ans_characteristics,
    },

    {
        0, /* No more services. */
    },
};

/**
 * ANS access function
 */
static int
ble_svc_ans_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg)
{
    uint16_t uuid16;
    int rc;

    /* ANS Control point command and category variables */
    uint8_t cmd_id;
    uint8_t cat_id;
    uint8_t cat_bit_mask;
    int i;

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    if (uuid16 == 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    switch (uuid16) {
    case BLE_SVC_ANS_CHR_UUID16_SUP_NEW_ALERT_CAT:
        if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        ble_hs_lock();
        rc = os_mbuf_append(ctxt->om, &ble_svc_ans_new_alert_cat,
                            sizeof ble_svc_ans_new_alert_cat);
        ble_hs_unlock();
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_SVC_ANS_CHR_UUID16_NEW_ALERT:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            ble_hs_lock();
            rc = ble_svc_ans_chr_write(ctxt->om, 0,
                                       sizeof ble_svc_ans_new_alert_val,
                                       ble_svc_ans_new_alert_val,
                                       &ble_svc_ans_new_alert_val_len);
            ble_hs_unlock();
            return rc;

        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            ble_hs_lock();
            rc = os_mbuf_append(ctxt->om, &ble_svc_ans_new_alert_val,
                                sizeof ble_svc_ans_new_alert_val);
            ble_hs_unlock();
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        return BLE_ATT_ERR_UNLIKELY;
    case BLE_SVC_ANS_CHR_UUID16_SUP_UNR_ALERT_CAT:
        if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        ble_hs_lock();
        rc = os_mbuf_append(ctxt->om, &ble_svc_ans_unr_alert_cat,
                            sizeof ble_svc_ans_unr_alert_cat);
        ble_hs_unlock();
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_SVC_ANS_CHR_UUID16_UNR_ALERT_STAT:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            ble_hs_lock();
            rc = ble_svc_ans_chr_write(ctxt->om,
                                       sizeof ble_svc_ans_unr_alert_stat,
                                       sizeof ble_svc_ans_unr_alert_stat,
                                       &ble_svc_ans_unr_alert_stat,
                                       NULL);
            ble_hs_unlock();
            return rc;
        } else {
            ble_hs_lock();
            rc = os_mbuf_append(ctxt->om, &ble_svc_ans_unr_alert_stat,
                                sizeof ble_svc_ans_unr_alert_stat);
            ble_hs_unlock();
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

    case BLE_SVC_ANS_CHR_UUID16_ALERT_NOT_CTRL_PT:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            rc = ble_svc_ans_chr_write(ctxt->om,
                                       sizeof ble_svc_ans_alert_not_ctrl_pt,
                                       sizeof ble_svc_ans_alert_not_ctrl_pt,
                                       &ble_svc_ans_alert_not_ctrl_pt,
                                       NULL);
            if (rc != 0) {
                return rc;
            }

            /* Get command ID and category ID */
            cmd_id = ble_svc_ans_alert_not_ctrl_pt[0];
            cat_id = ble_svc_ans_alert_not_ctrl_pt[1];


            /* Set cat_bit_mask to the appropriate bitmask based on cat_id */
            if (cat_id < BLE_SVC_ANS_CAT_NUM) {
                cat_bit_mask = (1 << cat_id);
            } else if (cat_id == BLE_SVC_ANS_CAT_ID_ALL) {
                cat_bit_mask = UINT8_MAX;
            } else {
                /* invalid category ID */
                return BLE_ATT_ERR_UNLIKELY;
            }

            switch (cmd_id) {
            case BLE_SVC_ANS_CMD_EN_NEW_ALERT_CAT:
                ble_hs_lock();
                ble_svc_ans_new_alert_notif_state |= (cat_bit_mask & ble_svc_ans_new_alert_cat);
                ble_hs_unlock();
                break;
            case BLE_SVC_ANS_CMD_EN_UNR_ALERT_CAT:
                ble_hs_lock();
                ble_svc_ans_unr_alert_notif_state |= (cat_bit_mask & ble_svc_ans_unr_alert_cat);
                ble_hs_unlock();
                break;
            case BLE_SVC_ANS_CMD_DIS_NEW_ALERT_CAT:
                ble_hs_lock();
                ble_svc_ans_new_alert_notif_state &= ~cat_bit_mask;
                ble_hs_unlock();
                break;
            case BLE_SVC_ANS_CMD_DIS_UNR_ALERT_CAT:
                ble_hs_lock();
                ble_svc_ans_unr_alert_notif_state &= ~cat_bit_mask;
                ble_hs_unlock();
                break;
            case BLE_SVC_ANS_CMD_NOT_NEW_ALERT_IMMEDIATE:
                if (cat_id == BLE_SVC_ANS_CAT_ID_ALL) {
                    /* If cat_id is 0xff, notify on all enabled categories */
                    for (i = BLE_SVC_ANS_CAT_NUM - 1; i >= 0; --i) {
                        ble_hs_lock();
                        if ((ble_svc_ans_new_alert_notif_state >> i) & 0x01) {
                            ble_hs_unlock();
                            ble_svc_ans_new_alert_notify(i, NULL);
                        } else {
                            ble_hs_unlock();
                        }
                    }
                } else {
                    ble_hs_lock();
                    if ((ble_svc_ans_new_alert_notif_state >> cat_id) & 0x01){
                        ble_hs_unlock();
                        ble_svc_ans_new_alert_notify(cat_id, NULL);
                    } else {
                        ble_hs_unlock();
                    }
                }
                break;
            case BLE_SVC_ANS_CMD_NOT_UNR_ALERT_IMMEDIATE:
                if (cat_id == BLE_SVC_ANS_CAT_ID_ALL) {
                    /* If cat_id is 0xff, notify on all enabled categories */
                    for (i = BLE_SVC_ANS_CAT_NUM - 1; i >= 0; --i) {
                        ble_hs_lock();
                        if ((ble_svc_ans_unr_alert_notif_state >> i) & 0x01) {
                            ble_hs_unlock();
                            ble_svc_ans_unr_alert_notify(i);
                        } else {
                            ble_hs_unlock();
                        }
                    }
                } else {
                    ble_hs_lock();
                    if ((ble_svc_ans_unr_alert_notif_state >> cat_id) & 0x01) {
                        ble_hs_unlock();
                        ble_svc_ans_unr_alert_notify(cat_id);
                    } else {
                        ble_hs_unlock();
                    }
                }
                break;
            default:
                return BLE_SVC_ANS_ERR_CMD_NOT_SUPPORTED;
            }
            return 0;
        } else {
            rc = BLE_ATT_ERR_UNLIKELY;
        }
        return rc;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * This function must be called with the connection handle when a gap
 * connect event is received in order to send notifications to the
 * client.
 *
 * @params conn_handle          The connection handle for the current
 *                                  connection.
 */
void
ble_svc_ans_on_gap_connect(uint16_t conn_handle)
{
    ble_hs_lock();
    ble_svc_ans_conn_handle = conn_handle;
    ble_hs_unlock();
}

/**
 * Adds a new alert to the given category then notifies the client
 * if the given category is valid and enabled.
 *
 * @param cat_flag              The id for the category which should
 *                                  should be incremented and notified
 * @param info_str              The info string to be sent to the client
 *                                  with the notification.
 *
 * @return 0 on success, non-zero error code otherwise.
 */
int
ble_svc_ans_new_alert_add(uint8_t cat_id, const char * info_str)
{
    uint8_t cat_bit_mask;
    int rc = 0;

    if (cat_id < BLE_SVC_ANS_CAT_NUM) {
        cat_bit_mask = (1 << cat_id);
    } else {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    ble_hs_lock();
    if ((cat_bit_mask & ble_svc_ans_new_alert_cat) == 0) {
        ble_hs_unlock();
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    if (ble_svc_ans_new_alert_cnt[cat_id] < 0xFF) {
        ble_svc_ans_new_alert_cnt[cat_id] += 1;
    }

    if ((cat_bit_mask & ble_svc_ans_new_alert_notif_state) != 0) {
        ble_hs_unlock();
        rc = ble_svc_ans_new_alert_notify(cat_id, info_str);
    } else {
        ble_hs_unlock();
    }

    return rc;
}

/**
 * Adds an unread alert to the given category then notifies the client
 * if the given category is valid and enabled.
 *
 * @param cat_flag              The flag for the category which should
 *                                  should be incremented and notified
 *
 * @return 0 on success, non-zero error code otherwise.
 */
int
ble_svc_ans_unr_alert_add(uint8_t cat_id)
{
    uint8_t cat_bit_mask;
    int rc = 0;

    if (cat_id < BLE_SVC_ANS_CAT_NUM) {
        cat_bit_mask = 1 << cat_id;
    } else {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    ble_hs_lock();
    if ((cat_bit_mask & ble_svc_ans_unr_alert_cat) == 0) {
        ble_hs_unlock();
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    if (ble_svc_ans_unr_alert_cnt[cat_id] < 0xFF) {
        ble_svc_ans_unr_alert_cnt[cat_id] += 1;
    }

    if ((cat_bit_mask & ble_svc_ans_unr_alert_notif_state) != 0) {
        ble_hs_unlock();
        rc = ble_svc_ans_unr_alert_notify(cat_id);
    } else {
        ble_hs_unlock();
    }

    return rc;
}

/**
 * Send a new alert notification to the given category with the
 * given info string.
 *
 * @param cat_id                The ID of the category to send the
 *                                  notification to.
 * @param info_str              The info string to send with the
 *                                  notification
 *
 * @return 0 on success, non-zero error code otherwise.
 */
static int
ble_svc_ans_new_alert_notify(uint8_t cat_id, const char * info_str)
{
    int info_str_len;
    int rc;
    uint16_t conn_handle;

    ble_hs_lock();
    /* Clear notification to remove old information that may persist */
    memset(&ble_svc_ans_new_alert_val, '\0',
           BLE_SVC_ANS_NEW_ALERT_MAX_LEN);

    /* Set ID and count values */
    ble_svc_ans_new_alert_val[0] = cat_id;
    ble_svc_ans_new_alert_val[1] = ble_svc_ans_new_alert_cnt[cat_id];

    if (info_str) {
        info_str_len = strlen(info_str);
        if (info_str_len > BLE_SVC_ANS_INFO_STR_MAX_LEN) {
            /* If info_str is longer than the max string length only
             * write up to the maximum length */
            memcpy(&ble_svc_ans_new_alert_val[2], info_str,
                   BLE_SVC_ANS_INFO_STR_MAX_LEN);
        } else {
            memcpy(&ble_svc_ans_new_alert_val[2], info_str, info_str_len);
        }
    }
    conn_handle = ble_svc_ans_conn_handle;
    ble_hs_unlock();

#if NIMBLE_BLE_CONNECT
    rc = ble_gatts_notify(conn_handle,
                          ble_svc_ans_new_alert_val_handle);
#else
    rc = 0;
#endif
    return rc;
}

/**
 * Send an unread alert notification to the given category.
 *
 * @param cat_id                The ID of the category to send the
 *                                  notification to.
 *
 * @return 0 on success, non-zero error code otherwise.
 */
static int
ble_svc_ans_unr_alert_notify(uint8_t cat_id)
{
    int rc;
    uint16_t conn_handle;

    ble_hs_lock();
    ble_svc_ans_unr_alert_stat[0] = cat_id;
    ble_svc_ans_unr_alert_stat[1] = ble_svc_ans_unr_alert_cnt[cat_id];
    conn_handle = ble_svc_ans_conn_handle;
    ble_hs_unlock();

#if NIMBLE_BLE_CONNECT
    rc = ble_gatts_notify(conn_handle,
                          ble_svc_ans_unr_alert_val_handle);
#else
    rc = 0;
#endif
    return rc;
}

/**
 * Writes the received value from a characteristic write to
 * the given destination.
 */
static int
ble_svc_ans_chr_write(struct os_mbuf *om, uint16_t min_len,
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

/**
 * Initialize the ANS with initial values for enabled categories
 * for new and unread alert characteristics. Bitwise or the
 * category bitmasks to enable multiple categories.
 *
 * XXX: We should technically be able to change the new alert and
 *      unread alert categories when we have no active connections.
 */
void
ble_svc_ans_init(void)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    rc = ble_gatts_count_cfg(ble_svc_ans_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);
    (void)rc;

    rc = ble_gatts_add_svcs(ble_svc_ans_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);
    (void)rc;

    ble_svc_ans_new_alert_cat = MYNEWT_VAL(BLE_SVC_ANS_NEW_ALERT_CAT);
    ble_svc_ans_unr_alert_cat = MYNEWT_VAL(BLE_SVC_ANS_UNR_ALERT_CAT);
}
#endif
