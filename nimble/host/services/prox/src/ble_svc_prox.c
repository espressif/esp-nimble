/*
 * SPDX-FileCopyrightText: 2017-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "syscfg/syscfg.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "modlog/modlog.h"
#include "services/prox/ble_svc_prox.h"

#if MYNEWT_VAL(BLE_GATTS) && CONFIG_BT_NIMBLE_PROX_SERVICE
/* Characteristic values */
static uint8_t ble_svc_prox_alert;
static int8_t ble_svc_prox_tx_pwr_lvl;

#define BLE_SVC_PROX_ALERT_NONE      0
#define BLE_SVC_PROX_ALERT_MILD      1
#define BLE_SVC_PROX_ALERT_HIGH      2
#define BLE_SVC_PROX_CONN_HANDLE_NONE 0xffff

/* Characteristic value handles */
static uint16_t ble_svc_prox_link_loss_val_handle;
static uint16_t ble_svc_prox_immediate_alert_loc_val_handle;
static uint16_t ble_svc_prox_tx_pwr_lvl_val_handle;

static struct {
    uint16_t conn_handle;
    uint8_t link_loss_alert;
    bool immediate_alert;
} ble_svc_prox_conn[MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 1];

static int
ble_svc_prox_link_loss_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg);
static int
ble_svc_prox_imm_alert_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg);
static int
ble_svc_prox_tx_pwr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg);

static int
ble_svc_prox_chr_write(struct os_mbuf *om, uint16_t min_len,
                       uint16_t max_len, void *dst,
                       uint16_t *len);

static int
ble_svc_prox_conn_slot(uint16_t conn_handle, int allocate)
{
    int free_slot;
    int i;

    free_slot = -1;
    for (i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
        if (ble_svc_prox_conn[i].conn_handle == conn_handle) {
            return i;
        }
        if (free_slot < 0 &&
            ble_svc_prox_conn[i].conn_handle == BLE_SVC_PROX_CONN_HANDLE_NONE) {
            free_slot = i;
        }
    }

    if (allocate && free_slot >= 0) {
        ble_svc_prox_conn[free_slot].conn_handle = conn_handle;
        ble_svc_prox_conn[free_slot].link_loss_alert = BLE_SVC_PROX_ALERT_NONE;
        ble_svc_prox_conn[free_slot].immediate_alert = false;
    }

    return free_slot;
}

static const struct ble_gatt_svc_def ble_svc_prox_defs[] = {
    {
        /*** Link Loss Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_LINK_LOSS_UUID16),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /** Alert level characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROX_CHR_UUID16_ALERT_LVL),
                .access_cb = ble_svc_prox_link_loss_access,
                .val_handle = &ble_svc_prox_link_loss_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
           }, {
               0, /* No more characteristics in this service. */
           }
        },
    },
    {
        /*** Immediate Alert Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_IMMEDIATE_ALERT_UUID16),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /** Alert level characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROX_CHR_UUID16_ALERT_LVL),
                .access_cb = ble_svc_prox_imm_alert_access,
                .val_handle = &ble_svc_prox_immediate_alert_loc_val_handle,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
           }, {
               0, /* No more characteristics in this service. */
           }
        },
    },
    {
        /*** TX Power Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_TX_POWER_UUID16),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /** TX Power Level Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROX_CHR_UUID16_TX_PWR_LVL),
                .access_cb = ble_svc_prox_tx_pwr_access,
                .val_handle = &ble_svc_prox_tx_pwr_lvl_val_handle,
                .flags = BLE_GATT_CHR_F_READ,
                .descriptors = (struct ble_gatt_dsc_def[])
                {
                    {
                        /** Presentation Format Descriptor */
                        .uuid = BLE_UUID16_DECLARE(BLE_SVC_PROX_DSC_UUID16_PRSNTN_FORMAT),
                        .att_flags = BLE_ATT_F_READ,
                        .access_cb = ble_svc_prox_tx_pwr_access,
                    }, {
                        0,
                    }
                },
           }, {
               0, /* No more characteristics in this service. */
           }
        },
    },
    {
        0, /* No more services. */
    },
};

static void
ble_prox_prph_task(void *pvParameters)
{
    while (1) {
        for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
            if (ble_svc_prox_conn[i].immediate_alert) {
                MODLOG_DFLT(INFO, "Immediate alert active for device connected with conn_handle %d",
                             ble_svc_prox_conn[i].conn_handle);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void
ble_prox_prph_alert_unalert(uint16_t conn_handle)
{
    int slot;

    slot = ble_svc_prox_conn_slot(conn_handle, 1);
    if (slot < 0) {
        MODLOG_DFLT(ERROR, "No proximity state slot for conn_handle %d", conn_handle);
        return;
    }

    if (ble_svc_prox_alert == BLE_SVC_PROX_ALERT_NONE) {
        if (ble_svc_prox_conn[slot].immediate_alert) {
            MODLOG_DFLT(INFO, "Stopping alert for device with conn_handle %d", conn_handle);
        }
        ble_svc_prox_conn[slot].immediate_alert = false;
    } else {
        if (!ble_svc_prox_conn[slot].immediate_alert) {
            MODLOG_DFLT(INFO, "Starting alert level %d for device with conn_handle %d",
                        ble_svc_prox_alert, conn_handle);
        }
        ble_svc_prox_conn[slot].immediate_alert = true;
    }
}

/**
 * Access function
 */
static int
ble_svc_prox_link_loss_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg)
{
    uint16_t uuid16;
    int rc;

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    switch (uuid16) {
    case BLE_SVC_PROX_CHR_UUID16_ALERT_LVL:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint8_t alert_lvl;
            int slot = ble_svc_prox_conn_slot(conn_handle, 1);
            if (slot < 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            rc = ble_svc_prox_chr_write(ctxt->om, 1,
                                        sizeof(alert_lvl),
                                        &alert_lvl, NULL);
            if (rc != 0) {
                return rc;
            }
            if (alert_lvl > BLE_SVC_PROX_ALERT_HIGH) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            ble_svc_prox_conn[slot].link_loss_alert = alert_lvl;
            return 0;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            int slot = ble_svc_prox_conn_slot(conn_handle, 1);
            if (slot < 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            rc = os_mbuf_append(ctxt->om, &ble_svc_prox_conn[slot].link_loss_alert,
                                sizeof(ble_svc_prox_conn[slot].link_loss_alert));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return BLE_ATT_ERR_UNLIKELY;

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int
ble_svc_prox_imm_alert_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt,
                              void *arg)
{
    uint16_t uuid16;
    int rc;

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    switch (uuid16) {
    case BLE_SVC_PROX_CHR_UUID16_ALERT_LVL:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            uint8_t alert_lvl;
            rc = ble_svc_prox_chr_write(ctxt->om, 1, 1, &alert_lvl, NULL);
            if (rc != 0) {
                return rc;
            }
            if (alert_lvl > BLE_SVC_PROX_ALERT_HIGH) {
                return BLE_ATT_ERR_UNLIKELY;
            }
            ble_svc_prox_alert = alert_lvl;

            MODLOG_DFLT(INFO, "Alert level = %d", ble_svc_prox_alert);

            ble_prox_prph_alert_unalert(conn_handle);
            return 0;
        }
        return BLE_ATT_ERR_INSUFFICIENT_RES;

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static int
ble_svc_prox_tx_pwr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg)
{
    uint16_t uuid16;
    int rc;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        uuid16 = ble_uuid_u16(ctxt->dsc->uuid);
    } else {
        uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    }
    assert(uuid16 != 0);

    switch (uuid16) {
    case BLE_SVC_PROX_CHR_UUID16_TX_PWR_LVL:
        if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        rc = os_mbuf_append(ctxt->om, &ble_svc_prox_tx_pwr_lvl,
                            sizeof(ble_svc_prox_tx_pwr_lvl));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case 0x2904: /* Presentation Format Descriptor UUID */
    {
        static const uint8_t tx_pwr_presentation_format[7] = {
            0x0c, 0x00, 0x28, 0x27, 0x01, 0x00, 0x00
        };

        if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        rc = os_mbuf_append(ctxt->om, tx_pwr_presentation_format,
                            sizeof(tx_pwr_presentation_format));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * Writes the received value from a characteristic write to
 * the given destination.
 */
static int
ble_svc_prox_chr_write(struct os_mbuf *om, uint16_t min_len,
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

void
ble_svc_prox_on_disconnect(uint16_t conn_handle)
{
    int slot;

    slot = ble_svc_prox_conn_slot(conn_handle, 0);
    if (slot < 0) {
        return;
    }

    if (ble_svc_prox_conn[slot].link_loss_alert != BLE_SVC_PROX_ALERT_NONE) {
        MODLOG_DFLT(INFO, "Link loss alert level %d for device with conn_handle %d",
                    ble_svc_prox_conn[slot].link_loss_alert, conn_handle);
    }

    ble_svc_prox_conn[slot].conn_handle = BLE_SVC_PROX_CONN_HANDLE_NONE;
    ble_svc_prox_conn[slot].link_loss_alert = BLE_SVC_PROX_ALERT_NONE;
    ble_svc_prox_conn[slot].immediate_alert = false;
}

void
ble_svc_prox_set_tx_power_level(int8_t tx_pwr_lvl)
{
    ble_svc_prox_tx_pwr_lvl = tx_pwr_lvl;
}

void
ble_svc_prox_init(void)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    rc = ble_gatts_count_cfg(ble_svc_prox_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_prox_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    /* Initializing alert array */
    for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
        ble_svc_prox_conn[i].conn_handle = BLE_SVC_PROX_CONN_HANDLE_NONE;
        ble_svc_prox_conn[i].link_loss_alert = BLE_SVC_PROX_ALERT_NONE;
        ble_svc_prox_conn[i].immediate_alert = false;
    }

    static TaskHandle_t ble_prox_task_handle;
    if (ble_prox_task_handle == NULL) {
        BaseType_t ret = xTaskCreate(ble_prox_prph_task, "ble_prox_prph_task",
                                     4096, NULL, 10, &ble_prox_task_handle);
        SYSINIT_PANIC_ASSERT(ret == pdPASS);
    }
}
#endif
