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
#include "host/ble_hs_log.h"
#include "services/prox/ble_svc_prox.h"
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#include "esp_nimble_mem.h"
#endif

#if MYNEWT_VAL(BLE_GATTS) && CONFIG_BT_NIMBLE_PROX_SERVICE

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)

typedef struct {
    uint8_t _ble_svc_prox_link_loss_alert;
    int8_t _ble_svc_prox_alert;
    uint8_t _ble_svc_prox_tx_pwr_lvl;
    bool _ble_svc_prox_alert_conn[MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 1];
    TaskHandle_t _ble_prox_task_handle;
} ble_svc_prox_ctx_t;

static ble_svc_prox_ctx_t * ble_svc_prox_ctx = NULL;

#define ble_svc_prox_link_loss_alert (ble_svc_prox_ctx->_ble_svc_prox_link_loss_alert)
#define ble_svc_prox_alert (ble_svc_prox_ctx->_ble_svc_prox_alert)
#define ble_svc_prox_tx_pwr_lvl (ble_svc_prox_ctx->_ble_svc_prox_tx_pwr_lvl)
#define ble_svc_prox_alert_conn (ble_svc_prox_ctx->_ble_svc_prox_alert_conn)
#define ble_prox_task_handle (ble_svc_prox_ctx->_ble_prox_task_handle)

#else /* MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) */

/* Characteristic values */
static uint8_t ble_svc_prox_link_loss_alert;
static int8_t ble_svc_prox_alert;
static uint8_t ble_svc_prox_tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

static bool ble_svc_prox_alert_conn[MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 1];

static TaskHandle_t ble_prox_task_handle;
#endif /* MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) */

/* Characteristic value handles */
static uint16_t ble_svc_prox_link_loss_val_handle;
static uint16_t ble_svc_prox_immediate_alert_loc_val_handle;
static uint16_t ble_svc_prox_tx_pwr_lvl_val_handle;

#define BLE_SVC_PROX_HIGH_THRESHOLD  (-70)
#define BLE_SVC_PROX_LOW_THRESHOLD   (-100)

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
int
ble_svc_prox_ensure_ctx_init()
{
    if (ble_svc_prox_ctx == NULL) {
        ble_svc_prox_ctx = nimble_platform_mem_calloc(1, sizeof(ble_svc_prox_ctx_t));
        if (ble_svc_prox_ctx == NULL) {
            return BLE_HS_ENOMEM;
        }
        ble_svc_prox_ctx->_ble_svc_prox_tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    }
    return 0;
}

/* This API is provided to free the memory used by the service in the heap
 * after the service is no longer required. However, the task ble_prox_prph_task
 * runs indefinately. Is it only recommended to use this API after that task has
 * been deinited.
 */
void
ble_svc_prox_ctx_deinit()
{
    if (ble_svc_prox_ctx == NULL) {
        return;
    }

    if (ble_prox_task_handle) {
        vTaskDelete(ble_prox_task_handle);
        ble_prox_task_handle = NULL;
    }

    if (ble_svc_prox_ctx) {
        nimble_platform_mem_free(ble_svc_prox_ctx);
        ble_svc_prox_ctx = NULL;
    }
}
#endif

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
            if (ble_svc_prox_alert_conn[i]) {
                MODLOG_DFLT(INFO, "Path loss increased for device connected with conn_handle %d", i);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void
ble_prox_prph_alert_unalert(uint16_t conn_handle)
{
    if (conn_handle > MYNEWT_VAL(BLE_MAX_CONNECTIONS)) {
        MODLOG_DFLT(ERROR, "conn_handle %d exceeds max connections", conn_handle);
        return;
    }
    if (ble_svc_prox_alert > BLE_SVC_PROX_HIGH_THRESHOLD &&
        !ble_svc_prox_alert_conn[conn_handle]) {
        MODLOG_DFLT(INFO, "Path loss exceeded threshold, starting alert for device with "
                    "conn_handle %d", conn_handle);
        ble_svc_prox_alert_conn[conn_handle] = true;
    } else if (ble_svc_prox_alert < BLE_SVC_PROX_LOW_THRESHOLD &&
               ble_svc_prox_alert_conn[conn_handle]) {
        MODLOG_DFLT(INFO, "Path loss lower than threshold, stopping alert for device with "
                    "conn_handle %d", conn_handle);
        ble_svc_prox_alert_conn[conn_handle] = false;
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
            rc = ble_svc_prox_chr_write(ctxt->om, 1, sizeof(ble_svc_prox_link_loss_alert),
                                        &ble_svc_prox_link_loss_alert, NULL);
            return rc;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            rc = os_mbuf_append(ctxt->om, &ble_svc_prox_link_loss_alert,
                                sizeof(ble_svc_prox_link_loss_alert));
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

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    assert(uuid16 != 0);

    switch (uuid16) {
    case BLE_SVC_PROX_CHR_UUID16_ALERT_LVL:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            int rc = ble_svc_prox_chr_write(ctxt->om, 1, 1, &ble_svc_prox_alert, NULL);
            if (rc != 0) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            MODLOG_DFLT(INFO, "Path loss = %d", ble_svc_prox_alert);

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

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);
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
        if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        /* Return empty for now */
        return 0;

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
ble_svc_prox_init(void)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if(ble_svc_prox_ensure_ctx_init()) {
        SYSINIT_PANIC_ASSERT(0);
        return;
    }
#endif

    rc = ble_gatts_count_cfg(ble_svc_prox_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_prox_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    /* Initializing alert array */
    for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
        ble_svc_prox_alert_conn[i] = false;
    }

    BaseType_t ret = xTaskCreate(ble_prox_prph_task, "ble_prox_prph_task", 4096, NULL, 10, &ble_prox_task_handle);
    SYSINIT_PANIC_ASSERT(ret == pdPASS);
}
#endif
