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
#include "services/htp/ble_svc_htp.h"
#include "host/ble_hs_log.h"

#if MYNEWT_VAL(BLE_GATTS) && CONFIG_BT_NIMBLE_HTP_SERVICE
/* Characteristic values */
static uint8_t ble_svc_htp_temp_type;
static uint16_t ble_svc_htp_temp_msr_itvl;

/* Health thermometer characteristic value handles */
static uint16_t ble_svc_htp_temp_measurement_val_handle;
static uint16_t ble_svc_htp_temp_type_val_handle;
static uint16_t ble_svc_htp_intr_temp_val_handle;
static uint16_t ble_svc_htp_msr_itvl_val_handle;

static struct {
    uint16_t conn_handle;
    struct chr_subscribe chr_subs;
} conn_chr_subs[MYNEWT_VAL(BLE_MAX_CONNECTIONS) + 1];

#define BLE_SVC_HTP_TEMP_MSR_MIN_ITVL 0x0001
#define BLE_SVC_HTP_TEMP_MSR_MAX_ITVL 0xffff
#define BLE_SVC_HTP_CONN_HANDLE_NONE 0xffff

static int
ble_svc_htp_conn_slot(uint16_t conn_handle)
{
    int free_slot;
    int i;

    free_slot = -1;
    for (i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
        if (conn_chr_subs[i].conn_handle == conn_handle) {
            return i;
        }
        if (free_slot < 0 &&
            conn_chr_subs[i].conn_handle == BLE_SVC_HTP_CONN_HANDLE_NONE) {
            free_slot = i;
        }
    }

    if (free_slot >= 0) {
        conn_chr_subs[free_slot].conn_handle = conn_handle;
        memset(&conn_chr_subs[free_slot].chr_subs, 0,
               sizeof(conn_chr_subs[free_slot].chr_subs));
    }

    return free_slot;
}

static uint32_t
ble_svc_htp_temp_to_ieee11073(float temp)
{
    int32_t mantissa;
    uint8_t exponent;

    exponent = (uint8_t)-2;
    if (temp >= 0) {
        mantissa = (int32_t)(temp * 100.0f + 0.5f);
    } else {
        mantissa = (int32_t)(temp * 100.0f - 0.5f);
    }

    if (mantissa > 0x7fffff) {
        mantissa = 0x7fffff;
    } else if (mantissa < -0x800000) {
        mantissa = -0x800000;
    }

    return ((uint32_t)exponent << 24) | ((uint32_t)mantissa & 0x00ffffff);
}

static int
ble_svc_htp_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt,
                   void *arg);
int ble_svc_htp_notify_measurement(void);
static int
ble_svc_htp_chr_write(struct os_mbuf *om, uint16_t min_len,
                      uint16_t max_len, void *dst,
                      uint16_t *len);

static const struct ble_gatt_svc_def ble_svc_htp_defs[] = {
    {
        /*** Health Thermometer Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_HTP_UUID16),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /** Temperature Measurement Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_HTP_CHR_UUID16_TEMP_MEASUREMENT),
                .access_cb = ble_svc_htp_access,
                .val_handle = &ble_svc_htp_temp_measurement_val_handle,
                .flags = BLE_GATT_CHR_F_INDICATE,
            }, {
                /** Temperature Type Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_HTP_CHR_UUID16_TEMP_TYPE),
                .access_cb = ble_svc_htp_access,
                .val_handle = &ble_svc_htp_temp_type_val_handle,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                /** Intermediate Temperature Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_HTP_CHR_UUID16_INTERMEDIATE_TEMP),
                .access_cb = ble_svc_htp_access,
                .val_handle = &ble_svc_htp_intr_temp_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            }, {
                /** Temperature Measurement Interval Characteristic */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_HTP_CHR_UUID16_MEASUREMENT_ITVL),
                .access_cb = ble_svc_htp_access,
                .val_handle = &ble_svc_htp_msr_itvl_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_INDICATE,
                .descriptors = (struct ble_gatt_dsc_def[])
                {
                    {
                        .uuid = BLE_UUID16_DECLARE(BLE_SVC_HTP_DSC_UUID16_VALID_RANGE),
                        .att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE,
                        .access_cb = ble_svc_htp_access,
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

/**
 * HTP access function
 */
static int
ble_svc_htp_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt,
                   void *arg)
{
    uint16_t uuid16;
    int rc;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC ||
        ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
        uuid16 = ble_uuid_u16(ctxt->dsc->uuid);
    } else {
        uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    }
    assert(uuid16 != 0);

    switch (uuid16) {
    case BLE_SVC_HTP_CHR_UUID16_TEMP_MEASUREMENT:
        return BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_SVC_HTP_CHR_UUID16_TEMP_TYPE:
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
        rc = os_mbuf_append(ctxt->om, &ble_svc_htp_temp_type,
                            sizeof(ble_svc_htp_temp_type));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_SVC_HTP_CHR_UUID16_INTERMEDIATE_TEMP:
        return BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_SVC_HTP_CHR_UUID16_MEASUREMENT_ITVL:
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            rc = ble_svc_htp_chr_write(ctxt->om, sizeof(ble_svc_htp_temp_msr_itvl),
                                       sizeof(ble_svc_htp_temp_msr_itvl),
                                       &ble_svc_htp_temp_msr_itvl,
                                       NULL);
            return rc;
        } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            rc = os_mbuf_append(ctxt->om, &ble_svc_htp_temp_msr_itvl,
                                sizeof(ble_svc_htp_temp_msr_itvl));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        } else {
            return BLE_SVC_HS_ERR_OUT_OF_RANGE;
        }

    case BLE_SVC_HTP_DSC_UUID16_VALID_RANGE: {
        uint8_t valid_range[4];

        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }

        valid_range[0] = BLE_SVC_HTP_TEMP_MSR_MIN_ITVL & 0xff;
        valid_range[1] = BLE_SVC_HTP_TEMP_MSR_MIN_ITVL >> 8;
        valid_range[2] = BLE_SVC_HTP_TEMP_MSR_MAX_ITVL & 0xff;
        valid_range[3] = BLE_SVC_HTP_TEMP_MSR_MAX_ITVL >> 8;

        rc = os_mbuf_append(ctxt->om, valid_range, sizeof(valid_range));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

void
ble_svc_htp_on_disconnect(uint16_t conn_handle)
{
    int slot = ble_svc_htp_conn_slot(conn_handle);

    if (slot < 0) {
        return;
    }

    conn_chr_subs[slot].conn_handle = BLE_SVC_HTP_CONN_HANDLE_NONE;
    memset(&conn_chr_subs[slot].chr_subs, 0,
           sizeof(conn_chr_subs[slot].chr_subs));
}

/**
 * Returns if the characteristic is subscribed or not
 */
bool
ble_svc_htp_is_subscribed(uint16_t conn_handle, int chr)
{
    int slot = ble_svc_htp_conn_slot(conn_handle);

    if (chr < TEMP_MEASUREMENT || chr > MEASUREMENT_ITVL || slot < 0) {
        return false;
    }
    return conn_chr_subs[slot].chr_subs.chr_subs[chr];
}

/**
 * Stores the subscribed state of each characteristic
 *
 * @params
 * attr_handle:     Attribute handle of the characteristic
 *
 * @return 0 on success, non-zero error code otherwise.
 */
void
ble_svc_htp_subscribe_state(uint16_t conn_handle, uint16_t attr_handle,
                            bool subscribed)
{
    int slot = ble_svc_htp_conn_slot(conn_handle);

    if (slot < 0) {
        return;
    }

    if (attr_handle == ble_svc_htp_temp_measurement_val_handle) {
        conn_chr_subs[slot].chr_subs.chr_subs[TEMP_MEASUREMENT] = subscribed;

    } else if (attr_handle == ble_svc_htp_intr_temp_val_handle) {
        conn_chr_subs[slot].chr_subs.chr_subs[INTERMEDIATE_TEMP] = subscribed;

    } else if (attr_handle == ble_svc_htp_msr_itvl_val_handle) {
        conn_chr_subs[slot].chr_subs.chr_subs[MEASUREMENT_ITVL] = subscribed;
    }
}

void
ble_svc_htp_subscribe(uint16_t conn_handle, uint16_t attr_handle)
{
    ble_svc_htp_subscribe_state(conn_handle, attr_handle, true);
}

/**
 * Send a notification for intermediate temperature
 *
 * @return 0 on success, non-zero error code otherwise.
 */
int
ble_svc_htp_notify(uint16_t conn_handle, float temp, bool temp_unit)
{
    int rc;
    struct os_mbuf *txom = NULL;

    /* 0th byte is flag, next 4 bytes is the temperature */
    uint8_t measurement[5];
    uint32_t temp_ieee11073;

    measurement[0] = 0x00;
    if (temp_unit) {
        measurement[0] |= 1 << 0; /* Temperature unit is Fahrenheit. */
    }

    temp_ieee11073 = ble_svc_htp_temp_to_ieee11073(temp);
    measurement[1] = temp_ieee11073 & 0xff;
    measurement[2] = (temp_ieee11073 >> 8) & 0xff;
    measurement[3] = (temp_ieee11073 >> 16) & 0xff;
    measurement[4] = (temp_ieee11073 >> 24) & 0xff;

    txom = ble_hs_mbuf_from_flat(measurement, sizeof(measurement));
    if (!txom) {
        return ESP_FAIL;
    }

    rc = ble_gatts_notify_custom(conn_handle,
                                 ble_svc_htp_intr_temp_val_handle, txom);
    return rc;
}

/**
 * Send a indicate for temperature measurement
 *
 * @return 0 on success, non-zero error code otherwise.
 */
int
ble_svc_htp_indicate(uint16_t conn_handle, float temp, bool temp_unit)
{
    int rc;
    struct os_mbuf *txom = NULL;

    /* 0th byte is flag, next 4 bytes is the temperature */

    uint8_t measurement[5];
    uint32_t temp_ieee11073;

    measurement[0] = 0x00;
    if (temp_unit) {
        measurement[0] |= 1 << 0;   /* Temperature unit is Fahrenheit. */
    }

    temp_ieee11073 = ble_svc_htp_temp_to_ieee11073(temp);
    measurement[1] = temp_ieee11073 & 0xff;
    measurement[2] = (temp_ieee11073 >> 8) & 0xff;
    measurement[3] = (temp_ieee11073 >> 16) & 0xff;
    measurement[4] = (temp_ieee11073 >> 24) & 0xff;

    txom = ble_hs_mbuf_from_flat(measurement, sizeof(measurement));
    if (!txom) {
        return ESP_FAIL;
    }

    rc = ble_gatts_indicate_custom(conn_handle,
                                   ble_svc_htp_temp_measurement_val_handle, txom);
    return rc;
}


/**
 * Writes the received value from a characteristic write to
 * the given destination.
 */
static int
ble_svc_htp_chr_write(struct os_mbuf *om, uint16_t min_len,
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
ble_svc_htp_init(void)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    rc = ble_gatts_count_cfg(ble_svc_htp_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_htp_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    ble_svc_htp_temp_type = 2;
    ble_svc_htp_temp_msr_itvl = 2; /* 2 sec */

    for (int i = 0; i <= MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
        conn_chr_subs[i].conn_handle = BLE_SVC_HTP_CONN_HANDLE_NONE;
        memset(&conn_chr_subs[i].chr_subs, 0, sizeof(conn_chr_subs[i].chr_subs));
    }
}
#endif
