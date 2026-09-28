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
#include <stdbool.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "syscfg/syscfg.h"
#include "os/endian.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/bas/ble_svc_bas.h"
#include "host/ble_hs_log.h"
#include "nimble/nimble_port.h"
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#include "esp_nimble_mem.h"
#endif

#if MYNEWT_VAL(BLE_GATTS) && CONFIG_BT_NIMBLE_BAS_SERVICE

#define BLE_SVC_BAS_LVL_STATUS   MYNEWT_VAL(BLE_SVC_BAS_BATTERY_LEVEL_STATUS_ENABLE)
#define BLE_SVC_BAS_EST_DATE     MYNEWT_VAL(BLE_SVC_BAS_EST_SERVICE_DATE_ENABLE)
#define BLE_SVC_BAS_CRIT_STATUS  MYNEWT_VAL(BLE_SVC_BAS_CRITICAL_STATUS_ENABLE)
#define BLE_SVC_BAS_ENERGY       MYNEWT_VAL(BLE_SVC_BAS_ENERGY_STATUS_ENABLE)
#define BLE_SVC_BAS_TIME         MYNEWT_VAL(BLE_SVC_BAS_TIME_STATUS_ENABLE)
#define BLE_SVC_BAS_HEALTH       MYNEWT_VAL(BLE_SVC_BAS_HEALTH_STATUS_ENABLE)
#define BLE_SVC_BAS_HEALTH_INFO  MYNEWT_VAL(BLE_SVC_BAS_HEALTH_INFO_ENABLE)
#define BLE_SVC_BAS_INFO         MYNEWT_VAL(BLE_SVC_BAS_BATTERY_INFO_ENABLE)
#define BLE_SVC_BAS_MFR_NAME     MYNEWT_VAL(BLE_SVC_BAS_MANUFACTURER_NAME_ENABLE)
#define BLE_SVC_BAS_MODEL_NUM    MYNEWT_VAL(BLE_SVC_BAS_MODEL_NUMBER_ENABLE)
#define BLE_SVC_BAS_SERIAL_NUM   MYNEWT_VAL(BLE_SVC_BAS_SERIAL_NUMBER_ENABLE)
#define BLE_SVC_BAS_ANY_STR      (BLE_SVC_BAS_MFR_NAME || BLE_SVC_BAS_MODEL_NUM || \
                                  BLE_SVC_BAS_SERIAL_NUM)

#if (BLE_SVC_BAS_EST_DATE || BLE_SVC_BAS_CRIT_STATUS || BLE_SVC_BAS_ENERGY || \
     BLE_SVC_BAS_TIME || BLE_SVC_BAS_HEALTH || BLE_SVC_BAS_INFO || \
     BLE_SVC_BAS_SERIAL_NUM) && !BLE_SVC_BAS_LVL_STATUS
#error "BAS: characteristic requires BLE_SVC_BAS_BATTERY_LEVEL_STATUS_ENABLE (BAS v1.1 C.2)"
#endif

#if BLE_SVC_BAS_HEALTH_INFO && !BLE_SVC_BAS_HEALTH
#error "BAS: Battery Health Information requires BLE_SVC_BAS_HEALTH_STATUS_ENABLE (BAS v1.1 C.3)"
#endif

#if BLE_SVC_BAS_ANY_STR
#define BLE_SVC_BAS_STR_LEN      MYNEWT_VAL(BLE_SVC_BAS_STR_MAX_LEN)
#endif

#define BLE_SVC_BAS_ONE_DAY_MS   (24UL * 60UL * 60UL * 1000UL)

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)

typedef struct {
    uint8_t _battery_level;
    bool _initialized;
    bool _counted;
#if BLE_SVC_BAS_LVL_STATUS
    struct ble_svc_bas_level_status _level_status;
#endif
#if BLE_SVC_BAS_EST_DATE
    uint32_t _est_service_date;
    uint32_t _est_service_date_tx_value;
    ble_npl_time_t _est_service_date_tx_time;
    bool _est_service_date_tx_valid;
    bool _est_service_date_pending;
    bool _est_service_date_timer_ready;
    struct ble_npl_callout _est_service_date_timer;
#endif
#if BLE_SVC_BAS_CRIT_STATUS
    uint8_t _critical_status;
#endif
#if BLE_SVC_BAS_ENERGY
    struct ble_svc_bas_energy_status _energy_status;
    struct ble_svc_bas_energy_status _energy_status_tx;
#endif
#if BLE_SVC_BAS_TIME
    struct ble_svc_bas_time_status _time_status;
    struct ble_svc_bas_time_status _time_status_tx;
#endif
#if BLE_SVC_BAS_HEALTH
    struct ble_svc_bas_health_status _health_status;
#endif
#if BLE_SVC_BAS_HEALTH_INFO
    struct ble_svc_bas_health_info _health_info;
#endif
#if BLE_SVC_BAS_INFO
    struct ble_svc_bas_battery_info _battery_info;
#endif
#if BLE_SVC_BAS_MFR_NAME
    char _mfr_name[BLE_SVC_BAS_STR_LEN];
    uint8_t _mfr_name_len;
#endif
#if BLE_SVC_BAS_MODEL_NUM
    char _model_number[BLE_SVC_BAS_STR_LEN];
    uint8_t _model_number_len;
#endif
#if BLE_SVC_BAS_SERIAL_NUM
    char _serial_number[BLE_SVC_BAS_STR_LEN];
    uint8_t _serial_number_len;
#endif
} ble_svc_bas_ctx_t;

static ble_svc_bas_ctx_t *ble_svc_bas_ctx;

#define ble_svc_bas_battery_level            (ble_svc_bas_ctx->_battery_level)
#define ble_svc_bas_initialized              (ble_svc_bas_ctx->_initialized)
#define ble_svc_bas_counted                  (ble_svc_bas_ctx->_counted)
#define ble_svc_bas_level_status_val         (ble_svc_bas_ctx->_level_status)
#define ble_svc_bas_est_service_date         (ble_svc_bas_ctx->_est_service_date)
#define ble_svc_bas_est_service_date_tx_value (ble_svc_bas_ctx->_est_service_date_tx_value)
#define ble_svc_bas_est_service_date_tx_time (ble_svc_bas_ctx->_est_service_date_tx_time)
#define ble_svc_bas_est_service_date_tx_valid (ble_svc_bas_ctx->_est_service_date_tx_valid)
#define ble_svc_bas_est_service_date_pending (ble_svc_bas_ctx->_est_service_date_pending)
#define ble_svc_bas_est_service_date_timer_ready (ble_svc_bas_ctx->_est_service_date_timer_ready)
#define ble_svc_bas_est_service_date_timer   (ble_svc_bas_ctx->_est_service_date_timer)
#define ble_svc_bas_critical_status          (ble_svc_bas_ctx->_critical_status)
#define ble_svc_bas_energy_status_val        (ble_svc_bas_ctx->_energy_status)
#define ble_svc_bas_energy_status_tx         (ble_svc_bas_ctx->_energy_status_tx)
#define ble_svc_bas_time_status_val          (ble_svc_bas_ctx->_time_status)
#define ble_svc_bas_time_status_tx           (ble_svc_bas_ctx->_time_status_tx)
#define ble_svc_bas_health_status_val        (ble_svc_bas_ctx->_health_status)
#define ble_svc_bas_health_info_val          (ble_svc_bas_ctx->_health_info)
#define ble_svc_bas_battery_info_val         (ble_svc_bas_ctx->_battery_info)
#define ble_svc_bas_mfr_name                 (ble_svc_bas_ctx->_mfr_name)
#define ble_svc_bas_mfr_name_len             (ble_svc_bas_ctx->_mfr_name_len)
#define ble_svc_bas_model_number             (ble_svc_bas_ctx->_model_number)
#define ble_svc_bas_model_number_len         (ble_svc_bas_ctx->_model_number_len)
#define ble_svc_bas_serial_number            (ble_svc_bas_ctx->_serial_number)
#define ble_svc_bas_serial_number_len        (ble_svc_bas_ctx->_serial_number_len)

#else /* MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) */

/* Battery level */
static uint8_t ble_svc_bas_battery_level;
static bool ble_svc_bas_initialized;
static bool ble_svc_bas_counted;
#if BLE_SVC_BAS_LVL_STATUS
static struct ble_svc_bas_level_status ble_svc_bas_level_status_val;
#endif
#if BLE_SVC_BAS_EST_DATE
static uint32_t ble_svc_bas_est_service_date;
static uint32_t ble_svc_bas_est_service_date_tx_value;
static ble_npl_time_t ble_svc_bas_est_service_date_tx_time;
static bool ble_svc_bas_est_service_date_tx_valid;
static bool ble_svc_bas_est_service_date_pending;
static bool ble_svc_bas_est_service_date_timer_ready;
static struct ble_npl_callout ble_svc_bas_est_service_date_timer;
#endif
#if BLE_SVC_BAS_CRIT_STATUS
static uint8_t ble_svc_bas_critical_status;
#endif
#if BLE_SVC_BAS_ENERGY
static struct ble_svc_bas_energy_status ble_svc_bas_energy_status_val;
static struct ble_svc_bas_energy_status ble_svc_bas_energy_status_tx;
#endif
#if BLE_SVC_BAS_TIME
static struct ble_svc_bas_time_status ble_svc_bas_time_status_val;
static struct ble_svc_bas_time_status ble_svc_bas_time_status_tx;
#endif
#if BLE_SVC_BAS_HEALTH
static struct ble_svc_bas_health_status ble_svc_bas_health_status_val;
#endif
#if BLE_SVC_BAS_HEALTH_INFO
static struct ble_svc_bas_health_info ble_svc_bas_health_info_val;
#endif
#if BLE_SVC_BAS_INFO
static struct ble_svc_bas_battery_info ble_svc_bas_battery_info_val;
#endif
#if BLE_SVC_BAS_MFR_NAME
static char ble_svc_bas_mfr_name[BLE_SVC_BAS_STR_LEN];
static uint8_t ble_svc_bas_mfr_name_len;
#endif
#if BLE_SVC_BAS_MODEL_NUM
static char ble_svc_bas_model_number[BLE_SVC_BAS_STR_LEN];
static uint8_t ble_svc_bas_model_number_len;
#endif
#if BLE_SVC_BAS_SERIAL_NUM
static char ble_svc_bas_serial_number[BLE_SVC_BAS_STR_LEN];
static uint8_t ble_svc_bas_serial_number_len;
#endif

#endif /* MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) */

/* Characteristic value handles */
#if MYNEWT_VAL(BLE_SVC_BAS_BATTERY_LEVEL_NOTIFY_ENABLE) > 0
static uint16_t ble_svc_bas_battery_handle;
#endif
#if BLE_SVC_BAS_LVL_STATUS
static uint16_t ble_svc_bas_level_status_handle;
#endif
#if BLE_SVC_BAS_EST_DATE
static uint16_t ble_svc_bas_est_service_date_handle;
#endif
#if BLE_SVC_BAS_CRIT_STATUS
static uint16_t ble_svc_bas_critical_status_handle;
#endif
#if BLE_SVC_BAS_ENERGY
static uint16_t ble_svc_bas_energy_status_handle;
#endif
#if BLE_SVC_BAS_TIME
static uint16_t ble_svc_bas_time_status_handle;
#endif
#if BLE_SVC_BAS_HEALTH
static uint16_t ble_svc_bas_health_status_handle;
#endif
#if BLE_SVC_BAS_HEALTH_INFO
static uint16_t ble_svc_bas_health_info_handle;
#endif
#if BLE_SVC_BAS_INFO
static uint16_t ble_svc_bas_battery_info_handle;
#endif
#if BLE_SVC_BAS_MFR_NAME
static uint16_t ble_svc_bas_mfr_name_handle;
#endif
#if BLE_SVC_BAS_MODEL_NUM
static uint16_t ble_svc_bas_model_number_handle;
#endif
#if BLE_SVC_BAS_SERIAL_NUM
static uint16_t ble_svc_bas_serial_number_handle;
#endif

#if BLE_SVC_BAS_EST_DATE
static void
ble_svc_bas_est_service_date_timer_deinit(void)
{
    if (ble_svc_bas_est_service_date_timer_ready) {
        ble_npl_callout_stop(&ble_svc_bas_est_service_date_timer);
        ble_npl_callout_deinit(&ble_svc_bas_est_service_date_timer);
        ble_svc_bas_est_service_date_timer_ready = false;
    }
}
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
int
ble_svc_bas_ensure_ctx_init(void)
{
    if (ble_svc_bas_ctx == NULL) {
        ble_svc_bas_ctx = nimble_platform_mem_calloc(1, sizeof(ble_svc_bas_ctx_t));
        if (ble_svc_bas_ctx == NULL) {
            return BLE_HS_ENOMEM;
        }
    }
    return 0;
}

void
ble_svc_bas_ctx_deinit(void)
{
    if (ble_svc_bas_ctx) {
#if BLE_SVC_BAS_EST_DATE
        /* The callout lives in the context and must be released first */
        ble_svc_bas_est_service_date_timer_deinit();
#endif
        nimble_platform_mem_free(ble_svc_bas_ctx);
        ble_svc_bas_ctx = NULL;
    }
}

#define BLE_SVC_BAS_ENSURE_CTX(err)             \
    do {                                        \
        if (ble_svc_bas_ensure_ctx_init()) {    \
            return (err);                       \
        }                                       \
    } while (0)
#else
#define BLE_SVC_BAS_ENSURE_CTX(err)
#endif

/* Access function */
static int
ble_svc_bas_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg);

#if BLE_SVC_BAS_EST_DATE
static void ble_svc_bas_est_service_date_try_send(void);
#endif

static const ble_uuid16_t uuid_svc_bas = BLE_UUID16_INIT(BLE_SVC_BAS_UUID16);
static const ble_uuid16_t uuid_chr_bat_lvl = BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_BATTERY_LEVEL);
#if MYNEWT_VAL(BLE_SVC_BAS_PRES_FMT_ENABLE)
static const ble_uuid16_t uuid_dsc_pres_fmt = BLE_UUID16_INIT(BLE_SVC_BAS_DSC_UUID16_PRES_FORMAT);
#endif
#if BLE_SVC_BAS_LVL_STATUS
static const ble_uuid16_t uuid_chr_lvl_status =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_BATTERY_LEVEL_STATUS);
#endif
#if BLE_SVC_BAS_EST_DATE
static const ble_uuid16_t uuid_chr_est_date =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_ESTIMATED_SERVICE_DATE);
#endif
#if BLE_SVC_BAS_CRIT_STATUS
static const ble_uuid16_t uuid_chr_crit_status =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_BATTERY_CRITICAL_STATUS);
#endif
#if BLE_SVC_BAS_ENERGY
static const ble_uuid16_t uuid_chr_energy_status =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_BATTERY_ENERGY_STATUS);
#endif
#if BLE_SVC_BAS_TIME
static const ble_uuid16_t uuid_chr_time_status =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_BATTERY_TIME_STATUS);
#endif
#if BLE_SVC_BAS_HEALTH
static const ble_uuid16_t uuid_chr_health_status =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_BATTERY_HEALTH_STATUS);
#endif
#if BLE_SVC_BAS_HEALTH_INFO
static const ble_uuid16_t uuid_chr_health_info =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_BATTERY_HEALTH_INFO);
#endif
#if BLE_SVC_BAS_INFO
static const ble_uuid16_t uuid_chr_battery_info =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_BATTERY_INFO);
#endif
#if BLE_SVC_BAS_MFR_NAME
static const ble_uuid16_t uuid_chr_mfr_name =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_MANUFACTURER_NAME);
#endif
#if BLE_SVC_BAS_MODEL_NUM
static const ble_uuid16_t uuid_chr_model_number =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_MODEL_NUMBER);
#endif
#if BLE_SVC_BAS_SERIAL_NUM
static const ble_uuid16_t uuid_chr_serial_number =
    BLE_UUID16_INIT(BLE_SVC_BAS_CHR_UUID16_SERIAL_NUMBER);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_PRES_FMT_ENABLE)
static const struct ble_gatt_dsc_def bas_battery_level_descriptors[] = {
    {
        .uuid = &uuid_dsc_pres_fmt.u,
        .att_flags = BLE_ATT_F_READ,
        .access_cb = ble_svc_bas_access,
    },
    {
        0, /* No more descriptors in this characteristic. */
    },
};
#endif

static const struct ble_gatt_chr_def bas_characteristics[] = {
    {
        /*** Battery level characteristic */
            .uuid = &uuid_chr_bat_lvl.u,
            .access_cb = ble_svc_bas_access,
#if MYNEWT_VAL(BLE_SVC_BAS_BATTERY_LEVEL_NOTIFY_ENABLE) > 0
            .val_handle = &ble_svc_bas_battery_handle,
#endif
#if MYNEWT_VAL(BLE_SVC_BAS_PRES_FMT_ENABLE)
            .descriptors = (struct ble_gatt_dsc_def *)bas_battery_level_descriptors,
#endif
            .flags = BLE_GATT_CHR_F_READ |
#if MYNEWT_VAL(BLE_SVC_BAS_BATTERY_LEVEL_NOTIFY_ENABLE) > 0
                     BLE_GATT_CHR_F_NOTIFY |
#endif
                     MYNEWT_VAL(BLE_SVC_BAS_BATTERY_LEVEL_READ_PERM),
    },
#if BLE_SVC_BAS_LVL_STATUS
    {
        /*** Battery Level Status characteristic */
        .uuid = &uuid_chr_lvl_status.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_level_status_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY
#if MYNEWT_VAL(BLE_SVC_BAS_BATTERY_LEVEL_STATUS_INDICATE_ENABLE)
                 | BLE_GATT_CHR_F_INDICATE
#endif
                 ,
    },
#endif
#if BLE_SVC_BAS_EST_DATE
    {
        /*** Estimated Service Date characteristic */
        .uuid = &uuid_chr_est_date.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_est_service_date_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY
#if MYNEWT_VAL(BLE_SVC_BAS_EST_SERVICE_DATE_INDICATE_ENABLE)
                 | BLE_GATT_CHR_F_INDICATE
#endif
                 ,
    },
#endif
#if BLE_SVC_BAS_CRIT_STATUS
    {
        /*** Battery Critical Status characteristic */
        .uuid = &uuid_chr_crit_status.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_critical_status_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
    },
#endif
#if BLE_SVC_BAS_ENERGY
    {
        /*** Battery Energy Status characteristic */
        .uuid = &uuid_chr_energy_status.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_energy_status_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY
#if MYNEWT_VAL(BLE_SVC_BAS_ENERGY_STATUS_INDICATE_ENABLE)
                 | BLE_GATT_CHR_F_INDICATE
#endif
                 ,
    },
#endif
#if BLE_SVC_BAS_TIME
    {
        /*** Battery Time Status characteristic */
        .uuid = &uuid_chr_time_status.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_time_status_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY
#if MYNEWT_VAL(BLE_SVC_BAS_TIME_STATUS_INDICATE_ENABLE)
                 | BLE_GATT_CHR_F_INDICATE
#endif
                 ,
    },
#endif
#if BLE_SVC_BAS_HEALTH
    {
        /*** Battery Health Status characteristic */
        .uuid = &uuid_chr_health_status.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_health_status_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY
#if MYNEWT_VAL(BLE_SVC_BAS_HEALTH_STATUS_INDICATE_ENABLE)
                 | BLE_GATT_CHR_F_INDICATE
#endif
                 ,
    },
#endif
#if BLE_SVC_BAS_HEALTH_INFO
    {
        /*** Battery Health Information characteristic */
        .uuid = &uuid_chr_health_info.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_health_info_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
    },
#endif
#if BLE_SVC_BAS_INFO
    {
        /*** Battery Information characteristic */
        .uuid = &uuid_chr_battery_info.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_battery_info_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
    },
#endif
#if BLE_SVC_BAS_MFR_NAME
    {
        /*** Manufacturer Name String characteristic */
        .uuid = &uuid_chr_mfr_name.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_mfr_name_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
    },
#endif
#if BLE_SVC_BAS_MODEL_NUM
    {
        /*** Model Number String characteristic */
        .uuid = &uuid_chr_model_number.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_model_number_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
    },
#endif
#if BLE_SVC_BAS_SERIAL_NUM
    {
        /*** Serial Number String characteristic */
        .uuid = &uuid_chr_serial_number.u,
        .access_cb = ble_svc_bas_access,
        .val_handle = &ble_svc_bas_serial_number_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE |
                 MYNEWT_VAL(BLE_SVC_BAS_SERIAL_NUM_READ_PERM),
    },
#endif
    {
        0, /* No more characteristics in this service. */
    },
};

static const struct ble_gatt_svc_def ble_svc_bas_defs[] = {
    {
        /*** Battery Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_bas.u,
        .characteristics = bas_characteristics,
    }, {
        0, /* No more services. */
    },
};

#if BLE_SVC_BAS_LVL_STATUS || BLE_SVC_BAS_ANY_STR
static bool
ble_svc_bas_battery_present(void)
{
#if BLE_SVC_BAS_LVL_STATUS
    return (ble_svc_bas_level_status_val.power_state &
            BLE_SVC_BAS_PWR_BATTERY_PRESENT) != 0;
#else
    return true;
#endif
}
#endif

#if BLE_SVC_BAS_ENERGY || BLE_SVC_BAS_TIME
/**
 * Returns true if new differs from old by at least 1 percent of range.
 * A zero range means any change is significant.
 */
static bool
ble_svc_bas_changed_1pct(float old_val, float new_val, float range)
{
    float diff;

    diff = new_val - old_val;
    if (diff < 0) {
        diff = -diff;
    }
    if (range < 0) {
        range = -range;
    }
    if (range == 0) {
        return diff != 0;
    }
    return diff * 100.0f >= range;
}
#endif

#if BLE_SVC_BAS_ENERGY
/* Returns true for NaN, NRes, +/-INFINITY and the reserved medfloat16 value */
static bool
ble_svc_bas_sfloat_is_special(uint16_t raw)
{
    uint16_t mantissa = raw & 0x0fff;

    return (raw >> 12) == 0 && mantissa >= 0x07fe && mantissa <= 0x0802;
}

static float
ble_svc_bas_sfloat_to_float(uint16_t raw)
{
    int16_t mantissa;
    int8_t exponent;
    float val;

    mantissa = raw & 0x0fff;
    if (mantissa & 0x0800) {
        mantissa -= 0x1000;
    }
    exponent = (raw >> 12) & 0x0f;
    if (exponent & 0x08) {
        exponent -= 0x10;
    }

    val = mantissa;
    for (; exponent > 0; exponent--) {
        val *= 10.0f;
    }
    for (; exponent < 0; exponent++) {
        val /= 10.0f;
    }
    return val;
}

static bool
ble_svc_bas_energy_field_changed(uint8_t flag, uint16_t old_raw, uint16_t new_raw,
                                 uint16_t range_raw, bool range_valid)
{
    const struct ble_svc_bas_energy_status *tx = &ble_svc_bas_energy_status_tx;
    const struct ble_svc_bas_energy_status *cur = &ble_svc_bas_energy_status_val;
    float old_val;
    float range;

    if ((tx->flags & flag) != (cur->flags & flag)) {
        return true;
    }
    if (!(cur->flags & flag) || old_raw == new_raw) {
        return false;
    }
    if (ble_svc_bas_sfloat_is_special(old_raw) ||
        ble_svc_bas_sfloat_is_special(new_raw)) {
        return true;
    }

    old_val = ble_svc_bas_sfloat_to_float(old_raw);
    if (range_valid && !ble_svc_bas_sfloat_is_special(range_raw)) {
        range = ble_svc_bas_sfloat_to_float(range_raw);
    } else {
        range = old_val;
    }
    return ble_svc_bas_changed_1pct(old_val, ble_svc_bas_sfloat_to_float(new_raw),
                                    range);
}

static void
ble_svc_bas_energy_status_send(void)
{
    ble_svc_bas_energy_status_tx = ble_svc_bas_energy_status_val;
    ble_gatts_chr_updated(ble_svc_bas_energy_status_handle);
}
#endif

#if BLE_SVC_BAS_CRIT_STATUS
static void
ble_svc_bas_critical_status_update(void)
{
    const struct ble_svc_bas_level_status *st = &ble_svc_bas_level_status_val;
    uint8_t crit = 0;

    if (((st->power_state & BLE_SVC_BAS_PWR_CHARGE_LEVEL_MASK) >>
         BLE_SVC_BAS_PWR_CHARGE_LEVEL_POS) == BLE_SVC_BAS_CHARGE_LEVEL_CRITICAL) {
        crit |= BLE_SVC_BAS_CRIT_CRITICAL_POWER_STATE;
    }
    if ((st->flags & BLE_SVC_BAS_LVL_STATUS_F_ADDITIONAL_STATUS) &&
        (st->additional_status & BLE_SVC_BAS_ADD_STATUS_SERVICE_REQ_MASK) ==
        BLE_SVC_BAS_SERVICE_REQ_TRUE) {
        crit |= BLE_SVC_BAS_CRIT_IMMEDIATE_SERVICE_REQ;
    }

    if (crit != ble_svc_bas_critical_status) {
        ble_svc_bas_critical_status = crit;
        if (ble_svc_bas_battery_present()) {
            ble_gatts_chr_updated(ble_svc_bas_critical_status_handle);
        }
    }
}
#endif

#if BLE_SVC_BAS_LVL_STATUS
static int
ble_svc_bas_level_status_encode(struct os_mbuf *om)
{
    const struct ble_svc_bas_level_status *st = &ble_svc_bas_level_status_val;
    uint8_t buf[7];
    uint8_t flags;
    int len;

    flags = st->flags & (BLE_SVC_BAS_LVL_STATUS_F_IDENTIFIER |
                         BLE_SVC_BAS_LVL_STATUS_F_BATTERY_LEVEL |
                         BLE_SVC_BAS_LVL_STATUS_F_ADDITIONAL_STATUS);
    if (!ble_svc_bas_battery_present()) {
        flags &= ~BLE_SVC_BAS_LVL_STATUS_F_BATTERY_LEVEL;
    }

    buf[0] = flags;
    put_le16(&buf[1], st->power_state);
    len = 3;
    if (flags & BLE_SVC_BAS_LVL_STATUS_F_IDENTIFIER) {
        put_le16(&buf[len], MYNEWT_VAL(BLE_SVC_BAS_PRES_FMT_DESCRIPTION));
        len += 2;
    }
    if (flags & BLE_SVC_BAS_LVL_STATUS_F_BATTERY_LEVEL) {
        buf[len++] = ble_svc_bas_battery_level;
    }
    if (flags & BLE_SVC_BAS_LVL_STATUS_F_ADDITIONAL_STATUS) {
        buf[len++] = st->additional_status;
    }

    return os_mbuf_append(om, buf, len);
}
#endif

#if BLE_SVC_BAS_ENERGY
static int
ble_svc_bas_energy_status_encode(struct os_mbuf *om)
{
    const struct ble_svc_bas_energy_status *st = &ble_svc_bas_energy_status_val;
    uint8_t buf[13];
    uint8_t flags;
    uint16_t wired;
    uint16_t wireless;
    int len;

    flags = st->flags & (BLE_SVC_BAS_ENERGY_F_EXT_SOURCE_POWER |
                         BLE_SVC_BAS_ENERGY_F_PRESENT_VOLTAGE |
                         BLE_SVC_BAS_ENERGY_F_AVAILABLE_ENERGY |
                         BLE_SVC_BAS_ENERGY_F_AVAILABLE_CAPACITY |
                         BLE_SVC_BAS_ENERGY_F_CHARGE_RATE |
                         BLE_SVC_BAS_ENERGY_F_ENERGY_AT_LAST_CHARGE);
    if (!ble_svc_bas_battery_present()) {
        flags &= BLE_SVC_BAS_ENERGY_F_EXT_SOURCE_POWER;
    }

    /* External Source Power shall not be present without an external source */
    wired = (ble_svc_bas_level_status_val.power_state & BLE_SVC_BAS_PWR_WIRED_MASK) >>
            BLE_SVC_BAS_PWR_WIRED_POS;
    wireless = (ble_svc_bas_level_status_val.power_state & BLE_SVC_BAS_PWR_WIRELESS_MASK) >>
               BLE_SVC_BAS_PWR_WIRELESS_POS;
    if (wired == BLE_SVC_BAS_PWR_EXT_NO && wireless == BLE_SVC_BAS_PWR_EXT_NO) {
        flags &= ~BLE_SVC_BAS_ENERGY_F_EXT_SOURCE_POWER;
    }

    buf[0] = flags;
    len = 1;
    if (flags & BLE_SVC_BAS_ENERGY_F_EXT_SOURCE_POWER) {
        put_le16(&buf[len], st->ext_source_power);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_ENERGY_F_PRESENT_VOLTAGE) {
        put_le16(&buf[len], st->present_voltage);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_ENERGY_F_AVAILABLE_ENERGY) {
        put_le16(&buf[len], st->available_energy);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_ENERGY_F_AVAILABLE_CAPACITY) {
        put_le16(&buf[len], st->available_capacity);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_ENERGY_F_CHARGE_RATE) {
        put_le16(&buf[len], st->charge_rate);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_ENERGY_F_ENERGY_AT_LAST_CHARGE) {
        put_le16(&buf[len], st->energy_at_last_charge);
        len += 2;
    }

    return os_mbuf_append(om, buf, len);
}
#endif

#if BLE_SVC_BAS_TIME
static int
ble_svc_bas_time_status_encode(struct os_mbuf *om)
{
    const struct ble_svc_bas_time_status *st = &ble_svc_bas_time_status_val;
    uint8_t buf[10];
    uint8_t flags;
    int len;

    flags = st->flags & (BLE_SVC_BAS_TIME_F_DISCHARGED_ON_STANDBY |
                         BLE_SVC_BAS_TIME_F_RECHARGED);

    /* Time until Recharged shall not be present while not charging */
    if (((ble_svc_bas_level_status_val.power_state & BLE_SVC_BAS_PWR_CHARGE_STATE_MASK) >>
         BLE_SVC_BAS_PWR_CHARGE_STATE_POS) != BLE_SVC_BAS_CHARGE_STATE_CHARGING) {
        flags &= ~BLE_SVC_BAS_TIME_F_RECHARGED;
    }
#if BLE_SVC_BAS_INFO
    if (!(ble_svc_bas_battery_info_val.features & BLE_SVC_BAS_FEAT_RECHARGEABLE)) {
        flags &= ~BLE_SVC_BAS_TIME_F_RECHARGED;
    }
#endif

    buf[0] = flags;
    put_le24(&buf[1], st->time_until_discharged);
    len = 4;
    if (flags & BLE_SVC_BAS_TIME_F_DISCHARGED_ON_STANDBY) {
        put_le24(&buf[len], st->time_until_discharged_on_standby);
        len += 3;
    }
    if (flags & BLE_SVC_BAS_TIME_F_RECHARGED) {
        put_le24(&buf[len], st->time_until_recharged);
        len += 3;
    }

    return os_mbuf_append(om, buf, len);
}

static bool
ble_svc_bas_time_field_changed(uint8_t flag, uint32_t old_val, uint32_t new_val)
{
    bool old_present = flag == 0 || (ble_svc_bas_time_status_tx.flags & flag);
    bool new_present = flag == 0 || (ble_svc_bas_time_status_val.flags & flag);

    if (old_present != new_present) {
        return true;
    }
    if (!new_present || old_val == new_val) {
        return false;
    }
    if (old_val >= BLE_SVC_BAS_TIME_GREATER_THAN_MAX ||
        new_val >= BLE_SVC_BAS_TIME_GREATER_THAN_MAX) {
        return true;
    }
    return ble_svc_bas_changed_1pct((float)old_val, (float)new_val, (float)old_val);
}
#endif

#if BLE_SVC_BAS_HEALTH
static int
ble_svc_bas_health_status_encode(struct os_mbuf *om)
{
    const struct ble_svc_bas_health_status *st = &ble_svc_bas_health_status_val;
    uint8_t buf[7];
    uint8_t flags;
    int len;

    flags = st->flags & (BLE_SVC_BAS_HEALTH_F_SUMMARY |
                         BLE_SVC_BAS_HEALTH_F_CYCLE_COUNT |
                         BLE_SVC_BAS_HEALTH_F_CURRENT_TEMP |
                         BLE_SVC_BAS_HEALTH_F_DEEP_DISCHARGE_COUNT);

    buf[0] = flags;
    len = 1;
    if (flags & BLE_SVC_BAS_HEALTH_F_SUMMARY) {
        buf[len++] = st->health_summary;
    }
    if (flags & BLE_SVC_BAS_HEALTH_F_CYCLE_COUNT) {
        put_le16(&buf[len], st->cycle_count);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_HEALTH_F_CURRENT_TEMP) {
        buf[len++] = (uint8_t)st->current_temp;
    }
    if (flags & BLE_SVC_BAS_HEALTH_F_DEEP_DISCHARGE_COUNT) {
        put_le16(&buf[len], st->deep_discharge_count);
        len += 2;
    }

    return os_mbuf_append(om, buf, len);
}
#endif

#if BLE_SVC_BAS_HEALTH_INFO
static int
ble_svc_bas_health_info_encode(struct os_mbuf *om)
{
    const struct ble_svc_bas_health_info *info = &ble_svc_bas_health_info_val;
    uint8_t buf[5];
    uint8_t flags;
    int len;

    flags = info->flags & (BLE_SVC_BAS_HEALTH_INFO_F_CYCLE_COUNT_LIFETIME |
                           BLE_SVC_BAS_HEALTH_INFO_F_DESIGNED_OP_TEMP);

    buf[0] = flags;
    len = 1;
    if (flags & BLE_SVC_BAS_HEALTH_INFO_F_CYCLE_COUNT_LIFETIME) {
        put_le16(&buf[len], info->cycle_count_designed_lifetime);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_HEALTH_INFO_F_DESIGNED_OP_TEMP) {
        buf[len++] = (uint8_t)info->min_designed_op_temp;
        buf[len++] = (uint8_t)info->max_designed_op_temp;
    }

    return os_mbuf_append(om, buf, len);
}
#endif

#if BLE_SVC_BAS_INFO
static int
ble_svc_bas_battery_info_encode(struct os_mbuf *om)
{
    const struct ble_svc_bas_battery_info *info = &ble_svc_bas_battery_info_val;
    uint8_t buf[21];
    uint16_t flags;
    int len;

    flags = info->flags & (BLE_SVC_BAS_INFO_F_MANUFACTURE_DATE |
                           BLE_SVC_BAS_INFO_F_EXPIRATION_DATE |
                           BLE_SVC_BAS_INFO_F_DESIGNED_CAPACITY |
                           BLE_SVC_BAS_INFO_F_LOW_ENERGY |
                           BLE_SVC_BAS_INFO_F_CRITICAL_ENERGY |
                           BLE_SVC_BAS_INFO_F_CHEMISTRY |
                           BLE_SVC_BAS_INFO_F_NOMINAL_VOLTAGE |
                           BLE_SVC_BAS_INFO_F_AGGREGATION_GROUP);

    put_le16(&buf[0], flags);
    buf[2] = info->features;
    len = 3;
    if (flags & BLE_SVC_BAS_INFO_F_MANUFACTURE_DATE) {
        put_le24(&buf[len], info->manufacture_date);
        len += 3;
    }
    if (flags & BLE_SVC_BAS_INFO_F_EXPIRATION_DATE) {
        put_le24(&buf[len], info->expiration_date);
        len += 3;
    }
    if (flags & BLE_SVC_BAS_INFO_F_DESIGNED_CAPACITY) {
        put_le16(&buf[len], info->designed_capacity);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_INFO_F_LOW_ENERGY) {
        put_le16(&buf[len], info->low_energy);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_INFO_F_CRITICAL_ENERGY) {
        put_le16(&buf[len], info->critical_energy);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_INFO_F_CHEMISTRY) {
        buf[len++] = info->chemistry;
    }
    if (flags & BLE_SVC_BAS_INFO_F_NOMINAL_VOLTAGE) {
        put_le16(&buf[len], info->nominal_voltage);
        len += 2;
    }
    if (flags & BLE_SVC_BAS_INFO_F_AGGREGATION_GROUP) {
        buf[len++] = info->aggregation_group;
    }

    return os_mbuf_append(om, buf, len);
}
#endif

#if BLE_SVC_BAS_ANY_STR
static int
ble_svc_bas_str_set(char *dst, uint8_t *dst_len, const char *src, uint16_t handle)
{
    size_t len;

    len = src ? strlen(src) : 0;
    if (len > BLE_SVC_BAS_STR_LEN) {
        return BLE_HS_EINVAL;
    }

    if (len == *dst_len && (len == 0 || memcmp(dst, src, len) == 0)) {
        return 0;
    }

    if (len > 0) {
        memcpy(dst, src, len);
    }
    *dst_len = len;

    if (ble_svc_bas_battery_present()) {
        ble_gatts_chr_updated(handle);
    }
    return 0;
}
#endif

/**
 * BAS access function
 */
static int
ble_svc_bas_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg)
{
    uint16_t uuid16;
    int rc;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_svc_bas_ctx == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_PRES_FMT_ENABLE)
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        uint8_t pres_fmt[7];

        if (ble_uuid_u16(ctxt->dsc->uuid) != BLE_SVC_BAS_DSC_UUID16_PRES_FORMAT) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        pres_fmt[0] = BLE_GATT_CHR_FORMAT_UINT8;
        pres_fmt[1] = 0;                            /* Exponent */
        put_le16(&pres_fmt[2], BLE_GATT_CHR_UNIT_PERCENTAGE);
        pres_fmt[4] = BLE_GATT_CHR_NAMESPACE_BT_SIG;
        put_le16(&pres_fmt[5], MYNEWT_VAL(BLE_SVC_BAS_PRES_FMT_DESCRIPTION));

        rc = os_mbuf_append(ctxt->om, pres_fmt, sizeof(pres_fmt));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
#endif

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uuid16 = ble_uuid_u16(ctxt->chr->uuid);

    switch (uuid16) {
    case BLE_SVC_BAS_CHR_UUID16_BATTERY_LEVEL:
        rc = os_mbuf_append(ctxt->om, &ble_svc_bas_battery_level,
                            sizeof(ble_svc_bas_battery_level));
        break;

#if BLE_SVC_BAS_LVL_STATUS
    case BLE_SVC_BAS_CHR_UUID16_BATTERY_LEVEL_STATUS:
        rc = ble_svc_bas_level_status_encode(ctxt->om);
        break;
#endif

#if BLE_SVC_BAS_EST_DATE
    case BLE_SVC_BAS_CHR_UUID16_ESTIMATED_SERVICE_DATE: {
        uint8_t date[3];

        put_le24(date, ble_svc_bas_est_service_date);
        rc = os_mbuf_append(ctxt->om, date, sizeof(date));
        break;
    }
#endif

#if BLE_SVC_BAS_CRIT_STATUS
    case BLE_SVC_BAS_CHR_UUID16_BATTERY_CRITICAL_STATUS:
        rc = os_mbuf_append(ctxt->om, &ble_svc_bas_critical_status,
                            sizeof(ble_svc_bas_critical_status));
        break;
#endif

#if BLE_SVC_BAS_ENERGY
    case BLE_SVC_BAS_CHR_UUID16_BATTERY_ENERGY_STATUS:
        rc = ble_svc_bas_energy_status_encode(ctxt->om);
        break;
#endif

#if BLE_SVC_BAS_TIME
    case BLE_SVC_BAS_CHR_UUID16_BATTERY_TIME_STATUS:
        rc = ble_svc_bas_time_status_encode(ctxt->om);
        break;
#endif

#if BLE_SVC_BAS_HEALTH
    case BLE_SVC_BAS_CHR_UUID16_BATTERY_HEALTH_STATUS:
        rc = ble_svc_bas_health_status_encode(ctxt->om);
        break;
#endif

#if BLE_SVC_BAS_HEALTH_INFO
    case BLE_SVC_BAS_CHR_UUID16_BATTERY_HEALTH_INFO:
        rc = ble_svc_bas_health_info_encode(ctxt->om);
        break;
#endif

#if BLE_SVC_BAS_INFO
    case BLE_SVC_BAS_CHR_UUID16_BATTERY_INFO:
        rc = ble_svc_bas_battery_info_encode(ctxt->om);
        break;
#endif

#if BLE_SVC_BAS_MFR_NAME
    case BLE_SVC_BAS_CHR_UUID16_MANUFACTURER_NAME:
        /* Value shall be empty while the battery is not present */
        rc = os_mbuf_append(ctxt->om, ble_svc_bas_mfr_name,
                            ble_svc_bas_battery_present() ?
                            ble_svc_bas_mfr_name_len : 0);
        break;
#endif

#if BLE_SVC_BAS_MODEL_NUM
    case BLE_SVC_BAS_CHR_UUID16_MODEL_NUMBER:
        rc = os_mbuf_append(ctxt->om, ble_svc_bas_model_number,
                            ble_svc_bas_model_number_len);
        break;
#endif

#if BLE_SVC_BAS_SERIAL_NUM
    case BLE_SVC_BAS_CHR_UUID16_SERIAL_NUMBER:
        /* Value shall be empty while the battery is not present */
        rc = os_mbuf_append(ctxt->om, ble_svc_bas_serial_number,
                            ble_svc_bas_battery_present() ?
                            ble_svc_bas_serial_number_len : 0);
        break;
#endif

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }

    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}


/**
 * Get the current battery level.
 *
 * @return The current battery level (0-100).
 */
uint8_t
ble_svc_bas_battery_level_get(void)
{
    BLE_SVC_BAS_ENSURE_CTX(0);

    return ble_svc_bas_battery_level;
}

/**
 * Set the battery level, must be between 0 and 100.
 * If greater than 100, it will be silently set to 100.
 */
int
ble_svc_bas_battery_level_set(uint8_t level)
{
    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    if (level > 100)
        level = 100;
    if (ble_svc_bas_battery_level != level) {
        ble_svc_bas_battery_level = level;
#if MYNEWT_VAL(BLE_SVC_BAS_BATTERY_LEVEL_NOTIFY_ENABLE) > 0
        ble_gatts_chr_updated(ble_svc_bas_battery_handle);
#endif
#if BLE_SVC_BAS_LVL_STATUS
        if ((ble_svc_bas_level_status_val.flags & BLE_SVC_BAS_LVL_STATUS_F_BATTERY_LEVEL) &&
            ble_svc_bas_battery_present()) {
            ble_gatts_chr_updated(ble_svc_bas_level_status_handle);
        }
#endif
    }
    return 0;
}

#if BLE_SVC_BAS_LVL_STATUS
/**
 * Set the Battery Level Status value.
 *
 * Notifies/indicates when the Power State or the Service Required bits
 * change. Battery Energy Status and Battery Critical Status are updated
 * accordingly.
 */
int
ble_svc_bas_level_status_set(const struct ble_svc_bas_level_status *status)
{
    bool power_changed;
    bool service_changed;
#if BLE_SVC_BAS_EST_DATE
    bool was_present;
#endif
    uint8_t old_sr;
    uint8_t new_sr;

    if (status == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    old_sr = (ble_svc_bas_level_status_val.flags & BLE_SVC_BAS_LVL_STATUS_F_ADDITIONAL_STATUS) ?
             (ble_svc_bas_level_status_val.additional_status &
              BLE_SVC_BAS_ADD_STATUS_SERVICE_REQ_MASK) : 0xff;
    new_sr = (status->flags & BLE_SVC_BAS_LVL_STATUS_F_ADDITIONAL_STATUS) ?
             (status->additional_status & BLE_SVC_BAS_ADD_STATUS_SERVICE_REQ_MASK) : 0xff;

    power_changed = ble_svc_bas_level_status_val.power_state != status->power_state;
    service_changed = old_sr != new_sr;
#if BLE_SVC_BAS_EST_DATE
    was_present = ble_svc_bas_battery_present();
#endif

    ble_svc_bas_level_status_val = *status;

#if BLE_SVC_BAS_EST_DATE
    if (!was_present && ble_svc_bas_battery_present() &&
        ble_svc_bas_est_service_date_pending) {
        ble_svc_bas_est_service_date_try_send();
    }
#endif

    if (power_changed || service_changed) {
        ble_gatts_chr_updated(ble_svc_bas_level_status_handle);
    }

#if BLE_SVC_BAS_ENERGY
    if (power_changed) {
        ble_svc_bas_energy_status_send();
    }
#endif

#if BLE_SVC_BAS_CRIT_STATUS
    ble_svc_bas_critical_status_update();
#endif

    return 0;
}

int
ble_svc_bas_level_status_get(struct ble_svc_bas_level_status *status)
{
    if (status == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    *status = ble_svc_bas_level_status_val;
    return 0;
}
#endif

#if BLE_SVC_BAS_EST_DATE
/**
 * Transmits the Estimated Service Date if it differs from the last
 * transmitted value. Transmissions are at least one day apart; a change
 * within that window is deferred to the end of the window. While the battery
 * is not present the change stays pending until the battery is reinserted.
 */
static void
ble_svc_bas_est_service_date_try_send(void)
{
    ble_npl_time_t one_day;
    ble_npl_time_t elapsed;

    if (ble_svc_bas_est_service_date_tx_valid &&
        ble_svc_bas_est_service_date == ble_svc_bas_est_service_date_tx_value) {
        ble_svc_bas_est_service_date_pending = false;
        if (ble_svc_bas_est_service_date_timer_ready) {
            ble_npl_callout_stop(&ble_svc_bas_est_service_date_timer);
        }
        return;
    }

    ble_svc_bas_est_service_date_pending = true;

    if (!ble_svc_bas_battery_present()) {
        if (ble_svc_bas_est_service_date_timer_ready) {
            ble_npl_callout_stop(&ble_svc_bas_est_service_date_timer);
        }
        return;
    }

    one_day = ble_npl_time_ms_to_ticks32(BLE_SVC_BAS_ONE_DAY_MS);
    elapsed = ble_npl_time_get() - ble_svc_bas_est_service_date_tx_time;

    if (ble_svc_bas_est_service_date_tx_valid && elapsed < one_day) {
        if (ble_svc_bas_est_service_date_timer_ready &&
            !ble_npl_callout_is_active(&ble_svc_bas_est_service_date_timer)) {
            ble_npl_callout_reset(&ble_svc_bas_est_service_date_timer,
                                  one_day - elapsed);
        }
        return;
    }

    ble_svc_bas_est_service_date_tx_time = ble_npl_time_get();
    ble_svc_bas_est_service_date_tx_value = ble_svc_bas_est_service_date;
    ble_svc_bas_est_service_date_tx_valid = true;
    ble_svc_bas_est_service_date_pending = false;
    if (ble_svc_bas_est_service_date_timer_ready) {
        ble_npl_callout_stop(&ble_svc_bas_est_service_date_timer);
    }
    ble_gatts_chr_updated(ble_svc_bas_est_service_date_handle);
}

static void
ble_svc_bas_est_service_date_timer_cb(struct ble_npl_event *ev)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_svc_bas_ctx == NULL) {
        return;
    }
#endif

    if (ble_svc_bas_est_service_date_pending) {
        ble_svc_bas_est_service_date_try_send();
    }
}

/**
 * Set the Estimated Service Date, in days since 1970-01-01.
 *
 * Notified/indicated at most once a day. A change made within a day of the
 * last transmission is sent when that day ends, unless the value has
 * reverted in the meantime. Nothing is sent while the battery is not
 * present; a pending change is sent when the battery is reinserted.
 */
int
ble_svc_bas_est_service_date_set(uint32_t days)
{
    if (days > 0xffffff) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    if (ble_svc_bas_est_service_date == days) {
        return 0;
    }
    ble_svc_bas_est_service_date = days;

    ble_svc_bas_est_service_date_try_send();
    return 0;
}

uint32_t
ble_svc_bas_est_service_date_get(void)
{
    BLE_SVC_BAS_ENSURE_CTX(0);

    return ble_svc_bas_est_service_date;
}
#endif

#if BLE_SVC_BAS_CRIT_STATUS
/**
 * Get the Battery Critical Status. The value is derived from the Battery
 * Level Status characteristic.
 */
uint8_t
ble_svc_bas_critical_status_get(void)
{
    BLE_SVC_BAS_ENSURE_CTX(0);

    return ble_svc_bas_critical_status;
}
#endif

#if BLE_SVC_BAS_ENERGY
/**
 * Set the Battery Energy Status value.
 *
 * Notified/indicated when the Available Energy or Available Battery Capacity
 * changes by at least 1 percent since the last transmission.
 */
int
ble_svc_bas_energy_status_set(const struct ble_svc_bas_energy_status *status)
{
    const struct ble_svc_bas_energy_status *tx;
    bool capacity_valid;
    bool send;

    if (status == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    ble_svc_bas_energy_status_val = *status;
    tx = &ble_svc_bas_energy_status_tx;

    if (!ble_svc_bas_battery_present()) {
        return 0;
    }

    capacity_valid = (status->flags & BLE_SVC_BAS_ENERGY_F_AVAILABLE_CAPACITY) != 0;
    send = ble_svc_bas_energy_field_changed(BLE_SVC_BAS_ENERGY_F_AVAILABLE_ENERGY,
                                            tx->available_energy,
                                            status->available_energy,
                                            status->available_capacity,
                                            capacity_valid) ||
           ble_svc_bas_energy_field_changed(BLE_SVC_BAS_ENERGY_F_AVAILABLE_CAPACITY,
                                            tx->available_capacity,
                                            status->available_capacity,
                                            0, false);
    if (send) {
        ble_svc_bas_energy_status_send();
    }
    return 0;
}

int
ble_svc_bas_energy_status_get(struct ble_svc_bas_energy_status *status)
{
    if (status == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    *status = ble_svc_bas_energy_status_val;
    return 0;
}
#endif

#if BLE_SVC_BAS_TIME
/**
 * Set the Battery Time Status value.
 *
 * Notified/indicated when any field changes by at least 1 percent since the
 * last transmission, and never while the battery is not present.
 */
int
ble_svc_bas_time_status_set(const struct ble_svc_bas_time_status *status)
{
    const struct ble_svc_bas_time_status *tx;
    bool send;

    if (status == NULL ||
        status->time_until_discharged > 0xffffff ||
        status->time_until_discharged_on_standby > 0xffffff ||
        status->time_until_recharged > 0xffffff) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    ble_svc_bas_time_status_val = *status;
    tx = &ble_svc_bas_time_status_tx;

    if (!ble_svc_bas_battery_present()) {
        return 0;
    }

    send = ble_svc_bas_time_field_changed(0, tx->time_until_discharged,
                                          status->time_until_discharged) ||
           ble_svc_bas_time_field_changed(BLE_SVC_BAS_TIME_F_DISCHARGED_ON_STANDBY,
                                          tx->time_until_discharged_on_standby,
                                          status->time_until_discharged_on_standby) ||
           ble_svc_bas_time_field_changed(BLE_SVC_BAS_TIME_F_RECHARGED,
                                          tx->time_until_recharged,
                                          status->time_until_recharged);
    if (send) {
        ble_svc_bas_time_status_tx = *status;
        ble_gatts_chr_updated(ble_svc_bas_time_status_handle);
    }
    return 0;
}

int
ble_svc_bas_time_status_get(struct ble_svc_bas_time_status *status)
{
    if (status == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    *status = ble_svc_bas_time_status_val;
    return 0;
}
#endif

#if BLE_SVC_BAS_HEALTH
/**
 * Set the Battery Health Status value. Notified/indicated on change, never
 * while the battery is not present.
 */
int
ble_svc_bas_health_status_set(const struct ble_svc_bas_health_status *status)
{
    if (status == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    if (ble_svc_bas_health_status_val.flags == status->flags &&
        ble_svc_bas_health_status_val.health_summary == status->health_summary &&
        ble_svc_bas_health_status_val.cycle_count == status->cycle_count &&
        ble_svc_bas_health_status_val.current_temp == status->current_temp &&
        ble_svc_bas_health_status_val.deep_discharge_count == status->deep_discharge_count) {
        return 0;
    }
    ble_svc_bas_health_status_val = *status;

    if (ble_svc_bas_battery_present()) {
        ble_gatts_chr_updated(ble_svc_bas_health_status_handle);
    }
    return 0;
}

int
ble_svc_bas_health_status_get(struct ble_svc_bas_health_status *status)
{
    if (status == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    *status = ble_svc_bas_health_status_val;
    return 0;
}
#endif

#if BLE_SVC_BAS_HEALTH_INFO
/**
 * Set the Battery Health Information value. Indicated on change, never
 * while the battery is not present.
 */
int
ble_svc_bas_health_info_set(const struct ble_svc_bas_health_info *info)
{
    if (info == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    if (ble_svc_bas_health_info_val.flags == info->flags &&
        ble_svc_bas_health_info_val.cycle_count_designed_lifetime ==
        info->cycle_count_designed_lifetime &&
        ble_svc_bas_health_info_val.min_designed_op_temp == info->min_designed_op_temp &&
        ble_svc_bas_health_info_val.max_designed_op_temp == info->max_designed_op_temp) {
        return 0;
    }
    ble_svc_bas_health_info_val = *info;

    if (ble_svc_bas_battery_present()) {
        ble_gatts_chr_updated(ble_svc_bas_health_info_handle);
    }
    return 0;
}

int
ble_svc_bas_health_info_get(struct ble_svc_bas_health_info *info)
{
    if (info == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    *info = ble_svc_bas_health_info_val;
    return 0;
}
#endif

#if BLE_SVC_BAS_INFO
/**
 * Set the Battery Information value. Indicated on change, never while the
 * battery is not present.
 */
int
ble_svc_bas_battery_info_set(const struct ble_svc_bas_battery_info *info)
{
    if (info == NULL ||
        info->manufacture_date > 0xffffff ||
        info->expiration_date > 0xffffff) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    if (ble_svc_bas_battery_info_val.flags == info->flags &&
        ble_svc_bas_battery_info_val.features == info->features &&
        ble_svc_bas_battery_info_val.manufacture_date == info->manufacture_date &&
        ble_svc_bas_battery_info_val.expiration_date == info->expiration_date &&
        ble_svc_bas_battery_info_val.designed_capacity == info->designed_capacity &&
        ble_svc_bas_battery_info_val.low_energy == info->low_energy &&
        ble_svc_bas_battery_info_val.critical_energy == info->critical_energy &&
        ble_svc_bas_battery_info_val.chemistry == info->chemistry &&
        ble_svc_bas_battery_info_val.nominal_voltage == info->nominal_voltage &&
        ble_svc_bas_battery_info_val.aggregation_group == info->aggregation_group) {
        return 0;
    }
    ble_svc_bas_battery_info_val = *info;

    if (ble_svc_bas_battery_present()) {
        ble_gatts_chr_updated(ble_svc_bas_battery_info_handle);
    }
    return 0;
}

int
ble_svc_bas_battery_info_get(struct ble_svc_bas_battery_info *info)
{
    if (info == NULL) {
        return BLE_HS_EINVAL;
    }

    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    *info = ble_svc_bas_battery_info_val;
    return 0;
}
#endif

#if BLE_SVC_BAS_MFR_NAME
int
ble_svc_bas_manufacturer_name_set(const char *name)
{
    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    return ble_svc_bas_str_set(ble_svc_bas_mfr_name, &ble_svc_bas_mfr_name_len,
                               name, ble_svc_bas_mfr_name_handle);
}
#endif

#if BLE_SVC_BAS_MODEL_NUM
int
ble_svc_bas_model_number_set(const char *model)
{
    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    return ble_svc_bas_str_set(ble_svc_bas_model_number,
                               &ble_svc_bas_model_number_len,
                               model, ble_svc_bas_model_number_handle);
}
#endif

#if BLE_SVC_BAS_SERIAL_NUM
int
ble_svc_bas_serial_number_set(const char *serial)
{
    BLE_SVC_BAS_ENSURE_CTX(BLE_HS_ENOMEM);

    return ble_svc_bas_str_set(ble_svc_bas_serial_number,
                               &ble_svc_bas_serial_number_len,
                               serial, ble_svc_bas_serial_number_handle);
}
#endif

static void
ble_svc_bas_reset_values(void)
{
    ble_svc_bas_battery_level = 0;
#if BLE_SVC_BAS_LVL_STATUS
    memset(&ble_svc_bas_level_status_val, 0, sizeof(ble_svc_bas_level_status_val));
    ble_svc_bas_level_status_val.power_state =
        BLE_SVC_BAS_PWR_BATTERY_PRESENT |
        (BLE_SVC_BAS_PWR_EXT_UNKNOWN << BLE_SVC_BAS_PWR_WIRED_POS) |
        (BLE_SVC_BAS_PWR_EXT_UNKNOWN << BLE_SVC_BAS_PWR_WIRELESS_POS);
#endif
#if BLE_SVC_BAS_EST_DATE
    ble_svc_bas_est_service_date = 0;
    ble_svc_bas_est_service_date_tx_value = 0;
    ble_svc_bas_est_service_date_tx_time = 0;
    ble_svc_bas_est_service_date_tx_valid = false;
    ble_svc_bas_est_service_date_pending = false;
#endif
#if BLE_SVC_BAS_CRIT_STATUS
    ble_svc_bas_critical_status = 0;
#endif
#if BLE_SVC_BAS_ENERGY
    memset(&ble_svc_bas_energy_status_val, 0, sizeof(ble_svc_bas_energy_status_val));
    memset(&ble_svc_bas_energy_status_tx, 0, sizeof(ble_svc_bas_energy_status_tx));
#endif
#if BLE_SVC_BAS_TIME
    memset(&ble_svc_bas_time_status_val, 0, sizeof(ble_svc_bas_time_status_val));
    ble_svc_bas_time_status_val.time_until_discharged = BLE_SVC_BAS_TIME_UNKNOWN;
    ble_svc_bas_time_status_tx = ble_svc_bas_time_status_val;
#endif
#if BLE_SVC_BAS_HEALTH
    memset(&ble_svc_bas_health_status_val, 0, sizeof(ble_svc_bas_health_status_val));
#endif
#if BLE_SVC_BAS_HEALTH_INFO
    memset(&ble_svc_bas_health_info_val, 0, sizeof(ble_svc_bas_health_info_val));
#endif
#if BLE_SVC_BAS_INFO
    memset(&ble_svc_bas_battery_info_val, 0, sizeof(ble_svc_bas_battery_info_val));
#endif
#if BLE_SVC_BAS_MFR_NAME
    ble_svc_bas_mfr_name_len = 0;
#endif
#if BLE_SVC_BAS_MODEL_NUM
    ble_svc_bas_model_number_len = 0;
#endif
#if BLE_SVC_BAS_SERIAL_NUM
    ble_svc_bas_serial_number_len = 0;
#endif
}

/**
 * Deinitialize the Battery Service.
 * Note: Frees all registered GATT services as part of full BLE stack teardown.
 */
void
ble_svc_bas_deinit(void)
{
    ble_gatts_free_svcs();
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    ble_svc_bas_ctx_deinit();
#else
#if BLE_SVC_BAS_EST_DATE
    ble_svc_bas_est_service_date_timer_deinit();
#endif
    ble_svc_bas_initialized = false;
    ble_svc_bas_counted = false;
#endif
}

/**
 * Initialize the Battery Service.
 */
void
ble_svc_bas_init(void)
{
    int rc;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_svc_bas_ensure_ctx_init()) {
        BLE_HS_LOG(ERROR, "%s: no memory for BAS context\n", __func__);
        return;
    }
#endif

    if (ble_svc_bas_initialized) {
        return;
    }

    ble_svc_bas_reset_values();

#if BLE_SVC_BAS_EST_DATE
    if (!ble_svc_bas_est_service_date_timer_ready) {
        rc = ble_npl_callout_init(&ble_svc_bas_est_service_date_timer,
                                  nimble_port_get_dflt_eventq(),
                                  ble_svc_bas_est_service_date_timer_cb, NULL);
        if (rc != 0) {
            BLE_HS_LOG(ERROR, "%s: callout init failed rc=%d\n", __func__, rc);
            return;
        }
        ble_svc_bas_est_service_date_timer_ready = true;
    }
#endif

    if (!ble_svc_bas_counted) {
        rc = ble_gatts_count_cfg(ble_svc_bas_defs);
        if (rc != 0) {
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, rc);
            return;
        }
        ble_svc_bas_counted = true;
    }

    rc = ble_gatts_add_svcs(ble_svc_bas_defs);
    if (rc != 0) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, rc);
        return;
    }

    ble_svc_bas_initialized = true;
}
#endif
