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

#ifndef H_BLE_SVC_BAS_
#define H_BLE_SVC_BAS_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 16 Bit Battery Service UUID */
#define BLE_SVC_BAS_UUID16                                   0x180F

/* 16 Bit Battery Service Characteristic UUIDs */
#define BLE_SVC_BAS_CHR_UUID16_BATTERY_LEVEL                 0x2A19
#define BLE_SVC_BAS_CHR_UUID16_MANUFACTURER_NAME             0x2A29
#define BLE_SVC_BAS_CHR_UUID16_MODEL_NUMBER                  0x2A24
#define BLE_SVC_BAS_CHR_UUID16_SERIAL_NUMBER                 0x2A25
#define BLE_SVC_BAS_CHR_UUID16_BATTERY_CRITICAL_STATUS       0x2BE9
#define BLE_SVC_BAS_CHR_UUID16_BATTERY_HEALTH_STATUS         0x2BEA
#define BLE_SVC_BAS_CHR_UUID16_BATTERY_HEALTH_INFO           0x2BEB
#define BLE_SVC_BAS_CHR_UUID16_BATTERY_INFO                  0x2BEC
#define BLE_SVC_BAS_CHR_UUID16_BATTERY_LEVEL_STATUS          0x2BED
#define BLE_SVC_BAS_CHR_UUID16_BATTERY_TIME_STATUS           0x2BEE
#define BLE_SVC_BAS_CHR_UUID16_ESTIMATED_SERVICE_DATE        0x2BEF
#define BLE_SVC_BAS_CHR_UUID16_BATTERY_ENERGY_STATUS         0x2BF0

/* 16 Bit Characteristic Presentation Format descriptor UUID */
#define BLE_SVC_BAS_DSC_UUID16_PRES_FORMAT                   0x2904

/* GATT Namespace Descriptor value for "main" */
#define BLE_SVC_BAS_DESCRIPTION_MAIN                         0x0106

/* Battery Level Status: Flags field */
#define BLE_SVC_BAS_LVL_STATUS_F_IDENTIFIER                  0x01
#define BLE_SVC_BAS_LVL_STATUS_F_BATTERY_LEVEL               0x02
#define BLE_SVC_BAS_LVL_STATUS_F_ADDITIONAL_STATUS           0x04

/* Battery Level Status: Power State field */
#define BLE_SVC_BAS_PWR_BATTERY_PRESENT                      0x0001

#define BLE_SVC_BAS_PWR_WIRED_POS                            1
#define BLE_SVC_BAS_PWR_WIRED_MASK                           (0x3 << BLE_SVC_BAS_PWR_WIRED_POS)
#define BLE_SVC_BAS_PWR_WIRELESS_POS                         3
#define BLE_SVC_BAS_PWR_WIRELESS_MASK                        (0x3 << BLE_SVC_BAS_PWR_WIRELESS_POS)
/* Values for the wired/wireless external power source connected bits */
#define BLE_SVC_BAS_PWR_EXT_NO                               0
#define BLE_SVC_BAS_PWR_EXT_YES                              1
#define BLE_SVC_BAS_PWR_EXT_UNKNOWN                          2

#define BLE_SVC_BAS_PWR_CHARGE_STATE_POS                     5
#define BLE_SVC_BAS_PWR_CHARGE_STATE_MASK                    (0x3 << BLE_SVC_BAS_PWR_CHARGE_STATE_POS)
#define BLE_SVC_BAS_CHARGE_STATE_UNKNOWN                     0
#define BLE_SVC_BAS_CHARGE_STATE_CHARGING                    1
#define BLE_SVC_BAS_CHARGE_STATE_DISCHARGING_ACTIVE          2
#define BLE_SVC_BAS_CHARGE_STATE_DISCHARGING_INACTIVE        3

#define BLE_SVC_BAS_PWR_CHARGE_LEVEL_POS                     7
#define BLE_SVC_BAS_PWR_CHARGE_LEVEL_MASK                    (0x3 << BLE_SVC_BAS_PWR_CHARGE_LEVEL_POS)
#define BLE_SVC_BAS_CHARGE_LEVEL_UNKNOWN                     0
#define BLE_SVC_BAS_CHARGE_LEVEL_GOOD                        1
#define BLE_SVC_BAS_CHARGE_LEVEL_LOW                         2
#define BLE_SVC_BAS_CHARGE_LEVEL_CRITICAL                    3

#define BLE_SVC_BAS_PWR_CHARGING_TYPE_POS                    9
#define BLE_SVC_BAS_PWR_CHARGING_TYPE_MASK                   (0x7 << BLE_SVC_BAS_PWR_CHARGING_TYPE_POS)
#define BLE_SVC_BAS_CHARGING_TYPE_UNKNOWN                    0
#define BLE_SVC_BAS_CHARGING_TYPE_CONSTANT_CURRENT           1
#define BLE_SVC_BAS_CHARGING_TYPE_CONSTANT_VOLTAGE           2
#define BLE_SVC_BAS_CHARGING_TYPE_TRICKLE                    3
#define BLE_SVC_BAS_CHARGING_TYPE_FLOAT                      4

#define BLE_SVC_BAS_PWR_FAULT_BATTERY                        0x1000
#define BLE_SVC_BAS_PWR_FAULT_EXT_POWER_SOURCE               0x2000
#define BLE_SVC_BAS_PWR_FAULT_OTHER                          0x4000

/* Battery Level Status: Additional Status field */
#define BLE_SVC_BAS_ADD_STATUS_SERVICE_REQ_MASK              0x03
#define BLE_SVC_BAS_SERVICE_REQ_FALSE                        0
#define BLE_SVC_BAS_SERVICE_REQ_TRUE                         1
#define BLE_SVC_BAS_SERVICE_REQ_UNKNOWN                      2
#define BLE_SVC_BAS_ADD_STATUS_BATTERY_FAULT                 0x04

/* Battery Critical Status field */
#define BLE_SVC_BAS_CRIT_CRITICAL_POWER_STATE                0x01
#define BLE_SVC_BAS_CRIT_IMMEDIATE_SERVICE_REQ               0x02

/* Battery Energy Status: Flags field */
#define BLE_SVC_BAS_ENERGY_F_EXT_SOURCE_POWER                0x01
#define BLE_SVC_BAS_ENERGY_F_PRESENT_VOLTAGE                 0x02
#define BLE_SVC_BAS_ENERGY_F_AVAILABLE_ENERGY                0x04
#define BLE_SVC_BAS_ENERGY_F_AVAILABLE_CAPACITY              0x08
#define BLE_SVC_BAS_ENERGY_F_CHARGE_RATE                     0x10
#define BLE_SVC_BAS_ENERGY_F_ENERGY_AT_LAST_CHARGE           0x20

/* Battery Time Status: Flags field */
#define BLE_SVC_BAS_TIME_F_DISCHARGED_ON_STANDBY             0x01
#define BLE_SVC_BAS_TIME_F_RECHARGED                         0x02

/* Battery Time Status special values (uint24, minutes) */
#define BLE_SVC_BAS_TIME_UNKNOWN                             0xFFFFFF
#define BLE_SVC_BAS_TIME_GREATER_THAN_MAX                    0xFFFFFE

/* Battery Health Status: Flags field */
#define BLE_SVC_BAS_HEALTH_F_SUMMARY                         0x01
#define BLE_SVC_BAS_HEALTH_F_CYCLE_COUNT                     0x02
#define BLE_SVC_BAS_HEALTH_F_CURRENT_TEMP                    0x04
#define BLE_SVC_BAS_HEALTH_F_DEEP_DISCHARGE_COUNT            0x08

/* Battery Health Information: Flags field */
#define BLE_SVC_BAS_HEALTH_INFO_F_CYCLE_COUNT_LIFETIME       0x01
#define BLE_SVC_BAS_HEALTH_INFO_F_DESIGNED_OP_TEMP           0x02

/* Battery Information: Flags field */
#define BLE_SVC_BAS_INFO_F_MANUFACTURE_DATE                  0x0001
#define BLE_SVC_BAS_INFO_F_EXPIRATION_DATE                   0x0002
#define BLE_SVC_BAS_INFO_F_DESIGNED_CAPACITY                 0x0004
#define BLE_SVC_BAS_INFO_F_LOW_ENERGY                        0x0008
#define BLE_SVC_BAS_INFO_F_CRITICAL_ENERGY                   0x0010
#define BLE_SVC_BAS_INFO_F_CHEMISTRY                         0x0020
#define BLE_SVC_BAS_INFO_F_NOMINAL_VOLTAGE                   0x0040
#define BLE_SVC_BAS_INFO_F_AGGREGATION_GROUP                 0x0080

/* Battery Information: Battery Features field */
#define BLE_SVC_BAS_FEAT_REPLACEABLE                         0x01
#define BLE_SVC_BAS_FEAT_RECHARGEABLE                        0x02

/**
 * Battery Level Status value.
 *
 * The Identifier field is taken from BLE_SVC_BAS_PRES_FMT_DESCRIPTION and the
 * Battery Level field from the Battery Level characteristic; set the
 * corresponding flags to include them.
 */
struct ble_svc_bas_level_status {
    uint8_t flags;
    uint16_t power_state;
    uint8_t additional_status;
};

/**
 * Battery Energy Status value. All numeric fields are medfloat16 (SFLOAT)
 * encoded values.
 */
struct ble_svc_bas_energy_status {
    uint8_t flags;
    uint16_t ext_source_power;
    uint16_t present_voltage;
    uint16_t available_energy;
    uint16_t available_capacity;
    uint16_t charge_rate;
    uint16_t energy_at_last_charge;
};

/** Battery Time Status value. Times are uint24 minutes. */
struct ble_svc_bas_time_status {
    uint8_t flags;
    uint32_t time_until_discharged;
    uint32_t time_until_discharged_on_standby;
    uint32_t time_until_recharged;
};

/** Battery Health Status value. */
struct ble_svc_bas_health_status {
    uint8_t flags;
    uint8_t health_summary;
    uint16_t cycle_count;
    int8_t current_temp;
    uint16_t deep_discharge_count;
};

/** Battery Health Information value. */
struct ble_svc_bas_health_info {
    uint8_t flags;
    uint16_t cycle_count_designed_lifetime;
    int8_t min_designed_op_temp;
    int8_t max_designed_op_temp;
};

/**
 * Battery Information value. Dates are uint24 days since 1970-01-01;
 * capacity, energy and voltage fields are medfloat16 (SFLOAT) encoded values.
 */
struct ble_svc_bas_battery_info {
    uint16_t flags;
    uint8_t features;
    uint32_t manufacture_date;
    uint32_t expiration_date;
    uint16_t designed_capacity;
    uint16_t low_energy;
    uint16_t critical_energy;
    uint8_t chemistry;
    uint16_t nominal_voltage;
    uint8_t aggregation_group;
};

int ble_svc_bas_battery_level_set(uint8_t level);
uint8_t ble_svc_bas_battery_level_get(void);

#if MYNEWT_VAL(BLE_SVC_BAS_BATTERY_LEVEL_STATUS_ENABLE)
int ble_svc_bas_level_status_set(const struct ble_svc_bas_level_status *status);
int ble_svc_bas_level_status_get(struct ble_svc_bas_level_status *status);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_EST_SERVICE_DATE_ENABLE)
int ble_svc_bas_est_service_date_set(uint32_t days);
uint32_t ble_svc_bas_est_service_date_get(void);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_CRITICAL_STATUS_ENABLE)
uint8_t ble_svc_bas_critical_status_get(void);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_ENERGY_STATUS_ENABLE)
int ble_svc_bas_energy_status_set(const struct ble_svc_bas_energy_status *status);
int ble_svc_bas_energy_status_get(struct ble_svc_bas_energy_status *status);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_TIME_STATUS_ENABLE)
int ble_svc_bas_time_status_set(const struct ble_svc_bas_time_status *status);
int ble_svc_bas_time_status_get(struct ble_svc_bas_time_status *status);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_HEALTH_STATUS_ENABLE)
int ble_svc_bas_health_status_set(const struct ble_svc_bas_health_status *status);
int ble_svc_bas_health_status_get(struct ble_svc_bas_health_status *status);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_HEALTH_INFO_ENABLE)
int ble_svc_bas_health_info_set(const struct ble_svc_bas_health_info *info);
int ble_svc_bas_health_info_get(struct ble_svc_bas_health_info *info);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_BATTERY_INFO_ENABLE)
int ble_svc_bas_battery_info_set(const struct ble_svc_bas_battery_info *info);
int ble_svc_bas_battery_info_get(struct ble_svc_bas_battery_info *info);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_MANUFACTURER_NAME_ENABLE)
int ble_svc_bas_manufacturer_name_set(const char *name);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_MODEL_NUMBER_ENABLE)
int ble_svc_bas_model_number_set(const char *model);
#endif

#if MYNEWT_VAL(BLE_SVC_BAS_SERIAL_NUMBER_ENABLE)
int ble_svc_bas_serial_number_set(const char *serial);
#endif

void ble_svc_bas_init(void);
void ble_svc_bas_deinit(void);

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
int ble_svc_bas_ensure_ctx_init(void);
void ble_svc_bas_ctx_deinit(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
