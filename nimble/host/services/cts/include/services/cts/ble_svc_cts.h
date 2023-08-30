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

#ifndef H_BLE_SVC_CTS_
#define H_BLE_SVC_CTS_

#define BLE_SVC_CTS_ERR_DATA_FIELD_IGNORED      0x80
/* 16 Bit Current Time Service UUID */
#define BLE_SVC_CTS_UUID16                      0x1805

/* 16 Bit Current Time Service Characteristics UUIDs */
#define BLE_SVC_CTS_CHR_UUID16_CURRENT_TIME     0x2A2B /* read write notify */
#define BLE_SVC_CTS_CHR_UUID16_LOCAL_TIME_INFO  0x2A0F /* read write */
#define BLE_SVC_CTS_CHR_UUID16_REF_TIME_INFO    0x2A14 /* read */

/* adjust reason masks */
#define MANUAL_TIME_UPDATE_MASK                 1
#define EXTERNAL_REFERENCE_TIME_UPDATE_MASK     2
#define CHANGE_OF_TIME_ZONE_MASK                4
#define CHANGE_OF_DST_MASK                      8

/* Time Source Values */
#define TIME_SOURCE_UNKNOWN                     0
#define TIME_SOURCE_NETWORK_TIME_PROTOCOL       1
#define TIME_SOURCE_GPS                         2
#define TIME_SOURCE_RADIO_TIME_SIGNAL           3
#define TIME_SOURCE_MANUAL                      4
#define TIME_SOURCE_ATOMIC_CLOCK                5
#define TIME_SOURCE_CELLULAR_NETWORK            6

/* DST VALUES */
#define TIME_STANDARD                           0
#define TIME_HALF_AN_HOUR_DAYLIGHT              2
#define TIME_DAYLIGHT                           4
#define TIME_DOUBLE_DAYLIGHT                    8
#define TIME_DST_OFFSET_UNKNOWN                 255

/* current time characteristic */
struct date_time {
    /** year.
     *  valid range : 1582 - 9999
     *  0 means year not known
     */
    uint16_t year;
    /** month.
     *  valid range : 1(January) - 12(December)
     *  0 means month not known
     */
    uint8_t month;
    /** day.
     *  valid range : 1 - 31
     *  0 means day not known
     */
    uint8_t day;
    /** hours.
     *  valid range : 0 - 23
     */
    uint8_t hours;
    /** minutes.
     *  valid range : 0 - 59
     */
    uint8_t minutes;
    /** seconds.
     *  valid range : 0 - 59
     */
    uint8_t seconds;
}__attribute__((packed));

struct day_date_time {
    struct date_time d_t;
    /** day_of_week.
        valid range : 1(Monday) - 7(Sunday)
     *  0 means day of week not known
     */
    uint8_t day_of_week;
}__attribute__((packed));

struct exact_time_256 {
    struct day_date_time d_d_t;
    /** fractions_256.
     *  the number of 1 / 256 fractions of second
     *  valid range : 0 - 255
     */
    uint8_t fractions_256;
}__attribute__((packed));

struct ble_svc_cts_curr_time {
    struct exact_time_256 et_256;
    uint8_t adjust_reason;
}__attribute__((packed));

/* local time information characteristic */
struct ble_svc_cts_local_time_info {
    /** timezone.
     *  valid range : -48 to 56
     *  -128 means timezone offset not known
     */
    int8_t timezone;
    /** dst_offset.
     *  allowed values : 0, 2, 4, 8, 255
     */
    uint8_t dst_offset;
}__attribute__((packed));

/* reference time information */
struct ble_svc_cts_reference_time_info {
    /** time_source.
     *  valid range : 0 - 253
     *  255 means not known
     */
    uint8_t time_source;
    uint8_t time_accuracy;
    /** days_since_update.
     *  valid range : 0 - 254
     */
    uint8_t days_since_update;
    /** hours_since_update.
     *  valid range : 0 - 23
     */
    uint8_t hours_since_update;
}__attribute__((packed));

/* callback to be called to get current time */
typedef int ble_svc_cts_fetch_current_time_fn(struct ble_svc_cts_curr_time *);
typedef int ble_svc_cts_fetch_local_time_info_fn(struct ble_svc_cts_local_time_info *);
typedef int ble_svc_cts_fetch_ref_time_info_fn(struct ble_svc_cts_reference_time_info *);
typedef int ble_svc_cts_set_local_time_info_fn(struct ble_svc_cts_local_time_info);
typedef int ble_svc_cts_set_current_time_fn(struct ble_svc_cts_curr_time);

/* CTS configuration to be filled at init */
struct ble_svc_cts_cfg {
    ble_svc_cts_fetch_current_time_fn *fetch_time_cb;
    ble_svc_cts_fetch_local_time_info_fn *local_time_info_cb;
    ble_svc_cts_fetch_ref_time_info_fn *ref_time_info_cb;
    ble_svc_cts_set_current_time_fn *set_time_cb;
    ble_svc_cts_set_local_time_info_fn *set_local_time_info_cb;
};

void
ble_svc_cts_init(struct ble_svc_cts_cfg cfg);
void
ble_svc_cts_time_updated(void);

#endif
