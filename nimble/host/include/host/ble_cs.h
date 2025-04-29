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

/* All Channel Sounding APIs are experimental and subject to change at any time */

#ifndef H_BLE_CS_
#define H_BLE_CS_
#include "syscfg/syscfg.h"
#include "host/ble_hs.h"
#include "nimble/hci_common.h"

#define BLE_CS_EVENT_CS_PROCEDURE_COMPLETE (0)
#define BLE_CS_EVENT_CS_PROCEDURE_ENABLE_COMPLETE (1)
#define BLE_CS_EVENT_SUBEVET_RESULT (2)
#define BLE_CS_EVENT_SUBEVET_RESULT_CONTINUE (3)

struct ble_cs_event {
    uint8_t type;
    union
    {
        struct
        {
            uint16_t conn_handle;
            uint8_t status;
        } procedure_complete;
        struct
        {
            uint16_t conn_handle;
            uint8_t config_id;
            uint16_t start_acl_conn_event_counter;
            uint16_t procedure_counter;
            uint16_t frequency_compensation;
            uint8_t reference_power_level;
            uint8_t procedure_done_status;
            uint8_t subevent_done_status;
            uint8_t abort_reason;
            uint8_t num_antenna_paths;
            uint8_t num_steps_reported;
            const struct cs_steps_data *steps;
        }subev_result;
        struct
        {
            uint16_t conn_handle;
            uint8_t config_id;
            uint8_t procedure_done_status;
            uint8_t subevent_done_status;
            uint8_t abort_reason;
            uint8_t num_antenna_paths;
            uint8_t num_steps_reported;
            const struct cs_steps_data *steps;
        }subev_result_continue;

    };
};

typedef int ble_cs_event_fn(struct ble_cs_event *event, void *arg);

struct ble_cs_initiator_procedure_start_params {
    uint16_t conn_handle;
    ble_cs_event_fn *cb;
    void *cb_arg;
};

struct ble_cs_reflector_setup_params {
    ble_cs_event_fn *cb;
    void *cb_arg;
};


int ble_cs_initiator_procedure_start(const struct ble_cs_initiator_procedure_start_params *params);
int ble_cs_initiator_procedure_terminate(uint16_t conn_handle);
int ble_cs_reflector_setup(struct ble_cs_reflector_setup_params *params);
#endif
