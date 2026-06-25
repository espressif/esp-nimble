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

#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "esp_log.h"
#if CONFIG_BT_CONTROLLER_ENABLED
#include "esp_bt.h"
#endif

#define NIMBLE_PORT_LOG_TAG "BLE_INIT"

static TaskHandle_t host_task_h = NULL;

/**
 * @brief esp_nimble_enable - Initialize the NimBLE host
 * 
 * @param host_task 
 * @return esp_err_t 
 */
esp_err_t esp_nimble_enable(void *host_task)
{
    if (host_task == NULL) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "esp_nimble_enable: host_task is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (host_task_h != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Create task where NimBLE host will run. It is not strictly necessary to
     * have separate task for NimBLE host, but since something needs to handle
     * default queue it is just easier to make separate task which does this.
     */
    BaseType_t ret = xTaskCreatePinnedToCore(host_task, "nimble_host",
                                             NIMBLE_HS_STACK_SIZE, NULL,
                                             (configMAX_PRIORITIES - 4),
                                             &host_task_h, NIMBLE_CORE);
    if (ret != pdPASS) {
        host_task_h = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;

}

/**
 * @brief esp_nimble_disable - Disable the NimBLE host
 * 
 * @return esp_err_t 
 */
esp_err_t esp_nimble_disable(void)
{
    if (host_task_h) {
        TaskHandle_t tmp = host_task_h;
        host_task_h = NULL;
        vTaskDelete(tmp);
    }
    return ESP_OK;
}


/**
 * @brief nimble_port_freertos_init - Adapt to native nimble api
 * 
 * @param host_task_fn 
 */
void
nimble_port_freertos_init(TaskFunction_t host_task_fn)
{
    esp_err_t err = esp_nimble_enable(host_task_fn);
    if (err != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG,
                 "nimble_port_freertos_init: esp_nimble_enable failed (%d)", err);
        /* BLE host task creation is critical; abort so the system does not
         * continue in a broken state where all BLE operations will fail. */
        abort();
    }
}

/**
 * @brief nimble_port_freertos_deinit - Adapt to native nimble api
 * 
 */
void
nimble_port_freertos_deinit(void)
{
    esp_nimble_disable();
}
