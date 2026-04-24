/*
 * SPDX-FileCopyrightText: 2015-2022 The Apache Software Foundation (ASF)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileContributor: 2025 Espressif Systems (Shanghai) CO LTD
 */

#ifndef _NIMBLE_PORT_FREERTOS_H
#define _NIMBLE_PORT_FREERTOS_H

#include "nimble/nimble_npl.h"

#ifdef __cplusplus
extern "C" {
#endif

void nimble_port_freertos_init(TaskFunction_t host_task_fn);

void nimble_port_freertos_deinit(void);

/**
 * @brief esp_nimble_enable - Initialize the NimBLE host task
 * 
 * @param host_task 
 * @return esp_err_t 
 */
esp_err_t esp_nimble_enable(void *host_task);

/**
 * @brief esp_nimble_disable - Disable the NimBLE host task
 * 
 * @return esp_err_t 
 */
esp_err_t esp_nimble_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* _NIMBLE_PORT_FREERTOS_H */
