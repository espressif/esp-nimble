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

#ifdef __cplusplus
}
#endif

#endif /* _NIMBLE_PORT_FREERTOS_H */
