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
#include "os/os.h"
#include "sysinit/sysinit.h"

#if CONFIG_BT_NIMBLE_ENABLED
#include "host/ble_hs.h"
#endif //CONFIG_BT_NIMBLE_ENABLED

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#ifdef ESP_PLATFORM
#include "esp_log.h"
#endif
#include "soc/soc_caps.h"

#if SOC_ESP_NIMBLE_CONTROLLER
#if CONFIG_SW_COEXIST_ENABLE
#include "esp_coexist_internal.h"
#endif
#endif

#ifdef CONFIG_BT_LE_HCI_INTERFACE_USE_UART
#include "transport/uart/ble_hci_uart.h"
#else
#include "transport/ram/ble_hci_ram.h"
#endif
#include "nimble/ble_hci_trans.h"

#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"

#define NIMBLE_PORT_LOG_TAG          "BLE_INIT"

extern void os_msys_init(void);

#if CONFIG_BT_NIMBLE_ENABLED 
extern void ble_hs_deinit(void);
static struct ble_hs_stop_listener stop_listener;
#endif //CONFIG_BT_NIMBLE_ENABLED

static struct ble_npl_eventq g_eventq_dflt;
static struct ble_npl_sem ble_hs_stop_sem;
static struct ble_npl_event ble_hs_ev_stop;

extern void os_msys_init(void);
extern void os_mempool_module_init(void);

void
nimble_port_init(void)
{
#if SOC_ESP_NIMBLE_CONTROLLER
    esp_bt_controller_config_t config_opts = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if(esp_bt_controller_init(&config_opts) != 0) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller init failed\n");
        return;
    }
#if CONFIG_BT_NIMBLE_ENABLED
     /* Initialize the host */
     ble_hs_init();
#endif
 
#else //SOC_ESP_NIMBLE_CONTROLLER

    /* Initialize the function pointers for OS porting */
    npl_freertos_funcs_init();

    npl_freertos_mempool_init();

    /* Initialize default event queue */

    ble_npl_eventq_init(&g_eventq_dflt);
    /* Initialize the global memory pool */
    os_mempool_module_init();
    os_msys_init();
    /* Initialize transport */
    ble_transport_init();
    /* Initialize the host */
    ble_transport_hs_init();

#if NIMBLE_CFG_CONTROLLER
#ifndef RIOT_VERSION
    hal_timer_init(5, NULL);
    os_cputime_init(32768);
#endif
    ble_transport_ll_init();
#endif
}

void
nimble_port_deinit(void)
{
#if SOC_ESP_NIMBLE_CONTROLLER

#if CONFIG_BT_NIMBLE_ENABLED
   ble_hs_deinit();
#endif //CONFIG_BT_NIMBLE_ENABLED

   esp_bt_controller_deinit();

   /* Delete the host task */
   nimble_port_freertos_deinit();
#else
   ble_npl_eventq_deinit(&g_eventq_dflt);

   ble_hs_deinit();
#endif
}

#if CONFIG_BT_NIMBLE_ENABLED
/**
 * Called when the host stop procedure has completed.
 */
static void
ble_hs_stop_cb(int status, void *arg)
{
    ble_npl_sem_release(&ble_hs_stop_sem);
}
#endif

static void
nimble_port_stop_cb(struct ble_npl_event *ev)
{
    ble_npl_sem_release(&ble_hs_stop_sem);
}

int
nimble_port_stop(void)
{
    int rc = 0;

    ble_npl_sem_init(&ble_hs_stop_sem, 0);
#if CONFIG_BT_NIMBLE_ENABLED
    /* Initiate a host stop procedure. */
    rc = ble_hs_stop(&stop_listener, ble_hs_stop_cb,
            NULL);
    if (rc != 0) {
        ble_npl_sem_deinit(&ble_hs_stop_sem);
        return rc;
    }
#endif //CONFIG_BT_NIMBLE_ENABLED

    /* Wait till the host stop procedure is complete */
    ble_npl_sem_pend(&ble_hs_stop_sem, BLE_NPL_TIME_FOREVER);

    ble_npl_event_init(&ble_hs_ev_stop, nimble_port_stop_cb,
            NULL);
    ble_npl_eventq_put(&g_eventq_dflt, &ble_hs_ev_stop);

    /* Wait till the event is serviced */
    ble_npl_sem_pend(&ble_hs_stop_sem, BLE_NPL_TIME_FOREVER);

    ble_npl_sem_deinit(&ble_hs_stop_sem);

    return rc;
}

void
IRAM_ATTR nimble_port_run(void)
{
    struct ble_npl_event *ev;

    while (1) {
        ev = ble_npl_eventq_get(&g_eventq_dflt, BLE_NPL_TIME_FOREVER);
        ble_npl_event_run(ev);
        if (ev == &ble_hs_ev_stop) {
            break;
        }

    }
}

struct ble_npl_eventq *
IRAM_ATTR nimble_port_get_dflt_eventq(void)
{
    return &g_eventq_dflt;
}
