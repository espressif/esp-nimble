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
#include <stdbool.h>
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

#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if CONFIG_BT_CONTROLLER_ENABLED
#include "esp_bt.h"
#endif
#if !SOC_ESP_NIMBLE_CONTROLLER && CONFIG_BT_CONTROLLER_ENABLED
#include "esp_nimble_hci.h"
#endif
#if !CONFIG_BT_CONTROLLER_ENABLED
#include "nimble/transport.h"
#endif
#if (BT_HCI_LOG_INCLUDED == TRUE)
#include "hci_log/bt_hci_log.h"
#endif // (BT_HCI_LOG_INCLUDED == TRUE)
#include "bt_common.h"

#define NIMBLE_PORT_LOG_TAG          "BLE_INIT"

extern void os_msys_init(void);

#if CONFIG_BT_NIMBLE_ENABLED 
extern void ble_hs_deinit(void);
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#include "esp_nimble_mem.h"
typedef struct {
    struct ble_npl_eventq eventq;
    struct ble_npl_sem stop_sem;
    struct ble_npl_event ev_stop;
#if CONFIG_BT_NIMBLE_ENABLED
    struct ble_hs_stop_listener listener;
#endif
} ble_npl_ctx_t;

static ble_npl_ctx_t *ble_npl_ctx;

static bool ble_npl_deiniting = false;
static struct ble_npl_eventq g_eventq_shutdown_fallback = {0};

#define g_eventq_dflt   (ble_npl_ctx->eventq)
#define ble_hs_stop_sem (ble_npl_ctx->stop_sem)
#define ble_hs_ev_stop  (ble_npl_ctx->ev_stop)

#if CONFIG_BT_NIMBLE_ENABLED
#define stop_listener   (ble_npl_ctx->listener)
#endif

#else
static struct ble_npl_eventq g_eventq_dflt;
static struct ble_npl_sem ble_hs_stop_sem;
static struct ble_npl_event ble_hs_ev_stop;

#if CONFIG_BT_NIMBLE_ENABLED
static struct ble_hs_stop_listener stop_listener;
#endif //CONFIG_BT_NIMBLE_ENABLED

#endif

extern void os_msys_init(void);
extern void os_mempool_module_init(void);
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
extern void os_mempool_deinit(void);
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
static int
ble_npl_ensure_ctx(void)
{
    if (ble_npl_ctx != NULL) {
        return 0;
    }

    /* Don't allocate during shutdown - context was already freed */
    if (ble_npl_deiniting) {
        return BLE_HS_ENOENT;
    }

    ble_npl_ctx = nimble_platform_mem_calloc(1, sizeof(*ble_npl_ctx));
    if (ble_npl_ctx == NULL) {
        return BLE_HS_ENOMEM;
    }

    return 0;
}

/* Reset the deiniting flag - called at start of new init cycle */
static void
ble_npl_reset_deinit_flag(void)
{
    ble_npl_deiniting = false;
}
#endif

/**
 * Called when the host stop procedure has completed.
 */
static void
ble_hs_stop_cb(int status, void *arg)
{
    ble_npl_sem_release(&ble_hs_stop_sem);
}

static void
nimble_port_stop_cb(struct ble_npl_event *ev)
{
    ble_npl_sem_release(&ble_hs_stop_sem);
}

/**
 * @brief esp_nimble_init - Initialize the NimBLE host stack
 * 
 * @return esp_err_t 
 */
esp_err_t esp_nimble_init(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    /* Reset deinit flag at start of new init cycle */
    ble_npl_reset_deinit_flag();
    
    if (ble_npl_ensure_ctx()) {
        return ESP_FAIL;
    }
#endif

#if CONFIG_BT_CONTROLLER_DISABLED
    esp_err_t ret;
#endif
#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
    /* Initialize the global memory pool */
    os_mempool_module_init();

    /* Initialize the function pointers for OS porting */
    npl_freertos_funcs_init();

    npl_freertos_mempool_init();

#if CONFIG_BT_CONTROLLER_ENABLED
    if(esp_nimble_hci_init() != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "hci inits failed\n");
        return ESP_FAIL;
    }
#else
    ret = ble_buf_alloc();
    if (ret != ESP_OK) {
        ble_buf_free();
        return ESP_FAIL;
    }
    ble_transport_init();
#if MYNEWT_VAL(BLE_QUEUE_CONG_CHECK)
    ble_adv_list_init();
#endif
#endif

    /* Initialize default event queue */
    ble_npl_eventq_init(&g_eventq_dflt);
    os_msys_init();

#endif

    /* Initialize the host */
    ble_transport_hs_init();
    
    ble_transport_ll_init();

    return ESP_OK;
}

/**
 * @brief esp_nimble_deinit - Deinitialize the NimBLE host stack
 * 
 * @return esp_err_t 
 */
esp_err_t esp_nimble_deinit(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    ble_npl_deiniting = true;
#endif

    ble_transport_ll_deinit();

#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
#if CONFIG_BT_CONTROLLER_ENABLED
    if(esp_nimble_hci_deinit() != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "hci deinit failed\n");
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        ble_npl_deiniting = false;  /* Reset flag before early return */
#endif
        return ESP_FAIL;
    }
#else
#if MYNEWT_VAL(BLE_QUEUE_CONG_CHECK)
    ble_adv_list_deinit();
#endif
    ble_transport_deinit();
    ble_buf_free();
#endif

#endif

#if CONFIG_BT_CONTROLLER_ENABLED
    ble_npl_eventq_deinit(&g_eventq_dflt);
#endif

    ble_hs_deinit();

#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
    npl_freertos_funcs_deinit();
#endif

#if !SOC_ESP_NIMBLE_CONTROLLER
    npl_freertos_mempool_deinit();
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_npl_ctx) {
        nimble_platform_mem_free(ble_npl_ctx);
        ble_npl_ctx = NULL;
    }
    /* Keep ble_npl_deiniting = true until nimble_port_deinit completes.
     * This prevents controller deinit from re-allocating ble_npl_ctx when it
     * calls nimble_port_get_dflt_eventq(). The flag will be reset at the START
     * of the next init cycle (in ble_npl_reset_deinit_flag). */

#endif
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    os_mempool_deinit();
#endif

    return ESP_OK;
}

/**
 * @brief nimble_port_init - Initialize controller and NimBLE host stack
 *
 * @return esp_err_t
 */
esp_err_t
nimble_port_init(void)
{
    esp_err_t ret;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    /* Reset deinit flag at start of new init cycle.
     * This is needed because the flag remains true after deinit to prevent
     * re-allocation during controller shutdown. */
    ble_npl_reset_deinit_flag();
#endif

#if CONFIG_IDF_TARGET_ESP32 && CONFIG_BT_CONTROLLER_ENABLED
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
#endif
#if CONFIG_BT_CONTROLLER_ENABLED
    esp_bt_controller_config_t config_opts = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&config_opts);
    if (ret != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller init failed\n");
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        // Deinit to free any memory the controller is using.
        if(esp_bt_controller_deinit() != ESP_OK) {
            ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller deinit failed\n");
        }

        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller enable failed\n");
        return ret;
    }
#endif

    ret = esp_nimble_init();
    if (ret != ESP_OK) {

#if CONFIG_BT_CONTROLLER_ENABLED
	// Disable and deinit controller to free memory
        if(esp_bt_controller_disable() != ESP_OK) {
            ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller disable failed\n");
        }

	if(esp_bt_controller_deinit() != ESP_OK) {
            ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller deinit failed\n");
        }
#endif

	ESP_LOGE(NIMBLE_PORT_LOG_TAG, "nimble host init failed\n");
        return ret;
    }

#if MYNEWT_VAL(BT_HCI_LOG_INCLUDED)
    bt_hci_log_init();
#endif // MYNEWT_VAL(BT_HCI_LOG_INCLUDED)

    return ESP_OK;
}

/**
 * @brief nimble_port_deinit - Deinitialize controller and NimBLE host stack
 *
 * @return esp_err_t
 */

esp_err_t
nimble_port_deinit(void)
{
    esp_err_t ret;

    ret = esp_nimble_deinit();
    if(ret != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "nimble host deinit failed\n");
        return ret;
    }

#if CONFIG_BT_CONTROLLER_ENABLED
    ret = esp_bt_controller_disable();
    if(ret != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller disable failed\n");
        return ret;
    }

    ret = esp_bt_controller_deinit();
    if(ret != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller deinit failed\n");
        return ret;
    }
#endif

#if (BT_HCI_LOG_INCLUDED == TRUE)
    bt_hci_log_deinit();
#endif // (BT_HCI_LOG_INCLUDED == TRUE)

    return ESP_OK;
}


int
nimble_port_stop(void)
{
    esp_err_t err = ESP_OK;
    ble_npl_error_t rc;

    rc = ble_npl_sem_init(&ble_hs_stop_sem, 0);

    if( rc != 0) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "sem init failed with reason: %d \n", rc);
	return rc;
    }

    /* Initiate a host stop procedure. */
    err = ble_hs_stop(&stop_listener, ble_hs_stop_cb,
                     NULL);
    if (err != 0) {
        ble_npl_sem_deinit(&ble_hs_stop_sem);
        return err;
    }

    /* Wait till the host stop procedure is complete */
    ble_npl_sem_pend(&ble_hs_stop_sem, BLE_NPL_TIME_FOREVER);

    ble_npl_event_init(&ble_hs_ev_stop, nimble_port_stop_cb,
                       NULL);
    ble_npl_eventq_put(&g_eventq_dflt, &ble_hs_ev_stop);

    /* Wait till the event is serviced */
    ble_npl_sem_pend(&ble_hs_stop_sem, BLE_NPL_TIME_FOREVER);

    ble_npl_sem_deinit(&ble_hs_stop_sem);

    ble_npl_event_deinit(&ble_hs_ev_stop);

    return ESP_OK;
}

#if !MYNEWT_VAL(BLE_LOW_SPEED_MODE)
IRAM_ATTR
#endif
void nimble_port_run(void)
{
    struct ble_npl_event *ev;

    while (1) {
        ev = ble_npl_eventq_get(&g_eventq_dflt, BLE_NPL_TIME_FOREVER);
        if (ev) {
            ble_npl_event_run(ev);
            if (ev == &ble_hs_ev_stop) {
                break;
            }
        }
    }
}

#if !MYNEWT_VAL(BLE_LOW_SPEED_MODE)
IRAM_ATTR
#endif
struct ble_npl_eventq *
nimble_port_get_dflt_eventq(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    /* If context exists, return the real eventq */
    if (ble_npl_ctx != NULL) {
        return &g_eventq_dflt;
    }

    /* Context is NULL - either not yet allocated or already freed during deinit.
     * If we're in shutdown, return the static fallback to prevent crashes.
     * The fallback eventq is not functional but callers can safely reference it. */
    if (ble_npl_deiniting) {
        return &g_eventq_shutdown_fallback;
    }

    /* Normal case: try to allocate context */
    if (ble_npl_ensure_ctx()) {
        return NULL;
    }
#endif

    return &g_eventq_dflt;
}
