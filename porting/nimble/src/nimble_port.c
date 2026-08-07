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
#include "esp_nimble_cfg.h"

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
#include "bt_common.h"
#if (BT_HCI_LOG_INCLUDED == TRUE)
#include "hci_log/bt_hci_log.h"
#endif
#include "bt_osal.h"
#include "bt_prf_task.h"
#if CONFIG_BLE_LOG_ENABLED && !CONFIG_BT_CONTROLLER_ENABLED
#include "ble_log.h"
#endif

#define NIMBLE_PORT_LOG_TAG          "BLE_INIT"

extern void os_msys_init(void);
extern void os_msys_buf_free(void);

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
extern void os_msys_buf_free(void);
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

static void
ble_npl_free_ctx(void)
{
    if (ble_npl_ctx != NULL) {
        nimble_platform_mem_free(ble_npl_ctx);
        ble_npl_ctx = NULL;
    }
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
    (void)ev;
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

#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
    int rc;
    /* Initialize the global memory pool */
    os_mempool_module_init();

    /* Initialize the function pointers for OS porting */
    rc = npl_freertos_funcs_init();
    if (rc != 0) {
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        ble_npl_free_ctx();
#endif
        return ESP_ERR_NO_MEM;
    }

    if (npl_freertos_mempool_init() != 0) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "mempool init failed\n");
        npl_freertos_funcs_deinit();
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        ble_npl_free_ctx();
#endif
        return ESP_FAIL;
    }

#if CONFIG_BT_CONTROLLER_ENABLED
    if(esp_nimble_hci_init() != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "hci inits failed\n");
        npl_freertos_mempool_deinit();
        npl_freertos_funcs_deinit();
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        ble_npl_free_ctx();
#endif
        return ESP_FAIL;
    }
    /* Initialize default event queue before ble_hs_init() posts start event. */
    ble_npl_eventq_init(&g_eventq_dflt);
    os_msys_init();
#else
    esp_err_t ret = ESP_OK;

    ret = ble_buf_alloc();
    if (ret != ESP_OK) {
        ble_buf_free();
        npl_freertos_mempool_deinit();
        npl_freertos_funcs_deinit();
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        ble_npl_free_ctx();
#endif
        return ESP_FAIL;
    }
    ble_transport_init();
    /* Initialize default event queue */
    ble_npl_eventq_init(&g_eventq_dflt);
    os_msys_init();
#endif
#else
    /* Initialize default event queue */
    ble_npl_eventq_init(&g_eventq_dflt);
#endif // !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED

    /* Bring up the host-agnostic bt_osal function table before host bring-up; */
    bt_osal_freertos_funcs_init();

#if CONFIG_BT_PRF_TASK_ENABLED
    /* Start the shared BLE profile task now that the bt_osal table is up, so
     * profiles can post work without spawning their own tasks. */
    if (bt_prf_task_init() != BT_OSAL_OK) {
        ESP_LOGW(NIMBLE_PORT_LOG_TAG, "bt_prf_task_init failed");
    }
#endif

#if MYNEWT_VAL(BLE_QUEUE_CONG_CHECK)
    if (ble_adv_list_init() != 0) {
#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
#if CONFIG_BT_CONTROLLER_ENABLED
        esp_nimble_hci_deinit();
        os_msys_buf_free();
#else
        ble_transport_deinit();
        ble_buf_free();
#endif
        ble_npl_eventq_deinit(&g_eventq_dflt);
        npl_freertos_mempool_deinit();
        npl_freertos_funcs_deinit();
#else
        ble_npl_eventq_deinit(&g_eventq_dflt);
#endif
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        ble_npl_free_ctx();
#endif
        return ESP_FAIL;
    }
#endif

    /* Initialize the host */
    ble_transport_hs_init();
    
    ble_transport_ll_init();

#if (BT_HCI_LOG_INCLUDED == TRUE)
    if (bt_hci_log_init() != ESP_OK) {
        ESP_LOGW(NIMBLE_PORT_LOG_TAG, "bt_hci_log_init failed");
    }
#endif

#if CONFIG_BLE_LOG_ENABLED && !CONFIG_BT_CONTROLLER_ENABLED
    if (!ble_log_init()) {
        ESP_LOGW(NIMBLE_PORT_LOG_TAG, "BLE Log init failed");
    }
#endif

    return ESP_OK;
}

/**
 * @brief esp_nimble_deinit - Deinitialize the NimBLE host stack
 * 
 * @return esp_err_t 
 */
esp_err_t esp_nimble_deinit(void)
{
    esp_err_t ret = ESP_OK;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_npl_ctx == NULL) {
        return ESP_OK;
    }
    ble_npl_deiniting = true;
#endif

#if CONFIG_BLE_LOG_ENABLED && !CONFIG_BT_CONTROLLER_ENABLED
    ble_log_deinit();
#endif

    ble_transport_ll_deinit();

#if (BT_HCI_LOG_INCLUDED == TRUE)
    bt_hci_log_deinit();
#endif

#if MYNEWT_VAL(BLE_QUEUE_CONG_CHECK)
    ble_adv_list_deinit();
#endif

#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
#if CONFIG_BT_CONTROLLER_ENABLED
    if(esp_nimble_hci_deinit() != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "hci deinit failed\n");
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        ble_npl_deiniting = false;
#endif
        return ESP_FAIL;
    }
#else
    ble_transport_deinit();
    ble_buf_free();
#endif

#endif

    ble_npl_eventq_deinit(&g_eventq_dflt);

    ble_hs_deinit();

#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
    npl_freertos_funcs_deinit();
#endif

#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
    npl_freertos_mempool_deinit();
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    ble_npl_free_ctx();
    /* Keep ble_npl_deiniting = true until nimble_port_deinit completes.
     * This prevents controller deinit from re-allocating ble_npl_ctx when it
     * calls nimble_port_get_dflt_eventq(). The flag will be reset at the START
     * of the next init cycle (in ble_npl_reset_deinit_flag). */

#endif

#if CONFIG_BT_PRF_TASK_ENABLED
    /* Stop the shared BLE profile task while the bt_osal table is still up,
     * symmetrically with the init in esp_nimble_init(). */
    bt_prf_task_deinit();
#endif

    /* Release the bt_osal function table set up in esp_nimble_init(). */
    bt_osal_freertos_funcs_deinit();

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    os_mempool_deinit();
#endif

    return ret;
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
    esp_err_t ret = ESP_OK;

    if(esp_nimble_deinit() != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "nimble host deinit failed\n");
        return ESP_FAIL;
    }

#if CONFIG_BT_CONTROLLER_ENABLED
    if(esp_bt_controller_disable() != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller disable failed\n");
        ret = ESP_FAIL;
    }

    if(esp_bt_controller_deinit() != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "controller deinit failed\n");
        ret = ESP_FAIL;
    }
#endif

    return ret;
}


int
nimble_port_stop(void)
{
    esp_err_t err = ESP_OK;
    ble_npl_error_t rc;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_npl_ctx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
#endif

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

    /* Wait till the host task services and releases the stop event. */
    ble_npl_sem_pend(&ble_hs_stop_sem, BLE_NPL_TIME_FOREVER);

    ble_npl_sem_deinit(&ble_hs_stop_sem);

    return ESP_OK;
}

#if !MYNEWT_VAL(BLE_LOW_SPEED_MODE)
IRAM_ATTR
#endif
void nimble_port_run(void)
{
    struct ble_npl_event *ev;
    /* Cache addresses before entering the loop. When BLE_STATIC_TO_DYNAMIC is
     * enabled these macros dereference ble_npl_ctx; caching prevents a race
     * where ble_npl_ctx is freed between the stop-event callback returning and
     * the loop-exit check below. */
    struct ble_npl_eventq * const evq = &g_eventq_dflt;
    struct ble_npl_event * const stop_ev = &ble_hs_ev_stop;

    while (1) {
        ev = ble_npl_eventq_get(evq, BLE_NPL_TIME_FOREVER);
        if (ev) {
            ble_npl_event_run(ev);
            if (ev == stop_ev) {
                ble_npl_event_deinit(&ble_hs_ev_stop);
                ble_npl_sem_release(&ble_hs_stop_sem);
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
        return &g_eventq_shutdown_fallback;
    }
#endif

    return &g_eventq_dflt;
}
