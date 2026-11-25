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

static bool nimble_port_run_active;
static bool nimble_port_stopping;   /* Set while a stop procedure is in progress. */

extern void os_msys_init(void);
extern void os_mempool_module_init(void);

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
    /* Propagate the host stop status to the waiter, so a failed or timed-out
     * stop is not reported back to the caller as success. */
    if (arg != NULL) {
        *(int *)arg = status;
    }
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
    /* Initialize the global memory pool */
    os_mempool_module_init();

    /* Initialize the function pointers for OS porting */
    npl_freertos_funcs_init();

    if (npl_freertos_mempool_init() != 0) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "npl mempool init failed\n");
        npl_freertos_funcs_deinit();
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        if (ble_npl_ctx) {
            nimble_platform_mem_free(ble_npl_ctx);
            ble_npl_ctx = NULL;
        }
#endif
        return ESP_FAIL;
    }

#if CONFIG_BT_CONTROLLER_ENABLED
    if(esp_nimble_hci_init() != ESP_OK) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "hci inits failed\n");
        /* Unwind the NPL state brought up above, so that a retried
         * nimble_port_init() does not re-run npl_freertos_mempool_init() and
         * orphan the previously allocated pool buffers. */
        npl_freertos_mempool_deinit();
        npl_freertos_funcs_deinit();
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        if (ble_npl_ctx) {
            nimble_platform_mem_free(ble_npl_ctx);
            ble_npl_ctx = NULL;
        }
#endif
        return ESP_FAIL;
    }
#else
    esp_err_t ret;
    ret = ble_buf_alloc();
    if (ret != ESP_OK) {
        ble_buf_free();
        return ESP_FAIL;
    }
    ble_transport_init();
#endif

    /* Initialize default event queue */
    ble_npl_eventq_init(&g_eventq_dflt);
    os_msys_init();
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
    ble_adv_list_init();
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
        ble_npl_deiniting = false;  /* Reset flag before early return */
#endif
        return ESP_FAIL;
    }
#else
    ble_transport_deinit();
    ble_buf_free();
#endif

#endif

    ble_hs_deinit();

    ble_npl_eventq_deinit(&g_eventq_dflt);

#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
    npl_freertos_funcs_deinit();
#endif

#if !SOC_ESP_NIMBLE_CONTROLLER || !CONFIG_BT_CONTROLLER_ENABLED
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

#if CONFIG_BT_PRF_TASK_ENABLED
    /* Stop the shared BLE profile task while the bt_osal table is still up,
     * symmetrically with the init in esp_nimble_init(). */
    bt_prf_task_deinit();
#endif

    /* Release the bt_osal function table set up in esp_nimble_init(). */
    bt_osal_freertos_funcs_deinit();

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    os_mempool_deinit_all(false);
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

    return ESP_OK;
}


int
nimble_port_stop(void)
{
    esp_err_t err = ESP_OK;
    ble_npl_error_t rc;
    int stop_status = 0;
    uint32_t ctx;

    /* A call from the host task itself would deadlock: that task is the only
     * one that services the stop event queued below, so it cannot both block
     * here and release us. Reject it instead of hanging forever.
     *
     * ESP-IDF stops from a non-host task (e.g. app_main) after sync; a host-task
     * stop before ble_hs_start() is not a supported API. */
    if (ble_hs_is_parent_task()) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG,
                 "nimble_port_stop() called from the host task; would deadlock");
        return ESP_FAIL;
    }

    /* Only one stop procedure may run at a time. Concurrent callers share one
     * semaphore and one event, so a second caller would pend on a release that
     * never comes. Serialise with a race-free test-and-set. */
    ctx = ble_npl_hw_enter_critical();
    if (nimble_port_stopping) {
        ble_npl_hw_exit_critical(ctx);
        ESP_LOGW(NIMBLE_PORT_LOG_TAG, "nimble_port_stop() already in progress");
        return ESP_FAIL;
    }
    nimble_port_stopping = true;
    ble_npl_hw_exit_critical(ctx);

    rc = ble_npl_sem_init(&ble_hs_stop_sem, 0);

    if( rc != 0) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "sem init failed with reason: %d \n", rc);
        err = rc;
        goto out;
    }

    /* Initiate a host stop procedure. */
    err = ble_hs_stop(&stop_listener, ble_hs_stop_cb,
                     &stop_status);
    if (err != 0) {
        if (err != BLE_HS_EALREADY) {
            ble_npl_sem_deinit(&ble_hs_stop_sem);
            goto out;
        }
        if (!nimble_port_run_active) {
            ble_npl_sem_deinit(&ble_hs_stop_sem);
            err = ESP_OK;
            goto out;
        }
    } else {
        /* Wait till the host stop procedure is complete */
        ble_npl_sem_pend(&ble_hs_stop_sem, BLE_NPL_TIME_FOREVER);

        if (stop_status != 0) {
            /* The stop procedure failed (e.g. a connection outlived the
             * graceful-disconnect timeout) and the host has reverted to the
             * enabled state. Keep the host task running: stopping it here
             * would leave an enabled host with no event loop, so a caller
             * that skips deinit on failure could never retry (the retried
             * stop would pend on events no task services). */
            ble_npl_sem_deinit(&ble_hs_stop_sem);
            err = ESP_FAIL;
            goto out;
        }
    }

    ble_npl_event_init(&ble_hs_ev_stop, nimble_port_stop_cb,
                       NULL);
    ble_npl_eventq_put(&g_eventq_dflt, &ble_hs_ev_stop);

    /* Wait till the host task services and releases the stop event. */
    ble_npl_sem_pend(&ble_hs_stop_sem, BLE_NPL_TIME_FOREVER);

    ble_npl_sem_deinit(&ble_hs_stop_sem);

    err = ESP_OK;

out:
    nimble_port_stopping = false;
    return err;
}

#if !MYNEWT_VAL(BLE_LOW_SPEED_MODE)
IRAM_ATTR
#endif
void nimble_port_run(void)
{
    struct ble_npl_event *ev;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_npl_ctx == NULL) {
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "nimble context not initialized, host task exiting");
        return;
    }
#endif
#if CONFIG_BT_DUAL_MODE_ARCH
    if (g_eventq_dflt.eventq.eventq == NULL) {
#else
    if (g_eventq_dflt.eventq == NULL) {
#endif
        ESP_LOGE(NIMBLE_PORT_LOG_TAG, "nimble event queue not initialized, host task exiting");
        return;
    }

    /* Snapshot stop_ev and stop_sem addresses before entering the loop.
     * When BLE_STATIC_TO_DYNAMIC is enabled, after ble_npl_sem_release(stop_sem)
     * at the end of stop-event handling, the caller of nimble_port_stop() can
     * wake up and free ble_npl_ctx. Any subsequent use of the &ble_hs_ev_stop
     * or &ble_hs_stop_sem macros (which dereference ble_npl_ctx) would be a
     * use-after-free. Using local snapshots avoids that.
     */
    struct ble_npl_event *stop_ev  = &ble_hs_ev_stop;
    struct ble_npl_sem  *stop_sem  = &ble_hs_stop_sem;

    nimble_port_run_active = true;
    while (1) {
        ev = ble_npl_eventq_get(&g_eventq_dflt, BLE_NPL_TIME_FOREVER);
        if (ev) {
            ble_npl_event_run(ev);

            /* Re-post a latched host reset after each event. */
            ble_hs_sched_reset_retry();

            if (ev == stop_ev) {
                ble_npl_event_deinit(stop_ev);
                nimble_port_run_active = false;
                ble_npl_sem_release(stop_sem);
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
    /* Check deiniting first: context may still be non-NULL during the deinit
     * window after ble_npl_deiniting is set but before ble_npl_ctx is freed.
     * Return the fallback to prevent callers from using a deinitialized queue. */
    if (ble_npl_deiniting) {
        return &g_eventq_shutdown_fallback;
    }

    /* If context exists, return the real eventq */
    if (ble_npl_ctx != NULL) {
        return &g_eventq_dflt;
    }

    /* Normal case: try to allocate context */
    if (ble_npl_ensure_ctx()) {
        return NULL;
    }
#endif

    return &g_eventq_dflt;
}
