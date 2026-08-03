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

#include <string.h>
#include <errno.h>
#include "syscfg/syscfg.h"
#include "os/os.h"
#include "host/ble_hs_id.h"
#include "ble_hs_priv.h"
#include "host/ble_hs_log.h"

#if MYNEWT_VAL(BLE_PERIODIC_ADV)
static SLIST_HEAD(, ble_hs_periodic_sync) g_ble_hs_periodic_sync_handles;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#include "esp_nimble_mem.h"
static struct
{
    struct os_mempool pool;
    os_membuf_t *elem_mem;
} *ble_hs_periodic_ctx;
#define ble_hs_periodic_sync_pool (ble_hs_periodic_ctx->pool)
#define ble_hs_psync_elem_mem (ble_hs_periodic_ctx->elem_mem)
#else
static struct os_mempool ble_hs_periodic_sync_pool;
#endif

#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
static os_membuf_t *ble_hs_psync_elem_mem = NULL;
#else
static os_membuf_t ble_hs_psync_elem_mem[
    OS_MEMPOOL_SIZE(MYNEWT_VAL(BLE_MAX_PERIODIC_SYNCS),
                    sizeof (struct ble_hs_periodic_sync))
];
#endif
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
static int
ble_hs_periodic_sync_ensure_ctx(void)
{
    if (ble_hs_periodic_ctx != NULL)
    {
        return 0;
    }

    ble_hs_periodic_ctx = nimble_platform_mem_calloc(1, sizeof(*ble_hs_periodic_ctx));
    return ble_hs_periodic_ctx ? 0 : BLE_HS_ENOMEM;
}
#endif

/**
 * Allocates an entry from the periodic sync pool.
 *
 * @return                      The allocated entry on success;
 *                              NULL on failure.
 *
 * @note This function expects the host lock to be held.
 */
struct ble_hs_periodic_sync *
ble_hs_periodic_sync_alloc(void)
{
    struct ble_hs_periodic_sync *psync;

    psync = os_memblock_get(&ble_hs_periodic_sync_pool);
    if (psync) {
        memset(psync, 0, sizeof(*psync));
    }

    return psync;
}

/**
 * Frees an entry to the periodic sync pool.
 *
 * @param psync                 The entry to free.
 *
 * @note This function expects the host lock to be held.
 */
void
ble_hs_periodic_sync_free(struct ble_hs_periodic_sync *psync)
{
    int rc;

    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    if (psync == NULL) {
        return;
    }

    /* Remove the event from the host event queue before deinitializing it.
     * If psync is freed while its lost_ev is still pending (e.g., via
     * ble_hs_atomic_conn_delete or the sync_receive disable path), the host
     * task would later dispatch the event and dereference freed memory. */
    if (ble_npl_event_is_queued(&psync->lost_ev)) {
        ble_npl_eventq_remove(ble_hs_evq_get(), &psync->lost_ev);
    }
    ble_npl_event_deinit(&psync->lost_ev);

#if MYNEWT_VAL(BLE_HS_DEBUG)
    memset(psync, 0xff, sizeof *psync);
#endif
    rc = os_memblock_put(&ble_hs_periodic_sync_pool, psync);
    BLE_HS_DBG_ASSERT_EVAL(rc == 0);
}

void
ble_hs_periodic_sync_insert(struct ble_hs_periodic_sync *psync)
{
    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    BLE_HS_DBG_ASSERT_EVAL(
                           ble_hs_periodic_sync_find_by_handle(psync->sync_handle) == NULL);

    SLIST_INSERT_HEAD(&g_ble_hs_periodic_sync_handles, psync, next);
}

void
ble_hs_periodic_sync_remove(struct ble_hs_periodic_sync *psync)
{
    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    SLIST_REMOVE(&g_ble_hs_periodic_sync_handles, psync, ble_hs_periodic_sync,
                 next);
}

struct ble_hs_periodic_sync *
ble_hs_periodic_sync_find_by_handle(uint16_t sync_handle)
{
    struct ble_hs_periodic_sync *psync;

    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    SLIST_FOREACH(psync, &g_ble_hs_periodic_sync_handles, next) {
        if (psync->sync_handle == sync_handle) {
            return psync;
        }
    }

    return NULL;
}

struct ble_hs_periodic_sync *
ble_hs_periodic_sync_find(const ble_addr_t *addr, uint8_t sid)
{
    struct ble_hs_periodic_sync *psync;
    ble_addr_t norm_addr;

    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    if (!addr) {
        return NULL;
    }

    /* Normalize identity address types so that BLE_ADDR_PUBLIC_ID matches
     * BLE_ADDR_PUBLIC and BLE_ADDR_RANDOM_ID matches BLE_ADDR_RANDOM. Without
     * this, a caller passing an identity-type address fails to find a sync
     * that was stored with the corresponding base type, and vice-versa.
     * Note: full RPA-to-identity resolution is not performed here; a lookup
     * using an unresolved RPA will still miss a sync stored by identity
     * address. */
    norm_addr = *addr;
    if (norm_addr.type == BLE_ADDR_PUBLIC_ID) {
        norm_addr.type = BLE_ADDR_PUBLIC;
    } else if (norm_addr.type == BLE_ADDR_RANDOM_ID) {
        norm_addr.type = BLE_ADDR_RANDOM;
    }

    SLIST_FOREACH(psync, &g_ble_hs_periodic_sync_handles, next) {
        ble_addr_t stored = psync->advertiser_addr;
        if (stored.type == BLE_ADDR_PUBLIC_ID) {
            stored.type = BLE_ADDR_PUBLIC;
        } else if (stored.type == BLE_ADDR_RANDOM_ID) {
            stored.type = BLE_ADDR_RANDOM;
        }
        if ((ble_addr_cmp(&stored, &norm_addr) == 0) && (psync->adv_sid == sid)) {
            return psync;
        }
    }

    return NULL;
}

/**
 * Retrieves the first periodic discovery handle in the list.
 *
 * @note The returned pointer is only valid while the host lock is held.
 *       The caller should hold the lock or use the handle immediately
 *       with caution (TOCTOU risk).
 */
struct ble_hs_periodic_sync *
ble_hs_periodic_sync_first(void)
{
    struct ble_hs_periodic_sync *psync;

    ble_hs_lock();
    psync = SLIST_FIRST(&g_ble_hs_periodic_sync_handles);
    ble_hs_unlock();

    return psync;
}

struct ble_hs_periodic_sync *
ble_hs_periodic_sync_first_locked(void)
{
    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());
    return SLIST_FIRST(&g_ble_hs_periodic_sync_handles);
}

#if MYNEWT_VAL(BLE_PERIODIC_ADV_WITH_RESPONSES)
/**
 * Find first PAwR sync (num_subevents > 0) in the active sync list.
 *
 * Regular periodic syncs share the same list; callers that need a PAwR
 * association must not use ble_hs_periodic_sync_first_locked().
 *
 * @note Host lock must be held.
 */
struct ble_hs_periodic_sync *
ble_hs_periodic_sync_find_pawr_locked(void)
{
    struct ble_hs_periodic_sync *psync;

    BLE_HS_DBG_ASSERT(ble_hs_locked_by_cur_task());

    SLIST_FOREACH(psync, &g_ble_hs_periodic_sync_handles, next) {
        if (psync->num_subevents > 0) {
            return psync;
        }
    }

    return NULL;
}
#endif

void
ble_hs_periodic_sync_deinit(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_periodic_ctx == NULL)
    {
        return;
    }
#if !MYNEWT_VAL(MP_RUNTIME_ALLOC)
    if (ble_hs_psync_elem_mem)
    {
        nimble_platform_mem_free(ble_hs_psync_elem_mem);
        ble_hs_psync_elem_mem = NULL;
    }
#endif
    os_mempool_unregister(&ble_hs_periodic_sync_pool);
    memset(&ble_hs_periodic_sync_pool, 0, sizeof(ble_hs_periodic_sync_pool));
    nimble_platform_mem_free(ble_hs_periodic_ctx);
    ble_hs_periodic_ctx = NULL;
#else
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    if (ble_hs_psync_elem_mem)
    {
        nimble_platform_mem_free(ble_hs_psync_elem_mem);
        ble_hs_psync_elem_mem = NULL;
    }
#endif
#endif
}

int
ble_hs_periodic_sync_init(void)
{
    int rc;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    rc = ble_hs_periodic_sync_ensure_ctx();
    if (rc != 0) {
        return rc;
    }
#endif

    if (MYNEWT_VAL(BLE_MAX_PERIODIC_SYNCS) == 0) {
        memset(&ble_hs_periodic_sync_pool, 0, sizeof(ble_hs_periodic_sync_pool));
        rc = os_mempool_init(&ble_hs_periodic_sync_pool, 0,
                             sizeof(struct ble_hs_periodic_sync), NULL,
                             "ble_hs_periodic_disc_pool");
        if (rc != 0) {
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EOS);
            return BLE_HS_EOS;
        }
        SLIST_INIT(&g_ble_hs_periodic_sync_handles);
        return 0;
    }

#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC) && MYNEWT_VAL(MP_RUNTIME_ALLOC)
    if (!ble_hs_psync_elem_mem) {
        ble_hs_psync_elem_mem = nimble_platform_mem_calloc(1,
                OS_MEMPOOL_SIZE(MYNEWT_VAL(BLE_MAX_PERIODIC_SYNCS),
                                sizeof(struct ble_hs_periodic_sync)) *
                sizeof(os_membuf_t));
    }
    if (!ble_hs_psync_elem_mem) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ENOMEM);
        return BLE_HS_ENOMEM;
    }
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#if !MYNEWT_VAL(MP_RUNTIME_ALLOC)
    if (!ble_hs_psync_elem_mem)
    {
        ble_hs_psync_elem_mem = nimble_platform_mem_calloc(1,
                                                           OS_MEMPOOL_SIZE(
                                                               MYNEWT_VAL(BLE_MAX_PERIODIC_SYNCS),
                                                               sizeof(struct ble_hs_periodic_sync)) *
                                                               sizeof(os_membuf_t));
    }

    if (ble_hs_psync_elem_mem == NULL)
    {
        if (ble_hs_periodic_ctx) {
            nimble_platform_mem_free(ble_hs_periodic_ctx);
            ble_hs_periodic_ctx = NULL;
        }
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ENOMEM);
        return BLE_HS_ENOMEM;
    }
#endif
#endif

    memset(&ble_hs_periodic_sync_pool, 0, sizeof(ble_hs_periodic_sync_pool));

    rc = os_mempool_init(&ble_hs_periodic_sync_pool,
                         MYNEWT_VAL(BLE_MAX_PERIODIC_SYNCS),
                         sizeof(struct ble_hs_periodic_sync),
                         ble_hs_psync_elem_mem, "ble_hs_periodic_disc_pool");
    if (rc != 0) {
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        if (ble_hs_periodic_ctx)
        {
#if !MYNEWT_VAL(MP_RUNTIME_ALLOC)
            if (ble_hs_psync_elem_mem)
            {
                nimble_platform_mem_free(ble_hs_psync_elem_mem);
                ble_hs_psync_elem_mem = NULL;
            }
#endif
            os_mempool_unregister(&ble_hs_periodic_sync_pool);
            nimble_platform_mem_free(ble_hs_periodic_ctx);
            ble_hs_periodic_ctx = NULL;
        }
#elif MYNEWT_VAL(MP_RUNTIME_ALLOC)
        if (ble_hs_psync_elem_mem)
        {
            nimble_platform_mem_free(ble_hs_psync_elem_mem);
            ble_hs_psync_elem_mem = NULL;
        }
#endif
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EOS);
        return BLE_HS_EOS;
    }

    SLIST_INIT(&g_ble_hs_periodic_sync_handles);

    return 0;
}
#endif
