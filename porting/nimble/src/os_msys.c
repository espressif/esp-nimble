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

#include <assert.h>
#include "os/os.h"
#include "mem/mem.h"
#include "sysinit/sysinit.h"
#include "esp_nimble_mem.h"
#include "esp_err.h"

static STAILQ_HEAD(, os_mbuf_pool) g_msys_pool_list =
    STAILQ_HEAD_INITIALIZER(g_msys_pool_list);

#if CONFIG_BT_NIMBLE_ENABLED
#define OS_MSYS_1_BLOCK_COUNT MYNEWT_VAL(MSYS_1_BLOCK_COUNT)
#define OS_MSYS_1_BLOCK_SIZE MYNEWT_VAL(MSYS_1_BLOCK_SIZE)
#define OS_MSYS_2_BLOCK_COUNT MYNEWT_VAL(MSYS_2_BLOCK_COUNT)
#define OS_MSYS_2_BLOCK_SIZE MYNEWT_VAL(MSYS_2_BLOCK_SIZE)
#else
#define OS_MSYS_1_BLOCK_COUNT CONFIG_BT_LE_MSYS_1_BLOCK_COUNT
#define OS_MSYS_1_BLOCK_SIZE CONFIG_BT_LE_MSYS_1_BLOCK_SIZE
#define OS_MSYS_2_BLOCK_COUNT CONFIG_BT_LE_MSYS_2_BLOCK_COUNT
#define OS_MSYS_2_BLOCK_SIZE CONFIG_BT_LE_MSYS_2_BLOCK_SIZE
#endif



#if OS_MSYS_1_BLOCK_COUNT > 0
#define SYSINIT_MSYS_1_MEMBLOCK_SIZE                \
    OS_ALIGN(OS_MSYS_1_BLOCK_SIZE, 4)
#define SYSINIT_MSYS_1_MEMPOOL_SIZE                 \
    OS_MEMPOOL_SIZE(OS_MSYS_1_BLOCK_COUNT,  \
                    SYSINIT_MSYS_1_MEMBLOCK_SIZE)
#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
static os_membuf_t *os_msys_init_1_data;
static struct os_mbuf_pool os_msys_init_1_mbuf_pool;
static struct os_mempool os_msys_init_1_mempool;
#endif // BLE_STATIC_TO_DYNAMIC
#endif // OS_MSYS_1_BLOCK_COUNT

#if OS_MSYS_2_BLOCK_COUNT > 0
#define SYSINIT_MSYS_2_MEMBLOCK_SIZE                \
    OS_ALIGN(OS_MSYS_2_BLOCK_SIZE, 4)
#define SYSINIT_MSYS_2_MEMPOOL_SIZE                 \
    OS_MEMPOOL_SIZE(OS_MSYS_2_BLOCK_COUNT,  \
                    SYSINIT_MSYS_2_MEMBLOCK_SIZE)
#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
static os_membuf_t *os_msys_init_2_data;
static struct os_mbuf_pool os_msys_init_2_mbuf_pool;
static struct os_mempool os_msys_init_2_mempool;
#endif // BLE_STATIC_TO_DYNAMIC
#endif // OS_MSYS_2_BLOCK_COUNT

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
/* Context structure holding all MSYS resources */
typedef struct {
#if OS_MSYS_1_BLOCK_COUNT > 0
    os_membuf_t *init_1_data;
    struct os_mbuf_pool init_1_mbuf_pool;
    struct os_mempool   init_1_mempool;
#endif

#if OS_MSYS_2_BLOCK_COUNT > 0
    os_membuf_t *init_2_data;
    struct os_mbuf_pool init_2_mbuf_pool;
    struct os_mempool   init_2_mempool;
#endif
} os_msys_ctx_t;

static os_msys_ctx_t *os_msys_ctx = NULL;

/* Macros for easier access */
#if OS_MSYS_1_BLOCK_COUNT > 0
#define os_msys_init_1_data       (os_msys_ctx->init_1_data)
#define os_msys_init_1_mbuf_pool  (os_msys_ctx->init_1_mbuf_pool)
#define os_msys_init_1_mempool    (os_msys_ctx->init_1_mempool)
#endif // OS_MSYS_1_BLOCK_COUNT

#if OS_MSYS_2_BLOCK_COUNT > 0
#define os_msys_init_2_data       (os_msys_ctx->init_2_data)
#define os_msys_init_2_mbuf_pool  (os_msys_ctx->init_2_mbuf_pool)
#define os_msys_init_2_mempool    (os_msys_ctx->init_2_mempool)
#endif // OS_MSYS_2_BLOCK_COUNT

static int
ble_os_msys_ensure_ctx(void)
{
    if(os_msys_ctx) {
        return 0;
    }

    os_msys_ctx = nimble_platform_mem_calloc(1, sizeof(*os_msys_ctx));
    if(!os_msys_ctx) {
        return -1;
    }

    return 0;
}

#endif // BLE_STATIC_TO_DYNAMIC

#define OS_MSYS_SANITY_ENABLED                  \
    (MYNEWT_VAL(MSYS_1_SANITY_MIN_COUNT) > 0 || \
     MYNEWT_VAL(MSYS_2_SANITY_MIN_COUNT) > 0)

#if OS_MSYS_SANITY_ENABLED
static struct os_sanity_check os_msys_sc;
#endif

#if OS_MSYS_SANITY_ENABLED

/**
 * Retrieves the minimum safe buffer count for an msys pool.  That is, the
 * lowest a pool's buffer count can be without causing the sanity check to
 * fail.
 *
 * @param idx                   The index of the msys pool to query.
 *
 * @return                      The msys pool's minimum safe buffer count.
 */
#if !MYNEWT_VAL(BLE_LOW_SPEED_MODE)
IRAM_ATTR
#endif
static int
os_msys_sanity_min_count(int idx)
{
    switch (idx) {
    case 0:
        return MYNEWT_VAL(MSYS_1_SANITY_MIN_COUNT);

    case 1:
        return MYNEWT_VAL(MSYS_2_SANITY_MIN_COUNT);

    default:
        BLE_LL_ASSERT(0);
        return ESP_OK;
    }
}

#if !MYNEWT_VAL(BLE_LOW_SPEED_MODE)
IRAM_ATTR
#endif
static int
os_msys_sanity(struct os_sanity_check *sc, void *arg)
{
    const struct os_mbuf_pool *omp;
    int min_count;
    int idx;

    idx = 0;
    STAILQ_FOREACH(omp, &g_msys_pool_list, omp_next) {
        min_count = os_msys_sanity_min_count(idx);
        if (omp->omp_pool->mp_num_free < min_count) {
            return OS_ENOMEM;
        }

        idx++;
    }

    return ESP_OK;
}
#endif

static void
os_msys_init_once(void *data, struct os_mempool *mempool,
                  struct os_mbuf_pool *mbuf_pool,
                  int block_count, int block_size, const char *name)
{
    int rc;

    rc = mem_init_mbuf_pool(data, mempool, mbuf_pool, block_count, block_size,
                            name);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_msys_register(mbuf_pool);
    SYSINIT_PANIC_ASSERT(rc == 0);
}

int
os_msys_buf_alloc(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_os_msys_ensure_ctx()){
        return ESP_FAIL;
    }
#endif

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    return ESP_OK;
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#if OS_MSYS_1_BLOCK_COUNT > 0
        if (!os_msys_ctx->init_1_data) {
            os_msys_ctx->init_1_data = nimble_platform_mem_calloc(1, (sizeof(os_membuf_t) * SYSINIT_MSYS_1_MEMPOOL_SIZE));
            if(!os_msys_ctx->init_1_data){
                nimble_platform_mem_free(os_msys_ctx);
                os_msys_ctx = NULL;
                return ESP_FAIL;
            }
        }
#endif

#if OS_MSYS_2_BLOCK_COUNT > 0
      if (!os_msys_ctx->init_2_data) {
          os_msys_ctx->init_2_data = nimble_platform_mem_calloc(1, (sizeof(os_membuf_t) * SYSINIT_MSYS_2_MEMPOOL_SIZE));
          if(!os_msys_ctx->init_2_data) {
#if OS_MSYS_1_BLOCK_COUNT > 0
              nimble_platform_mem_free(os_msys_ctx->init_1_data);
              os_msys_ctx->init_1_data = NULL;
#endif
              nimble_platform_mem_free(os_msys_ctx);
              os_msys_ctx = NULL;
              return ESP_FAIL;
          }
      }

#endif
#else
#if OS_MSYS_1_BLOCK_COUNT > 0
    os_msys_init_1_data = (os_membuf_t *)nimble_platform_mem_calloc(1, (sizeof(os_membuf_t) * SYSINIT_MSYS_1_MEMPOOL_SIZE));
    if (!os_msys_init_1_data) {
        return ESP_FAIL;
    }
#endif

#if OS_MSYS_2_BLOCK_COUNT > 0
    os_msys_init_2_data = (os_membuf_t *)nimble_platform_mem_calloc(1, (sizeof(os_membuf_t) * SYSINIT_MSYS_2_MEMPOOL_SIZE));
    if (!os_msys_init_2_data) {
#if OS_MSYS_1_BLOCK_COUNT > 0
       nimble_platform_mem_free(os_msys_init_1_data);
       os_msys_init_1_data = NULL;
#endif
        return ESP_FAIL;
    }
#endif
#endif // BLE_STATIC_TO_DYNAMIC
    return ESP_OK;
}

void
os_msys_buf_free(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (os_msys_ctx) {
#if OS_MSYS_1_BLOCK_COUNT > 0
        if (os_msys_ctx->init_1_data) {
            nimble_platform_mem_free(os_msys_ctx->init_1_data);
            os_msys_ctx->init_1_data = NULL;
        }
#endif
#if OS_MSYS_2_BLOCK_COUNT > 0
        if (os_msys_ctx->init_2_data) {
            nimble_platform_mem_free(os_msys_ctx->init_2_data);
            os_msys_ctx->init_2_data = NULL;
        }
#endif
        nimble_platform_mem_free(os_msys_ctx);
        os_msys_ctx = NULL;
    }

#else
#if OS_MSYS_1_BLOCK_COUNT > 0

    nimble_platform_mem_free(os_msys_init_1_data);
    os_msys_init_1_data = NULL;
#endif

#if OS_MSYS_2_BLOCK_COUNT > 0
    nimble_platform_mem_free(os_msys_init_2_data);
    os_msys_init_2_data = NULL;
#endif
#endif
}

void os_msys_init(void)
{
#if OS_MSYS_SANITY_ENABLED
    int rc;
#endif

    os_msys_reset();

#if OS_MSYS_1_BLOCK_COUNT > 0
    os_msys_init_once(os_msys_init_1_data,
                      &os_msys_init_1_mempool,
                      &os_msys_init_1_mbuf_pool,
                      OS_MSYS_1_BLOCK_COUNT,
                      SYSINIT_MSYS_1_MEMBLOCK_SIZE,
                      "msys_1");
#endif

#if OS_MSYS_2_BLOCK_COUNT > 0
    os_msys_init_once(os_msys_init_2_data,
                      &os_msys_init_2_mempool,
                      &os_msys_init_2_mbuf_pool,
                      OS_MSYS_2_BLOCK_COUNT,
                      SYSINIT_MSYS_2_MEMBLOCK_SIZE,
                      "msys_2");
#endif

#if OS_MSYS_SANITY_ENABLED
    os_msys_sc.sc_func = os_msys_sanity;
    os_msys_sc.sc_checkin_itvl =
        OS_TICKS_PER_SEC * MYNEWT_VAL(MSYS_SANITY_TIMEOUT) / 1000;
    rc = os_sanity_check_register(&os_msys_sc);
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif
}
