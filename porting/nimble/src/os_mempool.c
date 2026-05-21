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

#include "os/os.h"
#include "os/os_trace_api.h"

#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include "syscfg/syscfg.h"
#include "modlog/modlog.h"
#include "esp_nimble_mem.h"
#if !MYNEWT_VAL(OS_SYSVIEW_TRACE_MEMPOOL)
#define OS_TRACE_DISABLE_FILE_API
#endif

#define OS_MEM_TRUE_BLOCK_SIZE(bsize)   OS_ALIGN(bsize, OS_ALIGNMENT)
#if MYNEWT_VAL(OS_MEMPOOL_GUARD)
#define OS_MEMPOOL_TRUE_BLOCK_SIZE(mp)                                  \
    (((mp)->mp_flags & OS_MEMPOOL_F_EXT) ?                              \
      OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size) :                       \
      (OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size) + sizeof(os_membuf_t)))
#else
#define OS_MEMPOOL_TRUE_BLOCK_SIZE(mp) OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size)
#endif

#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
STAILQ_HEAD(, os_mempool) g_os_mempool_list = STAILQ_HEAD_INITIALIZER(g_os_mempool_list);
#else
STAILQ_HEAD(, os_mempool) g_os_mempool_list;
static bool g_os_mempool_list_inited;
#endif

#if MYNEWT_VAL(OS_MEMPOOL_POISON)
static uint32_t os_mem_poison = 0xde7ec7ed;

static_assert(sizeof(struct os_memblock) % 4 == 0, "sizeof(struct os_memblock) shall be aligned to 4");
static_assert(sizeof(os_mem_poison) == 4, "sizeof(os_mem_poison) shall be 4");

static void
os_mempool_poison(const struct os_mempool *mp, void *start)
{
    uint32_t *p;
    uint32_t *end;
    int sz;

    sz = OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size);
    p = start;
    end = p + sz / 4;
    p += sizeof(struct os_memblock) / 4;

    while (p < end) {
        *p = os_mem_poison;
        p++;
    }
}

static void
os_mempool_poison_check(const struct os_mempool *mp, void *start)
{
    uint32_t *p;
    uint32_t *end;
    int sz;

    sz = OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size);
    p = start;
    end = p + sz / 4;
    p += sizeof(struct os_memblock) / 4;

    while (p < end) {
        assert(*p == os_mem_poison);
        p++;
    }
}
#else
#define os_mempool_poison(mp, start)
#define os_mempool_poison_check(mp, start)
#endif
#if MYNEWT_VAL(OS_MEMPOOL_GUARD)
#define OS_MEMPOOL_GUARD_PATTERN 0xBAFF1ED1

static void
os_mempool_guard(const struct os_mempool *mp, void *start)
{
    uint32_t *tgt;

    if ((mp->mp_flags & OS_MEMPOOL_F_EXT) == 0) {
        tgt = (uint32_t *)((uintptr_t)start +
                                     OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size));
        *tgt = OS_MEMPOOL_GUARD_PATTERN;
    }
}

static void
os_mempool_guard_check(const struct os_mempool *mp, void *start)
{
    uint32_t *tgt;

    if ((mp->mp_flags & OS_MEMPOOL_F_EXT) == 0) {
        tgt = (uint32_t *)((uintptr_t)start +
                                     OS_MEM_TRUE_BLOCK_SIZE(mp->mp_block_size));
        assert(*tgt == OS_MEMPOOL_GUARD_PATTERN);
    }
}
#else
#define os_mempool_guard(mp, start)
#define os_mempool_guard_check(mp, start)
#endif

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
static void
os_mempool_list_ensure_init(void)
{
    if (!g_os_mempool_list_inited) {
        STAILQ_INIT(&g_os_mempool_list);
        g_os_mempool_list_inited = true;
    }
}
#endif

static os_error_t
os_mempool_init_internal(struct os_mempool *mp, uint16_t blocks,
                         uint32_t block_size, void *membuf, const char *name,
                         uint8_t flags)
{
    int true_block_size;
    int i;
    uint8_t *block_addr;
    struct os_memblock *block_ptr;

    /* Check for valid parameters */
    if (!mp || (block_size == 0)) {
        return OS_INVALID_PARM;
    }

    /* For runtime allocation mode, membuf can be NULL */
    #if !MYNEWT_VAL(MP_RUNTIME_ALLOC)
    if ((!membuf) && (blocks != 0)) {
        return OS_INVALID_PARM;
    }
    #endif

    if (membuf != NULL) {
        /* Blocks need to be sized properly and memory buffer should be
         * aligned
         */
        if (((uintptr_t)membuf & (OS_ALIGNMENT - 1)) != 0) {
            return OS_MEM_NOT_ALIGNED;
        }
    }

    /* Initialize the memory pool structure */
    mp->mp_block_size = block_size;
    mp->mp_num_free = blocks;
    mp->mp_min_free = blocks;
    mp->mp_flags = flags;
    mp->mp_num_blocks = blocks;
    mp->mp_membuf_addr = (uintptr_t)membuf;
    mp->name = name;
    SLIST_FIRST(mp) = membuf;

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    if (membuf == NULL) {
        /* Runtime allocation mode */
        mp->mp_membuf_addr = 0;
        mp->mp_flags |= OS_MEMPOOL_F_RUNTIME;
        #if MYNEWT_VAL(MP_BLOCK_REUSED)
        mp->mp_flags |= OS_MEMPOOL_F_REUSED;
        mp->mp_alloc_blocks = 0;
        #endif
        SLIST_FIRST(mp) = NULL;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        /* Ensure list is initialized before inserting */
        os_mempool_list_ensure_init();
#endif

        struct os_mempool *cur;
        /* Check if the mempool is already in the list. */
        STAILQ_FOREACH(cur, &g_os_mempool_list, mp_list) {
            if (cur == mp) {
                os_mempool_unregister(mp);
                break;
            }
        }

        STAILQ_INSERT_TAIL(&g_os_mempool_list, mp, mp_list);
        return OS_OK;
    }
#endif

    if (blocks > 0) {
        os_mempool_poison(mp, membuf);
        os_mempool_guard(mp, membuf);
        true_block_size = OS_MEMPOOL_TRUE_BLOCK_SIZE(mp);

        /* Chain the memory blocks to the free list */
        block_addr = (uint8_t *)membuf;
        block_ptr = (struct os_memblock *)block_addr;
        for (i = 1; i < blocks; i++) {
            block_addr += true_block_size;
            os_mempool_poison(mp, block_addr);
            os_mempool_guard(mp, block_addr);
            SLIST_NEXT(block_ptr, mb_next) = (struct os_memblock *)block_addr;
            block_ptr = (struct os_memblock *)block_addr;
        }

        /* Last one in the list should be NULL */
        SLIST_NEXT(block_ptr, mb_next) = NULL;
    }

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    os_mempool_list_ensure_init();
#endif

    /* Check if mempool is already in the list (reinitialization case) */
    {
        struct os_mempool *cur;
        STAILQ_FOREACH(cur, &g_os_mempool_list, mp_list) {
            if (cur == mp) {
                /* Mempool is already in the list, remove it first */
                os_mempool_unregister(mp);
                break;
            }
        }
    }

    STAILQ_INSERT_TAIL(&g_os_mempool_list, mp, mp_list);

    return OS_OK;
}

os_error_t
os_mempool_init(struct os_mempool *mp, uint16_t blocks, uint32_t block_size,
                void *membuf, const char *name)
{
    return os_mempool_init_internal(mp, blocks, block_size, membuf, name, 0);
}

os_error_t
os_mempool_ext_init(struct os_mempool_ext *mpe, uint16_t blocks,
                    uint32_t block_size, void *membuf, const char *name)
{
    int rc;

    rc = os_mempool_init_internal(&mpe->mpe_mp, blocks, block_size, membuf,
                                  name, OS_MEMPOOL_F_EXT);
    if (rc != 0) {
        return rc;
    }

    mpe->mpe_put_cb = NULL;
    mpe->mpe_put_arg = NULL;

    return 0;
}

os_error_t
os_mempool_unregister(struct os_mempool *mp)
{
    struct os_mempool *prev;
    struct os_mempool *next;
    struct os_mempool *cur;

    /* Remove the mempool from the global stailq.  This is done manually rather
     * than with `STAILQ_REMOVE` to allow for a graceful failure if the mempool
     * isn't found.
     */
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    os_mempool_list_ensure_init();
#endif

    prev = NULL;
    STAILQ_FOREACH(cur, &g_os_mempool_list, mp_list) {
        if (cur == mp) {
            break;
        }
        prev = cur;
    }

    if (cur == NULL) {
        return OS_INVALID_PARM;
    }

    if (prev == NULL) {
        STAILQ_REMOVE_HEAD(&g_os_mempool_list, mp_list);
    } else {
        next = STAILQ_NEXT(cur, mp_list);
        if (next == NULL) {
            g_os_mempool_list.stqh_last = &STAILQ_NEXT(prev, mp_list);
        }

        STAILQ_NEXT(prev, mp_list) = next;
    }

    return 0;
}

os_error_t
os_mempool_clear(struct os_mempool *mp)
{
    struct os_memblock *block_ptr;
    int true_block_size;
    uint8_t *block_addr;
    uint16_t blocks;

    if (!mp) {
        return OS_INVALID_PARM;
    }

    /* Handle zero-block pools safely */
    if (mp->mp_num_blocks == 0) {
        mp->mp_num_free = 0;
        mp->mp_min_free = 0;
        SLIST_FIRST(mp) = NULL;
        return OS_OK;
    }

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    /* For runtime allocation mode, check whether all blocks have been freed */
    if (mp->mp_flags & OS_MEMPOOL_F_RUNTIME) {
        assert(mp->mp_num_free == mp->mp_num_blocks);
        /* For block reused mode, free all allocated blocks */
        if (mp->mp_flags & OS_MEMPOOL_F_REUSED) {
            void *temp_ptr;
            block_ptr = SLIST_FIRST(mp);
            while (block_ptr) {
                temp_ptr = block_ptr;
                block_ptr = SLIST_NEXT(block_ptr, mb_next);
                nimble_platform_mem_free(temp_ptr);
            }
            mp->mp_alloc_blocks = 0;
        }
        /* Only reset statistics */
        SLIST_FIRST(mp) = NULL;
        mp->mp_min_free = mp->mp_num_blocks;
        return OS_OK;
    }
#endif

    true_block_size = OS_MEMPOOL_TRUE_BLOCK_SIZE(mp);

    /* cleanup the memory pool structure */
    mp->mp_num_free = mp->mp_num_blocks;
    mp->mp_min_free = mp->mp_num_blocks;
    os_mempool_poison(mp, (void *)mp->mp_membuf_addr);
    os_mempool_guard(mp, (void *)mp->mp_membuf_addr);
    SLIST_FIRST(mp) = (void *)mp->mp_membuf_addr;

    /* Chain the memory blocks to the free list */
    block_addr = (uint8_t *)mp->mp_membuf_addr;
    block_ptr = (struct os_memblock *)block_addr;
    blocks = mp->mp_num_blocks;

    while (blocks > 1) {
        block_addr += true_block_size;
        os_mempool_poison(mp, block_addr);
        os_mempool_guard(mp, block_addr);
        SLIST_NEXT(block_ptr, mb_next) = (struct os_memblock *)block_addr;
        block_ptr = (struct os_memblock *)block_addr;
        --blocks;
    }

    /* Last one in the list should be NULL */
    SLIST_NEXT(block_ptr, mb_next) = NULL;

    return OS_OK;
}

os_error_t
os_mempool_ext_clear(struct os_mempool_ext *mpe)
{
    int rc;

    /* Clear the mempool first while EXT flag is intact */
    rc = os_mempool_clear(&mpe->mpe_mp);

    /* Then clear the extended flags using bitwise operation to preserve other flags */
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    mpe->mpe_mp.mp_flags &= ~OS_MEMPOOL_F_EXT;
#else
    mpe->mpe_mp.mp_flags &= ~OS_MEMPOOL_F_EXT;
#endif
    mpe->mpe_put_cb = NULL;
    mpe->mpe_put_arg = NULL;

    return rc;
}

bool
os_mempool_is_sane(const struct os_mempool *mp)
{
    struct os_memblock *block;

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    /* Runtime mode cannot verify sanity */
    if (mp->mp_flags & OS_MEMPOOL_F_RUNTIME) {
        assert(0);
    }
#endif

    /* Verify that each block in the free list belongs to the mempool. */
    /* Limit iterations to prevent infinite loops in case of corruption */
    uint16_t iterations = 0;
    SLIST_FOREACH(block, mp, mb_next) {
        if (++iterations > mp->mp_num_blocks) {
            /* Potential cycle detected */
            return false;
        }
        if (!os_memblock_from(mp, block)) {
            return false;
        }
        os_mempool_poison_check(mp, block);
        os_mempool_guard_check(mp, block);
    }

    return true;
}

int
os_memblock_from(const struct os_mempool *mp, const void *block_addr)
{
    uint32_t true_block_size;
    uintptr_t baddr;
    uintptr_t end;

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    /* Runtime allocation mode doesn't support from */
    if (mp->mp_flags & OS_MEMPOOL_F_RUNTIME) {
        assert(0);
    }
#endif

    baddr = (uintptr_t)block_addr;
    true_block_size = OS_MEMPOOL_TRUE_BLOCK_SIZE(mp);
    end = mp->mp_membuf_addr + (mp->mp_num_blocks * true_block_size);

    /* Check that the block is in the memory buffer range. */
    if ((baddr < mp->mp_membuf_addr) || (baddr >= end)) {
        return 0;
    }

    /* All freed blocks should be on true block size boundaries! */
    if (((baddr - mp->mp_membuf_addr) % true_block_size) != 0) {
        return 0;
    }

    return 1;
}

void *
os_memblock_get(struct os_mempool *mp)
{
    os_sr_t sr;
    struct os_memblock *block;

    os_trace_api_u32(OS_TRACE_ID_MEMBLOCK_GET, (uintptr_t)mp);

    /* Check to make sure they passed in a memory pool (or something) */
    block = NULL;

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    /* Runtime allocation mode */
    if (mp && mp->mp_flags & OS_MEMPOOL_F_RUNTIME) {
        bool need_alloc = false;
        void *allocated_block;
        uint32_t alloc_size;

        OS_ENTER_CRITICAL(sr);

        if (mp->mp_num_free) {
            if (mp->mp_flags & OS_MEMPOOL_F_REUSED) {
                if (SLIST_FIRST(mp) != NULL) {
                    block = SLIST_FIRST(mp);
                    SLIST_FIRST(mp) = SLIST_NEXT(block, mb_next);
                } else if (mp->mp_alloc_blocks < mp->mp_num_blocks) {
                    need_alloc = true;
                    mp->mp_alloc_blocks++;
                } else {
                    /* apply the changes: pool budget exhausted and free list empty;
                     * exit without decrementing mp_num_free to prevent permanent
                     * pool exhaustion when mp_num_free > 0 but no block available */
                    OS_EXIT_CRITICAL(sr);
                    return NULL;
                }
            } else {
                need_alloc = true;
            }
            /* Decrement number free by 1 */
            mp->mp_num_free--;
            if (mp->mp_min_free > mp->mp_num_free) {
                mp->mp_min_free = mp->mp_num_free;
            }
        }

        OS_EXIT_CRITICAL(sr);

        /* Allocate outside critical section to avoid holding lock too long */
        if (need_alloc) {
            alloc_size = OS_MEMPOOL_TRUE_BLOCK_SIZE(mp);
            allocated_block = nimble_platform_mem_malloc(alloc_size);

            if (allocated_block) {
                /* Initialize poison and guard */
                os_mempool_poison(mp, allocated_block);
                os_mempool_guard(mp, allocated_block);
                /* Save mempool pointer for block */
                block = (struct os_memblock *)(allocated_block);
            } else {
                // Should not happen
                OS_ENTER_CRITICAL(sr);
                mp->mp_num_free++;
                /* apply the changes: restore pre-incremented mp_alloc_blocks on
                 * malloc failure to keep counter consistent with actual allocations */
#if MYNEWT_VAL(MP_BLOCK_REUSED)
                if (mp->mp_flags & OS_MEMPOOL_F_REUSED) {
                    mp->mp_alloc_blocks--;
                }
#endif
                OS_EXIT_CRITICAL(sr);
                esp_rom_printf("%s malloc failed, size=%u\n", __func__, alloc_size);
            }
        } else if (block) {
            os_mempool_poison_check(mp, block);
            os_mempool_guard_check(mp, block);
        }

        return block;
    }
#endif

    if (mp) {
        OS_ENTER_CRITICAL(sr);
        /* Check for any free */
        if (mp->mp_num_free) {
            /* Get a free block */
            block = SLIST_FIRST(mp);

            /* Set new free list head */
            SLIST_FIRST(mp) = SLIST_NEXT(block, mb_next);

            /* Decrement number free by 1 */
            mp->mp_num_free--;
            if (mp->mp_min_free > mp->mp_num_free) {
                mp->mp_min_free = mp->mp_num_free;
            }
        }
        OS_EXIT_CRITICAL(sr);

        if (block) {
            os_mempool_poison_check(mp, block);
            os_mempool_guard_check(mp, block);
        }
    }

    os_trace_api_ret_u32(OS_TRACE_ID_MEMBLOCK_GET, (uintptr_t)block);

    return (void *)block;
}

os_error_t
os_memblock_put_from_cb(struct os_mempool *mp, void *block_addr)
{
    os_sr_t sr;
    struct os_memblock *block;

    os_trace_api_u32x2(OS_TRACE_ID_MEMBLOCK_PUT_FROM_CB, (uintptr_t)mp,
                       (uintptr_t)block_addr);

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    if (mp->mp_flags & OS_MEMPOOL_F_RUNTIME) {
        bool need_free = true;
        os_mempool_guard_check(mp, block_addr);

        /* Runtime allocation mode - free directly */
        OS_ENTER_CRITICAL(sr);
        if (mp->mp_flags & OS_MEMPOOL_F_REUSED) {
            /* Check for double-free by searching the free list */
            struct os_memblock *existing_block;
            SLIST_FOREACH(existing_block, mp, mb_next) {
                if (existing_block == block_addr) {
                    OS_EXIT_CRITICAL(sr);
                    return OS_INVALID_PARM;
                }
            }

            block = (struct os_memblock *)block_addr;
            SLIST_NEXT(block, mb_next) = SLIST_FIRST(mp);
            SLIST_FIRST(mp) = block;
            need_free = false;
        }
        mp->mp_num_free++;
        assert(mp->mp_num_blocks >= mp->mp_num_free);
        OS_EXIT_CRITICAL(sr);

        /* Poison and free outside critical section to minimize lock hold time */
        if (!need_free) {
            os_mempool_poison(mp, block_addr);
        } else {
            nimble_platform_mem_free(block_addr);
        }
        return OS_OK;
    }
#endif

    /* Validate that the block belongs to this mempool for static pools */
    if (!(mp->mp_flags & OS_MEMPOOL_F_RUNTIME) && !os_memblock_from(mp, block_addr)) {
        return OS_INVALID_PARM;
    }

    os_mempool_guard_check(mp, block_addr);
    os_mempool_poison(mp, block_addr);

    block = (struct os_memblock *)block_addr;
    OS_ENTER_CRITICAL(sr);

    /* Check for duplicate free - verify block is not already in free list */
    {
        struct os_memblock *cur;
        SLIST_FOREACH(cur, mp, mb_next) {
            if (cur == block) {
                OS_EXIT_CRITICAL(sr);
                return OS_INVALID_PARM;
            }
        }
    }

    /* Check that the number free doesn't exceed number blocks */
    if (mp->mp_num_free >= mp->mp_num_blocks) {
        OS_EXIT_CRITICAL(sr);
        return OS_INVALID_PARM;
    }

    /* Chain current free list pointer to this block; make this block head */
    SLIST_NEXT(block, mb_next) = SLIST_FIRST(mp);
    SLIST_FIRST(mp) = block;

    /* Increment number free */
    mp->mp_num_free++;

    OS_EXIT_CRITICAL(sr);

    os_trace_api_ret_u32(OS_TRACE_ID_MEMBLOCK_PUT_FROM_CB, (uint32_t)OS_OK);

    return OS_OK;
}

os_error_t
os_memblock_put(struct os_mempool *mp, void *block_addr)
{
    struct os_mempool_ext *mpe;
    os_error_t ret;
#if MYNEWT_VAL(OS_MEMPOOL_CHECK)
    struct os_memblock *block;
    int sr;
#endif

    os_trace_api_u32x2(OS_TRACE_ID_MEMBLOCK_PUT, (uintptr_t)mp,
                       (uintptr_t)block_addr);

    /* Make sure parameters are valid */
    if ((mp == NULL) || (block_addr == NULL)) {
        ret = OS_INVALID_PARM;
        goto done;
    }

#if MYNEWT_VAL(OS_MEMPOOL_CHECK)
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
    if (!(mp->mp_flags & OS_MEMPOOL_F_RUNTIME))
#endif
    {
        /* Check that the block we are freeing is a valid block! */
        assert(os_memblock_from(mp, block_addr));
    }

    /*
     * Check for duplicate free.
     */
    OS_ENTER_CRITICAL(sr);
    SLIST_FOREACH(block, mp, mb_next) {
        assert(block != (struct os_memblock *)block_addr);
    }
    OS_EXIT_CRITICAL(sr);

#endif
    /* If this is an extended mempool with a put callback, call the callback
     * instead of freeing the block directly.
     */
    if (mp->mp_flags & OS_MEMPOOL_F_EXT) {
        mpe = (struct os_mempool_ext *)mp;
        if (mpe->mpe_put_cb != NULL) {
            ret = mpe->mpe_put_cb(mpe, block_addr, mpe->mpe_put_arg);
            goto done;
        }
    }

    /* No callback; free the block. */
    ret = os_memblock_put_from_cb(mp, block_addr);

done:
    os_trace_api_ret_u32(OS_TRACE_ID_MEMBLOCK_PUT, (uint32_t)ret);
    return ret;
}

struct os_mempool *
os_mempool_info_get_next(struct os_mempool *mp, struct os_mempool_info *omi)
{
    struct os_mempool *cur;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    os_mempool_list_ensure_init();
#endif

    if (mp == NULL) {
        cur = STAILQ_FIRST(&g_os_mempool_list);
    } else {
        cur = STAILQ_NEXT(mp, mp_list);
    }

    if (cur == NULL) {
        return (NULL);
    }

    omi->omi_block_size = cur->mp_block_size;
    omi->omi_num_blocks = cur->mp_num_blocks;
    omi->omi_num_free = cur->mp_num_free;
    omi->omi_min_free = cur->mp_min_free;
    strncpy(omi->omi_name, cur->name, sizeof(omi->omi_name) - 1);
    omi->omi_name[sizeof(omi->omi_name) - 1] = '\0';

    return (cur);
}

struct os_mempool *
os_mempool_get(const char *mempool_name, struct os_mempool_info *info)
{
    struct os_mempool *mp;

    if (mempool_name == NULL) {
        return NULL;
    }

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    os_mempool_list_ensure_init();
#endif

    mp = STAILQ_FIRST(&g_os_mempool_list);
    while (mp) {
        if (mp->name != NULL && strcmp(mempool_name, mp->name) == 0) {
            break;
        }
        mp = STAILQ_NEXT(mp, mp_list);
    }

    if (mp != NULL && info != NULL) {
        info->omi_block_size = mp->mp_block_size;
        info->omi_num_blocks = mp->mp_num_blocks;
        info->omi_num_free = mp->mp_num_free;
        info->omi_min_free = mp->mp_min_free;
        info->omi_name[0] = '\0';
        strncat(info->omi_name, mp->name, sizeof(info->omi_name) - 1);
    }

    return mp;
}

void
os_mempool_module_init(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    os_mempool_list_ensure_init();
#else
    /* Only initialize if not using dynamic initialization */
    STAILQ_INIT(&g_os_mempool_list);
#endif
}

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
void
os_mempool_deinit(void)
{
    struct os_mempool *mp = NULL;

    mp = STAILQ_FIRST(&g_os_mempool_list);

    // All mempool blocks should be reclaimed after nimble deinit
    while (mp) {
        if (mp->mp_flags & OS_MEMPOOL_F_RUNTIME) {
            os_mempool_clear(mp);
        }
        mp = STAILQ_NEXT(mp, mp_list);
    }
}
#endif
