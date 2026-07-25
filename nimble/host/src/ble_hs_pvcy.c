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
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "stats/stats.h"
#include "ble_hs_priv.h"
#include "ble_hs_resolv_priv.h"
#include "host/ble_hs_pvcy.h"
#include "host/ble_hs_log.h"
#include "nimble/hci_common.h"

#if MYNEWT_VAL(BLE_HS_PVCY)
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#include "esp_nimble_mem.h"

typedef struct {
    uint8_t pvcy_started;             /* Indicates if privacy is started */
    uint8_t pvcy_irk[16];             /* Static IRK */
    uint8_t pvcy_default_irk[16];     /* Static default IRK */
    uint16_t rpa_timeout;             /* Timeout for RPA rotation */
} ble_hs_pvcy_ctx_t;

static ble_hs_pvcy_ctx_t *ble_hs_pvcy_ctx;

#define ble_hs_pvcy_started        (ble_hs_pvcy_ctx->pvcy_started)
#define ble_hs_pvcy_irk            (ble_hs_pvcy_ctx->pvcy_irk)
#define ble_hs_pvcy_default_irk    (ble_hs_pvcy_ctx->pvcy_default_irk)
#define l_rpa_timeout              (ble_hs_pvcy_ctx->rpa_timeout)

#else
static uint8_t ble_hs_pvcy_started;
static uint8_t ble_hs_pvcy_irk[16];

/** Use this as a default IRK if none gets set. */
uint8_t ble_hs_pvcy_default_irk[16];
uint16_t l_rpa_timeout;
#endif

#define BLE_MAX_RPA_TIMEOUT_VAL 0x0E10

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
void ble_store_config_init(void);
#endif

const uint8_t *
ble_hs_pvcy_get_default_irk(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx) {
        return ble_hs_pvcy_ctx->pvcy_default_irk;
    }
    return NULL;
#else
    return ble_hs_pvcy_default_irk;
#endif
}

static int
ble_hs_pvcy_set_addr_timeout(uint16_t timeout)
{
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    return ble_hs_resolv_set_rpa_tmo(timeout);
#else
    struct ble_hci_le_set_rpa_tmo_cp cmd;

    if (timeout == 0 || timeout > BLE_MAX_RPA_TIMEOUT_VAL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    cmd.rpa_timeout = htole16(timeout);

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_SET_RPA_TMO),
                             &cmd, sizeof(cmd), NULL, 0);
#endif
}

int ble_hs_set_rpa_timeout(uint16_t timeout)
{
    int rc;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ENOMEM);
        return BLE_HS_ENOMEM;
    }
#endif

    rc = ble_hs_pvcy_set_addr_timeout(timeout);
    if (rc == 0) {
        ble_hs_lock();
        l_rpa_timeout = timeout;
        ble_hs_unlock();
    }

    return rc;
}

uint16_t ble_hs_get_rpa_timeout(void)
{
    uint16_t tmo;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx == NULL) {
        return 0;
    }
#endif
    ble_hs_lock();
    tmo = l_rpa_timeout;
    ble_hs_unlock();

    return tmo;
}

void ble_hs_reset_rpa_timeout(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx == NULL) {
        return;
    }
#endif
    l_rpa_timeout = 0;
}

#if (!MYNEWT_VAL(BLE_HOST_BASED_PRIVACY))
#if MYNEWT_VAL(BLE_DEFER_CONN_EVENTS)
static uint8_t ble_hs_pvcy_resolve_en;
#endif

int
ble_hs_pvcy_set_resolve_enabled(int enable)
{
    struct ble_hci_le_set_addr_res_en_cp cmd;
#if MYNEWT_VAL(BLE_DEFER_CONN_EVENTS)
    int rc;

    cmd.enable = !!enable;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_SET_ADDR_RES_EN),
                             &cmd, sizeof(cmd), NULL, 0);
    if (rc == 0) {
        ble_hs_pvcy_resolve_en = cmd.enable;
    }

    return rc;
#else
    cmd.enable = enable;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_SET_ADDR_RES_EN),
                             &cmd, sizeof(cmd), NULL, 0);
#endif
}

#if MYNEWT_VAL(BLE_DEFER_CONN_EVENTS)
static void
ble_hs_pvcy_restore_resolve_if_needed(uint8_t was_enabled)
{
    int rc_en;

    if (!was_enabled) {
        return;
    }

    rc_en = BLE_HS_EUNKNOWN;
    for (int i = 0; i < 3 && rc_en != 0; i++) {
        rc_en = ble_hs_pvcy_set_resolve_enabled(1);
    }
    if (rc_en != 0) {
        BLE_HS_LOG(ERROR,
                   "ble_hs_pvcy: address resolution restore failed after "
                   "retries; privacy broken\n");
    }
}
#endif
#endif

int
ble_hs_pvcy_remove_entry(uint8_t addr_type, const uint8_t *addr)
{
    struct ble_hci_le_rmv_resolve_list_cp cmd;
    int rc;

    BLE_HS_DBG_ASSERT(addr != NULL);

    if (addr_type > BLE_ADDR_RANDOM) {
        addr_type %= 2;
    }

    cmd.peer_addr_type = addr_type;
    memcpy(cmd.peer_id_addr, addr, BLE_DEV_ADDR_LEN);

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    ble_hs_lock();
    rc = ble_hs_resolv_list_rmv(addr_type, &cmd.peer_id_addr[0]);
    ble_hs_unlock();
#else
    ble_gap_preempt();
    ble_hs_lock();
    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                               BLE_HCI_OCF_LE_RMV_RESOLV_LIST),
                               &cmd, sizeof(cmd), NULL, 0);
    ble_hs_unlock();
    ble_gap_preempt_done();
#endif

    return rc;
}

#if (!MYNEWT_VAL(BLE_HOST_BASED_PRIVACY))
static int
ble_hs_pvcy_clear_entries(void)
{
    int rc;

    ble_gap_preempt();
    ble_hs_lock();
    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_CLR_RESOLV_LIST),
                           NULL, 0, NULL, 0);
    ble_hs_unlock();
    ble_gap_preempt_done();

    return rc;
}
#endif

static int
ble_hs_pvcy_add_entry_hci(const uint8_t *addr, uint8_t addr_type,
                          const uint8_t *irk)
{
    struct ble_hci_le_add_resolv_list_cp cmd;
    int rc;

    BLE_HS_DBG_ASSERT(addr != NULL);
    BLE_HS_DBG_ASSERT(irk != NULL);

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ENOMEM);
        return BLE_HS_ENOMEM;
    }
#endif

    if (addr_type > BLE_ADDR_RANDOM) {
        return BLE_HS_EINVAL;
    }

    cmd.peer_addr_type = addr_type;
    memcpy(cmd.peer_id_addr, addr, 6);
    ble_hs_lock();
    memcpy(cmd.local_irk, ble_hs_pvcy_irk, 16);
    ble_hs_unlock();
    memcpy(cmd.peer_irk, irk, 16);

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    ble_hs_lock_nested();
    rc = ble_hs_resolv_list_add((uint8_t *) &cmd);
    ble_hs_unlock_nested();
    if (rc != 0) {
        return rc;
    }

#else
    ble_addr_t peer_addr;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_ADD_RESOLV_LIST),
                           &cmd, sizeof(cmd), NULL, 0);
    if (rc != 0) {
        return rc;
    }

    /* Controller is BT5.0 and default privacy mode is network which
     * can cause problems for apps which are not aware of it. We set device
     * mode for all peer devices; the app can change it to network if needed.
     * BT4.2 controllers do not support LE Set Privacy Mode; ignore that error.
     */
    peer_addr.type = addr_type;
    memcpy(peer_addr.val, addr, sizeof peer_addr.val);
    rc = ble_hs_pvcy_set_mode(&peer_addr, BLE_GAP_PRIVATE_MODE_DEVICE);
    if (rc != 0 && rc != BLE_HS_HCI_ERR(BLE_ERR_UNKNOWN_HCI_CMD)) {
        ble_hs_pvcy_remove_entry(addr_type, addr);
        return rc;
    }

#endif

    return 0;
}

#if MYNEWT_VAL(BLE_DEFER_CONN_EVENTS)
int
ble_hs_pvcy_replace_entry(const uint8_t *addr, uint8_t addr_type,
                          const uint8_t *irk)
{
#if (MYNEWT_VAL(BLE_HOST_BASED_PRIVACY))
    (void)ble_hs_pvcy_remove_entry(addr_type, addr);
    return ble_hs_pvcy_add_entry(addr, addr_type, irk);
#else
    int rc;
    uint8_t resolve_was_en;

    STATS_INC(ble_hs_stats, pvcy_add_entry);

    ble_gap_preempt();
    resolve_was_en = ble_hs_pvcy_resolve_en;

    rc = ble_hs_pvcy_set_resolve_enabled(0);
    if (rc == 0) {
        (void)ble_hs_pvcy_remove_entry(addr_type, addr);
        rc = ble_hs_pvcy_add_entry_hci(addr, addr_type, irk);
        ble_hs_pvcy_restore_resolve_if_needed(resolve_was_en);
    }

    ble_gap_preempt_done();

    if (rc != 0) {
        STATS_INC(ble_hs_stats, pvcy_add_entry_fail);
    }

    return rc;
#endif
}
#endif /* MYNEWT_VAL(BLE_DEFER_CONN_EVENTS) */

int
ble_hs_pvcy_add_entry(const uint8_t *addr, uint8_t addr_type,
                      const uint8_t *irk)
{
    int rc;

    STATS_INC(ble_hs_stats, pvcy_add_entry);

    /* No GAP procedures can be active when adding an entry to the resolving
     * list (Vol 2, Part E, 7.8.38).  Stop all GAP procedures and temporarily
     * prevent any new ones from being started.
     */
#if (MYNEWT_VAL(BLE_HOST_BASED_PRIVACY))
    rc = ble_hs_pvcy_add_entry_hci(addr, addr_type, irk);
#else
    ble_gap_preempt();

#if MYNEWT_VAL(BLE_DEFER_CONN_EVENTS)
    uint8_t resolve_was_en;

    resolve_was_en = ble_hs_pvcy_resolve_en;
    rc = ble_hs_pvcy_set_resolve_enabled(0);
    if (rc == 0) {
        /* Try to add the entry now that GAP is halted and address resolution is disabled. */
        rc = ble_hs_pvcy_add_entry_hci(addr, addr_type, irk);
        ble_hs_pvcy_restore_resolve_if_needed(resolve_was_en);
    }
#else
    /* Try to add the entry now that GAP is halted. */
    rc = ble_hs_pvcy_add_entry_hci(addr, addr_type, irk);
#endif

    /* Allow GAP procedures to be started again. */
    ble_gap_preempt_done();

#endif
    if (rc != 0) {
        STATS_INC(ble_hs_stats, pvcy_add_entry_fail);
    }

    return rc;
}

int
ble_hs_pvcy_ensure_started(void)
{
    int rc;
    uint16_t rpa_timeout;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ENOMEM);
        return BLE_HS_ENOMEM;
    }
#endif

    ble_hs_lock_nested();

    if (ble_hs_pvcy_started) {
        ble_hs_unlock_nested();
        return 0;
    }

#if (MYNEWT_VAL(BLE_HOST_BASED_PRIVACY))
    /*This is to be called only once*/
    ble_hs_resolv_init();
#endif

    /* Check if user has already set any timeout. If yes, use it */
    rpa_timeout = l_rpa_timeout;

    /* Set up the periodic change of our RPA. */
    if (rpa_timeout) {
        rc = ble_hs_pvcy_set_addr_timeout(rpa_timeout);
    } else {
        rc = ble_hs_pvcy_set_addr_timeout(MYNEWT_VAL(BLE_RPA_TIMEOUT));
    }

    if (rc != 0) {
        ble_hs_unlock_nested();
        return rc;
    }

    ble_hs_pvcy_started = 1;

    ble_hs_unlock_nested();

    return 0;
}

void ble_hs_pvcy_set_default_irk(void)
{
    struct ble_store_value_local_irk  value_local_irk;
    struct ble_store_key_local_irk key_local_irk;

    uint8_t local_id[BLE_DEV_ADDR_LEN];
    bool have_local_id;
    int rc;

    memset(&key_local_irk, 0, sizeof key_local_irk);
    memset(&value_local_irk, 0x0, sizeof value_local_irk);

    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, local_id, NULL);
    have_local_id = (rc == 0);

    /* Create key / value */
    /* Some controllers give all 0s as address. Handle such case */
    if (have_local_id) {
        memcpy(key_local_irk.addr.val, local_id, BLE_DEV_ADDR_LEN);
    }

    key_local_irk.addr.type = BLE_ADDR_PUBLIC;

    /* Read NVS for local IRK */

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (!ble_hs_pvcy_ctx) {
        ble_hs_pvcy_ctx = nimble_platform_mem_calloc(1, sizeof(*ble_hs_pvcy_ctx));
        if (!ble_hs_pvcy_ctx) {
            BLE_HS_LOG(ERROR," Failed to allocate memory for ble_hs_pvcy_ctx");
            return;
        }
    }

    void ble_store_config_init(void);
    ble_store_config_init();
#endif

    rc = ble_store_read_local_irk(&key_local_irk, &value_local_irk);
    if (!rc) {
        memcpy(ble_hs_pvcy_default_irk, value_local_irk.irk, 16);
    } else {

#if !MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
        /* No entry for local IRK found . Generate one and load in NVS */
        memset(ble_hs_pvcy_default_irk, 0x0, 16);
#endif
        rc = ble_hs_hci_util_rand(ble_hs_pvcy_default_irk, 16);

        if (rc != 0) {
            BLE_HS_LOG(ERROR, "Failed to generate local IRK");
            return;
        }

        memset(&value_local_irk, 0x0, sizeof value_local_irk);

        memcpy(&value_local_irk.irk, ble_hs_pvcy_default_irk, 16);

        if (have_local_id) {
            memcpy(value_local_irk.addr.val, local_id, BLE_DEV_ADDR_LEN);
        }

        value_local_irk.addr.type = BLE_ADDR_PUBLIC;

        rc = ble_store_write_local_irk(&value_local_irk);
        if (rc != 0) {
            BLE_HS_LOG(WARN, "Failed to persist local IRK (rc=%d)", rc);
        }
    }
}

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
void
ble_hs_pvcy_irk_deinit(void)
{
    if (ble_hs_pvcy_ctx) {
        nimble_platform_mem_free(ble_hs_pvcy_ctx);
        ble_hs_pvcy_ctx = NULL;
    }
}
#else
void
ble_hs_pvcy_irk_deinit(void)
{
    ble_hs_pvcy_started = 0;
    memset(ble_hs_pvcy_irk, 0, sizeof(ble_hs_pvcy_irk));
    memset(ble_hs_pvcy_default_irk, 0, sizeof(ble_hs_pvcy_default_irk));
    l_rpa_timeout = 0;
}
#endif

int
ble_hs_pvcy_set_our_irk(const uint8_t *irk)
{
    uint8_t tmp_addr[6];
    uint8_t new_irk[16];
    int rc = 0;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ENOMEM);
        return BLE_HS_ENOMEM;
    }
#endif

    if (irk != NULL) {
        memcpy(new_irk, irk, 16);
    } else {
        memcpy(new_irk, ble_hs_pvcy_default_irk, 16);
    }

    ble_hs_lock();
    memcpy(ble_hs_pvcy_irk, new_irk, 16);
    ble_hs_unlock();

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    if (irk != NULL) {
       ble_hs_lock_nested();
       bool rpa_state = false;

       if ((rpa_state = ble_host_rpa_enabled()) == true) {
            ble_hs_resolv_enable(0);
       }

       ble_hs_resolv_list_clear_all();

       if (rpa_state) {
             ble_hs_resolv_enable(1);
       }
       ble_hs_unlock_nested();
   }
#else
    ble_gap_preempt();

    rc = ble_hs_pvcy_set_resolve_enabled(0);
    if (rc != 0) {
       goto done;
    }

    rc = ble_hs_pvcy_clear_entries();
    if (rc != 0) {
       goto done;
    }

    rc = ble_hs_pvcy_set_resolve_enabled(1);

done:
    ble_gap_preempt_done();
    if (rc != 0) {
        goto pvcy_done;
    }

#endif

#if MYNEWT_VAL(BLE_HS_PVCY)
    /*
      * Add local IRK entry with 00:00:00:00:00:00 address. This entry will
      * be used to generate RPA for non-directed advertising if own_addr_type
      * is set to rpa_pub since we use all-zero address as peer addres in
      * such case. Peer IRK should be left all-zero since this is not for an
      * actual peer.
      */
    uint8_t zero_irk[16] = {0};
    memset(tmp_addr, 0, 6);
    rc = ble_hs_pvcy_add_entry(tmp_addr, 0, zero_irk);
    if (rc != 0) {
#if !MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
        goto pvcy_done;
#else
        return rc;
#endif
    }
#endif

#if !MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
pvcy_done:
    ble_gap_preempt_done();
#endif

    return rc;
}

int
ble_hs_pvcy_our_irk(const uint8_t **out_irk)
{
    /* XXX: Return error if privacy not supported. */

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ENOMEM);
        return BLE_HS_ENOMEM;
    }
#endif

    if (out_irk == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    *out_irk = ble_hs_pvcy_irk;
    return 0;
}

int
ble_hs_pvcy_set_mode(const ble_addr_t *addr, uint8_t priv_mode)
{
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    return 0;
#else
    struct ble_hci_le_set_privacy_mode_cp cmd;

    if (addr == NULL) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    if (addr->type > BLE_ADDR_RANDOM_ID) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EINVAL);
        return BLE_HS_EINVAL;
    }

    cmd.mode = priv_mode;
    cmd.peer_id_addr_type = addr->type & 0x01;
    memcpy(cmd.peer_id_addr, addr->val, BLE_DEV_ADDR_LEN);

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_SET_PRIVACY_MODE),
                             &cmd, sizeof(cmd), NULL, 0);
#endif
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
bool
ble_hs_pvcy_enabled(void)
{
#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_hs_pvcy_ctx == NULL) {
        return false;
    }
#endif
    return ble_hs_pvcy_started;
}

int
ble_hs_pvcy_rpa_config(uint8_t enable)
{
    int rc = 0;

    if (enable != NIMBLE_HOST_DISABLE_PRIVACY) {
        rc = ble_hs_pvcy_ensure_started();
        if (rc != 0) {
            return rc;
        }
    }

    ble_hs_lock();

    if (enable != NIMBLE_HOST_DISABLE_PRIVACY) {
        ble_hs_resolv_enable(true);

        /* Configure NRPA address related flags according to input parameter */
        if (enable == NIMBLE_HOST_ENABLE_NRPA) {
            ble_hs_resolv_nrpa_enable();
        } else {
            ble_hs_resolv_nrpa_disable();
        }
    } else {
        ble_hs_resolv_enable(false);
    }

    ble_hs_unlock();

    if (rc == 0 && enable != NIMBLE_HOST_DISABLE_PRIVACY) {
        /* Generate local RPA address and set it in controller — must be
         * done outside the lock since ble_gap_preempt acquires it. */
        ble_gap_preempt();
        rc = ble_hs_gen_own_private_rnd();
        ble_gap_preempt_done();
    }

    return rc;
}
#endif

int
ble_hs_pvcy_rpa_ah(const uint8_t irk[16], const uint8_t prand[3], uint8_t out[3])
{
    uint8_t plaintext[16] = {0};
    uint8_t ciphertext[16] = {0};
    int rc;

    /* prand goes in MSB of plaintext */
    memcpy(plaintext, prand, 3);

    rc = ble_sm_alg_encrypt(irk, plaintext, ciphertext);
    if (rc != 0) {
        return rc;
    }

    memcpy(out, ciphertext, 3);
    return 0;
}

bool
ble_hs_pvcy_resolve_with_irk(const uint8_t rpa[6], const uint8_t irk[16])
{
    uint8_t hash[3];
    uint8_t out[3];

    /* hash = most significant 3 bytes of RPA */
    memcpy(hash, rpa, 3);

    /* prand = least significant 3 bytes of RPA */
    const uint8_t *prand = rpa + 3;

    if (ble_hs_pvcy_rpa_ah(irk, prand, out) != 0) {
        return false;
    }

    return (memcmp(hash, out, 3) == 0);
}
#endif
