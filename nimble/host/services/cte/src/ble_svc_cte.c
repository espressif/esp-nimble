/**
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
#include <string.h>
#include "sysinit/sysinit.h"
#include "syscfg/syscfg.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/cte/ble_svc_cte.h"

/* XXX: We shouldn't be including the host's private header files.  The host
 * API needs to be updated with a function to query the advertising transmit
 * power.
 */
#include "../src/ble_hs_hci_priv.h"
#include "host/ble_hs_log.h"

#if MYNEWT_VAL(BLE_AOA_AOD)

// Define the size of characteristic values
#define CTE_ENABLE_CHAR_SIZE        1
#define CTE_MIN_LENGTH_CHAR_SIZE    1
#define CTE_MIN_TRANSMIT_COUNT_SIZE 1
#define CTE_TRANSMIT_DURATION_SIZE  1
#define CTE_INTERVAL_CHAR_SIZE      2
#define CTE_PHY_CHAR_SIZE           1

// Constant Tone Extension Enable characteristic value range
#define CTE_ENABLE_MIN_VALUE          0x00
#define CTE_ENABLE_MAX_VALUE          0x03
// Define the constant tone extension enable bit control macros
#define CTE_ENABLE_AOA_CONNECTION (1 << 0) // Bit 0: Enable AoA Constant Tone Extension on ACL connection with client
#define CTE_ENABLE_AOD_ADVERTISING (1 << 1) // Bit 1: Enable AoD Constant Tone Extension in advertising packets

#define CTETYPE_ALLOW_AOA_CTE_RESPONSE (1 << 0)
#define CTETYPE_ALLOW_AOD_1US_CTE_RESPONSE (1 << 1)
#define CTETYPE_ALLOW_AOD_2US_CTE_RESPONSE (1 << 2)
#define CTETYPE_MASK_ALL (0x7) // mask for the first three bits

// Advertising Constant Tone Extension Minimum Length characteristic value range
#define CTE_MIN_LEN_MIN_VALUE         2
#define CTE_MIN_LEN_MAX_VALUE         20

// Advertising Constant Tone Extension Minimum Transmit Count characteristic value range
#define CTE_MIN_TX_COUNT_MIN_VALUE    1
#define CTE_MIN_TX_COUNT_MAX_VALUE    15

// Advertising Constant Tone Extension Transmit Duration characteristic value range
#define CTE_TX_DURATION_MIN_VALUE     0
#define CTE_TX_DURATION_MAX_VALUE     255

// Advertising Constant Tone Extension Interval characteristic value range
#define CTE_INTERVAL_MIN_VALUE        0x0006
#define CTE_INTERVAL_MAX_VALUE        0xFFFF

// Advertising Constant Tone Extension PHY characteristic value range
#define CTE_PHY_MIN_VALUE             0x00
#define CTE_PHY_MAX_VALUE             0x01

// SERVICE Error Code
#define SERVICE_ERROR_WRITE_REQUEST_REJECTED    0xFC
#define SERVICE_ERROR_OUT_OF_RANGE              0xFF

/* Per-connection CTE state (connection-specific only) */
typedef struct {
    uint16_t conn_handle;
    uint8_t cte_enable;
} cte_instance_config_t;

/* Global advertising CTE configuration — independent of individual GATT connections.
 * Advertising CTE parameters (min length, transmit count, duration, interval, PHY)
 * represent per-advertising-set state, not per-connection state. Storing them here
 * prevents settings from being lost on disconnection and ensures all connected
 * clients share a consistent view. */
typedef struct {
    uint8_t cte_min_length;
    uint8_t cte_min_transmit_count;
    uint8_t cte_transmit_duration;
    uint16_t cte_interval;
    uint8_t cte_phy;
} adv_cte_config_t;

static adv_cte_config_t adv_cte_config;

static cte_instance_config_t cte_config[MYNEWT_VAL(BLE_MAX_CONNECTIONS)];


/**
 * @brief Constant Tone Extension Service UUID
 */
static const ble_uuid128_t cte_svc_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
                     0x00, 0x10, 0x00, 0x00, 0x4A, 0x18, 0x00, 0x00);

static int ble_svc_cte_enable_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);

static int ble_svc_cte_adv_cte_min_length_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);

static int ble_svc_cte_adv_cte_min_transmit_count_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);

static int ble_svc_cte_adv_cte_transmit_duration_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);

static int ble_svc_cte_adv_cte_interval_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);

static int ble_svc_cte_adv_cte_phy_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);
/**
 * @brief Constant Tone Extension Service definition
 */

static const ble_uuid16_t uuid_svc_cte = BLE_UUID16_INIT(BLE_SVC_CTE_UUID16);
static const ble_uuid16_t uuid_chr_cte_enable = BLE_UUID16_INIT(BLE_SVC_CTE_CHR_UUID16_ENABLE);
static const ble_uuid16_t uuid_chr_cte_min_len = BLE_UUID16_INIT(BLE_SVC_CTE_CHR_UUID16_MINIMUM_LENGTH);
static const ble_uuid16_t uuid_chr_cte_min_trans_cnt = BLE_UUID16_INIT(BLE_SVC_CTE_CHR_UUID16_MINIMUM_TRANSMIT_COUNT);
static const ble_uuid16_t uuid_chr_cte_trans_dur = BLE_UUID16_INIT(BLE_SVC_CTE_CHR_UUID16_TRANSMIT_DURATION);
static const ble_uuid16_t uuid_chr_cte_ext_itvl = BLE_UUID16_INIT(BLE_SVC_CTE_CHR_UUID16_INTERVAL);
static const ble_uuid16_t uuid_chr_cte_ext_phy = BLE_UUID16_INIT(BLE_SVC_CTE_CHR_UUID16_PHY);

static const struct ble_gatt_chr_def cte_characteristics[] = {
    {
        /*** Characteristic: Constant Tone Extension Enable. */
        .uuid = &uuid_chr_cte_enable.u,
        .access_cb = ble_svc_cte_enable_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {
        /*** Characteristic: Advertising Constant Tone Extension Minimum Length. */
        .uuid = &uuid_chr_cte_min_len.u,
        .access_cb = ble_svc_cte_adv_cte_min_length_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {
        /*** Characteristic: Advertising Constant Tone Extension Minimum Transmit Count. */
        .uuid = &uuid_chr_cte_min_trans_cnt.u,
        .access_cb = ble_svc_cte_adv_cte_min_transmit_count_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {
        /*** Characteristic: Advertising Constant Tone Extension Transmit Duration. */
        .uuid = &uuid_chr_cte_trans_dur.u,
        .access_cb = ble_svc_cte_adv_cte_transmit_duration_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {
        /*** Characteristic: Advertising Constant Tone Extension Interval. */
        .uuid = &uuid_chr_cte_ext_itvl.u,
        .access_cb = ble_svc_cte_adv_cte_interval_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {
        /*** Characteristic: Advertising Constant Tone Extension PHY. */
        .uuid = &uuid_chr_cte_ext_phy.u,
        .access_cb = ble_svc_cte_adv_cte_phy_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
    },
    {
        0, /* No more characteristics in this service. */
    }
};

static const struct ble_gatt_svc_def ble_svc_cte_defs[] = {
    {
        /*** Service: Constant Tone Extension Service. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_cte.u,
        .characteristics = cte_characteristics,
    },
    {
        0, /* No more services. */
    },
};

/**
 * Find the CTE configuration instance for the given connection handle.
 *
 * @param conn_handle The connection handle to search for.
 * @return A pointer to the matching CTE configuration instance, or NULL if not found.
 */
static cte_instance_config_t* cte_find_config_by_conn_handle(uint16_t conn_handle) {
    /* BLE_HS_CONN_HANDLE_NONE (0xffff) is the sentinel for unused slots; matching
     * it would return a pointer to an unassigned entry, corrupting its data. */
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return NULL;
    }
    for (int i = 0; i < MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
        if (cte_config[i].conn_handle == conn_handle) {
            return &cte_config[i];
        }
    }
    return NULL;
}

/**@}*/

/**
 * Handles writing a one-octet (1 byte) characteristic value in a BLE service.
 *
 * This function is used to validate and write a characteristic value that is
 * exactly 1 byte long. It performs length checks, range validation, and then
 * updates the destination buffer with the new value if all validations pass.
 *
 * @param om            The mbuf containing the data to be written.
 * @param min_value     The minimum allowed value for the characteristic.
 * @param max_value     The maximum allowed value for the characteristic.
 * @param dst           A pointer to the destination buffer where the validated 
 *                      value will be stored.
 * @param len           A pointer to store the length of the written value (optional).
 *                      If provided, it will be set to 1 on success.
 *
 * @return              0 on success; BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN if the 
 *                      length of data in the mbuf does not match the expected 
 *                      one-octet length; BLE_ATT_ERR_UNLIKELY if an unexpected 
 *                      error occurs while flattening the mbuf; SERVICE_ERROR_OUT_OF_RANGE 
 *                      if the value is outside the allowed range.
 */
static int
ble_svc_cte_one_octet_chr_write(struct os_mbuf *om,
                      uint32_t min_value, uint32_t max_value,
                      void *dst, uint16_t *len)
{
    uint16_t om_len;
    uint8_t target_len = 1;
    uint8_t value = 0;
    int rc;

    // Get the length of the data in the mbuf
    om_len = OS_MBUF_PKTLEN(om);

    // Ensure the data length matches the expected target length
    if (om_len != target_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    // Flatten the mbuf data into the value buffer
    rc = ble_hs_mbuf_to_flat(om, &value, target_len, NULL);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    // Check if the value is within the allowed range
    if (value < min_value || value > max_value) {
        return SERVICE_ERROR_OUT_OF_RANGE;
    }

    // Update the destination with the validated value
    memcpy(dst, &value, target_len);
    if (len != NULL) {
        *len = target_len;
    }

    return 0;
}

/**
 * Handles writing a two-octet (2 bytes) characteristic value in a BLE service.
 *
 * This function is used to validate and write a characteristic value that is
 * exactly 2 bytes long. It performs length checks, range validation, and then
 * updates the destination buffer with the new value if all validations pass.
 *
 * @param om            The mbuf containing the data to be written.
 * @param min_value     The minimum allowed value for the characteristic.
 * @param max_value     The maximum allowed value for the characteristic.
 * @param dst           A pointer to the destination buffer where the validated 
 *                      value will be stored.
 * @param len           A pointer to store the length of the written value (optional).
 *                      If provided, it will be set to 2 on success.
 *
 * @return              0 on success; BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN if the 
 *                      length of data in the mbuf does not match the expected 
 *                      two-octet length; BLE_ATT_ERR_UNLIKELY if an unexpected 
 *                      error occurs while flattening the mbuf; SERVICE_ERROR_OUT_OF_RANGE 
 *                      if the value is outside the allowed range.
 */
static int
ble_svc_cte_two_octet_chr_write(struct os_mbuf *om,
                      uint32_t min_value, uint32_t max_value,
                      void *dst, uint16_t *len)
{
    uint16_t om_len;
    uint8_t target_len = 2;
    uint16_t value = 0;
    int rc;

    // Get the length of the data in the mbuf
    om_len = OS_MBUF_PKTLEN(om);

    // Ensure the data length matches the expected target length
    if (om_len != target_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    /* ble_hs_mbuf_to_flat copies the raw BLE wire bytes into &value. The BT spec
     * mandates Little-Endian on the wire, and all supported ESP targets (Xtensa,
     * RISC-V) are also Little-Endian, so no byte-swap is needed. Endianness
     * conversion (le16toh / htole16) would only be required on Big-Endian hosts,
     * which are not supported by this platform. */
    rc = ble_hs_mbuf_to_flat(om, &value, target_len, NULL);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (value < min_value || value > max_value) {
        return SERVICE_ERROR_OUT_OF_RANGE;
    }

    memcpy(dst, &value, target_len);
    if (len != NULL) {
        *len = target_len;
    }

    return 0;
}


/**
 * @brief Access callback for Constant Tone Extension Service cte enable
 *
 * This function is called when a read or write operation is performed on the Constant Tone Extension
 * characteristic. It handles the read and write requests.
 *
 * @param conn_handle   The connection handle
 * @param attr_handle   The attribute handle
 * @param ctxt          The GATT access context
 * @param arg           Unused argument
 *
 * @return              0 on success; non-zero error code otherwise
 */
static int ble_svc_cte_enable_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc = BLE_ATT_ERR_UNLIKELY;

    // Find the CTE configuration instance for the given connection handle
    cte_instance_config_t *config = cte_find_config_by_conn_handle(conn_handle);

    // Return an error if no matching configuration instance is found
    if (config == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Handle read characteristic request
            rc = os_mbuf_append(ctxt->om, &config->cte_enable, sizeof(config->cte_enable)) == 0 ?
                    0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            // Handle write characteristic request
            {
                uint8_t old_enable = config->cte_enable;
                rc = ble_svc_cte_one_octet_chr_write(ctxt->om,
                                        CTE_ENABLE_MIN_VALUE,
                                        CTE_ENABLE_MAX_VALUE,
                                        &config->cte_enable, NULL);

                if (rc == 0) {
                    if(((config->cte_enable & CTE_ENABLE_AOA_CONNECTION) == CTE_ENABLE_AOA_CONNECTION) &&
                       ((old_enable & CTE_ENABLE_AOA_CONNECTION) != CTE_ENABLE_AOA_CONNECTION)) {
                        /* 0->1 transition: enable AoA CTE response on this connection */
                        if(ble_gap_set_conn_cte_transmit_param(conn_handle, BLE_GAP_CTE_RSP_ALLOW_AOA_MASK, 0, NULL) != 0)
                        {
                            config->cte_enable = old_enable;
                            rc = 0xFC;
                            break;
                        }
                        if(ble_gap_conn_cte_rsp_enable(conn_handle, true) != 0) {
                            config->cte_enable = old_enable;
                            rc = 0xFC;
                            break;
                        }
                    } else if (((old_enable & CTE_ENABLE_AOA_CONNECTION) == CTE_ENABLE_AOA_CONNECTION) &&
                               ((config->cte_enable & CTE_ENABLE_AOA_CONNECTION) != CTE_ENABLE_AOA_CONNECTION)) {
                        /* 1->0 transition only: disable AoA CTE response.
                         * The previous else-if fired on any write where old bit was 1 (including
                         * redundant 1->1 writes and multi-bit changes like 1->3), incorrectly
                         * disabling CTE when it should remain enabled. */
                        if(ble_gap_conn_cte_rsp_enable(conn_handle, false) != 0) {
                            config->cte_enable = old_enable;
                            rc = 0xFC;
                            break;
                        }
                    }
                    if(((config->cte_enable & CTE_ENABLE_AOD_ADVERTISING) == CTE_ENABLE_AOD_ADVERTISING) &&
                       ((old_enable & CTE_ENABLE_AOD_ADVERTISING) != CTE_ENABLE_AOD_ADVERTISING)) {
                        /* Apply the current advertising CTE parameters to the controller
                         * using advertising instance 0 (default periodic advertising set). */
                        struct ble_gap_periodic_adv_cte_params cte_params = {
                            .cte_length  = adv_cte_config.cte_min_length,
                            .cte_type    = adv_cte_config.cte_phy,
                            .cte_count   = adv_cte_config.cte_min_transmit_count,
                            .switching_pattern_length = 0,
                            .antenna_ids  = NULL,
                        };
                        if (ble_gap_set_connless_cte_transmit_params(0, &cte_params) != 0 ||
                            ble_gap_set_connless_cte_transmit_enable(0, 1) != 0) {
                            config->cte_enable = old_enable;
                            rc = SERVICE_ERROR_WRITE_REQUEST_REJECTED;
                            break;
                        }
                    } else if ((old_enable & CTE_ENABLE_AOD_ADVERTISING) == CTE_ENABLE_AOD_ADVERTISING) {
                        /* 1->0 transition: disable connless CTE transmission */
                        ble_gap_set_connless_cte_transmit_enable(0, 0);
                    }
                }
            }
            break;

        default:
            break;
    }

    return rc;
}

static int ble_svc_cte_adv_cte_min_length_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc = BLE_ATT_ERR_UNLIKELY;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &adv_cte_config.cte_min_length,
                                sizeof(adv_cte_config.cte_min_length)) == 0 ?
                    0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_svc_cte_one_octet_chr_write(ctxt->om,
                                    CTE_MIN_LEN_MIN_VALUE,
                                    CTE_MIN_LEN_MAX_VALUE,
                                    &adv_cte_config.cte_min_length, NULL);
            break;

        default:
            break;
    }

    return rc;
}

static int ble_svc_cte_adv_cte_min_transmit_count_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc = BLE_ATT_ERR_UNLIKELY;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &adv_cte_config.cte_min_transmit_count,
                                sizeof(adv_cte_config.cte_min_transmit_count)) == 0 ?
                    0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_svc_cte_one_octet_chr_write(ctxt->om,
                                    CTE_MIN_TX_COUNT_MIN_VALUE,
                                    CTE_MIN_TX_COUNT_MAX_VALUE,
                                    &adv_cte_config.cte_min_transmit_count, NULL);
            break;

        default:
            break;
    }

    return rc;
}

static int ble_svc_cte_adv_cte_transmit_duration_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc = BLE_ATT_ERR_UNLIKELY;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &adv_cte_config.cte_transmit_duration,
                                sizeof(adv_cte_config.cte_transmit_duration)) == 0 ?
                    0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_svc_cte_one_octet_chr_write(ctxt->om,
                                    CTE_TX_DURATION_MIN_VALUE,
                                    CTE_TX_DURATION_MAX_VALUE,
                                    &adv_cte_config.cte_transmit_duration, NULL);
            break;

        default:
            break;
    }

    return rc;
}

static int ble_svc_cte_adv_cte_interval_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc = BLE_ATT_ERR_UNLIKELY;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &adv_cte_config.cte_interval,
                                sizeof(adv_cte_config.cte_interval)) == 0 ?
                    0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_svc_cte_two_octet_chr_write(ctxt->om,
                                    CTE_INTERVAL_MIN_VALUE,
                                    CTE_INTERVAL_MAX_VALUE,
                                    &adv_cte_config.cte_interval, NULL);
            break;

        default:
            break;
    }

    return rc;
}

static int ble_svc_cte_adv_cte_phy_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    int rc = BLE_ATT_ERR_UNLIKELY;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &adv_cte_config.cte_phy,
                                sizeof(adv_cte_config.cte_phy)) == 0 ?
                    0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            break;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = ble_svc_cte_one_octet_chr_write(ctxt->om,
                                    CTE_PHY_MIN_VALUE,
                                    CTE_PHY_MAX_VALUE,
                                    &adv_cte_config.cte_phy, NULL);
            break;

        default:
            break;
    }

    return rc;
}

static struct ble_gap_event_listener cte_gap_listener;

static int
cte_gap_event(struct ble_gap_event *event, void *arg)
{
    int i;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            for (i = 0; i < MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
                if (cte_config[i].conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                    /* Reset all per-connection fields so a new client never
                     * inherits state from a previous session or an unconnected
                     * write that slipped through the 0xffff sentinel. */
                    cte_config[i].conn_handle = event->connect.conn_handle;
                    cte_config[i].cte_enable = 0;
                    break;
                }
            }
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Skip events with an invalid handle — 0xffff is the unused-slot sentinel
         * and matching it would spuriously reset an already-empty entry. */
        if (event->disconnect.conn.conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            break;
        }
        for (i = 0; i < MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
            if (cte_config[i].conn_handle == event->disconnect.conn.conn_handle) {
                cte_config[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
                cte_config[i].cte_enable = 0;
                break;
            }
        }
        break;
    }

    return 0;
}

static void cte_init_config(void) {
    for (int i = 0; i < MYNEWT_VAL(BLE_MAX_CONNECTIONS); i++) {
        cte_config[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
        cte_config[i].cte_enable = 0;
    }
    adv_cte_config.cte_min_length = CTE_MIN_LEN_MIN_VALUE;
    adv_cte_config.cte_min_transmit_count = CTE_MIN_TX_COUNT_MIN_VALUE;
    adv_cte_config.cte_transmit_duration = CTE_TX_DURATION_MIN_VALUE;
    adv_cte_config.cte_interval = CTE_INTERVAL_MIN_VALUE;
    adv_cte_config.cte_phy = CTE_PHY_MIN_VALUE;
}

/**
 * @brief Initialize the Constant Tone Extension Service
 */
void
ble_svc_cte_init(void)
{
    int rc;
    /* Guard against repeated calls: ble_gatts_count_cfg and ble_gatts_add_svcs
     * are additive, so calling them more than once inflates resource counters
     * and inserts duplicate GATT service entries. */
    static int initialized = 0;
    if (initialized) {
        return;
    }
    initialized = 1;

    cte_init_config();
    ble_gap_event_listener_register(&cte_gap_listener, cte_gap_event, NULL);

    rc = ble_gatts_count_cfg(ble_svc_cte_defs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_cte_defs);
    assert(rc == 0);

}

#endif
