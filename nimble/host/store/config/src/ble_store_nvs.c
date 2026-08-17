/*
* Copyright 2015-2019 Espressif Systems (Shanghai) PTE LTD
*
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


#include "syscfg/syscfg.h"
#include "host/ble_hs_log.h"

#if MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST)

#include <string.h>
#include <esp_system.h>
#include "sysinit/sysinit.h"
#include "host/ble_hs.h"
#include "store/config/ble_store_config.h"
#include "ble_store_config_priv.h"
#include "esp_log.h"
#include "nvs.h"
#include "../../../src/ble_hs_resolv_priv.h"


#define NIMBLE_NVS_STR_NAME_MAX_LEN              16
#define NIMBLE_NVS_PEER_SEC_KEY                  "peer_sec"
#define NIMBLE_NVS_OUR_SEC_KEY                   "our_sec"
#define NIMBLE_NVS_CCCD_SEC_KEY                  "cccd_sec"
#define NIMBLE_NVS_CSFC_SEC_KEY                  "csfc_sec"
#define NIMBLE_NVS_PEER_RECORDS_KEY              "p_dev_rec"
#define NIMBLE_NVS_NAMESPACE                     "nimble_bond"

#if MYNEWT_VAL(ENC_ADV_DATA)
#define NIMBLE_NVS_EAD_SEC_KEY                   "ead_sec"
#endif

#define NIMBLE_NVS_LOCAL_IRK_KEY                "local_irk"
#define NIMBLE_NVS_RPA_RECORDS_KEY               "rpa_rec"

static const char *TAG = "NIMBLE_NVS";

static int get_nvs_max_obj_value(int obj_type);
static int get_nvs_db_value(nvs_handle_t nimble_handle, int obj_type, char *key_string,
                            union ble_store_value *val);

/*****************************************************************************
 * $ MISC                                                                    *
 *****************************************************************************/

static void
get_nvs_key_string(int obj_type, int index, char *key_string)
{
    if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
        snprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_PEER_RECORDS_KEY, index);
    } else {
        if (obj_type == BLE_STORE_OBJ_TYPE_PEER_SEC) {
            snprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_PEER_SEC_KEY, index);
        } else if (obj_type == BLE_STORE_OBJ_TYPE_OUR_SEC) {
            snprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_OUR_SEC_KEY, index);
#if MYNEWT_VAL(ENC_ADV_DATA)
        } else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
            snprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_EAD_SEC_KEY, index);
#endif
        } else if (obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK) {
            snprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_LOCAL_IRK_KEY, index);

        } else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_ADDR){
            snprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_RPA_RECORDS_KEY, index);
        } else if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
            snprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_CCCD_SEC_KEY, index);
        } else {
            snprintf(key_string, NIMBLE_NVS_STR_NAME_MAX_LEN, "%s_%d", NIMBLE_NVS_CSFC_SEC_KEY, index);
        }
    }
}

/* compares values at two addresses of size = item_size
* @Returns               index if entries match
*                       -1 if mismatch
*/
static int
get_nvs_matching_index(void *nvs_val, void *db_list, int db_num, size_t
                       item_size)
{
    uint8_t *db_item = (uint8_t *)db_list;
    int i;

    for (i = 0; i < db_num; i++) {
        if (memcmp(nvs_val, db_item, item_size) == 0) {
            /* Key matches with the one in RAM database */
            return i;
        }
        db_item += item_size;
    }
    return -1;
}

static int
get_nvs_sec_identity_index(nvs_handle_t nimble_handle,
                           const struct ble_store_value_sec *value_sec,
                           int obj_type)
{
    union ble_store_value cur = {0};
    esp_err_t err;
    int i;
    int max_limit;
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

    max_limit = get_nvs_max_obj_value(obj_type);

    for (i = 1; i <= max_limit; i++) {
        get_nvs_key_string(obj_type, i, key_string);
        err = get_nvs_db_value(nimble_handle, obj_type, key_string, &cur);
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            continue;
        } else if (err != ESP_OK) {
            return -1;
        }

        if (ble_addr_cmp(&cur.sec.peer_addr, &value_sec->peer_addr) == 0) {
            return i;
        }
    }

    return -1;
}

#if MYNEWT_VAL(ENC_ADV_DATA)
/* EAD entries contain a pointer (km), so raw memcmp() is not valid.
 * Match by stable key (peer_addr) to determine if an NVS entry exists in RAM.
 */
static int
get_nvs_matching_ead_index(const struct ble_store_value_ead *nvs_val,
                           void *db_list, int db_num)
{
    struct ble_store_value_ead *db_item = (struct ble_store_value_ead *)db_list;
    int i;

    for (i = 0; i < db_num; i++) {
        if (ble_addr_cmp(&nvs_val->peer_addr, &db_item->peer_addr) == 0) {
            return i;
        }
        db_item++;
    }

    return -1;
}

static int
ble_store_nvs_ead_cmp(const struct ble_store_value_ead *a,
                      const struct ble_store_value_ead *b)
{
    int rc;

    rc = ble_addr_cmp(&a->peer_addr, &b->peer_addr);
    if (rc != 0) {
        return rc;
    }

    if (a->km_present != b->km_present) {
        return a->km_present ? 1 : -1;
    }

    if (!a->km_present) {
        return 0;
    }

    rc = memcmp(a->km.session_key, b->km.session_key,
                sizeof a->km.session_key);
    if (rc != 0) {
        return rc;
    }

    return memcmp(a->km.iv, b->km.iv, sizeof a->km.iv);
}
#endif

static int
get_nvs_max_obj_value(int obj_type)
{
    /* If host based privacy is enabled */
    if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
        return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
    } else {
        if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
            return MYNEWT_VAL(BLE_STORE_MAX_CCCDS);
        } else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) {
            return MYNEWT_VAL(BLE_STORE_MAX_CSFCS);
#if MYNEWT_VAL(ENC_ADV_DATA)
        } else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
            return MYNEWT_VAL(BLE_STORE_MAX_EADS);
#endif
        } else {
            return MYNEWT_VAL(BLE_STORE_MAX_BONDS);
        }
    }
}

/*****************************************************************************
 * $ NVS                                                                     *
 *****************************************************************************/
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
static int
get_nvs_peer_record(nvs_handle_t nimble_handle, char *key_string, struct ble_hs_dev_records *p_dev_rec)
{
    esp_err_t err;
    size_t required_size = 0;

    err = nvs_get_blob(nimble_handle, key_string, NULL, &required_size);

    /* if Address pointer for value is NULL, filling of value not needed */
    if (err != ESP_OK || p_dev_rec == NULL) {
        return err;
    }

    /* Validate that NVS data size matches expected struct size */
    if (required_size != sizeof(struct ble_hs_dev_records)) {
        ESP_LOGE(TAG, "NVS data size mismatch for peer record: expected %d, got %d",
                 (int)sizeof(struct ble_hs_dev_records), (int)required_size);
        return ESP_ERR_NVS_INVALID_LENGTH;
    }

    err = nvs_get_blob(nimble_handle, key_string, p_dev_rec,
                       &required_size);

    return err;
}
#endif

static size_t
get_expected_size_for_obj_type(int obj_type)
{
    switch (obj_type) {
    case BLE_STORE_OBJ_TYPE_CCCD:
        return sizeof(struct ble_store_value_cccd);
    case BLE_STORE_OBJ_TYPE_CSFC:
        return sizeof(struct ble_store_value_csfc);
#if MYNEWT_VAL(ENC_ADV_DATA)
    case BLE_STORE_OBJ_TYPE_ENC_ADV_DATA:
        return sizeof(struct ble_store_value_ead);
#endif
    case BLE_STORE_OBJ_TYPE_LOCAL_IRK:
        return sizeof(struct ble_store_value_local_irk);
    case BLE_STORE_OBJ_TYPE_PEER_ADDR:
        return sizeof(struct ble_store_value_rpa_rec);
    case BLE_STORE_OBJ_TYPE_PEER_SEC:
    case BLE_STORE_OBJ_TYPE_OUR_SEC:
    default:
        return sizeof(struct ble_store_value_sec);
    }
}

static int
get_nvs_db_value(nvs_handle_t nimble_handle, int obj_type, char *key_string, union ble_store_value *val)
{
    esp_err_t err;
    size_t required_size = 0;
    size_t expected_size;

    err = nvs_get_blob(nimble_handle, key_string, NULL, &required_size);

    /* if Address pointer for value is NULL, filling of value not needed */
    if (err != ESP_OK || val == NULL) {
        return err;
    }

    /* Validate that NVS data size matches expected struct size */
    expected_size = get_expected_size_for_obj_type(obj_type);
    if (required_size != expected_size) {
        ESP_LOGE(TAG, "NVS data size mismatch for obj_type %d: expected %d, got %d",
                 obj_type, (int)expected_size, (int)required_size);
        return ESP_ERR_NVS_INVALID_LENGTH;
    }

    if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
        err = nvs_get_blob(nimble_handle, key_string, &val->cccd,
                           &required_size);
    } else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) {
        err = nvs_get_blob(nimble_handle, key_string, &val->csfc,
                           &required_size);
#if MYNEWT_VAL(ENC_ADV_DATA)
    } else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
        err = nvs_get_blob(nimble_handle, key_string, &val->ead,
                           &required_size);
#endif
    } else if (obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK) {
        err = nvs_get_blob(nimble_handle, key_string, &val->local_irk,
                           &required_size);

    } else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_ADDR) {
         err = nvs_get_blob(nimble_handle, key_string, &val->rpa_rec,
                           &required_size);

    } else {
        err = nvs_get_blob(nimble_handle, key_string, &val->sec,
                           &required_size);
    }

    return err;
}

/* Finds empty index or total count or index to be deleted in NVS database
* This function serves 3 different purposes depending upon 'empty' and `value`
* arguments.
* @ returns             - empty NVS index, if empty = 1
*                       - count of NVS database, if empty = 0, value = NULL
*                       - index that does not match with RAM db, if empty = 0 &
*                         value has valid database address.
*/
static int
get_nvs_db_attribute(nvs_handle_t nimble_handle, int obj_type, bool empty, void *value, int num_value)
{
    union ble_store_value cur = {0};
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    struct ble_hs_dev_records p_dev_rec = {0};
#endif
    esp_err_t err;
    int i, count = 0, max_limit = 0;
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

    max_limit = get_nvs_max_obj_value(obj_type);

    for (i = 1; i <= max_limit; i++) {
        get_nvs_key_string(obj_type, i, key_string);

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
        if (obj_type != BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
#endif
            err = get_nvs_db_value(nimble_handle, obj_type, key_string, &cur);
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
        } else {
            err = get_nvs_peer_record(nimble_handle, key_string, &p_dev_rec);
        }
#endif
        /* Check if the user is searching for empty index to write to */
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            if (empty) {
                ESP_LOGD(TAG, "Empty NVS index found = %d for obj_type = %d", i, obj_type);
                return i;
            }
        } else if (err == ESP_OK) {
            count++;
            /* If user has provided value, then the purpose is to find
             * non-matching entry from NVS */
            if (value) {
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
                if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
                    struct ble_hs_dev_records *recs = value;
                    err = -1;
                    for (int j = 0; j < num_value; j++) {
                        if (memcmp(&p_dev_rec.peer_sec, &recs[j].peer_sec, sizeof(struct ble_hs_peer_sec)) == 0) {
                            err = j;
                            break;
                        }
                    }
                } else
#endif
                {
                    if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
                        err = get_nvs_matching_index(&cur.cccd, value, num_value,
                                                     sizeof(struct ble_store_value_cccd));
                    } else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) {
                        err = get_nvs_matching_index(&cur.csfc, value, num_value,
                                                     sizeof(struct ble_store_value_csfc));
#if MYNEWT_VAL(ENC_ADV_DATA)
                    } else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
                        err = get_nvs_matching_ead_index(&cur.ead, value,
                                                         num_value);
#endif
                   } else if (obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK) {
                       err = get_nvs_matching_index(&cur.local_irk, value, num_value,
                                                    sizeof(struct ble_store_value_local_irk));

                   } else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_ADDR){
                        err = get_nvs_matching_index(&cur.rpa_rec,value,num_value,
                                                     sizeof(struct ble_store_value_rpa_rec));
                    } else {
                        err = get_nvs_matching_index(&cur.sec, value, num_value,
                                                     sizeof(struct ble_store_value_sec));
                    }
                }
                /* If found non-matching/odd entry of NVS with entries in the
                 * internal database, return NVS index so can be deleted */
                if (err == -1 && !empty) {
                    return i;
                }
            }
        } else {
            ESP_LOGE(TAG, "NVS read operation failed while fetching size !!");
            return -1;
        }
    }

    if (empty == 0) {
        if (value != NULL) {
            /* No non-matching entry found */
            return -1;
        }
        return count;
    } else {
        return (max_limit + 1);
    }
}

static int
ble_store_nvs_read_check(nvs_handle_t nimble_handle, int obj_type)
{
    union ble_store_value cur = {0};
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    struct ble_hs_dev_records p_dev_rec = {0};
#endif
    esp_err_t err;
    int i;
    int max_limit;
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

    max_limit = get_nvs_max_obj_value(obj_type);

    for (i = 1; i <= max_limit; i++) {
        get_nvs_key_string(obj_type, i, key_string);

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
        if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
            err = get_nvs_peer_record(nimble_handle, key_string, &p_dev_rec);
        } else
#endif
        {
            err = get_nvs_db_value(nimble_handle, obj_type, key_string, &cur);
        }

        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGE(TAG, "NVS read operation failed while checking obj_type %d",
                     obj_type);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            return BLE_HS_ESTORE_FAIL;
        }
    }

    return 0;
}

/* Deletes NVS value at given index
* @Returns               0 on success,
*                       -1 on NVS memory access failure
*/
static int
ble_nvs_delete_value(nvs_handle_t nimble_handle, int obj_type, int index)
{
    esp_err_t err;
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

    if (index <= 0 || index > get_nvs_max_obj_value(obj_type)) {
        ESP_LOGE(TAG, "Invalid index provided to delete");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_EUNKNOWN);
        return BLE_HS_EUNKNOWN;
    }

    get_nvs_key_string(obj_type, index, key_string);

    /* Erase the key with given index */
    err = nvs_erase_key(nimble_handle, key_string);
    if (err != ESP_OK) {
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    return 0;
}

static int
ble_nvs_write_key_value(nvs_handle_t nimble_handle, char *key, const void *value, size_t required_size)
{
    esp_err_t err;

    err = nvs_set_blob(nimble_handle, key, value, required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS write operation failed !!");
        if (err == ESP_ERR_NVS_NOT_ENOUGH_SPACE || err == ESP_ERR_NVS_NO_FREE_PAGES) {
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
            return BLE_HS_ESTORE_CAP;
        }
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    return 0;
}

static int
ble_nvs_commit_checked(nvs_handle_t nimble_handle)
{
    esp_err_t err;

    err = nvs_commit(nimble_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit operation failed !!");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    return 0;
}

/* To write key value in NVS.
* @Returns              0 if success
*                       BLE_HS_ESTORE_FAIL if failure
*                       BLE_HS_ESTORE_CAP if no space in NVS
*/
static int
ble_store_nvs_write(nvs_handle_t nimble_handle, int obj_type, const union ble_store_value *val)
{
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
    int write_key_index = 0;

    write_key_index = get_nvs_db_attribute(nimble_handle, obj_type, 1, NULL, 0);
    if (write_key_index == -1) {
        ESP_LOGE(TAG, "NVS operation failed !!");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    } else if (write_key_index > get_nvs_max_obj_value(obj_type)) {

        /* bare-bone config code will take care of capacity overflow event,
         * however another check added for consistency */
        ESP_LOGD(TAG, "NVS size overflow.");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
        return BLE_HS_ESTORE_CAP;
    }

    get_nvs_key_string(obj_type, write_key_index, key_string);

    if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->cccd, sizeof(struct
                                       ble_store_value_cccd));
    } else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->csfc, sizeof(struct
                                       ble_store_value_csfc));
#if MYNEWT_VAL(ENC_ADV_DATA)
    } else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->ead, sizeof(struct
                                       ble_store_value_ead));
#endif
    } else if (obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->local_irk, sizeof(struct
                                   ble_store_value_local_irk));

    } else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_ADDR) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->rpa_rec, sizeof(struct
                                       ble_store_value_rpa_rec));

    } else {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->sec, sizeof(struct
                                       ble_store_value_sec));
    }
}

static int
ble_store_nvs_update(nvs_handle_t nimble_handle, int obj_type, int index, const union ble_store_value *val)
{
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
    int max_limit;

    max_limit = get_nvs_max_obj_value(obj_type);
    if (index <= 0 || index > max_limit) {
        return BLE_HS_EUNKNOWN;
    }

    get_nvs_key_string(obj_type, index, key_string);

    if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->cccd,
                                       sizeof(struct ble_store_value_cccd));
    } else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->csfc,
                                       sizeof(struct ble_store_value_csfc));
#if MYNEWT_VAL(ENC_ADV_DATA)
    } else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->ead,
                                       sizeof(struct ble_store_value_ead));
#endif
    } else if (obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->local_irk,
                                       sizeof(struct ble_store_value_local_irk));
    } else if (obj_type == BLE_STORE_OBJ_TYPE_PEER_ADDR) {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->rpa_rec,
                                       sizeof(struct ble_store_value_rpa_rec));
    } else {
        return ble_nvs_write_key_value(nimble_handle, key_string, &val->sec,
                                       sizeof(struct ble_store_value_sec));
    }
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
/* If Host based privacy is enabled */
static int
ble_store_nvs_peer_records(nvs_handle_t nimble_handle, int obj_type, const struct ble_hs_dev_records *p_dev_rec)
{
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
    int write_key_index = 0;

    write_key_index = get_nvs_db_attribute(nimble_handle, obj_type, 1, NULL, 0);
    if (write_key_index == -1) {
        ESP_LOGE(TAG, "NVS operation failed !!");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    } else if (write_key_index > get_nvs_max_obj_value(obj_type)) {

        /* bare-bone config code will take care of capacity overflow event,
         * however another check added for consistency */
        ESP_LOGD(TAG, "NVS size overflow.");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_CAP);
        return BLE_HS_ESTORE_CAP;
    }

    get_nvs_key_string(obj_type, write_key_index, key_string);

    return ble_nvs_write_key_value(nimble_handle, key_string, p_dev_rec, sizeof(struct
                                   ble_hs_dev_records));
}

static int
ble_store_nvs_peer_records_update(nvs_handle_t nimble_handle, int obj_type, int index,
                                  const struct ble_hs_dev_records *p_dev_rec)
{
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
    int max_limit;

    max_limit = get_nvs_max_obj_value(obj_type);
    if (index <= 0 || index > max_limit) {
        return BLE_HS_EUNKNOWN;
    }

    get_nvs_key_string(obj_type, index, key_string);
    return ble_nvs_write_key_value(nimble_handle, key_string, p_dev_rec,
                                   sizeof(struct ble_hs_dev_records));
}
#endif

static int
populate_db_from_nvs(nvs_handle_t nimble_handle, int obj_type, void *dst, int *db_num)
{
    uint8_t *db_item = (uint8_t *)dst;
    union ble_store_value cur = {0};
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    struct ble_hs_dev_records p_dev_rec = {0};
#endif

    esp_err_t err;
    int i;
    int max_entries;
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

    /* Get the maximum number of entries allowed for this object type */
    max_entries = get_nvs_max_obj_value(obj_type);

    for (i = 1; i <= max_entries; i++) {
        get_nvs_key_string(obj_type, i, key_string);

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
        if (obj_type != BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
#endif
            err = get_nvs_db_value(nimble_handle, obj_type, key_string, &cur);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                continue;
            } else if (err != ESP_OK) {
                ESP_LOGE(TAG, "NVS read operation failed !!");
                BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
                return BLE_HS_ESTORE_FAIL;
            }
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
        } else {
            err = get_nvs_peer_record(nimble_handle, key_string, &p_dev_rec);
            if (err == ESP_ERR_NVS_NOT_FOUND) {
                continue;
            } else if (err != ESP_OK) {
                ESP_LOGE(TAG, "NVS read operation failed !!");
                BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
                return BLE_HS_ESTORE_FAIL;
            }
        }

        /* NVS index has data, fill up the ram db with it */
        if (obj_type == BLE_STORE_OBJ_TYPE_PEER_DEV_REC) {
            /* Bounds check before writing to RAM array */
            if (*db_num >= max_entries) {
                ESP_LOGW(TAG, "Peer dev records: RAM array full, skipping NVS index %d", i);
                continue;
            }
            ESP_LOGD(TAG, "Peer dev records filled from NVS index = %d", i);
            memcpy(db_item, &p_dev_rec, sizeof(struct ble_hs_dev_records));
            db_item += sizeof(struct ble_hs_dev_records);
            (*db_num)++;
        } else
#endif
        {
            if (obj_type == BLE_STORE_OBJ_TYPE_CCCD) {
                /* Bounds check before writing to RAM array */
                if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_CCCDS)) {
                    ESP_LOGW(TAG, "CCCD: RAM array full, skipping NVS index %d", i);
                    continue;
                }
                ESP_LOGD(TAG, "CCCD in RAM is filled up from NVS index = %d", i);
                memcpy(db_item, &cur.cccd, sizeof(struct ble_store_value_cccd));
                db_item += sizeof(struct ble_store_value_cccd);
                (*db_num)++;
            } else if (obj_type == BLE_STORE_OBJ_TYPE_CSFC) {
                /* Bounds check before writing to RAM array */
                if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_CSFCS)) {
                    ESP_LOGW(TAG, "CSFC: RAM array full, skipping NVS index %d", i);
                    continue;
                }
                ESP_LOGD(TAG, "CSFC in RAM is filled up from NVS index = %d", i);
                memcpy(db_item, &cur.csfc, sizeof(struct ble_store_value_csfc));
                db_item += sizeof(struct ble_store_value_csfc);
                (*db_num)++;
#if MYNEWT_VAL(ENC_ADV_DATA)
            } else if (obj_type == BLE_STORE_OBJ_TYPE_ENC_ADV_DATA) {
                  /* Bounds check before writing to RAM array */
                  if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_EADS)) {
                      ESP_LOGW(TAG, "EAD: RAM array full, skipping NVS index %d", i);
                      continue;
                  }
                  ESP_LOGD(TAG, "EAD in RAM is filled up from NVS index = %d", i);
                  memcpy(db_item, &cur.ead, sizeof(struct ble_store_value_ead));
                  db_item += sizeof(struct ble_store_value_ead);
                  (*db_num)++;
#endif
           } else if(obj_type == BLE_STORE_OBJ_TYPE_LOCAL_IRK) {
                  /* Bounds check before writing to RAM array */
                  if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_BONDS)) {
                      ESP_LOGW(TAG, "Local IRK: RAM array full, skipping NVS index %d", i);
                      continue;
                  }
                  ESP_LOGD(TAG, "Local IRK in RAM is filled up from NVS index = %d", i);
                  memcpy(db_item, &cur.local_irk, sizeof(struct ble_store_value_local_irk));
                  db_item += sizeof(struct ble_store_value_local_irk);
                  (*db_num)++;

            } else if(obj_type == BLE_STORE_OBJ_TYPE_PEER_ADDR) {
                  /* Bounds check before writing to RAM array */
                  if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_BONDS)) {
                      ESP_LOGW(TAG, "RPA_REC: RAM array full, skipping NVS index %d", i);
                      continue;
                  }
                  ESP_LOGD(TAG, "RPA_REC in RAM is filled up from NVS index = %d", i);
                  memcpy(db_item, &cur.rpa_rec, sizeof(struct ble_store_value_rpa_rec));
                  db_item += sizeof(struct ble_store_value_rpa_rec);
                  (*db_num)++;
            } else {
                /* Bounds check before writing to RAM array (sec type) */
                if (*db_num >= MYNEWT_VAL(BLE_STORE_MAX_BONDS)) {
                    ESP_LOGW(TAG, "SEC: RAM array full, skipping NVS index %d", i);
                    continue;
                }
                ESP_LOGD(TAG, "KEY in RAM is filled up from NVS index = %d", i);
                memcpy(db_item, &cur.sec, sizeof(struct ble_store_value_sec));
                db_item += sizeof(struct ble_store_value_sec);
                (*db_num)++;
            }
        }
    }
    return 0;
}

/* Gets the database in RAM filled up with keys stored in NVS. The sequence of
 * the keys in database may get lost.
 */
static int
ble_nvs_restore_sec_keys(void)
{
    int err;
    int flag = 0;
    nvs_handle_t nimble_handle;
    extern int ble_store_config_compare_bond_count(const void *a, const void *b);

    err = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return err;
    }

#if MYNEWT_VAL(BLE_STORE_MAX_BONDS)
    err = populate_db_from_nvs(nimble_handle, BLE_STORE_OBJ_TYPE_OUR_SEC, ble_store_config_our_secs,
                               &ble_store_config_num_our_secs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'our sec'");
        goto end;
    }
    ESP_LOGD(TAG, "ble_store_config_our_secs restored %d bonds", ble_store_config_num_our_secs);

    err = populate_db_from_nvs(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_SEC, ble_store_config_peer_secs,
                               &ble_store_config_num_peer_secs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'peer sec'");
        goto end;
    }

    /* Only check for out-of-order entries if we have valid data */
    if (ble_store_config_num_our_secs > 1 || ble_store_config_num_peer_secs > 1) {
        /* Use actual populated count, not max config value */
        int our_check_limit = (ble_store_config_num_our_secs > 1) ? (ble_store_config_num_our_secs - 1) : 0;
        int peer_check_limit = (ble_store_config_num_peer_secs > 1) ? (ble_store_config_num_peer_secs - 1) : 0;
        int check_limit = (our_check_limit > peer_check_limit) ? our_check_limit : peer_check_limit;

        for (int i = 0; i < check_limit; i++) {
            if ((i < our_check_limit &&
                 ble_store_config_our_secs[i].bond_count > ble_store_config_our_secs[i+1].bond_count) ||
                (i < peer_check_limit &&
                 ble_store_config_peer_secs[i].bond_count > ble_store_config_peer_secs[i+1].bond_count)) {
                flag = 1;
                break;
            }
        }
    }

    if (flag) {

        qsort(ble_store_config_our_secs, ble_store_config_num_our_secs,
            sizeof(struct ble_store_value_sec), ble_store_config_compare_bond_count);

        qsort(ble_store_config_peer_secs, ble_store_config_num_peer_secs,
            sizeof(struct ble_store_value_sec), ble_store_config_compare_bond_count);
    }

    /* Only access array if we have valid entries to prevent index -1 access */
    if (ble_store_config_num_our_secs > 0) {
        ble_store_config_our_bond_count = ble_store_config_our_secs[ble_store_config_num_our_secs - 1].bond_count;
    } else {
        ble_store_config_our_bond_count = 0;
    }

    if (ble_store_config_num_peer_secs > 0) {
        ble_store_config_peer_bond_count = ble_store_config_peer_secs[ble_store_config_num_peer_secs - 1].bond_count;
    } else {
        ble_store_config_peer_bond_count = 0;
    }

    ESP_LOGD(TAG, "ble_store_config_peer_secs restored %d bonds",
             ble_store_config_num_peer_secs);
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
    err = populate_db_from_nvs(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD, ble_store_config_cccds,
                               &ble_store_config_num_cccds);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'CCCD'");
        goto end;
    }
    ESP_LOGD(TAG, "ble_store_config_cccds restored %d bonds",
             ble_store_config_num_cccds);
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
    err = populate_db_from_nvs(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC, ble_store_config_csfcs,
                               &ble_store_config_num_csfcs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'CSFC'");
        goto end;
    }
    ESP_LOGD(TAG, "ble_store_config_csfcs restored %d bonds",
             ble_store_config_num_csfcs);
#endif

#if MYNEWT_VAL(ENC_ADV_DATA)
    err = populate_db_from_nvs(nimble_handle, BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, ble_store_config_eads,
                               &ble_store_config_num_eads);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'EAD'");
        goto end;
    }
    ESP_LOGD(TAG, "ble_store_config_eads restored %d bonds",
             ble_store_config_num_eads);
#endif
    err = populate_db_from_nvs(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK, ble_store_config_local_irks,
                           &ble_store_config_num_local_irks);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'Local IRK'");
        goto end;
    }
    ESP_LOGD(TAG, "ble_store_config_local_irks restored %d irks",
             ble_store_config_num_local_irks);

    err = populate_db_from_nvs(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR, ble_store_config_rpa_recs,
                               &ble_store_config_num_rpa_recs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed for 'RPA_REC'");
        goto end;
    }
    ESP_LOGD(TAG, "ble_store_config_rpa_recs restored %d bonds",
             ble_store_config_num_rpa_recs);

    err = 0;
end:
    nvs_close(nimble_handle);
    return err;
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
static int
ble_nvs_restore_peer_records(void)
{
    esp_err_t err;
    int ble_store_num_peer_dev_rec = 0;
    nvs_handle_t nimble_handle;
    struct ble_hs_dev_records *peer_dev_rec = ble_rpa_get_peer_dev_records();
    if (peer_dev_rec == NULL) {
        return BLE_HS_ENOMEM;
    }

    err = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return err;
    }

    err = populate_db_from_nvs(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_DEV_REC, peer_dev_rec,
                               &ble_store_num_peer_dev_rec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS operation failed fetching 'Peer Dev Records'");
        nvs_close(nimble_handle);
        return err;
    }

    nvs_close(nimble_handle);
    ble_rpa_set_num_peer_dev_records(ble_store_num_peer_dev_rec);
    ESP_LOGD(TAG, "peer_dev_rec restored %d records", ble_store_num_peer_dev_rec);

    return 0;
}
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CCCDS)
int ble_store_config_persist_cccds(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;
    int rc, i;
    nvs_handle_t nimble_handle;

    rc = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    nvs_count = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD, 0, NULL, 0);
    if (nvs_count == -1) {
        ESP_LOGE(TAG, "NVS operation failed while persisting CCCD");
        nvs_close(nimble_handle);
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    if (nvs_count < ble_store_config_num_cccds) {

        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting CCCD value in NVS...");
        val.cccd = ble_store_config_cccds[ble_store_config_num_cccds - 1];
        rc = ble_store_nvs_write(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD, &val);
    } else if (nvs_count > ble_store_config_num_cccds) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD, 0,
                                       ble_store_config_cccds, ble_store_config_num_cccds);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for CCCD");
            nvs_close(nimble_handle);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting CCCD, nvs idx = %d", nvs_idx);
        rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD, nvs_idx);
    } else {
        union ble_store_value nvs_val = {0};
        char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

        if (ble_store_config_num_cccds == 0) {
            nvs_close(nimble_handle);
            return 0;
        }

        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD, 0,
                                       ble_store_config_cccds,
                                       ble_store_config_num_cccds);
        if (nvs_idx == -1) {
            rc = ble_store_nvs_read_check(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD);
            goto end;
        }

        get_nvs_key_string(BLE_STORE_OBJ_TYPE_CCCD, nvs_idx, key_string);
        rc = get_nvs_db_value(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD, key_string, &nvs_val);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "NVS read operation failed for CCCD update");
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            rc = BLE_HS_ESTORE_FAIL;
            goto end;
        }

        for (i = 0; i < ble_store_config_num_cccds; i++) {
            if (ble_addr_cmp(&ble_store_config_cccds[i].peer_addr,
                             &nvs_val.cccd.peer_addr) == 0 &&
                ble_store_config_cccds[i].chr_val_handle ==
                nvs_val.cccd.chr_val_handle) {
                val.cccd = ble_store_config_cccds[i];
                rc = ble_store_nvs_update(nimble_handle, BLE_STORE_OBJ_TYPE_CCCD,
                                            nvs_idx, &val);
                goto end;
            }
        }

        ESP_LOGE(TAG, "NVS update operation failed for CCCD");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        rc = BLE_HS_ESTORE_FAIL;
    }

end:
    if (rc == 0) {
        rc = ble_nvs_commit_checked(nimble_handle);
    }
    nvs_close(nimble_handle);
    return rc;
}
#endif

#if MYNEWT_VAL(BLE_STORE_MAX_CSFCS)
int ble_store_config_persist_csfcs(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;
    int rc, i;
    nvs_handle_t nimble_handle;

    rc = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    nvs_count = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC, 0, NULL, 0);
    if (nvs_count == -1) {
        ESP_LOGE(TAG, "NVS operation failed while persisting CSFC");
        nvs_close(nimble_handle);
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    if (nvs_count < ble_store_config_num_csfcs) {
        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting CSFC value in NVS...");
        val.csfc = ble_store_config_csfcs[ble_store_config_num_csfcs - 1];
        rc = ble_store_nvs_write(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC, &val);
    } else if (nvs_count > ble_store_config_num_csfcs) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC, 0,
                                       ble_store_config_csfcs, ble_store_config_num_csfcs);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for CSFC");
            nvs_close(nimble_handle);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting CSFC, nvs idx = %d", nvs_idx);
        rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC, nvs_idx);
    } else {
        union ble_store_value nvs_val = {0};
        char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

        if (ble_store_config_num_csfcs == 0) {
            nvs_close(nimble_handle);
            return 0;
        }

        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC, 0,
                                       ble_store_config_csfcs,
                                       ble_store_config_num_csfcs);
        if (nvs_idx == -1) {
            rc = ble_store_nvs_read_check(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC);
            goto end;
        }

        get_nvs_key_string(BLE_STORE_OBJ_TYPE_CSFC, nvs_idx, key_string);
        rc = get_nvs_db_value(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC, key_string, &nvs_val);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "NVS read operation failed for CSFC update");
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            rc = BLE_HS_ESTORE_FAIL;
            goto end;
        }

        for (i = 0; i < ble_store_config_num_csfcs; i++) {
            if (ble_addr_cmp(&ble_store_config_csfcs[i].peer_addr,
                             &nvs_val.csfc.peer_addr) == 0) {
                val.csfc = ble_store_config_csfcs[i];
                rc = ble_store_nvs_update(nimble_handle, BLE_STORE_OBJ_TYPE_CSFC,
                                            nvs_idx, &val);
                goto end;
            }
        }

        ESP_LOGE(TAG, "NVS update operation failed for CSFC");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        rc = BLE_HS_ESTORE_FAIL;
    }

end:
    if (rc == 0) {
        rc = ble_nvs_commit_checked(nimble_handle);
    }
    nvs_close(nimble_handle);
    return rc;
}
#endif

#if MYNEWT_VAL(ENC_ADV_DATA)
int ble_store_config_persist_eads(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;
    int rc, i;
    nvs_handle_t nimble_handle;

    rc = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    nvs_count = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, 0, NULL, 0);
    if (nvs_count == -1) {
        ESP_LOGE(TAG, "NVS operation failed while persisting EAD");
        nvs_close(nimble_handle);
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    } 

    if (nvs_count < ble_store_config_num_eads) {
        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting EAD value in NVS...");
        val.ead = ble_store_config_eads[ble_store_config_num_eads - 1];
        rc = ble_store_nvs_write(nimble_handle, BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, &val);
    } else if (nvs_count > ble_store_config_num_eads) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, 0,
                                       ble_store_config_eads, ble_store_config_num_eads);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for EAD");
            nvs_close(nimble_handle);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting EAD, nvs idx = %d", nvs_idx);
        rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, nvs_idx);
    } else {
        union ble_store_value nvs_val = {0};
        bool ram_matched[MYNEWT_VAL(BLE_STORE_MAX_EADS)] = {0};
        char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
        int stale_nvs_idx = -1;
        int write_rc = 0;
        int max_limit;
        int j;

        if (ble_store_config_num_eads == 0) {
            nvs_close(nimble_handle);
            return 0;
        }

        max_limit = get_nvs_max_obj_value(BLE_STORE_OBJ_TYPE_ENC_ADV_DATA);
        for (i = 1; i <= max_limit; i++) {
            get_nvs_key_string(BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, i, key_string);
            rc = get_nvs_db_value(nimble_handle, BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, key_string,
                                  &nvs_val);
            if (rc == ESP_ERR_NVS_NOT_FOUND) {
                continue;
            } else if (rc != ESP_OK) {
                ESP_LOGE(TAG, "NVS read operation failed for EAD update");
                BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
                nvs_close(nimble_handle);
                return BLE_HS_ESTORE_FAIL;
            }

            for (j = 0; j < ble_store_config_num_eads; j++) {
                if (ble_addr_cmp(&ble_store_config_eads[j].peer_addr,
                                 &nvs_val.ead.peer_addr) == 0) {
                    ram_matched[j] = true;
                    if (ble_store_nvs_ead_cmp(&ble_store_config_eads[j],
                                              &nvs_val.ead) != 0) {
                        val.ead = ble_store_config_eads[j];
                        rc = ble_store_nvs_update(nimble_handle,
                                BLE_STORE_OBJ_TYPE_ENC_ADV_DATA, i, &val);
                        if (rc != 0) {
                            ESP_LOGE(TAG, "NVS write operation failed while updating EAD");
                            write_rc = rc;
                        }
                    }
                    break;
                }
            }

            if (j == ble_store_config_num_eads && stale_nvs_idx == -1) {
                stale_nvs_idx = i;
            }
        }

        for (i = 0; i < ble_store_config_num_eads; i++) {
            if (!ram_matched[i]) {
                if (stale_nvs_idx == -1) {
                    ESP_LOGE(TAG, "NVS update operation failed for EAD");
                    BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
                    nvs_close(nimble_handle);
                    return BLE_HS_ESTORE_FAIL;
                }

                val.ead = ble_store_config_eads[i];
                rc = ble_store_nvs_update(nimble_handle, BLE_STORE_OBJ_TYPE_ENC_ADV_DATA,
                                          stale_nvs_idx, &val);
                if (rc != 0) {
                    ESP_LOGE(TAG, "NVS write operation failed while updating EAD");
                    write_rc = rc;
                }
                break;
            }
        }

        rc = write_rc;
    }

    if (rc == 0) {
        rc = ble_nvs_commit_checked(nimble_handle);
    }
    nvs_close(nimble_handle);
    return rc;
}
#endif
int ble_store_config_persist_local_irk(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;
    int rc, i;
    nvs_handle_t nimble_handle;

    rc = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    nvs_count = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK, 0, NULL, 0);
    if (nvs_count == -1) {
        ESP_LOGE(TAG, "NVS operation failed while persisting Local IRK");
        nvs_close(nimble_handle);
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    if (nvs_count < ble_store_config_num_local_irks) {
        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting Local IRK value in NVS...");
        val.local_irk = ble_store_config_local_irks[ble_store_config_num_local_irks-1];
        rc = ble_store_nvs_write(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK, &val);
    } else if (nvs_count > ble_store_config_num_local_irks) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK, 0,
                                       ble_store_config_local_irks, ble_store_config_num_local_irks);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for Local IRK");
            nvs_close(nimble_handle);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting Local IRK, nvs idx = %d", nvs_idx);
        rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK, nvs_idx);
    } else {
        union ble_store_value nvs_val = {0};
        char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

        if (ble_store_config_num_local_irks == 0) {
            nvs_close(nimble_handle);
            return 0;
        }

        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK, 0,
                                       ble_store_config_local_irks,
                                       ble_store_config_num_local_irks);
        if (nvs_idx == -1) {
            rc = ble_store_nvs_read_check(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK);
            goto end;
        }

        get_nvs_key_string(BLE_STORE_OBJ_TYPE_LOCAL_IRK, nvs_idx, key_string);
        rc = get_nvs_db_value(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK, key_string, &nvs_val);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "NVS read operation failed for Local IRK update");
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            rc = BLE_HS_ESTORE_FAIL;
            goto end;
        }

        for (i = 0; i < ble_store_config_num_local_irks; i++) {
            if (ble_addr_cmp(&ble_store_config_local_irks[i].addr,
                             &nvs_val.local_irk.addr) == 0) {
                val.local_irk = ble_store_config_local_irks[i];
                rc = ble_store_nvs_update(nimble_handle, BLE_STORE_OBJ_TYPE_LOCAL_IRK,
                                            nvs_idx, &val);
                goto end;
            }
        }

        ESP_LOGE(TAG, "NVS update operation failed for Local IRK");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        rc = BLE_HS_ESTORE_FAIL;
    }

end:
    if (rc == 0) {
        rc = ble_nvs_commit_checked(nimble_handle);
    }
    nvs_close(nimble_handle);
    return rc;
}

int ble_store_config_persist_rpa_recs(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;
    int rc, i;
    union ble_store_value nvs_val = {0};
    char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
    nvs_handle_t nimble_handle;

    rc = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    nvs_count = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR, 0, NULL, 0);
     if (nvs_count == -1) {
        ESP_LOGE(TAG, "NVS operation failed while persisting RPA_RECS");
        nvs_close(nimble_handle);
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }
    if (nvs_count < ble_store_config_num_rpa_recs) {
        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting RPA_RECS value in NVS...");
        val.rpa_rec = ble_store_config_rpa_recs[ble_store_config_num_rpa_recs - 1];
        rc = ble_store_nvs_write(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR, &val);
    } else if (nvs_count > ble_store_config_num_rpa_recs) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR, 0,
                                       ble_store_config_rpa_recs, ble_store_config_num_rpa_recs);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for RPA_REC");
            nvs_close(nimble_handle);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting RPA_REC, nvs idx = %d", nvs_idx);
        rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR, nvs_idx);
    } else {
        if (ble_store_config_num_rpa_recs == 0) {
            nvs_close(nimble_handle);
            return 0;
        }

        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR, 0,
                                       ble_store_config_rpa_recs,
                                       ble_store_config_num_rpa_recs);
        if (nvs_idx == -1) {
            rc = ble_store_nvs_read_check(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR);
            goto end;
        }

        get_nvs_key_string(BLE_STORE_OBJ_TYPE_PEER_ADDR, nvs_idx, key_string);
        rc = get_nvs_db_value(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR, key_string, &nvs_val);
        if (rc != ESP_OK) {
            ESP_LOGE(TAG, "NVS read operation failed for RPA_REC update");
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            rc = BLE_HS_ESTORE_FAIL;
            goto end;
        }

        for (i = 0; i < ble_store_config_num_rpa_recs; i++) {
            if (ble_addr_cmp(&ble_store_config_rpa_recs[i].peer_addr,
                             &nvs_val.rpa_rec.peer_addr) == 0 ||
                ble_addr_cmp(&ble_store_config_rpa_recs[i].peer_rpa_addr,
                             &nvs_val.rpa_rec.peer_rpa_addr) == 0) {
                val.rpa_rec = ble_store_config_rpa_recs[i];
                rc = ble_store_nvs_update(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_ADDR,
                                            nvs_idx, &val);
                goto end;
            }
        }

        ESP_LOGE(TAG, "NVS update operation failed for RPA_REC");
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        rc = BLE_HS_ESTORE_FAIL;
    }

end:
    if (rc == 0) {
        rc = ble_nvs_commit_checked(nimble_handle);
    }
    nvs_close(nimble_handle);
    return rc;
}

int ble_store_config_persist_peer_secs(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;
    int rc, i;
    nvs_handle_t nimble_handle;

    rc = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    nvs_count = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_SEC, 0, NULL, 0);
    if (nvs_count == -1) {
        ESP_LOGE(TAG, "NVS operation failed while persisting peer sec");
        nvs_close(nimble_handle);
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    if (nvs_count < ble_store_config_num_peer_secs) {

        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting peer sec value in NVS...");
        val.sec = ble_store_config_peer_secs[ble_store_config_num_peer_secs - 1];
        rc = ble_store_nvs_write(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_SEC, &val);
    } else if (nvs_count > ble_store_config_num_peer_secs) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_SEC, 0,
                                       ble_store_config_peer_secs, ble_store_config_num_peer_secs);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for peer sec");
            nvs_close(nimble_handle);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting peer sec, nvs idx = %d", nvs_idx);
        rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_SEC, nvs_idx);
    } else {
        if (ble_store_config_num_peer_secs == 0) {
            nvs_close(nimble_handle);
            return 0;
        }

        /* Sync updates: Find which record changed and update it */
        for (i = 0; i < ble_store_config_num_peer_secs; i++) {
            val.sec = ble_store_config_peer_secs[i];
            nvs_idx = get_nvs_sec_identity_index(nimble_handle, &val.sec, BLE_STORE_OBJ_TYPE_PEER_SEC);
            if (nvs_idx != -1) {
                union ble_store_value nvs_val = {0};
                char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
                get_nvs_key_string(BLE_STORE_OBJ_TYPE_PEER_SEC, nvs_idx, key_string);
                if (get_nvs_db_value(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_SEC, key_string, &nvs_val) == 0) {
                    if (memcmp(&nvs_val.sec, &val.sec, sizeof(val.sec)) != 0) {
                        rc = ble_store_nvs_update(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_SEC, nvs_idx, &val);
                        goto end;
                    }
                }
            }
        }
        rc = 0;
    }

end:
    if (rc == 0) {
        rc = ble_nvs_commit_checked(nimble_handle);
    }
    nvs_close(nimble_handle);
    return rc;
}

int ble_store_config_persist_our_secs(void)
{
    int nvs_count, nvs_idx;
    union ble_store_value val;
    int rc, i;
    nvs_handle_t nimble_handle;

    rc = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    nvs_count = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_OUR_SEC, 0, NULL, 0);
    if (nvs_count == -1) {
        ESP_LOGE(TAG, "NVS operation failed while persisting our sec");
        nvs_close(nimble_handle);
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    if (nvs_count < ble_store_config_num_our_secs) {

        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting our sec value to NVS...");
        val.sec = ble_store_config_our_secs[ble_store_config_num_our_secs - 1];
        rc = ble_store_nvs_write(nimble_handle, BLE_STORE_OBJ_TYPE_OUR_SEC, &val);
    } else if (nvs_count > ble_store_config_num_our_secs) {
        /* NVS db count more than RAM count, delete operation */
        nvs_idx = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_OUR_SEC, 0,
                                       ble_store_config_our_secs, ble_store_config_num_our_secs);
        if (nvs_idx == -1) {
            ESP_LOGE(TAG, "NVS delete operation failed for our sec");
            nvs_close(nimble_handle);
            BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
            return BLE_HS_ESTORE_FAIL;
        }
        ESP_LOGD(TAG, "Deleting our sec, nvs idx = %d", nvs_idx);
        rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_OUR_SEC, nvs_idx);
    } else {
        if (ble_store_config_num_our_secs == 0) {
            nvs_close(nimble_handle);
            return 0;
        }

        /* Sync updates: Find which record changed and update it */
        for (i = 0; i < ble_store_config_num_our_secs; i++) {
            val.sec = ble_store_config_our_secs[i];
            nvs_idx = get_nvs_sec_identity_index(nimble_handle, &val.sec, BLE_STORE_OBJ_TYPE_OUR_SEC);
            if (nvs_idx != -1) {
                union ble_store_value nvs_val = {0};
                char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];
                get_nvs_key_string(BLE_STORE_OBJ_TYPE_OUR_SEC, nvs_idx, key_string);
                if (get_nvs_db_value(nimble_handle, BLE_STORE_OBJ_TYPE_OUR_SEC, key_string, &nvs_val) == 0) {
                    if (memcmp(&nvs_val.sec, &val.sec, sizeof(val.sec)) != 0) {
                        rc = ble_store_nvs_update(nimble_handle, BLE_STORE_OBJ_TYPE_OUR_SEC, nvs_idx, &val);
                        goto end;
                    }
                }
            }
        }
        rc = 0;
    }

end:
    if (rc == 0) {
        rc = ble_nvs_commit_checked(nimble_handle);
    }
    nvs_close(nimble_handle);
    return rc;
}

#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
int ble_store_persist_peer_records(void)
{
    int nvs_count, nvs_idx;
    struct ble_hs_dev_records peer_rec;
    int ble_store_num_peer_dev_rec = ble_rpa_get_num_peer_dev_records();
    struct ble_hs_dev_records *peer_dev_rec = ble_rpa_get_peer_dev_records();
    int rc, i;
    nvs_handle_t nimble_handle;

    if (peer_dev_rec == NULL) {
        return BLE_HS_ENOMEM;
    }

    rc = nvs_open(NIMBLE_NVS_NAMESPACE, NVS_READWRITE, &nimble_handle);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "NVS open operation failed");
        return BLE_HS_ESTORE_FAIL;
    }

    nvs_count = get_nvs_db_attribute(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_DEV_REC, 0, NULL, 0);
    if (nvs_count == -1) {
        ESP_LOGE(TAG, "NVS operation failed while persisting peer_dev_rec");
        nvs_close(nimble_handle);
        BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
        return BLE_HS_ESTORE_FAIL;
    }

    if (nvs_count < ble_store_num_peer_dev_rec) {

        /* NVS db count less than RAM count, write operation */
        ESP_LOGD(TAG, "Persisting peer dev record to NVS...");
        peer_rec = peer_dev_rec[ble_store_num_peer_dev_rec - 1];
        rc = ble_store_nvs_peer_records(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_DEV_REC, &peer_rec);
    } else if (nvs_count > ble_store_num_peer_dev_rec) {
        /* NVS db count more than RAM count, delete all non-matching records */
        while (nvs_count > ble_store_num_peer_dev_rec) {
            nvs_idx = get_nvs_db_attribute(nimble_handle,
                                           BLE_STORE_OBJ_TYPE_PEER_DEV_REC, 0,
                                           peer_dev_rec,
                                           ble_store_num_peer_dev_rec);
            if (nvs_idx == -1) {
                ESP_LOGE(TAG, "NVS delete operation failed for peer records");
                nvs_close(nimble_handle);
                BLE_HS_LOG(ERROR, "%s rc=%d\n", __func__, BLE_HS_ESTORE_FAIL);
                return BLE_HS_ESTORE_FAIL;
            }
            ESP_LOGD(TAG, "Deleting peer record, nvs idx = %d", nvs_idx);
            rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_DEV_REC,
                                      nvs_idx);
            if (rc != 0) {
                break;
            }
            nvs_count--;
        }
    } else {
        /* Same count: persist in-memory updates/order by updating existing indices. */
        int write_rc = 0;
        for (i = 0; i < ble_store_num_peer_dev_rec; i++) {
            rc = ble_store_nvs_peer_records_update(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_DEV_REC,
                                                   i + 1, &peer_dev_rec[i]);
            if (rc != 0) {
                ESP_LOGE(TAG, "NVS write operation failed while rewriting peer records");
                write_rc = rc;
            }
        }
        for (i = ble_store_num_peer_dev_rec + 1;
             i <= get_nvs_max_obj_value(BLE_STORE_OBJ_TYPE_PEER_DEV_REC);
             i++) {
            char key_string[NIMBLE_NVS_STR_NAME_MAX_LEN];

            get_nvs_key_string(BLE_STORE_OBJ_TYPE_PEER_DEV_REC, i, key_string);
            rc = get_nvs_peer_record(nimble_handle, key_string, &peer_rec);
            if (rc == ESP_ERR_NVS_NOT_FOUND) {
                continue;
            }
            if (rc != ESP_OK) {
                ESP_LOGE(TAG, "NVS read operation failed while cleaning peer records");
                write_rc = BLE_HS_ESTORE_FAIL;
                continue;
            }

            rc = ble_nvs_delete_value(nimble_handle, BLE_STORE_OBJ_TYPE_PEER_DEV_REC, i);
            if (rc != 0) {
                ESP_LOGE(TAG, "NVS delete operation failed while cleaning peer records");
                write_rc = rc;
            }
        }
        rc = write_rc;
    }

    if (rc == 0) {
        rc = ble_nvs_commit_checked(nimble_handle);
    }
    nvs_close(nimble_handle);
    return rc;
}
#endif

void ble_store_config_conf_init(void)
{
    int err;

    err = ble_nvs_restore_sec_keys();
    if (err != 0) {
        ESP_LOGE(TAG, "NVS operation failed, can't retrieve the bonding info");
    }
#if MYNEWT_VAL(BLE_HOST_BASED_PRIVACY)
    err = ble_nvs_restore_peer_records();
    if (err != 0) {
        ESP_LOGE(TAG, "NVS operation failed, can't retrieve the peer records");
    }
#endif
}

/***************************************************************************************/
#endif /* MYNEWT_VAL(BLE_STORE_CONFIG_PERSIST) */
