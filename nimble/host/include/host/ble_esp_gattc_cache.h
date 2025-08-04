/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef H_BLE_ESP_GATTC_GET_CACHE_
#define H_BLE_ESP_GATTC_GET_CACHE_

#ifdef __cplusplus
extern "C" {
#endif

#if MYNEWT_VAL(BLE_GATT_CACHING)
#include "../src/ble_gattc_cache_priv.h"
/** Enumerates types of GATT database attributes */

typedef enum {
    ESP_BLE_GATT_DB_PRIMARY_SERVICE,          /*!< Primary services in the GATT database */
    ESP_BLE_GATT_DB_SECONDARY_SERVICE,        /*!< Secondary services in the GATT database */
    ESP_BLE_GATT_DB_INCLUDED_SERVICE,         /*!< Included services in the GATT database */
    ESP_BLE_GATT_DB_CHARACTERISTIC,           /*!< Characteristics in the GATT database */
    ESP_BLE_GATT_DB_DESCRIPTOR,               /*!< Descriptors in the GATT database */ 
    ESP_BLE_GATT_DB_ALL,                      /*!< All attributes in the GATT database */
} esp_ble_gatt_db_attr_type_t;

/** @brief Represents a GATT service element. */
typedef struct {
    bool is_primary;                          /*!< Indicates if the service is primary. */
    uint16_t start_handle;                    /*!< Service start handle. */
    uint16_t end_handle;                      /*!< Service end handle. */
    ble_uuid_any_t uuid;                      /*!< Service UUID. */
} ble_gattc_service_elem_t;

/** @brief Represents a GATT included service element. */
typedef struct {
    uint16_t handle;                           /*!< handle */
    uint16_t incl_svc_s_handle;                /*!< Included service start handle. */
    uint16_t incl_svc_e_handle;                /*!< Included service end handle. */
    ble_uuid_any_t uuid;                       /*!< Included service UUID. */
} ble_gattc_included_svc_elem_t;

/** @brief Represents a GATT characteristic element. */
typedef struct {
    uint16_t char_handle;                     /*!< Characteristic value handle. */
    uint8_t properties;                       /*!< Characteristic properties. */
    ble_uuid_any_t uuid;                      /*!< Characteristic UUID. */
} ble_gattc_char_elem_t;

/** @brief Represents a GATT descriptor element. */
typedef struct {
    uint16_t handle;                          /*!< Descriptor handle. */
    ble_uuid_any_t uuid;                      /*!< Descriptor UUID. */
} ble_gattc_descr_elem_t;

/**
 * Retrieve service(s) from the cached GATT database for a given connection.
 *
 * This function searches the local GATT cache for a service matching the given UUID.
 * If `svc_uuid` is not NULL, it returns the matching service(s).
 * If `svc_uuid` is NULL, it returns all cached services.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param svc_uuid        UUID of the service to search for (can be NULL to fetch all services).
 * @param result          Output buffer to store the retrieved service(s).
 * @param count           The number of services to retrieve. It will be updated with the actual
 *                        number of services found, starting at the given offset (if specified).
 * @param offset          Index offset for paginated access (used when more services exist).
 *
 * @return                0 on success, error code otherwise.
 */


int ble_gattc_get_service(uint16_t conn_handle,
                          ble_uuid_t *svc_uuid,
                          ble_gattc_service_elem_t *result,
                          uint16_t *count, uint16_t offset);
/**
 * Retrieve all characteristics in a specified handle range from the cached GATT database.
 *
 * This function fetches all characteristics between the given start and end handles,
 * typically the range of a service. Characteristics are returned from the local cache.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param start_handle    Start handle of the attribute range (usually the service's start handle).
 * @param end_handle      End handle of the attribute range (usually the service's end handle).
 * @param result          Output buffer to store the retrieved characteristics.
 * @param count           The number of characteristics to retrieve. It will be updated with the actual
 *                        number of characteristics found, starting at the given offset (if specified).
 * @param offset          The position offset to retrieve
 *
 * @return                0 on success, error code otherwise.
 */


int ble_gattc_get_all_char(uint16_t conn_handle,
                           uint16_t start_handle,
                           uint16_t end_handle,
                           ble_gattc_char_elem_t *result,
                           uint16_t *count, uint16_t offset);
/**
 * Retrieve all descriptors of a given characteristic from the cached GATT database.
 *
 * This function fetches all descriptors associated with the specified characteristic
 * handle from the local GATT cache.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param char_handle     Handle of the characteristic whose descriptors are to be fetched.
 * @param result          Output buffer to store the retrieved descriptors.
 * @param count           The number of descriptors to retrieve. It will be updated with the actual
 *                        number of descriptors found, starting at the given offset (if specified).
 * @param offset          The position offset to retrieve
 *
 * @return                0 on success, error code otherwise.
 */



int ble_gattc_get_all_descr(uint16_t conn_handle,
                            uint16_t char_handle,
                            ble_gattc_descr_elem_t *result,
                            uint16_t *count, uint16_t offset);

/**
 * Retrieve characteristics with a specific UUID from the cached GATT database.
 *
 * This function fetches all characteristics matching the given UUID within the specified
 * handle range (typically a service's start and end handles) from the local GATT cache.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param start_handle    Start handle of the attribute range.
 * @param end_handle      End handle of the attribute range.
 * @param char_uuid       UUID of the characteristic to search for.
 * @param result          Output buffer to store the retrieved characteristics.
 * @param count           The number of characteristics to retrieve. It will be updated with the actual
 *                        number of characteristics found, starting at the given offset (if specified).
 *
 * @return                0 on success, error code otherwise.
 */


int ble_gattc_get_char_by_uuid(uint16_t conn_handle,
                               uint16_t start_handle,
                               uint16_t end_handle,
                               ble_uuid_t *char_uuid,
                               ble_gattc_char_elem_t *result, uint16_t *count);

/**
 * Retrieve descriptors with a specific UUID from the cached GATT database.
 *
 * This function searches for all descriptors that match the given descriptor UUID
 * within the specified handle range and associated with the given characteristic UUID.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param start_handle    Start handle of the attribute range.
 * @param end_handle      End handle of the attribute range.
 * @param char_uuid       UUID of the characteristic to which the descriptor belongs.
 * @param descr_uuid      UUID of the descriptor to search for.
 * @param result          Output buffer to store the retrieved descriptors.
 * @param count           The number of descriptors to retrieve. It will be updated with the actual
 *                        number of descriptors found, starting at the given offset (if specified).
 *
 * @return                0 on success, error code otherwise.
 */

int ble_gattc_get_descr_by_uuid(uint16_t conn_handle,
                                uint16_t start_handle,
                                uint16_t end_handle,
                                ble_uuid_t *char_uuid,
                                ble_uuid_t *descr_uuid,
                                ble_gattc_descr_elem_t *result, uint16_t *count);
/**
 * Retrieve descriptors with a specific UUID for a given characteristic handle from the cached GATT database.
 *
 * This function searches the local GATT cache for descriptors matching the provided UUID
 * that are associated with the specified characteristic handle.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param char_handle     Handle of the characteristic whose descriptors are to be searched.
 * @param descr_uuid      UUID of the descriptor to search for.
 * @param result          Output buffer to store the retrieved descriptors.
 * @param count           The number of descriptors to retrieve. It will be updated with the actual
 *                        number of descriptors found, starting at the given offset (if specified).
 *
 * @return                0 on success, error code otherwise.
 */

int ble_gattc_get_descr_by_char_handle(uint16_t conn_handle,
                                       uint16_t char_handle,
                                       ble_uuid_t *descr_uuid,
                                       ble_gattc_descr_elem_t *result, uint16_t *count);
/**
 * Retrieve included services within a specified handle range from the cached GATT database.
 *
 * This function searches for included (referenced) services within the specified start and end handle range.
 * If `incl_uuid` is not NULL, it returns only the included services that match the provided UUID.
 * Otherwise, it returns all included services found in the range.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param start_handle    Start handle of the attribute range.
 * @param end_handle      End handle of the attribute range.
 * @param incl_uuid       UUID of the included service to search for (can be NULL to fetch all).
 * @param result          Output buffer to store the retrieved included services.
 * @param count           The number of included services to retrieve. It will be updated with the actual
 *                        number of included services found, starting at the given offset (if specified).
 *
 * @return                0 on success, error code otherwise.
 */
#if MYNEWT_VAL(BLE_GATT_CACHING_INCLUDE_SERVICES)
int ble_gattc_get_include_service(uint16_t conn_handle,
                                  uint16_t start_handle,
                                  uint16_t end_handle,
                                  ble_uuid_t *incl_uuid,
                                  ble_gattc_included_svc_elem_t *result, uint16_t *count);
#endif

/**
 * Get the number of attributes of a given type from the cached GATT database.
 *
 * This function counts the number of attributes (services, characteristics, or descriptors)
 * present in the local GATT cache within a specified handle range or under a specific
 * characteristic, depending on the attribute type.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param type            Type of attribute to count (e.g., service, characteristic, descriptor).
 * @param start_handle    Start handle of the range to search.
 * @param end_handle      End handle of the range to search.
 * @param char_handle     Characteristic handle (used only if type is `ESP_BLE_GATT_DB_DESCRIPTOR`).
 * @param count           Output pointer to store the number of attributes found.
 *
 * @return                0 on success, error code otherwise.
 */

int ble_gattc_get_attr_count(uint16_t conn_handle, esp_ble_gatt_db_attr_type_t type,
                             uint16_t start_handle, uint16_t end_handle,
                             uint16_t char_handle, uint16_t *count);
/**
 * Retrieve cached GATT attributes within a given handle range.
 *
 * This function fetches services, characteristics, and descriptors from the local
 * GATT database cache between the specified start and end handles.
 *
 * @param conn_handle     Connection handle of the peer device.
 * @param start_handle    Start handle of the GATT database range.
 * @param end_handle      End handle of the GATT database range.
 * @param result          Output buffer to store the retrieved GATT database elements.
 * @param count           The number of attribute to retrieve. It will be updated with the actual number of attributes found.
 *
 * @return                0 on success, error code otherwise.
 */


int ble_gattc_get_db(uint16_t conn_handle,
                     uint16_t start_handle, uint16_t end_handle,
                     ble_gattc_db_elem_t* result, uint16_t *count);

#ifdef __cplusplus
}
#endif

#endif
#endif
