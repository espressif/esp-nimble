#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/ras/ble_svc_ras.h"

/*RAS control point state */
enum ble_ras_rreq_cp_state {
	BLE_RAS_RREQ_CP_STATE_NONE,
	BLE_RAS_RREQ_CP_STATE_GET_RD_WRITTEN,
	BLE_RAS_RREQ_CP_STATE_ACK_RD_WRITTEN,
};

#define BLE_UUID_RANGING_SERVICE_VAL (0x185B)

/** @brief UUID of the RAS Features Characteristic. **/
#define BLE_UUID_RAS_FEATURES_VAL (0x2C14)

/** @brief UUID of the Real-time Ranging Data Characteristic. **/
#define BLE_UUID_RAS_REALTIME_RD_VAL (0x2C15)

/** @brief UUID of the On-demand Ranging Data Characteristic. **/
#define BLE_UUID_RAS_ONDEMAND_RD_VAL (0x2C16)

/** @brief UUID of the RAS Control Point Characteristic. **/
#define BLE_UUID_RAS_CP_VAL (0x2C17)

/** @brief UUID of the Ranging Data Ready Characteristic. **/
#define BLE_UUID_RAS_RD_READY_VAL (0x2C18)

/** @brief UUID of the Ranging Data Overwritten Characteristic. **/
#define BLE_UUID_RAS_RD_OVERWRITTEN_VAL (0x2C19)
