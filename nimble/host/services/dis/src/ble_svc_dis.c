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
#include "host/ble_hs.h"
#include "services/dis/ble_svc_dis.h"

#if MYNEWT_VAL(BLE_GATTS) && CONFIG_BT_NIMBLE_DIS_SERVICE

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
#include "esp_nimble_mem.h"
struct ble_svc_dis_data *ble_svc_dis_data_ptr = NULL;
#define ble_svc_dis_data (*ble_svc_dis_data_ptr)
#else
/* Device information */
struct ble_svc_dis_data _ble_svc_dis_data = {
    .model_number      = MYNEWT_VAL(BLE_SVC_DIS_MODEL_NUMBER_DEFAULT),
    .serial_number     = MYNEWT_VAL(BLE_SVC_DIS_SERIAL_NUMBER_DEFAULT),
    .firmware_revision = MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_DEFAULT),
    .hardware_revision = MYNEWT_VAL(BLE_SVC_DIS_HARDWARE_REVISION_DEFAULT),
    .software_revision = MYNEWT_VAL(BLE_SVC_DIS_SOFTWARE_REVISION_DEFAULT),
    .manufacturer_name = MYNEWT_VAL(BLE_SVC_DIS_MANUFACTURER_NAME_DEFAULT),
    .system_id         = MYNEWT_VAL(BLE_SVC_DIS_SYSTEM_ID_DEFAULT),
    .pnp_id            = MYNEWT_VAL(BLE_SVC_DIS_PNP_ID_DEFAULT),
    .ieee              = "dummy_data",
    .udi               = NULL,  /** For now no UID fields are supported */
};
#define ble_svc_dis_data _ble_svc_dis_data
#endif

/* Access function */
#if (MYNEWT_VAL(BLE_SVC_DIS_MODEL_NUMBER_READ_PERM)      >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_SERIAL_NUMBER_READ_PERM)     >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_HARDWARE_REVISION_READ_PERM) >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_READ_PERM) >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_SOFTWARE_REVISION_READ_PERM) >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_MANUFACTURER_NAME_READ_PERM) >= 0) || \
    (MYNEWT_VAL(BLE_SVC_DIS_SYSTEM_ID_READ_PERM) >= 0) || \
    (MYNEWT_VAL(BLE_SVC_DIS_PNP_ID_READ_PERM) >= 0)
static int
ble_svc_dis_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg);
#endif

/* Pre-defined UUIDs to avoid compound literals in DRAM */
static const ble_uuid16_t uuid_svc_dis = BLE_UUID16_INIT(BLE_SVC_DIS_UUID16);
static const ble_uuid16_t uuid_chr_model_number = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_MODEL_NUMBER);
static const ble_uuid16_t uuid_chr_serial_number = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_SERIAL_NUMBER);
static const ble_uuid16_t uuid_chr_firmware_revision = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_FIRMWARE_REVISION);
static const ble_uuid16_t uuid_chr_hardware_revision = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_HARDWARE_REVISION);
static const ble_uuid16_t uuid_chr_software_revision = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_SOFTWARE_REVISION);
static const ble_uuid16_t uuid_chr_manufacturer_name = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_MANUFACTURER_NAME);
static const ble_uuid16_t uuid_chr_system_id = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_SYSTEM_ID);
static const ble_uuid16_t uuid_chr_ieee_cert = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_IEEE_REG_CERT_LIST);
static const ble_uuid16_t uuid_chr_pnp_id = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_PNP_ID);
static const ble_uuid16_t uuid_chr_udi = BLE_UUID16_INIT(BLE_SVC_DIS_CHR_UUID16_UDI);

/* Pre-defined characteristics array to avoid compound literal in DRAM */
static const struct ble_gatt_chr_def dis_characteristics[] = {
#if (MYNEWT_VAL(BLE_SVC_DIS_MODEL_NUMBER_READ_PERM) >= 0)
    {
        /*** Characteristic: Model Number String */
        .uuid = &uuid_chr_model_number.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ | MYNEWT_VAL(BLE_SVC_DIS_MODEL_NUMBER_READ_PERM),
    },
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_SERIAL_NUMBER_READ_PERM) >= 0)
    {
        /*** Characteristic: Serial Number String */
        .uuid = &uuid_chr_serial_number.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ | MYNEWT_VAL(BLE_SVC_DIS_SERIAL_NUMBER_READ_PERM),
    },
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_HARDWARE_REVISION_READ_PERM) >= 0)
    {
        /*** Characteristic: Hardware Revision String */
        .uuid = &uuid_chr_hardware_revision.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ | MYNEWT_VAL(BLE_SVC_DIS_HARDWARE_REVISION_READ_PERM),
    },
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_READ_PERM) >= 0)
    {
        /*** Characteristic: Firmware Revision String */
        .uuid = &uuid_chr_firmware_revision.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ | MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_READ_PERM),
    },
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_SOFTWARE_REVISION_READ_PERM) >= 0)
    {
        /*** Characteristic: Software Revision String */
        .uuid = &uuid_chr_software_revision.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ | MYNEWT_VAL(BLE_SVC_DIS_SOFTWARE_REVISION_READ_PERM),
    },
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_MANUFACTURER_NAME_READ_PERM) >= 0)
    {
        /*** Characteristic: Manufacturer Name */
        .uuid = &uuid_chr_manufacturer_name.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ | MYNEWT_VAL(BLE_SVC_DIS_MANUFACTURER_NAME_READ_PERM),
    },
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_SYSTEM_ID_READ_PERM) >= 0)
    {
        /*** Characteristic: System Id */
        .uuid = &uuid_chr_system_id.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ | MYNEWT_VAL(BLE_SVC_DIS_SYSTEM_ID_READ_PERM),
    },
#endif
    {
        /*** Characteristic: IEEE 11073-20601 Regulatory Certification Data List */
        .uuid = &uuid_chr_ieee_cert.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ,
    },
#if (MYNEWT_VAL(BLE_SVC_DIS_PNP_ID_READ_PERM) >= 0)
    {
        /*** Characteristic: PNP Id */
        .uuid = &uuid_chr_pnp_id.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ | MYNEWT_VAL(BLE_SVC_DIS_PNP_ID_READ_PERM),
    },
#endif
    {
        /*** Characteristic: UDI for Medical Devices */
        .uuid = &uuid_chr_udi.u,
        .access_cb = ble_svc_dis_access,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        0, /* Terminator: No more characteristics in this service */
    },
};

static const struct ble_gatt_svc_def ble_svc_dis_defs[] = {
    {
        /*** Service: Device Information Service (DIS). */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_svc_dis.u,
        .characteristics = dis_characteristics,
    },
    {
        0, /* No more services. */
    },
};

/* Make pointer array itself constant to place in Flash instead of DRAM */
static const struct ble_gatt_svc_def *included_services[] = {ble_svc_dis_defs, NULL};
const struct ble_gatt_svc_def ble_svc_dis_include_def[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &ble_svc_dis_include_uuid.u,
        .includes = included_services,
    }
};

/**
 * Simple read access callback for the device information service
 * characteristic.
 */
#if (MYNEWT_VAL(BLE_SVC_DIS_MODEL_NUMBER_READ_PERM)      >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_SERIAL_NUMBER_READ_PERM)     >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_HARDWARE_REVISION_READ_PERM) >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_READ_PERM) >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_SOFTWARE_REVISION_READ_PERM) >= 0) ||	\
    (MYNEWT_VAL(BLE_SVC_DIS_MANUFACTURER_NAME_READ_PERM) >= 0) || \
    (MYNEWT_VAL(BLE_SVC_DIS_SYSTEM_ID_READ_PERM) >= 0)
static int
ble_svc_dis_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid    = ble_uuid_u16(ctxt->chr->uuid);
    const char *info = NULL;

    switch(uuid) {
#if (MYNEWT_VAL(BLE_SVC_DIS_MODEL_NUMBER_READ_PERM) >= 0)
    case BLE_SVC_DIS_CHR_UUID16_MODEL_NUMBER:
        info = ble_svc_dis_data.model_number;
#ifdef MYNEWT_VAL_BLE_SVC_DIS_MODEL_NUMBER_NAME_DEFAULT
        if (info == NULL) {
            info = MYNEWT_VAL(BLE_SVC_DIS_MODEL_NUMBER_DEFAULT);
        }
#endif
        break;
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_SERIAL_NUMBER_READ_PERM) >= 0)
    case BLE_SVC_DIS_CHR_UUID16_SERIAL_NUMBER:
        info = ble_svc_dis_data.serial_number;
#ifdef MYNEWT_VAL_BLE_SVC_DIS_SERIAL_NUMBER_DEFAULT
        if (info == NULL) {
            info = MYNEWT_VAL(BLE_SVC_DIS_SERIAL_NUMBER_DEFAULT);
        }
#endif
        break;
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_READ_PERM) >= 0)
    case BLE_SVC_DIS_CHR_UUID16_FIRMWARE_REVISION:
        info = ble_svc_dis_data.firmware_revision;
#ifdef MYNEWT_VAL_BLE_SVC_DIS_FIRMWARE_REVISION_DEFAULT
        if (info == NULL) {
            info = MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_DEFAULT);
        }
#endif
        break;
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_HARDWARE_REVISION_READ_PERM) >= 0)
    case BLE_SVC_DIS_CHR_UUID16_HARDWARE_REVISION:
        info = ble_svc_dis_data.hardware_revision;
#ifdef MYNEWT_VAL_BLE_SVC_DIS_HARDWARE_REVISION_DEFAULT
        if (info == NULL) {
            info = MYNEWT_VAL(BLE_SVC_DIS_HARDWARE_REVISION_DEFAULT);
        }
#endif
        break;
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_SOFTWARE_REVISION_READ_PERM) >= 0)
    case BLE_SVC_DIS_CHR_UUID16_SOFTWARE_REVISION:
        info = ble_svc_dis_data.software_revision;
#ifdef MYNEWT_VAL_BLE_SVC_DIS_SOFTWARE_REVISION_DEFAULT
        if (info == NULL) {
            info = MYNEWT_VAL(BLE_SVC_DIS_SOFTWARE_REVISION_DEFAULT);
        }
#endif
        break;
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_MANUFACTURER_NAME_READ_PERM) >= 0)
    case BLE_SVC_DIS_CHR_UUID16_MANUFACTURER_NAME:
        info = ble_svc_dis_data.manufacturer_name;
#ifdef MYNEWT_VAL_BLE_SVC_DIS_MANUFACTURER_NAME_DEFAULT
        if (info == NULL) {
            info = MYNEWT_VAL(BLE_SVC_DIS_MANUFACTURER_NAME_DEFAULT);
        }
#endif
        break;
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_SYSTEM_ID_READ_PERM) >= 0)
    case BLE_SVC_DIS_CHR_UUID16_SYSTEM_ID:
        info = ble_svc_dis_data.system_id;
#ifdef MYNEWT_VAL_BLE_SVC_DIS_SYSTEM_ID_DEFAULT
        if (info == NULL) {
            info = MYNEWT_VAL(BLE_SVC_DIS_SYSTEM_ID_DEFAULT);
        }
#endif
        break;
#endif
#if (MYNEWT_VAL(BLE_SVC_DIS_PNP_ID_READ_PERM) >= 0)
    case BLE_SVC_DIS_CHR_UUID16_PNP_ID:
        info = ble_svc_dis_data.pnp_id;
#ifdef MYNEWT_VAL_BLE_SVC_PNP_SYSTEM_ID_DEFAULT
        if (info == NULL) {
            info = MYNEWT_VAL(BLE_SVC_PNP_SYSTEM_ID_DEFAULT);
        }
#endif
        uint8_t flag = 0x01;
        os_mbuf_append(ctxt->om, &flag, sizeof flag);
        break;
#endif
    case BLE_SVC_DIS_CHR_UUID16_IEEE_REG_CERT_LIST:
        info = ble_svc_dis_data.ieee;
        break;

    case BLE_SVC_DIS_CHR_UUID16_UDI:
        info = ble_svc_dis_data.udi;
        if (info == NULL) {
            uint8_t flag = 0x00;
            os_mbuf_append(ctxt->om, &flag, sizeof(flag));
        }
        break;

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (info != NULL) {
       int rc = os_mbuf_append(ctxt->om, info, strlen(info));
       return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return 0;
}
#endif

const char *
ble_svc_dis_model_number(void)
{
    return ble_svc_dis_data.model_number;
}

int
ble_svc_dis_model_number_set(const char *value)
{
    ble_svc_dis_data.model_number = value;
    return 0;
}

const char *
ble_svc_dis_serial_number(void)
{
    return ble_svc_dis_data.serial_number;
}

int
ble_svc_dis_serial_number_set(const char *value)
{
    ble_svc_dis_data.serial_number = value;
    return 0;
}

const char *
ble_svc_dis_firmware_revision(void)
{
    return ble_svc_dis_data.firmware_revision;
}

int
ble_svc_dis_firmware_revision_set(const char *value)
{
    ble_svc_dis_data.firmware_revision = value;
    return 0;
}

const char *
ble_svc_dis_hardware_revision(void)
{
    return ble_svc_dis_data.hardware_revision;
}

int
ble_svc_dis_hardware_revision_set(const char *value)
{
    ble_svc_dis_data.hardware_revision = value;
    return 0;
}

const char *
ble_svc_dis_software_revision(void)
{
    return ble_svc_dis_data.software_revision;
}

int
ble_svc_dis_software_revision_set(const char *value)
{
    ble_svc_dis_data.software_revision = value;
    return 0;
}

const char *
ble_svc_dis_manufacturer_name(void)
{
    return ble_svc_dis_data.manufacturer_name;
}

int
ble_svc_dis_manufacturer_name_set(const char *value)
{
    ble_svc_dis_data.manufacturer_name = value;
    return 0;
}

const char *
ble_svc_dis_system_id(void)
{
    return ble_svc_dis_data.system_id;
}

int
ble_svc_dis_system_id_set(const char *value)
{
    ble_svc_dis_data.system_id = value;
    return 0;
}

const char *
ble_svc_dis_pnp_id(void)
{
    return ble_svc_dis_data.pnp_id;
}

int
ble_svc_dis_pnp_id_set(const char *value)
{
    ble_svc_dis_data.pnp_id = value;
    return 0;
}

void
ble_svc_dis_included_init(void)
{
    int rc;

    SYSINIT_ASSERT_ACTIVE();

    rc = ble_gatts_count_cfg(ble_svc_dis_include_def);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_dis_include_def);
    SYSINIT_PANIC_ASSERT(rc == 0);
}

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
int
ble_svc_dis_init_dynamic(void)
{
    ble_svc_dis_data_ptr = nimble_platform_mem_calloc(1, sizeof(ble_svc_dis_data));
    if (!ble_svc_dis_data_ptr) {
        return BLE_HS_ENOMEM;
    }


    /* Initialize default fields */
    ble_svc_dis_data_ptr->model_number      = MYNEWT_VAL(BLE_SVC_DIS_MODEL_NUMBER_DEFAULT);
    ble_svc_dis_data_ptr->serial_number     = MYNEWT_VAL(BLE_SVC_DIS_SERIAL_NUMBER_DEFAULT);
    ble_svc_dis_data_ptr->firmware_revision = MYNEWT_VAL(BLE_SVC_DIS_FIRMWARE_REVISION_DEFAULT);
    ble_svc_dis_data_ptr->hardware_revision = MYNEWT_VAL(BLE_SVC_DIS_HARDWARE_REVISION_DEFAULT);
    ble_svc_dis_data_ptr->software_revision = MYNEWT_VAL(BLE_SVC_DIS_SOFTWARE_REVISION_DEFAULT);
    ble_svc_dis_data_ptr->manufacturer_name = MYNEWT_VAL(BLE_SVC_DIS_MANUFACTURER_NAME_DEFAULT);
    ble_svc_dis_data_ptr->system_id         = MYNEWT_VAL(BLE_SVC_DIS_SYSTEM_ID_DEFAULT);
    ble_svc_dis_data_ptr->pnp_id            = MYNEWT_VAL(BLE_SVC_DIS_PNP_ID_DEFAULT);
    ble_svc_dis_data_ptr->ieee              = "dummy_data";
    ble_svc_dis_data_ptr->udi               = NULL;

    return 0;
}
#endif

void
ble_svc_dis_deinit(void)
{
    ble_gatts_free_svcs();
}

/**
 * Initialize the DIS package.
 */
void
ble_svc_dis_init(void)
{
    int rc;

#if MYNEWT_VAL(BLE_STATIC_TO_DYNAMIC)
    if (ble_svc_dis_data_ptr != NULL) {
        nimble_platform_mem_free(ble_svc_dis_data_ptr);
	ble_svc_dis_data_ptr = NULL;
    }

    rc = ble_svc_dis_init_dynamic();
    SYSINIT_PANIC_ASSERT(rc == 0);
#endif

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    rc = ble_gatts_count_cfg(ble_svc_dis_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_dis_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);
}
#endif
