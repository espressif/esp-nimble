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

#if MYNEWT_VAL(BLE_SVC_HID_SERVICE)
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/hid/ble_svc_hid.h"

/* 1 more instance for empty service */
#define HID_MAX_SVC_INSTANCES (MYNEWT_VAL(BLE_SVC_HID_MAX_SVC_INSTANCES) + 1)

/* maximum 7 characteristics except report characteristic */
#define HID_MAX_CHRS (HID_MAX_SVC_INSTANCES * \
                    ((MYNEWT_VAL(BLE_SVC_HID_MAX_RPTS) + 7)))
/* 16 bit UUIDs */
static  ble_uuid_t *uuid_ext_rpt_ref = BLE_UUID16_DECLARE(BLE_SVC_HID_DSC_UUID16_EXT_RPT_REF);
static  ble_uuid_t *uuid_report_map = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_UUID16_REPORT_MAP);
static  ble_uuid_t *uuid_rpt_ref = BLE_UUID16_DECLARE(BLE_SVC_HID_DSC_UUID16_RPT_REF);
static  ble_uuid_t *uuid_report = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_UUID16_RPT);
static  ble_uuid_t *uuid_hid_info = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_UUID16_HID_INFO);
static  ble_uuid_t *uuid_hid_ctrl_pt = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_UUID16_HID_CTRL_PT);
static  ble_uuid_t *uuid_proto_mode = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_UUID16_PROTOCOL_MODE);
static  ble_uuid_t *uuid_boot_kbd_inp = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_UUID16_BOOT_KBD_INP);
static  ble_uuid_t *uuid_boot_kbd_out = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_UUID16_BOOT_KBD_OUT);
static  ble_uuid_t *uuid_boot_mouse_inp = BLE_UUID16_DECLARE(BLE_SVC_HID_CHR_UUID16_BOOT_MOUSE_INP);
static  ble_uuid_t *uuid_hid_svc = BLE_UUID16_DECLARE(BLE_SVC_HID_UUID16);

uint8_t ble_svc_hid_rpt_val[RPT_MAX_LEN];
uint8_t ble_svc_hid_rpt_len;
static struct ble_svc_hid_params hid_instances[HID_MAX_SVC_INSTANCES];

/* Access function */
static int
ble_svc_hid_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg);
static struct ble_gatt_dsc_def ble_svc_hid_dscs[HID_MAX_CHRS];
static uint8_t ble_svc_hid_dsc_index = 0; // used to store the current index in the dscs array
static struct ble_gatt_chr_def ble_svc_hid_chrs[HID_MAX_CHRS];
static uint8_t ble_svc_hid_chr_index = 0; // used to store the current index in the chrs array
static uint8_t ble_svc_hid_svc_index = 0; // used to store the current index in the svcs array

static struct ble_gatt_svc_def ble_svc_hid_defs[HID_MAX_SVC_INSTANCES];

static int
ble_svc_hid_chr_write(struct os_mbuf *om, uint16_t min_len,
                      uint16_t max_len, void *dst,
                      uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static struct ble_gatt_dsc_def*
ble_svc_hid_get_dsc(uint8_t num)
{
    if (ble_svc_hid_dsc_index + num - 1 >= HID_MAX_CHRS) {
        return NULL;
    }
    ble_svc_hid_dsc_index = ble_svc_hid_dsc_index + num;
    return &ble_svc_hid_dscs[ble_svc_hid_dsc_index - num];
}

static struct ble_gatt_chr_def*
ble_svc_hid_get_chr_block(void)
{
    if (ble_svc_hid_chr_index >= HID_MAX_CHRS) {
        return NULL;
    }
    ble_svc_hid_chr_index = ble_svc_hid_chr_index + 1;
    return &ble_svc_hid_chrs[ble_svc_hid_chr_index - 1];
}

/*returns current chr index */
static uint8_t
ble_svc_hid_get_curr_chr_idx(void)
{
    return ble_svc_hid_chr_index;
}

/*returns current svc index */
static uint8_t
get_curr_svc_idx(void)
{
    return ble_svc_hid_svc_index;
}

struct report *
find_rpt_by_handle(uint16_t handle)
{
    uint8_t instance, instances;
    int i;

    instances = get_curr_svc_idx();

    for (instance = 0; instance < instances; instance++) {
        for (i = 0; i < hid_instances[instance].rpts_len; i++) {
            if (hid_instances[instance].rpts[i].handle == handle) {
                return &hid_instances[instance].rpts[i];
            }
        }
    }
    /* return some non-zero value */
    return NULL;
}

static struct ble_gatt_svc_def*
ble_svc_get_svc_block(void)
{
    if (ble_svc_hid_svc_index >= HID_MAX_SVC_INSTANCES) {
        return NULL;
    }
    ble_svc_hid_svc_index = ble_svc_hid_svc_index + 1;
    return &ble_svc_hid_defs[ble_svc_hid_svc_index - 1];
}

/* fill protocol mode char */
static void
fill_proto_mode(uint8_t instance)
{
    struct ble_gatt_chr_def *chr, demo_chr;

    if (!hid_instances[instance].proto_mode_present) {
        return;
    }
    demo_chr = (struct ble_gatt_chr_def) {
        /*** Report Map characteristic */
        .uuid = uuid_proto_mode,
        .access_cb = ble_svc_hid_access,
        .val_handle = &hid_instances[instance].proto_mode_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP |
#if MYNEWT_VAL(BLE_SM_LVL) == 2
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#elif MYNEWT_VAL(BLE_SM_LVL) == 3
                 BLE_GATT_CHR_F_READ_AUTHEN | BLE_GATT_CHR_F_WRITE_AUTHEN |
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#endif
                 0,
    };

    chr = ble_svc_hid_get_chr_block();
    memcpy(chr, &demo_chr, sizeof(struct ble_gatt_chr_def));
}

/* fill boot keyboard inp char */
void
fill_boot_kbd_inp(uint8_t instance)
{
    struct ble_gatt_chr_def *chr, demo_chr;

    if (!hid_instances[instance].kbd_inp_present) {
        return;
    }
    demo_chr = (struct ble_gatt_chr_def) {
        /*** Report Map characteristic */
        .uuid = uuid_boot_kbd_inp,
        .access_cb = ble_svc_hid_access,
        .val_handle = &hid_instances[instance].kbd_inp_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY |
                 BLE_GATT_CHR_F_WRITE |
#if MYNEWT_VAL(BLE_SM_LVL) == 2
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#elif MYNEWT_VAL(BLE_SM_LVL) == 3
                 BLE_GATT_CHR_F_READ_AUTHEN | BLE_GATT_CHR_F_WRITE_AUTHEN |
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#endif
                 0,
    };

    chr = ble_svc_hid_get_chr_block();
    memcpy(chr, &demo_chr, sizeof(struct ble_gatt_chr_def));
}

/* fill boot keyboard out char */
void
fill_boot_kbd_out(uint8_t instance)
{
    struct ble_gatt_chr_def *chr, demo_chr;

    if (!hid_instances[instance].kbd_out_present) {
        return;
    }
    demo_chr = (struct ble_gatt_chr_def) {
        /*** Report Map characteristic */
        .uuid = uuid_boot_kbd_out,
        .access_cb = ble_svc_hid_access,
        .val_handle = &hid_instances[instance].kbd_out_handle,
        .flags = BLE_GATT_CHR_F_READ |
                 BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE |
#if MYNEWT_VAL(BLE_SM_LVL) == 2
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#elif MYNEWT_VAL(BLE_SM_LVL) == 3
                 BLE_GATT_CHR_F_READ_AUTHEN | BLE_GATT_CHR_F_WRITE_AUTHEN |
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#endif
                 0,
    };

    chr = ble_svc_hid_get_chr_block();
    memcpy(chr, &demo_chr, sizeof(struct ble_gatt_chr_def));
}
/* fill boot mouse inp char */
void
fill_boot_mouse_inp(uint8_t instance)
{
    struct ble_gatt_chr_def *chr, demo_chr;

    if (!hid_instances[instance].mouse_inp_present) {
        return;
    }
    demo_chr = (struct ble_gatt_chr_def) {
        /*** Report Map characteristic */
        .uuid = uuid_boot_mouse_inp,
        .access_cb = ble_svc_hid_access,
        .val_handle = &hid_instances[instance].mouse_inp_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY |
                 BLE_GATT_CHR_F_WRITE |
#if MYNEWT_VAL(BLE_SM_LVL) == 2
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#elif MYNEWT_VAL(BLE_SM_LVL) == 3
                 BLE_GATT_CHR_F_READ_AUTHEN | BLE_GATT_CHR_F_WRITE_AUTHEN |
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#endif
                 0,
    };

    chr = ble_svc_hid_get_chr_block();
    memcpy(chr, &demo_chr, sizeof(struct ble_gatt_chr_def));
}
/* create report map char */
static void
fill_rpt_map(uint8_t instance)
{
    struct ble_gatt_chr_def *chr, demo_chr;
    struct ble_gatt_dsc_def *dsc;
    struct ble_gatt_dsc_def *demo_dsc = (struct ble_gatt_dsc_def[]) {
        {
            /* External Report Reference descriptor */
            .uuid = uuid_ext_rpt_ref,
            .access_cb = ble_svc_hid_access,
            .att_flags = BLE_ATT_F_READ,
            .arg = &hid_instances[instance].report_map_handle,
        }, {
            0,
        }
    };
    demo_chr = (struct ble_gatt_chr_def) {
        /*** Report Map characteristic */
        .uuid = uuid_report_map,
        .access_cb = ble_svc_hid_access,
        .val_handle = &hid_instances[instance].report_map_handle,
        .flags = BLE_GATT_CHR_F_READ |
#if MYNEWT_VAL(BLE_SM_LVL) == 2
                 BLE_GATT_CHR_F_READ_ENC |
#elif MYNEWT_VAL(BLE_SM_LVL) == 3
                 BLE_GATT_CHR_F_READ_AUTHEN |
                 BLE_GATT_CHR_F_READ_ENC |
#endif
                 0,
    };
    /* logic : allocate the discriptor needed and then allocate one more and assign 0 to it,
    this is done to indicate there are no more descriptors */
    dsc = ble_svc_hid_get_dsc(2);
    memcpy(dsc, demo_dsc, 2 * sizeof(struct ble_gatt_dsc_def));

    demo_chr.descriptors = dsc,
    chr = ble_svc_hid_get_chr_block();
    memcpy(chr, &demo_chr, sizeof(struct ble_gatt_chr_def));
}

/* create report chars */
static void
fill_reports(uint8_t instance)
{
    struct ble_gatt_chr_def *chr, demo_chr;
    struct ble_gatt_dsc_def *dsc;
    int i;
    struct ble_gatt_dsc_def *demo_dsc = (struct ble_gatt_dsc_def[]) {
        {
            /* Report Reference descriptor */
            .uuid = uuid_rpt_ref,
            .access_cb = ble_svc_hid_access,
            .att_flags = BLE_ATT_F_READ
        }, {
            0,
        }
    };
    demo_chr = (struct ble_gatt_chr_def) {
        /*** Report characteristic */
        .uuid = uuid_report,
        .access_cb = ble_svc_hid_access,
    };
    /* Multiple instances of this characteristic are allowed*/
    for (i = 0; i < hid_instances[instance].rpts_len; i++) {
        /* logic : allocate the discriptor needed and then allocate one more and assign 0 to it,
        this is done to indicate there are no more descriptors */
        dsc = ble_svc_hid_get_dsc(2);
        demo_dsc[0].arg = &hid_instances[instance].rpts[i].handle;
        memcpy(dsc, demo_dsc, 2 * sizeof(struct ble_gatt_dsc_def));

        switch (hid_instances[instance].rpts[i].type) {
        case BLE_SVC_HID_RPT_TYPE_INPUT:
            demo_chr.flags = BLE_GATT_CHR_F_READ |  BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE;
            break;
        case BLE_SVC_HID_RPT_TYPE_OUTPUT:
            demo_chr.flags = BLE_GATT_CHR_F_READ |  BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE;
            break;
        case BLE_SVC_HID_RPT_TYPE_FEATURE:
            demo_chr.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |  BLE_GATT_CHR_F_WRITE;
            break;
        default :
            assert(0);
            break;
        }
        demo_chr.flags |= (
#if MYNEWT_VAL(BLE_SM_LVL) == 2
                              BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#elif MYNEWT_VAL(BLE_SM_LVL) == 3
                              BLE_GATT_CHR_F_READ_AUTHEN | BLE_GATT_CHR_F_WRITE_AUTHEN |
                              BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
#endif
                              0);
        demo_chr.val_handle = &hid_instances[instance].rpts[i].handle;
        demo_chr.descriptors = dsc;

        chr = ble_svc_hid_get_chr_block();
        memcpy(chr, &demo_chr, sizeof(struct ble_gatt_chr_def));
    }
}

static void
fill_hid_info(uint8_t instance)
{
    struct ble_gatt_chr_def *chr, demo_chr;

    demo_chr = (struct ble_gatt_chr_def) {
        /*** HID Information Characteristic */
        .uuid = uuid_hid_info,
        .access_cb = ble_svc_hid_access,
        .val_handle = &hid_instances[instance].hid_info_handle,
        .flags = BLE_GATT_CHR_F_READ |
#if MYNEWT_VAL(BLE_SM_LVL) == 2
                 BLE_GATT_CHR_F_READ_ENC |
#elif MYNEWT_VAL(BLE_SM_LVL) == 3
                 BLE_GATT_CHR_F_READ_AUTHEN |
                 BLE_GATT_CHR_F_READ_ENC |
#endif
                 0,
    };
    chr = ble_svc_hid_get_chr_block();
    memcpy(chr, &demo_chr, sizeof(struct ble_gatt_chr_def));
}

static void
fill_ctrl_pt(uint8_t instance)
{
    struct ble_gatt_chr_def *chr, demo_chr;

    demo_chr = (struct ble_gatt_chr_def) {
        /*** HID Control Point Characteristic */
        .uuid = uuid_hid_ctrl_pt,
        .access_cb = ble_svc_hid_access,
        .val_handle = &hid_instances[instance].ctrl_pt_handle,
        .flags = BLE_GATT_CHR_F_WRITE_NO_RSP |
#if MYNEWT_VAL(BLE_SM_LVL) == 2
                 BLE_GATT_CHR_F_WRITE_ENC |
#elif MYNEWT_VAL(BLE_SM_LVL) == 3
                 BLE_GATT_CHR_F_WRITE_AUTHEN |
                 BLE_GATT_CHR_F_WRITE_ENC |
#endif
                 0,
    };
    chr = ble_svc_hid_get_chr_block();
    memcpy(chr, &demo_chr, sizeof(struct ble_gatt_chr_def));
}

/**
 * allocating one more chr block with
 * value set to zero
 */
static void
ble_svc_hid_end_chrs(void)
{
    struct ble_gatt_chr_def *chr;
    chr = ble_svc_hid_get_chr_block();
    memset(chr, 0, sizeof(struct ble_gatt_chr_def));
}
/**
 * HID access function
 */
static int
ble_svc_hid_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt,
                   void *arg)
{
    uint16_t uuid16 = ble_uuid_u16(ctxt->chr->uuid);
    int rc;
    struct report *rpt;
    uint16_t rpt_ref_val = 0;
    uint16_t out_rpt_len = 0;
    uint8_t instances = get_curr_svc_idx();
    uint16_t handle;
    uint8_t val;

    for (int instance = 0; instance < instances; instance++) {
        switch (uuid16) {
        case BLE_SVC_HID_CHR_UUID16_REPORT_MAP:
            if (hid_instances[instance].report_map_handle != attr_handle) {
                continue;
            }
            assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
            rc = os_mbuf_append(ctxt->om, &hid_instances[instance].report_map,
                                hid_instances[instance].report_map_len);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case BLE_SVC_HID_DSC_UUID16_EXT_RPT_REF:
            handle = *(uint16_t*)(ctxt->dsc->arg);
            if (hid_instances[instance].report_map_handle != handle) {
                continue;
            }
            assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC);
            rc = os_mbuf_append(ctxt->om, &hid_instances[instance].external_rpt_ref,
                                sizeof hid_instances[instance].external_rpt_ref);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        case BLE_SVC_HID_DSC_UUID16_RPT_REF:
            /* this will work without having any instance check
            because find_rpt_by_handle already checks for the instance */
            assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC);
            rpt = find_rpt_by_handle(*(uint16_t *)ctxt->dsc->arg);
            rpt_ref_val = (0x00FF & rpt->id) | ((0x00FF & (uint16_t)rpt->type) << 8); /* check if this should be opposite */
            rc = os_mbuf_append(ctxt->om, &rpt_ref_val,
                                sizeof rpt_ref_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_SVC_HID_CHR_UUID16_HID_INFO:
            if (hid_instances[instance].hid_info_handle != attr_handle) {
                continue;
            }
            assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
            rc = os_mbuf_append(ctxt->om, &hid_instances[instance].hid_info,
                                sizeof hid_instances[instance].hid_info);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_SVC_HID_CHR_UUID16_HID_CTRL_PT:
            if (hid_instances[instance].ctrl_pt_handle != attr_handle) {
                continue;
            }
            assert(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR);
            /* check if the value is correct */
            rc = ble_hs_mbuf_to_flat(ctxt->om, &val, sizeof(uint8_t), NULL);
            if(rc != 0) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            if(val == 0 || val == 1) {
                rc = ble_svc_hid_chr_write(ctxt->om, 0, sizeof hid_instances[instance].ctrl_pt, &hid_instances[instance].ctrl_pt, NULL);
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return BLE_ATT_ERR_UNLIKELY;

        case BLE_SVC_HID_CHR_UUID16_BOOT_KBD_OUT:
            if (hid_instances[instance].kbd_out_handle != attr_handle) {
                continue;
            }
            assert(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR || ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, &hid_instances[instance].kbd_out_rpt,
                                    sizeof(hid_instances[instance].kbd_out_rpt));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                rc = ble_svc_hid_chr_write(ctxt->om, 0, sizeof(hid_instances[instance].kbd_out_rpt), &hid_instances[instance].kbd_out_rpt, NULL);
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return 0;
        case BLE_SVC_HID_CHR_UUID16_BOOT_KBD_INP:
            if (hid_instances[instance].kbd_inp_handle != attr_handle) {
                continue;
            }
            assert(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR || ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, &hid_instances[instance].kbd_inp_rpt,
                                    sizeof(hid_instances[instance].kbd_inp_rpt));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                rc = ble_svc_hid_chr_write(ctxt->om, 0, sizeof(hid_instances[instance].kbd_inp_rpt), hid_instances[instance].kbd_inp_rpt, NULL);
                if (ctxt->chr->flags & BLE_GATT_CHR_F_NOTIFY) {
                    ble_gatts_chr_updated(*(ctxt->chr->val_handle));
                }
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return 0;
        case BLE_SVC_HID_CHR_UUID16_BOOT_MOUSE_INP:
            if (hid_instances[instance].mouse_inp_handle != attr_handle) {
                continue;
            }
            assert(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR || ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, &hid_instances[instance].mouse_inp_rpt,
                                    hid_instances[instance].mouse_inp_rpt_len);
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                rc = ble_svc_hid_chr_write(ctxt->om, 0, sizeof(hid_instances[instance].mouse_inp_rpt), hid_instances[instance].mouse_inp_rpt, &out_rpt_len);
                if (rc != 0) {
                    return BLE_ATT_ERR_INSUFFICIENT_RES;
                }
                hid_instances[instance].mouse_inp_rpt_len = out_rpt_len;
                if (ctxt->chr->flags & BLE_GATT_CHR_F_NOTIFY) {
                    ble_gatts_chr_updated(*(ctxt->chr->val_handle));
                }
                return 0;
            }
            return 0;
        case BLE_SVC_HID_CHR_UUID16_PROTOCOL_MODE:
            if (hid_instances[instance].proto_mode_handle != attr_handle) {
                continue;
            }
            assert(ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR || ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, &hid_instances[instance].proto_mode,
                                    sizeof(hid_instances[instance].proto_mode));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                /* check if the value is correct */
                rc = ble_hs_mbuf_to_flat(ctxt->om, &val, sizeof(uint8_t), NULL);
                if(rc != 0) {
                    return BLE_ATT_ERR_INSUFFICIENT_RES;
                }
                if(val == 0 || val == 1) {
                    rc = ble_svc_hid_chr_write(ctxt->om, 0, sizeof(hid_instances[instance].proto_mode), &hid_instances[instance].proto_mode, NULL);
                    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }
                return BLE_ATT_ERR_UNLIKELY;
            }
            return 0;
        case BLE_SVC_HID_CHR_UUID16_RPT:
            /* this will work without any check of instance */
            rpt = find_rpt_by_handle(*(ctxt->chr->val_handle));
            assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR || ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR);
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                rc = os_mbuf_append(ctxt->om, rpt->data,
                                    rpt->len);
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            } else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                rc = ble_svc_hid_chr_write(ctxt->om, 0, RPT_MAX_LEN, &(rpt->data), &out_rpt_len);
                if (rc != 0) {
                    return BLE_ATT_ERR_INSUFFICIENT_RES;
                }
                rpt->len = out_rpt_len;
                if (ctxt->chr->flags & BLE_GATT_CHR_F_NOTIFY) {
                    ble_gatts_chr_updated(*(ctxt->chr->val_handle));
                }
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return 0;

        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/* can be called multiple times */
int
ble_svc_hid_add(struct ble_svc_hid_params params)
{
    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    uint8_t svc_idx;
    int rc = 0;
    struct ble_gatt_svc_def *svc;
    uint8_t chr_idx;

    svc_idx = get_curr_svc_idx();
    /* one instance is required for empty service */
    if (HID_MAX_SVC_INSTANCES  - 1 <= svc_idx) {
        /* increase instances count */
        return BLE_HS_ENOMEM;
    }

    memcpy(&hid_instances[svc_idx], &params, sizeof(struct ble_svc_hid_params));

    /* get the pointer to the first characteristic */
    chr_idx = ble_svc_hid_get_curr_chr_idx();

    /* Fill protocol mode characteristic */
    fill_proto_mode(svc_idx);
    /* Fill report map characteristic */
    fill_rpt_map(svc_idx);
    /* Fill report characteristics */
    fill_reports(svc_idx);
    /* Fill the boot keyboard input characteristic */
    fill_boot_kbd_inp(svc_idx);
    /* Fill the boot keyboard output characteristic */
    fill_boot_kbd_out(svc_idx);
    /* Fill the boot mouse input characteristic */
    fill_boot_mouse_inp(svc_idx);
    /* Fill the hid info characteristic */
    fill_hid_info(svc_idx);
    /* Fill the control point characteristic */
    fill_ctrl_pt(svc_idx);
    /* End the characteristics with the characteristic with empty block */
    ble_svc_hid_end_chrs();

    svc = ble_svc_get_svc_block();
    svc->type = BLE_GATT_SVC_TYPE_PRIMARY;
    svc->uuid = uuid_hid_svc;
    svc->characteristics = ble_svc_hid_chrs + chr_idx;
    return rc;
}

/**
 * Allocating one more svc block
 * with value set to 0
 */
static int
ble_svc_hid_end(void)
{
    struct ble_gatt_svc_def *svc;

    svc = ble_svc_get_svc_block();
    if (svc == NULL) {
        return BLE_HS_ENOMEM;
    }
    memset(svc, 0, sizeof(struct ble_gatt_svc_def));
    return 0;
}

/**
 * If the ble_gatts_reset() is called after ble_svc_hid_init(),
   call ble_svc_hid_reset() to reinitialize the service.
 */
void
ble_svc_hid_reset(void)
{
    ble_svc_hid_dsc_index = 0;
    ble_svc_hid_chr_index = 0;
    ble_svc_hid_svc_index = 0;
}

/**
 * Initialize the HID Service.
 */
void
ble_svc_hid_init(void)
{
    int rc;

    /* Ensure this function only gets called by sysinit. */
    SYSINIT_ASSERT_ACTIVE();

    rc = ble_svc_hid_end();
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_count_cfg(ble_svc_hid_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = ble_gatts_add_svcs(ble_svc_hid_defs);
    SYSINIT_PANIC_ASSERT(rc == 0);
}
#endif // CONFIG_BT_NIMBLE_HID_SERVICE
