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

#if MYNEWT_VAL(BLE_SVC_HID_SERVICE)
#ifndef H_BLE_SVC_HID_
#define H_BLE_SVC_HID_

/* 16 Bit Battery Service UUID */
#define BLE_SVC_HID_UUID16                                   0x1812

/* 16 Bit HID Service Characteristic UUIDs */
#define BLE_SVC_HID_CHR_UUID16_REPORT_MAP                    0x2A4B
#define BLE_SVC_HID_CHR_UUID16_HID_INFO                      0x2A4A
#define BLE_SVC_HID_CHR_UUID16_HID_CTRL_PT                   0x2A4C
#define BLE_SVC_HID_DSC_UUID16_EXT_RPT_REF                   0x2907
#define BLE_SVC_HID_CHR_UUID16_RPT                           0x2A4D
#define BLE_SVC_HID_DSC_UUID16_RPT_REF                       0x2908
#define BLE_SVC_HID_CHR_UUID16_PROTOCOL_MODE                 0x2A4E
#define BLE_SVC_HID_CHR_UUID16_BOOT_KBD_INP                  0x2A22
#define BLE_SVC_HID_CHR_UUID16_BOOT_KBD_OUT                  0x2A32
#define BLE_SVC_HID_CHR_UUID16_BOOT_MOUSE_INP                0x2A33

/* Report type values */
#define BLE_SVC_HID_RPT_TYPE_INPUT                          0x01
#define BLE_SVC_HID_RPT_TYPE_OUTPUT                         0x02
#define BLE_SVC_HID_RPT_TYPE_FEATURE                        0x03

/* Protocol Mode values */
#define BLE_SVC_HID_PROTO_MODE_BOOT                         0x00
#define BLE_SVC_HID_PROTO_MODE_REPORT                       0x01

#define REPORT_MAP_SIZE               512
#define RPT_MAX_LEN                   256
#define MAX_REPORTS                   MYNEWT_VAL(BLE_SVC_HID_MAX_RPTS)
#define MOUSE_INP_RPT_SIZE            8
#define KBD_INP_RPT_SIZE              8


struct report {
    uint8_t data[RPT_MAX_LEN];
    uint8_t len;
    uint8_t type;
    uint8_t id;
    uint16_t handle;
};

struct ble_svc_hid_params{
    unsigned int proto_mode_present : 1;
    unsigned int kbd_inp_present : 1;
    unsigned int kbd_out_present : 1;
    unsigned int mouse_inp_present : 1;
    /* protocol mode char */
    uint8_t proto_mode;
    uint16_t proto_mode_handle;

    /* boot keyboard input char */
    uint8_t kbd_inp_rpt[KBD_INP_RPT_SIZE];
    uint16_t kbd_inp_handle;

    /* boot keyboard output char */
    uint8_t kbd_out_rpt;
    uint16_t kbd_out_handle;

    /* boot mouse input char */
    /* NOTE : size of mouse inp report
    upto byte 2 is mandatory
    from byte 3 rest is device specific */
    uint8_t mouse_inp_rpt[MOUSE_INP_RPT_SIZE];
    uint8_t mouse_inp_rpt_len;
    uint16_t mouse_inp_handle;

    /* report char */
    struct report rpts[MAX_REPORTS];
    uint8_t rpts_len;

    /* report map char */
    uint8_t report_map[REPORT_MAP_SIZE];
    uint16_t report_map_handle;
    uint16_t external_rpt_ref;
    uint8_t report_map_len;

    /* hid info char */
    uint32_t hid_info;
    uint16_t hid_info_handle;

    /* hid control point char */
    uint8_t ctrl_pt;
    uint16_t ctrl_pt_handle;
};

void ble_svc_hid_init(void);
int ble_svc_hid_add(struct ble_svc_hid_params params);
void ble_svc_hid_reset(void);

#endif
#endif // CONFIG_BT_NIMBLE_HID_SERVICE
