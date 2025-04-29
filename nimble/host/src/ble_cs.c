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
#include "syscfg/syscfg.h"
#include "ble_hs_mbuf_priv.h"

#if MYNEWT_VAL(BLE_CHANNEL_SOUNDING)

#include "os/os_mbuf.h"
#include "host/ble_hs_log.h"
#include "host/ble_hs.h"
#include "host/ble_cs.h"
#include "nimble/hci_common.h"
#include "sys/queue.h"
#include "ble_hs_hci_priv.h"

#define BT_LE_CS_CHANNEL_BIT_SET_VAL(chmap, bit, val)                                              \
((chmap)[(bit) / 8] = ((chmap)[(bit) / 8] & ~BIT((bit) % 8)) | ((val) << ((bit) % 8)))


struct ble_cs_rd_rem_supp_cap_cp {
    uint16_t conn_handle;
} __attribute__((packed));

struct ble_cs_wr_cached_rem_supp_cap_cp {
    uint16_t conn_handle;
    uint8_t num_config_supported;
    uint16_t max_consecutive_procedures_supported;
    uint8_t num_antennas_supported;
    uint8_t max_antenna_paths_supported;
    uint8_t roles_supported;
    uint8_t optional_modes_supported;
    uint8_t rtt_capability;
    uint8_t rtt_aa_only_n;
    uint8_t rtt_sounding_n;
    uint8_t rtt_random_payload_n;
    uint16_t optional_nadm_sounding_capability;
    uint16_t optional_nadm_random_capability;
    uint8_t optional_cs_sync_phys_supported;
    uint16_t optional_subfeatures_supported;
    uint16_t optional_t_ip1_times_supported;
    uint16_t optional_t_ip2_times_supported;
    uint16_t optional_t_fcs_times_supported;
    uint16_t optional_t_pm_times_supported;
    uint8_t t_sw_time_supported;
} __attribute__((packed));
struct ble_cs_wr_cached_rem_supp_cap_rp {
    uint16_t conn_handle;
} __attribute__((packed));

struct ble_cs_sec_enable_cp {
    uint16_t conn_handle;
} __attribute__((packed));

struct ble_cs_set_def_settings_cp {
    uint16_t conn_handle;
    uint8_t role_enable;
    uint8_t cs_sync_antenna_selection;
    uint8_t max_tx_power;
} __attribute__((packed));
struct ble_cs_set_def_settings_rp {
    uint16_t conn_handle;
} __attribute__((packed));

struct ble_cs_rd_rem_fae_cp {
    uint16_t conn_handle;
} __attribute__((packed));

#if 0
struct ble_cs_wr_cached_rem_fae_cp {
    uint16_t conn_handle;
    uint8_t remote_fae_table[72];
} __attribute__((packed));
struct ble_cs_wr_cached_rem_fae_rp {
    uint16_t conn_handle;
} __attribute__((packed));

#endif
struct ble_cs_create_config_cp {
    uint16_t conn_handle;
    uint8_t config_id;
    /* If the config should be created on the remote controller too */
    uint8_t create_context;
    /* The main mode to be used in the CS procedures */
    uint8_t main_mode_type;
    /* The sub mode to be used in the CS procedures */
    uint8_t sub_mode_type;
    /* Minimum/maximum number of CS main mode steps to be executed before
     * a submode step.
     */
    uint8_t min_main_mode_steps;
    uint8_t max_main_mode_steps;
    /* The number of main mode steps taken from the end of the last
     * CS subevent to be repeated at the beginning of the current CS subevent
     * directly after the last mode 0 step of that event
     */
    uint8_t main_mode_repetition;
    uint8_t mode_0_steps;
    uint8_t role;
    uint8_t rtt_type;
    uint8_t cs_sync_phy;
    uint8_t channel_map[10];
    uint8_t channel_map_repetition;
    uint8_t channel_selection_type;
    uint8_t ch3c_shape;
    uint8_t ch3c_jump;
    uint8_t companion_signal_enable;
} __attribute__((packed));

#if 0
struct ble_cs_remove_config_cp {
    uint16_t conn_handle;
    uint8_t config_id;
} __attribute__((packed));

struct ble_cs_set_chan_class_cp {
    uint8_t channel_classification[10];
} __attribute__((packed));

#endif
struct ble_cs_set_proc_params_cp {
    uint16_t conn_handle;
    uint8_t config_id;
    /* The maximum duration of each CS procedure (time = N × 0.625 ms) */
    uint16_t max_procedure_len;
    /* The minimum and maximum number of connection events between
     * consecutive CS procedures. Ignored if only one CS procedure. */
    uint16_t min_procedure_interval;
    uint16_t max_procedure_interval;
    /* The maximum number of consecutive CS procedures to be scheduled */
    uint16_t max_procedure_count;
    /* Minimum/maximum suggested durations for each CS subevent in microseconds.
     * Only 3 bytes meaningful. */
    uint32_t min_subevent_len;
    uint32_t max_subevent_len;
    /* Antenna Configuration Index (ACI) for swithing during phase measuement */
    uint8_t tone_antenna_config_selection;
    /* The remote device’s Tx PHY. A 4 bits bitmap. */
    uint8_t phy;
    /* How many more or fewer of transmit power levels should the remote device’s
     * Tx PHY use during the CS tones and RTT transmission */
    uint8_t tx_power_delta;
    /* Preferred peer-ordered antenna elements to be used by the peer for
     * the selected antenna configuration (ACI). A 4 bits bitmap. */
    uint8_t preferred_peer_antenna;
    /* SNR Output Index (SOI) for SNR control adjustment. */
    uint8_t snr_control_initiator;
    uint8_t snr_control_reflector;
} __attribute__((packed));
struct ble_cs_set_proc_params_rp {
    uint16_t conn_handle;
} __attribute__((packed));

struct ble_cs_proc_enable_cp {
    uint16_t conn_handle;
    uint8_t config_id;
    uint8_t enable;
} __attribute__((packed));

struct ble_cs_state {
    uint8_t op;
    ble_cs_event_fn *cb;
    void *cb_arg;
};

static struct ble_cs_state cs_state;

static int
ble_cs_call_event_cb(struct ble_cs_event *event)
{
    int rc;

    if (cs_state.cb != NULL) {
        rc = cs_state.cb(event, cs_state.cb_arg);
    } else {
        rc = 0;
    }

    return rc;
}

static void
ble_cs_call_procedure_complete_cb(uint16_t conn_handle, uint8_t status)
{

    struct ble_cs_event event;

    memset(&event, 0, sizeof event);
    event.type = BLE_CS_EVENT_CS_PROCEDURE_COMPLETE;
    event.procedure_complete.conn_handle = conn_handle;
    event.procedure_complete.status = status;
    ble_cs_call_event_cb(&event);
}

static void
ble_cs_call_result_cb(const struct ble_hci_ev_le_subev_cs_subevent_result *ev)
{
    struct ble_cs_event event;

    memset(&event,0,sizeof event);

    event.type = BLE_CS_EVENT_SUBEVET_RESULT;
    event.subev_result.conn_handle = le16toh(ev->conn_handle);
    event.subev_result.config_id = ev->config_id;
    event.subev_result.start_acl_conn_event_counter=ev->start_acl_conn_event_counter;
    event.subev_result.procedure_counter=ev->procedure_counter;
    event.subev_result.frequency_compensation=ev->frequency_compensation;
    event.subev_result.reference_power_level=ev->reference_power_level;
    event.subev_result.procedure_done_status=ev->procedure_done_status;
    event.subev_result.subevent_done_status=ev->subevent_done_status;
    event.subev_result.abort_reason=ev->abort_reason;
    event.subev_result.num_antenna_paths=ev->num_antenna_paths;
    event.subev_result.num_steps_reported=ev->num_steps_reported;
    event.subev_result.steps=ev->steps;
    //memcpy(event.subev_result.steps, ev->steps, 5);

    ble_cs_call_event_cb(&event);
}

static void ble_cs_call_result_continue_cb(const struct ble_hci_ev_le_subev_cs_subevent_result_continue *ev){
    struct ble_cs_event event;
    memset(&event,0,sizeof event);
    event.type = BLE_CS_EVENT_SUBEVET_RESULT_CONTINUE;
    event.subev_result_continue.conn_handle=le16toh(ev->conn_handle);
    event.subev_result_continue.config_id=ev->config_id;
    event.subev_result_continue.procedure_done_status=ev->procedure_done_status;
    event.subev_result_continue.subevent_done_status=ev->procedure_done_status;
    event.subev_result_continue.abort_reason=ev->abort_reason;
    event.subev_result_continue.num_antenna_paths=ev->num_antenna_paths;
    event.subev_result_continue.num_steps_reported=ev->num_steps_reported;
    MODLOG_DFLT(INFO, "event.subev_result_continue.num_steps_reported= %d\n",ev->num_steps_reported);
    event.subev_result_continue.steps = ev->steps;
   // memcpy(event.subev_result_continue.steps, ev->steps, ev->num_steps_reported * sizeof(event.subev_result_continue.steps[0]));
    ble_cs_call_event_cb(&event);
}
#if 0
static int
ble_cs_rd_loc_supp_cap(void)
{
    int rc;
    struct ble_hci_le_cs_rd_loc_supp_cap_rp rp;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_CS_RD_LOC_SUPP_CAP),
                           NULL, 0, &rp, sizeof(rp));

    rp.max_consecutive_procedures_supported = le16toh(rp.max_consecutive_procedures_supported);
    rp.optional_nadm_sounding_capability = le16toh(rp.optional_nadm_sounding_capability);
    rp.optional_nadm_random_capability = le16toh(rp.optional_nadm_random_capability);
    rp.optional_subfeatures_supported = le16toh(rp.optional_subfeatures_supported);
    rp.optional_t_ip1_times_supported = le16toh(rp.optional_t_ip1_times_supported);
    rp.optional_t_ip2_times_supported = le16toh(rp.optional_t_ip2_times_supported);
    rp.optional_t_fcs_times_supported = le16toh(rp.optional_t_fcs_times_supported);
    rp.optional_t_pm_times_supported = le16toh(rp.optional_t_pm_times_supported);
    (void) rp;

    return rc;
}
#endif

static int
ble_cs_rd_rem_supp_cap(const struct ble_cs_rd_rem_supp_cap_cp *cmd)
{
    struct ble_hci_le_cs_rd_rem_supp_cap_cp cp;

    cp.conn_handle = htole16(cmd->conn_handle);

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CS_RD_REM_SUPP_CAP),
                             &cp, sizeof(cp), NULL, 0);
}

#if 0
static int
ble_cs_wr_cached_rem_supp_cap(const struct ble_cs_wr_cached_rem_supp_cap_cp *cmd,
                              struct ble_cs_wr_cached_rem_supp_cap_rp *rsp)
{
    struct ble_hci_le_cs_wr_cached_rem_supp_cap_cp cp;
    struct ble_hci_le_cs_wr_cached_rem_supp_cap_rp rp;
    int rc;

    cp.conn_handle = htole16(cmd->conn_handle);
    cp.num_config_supported = cmd->num_config_supported;
    cp.max_consecutive_procedures_supported = htole16(cmd->max_consecutive_procedures_supported);
    cp.num_antennas_supported = cmd->num_antennas_supported;
    cp.max_antenna_paths_supported = cmd->max_antenna_paths_supported;
    cp.roles_supported = cmd->roles_supported;
    cp.optional_modes_supported = cmd->optional_modes_supported;
    cp.rtt_capability = cmd->rtt_capability;
    cp.rtt_aa_only_n = cmd->rtt_aa_only_n;
    cp.rtt_sounding_n = cmd->rtt_sounding_n;
    cp.rtt_random_payload_n = cmd->rtt_random_payload_n;
    cp.optional_nadm_sounding_capability = htole16(cmd->optional_nadm_sounding_capability);
    cp.optional_nadm_random_capability = htole16(cmd->optional_nadm_random_capability);
    cp.optional_cs_sync_phys_supported = cmd->optional_cs_sync_phys_supported;
    cp.optional_subfeatures_supported = htole16(cmd->optional_subfeatures_supported);
    cp.optional_t_ip1_times_supported = htole16(cmd->optional_t_ip1_times_supported);
    cp.optional_t_ip2_times_supported = htole16(cmd->optional_t_ip2_times_supported);
    cp.optional_t_fcs_times_supported = htole16(cmd->optional_t_fcs_times_supported);
    cp.optional_t_pm_times_supported = htole16(cmd->optional_t_pm_times_supported);
    cp.t_sw_time_supported = cmd->t_sw_time_supported;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_CS_WR_CACHED_REM_SUPP_CAP),
                           &cp, sizeof(cp), &rp, sizeof(rp));

    rsp->conn_handle = le16toh(rp.conn_handle);

    return rc;
}
#endif

static int
ble_cs_sec_enable(const struct ble_cs_sec_enable_cp *cmd)
{
    struct ble_hci_le_cs_sec_enable_cp cp;

    cp.conn_handle = htole16(cmd->conn_handle);

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CS_SEC_ENABLE),
                             &cp, sizeof(cp), NULL, 0);
}

static int
ble_cs_set_def_settings(const struct ble_cs_set_def_settings_cp *cmd,
                        struct ble_cs_set_def_settings_rp *rsp)
{
    struct ble_hci_le_cs_set_def_settings_cp cp;
    struct ble_hci_le_cs_set_def_settings_rp rp;
    int rc;

    cp.conn_handle = htole16(cmd->conn_handle);
    cp.role_enable = cmd->role_enable;
    cp.cs_sync_antenna_selection = cmd->cs_sync_antenna_selection;
    cp.max_tx_power = cmd->max_tx_power;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_CS_SET_DEF_SETTINGS),
                           &cp, sizeof(cp), &rp, sizeof(rp));

    rsp->conn_handle = le16toh(rp.conn_handle);

    return rc;
}

static int
ble_cs_rd_rem_fae(const struct ble_cs_rd_rem_fae_cp *cmd)
{
    struct ble_hci_le_cs_rd_rem_fae_cp cp;

    cp.conn_handle = htole16(cmd->conn_handle);

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CS_RD_REM_FAE),
                             &cp, sizeof(cp), NULL, 0);
}

#if 0
static int
ble_cs_wr_cached_rem_fae(const struct ble_cs_wr_cached_rem_fae_cp *cmd,
                         struct ble_cs_wr_cached_rem_fae_rp *rsp)
{
    struct ble_hci_le_cs_wr_cached_rem_fae_cp cp;
    struct ble_hci_le_cs_wr_cached_rem_fae_rp rp;
    int rc;

    cp.conn_handle = htole16(cmd->conn_handle);
    memcpy(cp.remote_fae_table, cmd->remote_fae_table, 72);

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_CS_WR_CACHED_REM_FAE),
                           &cp, sizeof(cp), &rp, sizeof(rp));

    rsp->conn_handle = le16toh(rp.conn_handle);

    return rc;
}
#endif
static int
ble_cs_create_config(const struct ble_cs_create_config_cp *cmd)
{
    struct ble_hci_le_cs_create_config_cp cp;

    cp.conn_handle = htole16(cmd->conn_handle);

    cp.config_id = cmd->config_id;
    cp.create_context = cmd->create_context;
    cp.main_mode_type = cmd->main_mode_type;
    cp.sub_mode_type = cmd->sub_mode_type;
    cp.min_main_mode_steps = cmd->min_main_mode_steps;
    cp.max_main_mode_steps = cmd->max_main_mode_steps;
    cp.main_mode_repetition = cmd->main_mode_repetition;
    cp.mode_0_steps = cmd->mode_0_steps;
    cp.role = cmd->role;
    cp.rtt_type = cmd->rtt_type;
    cp.cs_sync_phy = cmd->cs_sync_phy;
    memcpy(cp.channel_map, cmd->channel_map, 10);
    cp.channel_map_repetition = cmd->channel_map_repetition;
    cp.channel_selection_type = cmd->channel_selection_type;
    cp.ch3c_shape = cmd->ch3c_shape;
    cp.ch3c_jump = cmd->ch3c_jump;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CS_CREATE_CONFIG),
                             &cp, sizeof(cp), NULL, 0);
}

#if 0
static int
ble_cs_remove_config(const struct ble_cs_remove_config_cp *cmd)
{
    struct ble_hci_le_cs_remove_config_cp cp;

    cp.conn_handle = htole16(cmd->conn_handle);
    cp.config_id = cmd->config_id;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CS_REMOVE_CONFIG),
                             &cp, sizeof(cp), NULL, 0);
}

static int
ble_cs_set_chan_class(const struct ble_cs_set_chan_class_cp *cmd)
{
    struct ble_hci_le_cs_set_chan_class_cp cp;
    int rc;

    memcpy(cp.channel_classification, cmd->channel_classification, 10);

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_CS_SET_CHAN_CLASS),
                           &cp, sizeof(cp), NULL, 0);

    return rc;
}

#endif
static int
ble_cs_set_proc_params(const struct ble_cs_set_proc_params_cp *cmd,
                       struct ble_cs_set_proc_params_rp *rsp)
{
    struct ble_hci_le_cs_set_proc_params_cp cp;
    struct ble_hci_le_cs_set_proc_params_rp rp;
    int rc;

    cp.conn_handle = htole16(cmd->conn_handle);
    cp.config_id = cmd->config_id;
    cp.max_procedure_len = htole16(cmd->max_procedure_len);
    cp.min_procedure_interval = htole16(cmd->min_procedure_interval);
    cp.max_procedure_interval = htole16(cmd->max_procedure_interval);
    cp.max_procedure_count = htole16(cmd->max_procedure_count);
    put_le24(cp.min_subevent_len, cmd->min_subevent_len);
    put_le24(cp.max_subevent_len, cmd->max_subevent_len);
    cp.tone_antenna_config_selection = cmd->tone_antenna_config_selection;
    cp.phy = cmd->phy;
    cp.tx_power_delta = cmd->tx_power_delta;
    cp.preferred_peer_antenna = cmd->preferred_peer_antenna;
    cp.snr_control_initiator = cmd->snr_control_initiator;
    cp.snr_control_reflector = cmd->snr_control_reflector;

    rc = ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                      BLE_HCI_OCF_LE_CS_SET_PROC_PARAMS),
                           &cp, sizeof(cp), &rp, sizeof(rp));

    rsp->conn_handle = le16toh(rp.conn_handle);

    return rc;
}

static int
ble_cs_proc_enable(const struct ble_cs_proc_enable_cp *cmd)
{
    struct ble_hci_le_cs_proc_enable_cp cp;

    cp.conn_handle = htole16(cmd->conn_handle);
    cp.config_id = cmd->config_id;
    cp.enable = cmd->enable;

    return ble_hs_hci_cmd_tx(BLE_HCI_OP(BLE_HCI_OGF_LE,
                                        BLE_HCI_OCF_LE_CS_PROC_ENABLE),
                             &cp, sizeof(cp), NULL, 0);
}

int
ble_hs_hci_evt_le_cs_rd_rem_supp_cap_complete(uint8_t subevent, const void *data,
                                              unsigned int len)
{

    int rc;
    const struct ble_hci_ev_le_subev_cs_rd_rem_supp_cap_complete *ev = data;
    struct ble_cs_set_def_settings_cp set_cmd;
    struct ble_cs_set_def_settings_rp set_rsp;
    struct ble_cs_rd_rem_fae_cp fae_cmd;
    struct ble_gap_conn_desc desc;

    if (len != sizeof(*ev) || ev->status) {

        return BLE_HS_ECONTROLLER;
    }

    BLE_HS_LOG(INFO, "CS capabilities exchanged");

    /* TODO: Save the remote capabilities somewhere */

    set_cmd.conn_handle = le16toh(ev->conn_handle);
    /* Only initiator role is enabled */

    set_cmd.role_enable = 0x00;

    rc = ble_gap_conn_find(ev->conn_handle, &desc);
    if (desc.role == BLE_GAP_ROLE_MASTER) {
        set_cmd.role_enable |= 1;
    } else{
        set_cmd.role_enable |= 2;
    }
    /* Use antenna with ID 0x01 */
    set_cmd.cs_sync_antenna_selection = 0xFE;
    /* Set max TX power to the max supported */
    set_cmd.max_tx_power = 20;

    rc = ble_cs_set_def_settings(&set_cmd, &set_rsp);
    if (rc) {
        BLE_HS_LOG(INFO, "Failed to set the default CS settings, err %dt", rc);

        return rc;
    }
     BLE_HS_LOG(INFO, "Set default CS settings ");



    /* Read the mode 0 Frequency Actuation Error table */
        fae_cmd.conn_handle = le16toh(ev->conn_handle);
        rc = ble_cs_rd_rem_fae(&fae_cmd);
        if (rc) {
            BLE_HS_LOG(INFO, "Failed to read FAE table");
        }

    return rc;
}

int
ble_hs_hci_evt_le_cs_rd_rem_fae_complete(uint8_t subevent, const void *data,
                                         unsigned int len)
{

    struct ble_gap_conn_desc desc;
    const struct ble_hci_ev_le_subev_cs_rd_rem_fae_complete *ev = data;
    struct ble_cs_create_config_cp cmd;
    memset(&cmd,0,sizeof cmd);
    int rc=0;

    if (len != sizeof(*ev) || ev->status) {
        //return BLE_HS_ECONTROLLER;
    }

    cmd.conn_handle = le16toh(ev->conn_handle);
    /* The config will use ID 0x00 */
    cmd.config_id = 0x00;
    /* Create the config on the remote controller too */
    cmd.create_context = 0x01;
    /* Measure phase rotations in main mode */
    cmd.main_mode_type = 0x02;
    /* Use sub mode 0xFF for now, meaning no sub mode */
    cmd.sub_mode_type = 0x01;

    /* Range from which the number of CS main mode steps to execute
     * will be randomly selected.
     */
    cmd.min_main_mode_steps = 10;
    cmd.max_main_mode_steps = 20;
    /* The number of main mode steps to be repeated at the beginning of
     * the current CS, irrespectively if there are some overlapping main
     * mode steps from previous CS subevent or not.
     */
    cmd.main_mode_repetition = 0x00;
    /* Number of CS mode 0 steps to be included at the beginning of
     * each CS subevent
     */
    cmd.mode_0_steps = 0x03;
    /* Take the Initiator role */

    ble_gap_conn_find(ev->conn_handle, &desc);

    cmd.role= 0x00;
    if (desc.role == BLE_GAP_ROLE_MASTER) {
        cmd.role = 0x00;
    } else {
        cmd.role = 0x01;
    }

    /* Use RTT type 0x00 for now */
    cmd.rtt_type = 0x00;
    cmd.cs_sync_phy = 0x01;
    memcpy(cmd.channel_map, (uint8_t[10]) {0x08, 0xfa, 0x4f, 0xac, 0xfa, 0xc0}, 10);


    cmd.channel_map_repetition = 5;
    /* Use Channel Selection Algorithm #3b */
    cmd.channel_selection_type = 0x00;
    /* Ignore these as used only with #3c algorithm */
    cmd.ch3c_shape = 0x00;

    cmd.ch3c_jump = 0x02;
    /* EDLC/ECLD attack protection not supported */
  //  cmd.companion_signal_enable = 0x00;

    /* Create CS config */

   if (desc.role == BLE_GAP_ROLE_MASTER) {
        rc = ble_cs_create_config(&cmd);
        if (rc) {
            BLE_HS_LOG(INFO, "Failed to create CS config");
        }
    }
    return rc;
}

int
ble_hs_hci_evt_le_cs_sec_enable_complete(uint8_t subevent, const void *data,
                                         unsigned int len)
{

    int rc;
    struct ble_cs_set_proc_params_cp cmd;
    struct ble_cs_set_proc_params_rp rsp;
    struct ble_cs_proc_enable_cp enable_cmd;
    const struct ble_hci_ev_le_subev_cs_sec_enable_complete *ev = data;
    struct ble_gap_conn_desc desc;

    if (len != sizeof(*ev)) {
        BLE_HS_LOG(INFO, "Failed to enable CS security BLE_HS_ECNOTEROLLER");

        return BLE_HS_ECONTROLLER;
    }

    BLE_HS_LOG(INFO, "CS setup phase completed");

    cmd.conn_handle = le16toh(ev->conn_handle);
    cmd.config_id = 0x00;
    /* The maximum duration of each CS procedure (time = N × 0.625 ms) */
    cmd.max_procedure_len = 100;
    /* The maximum number of consecutive CS procedures to be scheduled
     * as part of this measurement
     */
    cmd.max_procedure_count = 1;
    /* The minimum and maximum number of connection events between
     * consecutive CS procedures. Ignored if only one CS procedure.
     */
    cmd.min_procedure_interval = 10;
    cmd.max_procedure_interval = 10;
    /* Minimum/maximum suggested durations for each CS subevent in microseconds.
     * 1250us and 5000us selected.
     */
    cmd.min_subevent_len = 60000;
    cmd.max_subevent_len = 60000;
    /* Use ACI 0 as we have only one CS setup phase completedantenna on each side */
    cmd.tone_antenna_config_selection = 0x00;
    /* Use LE 1M PHY for CS procedures */
    cmd.phy = 0x01;
    /* Transmit power delta set to 0x80 means Host does not have a recommendation. */
    cmd.tx_power_delta = 0x80;
    /* Preferred antenna array elements to use. We have only a single antenna here. */
    cmd.preferred_peer_antenna = 0x01;
    /* SNR Output Index (SOI) for SNR control adjustment. 0xFF means SNR control
     * is not to be applied.
     */
    cmd.snr_control_initiator = 0xff;
    cmd.snr_control_reflector = 0xff;

    rc = ble_cs_set_proc_params(&cmd, &rsp);
    if (rc) {
        BLE_HS_LOG(INFO, "Failed to set CS procedure parameters");
    }else{
        BLE_HS_LOG(INFO, "CS procedure parameters set");
    }

    enable_cmd.conn_handle = le16toh(ev->conn_handle);
    enable_cmd.config_id = 0x00;
    enable_cmd.enable = 0x01;

    ble_gap_conn_find(ev->conn_handle, &desc);
if (desc.role == BLE_GAP_ROLE_MASTER) {

    rc = ble_cs_proc_enable(&enable_cmd);
    if (rc) {
        BLE_HS_LOG(INFO, "Failed to enable CS procedure");
    }else{
        BLE_HS_LOG(INFO, "CS procedure enabled");
    }
    return rc;
}
return 0;
}

int
ble_hs_hci_evt_le_cs_config_complete(uint8_t subevent, const void *data,
                                     unsigned int len)
{
    int rc;
    struct ble_gap_conn_desc desc;
    const struct ble_hci_ev_le_subev_cs_config_complete *ev = data;
    struct ble_cs_sec_enable_cp cmd;
    // struct ble_hs_conn *conn;

    if (len != sizeof(*ev) || ev->status) {
        return BLE_HS_ECONTROLLER;
    }

    cmd.conn_handle = le16toh(ev->conn_handle);

    /* Exchange CS security keys */
    rc = ble_gap_conn_find(ev->conn_handle, &desc);

    if (desc.role == BLE_GAP_ROLE_MASTER) {

        rc = ble_cs_sec_enable(&cmd);
        if (rc) {
            BLE_HS_LOG(DEBUG, "Failed to enable CS security");
            ble_cs_call_procedure_complete_cb(le16toh(ev->conn_handle), ev->status);
        }
    }

    return 0;
}

int
ble_hs_hci_evt_le_cs_proc_enable_complete(uint8_t subevent, const void *data,
                                          unsigned int len)
{
    const struct ble_hci_ev_le_subev_cs_proc_enable_complete *ev = data;

    if (len != sizeof(*ev) || ev->status) {

        return BLE_HS_ECONTROLLER;
    }

    return 0;
}

int
ble_hs_hci_evt_le_cs_subevent_result(uint8_t subevent, const void *data,
                                     unsigned int len)
{
    const struct ble_hci_ev_le_subev_cs_subevent_result *event = data;
    int expected_len = 0;
    void *step_ptr = NULL;
    int steps_remaining = 0;
    int step_size = 0;

    expected_len += sizeof(*event);
    steps_remaining = event->num_steps_reported;
    step_ptr = (void *)event->steps;

    while (steps_remaining) {
        step_size = sizeof(struct cs_steps_data) + ((struct cs_steps_data *)step_ptr)->data_len;
        expected_len += step_size;
        step_ptr += step_size;
        steps_remaining--;
    }

    if (len != expected_len) {
        MODLOG_DFLT(INFO, "ble_hs_hci_evt_le_cs_subevent_result\n len :%d  expected_len : %d\n", len, expected_len);
        return BLE_HS_ECONTROLLER;
    }

    ble_cs_call_result_cb(event);
    return 0;
}

int
ble_hs_hci_evt_le_cs_subevent_result_continue(uint8_t subevent, const void *data,
                                              unsigned int len)
{
    const struct ble_hci_ev_le_subev_cs_subevent_result_continue *event = data;
    int expected_len = 0;
    int steps_remaining = 0;
    void *step_ptr = NULL;
    int step_size = 0;

    expected_len = sizeof(*event);
    steps_remaining = event->num_steps_reported;
    step_ptr = (void *)event->steps;

    while (steps_remaining) {
        step_size = sizeof(struct cs_steps_data) + ((struct cs_steps_data *)step_ptr)->data_len;
        step_ptr += step_size;
        expected_len += step_size;
        steps_remaining--;
    }

    if (len != expected_len) {
        MODLOG_DFLT(INFO, "ble_hs_hci_evt_le_cs_subevent_result_continue len: %d     expected_len: %d\n", len, expected_len);
        return BLE_HS_ECONTROLLER;
    }
    ble_cs_call_result_continue_cb(event);
    return 0;
}

int
ble_hs_hci_evt_le_cs_test_end_complete(uint8_t subevent, const void *data,
                                       unsigned int len)
{
    const struct ble_hci_ev_le_subev_cs_test_end_complete *ev = data;

    if (len != sizeof(*ev)) {
        return BLE_HS_ECONTROLLER;
    }

    return 0;
}


int
ble_cs_initiator_procedure_start(const struct ble_cs_initiator_procedure_start_params *params)
{

    struct ble_cs_rd_rem_supp_cap_cp cmd;
    int rc;

    /* Channel Sounding setup phase:
     * 1. Set local default CS settings
     * 2. Exchange CS capabilities with the remote
     * 3. Read or write the mode 0 Frequency Actuation Error table
     * 4. Create CS configurations
     * 5. Start the CS Security Start procedure
     */

    cs_state.cb = params->cb;
    cs_state.cb_arg = params->cb_arg;

    cmd.conn_handle = params->conn_handle;
    rc = ble_cs_rd_rem_supp_cap(&cmd);
    if (rc) {
        BLE_HS_LOG(DEBUG, "Failed to read local supported CS capabilities,"
                   "err %dt", rc);
    }

    return rc;
}

int
ble_cs_initiator_procedure_terminate(uint16_t conn_handle)
{
    return 0;
}

int
ble_cs_reflector_setup(struct ble_cs_reflector_setup_params *params)
{
    cs_state.cb = params->cb;
    cs_state.cb_arg = params->cb_arg;

    return 0;
}
#endif
