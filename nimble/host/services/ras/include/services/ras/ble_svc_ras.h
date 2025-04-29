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


#ifndef H_BLE_SVC_RAS_
#define H_BLE_SVC_RAS_

#include <stdint.h>
#include "host/ble_hs.h"
#include "host/ble_cs.h"

#define BLE_SVC_RAS_RANGING_SERVICE_VAL (0x185B)

/** @brief UUID of the RAS Features Characteristic. **/
#define BLE_SVC_RAS_CHR_UUID_FEATURES_VAL (0x2C14)

/** @brief UUID of the Real-time Ranging Data Characteristic. **/
#define BLE_SVC_RAS_CHR_UUID_REALTIME_RD_VAL (0x2C15)

/** @brief UUID of the On-demand Ranging Data Characteristic. **/
#define BLE_SVC_RAS_CHR_UUID_ONDEMAND_RD_VAL (0x2C16)

/** @brief UUID of the RAS Control Point Characteristic. **/
#define BLE_SVC_RAS_CHR_UUID_CP_VAL (0x2C17)

/** @brief UUID of the Ranging Data Ready Characteristic. **/
#define BLE_SVC_RAS_CHR_UUID_RD_READY_VAL (0x2C18)

/** @brief UUID of the Ranging Data Overwritten Characteristic. **/
#define BLE_SVC_RAS_CHR_UUID_RD_OVERWRITTEN_VAL (0x2C19)


#define BLE_RAS_RANGING_HEADER_LEN  4
#define BLE_RAS_SUBEVENT_HEADER_LEN 8
#define BLE_RAS_STEP_MODE_LEN       1

#define BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE 32
#define BLE_RAS_MAX_STEPS_PER_PROCEDURE     256

#define BLE_RAS_MAX_STEP_DATA_LEN  32
#define BLE_RAS_PROCEDURE_MEM                                                                       \
	(BLE_RAS_RANGING_HEADER_LEN +                                                               \
	 (BLE_RAS_MAX_SUBEVENTS_PER_PROCEDURE * BLE_RAS_SUBEVENT_HEADER_LEN) +                       \
	 (BLE_RAS_MAX_STEPS_PER_PROCEDURE * BLE_RAS_STEP_MODE_LEN) +                                 \
	 (BLE_RAS_MAX_STEPS_PER_PROCEDURE * BLE_RAS_MAX_STEP_DATA_LEN))

#define REAL_TIME_RANGING_DATA_BIT     (1<<0)
#define RETRIEVE_LST_SEG_BIT           (1<<1)
#define ABORT_OP_BIT                   (1<<2)
#define FLTR_RANGING_DATA_BIT          (1<<3)

#define RASCP_CMD_OPCODE_LEN     1
#define RASCP_CMD_OPCODE_OFFSET  0
#define RASCP_CMD_PARAMS_OFFSET  RASCP_CMD_OPCODE_LEN
#define RASCP_CMD_PARAMS_MAX_LEN 4
#define RASCP_WRITE_MAX_LEN      (RASCP_CMD_OPCODE_LEN + RASCP_CMD_PARAMS_MAX_LEN)
#define RASCP_ACK_DATA_TIMEOUT   K_SECONDS(5)

/** @brief RAS Control Point opcodes as defined in RAS Specification, Table 3.10. */
enum rascp_opcode {
    RASCP_OPCODE_GET_RD                    = 0x00,
    RASCP_OPCODE_ACK_RD                    = 0x01,
    RASCP_OPCODE_RETRIEVE_LOST_RD_SEGMENTS = 0x02,
    RASCP_OPCODE_ABORT_OP                  = 0x03,
    RASCP_OPCODE_SET_FILTER                = 0x04,
};

/** @brief RAS Control Point Response opcodes as defined in RAS Specification, Table 3.11. */
enum rascp_rsp_opcode {
    RASCP_RSP_OPCODE_COMPLETE_RD_RSP          = 0x00,
    RASCP_RSP_OPCODE_COMPLETE_LOST_RD_SEG_RSP = 0x01,
    RASCP_RSP_OPCODE_RSP_CODE                 = 0x02,
};

#define RASCP_RSP_OPCODE_COMPLETE_RD_RSP_LEN          1
#define RASCP_RSP_OPCODE_COMPLETE_LOST_RD_SEG_RSP_LEN 4
#define RASCP_RSP_OPCODE_RSP_CODE_LEN                 1

struct ranging_header {
    /** Lower 12 bits used as the ranging session counter. */
    uint16_t ranging_counter : 12;

    /** Configuration identifier (4-bit value, range 0–3). */
    uint8_t config_id : 4;

    /** Transmit power used during the ranging session, in dBm (range: -127 to 20). */
    int8_t selected_tx_power;

    /** Bitmask indicating which antenna paths are active.
     *  Bit 0: Path 1
     *  Bit 1: Path 2
     *  Bit 2: Path 3
     *  Bit 3: Path 4
     *  Bits 4–7: Reserved
     */
    uint8_t antenna_paths_mask;
} __attribute__((packed));

struct subevent_header {
    /** Starting ACL connection event counter for the reported results. */
    uint16_t start_acl_conn_event;

    /** Frequency compensation value in 0.01 ppm units (15-bit signed integer).
     *  May be a reserved or unavailable value depending on the device role or support.
     */
    uint16_t freq_compensation;

    /** Status indicating completion of the overall ranging process.
     *  0x0: Complete
     *  0x1: Partial, more results to come
     *  0xF: Procedure aborted
     *  Other values: Reserved
     */
    uint8_t ranging_done_status : 4;

    /** Status for the subevent portion of the ranging process.
     *  0x0: Complete
     *  0xF: Aborted
     *  Other values: Reserved
     */
    uint8_t subevent_done_status : 4;

    /** Reason code for aborting the entire procedure.
     *  0x0: No abort
     *  0x1: Abort due to local/remote request
     *  0x2: Insufficient valid channels
     *  0x3: Instant missed
     *  0xF: Unspecified reason
     *  Other values: Reserved
     */
    uint8_t ranging_abort_reason : 4;

    /** Reason code for subevent-level aborts.
     *  0x0: No abort
     *  0x1: Abort due to local/remote request
     *  0x2: No sync packet received
     *  0x3: Scheduling/resource issue
     *  0xF: Unspecified reason
     *  Other values: Reserved
     */
    uint8_t subevent_abort_reason : 4;

    /** Reference transmit power used during the subevent (in dBm). */
    int8_t ref_power_level;

    /** Number of steps included in the report. May be zero if aborted. */
    uint8_t num_steps_reported;
}  __attribute__((packed));

struct ranging_buffer {
    /** Pointer to the associated Bluetooth connection. */
    int conn;

    /** Counter used for identifying the current ranging session. */
    uint16_t ranging_counter;

    /** Index for tracking the current write position in subevent data. */
    uint16_t subevent_cursor;

    /** Reference count to ensure safe access and prevent premature reuse. */
    uint8_t refcount;

    /** Indicates whether the buffer is completely filled and ready to transmit. */
    uint8_t isready;

    /** Flag showing that the buffer is currently in use for writing data. */
    uint8_t isbusy;

    /** Set when the remote side has acknowledged this buffer. */
    uint8_t isacked;

    /** Holds the entire data payload for a ranging operation. */
    union {
        uint8_t buf[BLE_RAS_PROCEDURE_MEM];
        struct {
            struct ranging_header ranging_header;
            uint8_t subevents[0];
        } __attribute__((packed));
    } ranging_data;
};


struct segment_header {
    /** Indicates if this is the first segment in a series. */
    bool first_seg : 1;

    /** Indicates if this is the last segment in a series. */
    bool last_seg : 1;

    /** Sequence number for this segment (6-bit counter). */
    uint8_t seg_counter : 6;
} __attribute__((packed));



/** Generic format for a data segment containing a header and payload. */
struct segment {
    struct segment_header header;

    /** Payload data of variable length. */
    uint8_t data[];
}  __attribute__((packed));


void ble_gatts_indicate_ranging_data_ready(uint16_t ranging_counter);
void ble_gatts_store_ranging_data(struct ble_cs_event ranging_subevent);
void ble_gatts_indicate_control_point_response(uint16_t attr_handle , uint16_t ranging_counter);
struct ranging_buffer *ranging_buffer_alloc(uint16_t conn_handle , uint16_t ranging_counter);
void ble_svc_ras_init(void);


#endif
