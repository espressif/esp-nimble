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

#ifndef _HCI_H4_H_
#define _HCI_H4_H_

#include <stdint.h>

#define HCI_H4_NONE      0x00
#define HCI_H4_CMD       0x01
#define HCI_H4_ACL       0x02
#define HCI_H4_EVT       0x04
#define HCI_H4_ISO       0x05

typedef void *(hci_h4_alloc_cmd)(void);
typedef void *(hci_h4_alloc_evt)(int);
typedef struct os_mbuf *(hci_h4_alloc_acl)(void);
typedef struct os_mbuf *(hci_h4_alloc_iso)(void);

struct hci_h4_allocators {
    hci_h4_alloc_cmd *cmd;
    hci_h4_alloc_acl *acl;
    hci_h4_alloc_evt *evt;
    hci_h4_alloc_iso *iso;
};

extern const struct hci_h4_allocators hci_h4_allocs_from_ll;
extern const struct hci_h4_allocators hci_h4_allocs_from_hs;

typedef int (hci_h4_frame_cb)(uint8_t pkt_type, void *data);

struct hci_h4_sm {
    uint8_t state;
    uint8_t pkt_type;
    uint8_t min_len;
    uint16_t len;
    uint16_t exp_len;
    uint8_t hdr[4];
    union {
        uint8_t *buf;
        struct os_mbuf *om;
    };

    const struct hci_h4_allocators *allocs;
    hci_h4_frame_cb *frame_cb;
};

void hci_h4_sm_init(struct hci_h4_sm *h4sm,
                    const struct hci_h4_allocators *allocs,
                    hci_h4_frame_cb *frame_cb);

/* Free a partial frame, then clear the parser. Call after RX has stopped. */
void hci_h4_sm_deinit(struct hci_h4_sm *h4sm);

/* Controller->host octets were lost. Enter resync and queue HCI_Reset. */
void hci_h4_sm_sync_lost(struct hci_h4_sm *h4sm);

int hci_h4_sm_rx(struct hci_h4_sm *h4sm, const uint8_t *buf, uint16_t len);

/*
 * Tell a controller->host framer that HCI_Reset has been queued.
 * After sync loss the framer ignores bytes until this, then accepts only
 * the Command Complete for that Reset.
 */
void hci_h4_sm_note_host_reset_cmd(struct hci_h4_sm *h4sm);

#endif /* _HCI_H4_H_ */
