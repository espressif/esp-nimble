/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <sysinit/sysinit.h>
#include <syscfg/syscfg.h>
#include "os/os_mbuf.h"
#include "nimble/transport.h"
#include "esp_hci_transport.h"
#include "esp_hci_internal.h"

static int
ble_transport_dummy_host_recv_cb(hci_trans_pkt_ind_t type, uint8_t *data, uint16_t len)
{
    /* Dummy function */
    return 0;
}

static int
ble_transport_host_recv_cb(hci_trans_pkt_ind_t type, uint8_t *data, uint16_t len)
{
    int rc;

    if (type == HCI_ACL_IND) {
        rc = ble_transport_to_hs_acl((struct os_mbuf *)data);
    } else {
        rc = ble_transport_to_hs_evt(data);
    }
    return rc;
}

int
ble_transport_to_ll_cmd_impl(void *buf)
{
    return hci_transport_host_cmd_tx(buf, 0);
}

int
ble_transport_to_ll_acl_impl(struct os_mbuf *om)
{
    return hci_transport_host_acl_tx((uint8_t *)om, 0);
}

void
ble_transport_ll_init(void)
{
    hci_transport_host_callback_register(ble_transport_host_recv_cb);
}

void
ble_transport_ll_deinit(void)
{
    hci_transport_host_callback_register(ble_transport_dummy_host_recv_cb);
}

void *
ble_transport_alloc_cmd(void)
{
    return r_ble_hci_trans_buf_alloc(ESP_HCI_INTERNAL_BUF_CMD);
}

void
ble_transport_free(void *buf)
{
    r_ble_hci_trans_buf_free(buf);
}
