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
#include "esp_hci_driver.h"
#include "esp_hci_internal.h"

static int
ble_transport_dummy_host_recv_cb(hci_driver_data_type_t type, uint8_t *data, uint16_t len)
{
    if (type == HCI_DRIVER_TYPE_ACL) {
        os_mbuf_free_chain((struct os_mbuf *)data);
    } else if (type == HCI_DRIVER_TYPE_ISO) {
        free(data);
    } else {
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
        ble_transport_free(BLE_HCI_EVT, data);
#else
        ble_transport_free(data);
#endif
    }
    return 0;
}

static int
ble_transport_host_recv_cb(hci_driver_data_type_t type, uint8_t *data, uint16_t len)
{
    int rc = 0;
    if (type == HCI_DRIVER_TYPE_ACL) {
#if CONFIG_BT_NIMBLE_ROLE_CENTRAL || CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
        rc = ble_transport_to_hs_acl((struct os_mbuf *)data);
#else
        os_mbuf_free_chain((struct os_mbuf *)data);
#endif
    }
#if MYNEWT_VAL(BLE_ISO)
    else if (type == HCI_DRIVER_TYPE_ISO) {
        rc = ble_transport_to_hs_iso_v2(data, len);
    }
#else
    else if (type == HCI_DRIVER_TYPE_ISO) {
        free(data);
    }
#endif /* MYNEWT_VAL(BLE_ISO) */
    else if (type == HCI_DRIVER_TYPE_EVT) {
        rc = ble_transport_to_hs_evt(data);
    } else {
#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
        ble_transport_free(BLE_HCI_EVT, data);
#else
        ble_transport_free(data);
#endif
    }
    return rc;
}

int
ble_transport_to_ll_cmd_impl(void *buf)
{
    hci_driver_packet_t *pkt;
    uint16_t len;

    pkt = HCI_DRIVER_D2P(buf);
    /* buf layout is [Opcode L][Opcode H][Len][Params] (no type prefix);
     * buf+2 IS the parameter length byte and len+3 correctly covers opcode(2)+len(1)+params. */
    len = *(uint8_t *)(buf + 2);
    pkt->length = len + 3;
    return hci_driver_host_cmd_tx((uint8_t *)pkt);
}

int
ble_transport_to_ll_acl_impl(struct os_mbuf *om)
{
    int rc = hci_driver_host_acl_tx((uint8_t *)om, 0);
    if (rc != 0) {
        os_mbuf_free_chain(om);
    }
    return rc;
}

void
ble_transport_ll_init(void)
{
    hci_driver_host_callback_register(ble_transport_host_recv_cb);
}

void
ble_transport_ll_deinit(void)
{
    hci_driver_host_callback_register(ble_transport_dummy_host_recv_cb);
}

void *
ble_transport_alloc_cmd(void)
{
    hci_driver_packet_t *pkt = NULL;
    
    pkt = btdm_hci_trans_buf_alloc(HCI_DRIVER_TYPE_CMD, 0);
    assert(pkt);
    pkt->type = HCI_DRIVER_TYPE_CMD;

    return pkt->data;   
}

#if MYNEWT_VAL(MP_RUNTIME_ALLOC)
void
ble_transport_free(uint8_t type, void *buf)
{
    r_ble_hci_trans_buf_free(buf);
}
#else
void
ble_transport_free(void *buf)
{
    r_ble_hci_trans_buf_free(buf);
}
#endif
