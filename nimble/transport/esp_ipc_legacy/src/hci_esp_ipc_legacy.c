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
#include "nimble/transport.h"
#include "esp_nimble_hci.h"

/* This file is only used by ESP32, ESP32C3 and ESP32S3. */
int
ble_transport_to_ll_cmd_impl(void *buf)
{
    return ble_hci_trans_hs_cmd_tx(buf);
}

int
ble_transport_to_ll_acl_impl(struct os_mbuf *om)
{
    return ble_hci_trans_hs_acl_tx(om);
}

void
ble_transport_ll_init(void)
{

}

void
ble_transport_ll_deinit(void)
{

}

