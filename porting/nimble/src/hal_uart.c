/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "hal/hal_uart.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_nimble_mem.h"
#include "esp_nimble_cfg.h"

static const char *TAG = "hal_uart";

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

typedef struct{
    bool uart_opened;
    uart_port_t port;
    uart_config_t cfg;
    QueueHandle_t evt_queue;
    TaskHandle_t rx_task_handler;
    hal_uart_tx_char tx_char;
    hal_uart_tx_done tx_done;
    hal_uart_rx_char rx_char;
    hal_uart_rx_sync_lost rx_sync_lost;
    void *rx_sync_lost_arg;
    void *u_func_arg;

}hci_uart_t;

static hci_uart_t hci_uart;

int hal_uart_init_cbs(int uart_no, hal_uart_tx_char tx_func,
                      hal_uart_tx_done tx_done, hal_uart_rx_char rx_func, void *arg)
{
    hci_uart.tx_char=tx_func;
    hci_uart.rx_char=rx_func;
    hci_uart.tx_done=tx_done;
    hci_uart.u_func_arg = arg;
    return 0;
}

void hal_uart_deinit_cbs(void)
{
    hci_uart.tx_char = NULL;
    hci_uart.rx_char = NULL;
    hci_uart.tx_done = NULL;
    hci_uart.rx_sync_lost = NULL;
    hci_uart.rx_sync_lost_arg = NULL;
    hci_uart.u_func_arg = NULL;
}

void hal_uart_set_rx_sync_lost_cb(hal_uart_rx_sync_lost cb, void *arg)
{
    hci_uart.rx_sync_lost = cb;
    hci_uart.rx_sync_lost_arg = arg;
}

#if !MYNEWT_VAL(BLE_LOW_SPEED_MODE)
IRAM_ATTR
#endif
static void hci_uart_rx_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) nimble_platform_mem_calloc(1,RD_BUF_SIZE);
    if (dtmp == NULL) {
        hci_uart.rx_task_handler = NULL;
        vTaskDelete(NULL);
        return;
    }
    while(hci_uart.uart_opened) {
        //Waiting for UART event.
        if(xQueueReceive(hci_uart.evt_queue, (void * )&event, pdMS_TO_TICKS(100))) {
            if (!hci_uart.uart_opened) {
                break;
            }
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGD(TAG, "uart[%d] event:", hci_uart.port);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA: {
                    size_t remaining = event.size;
                    while (remaining > 0 && hci_uart.uart_opened) {
                        size_t chunk = remaining > RD_BUF_SIZE ? RD_BUF_SIZE : remaining;
                        int n = uart_read_bytes(hci_uart.port, dtmp, chunk, pdMS_TO_TICKS(100));
                        if (n <= 0) {
                            if (!hci_uart.uart_opened) {
                                break;
                            }
                            continue;
                        }
                        for (int i = 0; i < n; i++) {
                            hci_uart.rx_char(hci_uart.u_func_arg, dtmp[i]);
                        }
                        remaining -= (size_t)n;
                    }
                    break;
                }
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(hci_uart.port);
                    xQueueReset(hci_uart.evt_queue);
                    if (hci_uart.rx_sync_lost != NULL) {
                        hci_uart.rx_sync_lost(hci_uart.rx_sync_lost_arg);
                    }
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(hci_uart.port);
                    xQueueReset(hci_uart.evt_queue);
                    if (hci_uart.rx_sync_lost != NULL) {
                        hci_uart.rx_sync_lost(hci_uart.rx_sync_lost_arg);
                    }
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    nimble_platform_mem_free(dtmp);
    dtmp = NULL;
    hci_uart.rx_task_handler = NULL;
    vTaskDelete(NULL);
}

int hal_uart_config(int uart, int32_t speed, uint8_t data_bits, uint8_t stop_bits,
  enum hal_uart_parity parity, enum hal_uart_flow_ctl flow_ctl)
{
    uart_parity_t idf_parity;
    uart_hw_flowcontrol_t idf_flow;
    int rts_pin = UART_PIN_NO_CHANGE;
    int cts_pin = UART_PIN_NO_CHANGE;

    switch (parity) {
    case HAL_UART_PARITY_NONE:
        idf_parity = UART_PARITY_DISABLE;
        break;
    case HAL_UART_PARITY_ODD:
        idf_parity = UART_PARITY_ODD;
        break;
    case HAL_UART_PARITY_EVEN:
        idf_parity = UART_PARITY_EVEN;
        break;
    default:
        return -1;
    }

    switch (flow_ctl) {
    case HAL_UART_FLOW_CTL_NONE:
        idf_flow = UART_HW_FLOWCTRL_DISABLE;
        break;
    case HAL_UART_FLOW_CTL_RTS_CTS:
        idf_flow = UART_HW_FLOWCTRL_CTS_RTS;
        rts_pin = CONFIG_BT_NIMBLE_HCI_UART_RTS_PIN;
        cts_pin = CONFIG_BT_NIMBLE_HCI_UART_CTS_PIN;
        break;
    default:
        return -1;
    }

    {
        int fifo_len = UART_HW_FIFO_LEN(uart);
        int rx_thresh = fifo_len - 8;

        if (rx_thresh < 1) {
            rx_thresh = 1;
        }

        uart_config_t uart_cfg = {
            .baud_rate = speed,
            .data_bits = data_bits,
            .parity    = idf_parity,
            .stop_bits = stop_bits,
            .flow_ctrl = idf_flow,
            /*
             * Must stay below UART_HW_FIFO_LEN (LP UART is 16). 0 would
             * throttle the controller after the first byte when RTS/CTS is on.
             */
            .rx_flow_ctrl_thresh = rx_thresh,
            .source_clk = UART_SCLK_DEFAULT,
        };
        hci_uart.port = uart;
        hci_uart.cfg = uart_cfg;
    }

    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(uart, BUF_SIZE * 2, BUF_SIZE * 2, 20, &hci_uart.evt_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(uart, &hci_uart.cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart, CONFIG_BT_NIMBLE_UART_TX_PIN, CONFIG_BT_NIMBLE_UART_RX_PIN, rts_pin, cts_pin));

    hci_uart.uart_opened=true;
    ESP_LOGI(TAG, "set uart pin tx:%d, rx:%d, rts:%d, cts:%d.",
             CONFIG_BT_NIMBLE_UART_TX_PIN, CONFIG_BT_NIMBLE_UART_RX_PIN, rts_pin, cts_pin);
    ESP_LOGI(TAG, "set baud_rate:%d.", speed);

    //Create a task to handler UART event from ISR
    xTaskCreate(hci_uart_rx_task, "hci_uart_rx_task", 2048, NULL, 12, &hci_uart.rx_task_handler);
    return 0;
}

#if !MYNEWT_VAL(BLE_LOW_SPEED_MODE)
IRAM_ATTR
#endif
void hal_uart_start_tx(int uart_no)
{
    int data;
    uint8_t u8_data = 0;

    while (1) {
        data = hci_uart.tx_char(hci_uart.u_func_arg);
        if (data < 0) {
            break;
        }
        u8_data = (uint8_t)data;
        /*
         * TX ring is installed, so uart_tx_chars() can drop the octet when the
         * FIFO is full. uart_write_bytes() copies into that ring and returns
         * only after the octet is queued.
         */
        if (uart_write_bytes(uart_no, (const char *)&u8_data, 1) != 1) {
            ESP_LOGE(TAG, "uart tx failed");
            break;
        }
    }
    if (hci_uart.tx_done) {
        hci_uart.tx_done(hci_uart.u_func_arg);
    }
}

int hal_uart_close(int uart_no)
{
    uart_event_t evt = { 0 };

    hci_uart.uart_opened = false;
    if (hci_uart.rx_task_handler != NULL) {
        ESP_LOGW(TAG, "Waiting for uart task finish...");
    }
    /*
     * The RX task blocks in xQueueReceive. Post a wake event, then yield
     * until it notices uart_opened and clears rx_task_handler. Do not delete
     * the driver while that task is still parked on the queue.
     */
    if (hci_uart.evt_queue != NULL) {
        evt.type = UART_EVENT_MAX;
        (void)xQueueSend(hci_uart.evt_queue, &evt, 0);
    }
    while (hci_uart.rx_task_handler != NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    uart_driver_delete(uart_no);
    hci_uart.evt_queue = NULL;
    ESP_LOGI(TAG, "hci uart close success.");
    return 0;
}
