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

static void IRAM_ATTR hci_uart_rx_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    while(hci_uart.uart_opened) {
        //Waiting for UART event.
        if(xQueueReceive(hci_uart.evt_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGD(TAG, "uart[%d] event:", hci_uart.port);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(hci_uart.port, dtmp, event.size, portMAX_DELAY);
                    for (int i = 0 ; i < event.size; i++) {
                        hci_uart.rx_char(hci_uart.u_func_arg, dtmp[i]);
                    }
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(hci_uart.port);
                    xQueueReset(hci_uart.evt_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(hci_uart.port);
                    xQueueReset(hci_uart.evt_queue);
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
    free(dtmp);
    dtmp = NULL;
    hci_uart.rx_task_handler = NULL;
    vTaskDelete(NULL);
}

int hal_uart_config(int uart, int32_t speed, uint8_t data_bits, uint8_t stop_bits,
  enum hal_uart_parity parity, enum hal_uart_flow_ctl flow_ctl)
{
    uart_config_t uart_cfg = {
        .baud_rate = speed,
        .data_bits = data_bits,
        .parity    = parity,
        .stop_bits = stop_bits,
        .flow_ctrl = flow_ctl,
        .source_clk = UART_SCLK_DEFAULT,
    };
    hci_uart.port = uart;
    hci_uart.cfg = uart_cfg;

    int intr_alloc_flags = 0;
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(uart, BUF_SIZE * 2, BUF_SIZE * 2, 20, &hci_uart.evt_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(uart, &hci_uart.cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart, CONFIG_BT_NIMBLE_UART_TX_PIN, CONFIG_BT_NIMBLE_UART_RX_PIN, -1, -1));

    hci_uart.uart_opened=true;
    ESP_LOGI(TAG, "set uart pin tx:%d, rx:%d.\n", CONFIG_BT_NIMBLE_UART_TX_PIN, CONFIG_BT_NIMBLE_UART_RX_PIN);
    ESP_LOGI(TAG, "set baud_rate:%d.\n", speed);

    //Create a task to handler UART event from ISR
    xTaskCreate(hci_uart_rx_task, "hci_uart_rx_task", 2048, NULL, 12, &hci_uart.rx_task_handler);
    return 0;
}

void IRAM_ATTR hal_uart_start_tx(int uart_no)
{
    int data;
    uint8_t u8_data=0;
    while(1){
        data = hci_uart.tx_char(hci_uart.u_func_arg);
        if (data >= 0) {
            u8_data = data;
            uart_tx_chars(uart_no,(char*)&u8_data,1);
        }else{
            break;
        }
    }
    if(hci_uart.tx_done) {
        hci_uart.tx_done(hci_uart.u_func_arg);
    }
}

int hal_uart_close(int uart_no)
{
    hci_uart.uart_opened=false;
    // Stop uart rx task
    if(hci_uart.rx_task_handler != NULL) {
        ESP_LOGW(TAG,"Waiting for uart task finish...");
    }
    while (hci_uart.rx_task_handler != NULL);

    uart_driver_delete(uart_no);
    ESP_LOGI(TAG,"hci uart close success.");
    return 0;
}
