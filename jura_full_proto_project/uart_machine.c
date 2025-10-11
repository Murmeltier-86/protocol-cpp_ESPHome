#include "uart_machine.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define UARTP UART_NUM_1
static const char *TAG="UARTM";
static uart_rx_cb_t s_cb;

static void rx_task(void *_) {
    uint8_t buf[512];
    for (;;) {
        int n = uart_read_bytes(UARTP, buf, sizeof buf, pdMS_TO_TICKS(50));
        if (n>0 && s_cb) s_cb(buf, (size_t)n);
    }
}

void uartm_init(int tx, int rx, int baud) {
    uart_config_t c = {
        .baud_rate = baud, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UARTP, 2048, 0, 0, NULL, 0);
    uart_param_config(UARTP, &c);
    uart_set_pin(UARTP, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(rx_task, "uart_rx", 3072, NULL, 10, NULL);
    ESP_LOGI(TAG, "init tx=%d rx=%d baud=%d", tx, rx, baud);
}
int uartm_write(const uint8_t *d, size_t n){ return uart_write_bytes(UARTP, (const char*)d, n); }
void uartm_set_rx_callback(uart_rx_cb_t cb){ s_cb = cb; }