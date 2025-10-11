#pragma once
#include <stddef.h>
#include <stdint.h>
typedef void (*uart_rx_cb_t)(const uint8_t *data, size_t len);
void uartm_init(int tx, int rx, int baud);
int  uartm_write(const uint8_t *data, size_t len);
void uartm_set_rx_callback(uart_rx_cb_t cb);