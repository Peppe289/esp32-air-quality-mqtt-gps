#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>

#define GPS_TXD (GPIO_NUM_21)
#define GPS_RXD (GPIO_NUM_20)
#define GPS_UART_PORT UART_NUM_1
#define GPS_BAUD_RATE 9600

const uart_config_t uart_config = {
    .baud_rate = GPS_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

void init_gps_uart(void) {
  uart_driver_install(GPS_UART_PORT, 256 * 2, 0, 0, NULL, 0);
  uart_param_config(GPS_UART_PORT, &uart_config);
  uart_set_pin(GPS_UART_PORT, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
}

void gps_read_task() {
  uint8_t *data = (uint8_t *)malloc(256);
  int len = uart_read_bytes(GPS_UART_PORT, data, 255, 20 / portTICK_PERIOD_MS);
  if (len > 0) {
    data[len] = '\0';
    printf("Dati GPS ricevuti: %s\n", (char *)data);
  }
  free(data);
}