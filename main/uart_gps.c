#include "driver/uart.h"
#include "esp_compiler.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "gpgga.h"
#include "gpgll.h"
#include "gpgsa.h"
#include "gpgsv.h"
#include "gprmc.h"
#include "gptxt.h"
#include "gpvtg.h"
#include "nmea.h"

#include "uart_gps.h"

#define GPS_TXD 21
#define GPS_RXD 20
#define GPS_UART_PORT UART_NUM_1
#define GPS_BAUD_RATE 9600
#define LENGHT_BUFFER 200

static const char *TAG = "GPS";

static const uart_config_t uart_config = {
    .baud_rate = GPS_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

void init_gps_uart(void) {
  uart_driver_install(GPS_UART_PORT, LENGHT_BUFFER * 2, 0, 0, NULL, 0);
  uart_param_config(GPS_UART_PORT, &uart_config);
  uart_set_pin(GPS_UART_PORT, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
}

static void gps_data(nmea_s *data, nmea_uart_data_s *nmea_uart_data) {
  if (NULL != data) {
    if (0 < data->errors) {
      ESP_LOGW(TAG, "Invalid String");
      return;
    }

    ESP_LOGD(TAG, "DATA Type: %d", data->type);

    if (NMEA_GPGGA == data->type) {
      nmea_uart_data->n_satellites = ((nmea_gpgga_s *)data)->n_satellites;
      ESP_LOGI(TAG, "Satellites: %d", ((nmea_gpgga_s *)data)->n_satellites);

      memcpy(&nmea_uart_data->position, &((nmea_gpgga_s *)data)->longitude,
             sizeof(nmea_position) * 2);

      ESP_LOGI(TAG, "Latitude: %d° %f' %c",
               nmea_uart_data->position.latitude.degrees,
               nmea_uart_data->position.latitude.minutes,
               (char)nmea_uart_data->position.latitude.cardinal);

      ESP_LOGI(TAG, "Longitude: %d° %f' %c",
               nmea_uart_data->position.longitude.degrees,
               nmea_uart_data->position.longitude.minutes,
               (char)nmea_uart_data->position.longitude.cardinal);

      nmea_uart_data->time = ((nmea_gpgga_s *)data)->time;
      char buf[100];
      if (strftime(buf, sizeof(buf), "%H:%M:%S",
                   (const struct tm *)&(((nmea_gpgga_s *)data)->time))) {
        ESP_LOGI(TAG, "Time: %s", buf);
      }
    }

    nmea_free(data);
  }
}

#define endline(x) unlikely((x) == '\0' || (x) == '\n')

static void gps_parse(const char *str, nmea_uart_data_s *nmea_uart_data,
                      int len) {
  char *start = (char *)str;
  char cpy[100] = {0};
  size_t index = 0;

  // Sync
  while (*start != '$' && *start != '\0' && (len-- > 0))
    start++;

  if (endline(*start))
    return; // Empty message

  while (len > 0) {
    index = 0;
    memset(cpy, 0, sizeof(cpy));
    for (;;) {
      if (endline(*start)) {
        start++;
        len--;
        cpy[index] = '\n';
        break;
      }
      cpy[index++] = *start;
      start++;
      len--;
    }

    // If data lenght is too long or not start with '$', skip to next line
    if (unlikely(index >= sizeof(cpy) - 1 || cpy[0] != '$'))
      continue;

    ESP_LOGD(TAG, "Get String:\n%s", cpy);

    gps_data(nmea_parse(cpy, strlen(cpy), 0), nmea_uart_data);
  }
}

static inline uint8_t is_gps_data_valid(const nmea_uart_data_s *data) {
  if (data->n_satellites == 0)
    return 0;

  return (data->position.latitude.degrees != 0 ||
          data->position.longitude.degrees != 0);
}

uint8_t gps_read_task(nmea_uart_data_s *nmea_uart_data) {
  uint8_t data[LENGHT_BUFFER];
  time_t start = time(NULL);

  if (nmea_uart_data == NULL) {
    ESP_LOGE(TAG, "Memory allocation failed");
    return 1;
  }
  memset(nmea_uart_data, 0, sizeof(nmea_uart_data_s));
  for (;;) {
    int len = uart_read_bytes(GPS_UART_PORT, data, LENGHT_BUFFER - 1,
                              20 / portTICK_PERIOD_MS);
    if (len > 0) {
      data[len] = '\0';
      gps_parse((char *)data, nmea_uart_data, len);

      if (!is_gps_data_valid(nmea_uart_data)) {
        ESP_LOGI(TAG, "Data Not Valid... Try Again");
        if (difftime(time(NULL), start) > 5) {
          ESP_LOGE(TAG, "GPS Time out!");
          break;
        }
      } else {
        return 0;
      }
    }
  }

  return 1;
}