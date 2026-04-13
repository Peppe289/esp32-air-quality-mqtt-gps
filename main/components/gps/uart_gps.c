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
#include "system_event_code.h"

#define GPS_TXD 21
#define GPS_RXD 20
#define GPS_UART_PORT UART_NUM_1
#define GPS_BAUD_RATE 9600
#define LENGHT_BUFFER 200

typedef struct gps_callbacks_t {
  void (*gps_state_handler)(uint32_t bit, bool add_bit);
  uint32_t (*system_get_state)(void);
} gps_callbacks_t;

static gps_callbacks_t s_gps_callbacks = {NULL, NULL};

static const char *TAG = "GPS";

static const uart_config_t s_uart_config = {
    .baud_rate = GPS_BAUD_RATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

/**
 * Register handler
 */
void gps_register_system_handler(void (*gps_state_handler)(uint32_t bit,
                                                             bool add_bit),
                                  uint32_t (*system_get_state)(void)) {
  s_gps_callbacks.gps_state_handler = gps_state_handler;
  s_gps_callbacks.system_get_state = system_get_state;
}

static inline void set_gps_status(uint32_t bit, bool add_bit) {
  if (s_gps_callbacks.gps_state_handler) {
    s_gps_callbacks.gps_state_handler(bit, add_bit);
  }
}

/**
 * @brief Initializes the GPS UART interface.
 * * Configures the UART driver with the defined baud rate, pins, and buffer
 * size. It uses UART_NUM_1 by default for GPS communication.
 */
void gps_init_uart(void) {
  uart_driver_install(GPS_UART_PORT, LENGHT_BUFFER * 2, 0, 0, NULL, 0);
  uart_param_config(GPS_UART_PORT, &s_uart_config);
  uart_set_pin(GPS_UART_PORT, GPS_TXD, GPS_RXD, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);

  set_gps_status(UART_GPS_SYS_STATUS_INITIALIZED, true);
}

/**
 * @brief Extracts and maps NMEA data to a custom simplified structure.
 * * This function processes raw NMEA objects (e.g., GPGGA), extracts relevant
 * fields like satellite count, position (lat/long), and time, and stores them
 * into the custom nmea_uart_data_s container.
 * * @param p_nmea_obj Pointer to the raw NMEA structure from the library.
 * @param p_gps_data Pointer to the custom destination structure.
 * * @note Frees the NMEA library data object before returning.
 */
static void gps_map_to_struct(nmea_s *p_nmea_obj, gps_data_t *p_gps_data) {
  if (NULL != p_nmea_obj) {
    if (0 < p_nmea_obj->errors) {
      ESP_LOGW(TAG, "Invalid String");
      return;
    }

    ESP_LOGD(TAG, "DATA Type: %d", p_nmea_obj->type);

    if (NMEA_GPGGA == p_nmea_obj->type) {
      p_gps_data->n_satellites = ((nmea_gpgga_s *)p_nmea_obj)->n_satellites;
      ESP_LOGI(TAG, "Satellites: %d",
               ((nmea_gpgga_s *)p_nmea_obj)->n_satellites);

      /**
       * Position fields (latitude/longitude) are contiguous in memory.
       * Perform a single block copy for both structures to optimize
       * performance.
       */
      memcpy(&p_gps_data->position, &((nmea_gpgga_s *)p_nmea_obj)->longitude,
             sizeof(nmea_position) * 2);

      ESP_LOGI(TAG, "Latitude: %d° %f' %c",
               p_gps_data->position.latitude.degrees,
               p_gps_data->position.latitude.minutes,
               (char)p_gps_data->position.latitude.cardinal);

      ESP_LOGI(TAG, "Longitude: %d° %f' %c",
               p_gps_data->position.longitude.degrees,
               p_gps_data->position.longitude.minutes,
               (char)p_gps_data->position.longitude.cardinal);

      p_gps_data->time = ((nmea_gpgga_s *)p_nmea_obj)->time;
      char buf[100];
      if (strftime(buf, sizeof(buf), "%H:%M:%S",
                   (const struct tm *)&(((nmea_gpgga_s *)p_nmea_obj)->time))) {
        ESP_LOGI(TAG, "Time: %s", buf);
      }
    }

    nmea_free(p_nmea_obj);
  }
}

#define endline(x) unlikely((x) == '\0' || (x) == '\n')

/**
 * @brief Parses raw UART strings into NMEA sentences.
 * * Synchronizes with the '$' start delimiter, splits the incoming buffer into
 * individual NMEA strings, and passes them to the NMEA library parser.
 * * @param p_raw_stream Raw string buffer from UART.
 * @param p_gps_data Pointer to the destination data structure.
 * @param buffer_size Length of the raw string buffer.
 */
static void gps_decode_packet(const char *p_raw_stream, gps_data_t *p_gps_data,
                              int buffer_size) {
  char *p_cursor = (char *)p_raw_stream;
  char sentence_buffer[100] = {0};
  size_t char_idx = 0;

  // Sync
  while (*p_cursor != '$' && *p_cursor != '\0' && (buffer_size-- > 0))
    p_cursor++;

  if (endline(*p_cursor))
    return; // Empty message

  while (buffer_size > 0) {
    char_idx = 0;
    memset(sentence_buffer, 0, sizeof(sentence_buffer));
    for (;;) {
      if (endline(*p_cursor)) {
        p_cursor++;
        buffer_size--;
        sentence_buffer[char_idx] = '\n';
        break;
      }
      sentence_buffer[char_idx++] = *p_cursor;
      p_cursor++;
      buffer_size--;
    }

    // If data lenght is too long or not start with '$', skip to next line
    if (unlikely(char_idx >= sizeof(sentence_buffer) - 1 ||
                 sentence_buffer[0] != '$'))
      continue;

    ESP_LOGD(TAG, "Get String:\n%s", sentence_buffer);

    gps_map_to_struct(nmea_parse(sentence_buffer, strlen(sentence_buffer), 0),
                      p_gps_data);
  }
}

/**
 * @brief Validates the parsed GPS data.
 * * Checks if the satellite count is greater than zero and if the
 * latitude/longitude coordinates contain non-zero values.
 * * @param data Pointer to the custom GPS data structure.
 * @return uint8_t 1 if data is valid, 0 otherwise.
 */
static inline uint8_t is_gps_data_valid(const gps_data_t *data) {
  if (data->n_satellites == 0)
    return 0;

  return (data->position.latitude.degrees != 0 ||
          data->position.longitude.degrees != 0);
}

/**
 * @brief Task-level function to perform a synchronous GPS data fetch.
 * * Reads bytes from the UART port, attempts to parse valid NMEA sentences,
 * and populates the provided structure. It includes a 5-second timeout
 * mechanism if no valid fix is obtained.
 * * @param p_gps_data Pointer to the structure where results will be
 * stored.
 * @return uint8_t
 * - 0: Success (Valid data obtained)
 * - 1: Failure (Timeout or invalid memory)
 */
uint8_t gps_read_uart(gps_data_t *p_gps_data) {
  uint8_t raw_buffer[LENGHT_BUFFER];
  time_t start_time = time(NULL);

  p_gps_data->valid = 0;
  if (p_gps_data == NULL) {
    ESP_LOGE(TAG, "Memory allocation failed");
    return 1;
  }
  memset(p_gps_data, 0, sizeof(gps_data_t));
  for (;;) {
    int read_len = uart_read_bytes(GPS_UART_PORT, raw_buffer, LENGHT_BUFFER - 1,
                                   20 / portTICK_PERIOD_MS);
    if (read_len > 0) {
      raw_buffer[read_len] = '\0';
      gps_decode_packet((char *)raw_buffer, p_gps_data, read_len);

      if (!is_gps_data_valid(p_gps_data)) {
        ESP_LOGI(TAG, "Data Not Valid... Try Again");
        if (difftime(time(NULL), start_time) > 5) {
          ESP_LOGE(TAG, "GPS Time out!");
          set_gps_status(UART_GPS_SYS_FIXED, false);
          break;
        }
      } else {
        set_gps_status(UART_GPS_SYS_FIXED, true);
        p_gps_data->valid = 1;
        return 0;
      }
    }
  }

  return 1;
}