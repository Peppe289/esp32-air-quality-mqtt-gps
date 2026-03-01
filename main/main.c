#include "connection/conn_thread.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_hm3301.h"
#include "nmea.h"
#include "spi_flash_mmap.h"
#include "uart_gps.h"
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <unistd.h>

#include "esp_log.h"

static const char *TAG = "MAIN";

#include "cJSON.h"
#include "connection/mqtt/conn_mqtt_client.h"

/* LED configuration */
#define LED_GREEN_GPIO 3
#define LED_RED_GPIO 4

/**
 * @brief Converts sensor data into a minified JSON string.
 * * Uses cJSON to create a structured object containing GPS coordinates,
 * time, and PM concentration values.
 * * @param p_gps_data Pointer to the validated NMEA data structure.
 * @param p_hm3301_data   Pointer to the parsed PM sensor data.
 * @return char* Pointer to the JSON string. Note: Caller is responsible for
 * calling free().
 */
static char *serialize_data_to_json_string(gps_data_t *p_gps_data,
                                           hm3301_data_t *p_hm3301_data) {
  cJSON *root;
  cJSON *pm, *position, *longitude, *latitude, *time;
  char *string;

  if (p_gps_data || p_hm3301_data)
    root = cJSON_CreateObject();
  else
    return NULL;

  if (p_gps_data) {
    cJSON_AddNumberToObject(root, "satellites", p_gps_data->n_satellites);

    position = cJSON_AddObjectToObject(root, "position");
    longitude = cJSON_AddObjectToObject(position, "longitude");
    latitude = cJSON_AddObjectToObject(position, "latitude");
    cJSON_AddNumberToObject(longitude, "degrees",
                            p_gps_data->position.longitude.degrees);
    cJSON_AddNumberToObject(longitude, "minutes",
                            p_gps_data->position.longitude.minutes);
    cJSON_AddStringToObject(
        longitude, "cardinal",
        (char[]){p_gps_data->position.longitude.cardinal, '\0'});
    cJSON_AddNumberToObject(latitude, "degrees",
                            p_gps_data->position.latitude.degrees);
    cJSON_AddNumberToObject(latitude, "minutes",
                            p_gps_data->position.latitude.minutes);
    cJSON_AddStringToObject(
        latitude, "cardinal",
        (char[]){p_gps_data->position.latitude.cardinal, '\0'});
    time = cJSON_AddObjectToObject(root, "time");
    cJSON_AddNumberToObject(time, "hours", p_gps_data->time.tm_hour);
    cJSON_AddNumberToObject(time, "minutes", p_gps_data->time.tm_min);
    cJSON_AddNumberToObject(time, "seconds", p_gps_data->time.tm_sec);
  }

  if (p_hm3301_data) {
    pm = cJSON_AddObjectToObject(root, "hm3301");
    cJSON_AddNumberToObject(pm, "PM1.0", p_hm3301_data->pm1_0);
    cJSON_AddNumberToObject(pm, "PM2.5", p_hm3301_data->pm2_5);
    cJSON_AddNumberToObject(pm, "PM10", p_hm3301_data->pm10);
  }

  string = cJSON_Print(root);
  cJSON_Delete(root);
  return string;
}

/**
 * @brief Initializes onboard status LEDs.
 * * Sets Red LED as Power/Status indicator and Green LED for
 * Connectivity/Activity.
 */
static void led_init(void) {
  // Initialize Green LED (GPIO2) - OFF initially
  gpio_reset_pin(LED_GREEN_GPIO);
  gpio_set_direction(LED_GREEN_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_GREEN_GPIO, 0); // Turn off green LED at startup

  // Initialize Red LED (GPIO4) - ON at power on
  gpio_reset_pin(LED_RED_GPIO);
  gpio_set_direction(LED_RED_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_RED_GPIO, 1); // Turn on red LED at startup

  ESP_LOGI(TAG, "LEDs initialized: Green=GPIO%d (OFF), Red=GPIO%d (ON)",
           LED_GREEN_GPIO, LED_RED_GPIO);
}

void app_main(void) {
  hm3301_data_t hm3301 = {0};
  gps_data_t nmea_gps = {0};
  char *json_string;

  led_init();
  hm3301_init_i2c();
  gps_init_uart();
  connection_listener_start();

  vTaskDelay(pdMS_TO_TICKS(1000));

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    if (hm3301_read_i2c(NULL, &hm3301)) {
      ESP_LOGE(TAG, "Error reading HM3301. Continue...\n");
      continue;
    }

    if (gps_read_uart(&nmea_gps)) {
      ESP_LOGE(TAG, "Error reading GPS. Continue...");
      continue;
    }

    json_string = serialize_data_to_json_string(&nmea_gps, &hm3301);

    if (json_string) {
      char buff[200] = {0};
      int index = 0;

      for (char *ptr = json_string;; ptr++) {
        if (*ptr != '\n' && *ptr != '\0') {
          buff[index++] = *ptr;
        } else {
          ESP_LOGI(TAG, "%s", buff);

          memset(buff, 0, sizeof(buff));
          index = 0;

          if (*ptr == '\0')
            break;
        }
      }

      mqtt_publish_data_client((const char *)json_string);
      free(json_string);
      json_string = NULL;
    }
  }
}
