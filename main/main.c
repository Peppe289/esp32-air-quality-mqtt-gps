#include "cJSON.h"
#include "components/god_header.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nmea.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "spi_flash_mmap.h"
#include "system_event_code.h"
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <unistd.h>

static const char *TAG = "MAIN";

/* LED configuration */
#define LED_GREEN_GPIO 3
#define LED_RED_GPIO 4

struct json_obj {
  char *unformatted;
  char *std;
} json_obj;

/**
 * @brief Converts sensor data into a minified JSON string.
 * * Uses cJSON to create a structured object containing GPS coordinates,
 * time, and PM concentration values.
 * * @param p_gps_data Pointer to the validated NMEA data structure.
 * @param p_hm3301_data   Pointer to the parsed PM sensor data.
 * @return struct json_obj Contains pointer to the JSON string
 * formatted/unformatted. Note: Caller is responsible for calling free().
 */
static struct json_obj
serialize_data_to_json_string(gps_data_t *p_gps_data,
                              hm3301_data_t *p_hm3301_data) {
  cJSON *root;
  cJSON *pm, *position, *longitude, *latitude, *time;
  struct json_obj json;

  if (p_gps_data || p_hm3301_data)
    root = cJSON_CreateObject();
  else
    return (struct json_obj){NULL, NULL};

  if (!root)
    return (struct json_obj){NULL, NULL};

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

  json.unformatted = cJSON_PrintUnformatted(root);
  json.std = cJSON_Print(root);
  cJSON_Delete(root);
  return json;
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

/**
 * @brief State machine task that manages the WiFi and Bluetooth lifecycle.
 * * Since ESP32-C3 has a single radio, this task alternates between BLE and
 * WiFi:
 * 1. Monitors the WiFi status; if connected or connecting, it stays idle.
 * 2. If new credentials are received via BLE (or set via macros), it shuts down
 * BLE and attempts a WiFi connection.
 * 3. If WiFi is disconnected and no credentials are pending, it ensures BLE
 * is active for new provisioning.
 * * @note Future updates will include loading/saving credentials from the
 * "storage" partition.
 * @param args Unused task parameters.
 */
void manageConnection(void *args) {

  char *ssid = ble_getSSID();
  char *password = ble_getPassword();

#ifdef WIFI_SSID
  memcpy(ssid, WIFI_SSID, sizeof(WIFI_SSID));
#endif

#ifdef WIFI_PASSWD
  memcpy(password, WIFI_PASSWD, sizeof(WIFI_PASSWD));
#endif

  for (;;) {
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // If the wifi is connected or not yet all is fine.
    if (system_event_mask_get() &
        (WIFI_SYS_STATUS_CONNECTING | WIFI_SYS_STATUS_CONNECTED))
      continue;

    // If the ssid and password have data, try WIFI connection.
    // Then clear up for the next connection.
    if (ssid[0] != '\0' && password[0] != '\0') {
      disable_bt();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      wifi_init_sta(ssid, password);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      memset(ssid, 0, AUTH_MAX_LENGTH);
      memset(password, 0, AUTH_MAX_LENGTH);
      continue;
    }

    // If the wifi isn't connected enable or keep alive the BT.
    if (!isBTEnabled()) {
      disableWIFI();
      start_bt();
    }
  }
}

/**
 * @brief Entry point for the connectivity management system.
 * * Initializes the Non-Volatile Storage (NVS) required for both WiFi and
 * Bluetooth stacks, then spawns the 'connManager' FreeRTOS task.
 * * @note NVS is erased and re-initialized if no free pages are found or a
 * version mismatch occurs.
 */
void connection_listener_start(void) {
  esp_err_t ret;

  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  xTaskCreate(manageConnection, "connManager", 4096, NULL, 5, NULL);
}

void app_main(void) {
  hm3301_data_t hm3301 = {0};
  gps_data_t nmea_gps = {0};
  struct json_obj json;

  system_event_mask_init();
  storage_init_fs();
  led_init();
  hm3301_init_i2c();
  gps_init_uart();
  connection_listener_start();

  vTaskDelay(pdMS_TO_TICKS(1000));

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    uint32_t bitmask = system_event_mask_get();

    if (bitmask & WIFI_SYS_STATUS_CONNECTED) {
      ESP_LOGI(TAG, "\nWifi connected\n");
    }
    if (bitmask & WIFI_SYS_STATUS_CONNECTING) {
      ESP_LOGI(TAG, "\nWifi connecting\n");
    }
    if (bitmask & WIFI_SYS_STATUS_ENABLED) {
      ESP_LOGI(TAG, "\nWifi enabled\n");
    }
    if (bitmask & MQTT_SYS_STATUS_CONNECTED) {
      ESP_LOGI(TAG, "\nMQTT connected\n");
    }
    if (bitmask & MQTT_SYS_STATUS_SUBSCRIBED) {
      ESP_LOGI(TAG, "\nMQTT subscribed\n");
    }
    if (bitmask & STORAGE_SYS_STATUS_FAILED) {
      ESP_LOGI(TAG, "\nStorage some error\n");
    }
    if (bitmask & STORAGE_SYS_STATUS_INIZIALIZED) {
      ESP_LOGI(TAG, "\nStorage inizialized\n");
    }

    if (hm3301_read_i2c(NULL, &hm3301)) {
      ESP_LOGE(TAG, "Error reading HM3301. Continue...\n");
      continue;
    }

    if (gps_read_uart(&nmea_gps)) {
      ESP_LOGE(TAG, "Error reading GPS. Continue...");
      continue;
    }

    json = serialize_data_to_json_string(&nmea_gps, &hm3301);

    if (json.std) {
      char buff[512] = {0};
      int index = 0;

      for (char *ptr = json.std;; ptr++) {
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

      mqtt_publish_data_client((const char *)json.unformatted);
      free(json.unformatted);
      free(json.std);
      memset(&json, 0, sizeof(char *) * 2);
    }
  }
}
