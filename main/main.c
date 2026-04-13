#include "cJSON.h"
#include "components/god_header.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "include/storage.h"
#include "network/mqtt_client.h"
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
 * @brief Callback function for MQTT data recovery.
 * * This function is passed to the storage recovery engine to handle the
 * re-transmission of stored JSON strings. It verifies the current system
 * network status (Wi-Fi, MQTT connection, and Subscription) before attempting
 * to publish.
 * * @param[in] p_str   Pointer to the JSON string retrieved from storage.
 * @param[in] p_topic Pointer to the MQTT topic (unused here as it defaults to
 * MQTT_RECOVERY_TOPIC).
 * * @return uint8_t
 * - 0: Success. Data was published and the recovery process can continue.
 * - 1: Failure. Network is unavailable; signals the recovery engine to abort
 * and save the current file offset.
 * * @note Includes a 1-second delay after successful publishing to prevent
 * network congestion during high-volume recovery bursts.
 */
uint8_t custom_mqtt_recovery_callback(const char *p_str, const char *p_topic) {
  uint32_t system_status = system_event_mask_get();

  if ((system_status & WIFI_SYS_STATUS_CONNECTED) &&
      (system_status & MQTT_SYS_STATUS_CONNECTED) &&
      (system_status & MQTT_SYS_STATUS_SUBSCRIBED))
    mqtt_publish_data_client(p_str, MQTT_RECOVERY_TOPIC);
  else
    return 1;

  vTaskDelay(pdMS_TO_TICKS(1000));
  return 0;
}

/**
 * @brief FreeRTOS task responsible for monitoring and recovering offline data.
 * * This task runs in a continuous loop, checking for the existence of offline
 * storage files. If a file is detected AND the system has a stable
 * internet/MQTT connection, it triggers the storage recovery routine.
 * * @param[in] args Task parameters (not used).
 * * @details The task yields the CPU for 3000ms at the end of each check to
 * prevent Task Watchdog Timer (TWDT) triggers and allow lower-priority tasks
 * (like IDLE) to run, ensuring system stability even when no recovery is
 * needed.
 * *
 */
void fs_recovery_json_task(void *args) {
  uint32_t system_status;

  for (;;) {
    system_status = system_event_mask_get();
    if (storage_have_file() && (system_status & WIFI_SYS_STATUS_CONNECTED) &&
        (system_status & MQTT_SYS_STATUS_CONNECTED) &&
        (system_status & MQTT_SYS_STATUS_SUBSCRIBED)) {
      storage_recovery_data(custom_mqtt_recovery_callback);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

/**
 * * @param gps_success is a boolean indicate if GPS fail or success
 */
void gps_led_callback(uint8_t gps_success) {
  if (gps_success)
    led_set_green();
  else
    led_set_red();
}

void app_main(void) {
  hm3301_data_t hm3301 = {0};
  gps_data_t nmea_gps = {0};
  struct json_obj json;

  led_init();
  led_blink_all();

  system_event_mask_init();
  storage_init_fs();
  hm3301_init_i2c();
  gps_init_uart();
  wifi_init_sta();
  xTaskCreate(fs_recovery_json_task, "fs_listener", 4096, NULL, 5, NULL);

  vTaskDelay(pdMS_TO_TICKS(1000));
  led_set_red();

  gps_set_handler(gps_led_callback);

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    uint32_t bitmask = system_event_mask_get();
    char bitmask_str[256];

    system_event_mask_code_str(bitmask, bitmask_str, sizeof(bitmask_str));
    ESP_LOGI(TAG, "Current system status:\n%s", bitmask_str);

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

      if ((bitmask & WIFI_SYS_STATUS_CONNECTED) &&
          (bitmask & MQTT_SYS_STATUS_CONNECTED) &&
          (bitmask & MQTT_SYS_STATUS_SUBSCRIBED))
        mqtt_publish_data_client((const char *)json.unformatted, "/topic/qos0");
      else
        storage_write_on_file((const char *)json.unformatted);

      free(json.unformatted);
      free(json.std);
      memset(&json, 0, sizeof(char *) * 2);
    }
  }
}
