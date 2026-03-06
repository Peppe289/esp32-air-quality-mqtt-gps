/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <string.h>

#include "nvs_flash.h"

#include "wifi_conn.h"

#include "../mqtt/network/mqtt_client.h"
#include "system_event_code.h"

#define ESP_MAXIMUM_RETRY 2

static const char *TAG = "WIFI";

typedef struct wifi_callbacks_t {
  void (*wifi_state_handler)(uint32_t bit, bool add_bit);
  uint32_t (*system_get_state)(void);
} wifi_callbacks_t;

wifi_callbacks_t s_wifi_event_group = {NULL, NULL};
static esp_netif_t *netif = NULL;

static esp_event_handler_instance_t instance_any_id;
static esp_event_handler_instance_t instance_got_ip;

static inline void set_wifi_status(uint32_t bit, bool add_bit) {
  if (s_wifi_event_group.wifi_state_handler) {
    s_wifi_event_group.wifi_state_handler(bit, add_bit);
  }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    set_wifi_status(WIFI_SYS_STATUS_CONNECTING, true);
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
    set_wifi_status(WIFI_SYS_STATUS_CONNECTING, false);
    set_wifi_status(WIFI_SYS_STATUS_CONNECTED, false);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    // remove connecting status and add connected status.
    set_wifi_status(WIFI_SYS_STATUS_CONNECTING, false);
    set_wifi_status(WIFI_SYS_STATUS_CONNECTED, true);
    mqtt_start_client();
  }
}

void wifi_init_sta() {
  esp_err_t ret;

  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  if (netif == NULL) {
    netif = esp_netif_create_default_wifi_sta();
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
  };

  strncpy((char *)wifi_config.sta.ssid, WIFI_SSID,
          sizeof(wifi_config.sta.ssid) - 1);
  wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
  strncpy((char *)wifi_config.sta.password, WIFI_PASSWD,
          sizeof(wifi_config.sta.password) - 1);
  wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';

  ESP_LOGI(TAG, "Try to connect %s WIFI", wifi_config.sta.ssid);

  set_wifi_status(WIFI_SYS_STATUS_ENABLED, true);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_register_system_handler(void (*wifi_state_handler)(uint32_t bit,
                                                             bool add_bit),
                                  uint32_t (*system_get_state)(void)) {
  s_wifi_event_group.wifi_state_handler = wifi_state_handler;
  s_wifi_event_group.system_get_state = system_get_state;
}