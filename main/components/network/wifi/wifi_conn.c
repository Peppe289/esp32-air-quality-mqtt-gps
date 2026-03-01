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

#include "ble_gatts.h"
#include "wifi_conn.h"

#include "../mqtt/network/mqtt_client.h"

#define ESP_MAXIMUM_RETRY 2

typedef struct wifi_callbacks_t {
  void (*wifi_state_handler)(uint32_t bit, bool add_bit);
  uint32_t (*system_get_state)(void);
} wifi_callbacks_t;

wifi_callbacks_t s_wifi_event_group = {NULL, NULL};
static int s_retry_num = 0;
static uint8_t s_wifi_status = WIFI_STATE_IDLE;
static esp_netif_t *netif = NULL;

static esp_event_handler_instance_t instance_any_id;
static esp_event_handler_instance_t instance_got_ip;

static inline void set_wifi_status(uint8_t status) { s_wifi_status = status; }

uint8_t get_wifi_status(void) { return s_wifi_status; }

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    set_wifi_status(WIFI_STATE_CONNECTING);
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
    } else {
      set_wifi_status(WIFI_STATE_IDLE);
      s_retry_num = 0;
    }
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    s_retry_num = 0;
    set_wifi_status(WIFI_STATE_CONNECTED);
    mqtt_start_client();
  }
}

void disableWIFI() {
  set_wifi_status(WIFI_STATE_IDLE);

  if (instance_any_id) {
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                          instance_any_id);
    instance_any_id = NULL;
  }

  if (instance_got_ip) {
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                          instance_got_ip);
    instance_got_ip = NULL;
  }

  esp_wifi_stop();
  esp_wifi_deinit();

  if (netif) {
    esp_netif_destroy_default_wifi(netif);
    netif = NULL;
  }
}

static uint8_t s_logic_initialized = 0;

void wifi_init_sta(char *_ssid, char *passwd) {

  set_wifi_status(WIFI_STATE_CONNECTING);

  if (!s_logic_initialized) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_logic_initialized = true;
  }

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

  strncpy((char *)wifi_config.sta.ssid, _ssid,
          sizeof(wifi_config.sta.ssid) - 1);
  wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
  strncpy((char *)wifi_config.sta.password, passwd,
          sizeof(wifi_config.sta.password) - 1);
  wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';

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