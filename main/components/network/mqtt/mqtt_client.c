#include "mqtt_client.h"
#include "esp_event_base.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "network/mqtt_client.h"
#include "system_event_code.h"

#ifndef MQTT_BROKER
#warning "Missing MQTT_BROKER env variable. Using default."
#define MQTT_BROKER "0.0.0.0"
#endif

#ifndef MQTT_PORT
#warning "Missing MQTT_BROKER env variable. Using default."
#define MQTT_PORT 8883
#endif

typedef struct mqtt_callbacks_t {
  void (*mqtt_state_handler)(uint32_t bit, bool add_bit);
  uint32_t (*system_get_state)(void);
} mqtt_callbacks_t;

mqtt_callbacks_t s_mqtt_event_group = {NULL, NULL};
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static const esp_mqtt_client_config_t s_mqtt_cfg = {
    .broker =
        {
            .address =
                {
                    .transport = MQTT_TRANSPORT_OVER_WS,
                    .hostname = MQTT_BROKER,
                    .port = MQTT_PORT,
                },
            .verification =
                {
                    .certificate = NULL,
                    .certificate_len = 0,
                    .psk_hint_key = NULL,
                    .crt_bundle_attach = NULL,
                    .use_global_ca_store = false,
                    .skip_cert_common_name_check = true,
                    .common_name = NULL,
                },
        },
    .credentials =
        {
#ifdef MQTT_USERNAME
            .username = MQTT_USERNAME,
#endif
            .authentication =
                {
#ifdef MQTT_PASSWORD
                    .password = MQTT_PASSWORD,
#endif
                    .certificate = NULL,
                    .certificate_len = 0,
                },
        },
};

static inline void set_mqtt_status(uint32_t bit, bool add_bit) {
  if (s_mqtt_event_group.mqtt_state_handler) {
    s_mqtt_event_group.mqtt_state_handler(bit, add_bit);
  }
}

void mqtt_register_system_handler(void (*mqtt_state_handler)(uint32_t bit,
                                                             bool add_bit),
                                  uint32_t (*system_get_state)(void)) {
  s_mqtt_event_group.mqtt_state_handler = mqtt_state_handler;
  s_mqtt_event_group.system_get_state = system_get_state;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = event_data;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    set_mqtt_status(MQTT_SYS_STATUS_CONNECTED, true);
    esp_mqtt_client_subscribe(event->client, MQTT_MAIN_TOPIC, 0);
    esp_mqtt_client_subscribe(event->client, MQTT_RECOVERY_TOPIC, 0);
    break;
  case MQTT_EVENT_DISCONNECTED:
    set_mqtt_status(MQTT_SYS_STATUS_CONNECTED, false);
    break;
  case MQTT_EVENT_SUBSCRIBED:
    set_mqtt_status(MQTT_SYS_STATUS_SUBSCRIBED, true);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    set_mqtt_status(MQTT_SYS_STATUS_SUBSCRIBED, false);
    break;
  case MQTT_EVENT_PUBLISHED:
    break;
  case MQTT_EVENT_DATA:
    break;
  case MQTT_EVENT_ERROR:
    break;
  default:
    break;
  }
}

void mqtt_publish_data_client(const char *p_str, const char *p_topic) {
  // Skip push if:
  // - not able to get system status.
  // - wifi isn't connected.
  if (s_mqtt_client && s_mqtt_event_group.system_get_state &&
      (s_mqtt_event_group.system_get_state() & WIFI_SYS_STATUS_CONNECTED))
    esp_mqtt_client_publish(s_mqtt_client, p_topic, p_str, 0, 0, 0);
}

void mqtt_start_client(void) {
  s_mqtt_client = esp_mqtt_client_init(&s_mqtt_cfg);
  esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler, NULL);
  esp_mqtt_client_start(s_mqtt_client);
}