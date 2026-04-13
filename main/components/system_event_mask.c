#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "god_header.h"
#include "i2c_hm3301.h"
#include "system_event_code.h"
#include <stdint.h>

typedef struct {
  uint32_t flag;
  const char *name;
} flag_map_t;

static const char *TAG = "SYS_MASK";
static EventGroupHandle_t s_event_group = NULL;

void system_event_mask_callback(uint32_t bit, bool add_bit) {
  if (!s_event_group)
    return;

  if (add_bit) {
    xEventGroupSetBits(s_event_group, bit);
  } else {
    xEventGroupClearBits(s_event_group, bit);
  }

  ESP_LOGI(TAG, "Status mask updated: 0x%02lX",
           xEventGroupGetBits(s_event_group));
}

uint32_t system_event_mask_get(void) {
  if (s_event_group == NULL)
    return 0;
  return xEventGroupGetBits(s_event_group);
}

void system_event_mask_code_str(uint32_t bitmask, char *buffer,
                                size_t buffer_size) {
  flag_map_t flags[] = {
      {WIFI_SYS_STATUS_CONNECTED, "WIFI_SYS_STATUS_CONNECTED"},
      {WIFI_SYS_STATUS_CONNECTING, "WIFI_SYS_STATUS_CONNECTING"},
      {WIFI_SYS_STATUS_ENABLED, "WIFI_SYS_STATUS_ENABLED"},
      {MQTT_SYS_STATUS_CONNECTED, "MQTT_SYS_STATUS_CONNECTED"},
      {MQTT_SYS_STATUS_SUBSCRIBED, "MQTT_SYS_STATUS_SUBSCRIBED"},
      {STORAGE_SYS_STATUS_FAILED, "STORAGE_SYS_STATUS_FAILED"},
      {STORAGE_SYS_STATUS_INIZIALIZED, "STORAGE_SYS_STATUS_INIZIALIZED"},
      {I2C_HM3301_SYS_STATUS_INITIALIZED, "I2C_HM3301_SYS_STATUS_INITIALIZED"},
      {UART_GPS_SYS_STATUS_INITIALIZED, "UART_GPS_SYS_STATUS_INITIALIZED"},
      {UART_GPS_SYS_FIXED, "UART_GPS_SYS_FIXED"},
  };

  buffer[0] = '\0'; // reset buffer

  size_t used = 0;

  for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); i++) {
    if (bitmask & flags[i].flag) {
      int written = snprintf(buffer + used, buffer_size - used, "%s%s",
                             (used > 0) ? " | " : "", flags[i].name);

      if (written < 0 || (size_t)written >= buffer_size - used) {
        // buffer pieno o errore
        break;
      }

      used += written;
    }
  }
}

#define REGISTER_MODULE_CALLBACKS(function)                                    \
  function(system_event_mask_callback, system_event_mask_get)

void system_event_register_handler() {
  REGISTER_MODULE_CALLBACKS(wifi_register_system_handler);
  REGISTER_MODULE_CALLBACKS(storage_register_system_handler);
  REGISTER_MODULE_CALLBACKS(mqtt_register_system_handler);
  REGISTER_MODULE_CALLBACKS(gps_register_system_handler);
  REGISTER_MODULE_CALLBACKS(hm3301_register_system_handler);
}

void system_event_mask_init() {
  if (!s_event_group) {
    s_event_group = xEventGroupCreate();
  }

  system_event_register_handler();
}
