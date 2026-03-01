#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "god_header.h"
#include <stdint.h>


static const char *TAG = "SYS_MASK";
static EventGroupHandle_t s_event_group = NULL;

void system_event_mask_init() {
  if (!s_event_group) {
    s_event_group = xEventGroupCreate();
  }
}

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
    if (s_event_group == NULL) return 0;
    return xEventGroupGetBits(s_event_group);
}

#define REGISTER_MODULE_CALLBACKS(function) function(system_event_mask_callback, system_event_mask_get)

void system_event_register_handler() {
 REGISTER_MODULE_CALLBACKS(wifi_register_system_handler);
 REGISTER_MODULE_CALLBACKS(storage_register_system_handler);
 REGISTER_MODULE_CALLBACKS(mqtt_register_system_handler);
}