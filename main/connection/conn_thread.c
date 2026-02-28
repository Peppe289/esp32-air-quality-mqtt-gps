#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>

#include "ble_gatts.h"
#include "portmacro.h"
#include "wifi_conn.h"

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
    if (get_wifi_status() & (WIFI_STATE_CONNECTING | WIFI_STATE_CONNECTED))
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