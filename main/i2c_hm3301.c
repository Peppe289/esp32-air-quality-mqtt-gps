// Enable `CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2`
// Version 1 is deprecated (from docs).
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "i2c_hm3301.h"

static i2c_master_dev_handle_t dev_handle;
static i2c_master_bus_handle_t bus_handle;

static const char *TAG = "HM3301";

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

static bool hm3301_verify_checksum(uint8_t *data) {
  uint8_t sum = 0;
  for (int i = 0; i < HM3301_BIT_LEN - 1; i++) {
    sum += data[i];
  }
  return ((uint8_t)(sum & 0xFF)) == data[HM3301_BIT_LEN - 1];
}

uint8_t i2c_hm3301_read(uint8_t *raw_data, struct hm3301_pm *hm3301) {
  uint8_t data_rd[HM3301_BIT_LEN] = {0};
  uint8_t cmd = HM3301_READ_CMD;
  esp_err_t err;

  // Send command code to read (0x88) then read (29 byte)
  err = i2c_master_transmit_receive(dev_handle, &cmd, 1, data_rd,
                                    HM3301_BIT_LEN, 1000 / portTICK_PERIOD_MS);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C Read-Receive failed: %s", esp_err_to_name(err));
    return err;
  }

  if (!hm3301_verify_checksum(data_rd)) {
    ESP_LOGE(TAG, "Checksum Failed");
    return -1;
  }

  if (raw_data != NULL) {
    memcpy(raw_data, data_rd, sizeof(data_rd));
  }

  if (!hm3301) {
    ESP_LOGE(TAG, "Null pointer destination. Allocation Failled!");
    return -1;
  }

  hm3301->pm1_0 = HM3301_PM1_0(data_rd);
  hm3301->pm2_5 = HM3301_PM2_5(data_rd);
  hm3301->pm10 = HM3301_PM10(data_rd);

  return 0;
}

void init_i2c_hm3301() {
  i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = I2C_NUM_0,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .scl_io_num = HM3301_SCL_IO_NUM,
      .sda_io_num = HM3301_SDA_IO_NUM,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C Bus Creation Failed: %s", esp_err_to_name(err));
    return;
  }

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = HM3301_DEV_ADDR,
      .scl_speed_hz = 50000,
  };

  err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HM3301 addition failed: %s", esp_err_to_name(err));
    return;
  }

  err = i2c_master_probe(bus_handle, HM3301_DEV_ADDR, pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Sensor not found on 0x%02x address!.", HM3301_DEV_ADDR);
  } else {
    ESP_LOGI(TAG, "Found HM3301.");
  }

  ESP_LOGI(TAG, "Wait for the sensor to stabilize...");
  vTaskDelay(pdMS_TO_TICKS(2000));

  uint8_t dummy[29];
  uint8_t cmd = 0x88;
  for (int i = 0; i < 3; i++) {
    i2c_master_transmit_receive(dev_handle, &cmd, 1, dummy, 29,
                                pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  ESP_LOGI(TAG, "Buffer empty");
}