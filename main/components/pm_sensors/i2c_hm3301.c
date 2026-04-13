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
#include "system_event_code.h"

#include "i2c_hm3301.h"

typedef struct hm3301_callbacks_t {
  void (*hm3301_state_handler)(uint32_t bit, bool add_bit);
  uint32_t (*system_get_state)(void);
} hm3301_callbacks_t;

static hm3301_callbacks_t s_hm3301_callbacks = {NULL, NULL};

static i2c_master_dev_handle_t dev_handle;
static i2c_master_bus_handle_t bus_handle;

static const char *TAG = "HM3301";

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */

/**
 * @brief Validates the integrity of the data packet from HM3301.
 * * Computes the sum of the first 28 bytes and compares it with the checksum
 * byte (the 29th byte) to ensure data transmission accuracy.
 * * @param p_raw_buffer Pointer to the 29-byte raw data buffer.
 * @return true if checksum is valid, false otherwise.
 */
static bool hm3301_verify_checksum(uint8_t *p_raw_buffer) {
  uint8_t sum = 0;
  for (int i = 0; i < HM3301_FRAME_LENGTH - 1; i++) {
    sum += p_raw_buffer[i];
  }
  return ((uint8_t)(sum & 0xFF)) == p_raw_buffer[HM3301_FRAME_LENGTH - 1];
}

void hm3301_register_system_handler(void (*hm3301_state_handler)(uint32_t bit,
                                                                   bool add_bit),
                                      uint32_t (*system_get_state)(void)) {
  s_hm3301_callbacks.hm3301_state_handler = hm3301_state_handler;
  s_hm3301_callbacks.system_get_state = system_get_state;
}

static inline void set_hm3301_status(uint32_t bit, bool add_bit) {
  if (s_hm3301_callbacks.hm3301_state_handler) {
    s_hm3301_callbacks.hm3301_state_handler(bit, add_bit);
  }
}

/**
 * @brief Performs a synchronized read from the HM3301 sensor.
 * * Sends the read command (0x88) and receives the 29-byte data packet.
 * It handles checksum verification and maps the raw data to the PM structure.
 * * @param[out] p_opt_raw_buffer Optional pointer to store the raw 29-byte I2C
 * buffer. Pass NULL if not needed.
 * @param[out] p_hm3301_data Pointer to the destination structure for parsed
 * PM1.0, PM2.5, and PM10 values.
 * @return uint8_t
 * - 0: Success.
 * - ESP_ERR_*: I2C transmission error codes.
 * - -1: Checksum failure or null pointer assignment.
 */
uint8_t hm3301_read_i2c(uint8_t *p_opt_raw_buffer,
                        hm3301_data_t *p_hm3301_data) {
  uint8_t read_buffer[HM3301_FRAME_LENGTH] = {0};
  const uint8_t trigger_cmd = HM3301_READ_COMMAND;
  esp_err_t err;

  // Send command code to read (0x88) then read (29 byte)
  err = i2c_master_transmit_receive(dev_handle, &trigger_cmd, 1, read_buffer,
                                    HM3301_FRAME_LENGTH, 1000 / portTICK_PERIOD_MS);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C Read-Receive failed: %s", esp_err_to_name(err));
    return err;
  }

  if (!hm3301_verify_checksum(read_buffer)) {
    ESP_LOGE(TAG, "Checksum Failed");
    return -1;
  }

  if (p_opt_raw_buffer != NULL) {
    memcpy(p_opt_raw_buffer, read_buffer, sizeof(read_buffer));
  }

  if (!p_hm3301_data) {
    ESP_LOGE(TAG, "Null pointer destination. Allocation Failled!");
    return -1;
  }

  p_hm3301_data->pm1_0 = HM3301_GET_PM1_0(read_buffer);
  p_hm3301_data->pm2_5 = HM3301_GET_PM2_5(read_buffer);
  p_hm3301_data->pm10 = HM3301_GET_PM10(read_buffer);

  return 0;
}

/**
 * @brief Initializes the I2C Master Bus and the HM3301 device.
 * * This function:
 * 1. Allocates the I2C master bus (Driver V2).
 * 2. Adds the HM3301 sensor to the bus with a specific clock speed (50kHz).
 * 3. Probes the bus to confirm sensor presence.
 * 4. Implements a stabilization delay and performs dummy reads to clear
 * the sensor's internal buffer for consistent data.
 */
void hm3301_init_i2c() {
  i2c_master_bus_config_t i2c_mst_config = {
      .i2c_port = I2C_NUM_0,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .scl_io_num = HM3301_SCL_GPIO,
      .sda_io_num = HM3301_SDA_GPIO,
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
      .device_address = HM3301_I2C_ADDRESS,
      .scl_speed_hz = 50000,
  };

  err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HM3301 addition failed: %s", esp_err_to_name(err));
    return;
  }

  err = i2c_master_probe(bus_handle, HM3301_I2C_ADDRESS, pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Sensor not found on 0x%02x address!.", HM3301_I2C_ADDRESS);
  } else {
    ESP_LOGI(TAG, "Found HM3301.");
    set_hm3301_status(I2C_HM3301_SYS_STATUS_INITIALIZED, true);
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