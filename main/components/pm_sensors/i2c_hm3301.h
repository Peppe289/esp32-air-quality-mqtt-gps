#ifndef __I2C_HM3301_H__
#define __I2C_HM3301_H__

#include <stdint.h>

typedef struct hm3301_data_s {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
} hm3301_data_t;

/* --- I2C Protocol Definitions --- */
#define HM3301_I2C_ADDRESS 0x40  /**< Standard 7-bit I2C address */
#define HM3301_READ_COMMAND 0x88 /**< Command to trigger data transmission */
#define HM3301_FRAME_LENGTH 29   /**< Total bytes received per sensor read */

/* --- Hardware Mapping (GPIO) --- */
#define HM3301_SDA_GPIO 6 /**< ESP32-C3 SDA Pin */
#define HM3301_SCL_GPIO 7 /**< ESP32-C3 SCL Pin */

// #define HM3301_RAW_DATA

// 2 way to get value. RAW or Standard (fixed from proprietary algoritm)
#ifdef HM3301_RAW_DATA
#define HM3301_GET_PM1_0(data) (data[4] << 8) | data[5]
#define HM3301_GET_PM2_5(data) (data[6] << 8) | data[7]
#define HM3301_GET_PM10(data) (data[8] << 8) | data[9]
#else
#define HM3301_GET_PM1_0(data) (data[10] << 8) | data[11]
#define HM3301_GET_PM2_5(data) (data[12] << 8) | data[13]
#define HM3301_GET_PM10(data) (data[14] << 8) | data[15]
#endif

void hm3301_init_i2c();
uint8_t hm3301_read_i2c(uint8_t *raw_data, hm3301_data_t *hm3301);

#endif