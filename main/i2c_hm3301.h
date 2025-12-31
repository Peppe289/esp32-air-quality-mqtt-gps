#ifndef __I2C_HM3301_H__
#define __I2C_HM3301_H__

#include <stdint.h>

typedef struct hm3301_pm {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
} hm3301_pm;

// Stream data length
#define HM3301_BIT_LEN 29
// Standard HM3301 device address
#define HM3301_DEV_ADDR 0x40
#define HM3301_READ_CMD 0x88
// From https://wiki.seeedstudio.com/Grove-Laser_PM2.5_Sensor-HM3301/
// This module use 0x00 0xFF and not standard 0x42 0x4D like other PMS.
#define HM3301_HEADER_INTEGRITY(slave_data)                                    \
  (slave_data[0] == 0x00 && slave_data[1] == 0xFF)
#define HM3301_SDA_IO_NUM 6 // This is my SDA PIN
#define HM3301_SCL_IO_NUM 7 // This is my SCL PIN

//#define HM3301_RAW_DATA

// 2 way to get value. RAW or Standard (fixed from proprietary algoritm)
#ifdef HM3301_RAW_DATA
#define HM3301_PM1_0(data) (data[4] << 8) | data[5]
#define HM3301_PM2_5(data) (data[6] << 8) | data[7]
#define HM3301_PM10(data) (data[8] << 8) | data[9]
#else
#define HM3301_PM1_0(data) (data[10] << 8) | data[11]
#define HM3301_PM2_5(data) (data[12] << 8) | data[13]
#define HM3301_PM10(data) (data[14] << 8) | data[15]
#endif

void init_i2c_hm3301();
uint8_t i2c_hm3301_read(uint8_t *raw_data, struct hm3301_pm *hm3301);

#endif