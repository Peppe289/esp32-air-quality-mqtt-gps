#ifndef __GOD_HEADER_H__
#define __GOD_HEADER_H__

#include "gps/uart_gps.h"
#include "network/mqtt/network/mqtt_client.h"
#include "network/wifi/wifi_conn.h"
#include "pm_sensors/i2c_hm3301.h"
#include "storage/include/storage.h"
#include "led.h"

void system_event_mask_init();
uint32_t system_event_mask_get(void);
void system_event_mask_code_str(uint32_t bitmask, char *buffer,
                                size_t buffer_size);

#endif // end __GOD_HEADER_H__