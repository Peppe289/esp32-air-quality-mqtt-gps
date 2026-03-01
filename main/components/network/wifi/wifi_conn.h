#ifndef __WIFI_CONN_H__
#define __WIFI_CONN_H__
#include <stdint.h>

#define WIFI_STATE_IDLE          BIT(0)
#define WIFI_STATE_CONNECTING    BIT(1)
#define WIFI_STATE_CONNECTED     BIT(2)
#define WIFI_STATE_DISCONNECTED  BIT(3)
#define WIFI_STATE_ERROR         BIT(4)

void wifi_init_sta(char *ssid, char *passwd);
uint8_t get_wifi_status();
void disableWIFI();

#endif // __WIFI_CONN_H__