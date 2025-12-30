#ifndef __WIFI_CONN_H__
#define __WIFI_CONN_H__
#include <stdint.h>

void wifi_init_sta(char *ssid, char *passwd);
uint8_t isWiFiConnecting();
void disableWIFI();

#endif // __WIFI_CONN_H__