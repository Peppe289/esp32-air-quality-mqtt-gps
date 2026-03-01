#ifndef __WIFI_CONN_H__
#define __WIFI_CONN_H__

#include <stdbool.h>
#include <stdint.h>

#define WIFI_STATE_IDLE          BIT(0)
#define WIFI_STATE_CONNECTING    BIT(1)
#define WIFI_STATE_CONNECTED     BIT(2)
#define WIFI_STATE_DISCONNECTED  BIT(3)
#define WIFI_STATE_ERROR         BIT(4)

void wifi_init_sta(char *ssid, char *passwd);
uint8_t get_wifi_status();
void disableWIFI();
void wifi_register_system_handler(void (*wifi_state_handler)(uint32_t bit,
                                                             bool add_bit),
                                  uint32_t (*system_get_state)(void));

#endif // __WIFI_CONN_H__