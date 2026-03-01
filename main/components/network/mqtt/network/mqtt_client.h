#ifndef __CONN_MQTT_CLIENT_H__
#define __CONN_MQTT_CLIENT_H__

#include <stdbool.h>
#include <stdint.h>

void mqtt_start_client();
void mqtt_publish_data_client(const char *p_str);
void mqtt_register_system_handler(void (*mqtt_state_handler)(uint32_t bit,
                                                             bool add_bit),
                                  uint32_t (*system_get_state)(void));

#endif // __CONN_MQTT_CLIENT_H__