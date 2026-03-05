#ifndef __CONN_MQTT_CLIENT_H__
#define __CONN_MQTT_CLIENT_H__

#include <stdbool.h>
#include <stdint.h>
#include "../../../system_event_code.h"

#define MQTT_MAIN_TOPIC "/topic/qos0"
#define MQTT_RECOVERY_TOPIC "/topic/qos1"

void mqtt_start_client();
void mqtt_publish_data_client(const char *p_str, const char *p_topic);
void mqtt_register_system_handler(void (*mqtt_state_handler)(uint32_t bit,
                                                             bool add_bit),
                                  uint32_t (*system_get_state)(void));

#endif // __CONN_MQTT_CLIENT_H__