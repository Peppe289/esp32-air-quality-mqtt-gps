#ifndef __COMPONENT_STORAGE_H__
#define __COMPONENT_STORAGE_H__

#include <stdbool.h>
#include <stdint.h>
#include "../../system_event_code.h"

void storage_init_fs(void);
void storage_write_on_file(const char *p_str);
void storage_recovery_data(void (*mqtt_callback)(const char *p_json));
void storage_register_system_handler(
    void (*storage_state_handler)(uint32_t bit, bool add_bit),
    uint32_t (*system_get_state)(void));

#endif // __COMPONENT_STORAGE_H__