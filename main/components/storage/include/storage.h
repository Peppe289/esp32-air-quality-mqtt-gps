#ifndef __COMPONENT_STORAGE_H__
#define __COMPONENT_STORAGE_H__

void storage_init_fs(void);
void storage_write_on_file(const char *p_str);
void storage_recovery_data(void (*mqtt_callback)(const char *p_json));

#endif // __COMPONENT_STORAGE_H__