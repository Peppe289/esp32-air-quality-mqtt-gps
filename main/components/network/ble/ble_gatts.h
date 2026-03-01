#ifndef __BLE_GATTS_H__
#define __BLE_GATTS_H__
#include <stdint.h>

#define AUTH_MAX_LENGTH 64

void start_bt();
void disable_bt();
uint8_t isBTEnabled();

char *ble_getSSID(void);     /**< SSID of target AP. */
char *ble_getPassword(void); /**< Password of target AP. */

#endif // __BLE_GATTS_H__