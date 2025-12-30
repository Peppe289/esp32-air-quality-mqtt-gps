#ifndef __BLE_GATTS_H__
#define __BLE_GATTS_H__
#include <stdint.h>

void start_bt();
void disable_bt();
uint8_t isBTEnabled();

extern char ssid[32];     /**< SSID of target AP. */
extern char password[64]; /**< Password of target AP. */

#endif // __BLE_GATTS_H__