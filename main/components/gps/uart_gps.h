#ifndef __UART_GPS_H__
#define __UART_GPS_H__

#include "driver/uart.h"

#include "gpgga.h"
#include "gpgll.h"
#include "gpgsa.h"
#include "gpgsv.h"
#include "gprmc.h"
#include "gptxt.h"
#include "gpvtg.h"
#include "nmea.h"

typedef struct gps_data_s {
  int n_satellites;
  struct {
    nmea_position longitude;
    nmea_position latitude;
  } position;
  struct tm time;
  uint8_t valid; // 1 if data is valid, 0 otherwise
} gps_data_t;

void gps_register_system_handler(void (*gps_state_handler)(uint32_t bit,
                                                             bool add_bit),
                                  uint32_t (*system_get_state)(void));
void gps_init_uart(void);
uint8_t gps_read_uart(gps_data_t *);

#endif