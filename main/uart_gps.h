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

typedef struct {
  int n_satellites;
  struct {
    nmea_position longitude;
    nmea_position latitude;
  } position;
} nmea_uart_data_s;

//nmea_uart_data_s get_latest_nmea_data(nmea_t index);
void init_gps_uart(void);
nmea_uart_data_s *gps_read_task();

#endif