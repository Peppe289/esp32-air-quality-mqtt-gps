# ESP32 HM3301 & GPS

#### Module:

- GPS: [Grove GPS Module](https://www.seeedstudio.com/Grove-GPS-Module.html)
  - UART: TX GPIO21; RX GPIO 20.
- HM3301: [Grove Laser PM2.5 Sensor HM3301](https://wiki.seeedstudio.com/Grove-Laser_PM2.5_Sensor-HM3301/)
  - I2C: SDA GPIO6; SCL GPIO7.

#### GPS Functionality:
To convert RAW GPS data into a readable format, use the `igrr/libnmea` component.


# Plane to do:

- [ ] MQTT server for push log.
- [ ] SPIFFS: save log when offline.

<!-- ### Notes

build command (build with Make): `idf.py -G "Unix Makefiles" build` -->