# ESP32 HM3301 & GPS

#### Module:

- GPS: [Grove GPS Module](https://www.seeedstudio.com/Grove-GPS-Module.html)
  - UART: TX GPIO21; RX GPIO 20.
- HM3301: [Grove Laser PM2.5 Sensor HM3301](https://wiki.seeedstudio.com/Grove-Laser_PM2.5_Sensor-HM3301/)
  - I2C: SDA GPIO6; SCL GPIO7.

#### GPS Functionality:
To convert RAW GPS data into a readable format, use the `igrr/libnmea` component.

# Work Stuff 

- Synchronization type from: [Synchronization Techniques in Real-Time Operating Systems Implementation and Evaluation on Arduino with FreeRTOS](https://www.researchgate.net/publication/379063740_Synchronization_Techniques_in_Real-Time_Operating_Systems_Implementation_and_Evaluation_on_Arduino_with_FreeRTOS)
- Why C? from: [Performance Evaluation of C/C++, MicroPython, Rust and TinyGo Programming Languages on ESP32 Microcontroller](https://www.mdpi.com/2036322)
- Why LittleFS? from docs: [file system considerations](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-guides/file-system-considerations.html). (SPIFFS is (g)old; I don't need FatFS; I(the docs) like LittleFS).

# Plane to do:

- [ ] MQTT server for push data.
- [ ] LittleFS: save data when offline.

<!-- ### Notes

build command (build with Make): `idf.py -G "Unix Makefiles" build` -->