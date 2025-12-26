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

# Logbook:

This repository is part of my **internship** project where I am working with an **ESP32** and various sensors like the **GPS** and **PM2.5 sensor (HM3301)**. The purpose of this project is to implement a real-time system for data collection and processing from multiple devices. Below are some key learnings and challenges I have encountered so far:

- **GPS**: Using pthread (POSIX) introduces more overhead than directly reading from UART until all data is received. Previously, I used a pthread with a shared variable protected by a mutex to continuously read the latest data from UART. However, I found that this introduced unnecessary overhead compared to simply reading directly from the UART until all the data is received, as the thread and mutex handling added complexity and performance costs.
- **HM3301**: It's necessary to enable `CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2` in sdkconfig. If this option is disabled, the system will use the older drivers, which is not optimal.

# Plane to do:

- [ ] MQTT server for push data.
- [ ] LittleFS: save data when offline.

<!-- ### Notes

build command (build with Make): `idf.py -G "Unix Makefiles" build` -->