# ESP32 HM3301 & GPS

#### Module:

- GPS: [Grove GPS Module](https://www.seeedstudio.com/Grove-GPS-Module.html)
  - UART: TX GPIO21; RX GPIO 20.
- HM3301: [Grove Laser PM2.5 Sensor HM3301](https://wiki.seeedstudio.com/Grove-Laser_PM2.5_Sensor-HM3301/)
  - I2C: SDA GPIO6; SCL GPIO7.

#### GPS Functionality:
To convert RAW GPS data into a readable format, use the `igrr/libnmea` component.

# Logbook:

This repository is part of my **internship** project where I am working with an **ESP32** and various sensors like the **GPS** and **PM2.5 sensor (HM3301)**. The purpose of this project is to implement a real-time system for data collection and processing from multiple devices. Below are some key learnings and challenges I have encountered so far:

- **GPS**: Using pthread (POSIX) introduces more overhead than directly reading from UART until all data is received. Previously, I used a pthread with a shared variable protected by a mutex to continuously read the latest data from UART. However, I found that this introduced unnecessary overhead compared to simply reading directly from the UART until all the data is received, as the thread and mutex handling added complexity and performance costs.
- **HM3301**: It's necessary to enable `CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2` in sdkconfig. If this option is disabled, the system will use the older drivers, which is not optimal.
- ~~**Wi-Fi Configuration via Bluetooth**:I implemented a feature where, when the device is not connected to any network via Wi-Fi, it automatically opens a Bluetooth connection (GATT BLE) for Wi-Fi configuration. The user can send Wi-Fi authentication data over Bluetooth using the **nRF Connect** app (available on the Play Store). By connecting to the device, the user can write the network credentials to **Service 0x0104**, specifically using **Characteristic 0x104A for the SSID** and **Characteristic 0x104B for the PASSWORD**. Once the data is received, the Bluetooth connection is closed, and the system attempts to connect to the Wi-Fi network. If the connection fails or is lost, the system shuts down the Wi-Fi and reopens the Bluetooth for a new configuration attempt. This ensures that the device can be easily reconfigured without needing physical access or resetting it.~~
- **Custom Partition for Larger Code**: To fit the larger codebase, including both Bluetooth and Wi-Fi drivers, I had to create a custom partition. Without this partition, the default partition layout was insufficient, and I wasn't able to fit both the Bluetooth and Wi-Fi drivers into the same firmware for flashing on the **ESP32-C3**. This solution allowed me to properly include all the necessary drivers and functionalities in the firmware. [Custom Partition here](partitions.csv).
- **System Event Mask**: To manage global system states (Wi-Fi, MQTT, Storage) efficiently, I implemented the `system_event_mask.c` module. Instead of using global variables protected by Mutexes—which would have introduced overhead and potential race conditions—I utilized FreeRTOS Event Groups. This ensures atomic operations on individual bits and allows tasks to react to state changes without polling, significantly reducing CPU usage. I adopted a Dependency Injection pattern: sub-modules (Wi-Fi, MQTT, etc.) do not depend directly on the event manager; instead, they receive function pointers (callbacks) during initialization. All state codes are centralized in `system_event_code.h` to prevent bit collisions and simplify system-wide debugging.
- **Offline Data Persistence**: I implemented a persistence layer using a dedicated **SPIFFS partition** to handle network outages. Telemetry data is stored in **JSONL (JSON Lines)** format, where each line represents a standalone JSON object. To ensure efficiency and durability, I used fseek and ftell to implement a byte-offset tracking system; this allows the recovery process to resume from the exact last successful transmission point after a crash or disconnection, avoiding data duplication.

## Private Environment Variables

Create `private.txt` file:

```CMake
set(ENV{MQTT_BROKER}    "IP")
set(ENV{MQTT_PORT}      "PORT")
set(ENV{WIFI_SSID}      "WIFI SSID")
set(ENV{WIFI_PASSWD}    "WIFI PASSWD")

```

<!-- ### Notes

build command (build with Make): `idf.py -G "Unix Makefiles" build` -->
