# ESP32 HM3301 & GPS

#### Module:

- GPS: [Grove GPS Module](https://www.seeedstudio.com/Grove-GPS-Module.html)
  - UART: TX GPIO21; RX GPIO 20.
- HM3301: [Grove Laser PM2.5 Sensor HM3301](https://wiki.seeedstudio.com/Grove-Laser_PM2.5_Sensor-HM3301/)
  - I2C: SDA GPIO6; SCL GPIO7.

#### GPS Functionality:
To convert RAW GPS data into a readable format, use the `igrr/libnmea` component.

### LED Status Legend

| Status                      | LED Behavior           | Description |
|---------------------------|----------------------|-------------|
| 🔴 Critical Error          | Fast red blinking     | HM3301 not initialized, or GPS not initialized, or storage failure. *The system cannot operate correctly.* |
| 🟢 Normal Operation        | Solid green           | HM3301 initialized, GPS fix acquired, MQTT connected. System is fully operational and transmitting data. |
| 🟢 MQTT Disconnected       | Fast green blinking   | HM3301 initialized, GPS fix acquired, MQTT not connected. Data is not being sent to the server. |
| 🟢 Wi-Fi Disconnected      | Slow green blinking   | HM3301 initialized, GPS fix acquired, Wi-Fi not connected. |
| 🔴 Warning / Fallback State| Slow red blinking     | System is in an unexpected or partially working state. |

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

# PM Monitoring Backend

A robust Node.js backend designed to collect, process, and serve environmental data from both mobile and fixed sensors. This system integrates multiple MQTT brokers and provides a REST API restricted to local and VPN networks.

## 🚀 Features

- **Dual MQTT Integration**: Simultaneously handles data from a public mobile sensor broker and a confidential fixed station broker.
- **Persistent Storage**: High-performance data logging using SQLite with `better-sqlite3` and WAL (Write-Ahead Logging) mode.
- **Security**: Integrated VPN-only middleware to protect sensitive fixed station data.
- **Data Processing**: Automatic `NMEA`-to-Decimal coordinate conversion and GPS quality filtering.
- **Clean Architecture**: Fully documented JSDoc code following the Service-Route pattern.

## 📂 Project Structure

```sh
backend/
├── config/              # Database and MQTT connection settings
├── data/                # SQLite .db files and static JSON configurations
├── routes/              # Express API route definitions
├── services/            # Business logic and data transformation
├── .env                 # Environment variables (Sensitive data)
└── server.js            # Main entry point
```

## 🛠️ Environment Configuration

```sh
# Mobile Sensor Broker
MQTT_APPLICATION_ADDR_WS=<your-mobile-broker-url>
MQTT_APPLICATION_TOPIC=<topic>

# Confidential Fixed Stations Broker
MQTT_CONFIDENTIALS_WS=<addr>
MQTT_CONFIDENTIALS_PORT=<port>
MQTT_CONFIDENTIALS_USERNAME=<your_username>
MQTT_CONFIDENTIALS_PASSWORD=<your_password>
MQTT_CONFIDENTIALS_TOPIC=<#>
```


## 🛡️ Security Note

The **vpnOnly** middleware identifies clients based on the `10.6.0.0/24` subnet. Ensure your **WireGuard** or **OpenVPN** server correctly forwards the client's real IP address via headers if using a reverse proxy.

# PM Monitor Frontend

A React-based frontend application that visualizes environmental monitoring data on an interactive map. It displays sampling points from mobile sensors and provides access to fixed station data restricted to VPN connections.

## 🚀 Features

- **Interactive Map**: Displays sampling points from mobile sensors on a map interface.
- **Real-time Updates**: Fetches data from the backend API with configurable update intervals and latency monitoring.
- **VPN-Restricted Access**: Fixed station data is only visible to users connected via VPN, ensuring data security.
- **User-Friendly Interface**: Built with React and Vite for fast development and optimal performance.

