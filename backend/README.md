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