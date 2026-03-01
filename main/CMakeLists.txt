set(srcs
    "main.c"
    "components/pm_sensors/i2c_hm3301.c"
    "components/gps/uart_gps.c"
    "components/network/wifi/wifi_conn.c"
    "components/network/ble/ble_gatts.c"
    "components/network/mqtt/mqtt_client.c"
    "components/storage/storage.c"
)

set(include_dirs
    "."
    "components"
    "components/pm_sensors"
    "components/gps"
    "components/network/wifi"
    "components/network/ble"
    "components/network/mqtt"
    "components/storage"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES libnmea spi_flash driver esp_driver_i2c bt spiffs nvs_flash esp_wifi mqtt esp-tls
)

target_compile_options(${COMPONENT_LIB} PRIVATE 
    "-Wshadow" 
    "-Wformat=2" 
    "-ffunction-sections" 
    "-fdata-sections"
)