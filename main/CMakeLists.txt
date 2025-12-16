idf_component_register(SRCS "i2c_hm3301.c" "main.c" "uart_gps.c"
                    INCLUDE_DIRS "."
                    REQUIRES libnmea spi_flash driver esp_driver_i2c)