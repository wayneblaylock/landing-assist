idf_component_register(SRCS "lidar.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES freertos esp_driver_uart)
