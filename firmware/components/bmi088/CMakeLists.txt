idf_component_register(SRCS "bmi088.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES freertos esp_driver_spi esp_driver_gpio)