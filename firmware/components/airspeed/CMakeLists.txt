idf_component_register(SRCS "airspeed.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES freertos esp_driver_i2c)
