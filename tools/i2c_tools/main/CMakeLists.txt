idf_component_register(SRCS "main.c"
                            "cmd_i2ctools.c"
                       PRIV_REQUIRES esp_driver_i2c console esp_driver_gpio
                       INCLUDE_DIRS ".")
