idf_component_register(
    SRCS "lsm9ds1.cpp"
    INCLUDE_DIRS "."
    REQUIRES common i2c_driver freertos
)
