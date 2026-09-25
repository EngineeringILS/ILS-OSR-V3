idf_component_register(
    SRCS "lsm9ds1_test.cpp"
    INCLUDE_DIRS "."
    REQUIRES lsm9ds1_driver SerialIO
)
