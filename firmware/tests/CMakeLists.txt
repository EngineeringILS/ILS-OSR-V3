cmake_minimum_required(VERSION 3.16)
project(firmware_host_tests CXX)
enable_testing()

add_executable(sensor_tests
    sensor_tests.cpp
    ../components/ina3221_driver/ina3221.cpp
    ../components/max1704x_driver/Max1704x.cpp
    ../components/SerialIO/SerialIO.cpp
)
target_compile_features(sensor_tests PRIVATE cxx_std_17)
target_include_directories(sensor_tests PRIVATE
    stubs
    ../../common/include
    ../components/i2c_driver
    ../components/ina3221_driver
    ../components/max1704x_driver
    ../components/SerialIO
)
add_test(NAME sensor_tests COMMAND sensor_tests)

add_executable(lsm9ds1_tests
    lsm9ds1_tests.cpp
    ../components/lsm9ds1_driver/lsm9ds1.cpp
)
target_compile_features(lsm9ds1_tests PRIVATE cxx_std_17)
target_include_directories(lsm9ds1_tests PRIVATE
    stubs
    ../../common/include
    ../components/i2c_driver
    ../components/lsm9ds1_driver
)
add_test(NAME lsm9ds1_tests COMMAND lsm9ds1_tests)
