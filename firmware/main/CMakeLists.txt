# The ESP32-S2 has no USB Serial/JTAG console for the test harness, so its build is the
# stand-alone I2C breakout for the Feather ESP32-S2 Reverse TFT.
idf_build_get_property(target IDF_TARGET)
if(target STREQUAL "esp32s2")
    set(main_src "breakout_main.cpp")
else()
    set(main_src "main.cpp")
endif()

# Register ESP-IDF Components
idf_component_register(
    SRCS ${main_src}     # Register the entry point for this target
    INCLUDE_DIRS "."     # Register the working directory around main.cpp and this CMakeLists.txt
    REQUIRES common 
             freertos 
             SerialIO    
             max1704x_driver
             max1704x_test
             ina3221_driver
             ina3221_test
             lsm9ds1_driver
             lsm9ds1_test
             i2c_driver
             i2c_tiny_usb
             i2c_tools
             led_driver
             platforms
)
