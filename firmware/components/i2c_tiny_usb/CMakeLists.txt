idf_component_register(
    SRCS "i2c_tiny_usb.cpp"
         "i2c_tiny_usb_bridge.cpp"
         "i2c_tiny_usb_transport.cpp"

    INCLUDE_DIRS "."

    REQUIRES common
             driver
             i2c_driver
             esp_tinyusb
)
