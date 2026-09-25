idf_component_register(
    SRCS "LED.cpp"
         "Neopixel.cpp"

    INCLUDE_DIRS "."

    REQUIRES common
             driver
             esp_timer
             led_strip
)
