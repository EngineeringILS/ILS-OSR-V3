idf_component_register(
    SRCS "LED.cpp"

    INCLUDE_DIRS "."

    REQUIRES common
             driver
             esp_timer
)
