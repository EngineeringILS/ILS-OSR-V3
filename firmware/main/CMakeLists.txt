# Register ESP-IDF Components
idf_component_register(
    SRCS "main.cpp"      # Register main.cpp
    INCLUDE_DIRS "."     # Register the working directory around main.cpp and this CMakeLists.txt
    REQUIRES common 
             freertos 
             SerialIO    
             lsm9ds1_driver
             i2c_driver
             platforms
             esp_timer
             log
)
