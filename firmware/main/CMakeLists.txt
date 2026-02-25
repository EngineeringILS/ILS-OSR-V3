# Register ESP-IDF Components
idf_component_register(
    SRCS "main.cpp"      # Register main.cpp
    INCLUDE_DIRS "."     # Register the working directory around main.cpp and this CMakeLists.txt
    REQUIRES common 
             freertos 
             SerialIO    
             max1704x_driver
             max1704x_test
             i2c_driver
             i2c_tools
               # Automatically Include and Register the Common Library as a Requirement
)