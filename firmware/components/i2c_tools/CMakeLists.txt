idf_component_register(
    SRCS "i2c_tools.cpp"

    INCLUDE_DIRS "."

    REQUIRES common        
             driver         
             freertos      
             i2c_driver
             SerialIO
)