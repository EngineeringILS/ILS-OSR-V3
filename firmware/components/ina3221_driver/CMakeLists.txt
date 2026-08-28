idf_component_register(
    SRCS "ina3221.cpp"

    INCLUDE_DIRS "."

    REQUIRES common        
             driver         
             freertos      
             i2c_driver
)