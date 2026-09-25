idf_component_register(
    SRCS "ina3221_test.cpp"

    INCLUDE_DIRS "."

    REQUIRES common        
             driver         
             freertos      
             max1704x 
             i2c_driver
             i2c_tools
             ina3221_driver
)