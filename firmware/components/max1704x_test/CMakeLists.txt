idf_component_register(
    SRCS "Max1704x_test.cpp"

    INCLUDE_DIRS "."

    REQUIRES common        
             driver         
             freertos      
             max1704x 
             i2c_driver
             i2c_tools
             max1704x_driver
)