idf_component_register(
    SRCS "Max1704x.cpp"

    INCLUDE_DIRS "."

    REQUIRES common        
             driver         
             freertos      
             max1704x 
             i2c_driver
)