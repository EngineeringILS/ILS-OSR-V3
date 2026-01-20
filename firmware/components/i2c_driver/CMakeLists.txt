idf_component_register(
    SRCS "i2c_driver.cpp"

    INCLUDE_DIRS "."

    REQUIRES common        
             driver         
             freertos      
             max1704x 
             platforms
)