idf_component_register(
    SRCS "SerialIO.cpp"

    INCLUDE_DIRS "."

    REQUIRES common        
             driver         
             freertos      
)