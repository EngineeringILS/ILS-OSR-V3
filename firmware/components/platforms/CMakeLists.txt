idf_component_register(
    SRCS "Platforms.cpp"

    INCLUDE_DIRS "."

    REQUIRES common        
             driver         
             freertos       
)