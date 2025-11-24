# Register ESP-IDF Components
idf_component_register(
    SRCS "main.cpp"      # Register main.cpp
    INCLUDE_DIRS "."     # Register the working directory around main.cpp and this CMakeLists.txt
    REQUIRES common      # Automatically Include and Register the Common Library as a Requirement
)