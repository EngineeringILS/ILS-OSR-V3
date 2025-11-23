# common

## Common Library Directory
This directory contains all shared code between the ESP32 and Jetson OBCs. 

## Common Library Directory Structure
Generally, some sensors, if they are truly ESP-IDF/Jetson specific (e.g. the ESP-IDF JTAG Driver), may be housed in the respective `/firmware` or `/ros2_ws` directories.  
The general structure for common libraries is below:
```
common/
├── CMakeLists.txt # Defines how drivers are to be compiled
├── include
│   └── common
│       ├── drivers  # C++ Header Files for Drivers
│       └── utils    # C++ Header Files for Utilities
└── src
    ├── drivers # C++ Program Files for Drivers
    └── utils   # C++ Program files for Utilities


```

## Common Library Wrapper Structure
- `/common/include/common/drivers/SensorInterface.hpp` Provides the base C++ class to wrap drivers around to ensure they are compliant with our design requirements.   
- **All Drivers are to be wrapped using this class template! Non-Compliant Drivers will not be accepted into this codebase.**



