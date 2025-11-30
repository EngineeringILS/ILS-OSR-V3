# ESP-IDF ESP32 Firmware

### Purpose
Provide a Wi-Fi/Serial command interface for:
1. Recieving Ground Station commands (over Wi-Fi) and OBC System State (over Serial)
2. Processing commands and monitoring the system state
3. Transmitting OBC System State (over Wi-Fi), Processed Commands (over Serial), and physical Start/Stop/Restart (over GPIO)

### Simplified Communication Flows
1. Ground Station (Wi-Fi) <--> ESP32 (Serial) <--> OBC
2. Ground Station (commands) --> ESP32 (processed commands + physical controls) --> OBC
3. OBC (System State) --> ESP32 (processed System State) --> Ground Station

### Project Structure
1. The ESP32 project is programmed in C++ using the ESP-IDF framework.
2. The `/common` library is incorporated into the `CMakeLists` of this project, ensuring proper compilation and linking.
3. Communication standards are to be defined using the `/common` library framework. 
4. The filesystem structure of this module is below:
```
lunabotics-cdh-dev/firmware
├── drivers # ESP32 drivers derived from base classes in /common
├── main  # ESP32 app_main() in main.cpp
└── tests # ESP32 platform specific tests.

lunabotics-cdh-dev/common
├── include  
│   └── common  
│       ├── drivers # /common base class definitions
│       └── utils   # /common utility definitions
├── src
│   ├── drivers # /common base class implementations
│   └── utils   # /common utility implementations
└── test # /common specific tests, likely un-used for the ESP32 portion of this project.
```

### Development Standards
1. Plan before programming, document program plans in `/documentation`
2. Ensure adherence to `/common` libraries and communication standards.
3. Create unit tests for `/firmware` specific functionality and `/common` specific functionality.