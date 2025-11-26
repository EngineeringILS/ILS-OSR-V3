# Common Shared Library

### Purpose
Provide a Pure C++ shared library that can be used for:
1. Embedded systems using ESP-IDF (ESP32)
2. Computer systems using ROS2 (OBC)
2. Any other system capable of running C++ 

### Key Requirements
- Presently, the `CMakeList` for this library has special handling for ESP-IDF based targets. It follows that if this library were to be made compatible for compilation on more platforms, further inclusions would be needed on a per-platform basis; however, this does not undermine the shared nature of this library.

- This project must be able to be compiled with a simple C++ only development environment to prioritize universal development. Therefore, no platform-specific code is to be introduced.

### Project Structure
```
lunabotics-cdh-dev/common
├── CMakeLists.txt # Defines how base classes are to be compiled
├── include  
│   └── common  
│       ├── drivers # /common base class definitions
│       └── utils   # /common utility definitions
├── src
│   ├── drivers # /common base class implementations
│   └── utils   # /common utility implementations
└── test # /common specific tests
```

### Planned and Implemented Base Classes
| Base Class | Description | Implementation Status | 
|----------|----------|----------|
| SensorInterface.hpp | Abstract base class for all sensors | Implemented |
| MotorInterface.hpp  | Abstract base class for all motors  | Planned |
| SerialCommunication.hpp | Abstract base class for all serial communication | Planned |
| WiFiCommunication.hpp | Abstract base class for all wifi communication | Planned |

### Planned and Implemented Example Drivers
| Driver | Description | Implementation Status |
|----------|----------|----------|
| FakeIMU.hpp | Fake IMU Driver with example methods for SensorInterface wrapping | Implemented |
| FakeMotor.hpp | Fake Motor Driver with example methods for Motor Interface wrapping | Planned |

### Supported Platforms 
| Platform | Support Status |
|----------|----------|
| ESP-IDF | Full Support |
| WSL with CMake | Full Support
| ROS2 | Planned |

### Development Standards
1. Plan before programming, document driver plans in `/documentation`.
2. Ensure platform agnosticism for driver templates built in `/common`.
3. Create unit tests for `/common` specific functionality.
4. Ensure proper addition of drivers to the `/common/CMakeLists` file.