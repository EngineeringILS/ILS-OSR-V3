### Lunabotics ESP32 I2C Driver Component
- This library enables full I2C device support for the ESP32 using ESP-IDF's i2c_master driver using C++ wrappers to define the `I2CBus` and `I2CDevice` classes.
- This class must be completed in order to comply with Project Design Principles.
- [ ] Code Implemented to Meet Project Design Principles and provide full I2C functionality.
    - [x] Provides tested I2C Bus Support
    - [x] Provides tested I2C Device Register Read Support
    - [ ] Provides tested I2C Device Register Write Support
    - [ ] Gracefully handles I2C Device Disconnections 
    - [ ] Handles Low Battery Deep Sleep (stretch goal)
    - [ ] Handles Severe Hardware Failures (IC Failure or Disconnection for unknown reason)
    - [ ] Rule of Three Compliance in `I2CBus`
    - [x] Rule of Three Compliance in `I2CDevice`
Functionality
- [ ] Code Functionally Complete 
    - [x] Compilation
    - [ ] Unit Tests
    - [ ] Hardware Tests
    - [x] Platform specific I2C options
