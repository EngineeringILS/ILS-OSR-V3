### Basic SerialIO Library
- This library provides a simple interface for managing serial input/output operations using the ESP32's JTAG interface.
- This class will need to be refactored in order to comply with this project's design principles.
- [ ] Code Refactored to Meet Project Design Principles
- [ ] Code Functionally Complete 
    - [x] Compilation (ESP-IDF v5.4.3, ESP32-S3)
    - [x] Host tests for initialization, cleanup, and long/partial output
    - [ ] Hardware Tests
    - [ ] Refactored Derivative of the `SerialInterface` / `ESPSerialInterface`
    - [ ] Compliance with `Units.hpp` and `DataTypes.hpp`

### Usage Notes
- `init()` is idempotent and rejects empty buffers or input capacity below two bytes.
- Input accepts printable characters up to `io_buffer_size - 1` and supports backspace.
- Output is independent of input capacity and retries partial writes.
- Copying is disabled because the instance owns the installed driver.

> Note: This remains a hardware-test console. Migration to the shared serial interface and physical regression testing are pending.
