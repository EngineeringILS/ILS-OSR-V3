### Lunabotics ESP32 I2C-Tiny-USB Component
- Exposes an `I2CBus` to a USB host as an [I2C-Tiny-USB](https://github.com/harbaum/I2C-Tiny-USB) adapter. Linux binds its `i2c-tiny-usb` driver, and the bus becomes `/dev/i2c-N` for `i2cdetect`, `i2cget`, `i2cset`, `i2cdump` and `i2ctransfer`.
- This is a port of the Zephyr [`i2c_tiny_usb` sample](https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb/i2c_tiny_usb) to ESP-IDF. It uses TinyUSB (`espressif/esp_tinyusb`) and this project's `i2c_driver`.
- The USB function is one vendor-specific interface without endpoints. Every command is a vendor request on the default control pipe.

| Class | Header | Role |
|-------|--------|------|
| `I2CTinyUSB` | `i2c_tiny_usb.hpp` | Entry point. Installs TinyUSB, provides the descriptors, and routes vendor requests to the bridge |
| `I2CTinyUSBBridge` | `i2c_tiny_usb_bridge.hpp` | Protocol handling ported from the Zephyr sample. No USB or ESP-IDF dependency |
| `I2CBusTransport` | `i2c_tiny_usb_transport.hpp` | Runs each transaction on an `I2CBus` with i2c_master defined operations, which keeps the repeated starts |

### Quick Start
Add the component to `main/CMakeLists.txt`. The component's `idf_component.yml` fetches `esp_tinyusb` on the first build.
```cmake
idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES i2c_driver
             i2c_tiny_usb
             platforms
)
```

A complete `main.cpp` that turns the board into a USB to I2C adapter at boot:
```cpp
#include <Platforms.hpp>
#include <i2c_driver.hpp>
#include <i2c_tiny_usb.hpp>
#include <esp_log.h>

using namespace Lunabotics::ESP32;
using namespace Lunabotics::Common;

extern "C" void app_main(void) {
    Boards::FeatherS3TFT board;
    board.enableI2C();

    Protocols::I2CPort port;
    board.I2C(0, port);

    // Both objects must outlive app_main. The bridge also holds about 1.2 KB of buffers,
    // which is too much for the 3.5 KB main task stack.
    static Drivers::I2CBus bus(port);
    static Drivers::I2CTinyUSB bridge(&bus);

    if (!bridge.init()) {
        ESP_LOGE("main", "i2c-tiny-usb: %s", esp_err_to_name(bridge.getErr()));
        return;
    }
    // The TinyUSB task now serves the host, so app_main may return.
}
```

### From the Test Console
`firmware/main/main.cpp` keeps the `SerialIO` console and starts the bridge on the `usbi2c` command. Both use the USB port, so the console releases its driver before the bridge takes over:
```cpp
// Before the command loop. Construction does not touch USB.
static Drivers::I2CTinyUSB usb_bridge(&i2cBus0);

// In the command loop:
if (ioMsg == "usbi2c") {
    // The bridge takes the USB PHY from the USB Serial/JTAG console until reset.
    Terminal.serial_out("Switching USB to the i2c-tiny-usb bridge. Press RESET to return to this console.\n");
    vTaskDelay(pdMS_TO_TICKS(100));
    Terminal.deinit();
    if (usb_bridge.init()) {
        while (true) { vTaskDelay(portMAX_DELAY); }
    }
    Terminal.init();
    Terminal.serial_out("i2c-tiny-usb [INIT FAIL] " + std::string(esp_err_to_name(usb_bridge.getErr())) + "\n");
}
```
After a reset, the console comes back and `idf.py flash` works normally.

### Configuration
`I2CTinyUSBConfig` sets the USB identity and the bus limits:
```cpp
Drivers::I2CTinyUSBConfig config;
config.product = "Rover I2C bridge"; // init() copies the strings, up to 31 characters each.
config.serial = "rover-01";          // nullptr uses the base MAC address.
config.max_frequency_hz = 100000;    // Stay at 100 kHz even if the host asks for 400 kHz.
static Drivers::I2CTinyUSB bridge(&bus, config);
```

| Field | Default | Meaning |
|-------|---------|---------|
| `vid`, `pid` | `0x303A`, `0x4020` | Espressif vendor ID, and esp_tinyusb's product ID for a vendor-class device |
| `bcd_device` | `0x0100` | Linux reports it as the adapter version (`version 1.00`) |
| `manufacturer` | `"Engineering ILS Foundation"` | USB manufacturer string |
| `product` | `"OSR V3 i2c-tiny-usb bridge"` | USB product string |
| `serial` | `nullptr` | Base MAC address in hex, for example `240AC4123456` |
| `max_frequency_hz` | `400000` | Highest SCL frequency the host may select |
| `timeout_ms` | `100` | Timeout of one I2C transaction |

| `I2CTinyUSB` method | Description |
|---------------------|-------------|
| `I2CTinyUSB(I2CBus* bus, const I2CTinyUSBConfig& config = I2CTinyUSBConfig())` | Stores the bus and the configuration. Does not touch USB |
| `bool init()` | Installs TinyUSB and connects to the host. On failure, check `getErr()` |
| `void deinit()` | Disconnects and uninstalls TinyUSB. The destructor also calls it |
| `bool isInitialized() const` | `init()` succeeded |
| `bool isMounted() const` | The host has configured the device |
| `esp_err_t getErr() const` | Last `init()` or `deinit()` error |

### How It Works
```
i2cget / i2cset / i2cdetect
        |  /dev/i2c-N
Linux i2c-tiny-usb driver
        |  vendor control transfers on endpoint 0
TinyUSB task: tud_vendor_control_xfer_cb()
        |
I2CTinyUSBBridge     collects the messages of a transaction, keeps the status
        |
I2CBusTransport      START / WRITE / READ / STOP operations
        |
I2CBus (i2c_master)  SDA / SCL
```

| `bRequest` | Command | Direction | Behavior |
|------------|---------|-----------|----------|
| 0 | `CMD_ECHO` | IN | Returns `wValue`, 2 bytes little-endian |
| 1 | `CMD_GET_FUNC` | IN | Returns `0x0EFF0009`: plain I2C and SMBus emulation |
| 2 | `CMD_SET_DELAY` | OUT | Maps the bit-bang half period in `wValue` (µs) to 100 or 400 kHz, limited by `max_frequency_hz` |
| 3 | `CMD_GET_STATUS` | IN | Status of the last message: 0 idle, 1 ACK, 2 NAK |
| 4 to 7 | `CMD_I2C_IO`, plus `BEGIN` (1) and `END` (2) | OUT write, IN read | One I2C message. `wValue` holds the Linux `I2C_M_RD` flag, `wIndex` the 7-bit address, and the data stage the payload |

Linux sends each message of a transaction as its own request, and reads the status after every one. The bridge collects the messages and runs them as one transaction when the host sends `END` or asks for read data. This keeps the repeated starts. `i2cget -y N 0x6b 0x0f` looks like this:

| Step | Request | `bRequest` | `wValue` | `wIndex` | Data | Bridge |
|------|---------|------------|----------|----------|------|--------|
| 1 | OUT `CMD_I2C_IO` + `BEGIN` | 5 | 0 | `0x6B` | `0F` | Collects the write and reports ACK |
| 2 | IN `CMD_GET_STATUS` | 3 | 0 | 0 | `01` | ACK |
| 3 | IN `CMD_I2C_IO` + `END` | 6 | 1 | `0x6B` | `68` | Runs `S D6 0F S D7 [68] N P` and returns the byte |
| 4 | IN `CMD_GET_STATUS` | 3 | 0 | 0 | `01` | ACK |

`S` is a start or repeated start, `N` the NACK of the last byte read, and `P` the stop. `i2cdetect` sends one empty write per address (`S <addr+W> P`). It uses a 1-byte read instead at 0x30 to 0x37 and 0x50 to 0x5F.

The bridge has no USB dependency. `tud_vendor_control_xfer_cb()` passes it the SETUP fields and the data stage. The same calls drive it directly, as the host tests do:
```cpp
using Bridge = Drivers::I2CTinyUSBBridge;
static Drivers::I2CBusTransport transport(&bus);
static Bridge bridge(transport);

// 1. OUT I2C_IO|BEGIN to 0x6B with data {0x0F}: the write is collected, not sent yet.
const uint8_t reg = 0x0F;
bridge.controlToDevice({Bridge::CMD_I2C_IO | Bridge::CMD_I2C_IO_BEGIN, 0, 0x6B, 1}, &reg);

// 2. IN I2C_IO|END with I2C_M_RD: runs S D6 0F S D7 [read] N P as one transaction.
uint8_t data[Bridge::MAX_DATA];
size_t len = 0;
bridge.controlToHost({Bridge::CMD_I2C_IO | Bridge::CMD_I2C_IO_END, Bridge::M_RD, 0x6B, 1}, data, sizeof(data), len);

// 3. IN GET_STATUS: the host checks the status after every message.
if (bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK) {
    ESP_LOGI("main", "WHO_AM_I 0x%02x", data[0]); // 0x68 on the LSM9DS1
}
```

### Linux Host
The `i2c-tiny-usb` driver only knows the IDs of the original adapter. Add this bridge's IDs to it. They stay until the module is unloaded, and the driver also binds to a bridge that is already connected.
```bash
sudo modprobe i2c-tiny-usb
sudo sh -c "echo 0x303a 0x4020 > /sys/bus/usb/drivers/i2c-tiny-usb/new_id"
```

Then find the bus number `N` and use i2c-tools. The number can change when the board is plugged in again.
```bash
sudo dmesg | grep i2c-tiny-usb      # "version 1.00 found at bus ...", "connected i2c-tiny-usb device"
i2cdetect -l | grep tiny            # i2c-N   i2c   i2c-tiny-usb at bus 001 device 016
sudo i2cdetect -y N
sudo i2cget -y N 0x6b 0x0f          # LSM9DS1 WHO_AM_I: 0x68
sudo i2ctransfer -y N w1@0x6b 0x0f r1
```

The host picks the bus speed with the module's `delay` parameter: 10 µs (default) gives 100 kHz, and `delay=1` gives 400 kHz. Reloading the module drops the added IDs, so add them again:
```bash
sudo modprobe -r i2c-tiny-usb
sudo modprobe i2c-tiny-usb delay=1
sudo sh -c "echo 0x303a 0x4020 > /sys/bus/usb/drivers/i2c-tiny-usb/new_id"
```

### Limits
| Limit | Value | Source |
|-------|-------|--------|
| Targets per transaction | 1 | Zephyr sample |
| Messages per transaction | 8 | Zephyr sample |
| Data per transaction | 256 bytes | Zephyr sample |
| Controller operations per transaction | 8: 2 per write message, 3 or 4 per read message, 1 for the stop | ESP32-S3 command registers |
| Bytes sent in a transaction with several messages | 32: address bytes plus write data | ESP32-S3 transmit FIFO. i2c_master refills it only inside one message |
| Read message | Last in the transaction | i2c_master collects one read per transaction |

These limits fit `i2cdetect`, `i2cget`, `i2cset`, `i2cdump` (byte, word and `i` block modes), single writes of up to 256 bytes such as EEPROM page writes, and short `i2ctransfer` sequences. A transaction outside the limits is not split, because splitting would lose its repeated starts. The host gets a NAK (`No such device or address`), and a warning is logged on UART0.

### Usage Notes
- `init()` moves the internal USB PHY from USB Serial/JTAG to USB-OTG. `SerialIO` stops working until reset. `ESP_LOG` output continues on UART0. If an application starts the bridge at boot, flash it from the ROM download mode: hold BOOT and press RESET.
- Only one instance can be initialized, because there is one USB-OTG controller. A second `init()` fails with `ESP_ERR_INVALID_STATE`.
- The bus must outlive the bridge. Use static storage for both.
- The bus stays shared. The other drivers keep working, and i2c_master serializes the transactions. Host writes can change the configuration of sensors that the firmware also uses.
- I2C runs in the esp_tinyusb task (priority 5). Each transaction blocks that task for at most `timeout_ms`.
- A NAK on a write that waits for the following read is reported with the read, as in the Zephyr sample.
- The component defines the TinyUSB callbacks `tud_vendor_control_xfer_cb()` and `usbd_app_driver_get_cb()`. Do not combine it with another TinyUSB vendor class in the same application.

### Hardware Validation
1. Build and flash with ESP-IDF v5.4.3, target `esp32s3`. Open the console and run `scan` to list the targets on I2C0.
2. On the Linux host, run the two commands under Linux Host to add the IDs.
3. Run `usbi2c`. The USB Serial/JTAG device disappears, and `lsusb` shows `303a:4020`.
4. `dmesg` shows `connected i2c-tiny-usb device`. `i2cdetect -l` lists the new bus `N`.
5. `sudo i2cdetect -y N` matches the `scan` output.
6. `sudo i2cget -y N 0x6b 0x0f` returns `0x68`, and `sudo i2cget -y N 0x1e 0x0f` returns `0x3d` (LSM9DS1).
7. `sudo i2ctransfer -y N w1@0x6b 0x0f r1` returns `0x68` from one transaction with a repeated start.
8. Write and read back: `sudo i2cset -y N 0x6b 0x07 0x5a`, then `sudo i2cget -y N 0x6b 0x07` returns `0x5a`. The LSM9DS1 is reset by its `init()` on the next boot.
9. Press RESET and confirm that the console comes back.

### Functionality
- [x] Zephyr protocol port: `ECHO`, `GET_FUNC`, `SET_DELAY`, `GET_STATUS`, and `I2C_IO` with `BEGIN`/`END`
- [x] Repeated starts kept through i2c_master defined operations
- [x] Compilation (ESP-IDF v5.4.3, ESP32-S3)
- [x] Host regression tests: protocol, emulated I2C bus, USB control stages and descriptors
- [x] Interactive hardware harness (`usbi2c`)
- [ ] Hardware validation against a Linux host
- [ ] Maintainer code review after hardware validation

Requires
1. `i2c_driver` for the shared `I2CBus`.
2. `espressif/esp_tinyusb`, declared in `idf_component.yml`, for the USB device stack.
3. ESP-IDF 5.4.1 or later for `i2c_master_execute_defined_operations()`.

### References
- [Zephyr i2c_tiny_usb sample](https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/subsys/usb/i2c_tiny_usb)
- [I2C-Tiny-USB project](https://github.com/harbaum/I2C-Tiny-USB)
- [Linux i2c-tiny-usb driver](https://elixir.bootlin.com/linux/latest/source/drivers/i2c/busses/i2c-tiny-usb.c)
- [Dynamic USB device IDs (new_id)](https://lwn.net/Articles/160944/)
- [esp_tinyusb component](https://components.espressif.com/components/espressif/esp_tinyusb)
- [ESP-IDF I2C master driver, custom transactions](https://docs.espressif.com/projects/esp-idf/en/v5.4.3/esp32s3/api-reference/peripherals/i2c.html)
