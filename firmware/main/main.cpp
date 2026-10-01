#include <stdio.h>
#include <common/drivers/FakeIMU.hpp>
#include <SerialIO.hpp>
#include <i2c_driver.hpp>
#include <i2c_tools.hpp>
#include <Max1704x_test.hpp>
#include <Max1704x.hpp>
#include <ina3221.hpp>
#include <ina3221_test.hpp>
#include <lsm9ds1_test.hpp>
#include <Neopixel.hpp>
#include <LED.hpp>
#include <Platforms.hpp>
#include <driver/gpio.h>
#include <cerrno>
#include <cstdlib>
#include <freertos/task.h>
#include <cstring>
#include <cstdint>


using namespace Lunabotics::Common::Sensors;
using namespace Lunabotics::Common::DataTypes;
using namespace Lunabotics::ESP32;
using namespace Lunabotics::Common;
using SI = SensorInterface;
#define TFT_I2C_POWER_GPIO 21

void build_data_string(LSM9DS1Data &data, imuStringData &str_data,  std::string &ioMsg);
void imu_data_output(std::string &ioMsg, LSM9DS1 &imu);
void interactive_loop(std::string &ioMsg, SerialIO &Terminal,I2CBus &i2cBus0, LED &red_led, Max1704x &max1704x,INA3221 &ina3221,Neopixel &neopixel,Drivers::LSM9DS1 &imu);


/**
 * @brief IMU Data Output:
 *  
 * */ 
struct imuStringData {
    char ax[9], ay[9], az[9], gx[9], gy[9], gz[9], mx[9], my[9], mz[9];
    char time[17];
};
 
static void to_hex(uint64_t v, char *out, int n) {
    for (int i = n - 1; i >= 0; --i, v >>= 4) out[i] = "0123456789ABCDEF"[v & 0xF];
    out[n] = '\0';
}
 
static void f32_hex(float f, char *out) {
    uint32_t b;
    std::memcpy(&b, &f, 4);
    to_hex(b, out, 8);
}
 
// Frame: "$IMU\n" + 9x "XXXXXXXX\n" + "time16hex\n" + "#END" = 107 chars
// Heap-free as long as ioMsg.capacity() >= 107 (call ioMsg.reserve(128) once at init).
void build_data_string(LSM9DS1Data &data, imuStringData &s, std::string &ioMsg) {
    using namespace Units;
    constexpr auto MPS2 = meters / squared(seconds);
    constexpr auto RADPS = radians / seconds;
    constexpr auto UT = micro(tesla);
 
    f32_hex(data.acceleration.a_x.in<float>(MPS2), s.ax);
    f32_hex(data.acceleration.a_y.in<float>(MPS2), s.ay);
    f32_hex(data.acceleration.a_z.in<float>(MPS2), s.az);
    f32_hex(data.angular_velocity.x.in<float>(RADPS), s.gx);
    f32_hex(data.angular_velocity.y.in<float>(RADPS), s.gy);
    f32_hex(data.angular_velocity.z.in<float>(RADPS), s.gz);
    f32_hex(data.magnetic_field.x.in<float>(UT), s.mx);
    f32_hex(data.magnetic_field.y.in<float>(UT), s.my);
    f32_hex(data.magnetic_field.z.in<float>(UT), s.mz);
    to_hex(std::chrono::duration_cast<std::chrono::nanoseconds>(
               data.timestamp.time_since_epoch()).count(), s.time, 16);
 
    ioMsg.resize(107);  // no alloc if capacity already >= 107
    char *p = &ioMsg[0];
    std::memcpy(p, "$IMU\n", 5); p += 5;
    for (const char *f : {s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.mx, s.my, s.mz}) {
        std::memcpy(p, f, 8); p[8] = '\n'; p += 9;
    }
    std::memcpy(p, s.time, 16); p[16] = '\n'; p += 17;
    std::memcpy(p, "#END", 4);
}




void interactive_loop(
    std::string &ioMsg, 
    SerialIO &Terminal,
    I2CBus &i2cBus0, 
    LED &red_led, 
    Max1704x &max1704x,
    INA3221 &ina3221,
    Neopixel &neopixel,
    Drivers::LSM9DS1 &imu) 
    {

    while (true) {
        ioMsg = "Test I/O > 'check', 'scan', 'dump', 'checkread', 'read', 'imu', 'imuinit', 'blink', 'stopblink', or 'q': \n";
        Terminal.serial_out(ioMsg);
        ioMsg = "Input: ";
        ioMsg = Terminal.serial_in(ioMsg);

        if (ioMsg == "check") {
            i2c_status(Terminal, i2cBus0);
            i2c_device_status(Terminal, max1704x);
        }
        else if (ioMsg == "scan") {
            i2c_scan(Terminal, i2cBus0);
        }
        else if (ioMsg == "dump" || ioMsg.compare(0, 5, "dump ") == 0) {
            // Default to MAX17048
            uint8_t targetAddr = 0x36;

            if (ioMsg.size() > 4) {
                std::string arg = ioMsg.substr(5);
                
                char* endPtr;
                // Accept hexadecimal or decimal 7-bit device addresses.
                errno = 0;
                unsigned long val = strtoul(arg.c_str(), &endPtr, 0);

                // check if conversion failed:
                // 1. endPtr == arg.c_str() -> No digits found
                // 2. *endPtr != '\0'       -> Junk characters at end (e.g. "0x36xyz")
                // 3. Overflow or an address outside the 7-bit range
                if (endPtr == arg.c_str() || *endPtr != '\0' || errno == ERANGE || val > 0x7F) {
                    Terminal.serial_out("Invalid address. Usage: dump <hex|dec>\n");
                    continue;
                }
                
                targetAddr = static_cast<uint8_t>(val);
            }

            i2c_dump(Terminal, i2cBus0, targetAddr, 1);
        } else if (ioMsg == "checkread") {
            uint8_t addresses[16] = {0x02, 0x04, 0x06, 0x08, 0x0C, 0x14, 0x16, 0x18, 0x1A};
            uint8_t numAddresses = 9;
            i2c_device_read(Terminal, max1704x, addresses, numAddresses);
        } else if (ioMsg == "read") {
            max1704x_test_data(Terminal, max1704x);
            ina3221_test_data(Terminal, ina3221);
        } else if (ioMsg == "imu") {
            lsm9ds1_test_data(Terminal, imu);
        } else if (ioMsg == "imuinit") {
            Terminal.serial_out(imu.init() ? "LSM9DS1 [INIT OK]\n" :
                "LSM9DS1 [INIT FAIL] " + std::string(esp_err_to_name(imu.getErr())) + "\n");
        } else if (ioMsg == "blink") {
            red_led.blink(500);
            neopixel.blink(500);
        } else if (ioMsg == "stopblink") {
            red_led.stopBlink();
            neopixel.stopBlink();
        }
        else if (ioMsg == "q") {
            Terminal.serial_out("[TEST END] Quitting...\n");
            break; // Exit the loop
        }
        else {
            Terminal.serial_out("Echo: " + ioMsg + "\n");
        }
        
        // Small delay to keep the terminal responsive but not spammy
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return;
}





void app_main(void) {
    // 1. Setup Hardware
    Boards::FeatherS3TFT board;
    board.enableI2C();
   
    Protocols::I2CPort i2cPort0;
    // Get I2C Port 0 from the board (Port 0 exists on FeatherS3)
    board.I2C(0, i2cPort0);

    // Attempt to initialize the driver:
    Drivers::I2CBus i2cBus0(i2cPort0);
    
    // Attempt to initialize the max1704x:
    // Drivers::I2CDevice max1704x(0x36, &i2cBus0); 
    Max1704x max1704x(0x36,&i2cBus0);
    max1704x.init();

    INA3221 ina3221(0x40, &i2cBus0);
    ina3221.init();

    LED red_led(board.led_pwr_pin);
    red_led.init();
    
    // 2. Setup Terminal
    SerialIO Terminal;
    Terminal.init();

    Drivers::LSM9DS1 imu(0x6B, &i2cBus0);
    if (!imu.init()) {
        Terminal.serial_out("LSM9DS1 [INIT FAIL] " + std::string(esp_err_to_name(imu.getErr())) + "\n");
    }
    
    static constexpr Drivers::NeopixelConfig NeoPixelConfig {
        .data = {.gpio_pin = 33},
        .pixel_count = 1,
        .spi_host = SPI2_HOST,
        .with_dma = true,
        .invert_out = false,
        .has_power_pin = true,
        .power = {.gpio_pin = 34},
        .power_active_high = true
    };
    Drivers::Neopixel neopixel(NeoPixelConfig);

    // Run Interactive Loop:
    std::string ioMsg;
    ioMsg.reserve(512);
    ioMsg = "[TEST START] System Ready. \n";
    Terminal.serial_out(ioMsg);
    
    interactive_loop(ioMsg, i2cBus0, Terminal, red_led, max1704x, ina3221, neopixel, imu);
    imu_data_pipeline()
   

    // 4. Cleanup
    Terminal.deinit();
    
    // In a real RTOS app, app_main should not return, but for a test, this is fine.
    // Ideally, delete the tasks or loop forever here.
    while(1) { vTaskDelay(1000); }
}


