#include "interactive_shell.hpp"

MainTerminal::run() {
    
    ioMsg.reserve(512);
    ioMsg = "[TEST START] System Ready. \n";
    Terminal.serial_out(ioMsg);
    
    while (true) {
        ioMsg = "Test I/O > 'check', 'scan', 'dump', 'checkread', 'read', 'blink', 'stopblink', or 'q' to quit: \n";
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
        else if (ioMsg.size() >= 4 && ioMsg.substr(0, 4) == "dump") {
            // Default to MAX17048
            uint8_t targetAddr = 0x36;

            if (ioMsg.size() > 5) {
                std::string arg = ioMsg.substr(5);
                
                // --- REPLACEMENT LOGIC START ---
                char* endPtr;
                // strtoul(string, end_pointer, base 0 for auto-detect)
                unsigned long val = strtoul(arg.c_str(), &endPtr, 0);

                // check if conversion failed:
                // 1. endPtr == arg.c_str() -> No digits found
                // 2. *endPtr != '\0'       -> Junk characters at end (e.g. "0x36xyz")
                // 3. val > 255             -> Address too big for I2C
                if (endPtr == arg.c_str() || *endPtr != '\0' || val > 255) {
                    Terminal.serial_out("Invalid address. Usage: dump <hex|dec>\n");
                    return; // Changed from continue if inside a void function
                }
                
                targetAddr = static_cast<uint8_t>(val);
                // --- REPLACEMENT LOGIC END ---
            }

            i2c_dump(Terminal, i2cBus0, targetAddr, 1);
        } else if (ioMsg == "checkread") {
            uint8_t addresses[16] = {0x02, 0x04, 0x06, 0x08, 0x0C, 0x14, 0x16, 0x18, 0x1A};
            uint8_t numAddresses = 9;
            i2c_device_read(Terminal, max1704x, addresses, numAddresses);
        } else if (ioMsg == "read") {
            max1704x_test_data(Terminal, max1704x);
            ina3221_test_data(Terminal, ina3221);
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
        
        // Small delay to keep the terminal respo
}
}