#include "i2c_tools.hpp"

// Helper lambda or function to format hex cleanly
auto to_hex = [](int value) -> std::string {
    char buffer[5];
    snprintf(buffer, sizeof(buffer), "%02x", value);
    std::string hex = std::string(buffer);
    return hex;
};

auto to_0x_hex = [](int value) -> std::string {
    char buffer[5];
    snprintf(buffer, sizeof(buffer), "%02x", value);
    std::string hex = "0x" + std::string(buffer);
    return hex;
};

void i2c_status(SerialIO &terminal, I2CBus &bus) {
     if (bus.isInitialized()) {
                terminal.serial_out("I2C Bus 0 Status: [OK] (Initialized)\n");
            } else {
                // If failed, print the error code for debugging
                std::string errName = esp_err_to_name(bus.getErr());
                terminal.serial_out("I2C Bus 0 Status: [FAIL] Error Code: " + errName + "\n");
            }
}

void i2c_device_status(SerialIO &terminal, I2CDevice &device) {
    // 1. Safe Bus Port Retrieval
    std::string busPort = "?";
    if (device.getBus() != nullptr) {
        busPort = std::to_string(device.getBus()->getI2CPort().i2c_port);
    }

    if (device.isInitialized()) {
        terminal.serial_out("I2C Bus " + busPort + " Device: " + to_hex(device.getAddress()) + " Status: [OK] (Initialized)\n");
    } else {
        std::string errName = esp_err_to_name(device.getErr());
        terminal.serial_out("I2C Bus " + busPort + " Device: " + to_hex(device.getAddress()) + " Status: [FAIL] Error Code: " +  errName + "\n");
    }   
}

void i2c_device_read(SerialIO &terminal, I2CDevice &device, const uint8_t* addresses, const uint16_t num_addresses) {
    
    // 2. Optimization: No array needed. Just 2 bytes of stack memory.
    uint16_t temp_data = 0; 

    for (size_t i = 0; i < num_addresses; i++) {
        if (device.readRegister(addresses[i], temp_data)) {
            // 3. Fix: Added "\r\n" for clean newlines
            terminal.serial_out("Register: " + to_hex(addresses[i]) + " : " + to_0x_hex(temp_data) + "\r\n");
        } else {
            terminal.serial_out("Register: " + to_hex(addresses[i]) + " : 0xFF [ERR]\r\n");
        }
    }
}

void i2c_scan(SerialIO &terminal, I2CBus &bus) {
    if (bus.isInitialized()) {
        terminal.serial_out("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
        uint8_t address;
        // Iterate through all 128 possible addresses
        for (int i = 0; i < 128; i += 16) {
            // Fix: Use the helper to print the Row Header (00:, 10:, etc.)
            terminal.serial_out(to_hex(i) + ": ");

            for (int j = 0; j < 16; j++) {
                // fflush is usually for stdout/printf, might not affect your terminal class, 
                // but harmless to keep if you are using underlying stdio.
                fflush(stdout); 
                
                address = i + j;

                // Skip reserved addresses (0x00-0x02, 0x78-0x7F) if you want to be strict,
                // but probing them is usually fine.
                esp_err_t ret = i2c_master_probe(bus.getI2CBus(), address, 50);

                if (ret == ESP_OK) {
                    // Fix: Print the address in Hex, not Decimal
                    terminal.serial_out(to_hex(address) + " ");
                } else if (ret == ESP_ERR_TIMEOUT) {
                    terminal.serial_out("UU "); // Timeout is usually just NACK ("--")
                } else {
                    terminal.serial_out("-- ");
                }
            }
            terminal.serial_out("\r\n");
        }
        }
        else {
            std::string errName = esp_err_to_name(bus.getErr());
            terminal.serial_out("I2C Bus 0 Status: [FAIL] Error Code: " + errName + "\n");
    }
    return;

}



void i2c_dump(SerialIO &terminal, I2CBus &bus, uint8_t chip_address, int size) {
    if (!bus.isInitialized()) {
        terminal.serial_out("I2C Bus not initialized.\n");
        return;
    }

    // 1. Setup a temporary device config for the target address
    i2c_master_dev_handle_t dev_handle = nullptr;
    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = chip_address,
        .scl_speed_hz = bus.getI2CPort().frequency,
    };

    // 2. Add device to the bus temporarily
    esp_err_t err = i2c_master_bus_add_device(bus.getI2CBus(), &dev_conf, &dev_handle);
    if (err != ESP_OK) {
        std::string errName = esp_err_to_name(err);
        terminal.serial_out("Failed to add temporary device for dumping. Error: " + errName + "\n");
        return;
    }

    // 3. Print Header
    terminal.serial_out("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef\r\n");

    // 4. Iterate through register addresses (0x00 to 0x7F)
    // Note: Standard 'i2cdump' goes to 0xFF, but your snippet did 0x7F (128). 
    // Change loop to 'i < 256' if you want the full range.
    uint8_t data[4];
    int32_t block[16]; // Stores values for the ASCII column

    for (int i = 0; i < 128; i += 16) {
        // Print Row Label (e.g., "00: ")
        terminal.serial_out(to_hex(i) + ": ");

        // Loop through the 16 bytes in this row
        for (int j = 0; j < 16; j += size) {
            uint8_t reg_addr = i + j;
            
            // Perform the Read: Write 1 byte (Reg Addr) -> Read 'size' bytes
            esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, size, 50);

            if (ret == ESP_OK) {
                for (int k = 0; k < size; k++) {
                    terminal.serial_out(to_hex(data[k]) + " ");
                    block[j + k] = data[k];
                }
            } else {
                // Read failed
                for (int k = 0; k < size; k++) {
                    terminal.serial_out("XX ");
                    block[j + k] = -1; // Mark as invalid
                }
            }
        }

        // Print the ASCII representation on the right
        terminal.serial_out("   ");
        for (int k = 0; k < 16; k++) {
            if (block[k] < 0) {
                terminal.serial_out("X");
            } else {
                uint8_t val = (uint8_t)block[k];
                // Check if printable ASCII (32-126)
                if (val >= 32 && val < 127) {
                    char buf[2] = {(char)val, 0};
                    terminal.serial_out(std::string(buf));
                } else {
                    terminal.serial_out(".");
                }
            }
        }
        terminal.serial_out("\r\n");
    }

    // 5. Cleanup: Remove the device handle so we don't leak memory or clutter the bus driver
    i2c_master_bus_rm_device(dev_handle);
}