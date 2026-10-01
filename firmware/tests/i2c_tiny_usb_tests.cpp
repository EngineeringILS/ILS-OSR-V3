#include <i2c_tiny_usb.hpp>
#include <i2c_tiny_usb_bridge.hpp>
#include <i2c_tiny_usb_transport.hpp>
#include <tinyusb.h>
#include <cstring>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>

using namespace Lunabotics::Common;
using namespace Lunabotics::ESP32::Drivers;
using Bridge = I2CTinyUSBBridge;

static_assert(!std::is_copy_constructible<I2CTinyUSBBridge>::value);
static_assert(!std::is_copy_constructible<I2CBusTransport>::value);
static_assert(!std::is_copy_constructible<I2CTinyUSB>::value);

#define CHECK(condition) do { if (!(condition)) { \
    std::cerr << "Failed at line " << __LINE__ << ": " << #condition << '\n'; \
    return 1; } } while (false)

// Emulated bus of register-pointer targets, the model of most sensors and EEPROMs.
namespace Bus {
struct Target {
    bool present = false;
    uint8_t pointer = 0;
    uint8_t regs[256]{};
};
Target targets[128];

// Operations of the last transaction: S start, W write, A read with ACK, N read with NACK, P stop.
std::string shape;

void record(const std::string& token) {
    shape += (shape.empty() ? "" : " ") + token;
}

esp_err_t execute(const i2c_operation_job_t* ops, size_t count) {
    shape.clear();
    if (count > SOC_I2C_CMD_REG_NUM) {
        return ESP_ERR_INVALID_ARG;
    }
    Target* target = nullptr;
    bool reading = false;
    bool pointer_set = false;
    for (size_t i = 0; i < count; i++) {
        const i2c_operation_job_t& op = ops[i];
        switch (op.command) {
            case I2C_MASTER_CMD_START:
                record("S");
                target = nullptr;
                break;
            case I2C_MASTER_CMD_WRITE: {
                record("W" + std::to_string(op.write.total_bytes));
                if (!op.write.ack_check || op.write.total_bytes == 0) {
                    return ESP_ERR_INVALID_ARG;
                }
                size_t k = 0;
                if (target == nullptr) {
                    // The first byte after a start addresses the target.
                    target = &targets[op.write.data[0] >> 1];
                    if (!target->present) {
                        return ESP_ERR_INVALID_STATE;
                    }
                    reading = op.write.data[0] & 1;
                    pointer_set = false;
                    k = 1;
                }
                for (; k < op.write.total_bytes; k++) {
                    if (!pointer_set) {
                        target->pointer = op.write.data[k];
                        pointer_set = true;
                    } else {
                        target->regs[target->pointer++] = op.write.data[k];
                    }
                }
                break;
            }
            case I2C_MASTER_CMD_READ:
                record((op.read.ack_value == I2C_ACK_VAL ? "A" : "N") + std::to_string(op.read.total_bytes));
                // The driver requires a NACK before the stop.
                if (target == nullptr || !reading ||
                    (i + 1 < count && ops[i + 1].command == I2C_MASTER_CMD_STOP && op.read.ack_value != I2C_NACK_VAL)) {
                    return ESP_ERR_INVALID_ARG;
                }
                for (size_t k = 0; k < op.read.total_bytes; k++) {
                    op.read.data[k] = target->regs[target->pointer++];
                }
                break;
            case I2C_MASTER_CMD_STOP:
                record("P");
                break;
        }
    }
    return ESP_OK;
}
}

Bridge::Setup io(uint8_t flags, uint16_t address, uint16_t len, bool read) {
    return {
        static_cast<uint8_t>(Bridge::CMD_I2C_IO | flags),
        static_cast<uint16_t>(read ? Bridge::M_RD : 0),
        address,
        len
    };
}

constexpr uint8_t BEGIN = Bridge::CMD_I2C_IO_BEGIN;
constexpr uint8_t END = Bridge::CMD_I2C_IO_END;

// A vendor request with interface recipient, as sent by the Linux i2c-tiny-usb driver.
tusb_control_request_t request(bool in, uint8_t command, uint16_t value, uint16_t index, uint16_t length) {
    tusb_control_request_t setup{};
    setup.bmRequestType_bit.recipient = 1;
    setup.bmRequestType_bit.type = 2;
    setup.bmRequestType_bit.direction = in ? TUSB_DIR_IN : TUSB_DIR_OUT;
    setup.bRequest = command;
    setup.wValue = value;
    setup.wIndex = index;
    setup.wLength = length;
    return setup;
}

// Runs every stage of a control transfer, supplying OUT data when TinyUSB asks for it.
bool control(const tusb_control_request_t& setup, const uint8_t* out = nullptr) {
    if (!tud_vendor_control_xfer_cb(0, CONTROL_STAGE_SETUP, &setup)) {
        return false;
    }
    if (setup.wLength != 0 && out != nullptr) {
        std::memcpy(TestUSB::xfer_buffer, out, setup.wLength);
    }
    if (setup.wLength != 0 && !tud_vendor_control_xfer_cb(0, CONTROL_STAGE_DATA, &setup)) {
        return false;
    }
    return tud_vendor_control_xfer_cb(0, CONTROL_STAGE_ACK, &setup);
}

int main() {
    TestI2C::execute = Bus::execute;
    Bus::targets[0x50].present = true;
    Protocols::I2CPort port{42, 41, 0, 100000, true};
    {
        I2CBus bus(port);
        I2CBusTransport transport(&bus);
        Bridge bridge(transport);
        uint8_t in[Bridge::MAX_DATA];
        size_t len = 0;

        // Adapter queries of the Linux driver.
        CHECK(bridge.getStatus() == Bridge::STATUS_IDLE);
        CHECK(bridge.controlToHost({Bridge::CMD_ECHO, 0x1234, 0, 2}, in, sizeof(in), len));
        CHECK(len == 2 && in[0] == 0x34 && in[1] == 0x12);
        CHECK(bridge.controlToHost({Bridge::CMD_ECHO, 0x1234, 0, 1}, in, sizeof(in), len));
        CHECK(len == 1);
        CHECK(bridge.controlToHost({Bridge::CMD_GET_FUNC, 0, 0, 4}, in, sizeof(in), len));
        CHECK(len == 4 && in[0] == 0x09 && in[1] == 0x00 && in[2] == 0xFF && in[3] == 0x0E);
        CHECK(bridge.controlToHost({Bridge::CMD_GET_STATUS, 0, 0, 1}, in, sizeof(in), len));
        CHECK(len == 1 && in[0] == Bridge::STATUS_IDLE);
        CHECK(!bridge.controlToHost({9, 0, 0, 1}, in, sizeof(in), len));
        CHECK(!Bridge::acceptsToDevice({Bridge::CMD_ECHO, 0, 0, 0}));
        CHECK(!bridge.controlToDevice({Bridge::CMD_GET_STATUS, 0, 0, 0}, nullptr));

        // SET_DELAY selects the closest standard speed for the next transaction.
        CHECK(transport.getFrequency() == 100000);
        CHECK(Bridge::acceptsToDevice({Bridge::CMD_SET_DELAY, 1, 0, 0}));
        CHECK(bridge.controlToDevice({Bridge::CMD_SET_DELAY, 2, 0, 0}, nullptr));
        CHECK(transport.getFrequency() == 100000);
        CHECK(bridge.controlToDevice({Bridge::CMD_SET_DELAY, 0, 0, 0}, nullptr));
        CHECK(transport.getFrequency() == 400000);
        CHECK(TestI2C::devices == 0);

        // i2cset: one message carries the register and the value.
        const uint8_t set[] = {0x10, 0xAB};
        CHECK(Bridge::acceptsToDevice(io(BEGIN | END, 0x50, 2, false)));
        CHECK(bridge.controlToDevice(io(BEGIN | END, 0x50, 2, false), set));
        CHECK(Bus::shape == "S W3 P");
        CHECK(Bus::targets[0x50].regs[0x10] == 0xAB);
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK);
        // One registration without an address serves every target.
        CHECK(TestI2C::devices == 1);
        CHECK(TestI2C::device_address == I2C_DEVICE_ADDRESS_NOT_USED);
        CHECK(TestI2C::device_speed == 400000);

        // i2cget: the register write waits for the read and joins it with a repeated start.
        unsigned transactions = TestI2C::transactions;
        const uint8_t reg = 0x10;
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 1, false), &reg));
        CHECK(TestI2C::transactions == transactions);
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK);
        CHECK(bridge.controlToHost(io(END, 0x50, 1, true), in, sizeof(in), len));
        CHECK(TestI2C::transactions == transactions + 1);
        CHECK(Bus::shape == "S W2 S W1 N1 P");
        CHECK(len == 1 && in[0] == 0xAB);
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK);

        // Block reads ACK every byte but the last.
        for (int i = 0; i < 256; i++) {
            Bus::targets[0x50].regs[i] = static_cast<uint8_t>(i);
        }
        const uint8_t zero = 0;
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 1, false), &zero));
        CHECK(bridge.controlToHost(io(END, 0x50, 200, true), in, sizeof(in), len));
        CHECK(Bus::shape == "S W2 S W1 A199 N1 P");
        CHECK(len == 200 && in[0] == 0 && in[100] == 100 && in[199] == 199);

        // A new speed replaces the registration from the next transaction.
        CHECK(bridge.controlToDevice({Bridge::CMD_SET_DELAY, 10, 0, 0}, nullptr));
        CHECK(bridge.controlToDevice(io(BEGIN | END, 0x50, 2, false), set));
        CHECK(TestI2C::devices == 1 && TestI2C::device_speed == 100000);
        // Removal is refused while another device is busy, so the old speed stays until it succeeds.
        TestI2C::refuse_removal = true;
        CHECK(bridge.controlToDevice({Bridge::CMD_SET_DELAY, 1, 0, 0}, nullptr));
        CHECK(bridge.controlToDevice(io(BEGIN | END, 0x50, 2, false), set));
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK);
        CHECK(TestI2C::devices == 1 && TestI2C::device_speed == 100000);
        TestI2C::refuse_removal = false;
        CHECK(bridge.controlToDevice(io(BEGIN | END, 0x50, 2, false), set));
        CHECK(TestI2C::devices == 1 && TestI2C::device_speed == 400000);

        // Absent targets NAK, and read data is zero-filled to the requested length.
        CHECK(bridge.controlToDevice(io(BEGIN | END, 0x51, 0, false), nullptr));
        CHECK(Bus::shape == "S W1");
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_NAK);
        in[0] = in[1] = 0xFF;
        CHECK(bridge.controlToHost(io(BEGIN | END, 0x51, 2, true), in, sizeof(in), len));
        CHECK(len == 2 && in[0] == 0 && in[1] == 0);
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_NAK);

        // i2cdetect: a quick write sends only the address.
        CHECK(bridge.controlToDevice(io(BEGIN | END, 0x50, 0, false), nullptr));
        CHECK(Bus::shape == "S W1 P");
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK);

        // A zero-length read clocks one discarded byte, so the target releases SDA before the stop.
        CHECK(bridge.controlToHost(io(BEGIN | END, 0x50, 0, true), in, sizeof(in), len));
        CHECK(len == 0 && Bus::shape == "S W1 N1 P");
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK);

        // A read without END ends the collected messages; the next message starts a new transaction.
        CHECK(bridge.controlToHost(io(BEGIN, 0x50, 1, true), in, sizeof(in), len));
        CHECK(Bus::shape == "S W1 N1 P");
        const uint8_t later[] = {0x20, 0x55};
        CHECK(bridge.controlToDevice(io(END, 0x50, 2, false), later));
        CHECK(Bus::shape == "S W3 P" && Bus::targets[0x50].regs[0x20] == 0x55);

        // BEGIN discards messages of a transaction the host abandoned.
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 1, false), &reg));
        CHECK(bridge.controlToDevice(io(BEGIN | END, 0x50, 2, false), later));
        CHECK(Bus::shape == "S W3 P");

        // Bridge limits: one target, eight messages, 256 data bytes, and matching directions.
        transactions = TestI2C::transactions;
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 1, false), &reg));
        in[0] = 0xFF;
        CHECK(bridge.controlToHost(io(END, 0x52, 1, true), in, sizeof(in), len));
        CHECK(len == 1 && in[0] == 0 && bridge.getStatus() == Bridge::STATUS_ADDRESS_NAK);
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 1, false), &reg));
        for (int i = 0; i < 7; i++) {
            CHECK(bridge.controlToDevice(io(0, 0x50, 1, false), &reg));
        }
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK);
        CHECK(bridge.controlToDevice(io(END, 0x50, 1, false), &reg));
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_NAK);
        const std::vector<uint8_t> block(200, 0x5A);
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 200, false), block.data()));
        CHECK(bridge.controlToDevice(io(END, 0x50, 100, false), block.data()));
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_NAK);
        CHECK(bridge.controlToDevice({Bridge::CMD_I2C_IO | BEGIN | END, Bridge::M_RD, 0x50, 1}, &reg));
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_NAK);
        CHECK(!bridge.controlToHost(io(BEGIN | END, 0x50, Bridge::MAX_DATA + 1, true), in, sizeof(in), len));
        CHECK(TestI2C::transactions == transactions);

        // Controller limits: eight operations per transaction and a 32-byte FIFO for several messages.
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 1, false), &reg));
        CHECK(bridge.controlToDevice(io(0, 0x50, 1, false), &reg));
        CHECK(bridge.controlToDevice(io(END, 0x50, 1, false), &reg));
        CHECK(Bus::shape == "S W2 S W2 S W2 P");
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 1, false), &reg));
        CHECK(bridge.controlToDevice(io(0, 0x50, 1, false), &reg));
        CHECK(bridge.controlToHost(io(END, 0x50, 1, true), in, sizeof(in), len));
        CHECK(Bus::shape == "S W2 S W2 S W1 N1 P");
        transactions = TestI2C::transactions;
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 1, false), &reg));
        CHECK(bridge.controlToDevice(io(0, 0x50, 1, false), &reg));
        CHECK(bridge.controlToHost(io(END, 0x50, 2, true), in, sizeof(in), len));
        CHECK(len == 2 && bridge.getStatus() == Bridge::STATUS_ADDRESS_NAK);
        CHECK(TestI2C::transactions == transactions && transport.getErr() == ESP_ERR_NOT_SUPPORTED);
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 31, false), block.data()));
        CHECK(bridge.controlToHost(io(END, 0x50, 1, true), in, sizeof(in), len));
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_NAK);
        CHECK(TestI2C::transactions == transactions);
        CHECK(bridge.controlToDevice(io(BEGIN, 0x50, 30, false), block.data()));
        CHECK(bridge.controlToHost(io(END, 0x50, 1, true), in, sizeof(in), len));
        CHECK(Bus::shape == "S W31 S W1 N1 P");
        CHECK(bridge.getStatus() == Bridge::STATUS_ADDRESS_ACK);
        // A single message may exceed the FIFO; the driver refills it.
        const std::vector<uint8_t> page(Bridge::MAX_DATA, 0xA5);
        CHECK(bridge.controlToDevice(io(BEGIN | END, 0x50, Bridge::MAX_DATA, false), page.data()));
        CHECK(Bus::shape == "S W257 P" && Bus::targets[0x50].regs[0x00] == 0xA5);

        // Direct transport checks.
        I2CTinyUSBMessage msgs[2];
        uint8_t byte = 0;
        msgs[0].data = &byte;
        msgs[0].len = 1;
        msgs[0].read = true;
        msgs[1].data = &byte;
        msgs[1].len = 1;
        CHECK(!transport.transfer(0x80, msgs, 1) && transport.getErr() == ESP_ERR_INVALID_ARG);
        CHECK(!transport.transfer(0x50, nullptr, 1) && transport.getErr() == ESP_ERR_INVALID_ARG);
        CHECK(!transport.transfer(0x50, msgs, 0) && transport.getErr() == ESP_ERR_INVALID_ARG);
        // The driver collects a single read, which must end the transaction.
        CHECK(!transport.transfer(0x50, msgs, 2) && transport.getErr() == ESP_ERR_NOT_SUPPORTED);
        msgs[0].data = nullptr;
        CHECK(!transport.transfer(0x50, msgs, 1) && transport.getErr() == ESP_ERR_INVALID_ARG);
        CHECK(!transport.setFrequency(0));
        // The byte clocked by a zero-length read stays inside the transport.
        uint8_t sentinel = 0x77;
        msgs[0].data = &sentinel;
        msgs[0].len = 0;
        Bus::targets[0x50].pointer = 0;
        Bus::targets[0x50].regs[0] = 0x11;
        CHECK(transport.transfer(0x50, msgs, 1) && Bus::shape == "S W1 N1 P");
        CHECK(sentinel == 0x77 && Bus::targets[0x50].pointer == 1);

        I2CBusTransport slow(&bus, 100000);
        CHECK(slow.setFrequency(400000) && slow.getFrequency() == 100000);

        Protocols::I2CPort missing{};
        I2CBus absent(missing);
        I2CBusTransport detached(&absent);
        msgs[0].data = &byte;
        CHECK(!detached.transfer(0x50, msgs, 1) && detached.getErr() == ESP_ERR_INVALID_STATE);
    }
    CHECK(TestI2C::devices == 0 && TestI2C::buses == 0);

    // USB function: descriptors, class driver, and control transfer stages.
    {
        I2CBus bus(port);
        I2CTinyUSB usb(&bus);
        uint8_t count = 0xFF;
        const tusb_control_request_t status = request(true, Bridge::CMD_GET_STATUS, 0, 0, 1);
        CHECK(usbd_app_driver_get_cb(&count) == nullptr && count == 0);
        CHECK(!tud_vendor_control_xfer_cb(0, CONTROL_STAGE_SETUP, &status));
        CHECK(!usb.isMounted());

        CHECK(usb.init() && usb.isInitialized() && TestUSB::installs == 1);
        I2CTinyUSB second(&bus);
        CHECK(!second.init() && second.getErr() == ESP_ERR_INVALID_STATE);

        const tinyusb_desc_config_t& desc = TestUSB::config.descriptor;
        CHECK(TestUSB::config.port == TINYUSB_PORT_FULL_SPEED_0 && !TestUSB::config.phy.skip_setup);
        CHECK(desc.device->bLength == 18 && desc.device->bDescriptorType == TUSB_DESC_DEVICE);
        CHECK(desc.device->idVendor == 0x303A && desc.device->idProduct == 0x4020);
        CHECK(desc.device->bMaxPacketSize0 == 64 && desc.device->bNumConfigurations == 1);
        CHECK(desc.device->bDeviceClass == 0 && desc.device->iSerialNumber == 3);
        const uint8_t* config = desc.full_speed_config;
        CHECK(config[0] == 9 && config[1] == TUSB_DESC_CONFIGURATION && config[2] == 18 && config[3] == 0);
        CHECK(config[4] == 1 && config[5] == 1 && config[7] == 0x80 && config[8] == 125);
        const auto* itf = reinterpret_cast<const tusb_desc_interface_t*>(config + 9);
        CHECK(itf->bLength == 9 && itf->bDescriptorType == TUSB_DESC_INTERFACE && itf->bInterfaceNumber == 0);
        CHECK(itf->bNumEndpoints == 0 && itf->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC);
        CHECK(desc.string_count == 4 && desc.string[0][0] == 0x09 && desc.string[0][1] == 0x04);
        CHECK(std::string(desc.string[1]) == "Engineering ILS Foundation");
        CHECK(std::string(desc.string[2]) == "OSR V3 i2c-tiny-usb bridge");
        CHECK(std::string(desc.string[3]) == "240AC4123456");

        // The class driver opens only the bridge interface, which TinyUSB needs for SET_CONFIGURATION.
        const usbd_class_driver_t* driver = usbd_app_driver_get_cb(&count);
        CHECK(driver != nullptr && count == 1);
        CHECK(driver->init && driver->deinit && driver->reset && driver->xfer_cb);
        CHECK(driver->open(0, itf, 9) == 9);
        CHECK(driver->open(0, itf, 8) == 0);
        tusb_desc_interface_t other = *itf;
        other.bNumEndpoints = 2;
        CHECK(driver->open(0, &other, 9) == 0);
        other = *itf;
        other.bInterfaceClass = 0x02;
        CHECK(driver->open(0, &other, 9) == 0);
        CHECK(!driver->control_xfer_cb(0, CONTROL_STAGE_SETUP, &status));

        // Probe of the Linux driver: SET_DELAY without data is acknowledged in the SETUP stage.
        TestUSB::statuses = 0;
        CHECK(control(request(false, Bridge::CMD_SET_DELAY, 10, 0, 0)));
        CHECK(TestUSB::statuses == 1);

        // IN requests send their data from the SETUP stage only.
        int xfers = TestUSB::xfers;
        CHECK(control(request(true, Bridge::CMD_GET_FUNC, 0, 0, 4)));
        CHECK(TestUSB::xfers == xfers + 1 && TestUSB::xfer_len == 4);
        const uint8_t* sent = static_cast<const uint8_t*>(TestUSB::xfer_buffer);
        CHECK(sent[0] == 0x09 && sent[1] == 0x00 && sent[2] == 0xFF && sent[3] == 0x0E);

        // i2cget through the USB function: the write data arrives in the DATA stage.
        const uint8_t pointer[] = {0x30};
        const uint8_t value[] = {0x30, 0x66};
        CHECK(control(request(false, Bridge::CMD_I2C_IO | BEGIN | END, 0, 0x50, 2), value));
        CHECK(Bus::shape == "S W3 P" && Bus::targets[0x50].regs[0x30] == 0x66);
        CHECK(control(request(false, Bridge::CMD_I2C_IO | BEGIN, 0, 0x50, 1), pointer));
        CHECK(control(request(true, Bridge::CMD_I2C_IO | END, Bridge::M_RD, 0x50, 1)));
        CHECK(Bus::shape == "S W2 S W1 N1 P" && TestUSB::xfer_len == 1);
        CHECK(static_cast<const uint8_t*>(TestUSB::xfer_buffer)[0] == 0x66);
        CHECK(control(status) && static_cast<const uint8_t*>(TestUSB::xfer_buffer)[0] == Bridge::STATUS_ADDRESS_ACK);

        // A zero-length read has no data stage.
        CHECK(control(request(true, Bridge::CMD_I2C_IO | BEGIN | END, Bridge::M_RD, 0x50, 0)));
        CHECK(TestUSB::xfer_len == 0 && Bus::shape == "S W1 N1 P");

        // Unsupported or oversized requests stall in the SETUP stage, before any data is accepted.
        xfers = TestUSB::xfers;
        CHECK(!control(request(false, Bridge::CMD_ECHO, 0, 0, 2), value));
        CHECK(!control(request(true, 9, 0, 0, 1)));
        CHECK(!control(request(false, Bridge::CMD_I2C_IO | BEGIN | END, 0, 0x50, Bridge::MAX_DATA + 1)));
        CHECK(!control(request(true, Bridge::CMD_I2C_IO | BEGIN | END, Bridge::M_RD, 0x50, Bridge::MAX_DATA + 1)));
        CHECK(TestUSB::xfers == xfers);

        TestUSB::mounted = true;
        CHECK(usb.isMounted());
        usb.deinit();
        CHECK(!usb.isInitialized() && !usb.isMounted() && TestUSB::installs == 0);
        CHECK(usbd_app_driver_get_cb(&count) == nullptr && count == 0);
        CHECK(!tud_vendor_control_xfer_cb(0, CONTROL_STAGE_SETUP, &status));

        // A failed installation leaves no active bridge, so another instance can take the port.
        TestUSB::install_result = ESP_ERR_NOT_FOUND;
        CHECK(!usb.init() && usb.getErr() == ESP_ERR_NOT_FOUND);
        CHECK(usbd_app_driver_get_cb(&count) == nullptr);
        TestUSB::install_result = ESP_OK;
        {
            I2CTinyUSBConfig custom;
            custom.vid = 0x1C40;
            custom.pid = 0x0534;
            custom.serial = "bench-1";
            I2CTinyUSB scoped(&bus, custom);
            CHECK(scoped.init() && TestUSB::installs == 1);
            CHECK(desc.device->idVendor == 0x1C40 && desc.device->idProduct == 0x0534);
            CHECK(std::string(desc.string[3]) == "bench-1");
        }
        CHECK(TestUSB::installs == 0);

        Protocols::I2CPort missing{};
        I2CBus absent(missing);
        I2CTinyUSB unpowered(&absent);
        CHECK(!unpowered.init() && unpowered.getErr() == ESP_ERR_INVALID_STATE && TestUSB::installs == 0);
    }
    CHECK(TestI2C::devices == 0 && TestI2C::buses == 0);

    std::cout << "i2c_tiny_usb tests passed\n";
    return 0;
}
