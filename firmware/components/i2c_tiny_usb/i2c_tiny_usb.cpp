#include "i2c_tiny_usb.hpp"
#include <cstdio>
#include <cstring>
#include "esp_log.h"
#include "esp_mac.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "device/usbd_pvt.h"

namespace Lunabotics {
namespace ESP32 {
namespace Drivers {

static const char* TAG = "i2c_tiny_usb";

namespace {

constexpr uint8_t ITF_NUM_BRIDGE = 0;
constexpr uint16_t CONFIG_TOTAL_LEN = TUD_CONFIG_DESC_LEN + sizeof(tusb_desc_interface_t);

// Without endpoints, the same configuration serves every speed.
const uint8_t configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attributes, power in mA.
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CONFIG_TOTAL_LEN, 0, 250),
    // Vendor specific interface without endpoints.
    sizeof(tusb_desc_interface_t), TUSB_DESC_INTERFACE, ITF_NUM_BRIDGE, 0, 0,
    TUSB_CLASS_VENDOR_SPECIFIC, 0, 0, 0,
};

// The descriptors must stay valid while the USB stack runs, and only one bridge can own the USB port.
tusb_desc_device_t device_descriptor;
const char language[] = {0x09, 0x04}; // English (United States)
char manufacturer[32];
char product[32];
char serial[32];
const char* strings[] = {language, manufacturer, product, serial};

usbd_class_driver_t class_driver;

}

/**
 * @brief TinyUSB callbacks of the active I2CTinyUSB instance.
 */
struct I2CTinyUSBCallbacks {
    static I2CTinyUSB* active;

    // Adds the class driver to the TinyUSB built-in drivers while a bridge is active.
    static const usbd_class_driver_t* drivers(uint8_t* driver_count) {
        *driver_count = (active != nullptr) ? 1 : 0;
        return (active != nullptr) ? &class_driver : nullptr;
    }

    static void init() {}

    static bool deinit() { return true; }

    static void reset(uint8_t rhport) { (void)rhport; }

    // Claims the vendor interface. TinyUSB fails SET_CONFIGURATION for interfaces no driver opens.
    static uint16_t open(uint8_t rhport, const tusb_desc_interface_t* itf, uint16_t max_len) {
        (void)rhport;
        if (max_len < sizeof(tusb_desc_interface_t) || itf->bInterfaceNumber != ITF_NUM_BRIDGE ||
            itf->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC || itf->bNumEndpoints != 0) {
            return 0;
        }
        return sizeof(tusb_desc_interface_t);
    }

    // Standard and class requests to the interface. TinyUSB answers GET/SET_INTERFACE itself.
    static bool control(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
        (void)rhport; (void)stage; (void)request;
        return false;
    }

    static bool xfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
        (void)rhport; (void)ep_addr; (void)result; (void)xferred_bytes;
        return false;
    }

    // TinyUSB passes every vendor request here, whatever its recipient. CMD_I2C_IO carries the
    // target address in wIndex, so it rarely matches the interface number.
    static bool vendorRequest(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
        I2CTinyUSB* self = active;
        if (self == nullptr) {
            return false;
        }

        const I2CTinyUSBBridge::Setup setup = {
            request->bRequest, request->wValue, request->wIndex, request->wLength
        };

        if (request->bmRequestType_bit.direction == TUSB_DIR_IN) {
            // The response is prepared in the SETUP stage. Later stages need no action.
            if (stage != CONTROL_STAGE_SETUP) {
                return true;
            }
            size_t len = 0;
            if (!self->bridge_.controlToHost(setup, self->in_, sizeof(self->in_), len)) {
                return false;
            }
            return tud_control_xfer(rhport, request, self->in_, static_cast<uint16_t>(len));
        }

        switch (stage) {
            case CONTROL_STAGE_SETUP:
                if (!I2CTinyUSBBridge::acceptsToDevice(setup) || request->wLength > sizeof(self->out_)) {
                    return false;
                }
                if (request->wLength == 0) {
                    return self->bridge_.controlToDevice(setup, nullptr) && tud_control_status(rhport, request);
                }
                // A short data stage must not leave bytes of an earlier request behind.
                std::memset(self->out_, 0, request->wLength);
                return tud_control_xfer(rhport, request, self->out_, request->wLength);
            case CONTROL_STAGE_DATA:
                return self->bridge_.controlToDevice(setup, self->out_);
            default:
                return true;
        }
    }
};

I2CTinyUSB* I2CTinyUSBCallbacks::active = nullptr;

I2CTinyUSB::I2CTinyUSB(I2CBus* bus, const I2CTinyUSBConfig& config) :
    bus_(bus),
    config_(config),
    transport_(bus, config.max_frequency_hz, config.timeout_ms),
    bridge_(transport_)
    {}

I2CTinyUSB::~I2CTinyUSB() {
    deinit();
}

bool I2CTinyUSB::init() {
    if (initialized_) {
        return true;
    }
    if (bus_ == nullptr || !bus_->isInitialized()) {
        err_ = ESP_ERR_INVALID_STATE;
        ESP_LOGE(TAG, "I2C bus is not initialized");
        return false;
    }
    if (I2CTinyUSBCallbacks::active != nullptr) {
        // The USB-OTG controller hosts one device.
        err_ = ESP_ERR_INVALID_STATE;
        return false;
    }

    device_descriptor = {};
    device_descriptor.bLength = sizeof(tusb_desc_device_t);
    device_descriptor.bDescriptorType = TUSB_DESC_DEVICE;
    device_descriptor.bcdUSB = 0x0200;
    // Class codes come from the interface descriptor.
    device_descriptor.bDeviceClass = 0;
    device_descriptor.bDeviceSubClass = 0;
    device_descriptor.bDeviceProtocol = 0;
    device_descriptor.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE;
    device_descriptor.idVendor = config_.vid;
    device_descriptor.idProduct = config_.pid;
    device_descriptor.bcdDevice = config_.bcd_device;
    device_descriptor.iManufacturer = 1;
    device_descriptor.iProduct = 2;
    device_descriptor.iSerialNumber = 3;
    device_descriptor.bNumConfigurations = 1;

    snprintf(manufacturer, sizeof(manufacturer), "%s", config_.manufacturer ? config_.manufacturer : "");
    snprintf(product, sizeof(product), "%s", config_.product ? config_.product : "");
    if (config_.serial != nullptr) {
        snprintf(serial, sizeof(serial), "%s", config_.serial);
    } else {
        uint8_t mac[6] = {};
        esp_read_mac(mac, ESP_MAC_BASE);
        snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    class_driver = {};
    class_driver.name = "I2C-TINY-USB";
    class_driver.init = I2CTinyUSBCallbacks::init;
    class_driver.deinit = I2CTinyUSBCallbacks::deinit;
    class_driver.reset = I2CTinyUSBCallbacks::reset;
    class_driver.open = I2CTinyUSBCallbacks::open;
    class_driver.control_xfer_cb = I2CTinyUSBCallbacks::control;
    class_driver.xfer_cb = I2CTinyUSBCallbacks::xfer;

    tinyusb_config_t usb_config = {};
    usb_config.port = TINYUSB_PORT_FULL_SPEED_0;
    usb_config.phy.skip_setup = false;
    usb_config.phy.self_powered = false;
    usb_config.phy.vbus_monitor_io = -1;
    usb_config.task.size = TINYUSB_DEFAULT_TASK_SIZE;
    usb_config.task.priority = TINYUSB_DEFAULT_TASK_PRIO;
    usb_config.task.xCoreID = TINYUSB_DEFAULT_TASK_AFFINITY;
    usb_config.descriptor.device = &device_descriptor;
    usb_config.descriptor.string = strings;
    usb_config.descriptor.string_count = sizeof(strings) / sizeof(strings[0]);
    usb_config.descriptor.full_speed_config = configuration_descriptor;
    usb_config.descriptor.high_speed_config = configuration_descriptor;

    // Set before the stack starts, which queries the class driver during initialization.
    I2CTinyUSBCallbacks::active = this;
    err_ = tinyusb_driver_install(&usb_config);
    if (err_ != ESP_OK) {
        I2CTinyUSBCallbacks::active = nullptr;
        ESP_LOGE(TAG, "TinyUSB installation failed: %s", esp_err_to_name(err_));
        return false;
    }

    initialized_ = true;
    ESP_LOGI(TAG, "i2c-tiny-usb bridge %04x:%04x on I2C port %u", config_.vid, config_.pid,
             static_cast<unsigned>(bus_->getI2CPort().i2c_port));
    return true;
}

void I2CTinyUSB::deinit() {
    if (!initialized_) {
        return;
    }
    err_ = tinyusb_driver_uninstall();
    // Requests that still arrive are stalled once no instance is active.
    I2CTinyUSBCallbacks::active = nullptr;
    initialized_ = false;
}

bool I2CTinyUSB::isMounted() const {
    return initialized_ && tud_mounted();
}

}
}
}

extern "C" {

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request) {
    return Lunabotics::ESP32::Drivers::I2CTinyUSBCallbacks::vendorRequest(rhport, stage, request);
}

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) {
    return Lunabotics::ESP32::Drivers::I2CTinyUSBCallbacks::drivers(driver_count);
}

}
