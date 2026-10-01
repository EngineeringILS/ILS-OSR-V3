#pragma once

#include "driver/i2c_master.h"
#include <cstddef>
#include <cstdint>

// Host test substitute: the TinyUSB and esp_tinyusb API used by the i2c-tiny-usb bridge.
#define CFG_TUD_ENDPOINT0_SIZE 64
#define TUD_CONFIG_DESC_LEN 9
#define TUD_CONFIG_DESCRIPTOR(config_num, _itfcount, _stridx, _total_len, _attribute, _power_ma) \
    9, TUSB_DESC_CONFIGURATION, (_total_len) & 0xFF, ((_total_len) >> 8) & 0xFF, \
    _itfcount, config_num, _stridx, 0x80 | (_attribute), (_power_ma) / 2

enum { TUSB_DESC_DEVICE = 1, TUSB_DESC_CONFIGURATION = 2, TUSB_DESC_STRING = 3, TUSB_DESC_INTERFACE = 4 };
enum { TUSB_CLASS_VENDOR_SPECIFIC = 0xFF };
enum { TUSB_DIR_OUT = 0, TUSB_DIR_IN = 1 };
enum { CONTROL_STAGE_IDLE, CONTROL_STAGE_SETUP, CONTROL_STAGE_DATA, CONTROL_STAGE_ACK };
enum xfer_result_t { XFER_RESULT_SUCCESS };

struct tusb_control_request_t {
    struct { uint8_t recipient : 5; uint8_t type : 2; uint8_t direction : 1; } bmRequestType_bit;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

struct __attribute__((packed)) tusb_desc_device_t {
    uint8_t bLength, bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0;
    uint16_t idVendor, idProduct, bcdDevice;
    uint8_t iManufacturer, iProduct, iSerialNumber, bNumConfigurations;
};

struct __attribute__((packed)) tusb_desc_interface_t {
    uint8_t bLength, bDescriptorType, bInterfaceNumber, bAlternateSetting, bNumEndpoints;
    uint8_t bInterfaceClass, bInterfaceSubClass, bInterfaceProtocol, iInterface;
};

struct tusb_desc_device_qualifier_t;

struct usbd_class_driver_t {
    const char* name;
    void (*init)(void);
    bool (*deinit)(void);
    void (*reset)(uint8_t rhport);
    uint16_t (*open)(uint8_t rhport, const tusb_desc_interface_t* desc_intf, uint16_t max_len);
    bool (*control_xfer_cb)(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request);
    bool (*xfer_cb)(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
    bool (*xfer_isr)(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
    void (*sof)(uint8_t rhport, uint32_t frame_count);
};

enum tinyusb_port_t { TINYUSB_PORT_FULL_SPEED_0 = 0 };
struct tinyusb_phy_config_t { bool skip_setup; bool self_powered; int vbus_monitor_io; };
struct tinyusb_task_config_t { size_t size; uint8_t priority; int xCoreID; };
struct tinyusb_desc_config_t {
    const tusb_desc_device_t* device;
    const tusb_desc_device_qualifier_t* qualifier;
    const char** string;
    int string_count;
    const uint8_t* full_speed_config;
    const uint8_t* high_speed_config;
};
struct tinyusb_event_t;
using tinyusb_event_cb_t = void (*)(tinyusb_event_t* event, void* arg);
struct tinyusb_config_t {
    tinyusb_port_t port;
    tinyusb_phy_config_t phy;
    tinyusb_task_config_t task;
    tinyusb_desc_config_t descriptor;
    bool pm_lock_enable;
    tinyusb_event_cb_t event_cb;
    void* event_arg;
};

namespace TestUSB {
inline tinyusb_config_t config{};
inline esp_err_t install_result = ESP_OK;
inline int installs = 0;
inline bool mounted = false;
inline void* xfer_buffer = nullptr;
inline uint16_t xfer_len = 0;
inline int xfers = 0;
inline int statuses = 0;
}

inline esp_err_t tinyusb_driver_install(const tinyusb_config_t* config) {
    if (TestUSB::install_result != ESP_OK) return TestUSB::install_result;
    TestUSB::config = *config;
    ++TestUSB::installs;
    return ESP_OK;
}
inline esp_err_t tinyusb_driver_uninstall() {
    --TestUSB::installs;
    return ESP_OK;
}
inline bool tud_mounted() { return TestUSB::mounted; }
// Records the data stage; the test plays the host side of OUT data through xfer_buffer.
inline bool tud_control_xfer(uint8_t, const tusb_control_request_t*, void* buffer, uint16_t len) {
    TestUSB::xfer_buffer = buffer;
    TestUSB::xfer_len = len;
    ++TestUSB::xfers;
    return true;
}
inline bool tud_control_status(uint8_t, const tusb_control_request_t*) {
    ++TestUSB::statuses;
    return true;
}

extern "C" bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request);
extern "C" const usbd_class_driver_t* usbd_app_driver_get_cb(uint8_t* driver_count);
