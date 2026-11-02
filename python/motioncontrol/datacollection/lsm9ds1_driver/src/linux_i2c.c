#define _POSIX_C_SOURCE 200809L
#include "lsm9ds1_linux_i2c.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static void set_error(char *error, size_t size, const char *message) {
    if (error != NULL && size > 0u) snprintf(error, size, "%s", message);
}

int lsm9ds1_linux_discover(char *device_path, size_t path_size,
                           char *error, size_t error_size) {
    DIR *directory;
    struct dirent *entry;
    unsigned matches = 0u;
    char match[261] = {0};
    if (device_path == NULL || path_size == 0u) {
        set_error(error, error_size, "invalid discovery output buffer");
        return -1;
    }
    directory = opendir("/sys/class/i2c-adapter");
    if (directory == NULL) {
        set_error(error, error_size, "cannot open /sys/class/i2c-adapter");
        return -1;
    }
    while ((entry = readdir(directory)) != NULL) {
        char filename[256];
        char name[256];
        FILE *file;
        if (strncmp(entry->d_name, "i2c-", 4u) != 0) continue;
        if (snprintf(filename, sizeof(filename), "/sys/class/i2c-adapter/%s/name",
                     entry->d_name) >= (int)sizeof(filename)) continue;
        file = fopen(filename, "r");
        if (file == NULL) continue;
        if (fgets(name, sizeof(name), file) != NULL &&
            strstr(name, "i2c-tiny-usb") != NULL) {
            ++matches;
            snprintf(match, sizeof(match), "/dev/%s", entry->d_name);
        }
        fclose(file);
    }
    closedir(directory);
    if (matches == 0u) {
        set_error(error, error_size,
                  "no sysfs I2C adapter name contains 'i2c-tiny-usb'; use --bus /dev/i2c-N if appropriate");
        return -1;
    }
    if (matches != 1u) {
        set_error(error, error_size,
                  "multiple sysfs I2C adapter names contain 'i2c-tiny-usb'; select one with --bus /dev/i2c-N");
        return -1;
    }
    if (snprintf(device_path, path_size, "%s", match) >= (int)path_size) {
        set_error(error, error_size, "discovered I2C device path is too long");
        return -1;
    }
    return 0;
}

int lsm9ds1_linux_open(lsm9ds1_linux_i2c *bus, const char *device_path,
                       char *error, size_t error_size) {
    if (bus == NULL || device_path == NULL || strncmp(device_path, "/dev/i2c-", 9u) != 0) {
        set_error(error, error_size, "--bus must name a /dev/i2c-N device");
        return -1;
    }
    bus->fd = open(device_path, O_RDWR | O_CLOEXEC | O_NONBLOCK);
    if (bus->fd < 0) {
        char message[256];
        snprintf(message, sizeof(message), "cannot open %s: %s", device_path, strerror(errno));
        set_error(error, error_size, message);
        return -1;
    }
    return 0;
}

void lsm9ds1_linux_close(lsm9ds1_linux_i2c *bus) {
    if (bus != NULL && bus->fd >= 0) {
        close(bus->fd);
        bus->fd = -1;
    }
}

static int linux_read(void *user, uint8_t address, uint8_t reg,
                      uint8_t *data, size_t length) {
    lsm9ds1_linux_i2c *bus = user;
    uint8_t command = reg;
    struct i2c_msg messages[2];
    struct i2c_rdwr_ioctl_data transaction;
    if (bus == NULL || bus->fd < 0 || data == NULL || length == 0u || length > 65535u)
        return -1;
    messages[0].addr = address;
    messages[0].flags = 0;
    messages[0].len = 1u;
    messages[0].buf = &command;
    messages[1].addr = address;
    messages[1].flags = I2C_M_RD;
    messages[1].len = (uint16_t)length;
    messages[1].buf = data;
    transaction.msgs = messages;
    transaction.nmsgs = 2u;
    return ioctl(bus->fd, I2C_RDWR, &transaction) == 2 ? 0 : -1;
}

static int linux_write(void *user, uint8_t address, uint8_t reg,
                       const uint8_t *data, size_t length) {
    lsm9ds1_linux_i2c *bus = user;
    uint8_t bytes[257];
    struct i2c_msg message;
    struct i2c_rdwr_ioctl_data transaction;
    if (bus == NULL || bus->fd < 0 || data == NULL || length == 0u || length > 256u)
        return -1;
    bytes[0] = reg;
    memcpy(&bytes[1], data, length);
    message.addr = address;
    message.flags = 0;
    message.len = (uint16_t)(length + 1u);
    message.buf = bytes;
    transaction.msgs = &message;
    transaction.nmsgs = 1u;
    return ioctl(bus->fd, I2C_RDWR, &transaction) == 1 ? 0 : -1;
}

static int linux_clock(void *user, uint64_t *nanoseconds) {
    struct timespec now;
    (void)user;
    if (nanoseconds == NULL || clock_gettime(CLOCK_MONOTONIC, &now) != 0) return -1;
    *nanoseconds = (uint64_t)now.tv_sec * UINT64_C(1000000000) + (uint64_t)now.tv_nsec;
    return 0;
}

lsm9ds1_io lsm9ds1_linux_io(lsm9ds1_linux_i2c *bus) {
    lsm9ds1_io io = {bus, linux_read, linux_write, linux_clock};
    return io;
}
