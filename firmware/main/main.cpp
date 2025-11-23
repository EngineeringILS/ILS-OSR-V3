#include <stdio.h>
#include <common/drivers/FakeIMU.hpp>

extern "C" {
    void app_main(void);
};

void app_main(void) {
    int myint = 2;
    myint++;
}