#include <stdio.h>

extern "C" {
    void app_main(void);
};

void app_main(void) {
    int myint = 2;
    myint++;
}