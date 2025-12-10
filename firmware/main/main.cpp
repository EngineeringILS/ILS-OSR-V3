#include <stdio.h>
#include <common/drivers/FakeIMU.hpp>
#include <drivers/SerialIO/SerialIO.hpp>


extern "C" {
    void app_main(void);
};

void app_main(void) {
    // Instantiate a new Terminal:
    SerialIO Terminal;
    Terminal.init();

    // Insantiate a looping variable to control the test loop:
    bool inputTest = true;
    // Instantiate a string to hold the input and output for the Terminal:
    std::string ioMsg;
    Terminal.serial_out("[TEST START] \n");
    // Run a simple loop to test functionality:
    while (inputTest) {
        ioMsg = Terminal.serial_in("Test I/O (Enter any message, 'q' to quit): ");
        Terminal.serial_out(ioMsg + "\n");
        if (ioMsg == "q") {
            Terminal.serial_out("[TEST END] \n");
            break;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    Terminal.deinit();
}