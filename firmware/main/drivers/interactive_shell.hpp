#include <Max1704x_test.hpp>
#include <ina3221_test.hpp>
#include <SerialIO.hpp>
#include <common/drivers/Application.hpp>
#include <string>

// namespace Lunabotics {
// namespace ESP32 {
// namespace Drivers {

namespace Lunabotics {
namespace ESP32 {
namespace Tools {


using namespace Applications;

class MainTerminal : public Application {
public:
    /**
     * @brief Constructor for the MainTerminal Application.
     * 
     * @param Terminal The SerialIO Terminal, it is expected to be reserved to 128 characters for this application.
     */
    explicit MainTerminal(SerialIO &Terminal) : Application(HostController::ESP32), Terminal_(Terminal) {
        Message_.reserve(128);
    }

    /**
     * @brief Main Terminal Loop for the MainTerminal Application.
     */
    void run() override;

    // Copy constructor is indeed missing, copying an entire application is a philosophically impossible idea.
    /**
     * @brief Destructor for the MainTerminal Application. Gracefully returns the borrowed Terminal instance.
     */
    ~MainTerminal() {}

private:
    SerialIO& Terminal_;
    std::string Message_;

};

} // Tools
} // ESP32
} // Lunabotics