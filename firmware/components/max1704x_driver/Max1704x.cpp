#include "Max1704x.hpp"

namespace Lunabotics {
namespace Common {
namespace Sensors{

// bool Max1704x::init() {
//     esp_err_t initStatus = max1704x_init_desc(
//         // device descriptor,
//         // i2c port number,
//         // sda GPIO
//         // cl GPIO
//     );

//     if (initStatus != ESP_OK) {
//         // TODO: Need to qualify the error to properly determine what kind of error and respective state change.
//         // Read ESP32 Docs for ESP_ERR_T status codes.
//         _State = SensorState::ERROR;
//         return false;
//     } else {
//         _State = SensorState::CONNECTED;
//     }
// }


} // namespace Sensors
} // namespace Common
} // namespace Lunabotics