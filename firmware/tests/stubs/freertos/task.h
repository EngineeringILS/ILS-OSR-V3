#pragma once
#include <functional>
#define pdMS_TO_TICKS(ms) (ms)
namespace TestTask { inline std::function<void(unsigned)> on_delay; }
inline void vTaskDelay(unsigned ticks) { if (TestTask::on_delay) TestTask::on_delay(ticks); }
