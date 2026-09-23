#pragma once
#include <cstdint>
namespace TestClock { inline int64_t now = 4294967296LL; }
inline int64_t esp_timer_get_time() { return TestClock::now += 10000; }
