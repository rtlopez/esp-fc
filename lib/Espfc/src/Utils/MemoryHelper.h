#pragma once

#ifdef ESP32
#include <esp_attr.h>
#define FAST_CODE_ATTR IRAM_ATTR
#else
#define FAST_CODE_ATTR
#endif
