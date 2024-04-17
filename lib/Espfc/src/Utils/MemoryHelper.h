#pragma once

#ifdef ESP32
#include <esp_attr.h>
#define IRAM_ATTR_ALT IRAM_ATTR
#else
#define IRAM_ATTR_ALT
#endif
