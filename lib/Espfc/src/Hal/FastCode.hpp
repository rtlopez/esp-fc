#pragma once

#if defined(ESP32)
#include <esp_attr.h>
#define FAST_CODE_ATTR IRAM_ATTR
#define ISR_CODE_ATTR IRAM_ATTR
#elif defined(ESP8266)
#include <c_types.h>
#define FAST_CODE_ATTR
#define ISR_CODE_ATTR IRAM_ATTR
#else
#define FAST_CODE_ATTR
#define ISR_CODE_ATTR
#endif
