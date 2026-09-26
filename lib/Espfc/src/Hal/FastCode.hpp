#pragma once

#if defined(ESP32)
#include <esp_attr.h>
#define FAST_CODE_ATTR IRAM_ATTR
#define ISR_CODE_ATTR IRAM_ATTR
#elif defined(ARCH_RP2040)
#include <pico/platform/sections.h>
#define FAST_CODE_ATTR __not_in_flash("fast_code")
#define ISR_CODE_ATTR __not_in_flash("isr_code")
#elif defined(ESP8266)
#include <c_types.h>
#define FAST_CODE_ATTR
#define ISR_CODE_ATTR IRAM_ATTR
#else
#define FAST_CODE_ATTR
#define ISR_CODE_ATTR
#endif
