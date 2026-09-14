#pragma once

#if defined(ESP32C3)
#include "TargetESP32c3.h"
#elif defined(ESP32S2)
#include "TargetESP32s2.h"
#elif defined(ESP32S3)
#include "TargetESP32s3.h"
#elif defined(ESP32)
#include "TargetESP32.h"
#elif defined(ESP8266)
#include "TargetESP8266.h"
#elif defined(ARCH_RP2040)
#include "TargetRP2040.h"
#elif defined(UNIT_TEST)
#include "TargetUnitTest.h"
#else
#error "Unsupported platform!"
#endif

#include "Queue.h"
#include "Utils/MemoryHelper.h"
