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
#elif defined(ARCH_NRF52840)
  #include "TargetNRF52840.h"
#elif defined(UNIT_TEST)
  #include "TargetUnitTest.h"
#else
  #error "Unsupported platform!"
#endif

#include "Queue.h"
#include "Utils/MemoryHelper.h"

#if defined(ESPFC_I2C_0)
  #if defined(ESPFC_I2C_0_SOFT)
    #include "EspWire.h"
    #define WireClass EspTwoWire
    #define WireInstance EspWire
  #else
    #include <Wire.h>
    #define WireClass TwoWire
    #define WireInstance Wire
  #endif
  #if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_TWOWIRE)
    WireClass WireInstance;
  #endif
#endif

#if defined(ESPFC_SPI_0)
  #include <SPI.h>
  #if !defined(ESPFC_SPI_0_DEV)
    #define ESPFC_SPI_0_DEV SPI
  #endif

  #if !defined(ESPFC_SPI_0_DEV_T)
    #define ESPFC_SPI_0_DEV_T SPIClass
  #endif
  #if defined(NO_GLOBAL_INSTANCES) || defined(NO_GLOBAL_SPI)
    #if !defined(ARCH_RP2040)
      ESPFC_SPI_0_DEV_T ESPFC_SPI_0_DEV;
    #endif
  #endif
#endif
