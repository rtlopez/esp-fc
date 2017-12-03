#ifndef _ESP32_ESC_DRIVER_H_
#define _ESP32_ESC_DRIVER_H_

#include "EscDriver.h"
#include "Arduino.h"

#if defined(ESP32)

class Esp32EscDriver
{
  public:
    Esp32EscDriver() {}

    int begin(EscProtocol protocol, bool async, int16_t rate) { return 1; }

    int attach(size_t channel, int pin, int pulse) { return 1; }

    int write(size_t channel, int pulse) { return 1; }

    void apply() {}
};

#endif

#endif
