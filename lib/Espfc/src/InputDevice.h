#ifndef _ESPFC_INPUT_DEVICE_H_
#define _ESPFC_INPUT_DEVICE_H_

#include <Arduino.h>

namespace Espfc {

enum InputStatus {
  INPUT_IDLE,
  INPUT_RECEIVED,
  INPUT_FAILED
};

class InputDevice
{
  public:
    virtual InputStatus update() = 0;
    virtual uint16_t get(uint8_t channel) const = 0;
};

}

#endif // _ESPFC_INPUT_DEVICE_H_
