#ifndef _ESPFC_DEVICE_INPUT_DEVICE_H_
#define _ESPFC_DEVICE_INPUT_DEVICE_H_

#include <cstddef>
#include <cstdint>

namespace Espfc {

enum InputStatus {
  INPUT_IDLE,
  INPUT_RECEIVED,
  INPUT_LOST,
  INPUT_FAILSAFE
};

namespace Device {

class InputDevice
{
  public:
    virtual InputStatus update() = 0;
    virtual uint16_t get(uint8_t channel) const = 0;
    virtual size_t getChannelCount() const = 0;
    virtual bool needAverage() const = 0;
};

}

}

#endif
