#pragma once

#include "Device/InputDevice.h"
#include <EspNowRcLink/Receiver.h>
#include <cstdint>
#include <cstddef>

namespace Espfc {

namespace Device {

class InputEspNow: public InputDevice
{
public:
  int begin(void);
  InputStatus update() override;
  uint16_t get(uint8_t i) const override;
  void get(uint16_t * data, size_t len) const override;
  size_t getChannelCount() const override;
  bool needAverage() const override;

private:
  static const size_t CHANNELS = EspNowRcLink::RC_CHANNEL_MAX + 1;
  uint16_t _channels[CHANNELS];
  EspNowRcLink::Receiver _rx;
};

}

}
