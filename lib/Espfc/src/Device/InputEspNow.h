#pragma once

#include "Device/InputDevice.h"
#include <EspNowRcLink/Receiver.h>

namespace Espfc {

namespace Device {

class InputEspNow: public InputDevice
{
  public:
    int begin(void)
    {
      for(size_t i = 0; i < CHANNELS; i++)
      {
        _channels[i] = i == 2 ? 1000 : 1500;
      }
      return _rx.begin();
    }

    InputStatus update() override
    {
      _rx.update();
      if(_rx.available())
      {
        for(size_t i = 0; i < CHANNELS; i++)
        {
          _channels[i] = _rx.getChannel(i);
        }
        return INPUT_RECEIVED;
      }
      return INPUT_IDLE;
    }

    uint16_t get(uint8_t i) const override
    {
      return _channels[i];
    }

    void get(uint16_t * data, size_t len) const override
    {
      const uint16_t * src = _channels;
      while(len--)
      {
        *data++ = *src++;
      }
    }

    size_t getChannelCount() const override { return CHANNELS; }

    bool needAverage() const override { return false; }

  private:

  static const size_t CHANNELS = EspNowRcLink::RC_CHANNEL_MAX + 1;
  uint16_t _channels[CHANNELS];
  EspNowRcLink::Receiver _rx;
};

}

}
