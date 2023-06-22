#ifndef _INPUT_DEVICE_INPUT_PPM_H_
#define _INPUT_DEVICE_INPUT_PPM_H_

#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include "InputDevice.h"

namespace Espfc {

enum PPMMode {
  PPM_MODE_NORMAL   = 0x01, // RISING edge
  PPM_MODE_INVERTED = 0x02  // FALLING edge
};

namespace Device {

class InputPPM: public InputDevice
{
  public:
    void begin(uint8_t pin, int mode = PPM_MODE_NORMAL);
    void handle() IRAM_ATTR;
    static void handle_isr() IRAM_ATTR;

    uint16_t get(uint8_t i) const override
    {
      return _channels[i];
    }

    void get(uint16_t * data, size_t len) const override
    {
      const uint16_t * src = const_cast<const uint16_t *>(_channels);
      while(len--)
      {
        *data++ = *src++;
      }
    }

    InputStatus update() override
    {
      if(_new_data)
      {
        _new_data = false;
        return INPUT_RECEIVED;
      }
      //if(fail()) return INPUT_FAILED;
      return INPUT_IDLE;
    }

    size_t getChannelCount() const override { return CHANNELS; }

    bool needAverage() const override { return true; }

    static const size_t CHANNELS = 16;
    static const uint32_t BROKEN_LINK_US = 100000UL; // 100ms

  private:
    volatile uint16_t _channels[CHANNELS];
    volatile uint32_t _last_tick;
    volatile uint8_t  _channel;
    volatile bool     _new_data;
    uint8_t _pin;
    static InputPPM * _instance;
};

}

}

#endif
