#include "Device/InputPPM.h"
#include <Arduino.h>
#include "Utils/MemoryHelper.h"

namespace Espfc {

namespace Device {

void InputPPM::begin(uint8_t pin, int mode)
{
  if(_pin != -1)
  {
    detachInterrupt(_pin);
    _pin = -1;
  }
  if(pin != -1)
  {
    _pin = pin;
    _channel = 0;
    _last_tick = micros();
    for(size_t i = 0; i < CHANNELS; i++)
    {
      _channels[i] = (i == 2) ? 1000 : 1500; // throttle
    }
    pinMode(_pin, INPUT);
#if defined(UNIT_TEST)
    // no mock available
#elif defined(ARCH_RP2040)
    attachInterruptParam(_pin, InputPPM::handle_isr, (PinStatus)mode, this);
#else
    attachInterruptArg(_pin, InputPPM::handle_isr, this, mode);
#endif
  }
}

InputStatus FAST_CODE_ATTR InputPPM::update()
{
  if(_new_data)
  {
    _new_data = false;
    return INPUT_RECEIVED;
  }
  return INPUT_IDLE;
}

uint16_t FAST_CODE_ATTR InputPPM::get(uint8_t i) const
{
  return _channels[i];
}

void FAST_CODE_ATTR InputPPM::get(uint16_t * data, size_t len) const
{
  const uint16_t * src = const_cast<const uint16_t *>(_channels);
  while(len--)
  {
    *data++ = *src++;
  }
}

size_t InputPPM::getChannelCount() const { return CHANNELS; }

bool InputPPM::needAverage() const { return true; }

void IRAM_ATTR InputPPM::handle()
{
  uint32_t now = micros();
  uint32_t width = now - _last_tick;

  _last_tick = now;

  if(width > 3000) // sync
  {
    _channel = 0;
    return;
  }

  if(_channel < CHANNELS) // ignore exceding channels
  {
    _channels[_channel] = width;
  }
  if(_channel == 3)
  {
    _new_data = true; // increase responsivnes for sticks channels
  }
  _channel++;
}

void IRAM_ATTR InputPPM::handle_isr(void* args)
{
  if(args) reinterpret_cast<InputPPM*>(args)->handle();
}

}

}
