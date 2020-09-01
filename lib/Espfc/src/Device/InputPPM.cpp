#include "Device/InputPPM.h"
#include <Arduino.h>

namespace Espfc {

namespace Device {

InputPPM * InputPPM::_instance = NULL;

void InputPPM::begin(uint8_t pin, int mode)
{
  if(_instance && _pin != -1)
  {
    detachInterrupt(_pin);
    _instance = NULL;
  }
  if(pin != -1)
  {
    _instance = this;
    _pin = pin;
    _channel = 0;
    _last_tick = micros();
    for(size_t i = 0; i < CHANNELS; i++)
    {
      _channels[i] = (i == 0 || i == 1 || i == 3) ? 1500 : 1000; // ail, elev, rud
    }
    pinMode(_pin, INPUT);
    attachInterrupt(_pin, InputPPM::handle_isr, mode);
  }
}

void InputPPM::handle()
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

void InputPPM::handle_isr()
{
  if(_instance) _instance->handle();
}

}

}
