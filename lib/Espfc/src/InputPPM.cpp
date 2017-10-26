#include "InputPPM.h"

namespace Espfc {

static void _ppm_input_handler_isr() ICACHE_RAM_ATTR;

void InputPPM::begin(uint8_t pin, int mode)
{
  _pin = pin;
  _channel = 0;
  _last_tick = micros();
  for(size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i] = (i == 0 || i == 1 || i == 3) ? 1500 : 1000; // ail, elev, rud
  }
  pinMode(_pin, INPUT);
  attachInterrupt(_pin, _ppm_input_handler_isr, mode);
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
  _channel++;
  if(_channel == 4) _new_data = true; // increase responsivnes for sticks channels
}

InputPPM PPM;

static void _ppm_input_handler_isr()
{
  PPM.handle();
}

}
