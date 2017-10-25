#include "InputPPM.h"

namespace Espfc {

static void _ppm_input_handler_callback() ICACHE_RAM_ATTR;

void InputPPM::begin(uint8_t pin, int mode)
{
  _pin = pin;
  _channel = 0;
  _start = micros();
  for(size_t i = 0; i < CHANNELS; i++)
  {
    _channels[i] = 1500;
  }
  _channels[2] = 1000; // usually throttle channel (3)
  pinMode(_pin, INPUT);
  attachInterrupt(_pin, _ppm_input_handler_callback, mode);
}

void InputPPM::handle()
{
  uint32_t now = micros();
  uint32_t width = now - _start;

  _start = now;

  if(width > 3000) // sync
  {
    _channel = 0;
    //_new_data = true;
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

static void _ppm_input_handler_callback()
{
  PPM.handle();
}

}
