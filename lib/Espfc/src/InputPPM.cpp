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
  _channels[2] = 1000; // usually throttle channel
  pinMode(_pin, INPUT);
  attachInterrupt(_pin, _ppm_input_handler_callback, mode);
}

void InputPPM::handle()
{
  unsigned long now = micros();
  unsigned long width = now - _start;

  _start = now;

  if(width > 3000) // sync
  {
    _channel = 0;
    _new_data = 1;
    return;
  }

  if(_channel < CHANNELS) // ignore exceding channels
  {
    _channels[_channel] = width;
  }
  _channel++;
}

InputPPM PPM;

static void _ppm_input_handler_callback()
{
  PPM.handle();
}

}
