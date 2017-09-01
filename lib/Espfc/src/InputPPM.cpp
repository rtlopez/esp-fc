#include "InputPPM.h"

namespace Espfc {

static void _ppm_input_handler_callback() ICACHE_RAM_ATTR;

void InputPPM::begin(uint8_t pin)
{
  _pin = pin;
  _channel = 0;
  _start = micros();
  for(size_t i = 0; i < CHANNELS * 2; i++)
  {
    _channels[i] = TICK_SPACE;
    //uint16_t default_tick = i / 2 == THROTTLE_CHANNEL ? TICK_MIN : TICK_CENTER;  // except throttle channel 3 (zero indexed)
    //_channels[i] = i & 1 == 0 ? TICK_SPACE : default_tick; // init with default values
  }
  pinMode(_pin, INPUT);
  attachInterrupt(_pin, _ppm_input_handler_callback, CHANGE);
}

void InputPPM::handle()
{
  static unsigned long lastWidth = 0;

  unsigned long now = micros();
  unsigned long width = now - _start;

  _start = now;

  if(_channel < CHANNELS * 2) // ignore exceding pulses
  {
    _channels[_channel] = width;
  }

  if(lastWidth > 0 && lastWidth < TICK_SYNC) // FrSky X4RSB fix
  {
    _channel++; // ignore first pulse after sync
  }

  lastWidth = width;

  // sync
  if(width > TICK_SYNC)
  {
    _channel = 0;
    _new_data = 1;
    return;
  }
}

InputPPM PPM;

static void _ppm_input_handler_callback()
{
  PPM.handle();
}

}
