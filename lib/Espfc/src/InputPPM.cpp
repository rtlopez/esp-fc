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
    uint16_t default_tick = i / 2 == THROTTLE_CHANNEL ? TICK_MIN : TICK_CENTER;  // except throttle channel 3 (zero indexed)
    _channels[i] = i & 1 == 0 ? TICK_SPACE : default_tick; // init with default values
  }
  pinMode(_pin, INPUT);
  attachInterrupt(_pin, _ppm_input_handler_callback, CHANGE);
}

void InputPPM::handle()
{
  unsigned long now = micros();

  uint8_t old = _value;
  uint8_t v = digitalRead(_pin);

  if(v == old) return; // pin not changed, do nothing, sanity

  // TODO: detect broken link and set fail-safe
  // TODO: handle inversed PPM?

  unsigned long time = now - _start;
  //if(time < TICK_SPACE) return; // noise??

  _value = v;
  _start = now;
  _channels[_channel] = time;
  _channel++;

  // sync
  if(time > TICK_SYNC)
  {
    _channel = 0;
    _new_data = 1;
    return;
  }

  // rewind
  if(_channel >= CHANNELS * 2)
  {
    _new_data = 1;
    _channel = 0;
  }
}

InputPPM PPM;

static void _ppm_input_handler_callback()
{
  PPM.handle();
}

}
