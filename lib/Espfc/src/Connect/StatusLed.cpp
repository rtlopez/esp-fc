#include "StatusLed.hpp"
#include <Arduino.h>

namespace Espfc::Connect
{

static int LED_OFF_PATTERN[] = {0};
static int LED_OK_PATTERN[] = {100, 900, 0};
static int LED_ERROR_PATTERN[] = {100, 100, 100, 100, 100, 1500, 0};
static int LED_ON_PATTERN[] = {100, 0};

StatusLed::StatusLed() : _pin(-1), _invert(0), _status(LED_OFF), _next(0), _state(LOW), _step(0), _pattern(LED_OFF_PATTERN) {}

void StatusLed::begin(int8_t pin, uint8_t invert)
{
  if(pin == -1) return;
  _pin = pin;
  _invert = invert;
  pinMode(_pin, OUTPUT);
  setStatus(LED_ON, true);
}

void StatusLed::setStatus(LedStatus newStatus, bool force)
{
  if(_pin == -1) return;
  if(!force && newStatus == _status) return;

  _status = newStatus;
  _state = LOW;
  _step = 0;
  _next = millis();

  switch (_status)
  {
    case LED_OK:
      _pattern = LED_OK_PATTERN;
      break;
    case LED_ERROR:
      _pattern = LED_ERROR_PATTERN;
      break;
    case LED_ON:
      _pattern = LED_ON_PATTERN;
      _state = HIGH;
      break;
    case LED_OFF:
    default:
      _pattern = LED_OFF_PATTERN;
      break;
  }
  _write(_state);
}

void StatusLed::update()
{
  if(_pin == -1 || !_pattern) return;
  
  uint32_t now = millis();
  
  if(now < _next) return;

  if (!_pattern[_step])
  {
    _step = 0;
    _next = now + 20;
    return;
  }

  _state = !(_step & 1);
  _write(_state);

  _next = now + _pattern[_step];
  _step++;
}

void StatusLed::_write(uint8_t val)
{
  digitalWrite(_pin, val ^ _invert);
}

}
