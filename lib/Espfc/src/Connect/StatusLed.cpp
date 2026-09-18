#include "StatusLed.hpp"
#include "Hal/Gpio.hpp"
#include "Hal/Time.hpp"
#include "Target/Target.h"

namespace Espfc::Connect {

#ifdef ESPFC_LED_WS2812
static const Hal::RgbColor PIXEL_ON = {0x40, 0x40, 0x80};
static const Hal::RgbColor PIXEL_OFF = {0, 0, 0};
#endif

static int LED_OFF_PATTERN[] = {0};
static int LED_OK_PATTERN[] = {100, 900, 0};
static int LED_ERROR_PATTERN[] = {100, 100, 100, 100, 100, 1500, 0};
static int LED_ON_PATTERN[] = {100, 0};

StatusLed::StatusLed()
    : _pin(-1), _invert(0), _status(LED_OFF), _next(0), _state(false), _step(0), _pattern(LED_OFF_PATTERN)
{
}

void StatusLed::begin(int8_t pin, uint8_t type, uint8_t invert)
{
  if (pin == -1) return;

  _pin = pin;
  _type = type;
  _invert = invert;

#ifdef ESPFC_LED_WS2812
  if (_type == LED_STRIP) _rgb.begin(_pin);
  if (_type == LED_SIMPLE) Hal::Gpio::pinMode(_pin, Hal::Gpio::Output);
#else
  Hal::Gpio::pinMode(_pin, Hal::Gpio::Output);
#endif
  setStatus(LED_ON, true);
}

void StatusLed::setStatus(LedStatus newStatus, bool force)
{
  if (_pin == -1) return;
  if (!force && newStatus == _status) return;

  _status = newStatus;
  _state = false;
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
      _state = true;
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
  if (_pin == -1 || !_pattern) return;

  uint32_t now = millis();

  if (now < _next) return;

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
  const Hal::Gpio::PinStatus state = (val ^ _invert) ? Hal::Gpio::High : Hal::Gpio::Low;
#ifdef ESPFC_LED_WS2812
  if (_type == LED_STRIP) _rgb.write(val ? PIXEL_ON : PIXEL_OFF);
  if (_type == LED_SIMPLE) Hal::Gpio::digitalWrite(_pin, state);
#else
  Hal::Gpio::digitalWrite(_pin, state);
#endif
}

} // namespace Espfc::Connect
